//! The `eframe::App`: a tiny screen state machine -- [`Screen::NewBoard`]
//! (a params form) and [`Screen::Editor`] (the created board, now with
//! manual part placement: pick a built-in footprint template, click to
//! place it, drag an already-placed one around -- every placement/move
//! hard-gated by [`BoardDoc::check_placement`] so a part can never
//! actually end up off-board or overlapping another one, "correct-by-
//! construction" rather than "flag it after the fact". Interactive
//! routing is the next step, see the development log's "Teil 29"
//! entry for the full MVP order).

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::mpsc;

use alladin_core::{Item, ItemId, JlcpcbDfm, LayerId, NetClass, NetId, Node, PadShape};
use alladin_geom::{Aabb, Point, Polygon, Unit, MM};
use alladin_render::{Camera, LayerToggles};
use alladin_router::route_single_net;
use eframe::egui::{self, Color32, Stroke};
use tokio::sync::oneshot;

use crate::background::{BackgroundJob, JobPoll};
use crate::board_doc::{
    BoardDoc, CopperWeight, FootprintId, LayerCount, NewBoardParams, RouteError, SilkDotId, SilkTextId, ZoneId, ZoneRecord,
    DEFAULT_SILK_DOT_DIAMETER, DEFAULT_SILK_TEXT_HEIGHT, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL, SILK_DOT_DIAMETER_STEPS_MM,
    SILK_TEXT_HEIGHT_STEPS_MM,
};
use crate::footprint::{self, world_items, FootprintTemplate, PadShapeKind};
use crate::parts_db::PartsDb;
use crate::ratsnest;
use crate::routing::{RoutingDrag, TraceDrag};
use crate::zone_fill;

enum Screen {
    NewBoard(NewBoardParams),
    Editor(EditorState),
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tool {
    Select,
    Place(usize),
    /// Direct pin-to-net assignment -- the "netlist" this MVP slice has,
    /// see the development log's "Teil 29" entry: no schematic, click
    /// one pin then another to join them onto the same net.
    Connect,
    /// Freehand via placement (stitching vias) -- click a pin to pick
    /// (or switch to) which net to stitch (see [`EditorState::via_net`]),
    /// then every click that doesn't land on a pin drops a via there on
    /// the currently picked net, until the tool changes or Escape is
    /// pressed. Deliberately not tied to an in-progress [`RoutingDrag`]
    /// -- see `crate::routing::RoutingDrag::drop_via_and_switch_layer`
    /// for the mid-route via/layer-switch case instead.
    PlaceVia,
    /// Interactive trace dragging between two same-net pins (see
    /// `crate::routing`) -- click one pin, move the mouse to see a live
    /// walkaround preview, click a same-net pin to commit.
    Route,
    /// Freehand copper zone/pour outline drawing -- each click adds a
    /// polygon vertex (see [`EditorState::zone_points`]); clicking back
    /// on the first point (or pressing Enter) closes it and runs
    /// `crate::zone_fill::fill_zone` via [`BoardDoc::add_zone`]. Target
    /// net/layer are picked from a dropdown in the side panel (see
    /// [`EditorState::zone_net`]/[`EditorState::zone_layer`]) rather
    /// than by clicking a pin like [`Tool::PlaceVia`] does -- a pour
    /// has no single "starting pin" the way a stitching-via run does.
    DrawZone,
    /// Free-standing silkscreen text placement (see
    /// `crate::board_doc::SilkText`'s own doc comment) -- a single
    /// click drops [`EditorState::silk_text_input`] at the cursor, on
    /// [`EditorState::silk_layer`], at
    /// [`EditorState::silk_text_place_rotation_deg`] (its *own*
    /// rotation state, deliberately not shared with [`Tool::Place`]'s
    /// [`EditorState::place_rotation_deg`] -- see that field's own doc
    /// comment for why). Ghost-preview/click-commit shape closely
    /// mirrors [`Tool::Place`]'s own (see [`BoardDoc::check_silk_text_placement`]/
    /// [`BoardDoc::try_place_silk_text`]), just placing a text instead
    /// of a footprint instance.
    PlaceSilkText,
    /// Free-standing silkscreen *dot* placement (see
    /// `crate::board_doc::SilkDot`'s own doc comment) -- a single
    /// click drops a filled dot of [`EditorState::silk_dot_diameter`]
    /// at the cursor, on [`EditorState::silk_layer`] (shared with
    /// [`Tool::PlaceSilkText`]: "which side am I annotating" is the
    /// same question for both kinds of silk). No rotation state at
    /// all -- a circle has none.
    PlaceSilkDot,
}

/// An in-progress drag of an already-placed footprint: the mouse doesn't
/// directly set the footprint's position (that would visually "snap" it
/// so its origin jumps under the cursor) -- `grab_offset` preserves
/// wherever on the part it was actually grabbed. Nothing here touches
/// `doc.node`/`doc.footprints` until release (see
/// [`EditorState::finish_drag`]): `candidate_position`/`valid` only drive
/// the live ghost preview.
struct Dragging {
    id: FootprintId,
    template_index: usize,
    rotation_deg: f64,
    grab_offset: Point,
    candidate_position: Point,
    valid: bool,
}

/// [`Dragging`]'s counterpart for an already-placed
/// [`crate::board_doc::SilkText`] -- same "grab offset preserves where
/// it was actually clicked, nothing touches `doc.silk_texts` until
/// release" shape, checked every frame via
/// [`BoardDoc::check_silk_text_move`] rather than [`BoardDoc::check_placement`].
/// `rotation_deg` is carried along (not just read fresh from `doc` each
/// frame) purely so an in-progress drag can still honour an `R`-key
/// rotation mid-drag exactly like [`Dragging::rotation_deg`] already
/// does for a footprint.
struct SilkTextDrag {
    id: SilkTextId,
    rotation_deg: f64,
    grab_offset: Point,
    candidate_position: Point,
    valid: bool,
}

/// [`SilkTextDrag`]'s counterpart for an already-placed
/// [`crate::board_doc::SilkDot`] -- same "nothing touches the doc
/// until release" shape, checked per frame via
/// [`BoardDoc::check_silk_dot_move`]. No `rotation_deg`: a dot has
/// none.
struct SilkDotDrag {
    id: SilkDotId,
    grab_offset: Point,
    candidate_position: Point,
    valid: bool,
}

/// The "move this footprint and its about-to-be-placed pin-stitching
/// via together, as one rigid unit, until both fit" ghost mode --
/// entered automatically (see [`EditorState::add_pin_stitching_via_at`])
/// whenever the right-click "Add via near pin" menu's natural
/// candidate point turns out to be refused: rather than just failing
/// silently, the footprint re-enters the exact same red/green
/// drag-ghost UX an ordinary [`Dragging`] move already gives the user,
/// letting them find a spot where *both* the part and the new via
/// legally fit, then commit both together with one click. Deliberately
/// its own struct rather than a field bolted onto `Dragging`: unlike
/// an ordinary move, this one didn't start from an actual mouse-down
/// (there's no button being held), so it needs its own frame-update/
/// commit methods driven purely by hover position, matching
/// [`Tool::Place`]'s own hover-follows-then-click-commits shape rather
/// than `Dragging`'s press-drag-release one.
struct PendingPinVia {
    footprint_id: FootprintId,
    template_index: usize,
    rotation_deg: f64,
    pad_id: ItemId,
    net: NetId,
    diameter: Unit,
    drill: Unit,
    stub_width: Unit,
    /// Fixed world-space offset from the footprint's own position to
    /// the via's own candidate center, captured once when this mode
    /// began and re-applied every frame -- what actually makes the
    /// footprint and the not-yet-placed via move as a single rigid
    /// unit rather than the via staying pinned to its own original
    /// (refused) spot while only the footprint follows the cursor.
    via_offset: Point,
    /// Same role as [`Dragging::grab_offset`]: preserves the footprint's
    /// spatial relationship to wherever the cursor actually was when
    /// this mode began (right on the pin that was right-clicked),
    /// rather than snapping the footprint's origin under the cursor.
    grab_offset: Point,
    candidate_position: Point,
    valid: bool,
}

/// A "Place part" panel delete button's click, staged in
/// [`EditorState::pending_delete`] rather than acted on immediately --
/// a single stray click on either delete button (especially a
/// category header's, which can silently remove anywhere from one to
/// several hundred parts at once) used to be permanent and
/// instantaneous with no way back. Carries everything
/// [`draw_delete_confirmation_window`] needs to both word the "are you
/// sure?" prompt and, on confirmation, actually perform the delete --
/// so that function never has to re-derive a part's name or a
/// category's part count from scratch.
enum PendingDelete {
    /// One database-backed template's own [`crate::parts_db::PartsDb`]
    /// row id (`templates`/`template_origin`/etc.'s shared index, same
    /// pair [`place_part_row`] already carries in its own
    /// `delete_part_requested` out-param) plus its display name, purely
    /// so the confirmation prompt can name the part instead of a bare
    /// id.
    Part { index: usize, db_id: i64, name: String },
    /// A category tree header's delete click -- the exact prefix
    /// [`crate::parts_db::PartsDb::delete_category_tree`] would be
    /// given, plus how many parts it would actually remove (already
    /// known at the header's own render site, see
    /// `group_templates_by_category`'s doc comment), so the prompt can
    /// say "delete all 17 parts under X?" instead of leaving the count
    /// a surprise.
    Category { prefix: String, count: usize },
}

/// Rounds `p` to the nearest multiple of `spacing` in both X and Y --
/// [`EditorState::update_drag`]'s and the `Tool::Place` click handler's
/// shared "snap to grid" primitive. Kept as a free function rather
/// than an `&self` method so it can be called while a caller already
/// holds a live `&mut self.dragging` borrow (a method taking `&self`
/// would conflict with that, since Rust can't see "only reads
/// `grid_spacing`/`grid_snap_enabled`" through an opaque method call
/// the way it can see two directly-named disjoint fields).
/// `enabled: false` (or a non-positive `spacing`) is a no-op passthrough,
/// so every call site stays correct even if a caller forgets to check
/// `grid_snap_enabled` itself first.
fn snap_to_grid_point(p: Point, spacing: Unit, enabled: bool) -> Point {
    if !enabled || spacing <= 0 {
        return p;
    }
    let round = |v: Unit| ((v as f64 / spacing as f64).round() as Unit) * spacing;
    Point::new(round(p.x), round(p.y))
}

struct EditorState {
    doc: BoardDoc,
    camera: Camera,
    fitted: bool,
    layers: LayerToggles,
    /// Whether [`draw_ratsnest`]'s "still needs a track here" lines are
    /// drawn at all -- a straight line between two already-connected
    /// pads that happen to sit far apart can visually cut right across
    /// an unrelated, still-unconnected part placed in between, which
    /// reads as "this new part got wired up on its own" even though
    /// its own pads are untouched (see the exchange this toggle came
    /// out of). Kept separate from [`LayerToggles`] rather than added
    /// there: every other toggle in it hides real board *content*
    /// (`alladin_render::draw_board`'s own concern), while this one
    /// hides a purely editor-side visual aid this module draws itself.
    show_ratsnest: bool,
    /// The trace width every new manual route (mouse-driven
    /// [`Self::handle_route_click`], or a pin-stitching via's own short
    /// connecting stub) uses -- editable via the toolbar's "Trace width"
    /// field instead of always being [`crate::routing::DEFAULT_TRACE_WIDTH`].
    /// A [`crate::routing::RoutingDrag`] already in progress keeps
    /// whatever width it started with (see that struct's own `width`
    /// field) -- changing this mid-drag only affects the *next* one.
    trace_width: Unit,
    /// The via diameter/drill every new manually-placed via
    /// ([`Tool::PlaceVia`], `V`-mid-drag layer switch, or a pin-
    /// stitching via) uses -- same "editable default, MCP calls can
    /// still override per-call" relationship to
    /// [`crate::board_doc::DEFAULT_VIA_DIAMETER`]/[`crate::board_doc::DEFAULT_VIA_DRILL`]
    /// that `trace_width` has to `DEFAULT_TRACE_WIDTH` above.
    via_diameter: Unit,
    via_drill: Unit,
    /// The spacing of the placement snap grid -- every [`Tool::Place`]
    /// click (single part or matrix) and every already-placed
    /// footprint drag (see [`Self::update_drag`]) rounds its resulting
    /// position to the nearest multiple of this, in both X and Y,
    /// whenever [`Self::grid_snap_enabled`] is on. Defaults to a plain
    /// 1mm -- the smallest grid "line parts up neatly next to/under
    /// each other" manual placement actually needs -- editable via the
    /// toolbar's "Grid (mm)" field. Also what [`draw_placement_grid`]
    /// draws dots at, so the grid a part snaps to is never invisible.
    grid_spacing: Unit,
    /// Whether [`snap_to_grid_point`] actually rounds at all, and
    /// whether [`draw_placement_grid`] draws anything -- on by default
    /// (unlike e.g. `show_ratsnest`'s off-by-default) since a fresh
    /// board with no parts on it yet has nothing a grid could get in
    /// the way of, and the whole point of this field existing is to
    /// make "cleanly aligned" the default outcome of an ordinary click/
    /// drag, not an opt-in extra step.
    grid_snap_enabled: bool,
    templates: Vec<FootprintTemplate>,
    /// `template_origin[i]` is `Some(db_row_id)` if `templates[i]` came
    /// from the user's [`crate::parts_db::PartsDb`], `None` if it's one
    /// of [`footprint::builtin_templates`] -- the only thing that decides
    /// whether the "Parts Database" panel offers a delete button for it
    /// (built-ins aren't the user's data to delete).
    template_origin: Vec<Option<i64>>,
    /// `template_hover[i]` is the tooltip text for `templates[i]` (its
    /// database description/LCSC code, when it has one) -- kept as a
    /// parallel vec for the same reason as `template_origin`.
    template_hover: Vec<Option<String>>,
    /// `template_category[i]` is `templates[i]`'s own `parts_db`
    /// category (see [`crate::parts_db::PartRecord::category`]'s doc
    /// comment), `None` for a built-in template or a database-backed
    /// one with no category set -- another parallel vec, same reason as
    /// `template_origin`/`template_hover`. What the "Place part" panel
    /// groups its collapsible category tree by.
    template_category: Vec<Option<String>>,
    /// A "Place part" panel delete button (single part or a whole
    /// category) that was clicked but not yet confirmed -- see
    /// [`PendingDelete`]'s own doc comment for why this two-step
    /// "stage it, then confirm" is here at all. `None` means no
    /// confirmation dialog is showing.
    pending_delete: Option<PendingDelete>,
    tool: Tool,
    place_rotation_deg: f64,
    /// [`Self::place_rotation_deg`]'s own counterpart for
    /// [`Tool::PlaceSilkText`], deliberately its *own* field rather
    /// than sharing that one: a footprint placement left rotated (say
    /// 90/180 from stepping through a matrix) must never silently
    /// carry that rotation over into the next silk text a user places
    /// with a completely unrelated tool switch -- "0deg is silk text's
    /// own sensible standard" only holds if switching to
    /// [`Tool::PlaceSilkText`] actually resets this field each time
    /// (see the "Place silk text" tab's click handler).
    silk_text_place_rotation_deg: f64,
    /// How many rows/columns of the currently selected [`Tool::Place`]
    /// template get placed at once, at [`Self::matrix_pitch_x_mm`]/
    /// [`Self::matrix_pitch_y_mm`] spacing -- `1`x`1` (the default)
    /// degenerates to an ordinary single-part placement, so there is no
    /// separate "matrix mode" flag: [`Self::matrix_ghost_positions`]
    /// just returns one position in that case. Persists across tool/
    /// template switches on purpose (unlike `via_net`/`zone_points`,
    /// which reset on Escape) -- placing several same-sized parts in a
    /// row, then switching to a differently-sized part for the next
    /// row of the same panel, is a plausible real workflow.
    matrix_rows: u32,
    matrix_cols: u32,
    matrix_pitch_x_mm: f32,
    matrix_pitch_y_mm: f32,
    selected: Option<FootprintId>,
    /// A selected bare `Item::Track`/`Item::Via` (see
    /// [`BoardDoc::track_at`]/[`BoardDoc::via_at`]) -- mutually
    /// exclusive with [`Self::selected`] (clicking one clears the
    /// other), since Delete/Backspace can only apply to whichever kind
    /// is currently selected. This is the anchor for the "delete this
    /// whole wire, leave the net alone" UI action -- Delete/Backspace
    /// removes every leg/via [`BoardDoc::connected_wire`] finds
    /// starting from here, not just this one item, since a routed
    /// connection between two pins is almost never a single
    /// `Item::Track` (one per corner). Deleting the whole net (every
    /// wire on it) is the net panel's own "\u{2716}" button instead
    /// (see [`BoardDoc::remove_net`]'s doc comment for why the two need
    /// to be different actions in the first place).
    selected_item: Option<ItemId>,
    dragging: Option<Dragging>,
    /// The in-progress "reposition this footprint and its new pin-via
    /// together until both fit" ghost (see [`PendingPinVia`]'s own doc
    /// comment) -- `None` whenever no right-click "Add via near pin"
    /// attempt is currently stuck needing a new spot.
    pending_pin_via: Option<PendingPinVia>,
    /// Which pad (if any) was under the cursor the moment the canvas's
    /// right-click context menu was opened -- captured once, right on
    /// `secondary_clicked()`, rather than re-derived from the current
    /// hover position every frame the menu stays open (which may no
    /// longer report the pad once the popup itself has mouse focus).
    context_menu_pad: Option<ItemId>,
    /// An in-progress "grab a trace segment and drag it" gesture in
    /// [`Tool::Select`] (see `crate::routing::TraceDrag`) -- started by
    /// [`Self::begin_drag`] when the drag didn't start on a footprint,
    /// but did land on an `Item::Track`. Mutually exclusive with
    /// [`Self::dragging`] for the same reason `selected`/`selected_item`
    /// are: only one thing can be grabbed by a single mouse-down.
    trace_dragging: Option<TraceDrag>,
    /// The first pin clicked in an in-progress "connect two pins" gesture
    /// (see [`Tool::Connect`]) -- `None` when nothing is pending.
    pending_connect: Option<ItemId>,
    /// The reason the *last* connect/disconnect attempt was refused, if
    /// any -- shown once in the side panel until the next attempt
    /// replaces or clears it. Purely informational, never affects state.
    net_message: Option<String>,
    /// The in-progress interactive routing drag (see [`Tool::Route`] and
    /// `crate::routing::RoutingDrag`), if any.
    routing: Option<RoutingDrag>,
    /// Why the last routing click was refused (no net on the start pin,
    /// or no legal route to where the user clicked) -- same "shown until
    /// replaced" convention as `net_message`.
    route_message: Option<String>,
    /// The net an in-progress [`Tool::PlaceVia`] session is stitching --
    /// `None` until the user has clicked a pin to pick one, at which
    /// point every further click places a via on it. Reset on tool
    /// change/Escape, same lifecycle as `pending_connect`.
    via_net: Option<NetId>,
    /// Why the last via placement (or net pick) was refused, if any --
    /// same "shown until replaced" convention as `net_message`/
    /// `route_message`.
    via_message: Option<String>,
    /// The in-progress polygon vertices for an active [`Tool::DrawZone`]
    /// session -- empty until the first click, cleared on finish/
    /// cancel/Esc/tool change.
    zone_points: Vec<Point>,
    /// Target net for the zone currently being drawn, picked from a
    /// side-panel dropdown -- `None` until the user picks one, which
    /// [`EditorState::finish_zone`] then requires before it will
    /// actually fill anything.
    zone_net: Option<NetId>,
    /// Target copper layer for the zone currently being drawn.
    zone_layer: LayerId,
    /// Why the last "finish outline" attempt was refused, if any -- same
    /// "shown until replaced" convention as `net_message`/`route_message`/
    /// `via_message`.
    zone_message: Option<String>,
    /// The text an active [`Tool::PlaceSilkText`] session will place on
    /// the next click -- freely editable in the side panel the whole
    /// time the tool is active, so the user can place several
    /// different labels in a row without reselecting the tool.
    silk_text_input: String,
    /// Target side for the silk text currently being placed, picked
    /// from a side-panel toggle -- same "which side" reuse of
    /// [`LayerId`] `crate::board_doc::SilkText::layer` itself uses.
    silk_layer: LayerId,
    /// Why the last [`Tool::PlaceSilkText`] click was refused, if any
    /// -- same "shown until replaced" convention as `zone_message`/
    /// `via_message`.
    silk_text_message: Option<String>,
    /// The character height an active [`Tool::PlaceSilkText`] session
    /// places its next text at, and (while one is selected in
    /// [`Tool::Select`]) what the size-stepper's "bigger"/"smaller"
    /// buttons resize [`Self::selected_silk_text`] to -- one of
    /// [`SILK_TEXT_HEIGHT_STEPS_MM`], never a free-form value (see that
    /// constant's own doc comment).
    silk_text_height: Unit,
    /// The placed [`crate::board_doc::SilkText`] currently selected in
    /// [`Tool::Select`], if any -- [`Self::selected`]/[`Self::selected_item`]'s
    /// counterpart for a silk text rather than a footprint/track. A
    /// third, separate field rather than folded into either of those:
    /// a `SilkTextId` isn't an `ItemId` (a silk text is never a `Node`
    /// item at all, see `SilkText`'s own doc comment) and isn't a
    /// `FootprintId` either, so it needs its own slot, kept mutually
    /// exclusive with the other two by [`Self::clear_selection`].
    selected_silk_text: Option<SilkTextId>,
    /// An in-progress "grab a placed silk text and drag it" gesture --
    /// [`Self::dragging`]'s counterpart for [`Self::selected_silk_text`],
    /// started by [`Self::begin_drag`] when the drag landed on a silk
    /// text rather than a footprint or track.
    silk_text_dragging: Option<SilkTextDrag>,
    /// The dot diameter an active [`Tool::PlaceSilkDot`] session places
    /// its next dot at, and what the size stepper resizes
    /// [`Self::selected_silk_dot`] to -- one of
    /// [`SILK_DOT_DIAMETER_STEPS_MM`], same fixed-steps reasoning as
    /// [`Self::silk_text_height`].
    silk_dot_diameter: Unit,
    /// Why the last [`Tool::PlaceSilkDot`] click (or pin-1 marker
    /// toggle) was refused, if any -- same "shown until replaced"
    /// convention as [`Self::silk_text_message`].
    silk_dot_message: Option<String>,
    /// The placed [`crate::board_doc::SilkDot`] currently selected in
    /// [`Tool::Select`], if any -- a fourth selection slot next to
    /// [`Self::selected`]/[`Self::selected_item`]/[`Self::selected_silk_text`],
    /// for the same "own id space needs its own slot" reason, kept
    /// mutually exclusive by [`Self::clear_selection`].
    selected_silk_dot: Option<SilkDotId>,
    /// An in-progress "grab a placed silk dot and drag it" gesture --
    /// [`Self::silk_text_dragging`]'s counterpart for a dot.
    silk_dot_dragging: Option<SilkDotDrag>,
    /// The board-outline-matching "solid plane" zone(s) currently active
    /// on F.Cu, if any -- see [`Self::set_layer_plane`]'s doc comment
    /// for how the "Solid F.Cu plane" checkbox uses this. A `Vec`, not a
    /// single `ZoneId`, because `BoardDoc::outline` is itself a
    /// `Vec<Polygon>` (almost always exactly one entry today, but this
    /// stays correct if that ever changes) -- one zone gets created per
    /// outline polygon. Empty, not `None`, when no plane is active,
    /// matching `ZoneRecord::item_ids`'s own "empty means nothing here
    /// right now" convention.
    front_plane_zones: Vec<ZoneId>,
    /// Same as `front_plane_zones`, for B.Cu.
    back_plane_zones: Vec<ZoneId>,
    /// Net picked in the side panel's F.Cu plane net-picker, independent
    /// of whether a plane is actually active right now -- lets the user
    /// pick the target net *before* ticking the checkbox, same shape as
    /// `Self::zone_net`.
    front_plane_net: Option<NetId>,
    /// Same as `front_plane_net`, for B.Cu.
    back_plane_net: Option<NetId>,
    /// The net currently spotlighted on the board -- `None` means "no
    /// highlight, render every net at its normal colour" (see
    /// `alladin_render::net_highlight_dim`'s doc comment for exactly
    /// what changes once this is `Some`). Toggled by clicking the small
    /// highlight button next to a net's name in the side panel's net
    /// list; deliberately independent of `selected`/`selected_item` --
    /// highlighting "which net is this" and selecting "this one
    /// specific track/via to delete/move" are two different questions,
    /// and a user should be able to do either without disturbing the
    /// other.
    highlighted_net: Option<NetId>,
    /// Where this board was last saved to/loaded from, if anywhere --
    /// `None` for a brand new, never-saved board. "Save" writes here
    /// directly; "Save As..." always asks and updates this.
    file_path: Option<PathBuf>,
    /// `Self::file_path`'s `mtime` as of the last successful load/save/
    /// reload -- compared against the file's *current* `mtime` once per
    /// frame (see [`Self::maybe_reload_from_disk`]) to detect another
    /// process (an AI/script driving the board via `crate::cli`, see
    /// that module's own doc comment) having changed the same file, so
    /// the GUI live-follows along instead of only ever reflecting
    /// whatever was open when the user last used "Open". `None` exactly
    /// when `file_path` is (a brand new, unsaved board has nothing on
    /// disk to watch).
    disk_mtime: Option<std::time::SystemTime>,
    /// `egui`'s own frame timestamp (`ui.input(|i| i.time)`) as of the
    /// last external-change check -- throttles that check (see
    /// [`Self::maybe_reload_from_disk`]) to roughly every 300ms instead
    /// of `stat()`ing the board file on every single frame.
    last_reload_check_secs: f64,
    /// The reason the *last* save/open attempt failed, if any -- same
    /// "shown until replaced" convention as `net_message`/`route_message`.
    io_message: Option<String>,
    /// The in-progress "Add part..." form, if the panel is open --
    /// `None` when closed. See [`AddPartForm`].
    add_part_form: Option<AddPartForm>,
    /// The C-number text field for "Download part (LCSC)".
    lcsc_input: String,
    /// The in-progress background download, if any -- see
    /// `crate::lcsc::fetch_in_background`. Polled once per frame with
    /// `try_recv()`; `None` when no download is running.
    lcsc_fetch: Option<std::sync::mpsc::Receiver<Result<crate::lcsc::FetchedPart, crate::lcsc::FetchError>>>,
    /// The outcome of the *last* download/save attempt (`true` = ok),
    /// shown until the next attempt replaces it -- same convention as
    /// `net_message`/`route_message`/`io_message`.
    lcsc_message: Option<(bool, String)>,
    /// The board-space position the mouse was over as of the last
    /// frame's `hover_board` computation (`None` while the pointer is
    /// outside the canvas) -- purely so [`crate::mcp`]'s
    /// `get_editor_state` tool can report it; nothing in the editor
    /// itself reads this back.
    last_hover_board: Option<Point>,
    /// The external autorouter's persisted configuration (tool
    /// folder, Python binary, routing defaults, extra args) -- loaded
    /// once per [`EditorState::new`] (cheap: a small JSON file) and
    /// edited live by [`Self::show_external_router_settings`]'s
    /// window; [`crate::external_router::ExternalRouterSettings::save`]
    /// persists it back for next launch.
    external_router_settings: crate::external_router::ExternalRouterSettings,
    /// Whether the "Autoroute (extern) settings" window is open.
    show_external_router_settings: bool,
    /// The result of the last "Diagnose" click in that settings
    /// window -- `None` before it's ever been run this session (the
    /// window shows a prompt to run it rather than a stale checklist).
    external_router_diagnose: Option<crate::external_router::DiagnoseReport>,
    /// Whether the "Autoroute (extern)..." net-selection dialog is
    /// open.
    show_autoroute_dialog: bool,
    /// Which nets are checked in that dialog -- reset to "every net
    /// with fewer tracks/vias than pads" (see
    /// [`Self::open_autoroute_dialog`]) each time the dialog is
    /// (re)opened, then freely toggled by the user before clicking
    /// "Route".
    autoroute_selected_nets: std::collections::HashSet<NetId>,
    /// The in-progress or just-finished background autoroute job, if
    /// any -- see [`AutorouteJob`].
    autoroute_job: Option<AutorouteJob>,
    /// A "Draw zone"/"Refill zones"/solid-plane-checkbox fill running
    /// on its own `crate::background::BackgroundJob`, if any -- see
    /// [`Self::start_zone_job`]'s doc comment. `.0` is what
    /// [`PcbApp::ui`]'s busy status line shows while it runs. Kept
    /// separate from [`PcbApp::pending_job`] (which covers MCP-driven
    /// writes): these are the GUI's own direct, mouse/keyboard-
    /// triggered actions, and there's no path from here back to
    /// `PcbApp`'s own fields to share a single slot with it -- see this
    /// project's "Background heavy computations" plan (development log)
    /// for the accepted, narrow race this leaves (a GUI zone edit
    /// landing while an MCP-driven write is *also* mid-flight) against
    /// the alternative of not backgrounding GUI zone actions at all.
    zone_job: Option<(&'static str, BackgroundJob<Box<dyn FnOnce(&mut EditorState) + Send>>)>,
    /// The in-progress GUI "Export manufacturing files..." background
    /// job, if any -- see [`Self::export_manufacturing_files_in_background`].
    /// Independent of `zone_job` for the same reason as above; the two
    /// can never conflict with each other anyway since this one never
    /// touches `self.doc` at all (see
    /// `crate::app::export_manufacturing_files_to_dir`'s own doc
    /// comment).
    export_job: Option<BackgroundJob<Result<crate::native_gerber::ManufacturingFiles, String>>>,
}

/// The live state of one external-autoroute run this session, from the
/// moment [`EditorState::start_autoroute`] spawns it until the user
/// either merges or discards its finished report. Kept entirely
/// separate from [`crate::external_router::AutorouteHandle`] itself
/// (which only knows about the channel/process) -- this is the
/// UI-thread-side bookkeeping around it: the accumulated live log
/// (the handle's own `Receiver` only ever yields *new* lines once,
/// see [`EditorState::poll_autoroute_job`]) and the final outcome once
/// it arrives.
struct AutorouteJob {
    handle: crate::external_router::AutorouteHandle,
    /// Every [`crate::external_router::AutorouteEvent::Log`] line seen
    /// so far, oldest first -- what the dialog's scrolling log view
    /// actually renders.
    log: Vec<String>,
    /// `Some` once [`crate::external_router::AutorouteEvent::Done`]
    /// arrives; `Err` holds a plain message rather than the original
    /// [`crate::external_router::ExternalRouterError`] (which isn't
    /// `Clone` and never needs to be retried/matched on again once
    /// it's just being displayed).
    result: Option<Result<crate::external_router::AutorouteReport, String>>,
}

/// The one parametric shape [`footprint::straight_row_template`] can
/// generate: a straight row of THT pads. Enough for a user to register
/// their own simple parts (resistors, headers, ...) into
/// [`crate::parts_db::PartsDb`] by hand; see that module's doc comment
/// for why a real LCSC/EasyEDA importer is a separate, later step.
struct AddPartForm {
    name: String,
    reference_prefix: String,
    pin_count: u32,
    pitch_mm: f32,
    pad_radius_mm: f32,
    description: String,
    /// Free-text `parts_db` category (see
    /// [`crate::parts_db::PartRecord::category`]'s doc comment) --
    /// blank (the default) means "Uncategorized", same as every
    /// hand-added part before this field existed.
    category: String,
}

impl Default for AddPartForm {
    fn default() -> Self {
        Self {
            name: String::new(),
            reference_prefix: "U".to_string(),
            pin_count: 2,
            pitch_mm: 2.54,
            pad_radius_mm: 0.45,
            description: String::new(),
            category: String::new(),
        }
    }
}

/// Every built-in template plus every part currently in `parts_db`, as
/// one flat list ready for [`EditorState::templates`], alongside a
/// parallel `template_origin` (see that field's doc comment) recording
/// which entries came from the database. A `parts_db` read failure is
/// swallowed on purpose -- a broken/missing parts file should degrade to
/// "just the built-ins available this session", not stop the editor from
/// opening at all.
pub(crate) fn load_templates(parts_db: &PartsDb) -> (Vec<FootprintTemplate>, Vec<Option<i64>>, Vec<Option<String>>, Vec<Option<String>>) {
    let mut templates = footprint::builtin_templates();
    let mut origin: Vec<Option<i64>> = vec![None; templates.len()];
    let mut hover: Vec<Option<String>> = vec![None; templates.len()];
    let mut category: Vec<Option<String>> = vec![None; templates.len()];
    if let Ok(parts) = parts_db.list_parts() {
        for part in parts {
            let tooltip = match &part.lcsc_code {
                Some(code) if !part.description.is_empty() => Some(format!("{code}: {}", part.description)),
                Some(code) => Some(code.clone()),
                None if !part.description.is_empty() => Some(part.description.clone()),
                None => None,
            };
            templates.push(part.template);
            origin.push(Some(part.id));
            hover.push(tooltip);
            category.push(part.category);
        }
    }
    (templates, origin, hover, category)
}

/// Splits every *database-backed* template's index (built-ins,
/// `template_origin[i].is_none()`, are never included -- the "Place
/// part" panel renders those separately, ungrouped, exactly as before
/// this feature) into a two-level `top-level category -> sub-category
/// -> indices` tree for that panel's own collapsible grouping. A
/// template with no category at all (see
/// [`crate::parts_db::PartRecord::category`]'s own doc comment) is
/// grouped under the literal `"Uncategorized"` top-level bucket rather
/// than silently dropped -- every part predating this feature (every
/// hand-added part, every already-imported KiCad footprint) still
/// shows up somewhere. The inner map's `""` key holds every index with
/// *no* sub-category (a plain `"Resistors"`-style category, or
/// "Uncategorized" itself) -- rendered directly under the top-level
/// header rather than one more empty-titled nested header.
fn group_templates_by_category(template_origin: &[Option<i64>], template_category: &[Option<String>]) -> BTreeMap<String, BTreeMap<String, Vec<usize>>> {
    let mut tree: BTreeMap<String, BTreeMap<String, Vec<usize>>> = BTreeMap::new();
    for (i, origin) in template_origin.iter().enumerate() {
        if origin.is_none() {
            continue;
        }
        let full_category = template_category[i].as_deref().unwrap_or(crate::parts_db::UNCATEGORIZED_LABEL);
        let (top, sub) = full_category.split_once('/').map_or((full_category, ""), |(top, sub)| (top, sub));
        tree.entry(top.to_string()).or_default().entry(sub.to_string()).or_default().push(i);
    }
    tree
}

/// One row of the "Place part" panel (shared by the ungrouped built-in
/// list and every bucket of the category tree, so they can never
/// visually drift apart): a selectable label (with the template's own
/// hover tooltip, if any) that switches to [`Tool::Place`] on click,
/// plus -- only for a database-backed template, see `template_origin`'s
/// own doc comment -- a small delete button. Returns whether the tool
/// actually changed, so the caller can clear the current selection
/// exactly like a direct click used to (kept out of this function since
/// it needs a `&mut EditorState`, not just its `tool` field).
fn place_part_row(
    ui: &mut egui::Ui,
    i: usize,
    templates: &[FootprintTemplate],
    template_origin: &[Option<i64>],
    template_hover: &[Option<String>],
    tool: &mut Tool,
    delete_part_requested: &mut Option<(usize, i64, String)>,
) -> bool {
    let mut changed = false;
    let selected = *tool == Tool::Place(i);
    ui.horizontal(|ui| {
        let label = ui.selectable_label(selected, templates[i].name.clone());
        let label = match &template_hover[i] {
            Some(tooltip) => label.on_hover_text(tooltip),
            None => label,
        };
        if label.clicked() {
            *tool = Tool::Place(i);
            changed = true;
        }
        if let Some(db_id) = template_origin[i] {
            if ui.small_button("\u{2716}").on_hover_text("Remove from parts database").clicked() {
                *delete_part_requested = Some((i, db_id, templates[i].name.clone()));
            }
        }
    });
    changed
}

/// Matches already-loaded `ZoneRecord`s in `doc.zones` back to
/// `EditorState`'s own transient `front_plane_zones`/`back_plane_zones`
/// bookkeeping -- see [`EditorState::new`]'s call site for why this
/// reconciliation has to happen at all. A `ZoneRecord` on `layer` whose
/// own outline is exactly one of `doc.outline`'s polygons is treated as
/// a plane; every match's net is expected to agree (`Self::set_layer_plane`
/// only ever creates same-net records for the same layer in one go), so
/// the first one found wins for the returned `NetId` if they don't.
fn detect_plane_zones(doc: &BoardDoc, layer: LayerId) -> (Vec<ZoneId>, Option<NetId>) {
    let matches: Vec<&ZoneRecord> = doc.zones.iter().filter(|z| z.layer == layer && doc.outline.iter().any(|o| *o == z.outline)).collect();
    let net = matches.first().map(|z| z.net);
    (matches.into_iter().map(|z| z.id).collect(), net)
}

impl EditorState {
    fn new(
        doc: BoardDoc,
        templates: Vec<FootprintTemplate>,
        template_origin: Vec<Option<i64>>,
        template_hover: Vec<Option<String>>,
        template_category: Vec<Option<String>>,
    ) -> Self {
        // `front_plane_zones`/`back_plane_zones` (`Self::set_layer_plane`'s
        // own bookkeeping for the "Solid F.Cu/B.Cu plane" checkboxes) is
        // transient `EditorState`, never persisted -- but the
        // `ZoneRecord`s it created very much are, straight through a
        // save/load round-trip. Without reconciling the two right here,
        // a freshly loaded board that already had a plane on disk would
        // show its checkbox unchecked; re-checking it then wouldn't
        // remove that already-loaded zone at all (nothing in the empty
        // `Vec::new()` below names it), just add a second, independent
        // one right on top -- both now silently drifting apart the
        // moment anything on the board moves, since only the *new* one
        // ever gets refilled again. A `ZoneRecord` whose own outline is
        // exactly one of the board's own outline polygons can only ever
        // have been created by `Self::set_layer_plane` in the first
        // place (`Tool::DrawZone` outlines are hand-drawn, essentially
        // never identical to the board edge point-for-point), so that's
        // exactly what's matched back up here.
        let (front_plane_zones, front_plane_net) = detect_plane_zones(&doc, LayerId::FCu);
        let (back_plane_zones, back_plane_net) = detect_plane_zones(&doc, LayerId::BCu);
        Self {
            doc,
            camera: Camera::default(),
            fitted: false,
            layers: LayerToggles::default(),
            show_ratsnest: true,
            trace_width: crate::routing::DEFAULT_TRACE_WIDTH,
            via_diameter: DEFAULT_VIA_DIAMETER,
            via_drill: DEFAULT_VIA_DRILL,
            grid_spacing: MM,
            grid_snap_enabled: true,
            templates,
            template_origin,
            template_hover,
            template_category,
            pending_delete: None,
            tool: Tool::Select,
            place_rotation_deg: 0.0,
            silk_text_place_rotation_deg: 0.0,
            matrix_rows: 1,
            matrix_cols: 1,
            matrix_pitch_x_mm: 5.0,
            matrix_pitch_y_mm: 5.0,
            selected: None,
            selected_item: None,
            trace_dragging: None,
            dragging: None,
            pending_pin_via: None,
            context_menu_pad: None,
            pending_connect: None,
            net_message: None,
            routing: None,
            route_message: None,
            via_net: None,
            via_message: None,
            zone_points: Vec::new(),
            zone_net: None,
            zone_layer: LayerId::FCu,
            zone_message: None,
            silk_text_input: String::new(),
            silk_layer: LayerId::FCu,
            silk_text_message: None,
            silk_text_height: DEFAULT_SILK_TEXT_HEIGHT,
            selected_silk_text: None,
            silk_text_dragging: None,
            silk_dot_diameter: DEFAULT_SILK_DOT_DIAMETER,
            silk_dot_message: None,
            selected_silk_dot: None,
            silk_dot_dragging: None,
            front_plane_zones,
            back_plane_zones,
            front_plane_net,
            back_plane_net,
            zone_job: None,
            export_job: None,
            highlighted_net: None,
            file_path: None,
            disk_mtime: None,
            last_reload_check_secs: 0.0,
            io_message: None,
            add_part_form: None,
            lcsc_input: String::new(),
            lcsc_fetch: None,
            lcsc_message: None,
            last_hover_board: None,
            external_router_settings: crate::external_router::ExternalRouterSettings::load(),
            show_external_router_settings: false,
            external_router_diagnose: None,
            show_autoroute_dialog: false,
            autoroute_selected_nets: std::collections::HashSet::new(),
            autoroute_job: None,
        }
    }

    fn board_bounds(&self) -> Option<Aabb> {
        self.doc
            .outline
            .iter()
            .map(Aabb::from_polygon)
            .reduce(|a, b| Aabb {
                min: Point::new(a.min.x.min(b.min.x), a.min.y.min(b.min.y)),
                max: Point::new(a.max.x.max(b.max.x), a.max.y.max(b.max.y)),
            })
    }

    /// The already-placed footprint's own template, looked up by the
    /// name `BoardDoc` stored for it (see `PlacedFootprint::template_name`'s
    /// doc comment for why `BoardDoc` itself doesn't hold this mapping).
    fn template_for(&self, id: FootprintId) -> Option<(usize, &FootprintTemplate)> {
        let name = &self.doc.footprints.iter().find(|f| f.id == id)?.template_name;
        self.templates.iter().enumerate().find(|(_, t)| &t.name == name)
    }

    /// A footprint under the cursor takes priority (matching this
    /// tool's original, drag-to-move behaviour); failing that, a bare
    /// `Item::Track` starts a [`TraceDrag`] instead (see
    /// [`Self::trace_dragging`]) -- there's deliberately no via
    /// equivalent, a via's own position never moves this way (see
    /// `crate::routing::resolve_anchor`'s doc comment). Missing both
    /// clears the selection, same "click elsewhere deselects"
    /// convention [`Self::handle_select_click`] uses, and leaves the
    /// drag free to pan the camera instead.
    fn begin_drag(&mut self, board_pos: Point) {
        if let Some(id) = self.doc.footprint_at(board_pos) {
            self.clear_selection();
            self.selected = Some(id);
            let Some((template_index, _)) = self.template_for(id) else { return };
            let position = self.doc.footprints.iter().find(|f| f.id == id).unwrap().position;
            let rotation_deg = self.doc.footprints.iter().find(|f| f.id == id).unwrap().rotation_deg;
            self.dragging = Some(Dragging {
                id,
                template_index,
                rotation_deg,
                grab_offset: position.sub(board_pos),
                candidate_position: position,
                valid: true,
            });
            return;
        }
        if let Some(id) = self.doc.silk_text_at(board_pos) {
            self.clear_selection();
            self.selected_silk_text = Some(id);
            let text = self.doc.silk_texts.iter().find(|t| t.id == id).unwrap();
            let position = text.position;
            let rotation_deg = text.rotation_deg;
            self.silk_text_dragging = Some(SilkTextDrag {
                id,
                rotation_deg,
                grab_offset: position.sub(board_pos),
                candidate_position: position,
                valid: true,
            });
            return;
        }
        if let Some(id) = self.doc.silk_dot_at(board_pos) {
            self.clear_selection();
            self.selected_silk_dot = Some(id);
            let position = self.doc.silk_dots.iter().find(|d| d.id == id).unwrap().position;
            self.silk_dot_dragging = Some(SilkDotDrag { id, grab_offset: position.sub(board_pos), candidate_position: position, valid: true });
            return;
        }
        let tolerance = self.snap_threshold_px(5.0);
        if let Some(track_id) = self.doc.track_at(board_pos, tolerance) {
            if let Some(drag) = TraceDrag::start(&self.doc, track_id) {
                self.clear_selection();
                self.selected_item = Some(track_id);
                self.trace_dragging = Some(drag);
                return;
            }
        }
        self.clear_selection();
    }

    fn update_drag(&mut self, board_pos: Point) {
        let grid_spacing = self.grid_spacing;
        let grid_snap_enabled = self.grid_snap_enabled;
        if let Some(dragging) = &mut self.dragging {
            let candidate = snap_to_grid_point(board_pos.add(dragging.grab_offset), grid_spacing, grid_snap_enabled);
            dragging.candidate_position = candidate;
            let template = &self.templates[dragging.template_index];
            dragging.valid = self.doc.check_placement(template, candidate, dragging.rotation_deg, Some(dragging.id)).is_ok();
            return;
        }
        if let Some(drag) = &mut self.silk_text_dragging {
            let candidate = snap_to_grid_point(board_pos.add(drag.grab_offset), grid_spacing, grid_snap_enabled);
            drag.candidate_position = candidate;
            drag.valid = self.doc.check_silk_text_move(drag.id, candidate, drag.rotation_deg).is_ok();
            return;
        }
        let Some(drag) = &mut self.silk_dot_dragging else { return };
        let candidate = snap_to_grid_point(board_pos.add(drag.grab_offset), grid_spacing, grid_snap_enabled);
        drag.candidate_position = candidate;
        drag.valid = self.doc.check_silk_dot_move(drag.id, candidate).is_ok();
    }

    fn finish_drag(&mut self) {
        if let Some(dragging) = self.dragging.take() {
            let template = &self.templates[dragging.template_index];
            // Errors are deliberately ignored: `try_move_footprint` leaves
            // everything untouched on `Err`, so "do nothing" *is* the correct
            // "snap back to where it was" behaviour -- no separate rollback.
            let _ = self.doc.try_move_footprint(dragging.id, template, dragging.candidate_position, dragging.rotation_deg);
            return;
        }
        if let Some(drag) = self.silk_text_dragging.take() {
            let _ = self.doc.try_move_silk_text(drag.id, drag.candidate_position, drag.rotation_deg);
            return;
        }
        let Some(drag) = self.silk_dot_dragging.take() else { return };
        let _ = self.doc.try_move_silk_dot(drag.id, drag.candidate_position);
    }

    /// The right-click "Add via near pin" menu's one action: tries the
    /// natural spot right away (see [`BoardDoc::try_add_pin_stitching_via`]),
    /// and only falls back to the interactive
    /// [`Self::begin_pin_via_relocation`] ghost if that first, most
    /// common case is refused -- most pins have room right next to
    /// them, and those shouldn't need a click-drag-click dance just to
    /// confirm a spot that was already fine.
    fn add_pin_stitching_via_at(&mut self, pad_id: ItemId) {
        use crate::board_doc::PinStitchingViaError;
        match self.doc.try_add_pin_stitching_via(pad_id, self.via_diameter, self.via_drill, self.trace_width) {
            Ok(_) => {
                self.pending_pin_via = None;
                self.io_message = None;
            }
            // Neither of these can ever be fixed by moving the part
            // to a different spot -- entering the relocation ghost for
            // them would just silently do nothing (there's no net to
            // find a `via_offset`/`net` for in the first place), the
            // exact bug this arm exists to avoid: report it plainly
            // instead.
            Err(e @ (PinStitchingViaError::NotAPad | PinStitchingViaError::NoNet)) => {
                self.io_message = Some(format!("Couldn't add a via there: {e}"));
            }
            Err(e) => self.begin_pin_via_relocation(pad_id, e),
        }
    }

    /// Enters [`Self::pending_pin_via`]: the owning footprint and a
    /// not-yet-placed via at the same fixed offset from it now move as
    /// one unit under the cursor (see [`Self::update_pending_pin_via`]),
    /// exactly like an ordinary [`Self::begin_drag`] move, until the
    /// user finds a spot where both fit and clicks to commit (see
    /// [`Self::finish_pending_pin_via`]).
    fn begin_pin_via_relocation(&mut self, pad_id: ItemId, reason: crate::board_doc::PinStitchingViaError) {
        // Every early return below is a "shouldn't happen" case at
        // this point (`add_pin_stitching_via_at` already filtered out
        // the one real, common reason `pad_net`/the template lookup
        // could fail -- no net yet, see `PinStitchingViaError::NoNet`)
        // -- still surfaced as a message rather than silently doing
        // nothing, which is exactly the bug this whole function exists
        // to avoid.
        let Some(net) = self.doc.pad_net(pad_id).ok().flatten() else {
            self.io_message = Some(format!("Couldn't add a via there: {reason}"));
            return;
        };
        let Some(footprint) = self.doc.footprints.iter().find(|f| f.pad_item_ids.contains(&pad_id)) else {
            self.io_message = Some(format!("Couldn't add a via there: {reason}"));
            return;
        };
        let footprint_id = footprint.id;
        let position = footprint.position;
        let rotation_deg = footprint.rotation_deg;
        let Some((template_index, _)) = self.template_for(footprint_id) else {
            self.io_message = Some(format!("Couldn't add a via there: {reason}"));
            return;
        };
        let Some(via_candidate) = self.doc.pin_stitching_via_candidate(pad_id, self.via_diameter) else {
            self.io_message = Some(format!("Couldn't add a via there: {reason}"));
            return;
        };
        let cursor = self.last_hover_board.unwrap_or(position);

        self.clear_selection();
        self.selected = Some(footprint_id);
        self.pending_pin_via = Some(PendingPinVia {
            footprint_id,
            template_index,
            rotation_deg,
            pad_id,
            net,
            diameter: self.via_diameter,
            drill: self.via_drill,
            stub_width: self.trace_width,
            via_offset: via_candidate.sub(position),
            grab_offset: position.sub(cursor),
            candidate_position: position,
            valid: false,
        });
        self.io_message = Some(format!("Via near pin needs a new spot ({reason}) -- move the part until it turns green, then click to place both."));
    }

    fn update_pending_pin_via(&mut self, board_pos: Point) {
        let Some(pending) = &mut self.pending_pin_via else { return };
        let candidate = board_pos.add(pending.grab_offset);
        pending.candidate_position = candidate;
        let template = &self.templates[pending.template_index];
        let footprint_ok = self.doc.check_placement(template, candidate, pending.rotation_deg, Some(pending.footprint_id)).is_ok();
        let via_ok = self.doc.via_would_fit(candidate.add(pending.via_offset), pending.net, pending.diameter);
        pending.valid = footprint_ok && via_ok;
    }

    /// Commits the footprint's move and the pin-stitching via together
    /// -- clicking while [`PendingPinVia::valid`] is `false` does
    /// nothing, same "click-while-invalid is a no-op, ghost just stays
    /// up" convention [`Tool::Place`]'s own ghost already uses, rather
    /// than [`Self::finish_drag`]'s "release always ends the gesture"
    /// one: there's no mouse button being held here for a release to
    /// end in the first place, so the only way out is a valid drop or
    /// Escape.
    fn finish_pending_pin_via(&mut self) {
        let Some(pending) = &self.pending_pin_via else { return };
        if !pending.valid {
            return;
        }
        let pending = self.pending_pin_via.take().unwrap();
        let template = &self.templates[pending.template_index];
        if self.doc.try_move_footprint(pending.footprint_id, template, pending.candidate_position, pending.rotation_deg).is_err() {
            self.io_message = Some("Couldn't place the part there after all -- try again.".to_string());
            return;
        }
        match self.doc.try_add_pin_stitching_via(pending.pad_id, pending.diameter, pending.drill, pending.stub_width) {
            Ok(_) => self.io_message = None,
            Err(e) => {
                // Shouldn't normally happen right after both live
                // checks above passed, but if the board changed out
                // from under this exact frame, re-enter relocation
                // rather than silently dropping the user's request.
                self.begin_pin_via_relocation(pending.pad_id, e);
            }
        }
    }

    fn update_trace_drag(&mut self, board_pos: Point) {
        let Some(drag) = &mut self.trace_dragging else { return };
        drag.update(&self.doc, board_pos);
    }

    /// Commits the in-progress [`Self::trace_dragging`], same "leave it
    /// alone on `Err`" contract as [`Self::finish_drag`]: releasing the
    /// mouse over a blocked position just snaps back to the original
    /// trace, no separate rollback needed since nothing was ever
    /// touched until here. Clears [`Self::selected_item`] either way --
    /// on a successful commit the old id is gone (the legs were
    /// replaced with fresh ones), and re-selecting the result isn't
    /// worth the complexity for a first cut of this gesture.
    fn finish_trace_drag(&mut self) {
        let Some(drag) = self.trace_dragging.take() else { return };
        let _ = drag.commit(&mut self.doc);
        self.selected_item = None;
    }

    /// Sets [`Self::file_path`] and [`Self::disk_mtime`] together --
    /// every call site that used to just assign `file_path` directly
    /// (Open/Save/Save As) goes through this instead, so `disk_mtime`
    /// can never drift out of sync with what's actually on disk right
    /// now, which would otherwise make [`Self::maybe_reload_from_disk`]
    /// immediately (and harmlessly, but pointlessly) reload on the very
    /// next frame after every single Open/Save.
    fn set_file_path(&mut self, path: PathBuf) {
        self.disk_mtime = file_mtime(&path);
        self.file_path = Some(path);
    }

    /// Once per (throttled) frame: if [`Self::file_path`]'s `mtime` on
    /// disk has moved on from [`Self::disk_mtime`], something else
    /// changed the file since we last looked -- reload it. This is the
    /// entire "live watch an AI/script driving the board" mechanism
    /// (see `crate::cli`'s own doc comment for that interface): no
    /// polling loop or file-watcher thread, just a cheap `stat()`
    /// dropped into the frame that's already running anyway.
    fn maybe_reload_from_disk(&mut self, parts_db: &PartsDb, now_secs: f64) {
        if now_secs - self.last_reload_check_secs < 0.3 {
            return;
        }
        self.last_reload_check_secs = now_secs;
        let Some(path) = self.file_path.clone() else { return };
        if file_mtime(&path) != self.disk_mtime {
            // Bumped here, *before* the background reload below even
            // starts, not once it resolves -- `Self::reload_from_disk`
            // can now take a while for a board with real zones (see its
            // own doc comment), and leaving the *old* `disk_mtime` in
            // place until then would just make every following frame
            // notice the same already-in-flight change all over again
            // and queue up a redundant second reload job on top of the
            // first ([`Self::start_zone_job`]'s own "one job at a time"
            // guard would refuse it anyway, but there's no reason to
            // even try).
            self.disk_mtime = file_mtime(&path);
            self.reload_from_disk(&path, parts_db);
        }
    }

    /// Re-reads `path` from disk -- both the board itself and the
    /// template list, since the external change that triggered this
    /// (see [`Self::maybe_reload_from_disk`]) might have been a
    /// `download-part` that only added to `crate::parts_db`, not this
    /// board file -- and swaps it in, leaving everything about *how*
    /// the user is looking at the board untouched: camera, zoom,
    /// current tool. Any in-progress interaction that could be holding
    /// onto now-invalid `ItemId`/`FootprintId`s from the *old* doc is
    /// dropped first. Silently does nothing on a read/parse failure --
    /// "keep showing the last good state and retry next tick" rather
    /// than surfacing a scary transient error for something that, on a
    /// well-behaved writer (see `crate::app::write_atomic`), shouldn't
    /// even be reachable, and on any other writer will resolve itself
    /// within a fraction of a second anyway.
    ///
    /// Runs on a [`crate::background::BackgroundJob`] via
    /// [`Self::start_zone_job`] (despite the name, that helper is
    /// generic over any `T`, not just an actual zone fill -- see its
    /// own doc comment) rather than inline, for the exact same reason
    /// as every other hotspot this project's "Background heavy
    /// computations" plan (development log) covers: [`load_from_path`]
    /// re-fills every one of the board's own saved zones from scratch
    /// (see `crate::persistence::from_json`'s doc comment on why), so
    /// on a real board this "just noticed the file changed" check used
    /// to freeze the whole editor -- including an AI/script's own
    /// `save_board` call over MCP appearing to hang, since that reply
    /// only comes back once the *next* `PcbApp::ui` frame runs -- for
    /// however long that fill took. A fresh [`PartsDb`] connection is
    /// opened on the worker thread rather than sharing `parts_db`'s
    /// (multiple connections to one on-disk SQLite file are fine for
    /// this read-mostly usage -- see `spawn_run_batch_job`'s own doc
    /// comment for the identical reasoning).
    fn reload_from_disk(&mut self, path: &Path, parts_db: &PartsDb) {
        let _ = parts_db;
        let job_path = path.to_path_buf();
        let apply_path = path.to_path_buf();
        self.start_zone_job(
            "board reload",
            "Reloading board from disk...",
            move || {
                let scratch_parts_db =
                    PartsDb::open(&PartsDb::default_path()).or_else(|_| PartsDb::open_in_memory()).expect("an in-memory sqlite database must always succeed");
                let (templates, template_origin, template_hover, template_category) = load_templates(&scratch_parts_db);
                let doc = load_from_path(&job_path, &templates);
                (doc, templates, template_origin, template_hover, template_category)
            },
            move |state, (doc, templates, template_origin, template_hover, template_category)| {
                let Ok(doc) = doc else {
                    state.zone_message = Some("Reload from disk failed -- keeping the last good board (will retry on the next external change).".to_string());
                    return;
                };
                state.dragging = None;
                state.trace_dragging = None;
                state.routing = None;
                state.pending_connect = None;
                state.clear_selection();

                state.doc = doc;
                state.templates = templates;
                state.template_origin = template_origin;
                state.template_hover = template_hover;
                state.template_category = template_category;
                state.disk_mtime = file_mtime(&apply_path);
                state.zone_message = Some("Board reloaded from disk.".to_string());
            },
        );
    }

    /// Clears both kinds of selection at once (see [`Self::selected_item`]'s
    /// doc comment for why there are two) -- every tool switch and
    /// empty-space click uses this instead of touching `selected` alone,
    /// so a stray Delete/Backspace afterwards never lands on whatever
    /// was selected before the switch.
    fn clear_selection(&mut self) {
        self.selected = None;
        self.selected_item = None;
        self.selected_silk_text = None;
        self.selected_silk_dot = None;
    }

    /// Rotates the currently selected (and not currently being dragged)
    /// footprint *or* placed silk text by 90°, refused (silently, same
    /// "do nothing on `Err`" pattern as everywhere else here) if that
    /// would collide or leave the board. The `R`-key's [`Tool::Select`]
    /// handler for both kinds of selection at once -- a footprint and a
    /// silk text can never be selected simultaneously (see
    /// [`Self::clear_selection`]), so at most one of these two branches
    /// ever actually does anything on a given call.
    fn rotate_selected(&mut self) {
        if let Some(id) = self.selected {
            let Some((template_index, _)) = self.template_for(id) else { return };
            let footprint = self.doc.footprints.iter().find(|f| f.id == id).unwrap();
            let position = footprint.position;
            let new_rotation = (footprint.rotation_deg + 90.0) % 360.0;
            let template = &self.templates[template_index];
            let _ = self.doc.try_move_footprint(id, template, position, new_rotation);
            return;
        }
        if let Some(id) = self.selected_silk_text {
            let Some(text) = self.doc.silk_texts.iter().find(|t| t.id == id) else { return };
            let position = text.position;
            let new_rotation = (text.rotation_deg + 90.0) % 360.0;
            let _ = self.doc.try_move_silk_text(id, position, new_rotation);
        }
    }

    /// One click in [`Tool::Connect`] mode, already resolved to whichever
    /// pad (if any) is under the cursor. `unassign` (held modifier key,
    /// see the canvas handler) disconnects that pad instead of
    /// continuing the two-click "connect" gesture. Clicking empty space
    /// (`pad_id: None`) cancels a pending first pin, matching every other
    /// tool's "click elsewhere to back out" convention in this editor.
    fn handle_connect_click(&mut self, pad_id: Option<ItemId>, unassign: bool) {
        let Some(pad_id) = pad_id else {
            self.pending_connect = None;
            return;
        };
        if unassign {
            let _ = self.doc.disconnect_pad(pad_id);
            self.pending_connect = None;
            return;
        }
        match self.pending_connect.take() {
            None => self.pending_connect = Some(pad_id),
            Some(first) if first == pad_id => {} // clicked the same pin twice: no-op
            Some(first) => match self.doc.connect_pads(first, pad_id) {
                Ok(_) => self.net_message = None,
                Err(e) => self.net_message = Some(e.to_string()),
            },
        }
    }

    /// One click in [`Tool::Route`] mode. The first click (no drag yet)
    /// tries to pick up a trace from whatever pin is under the cursor;
    /// the second click tries to dock it onto a different pin. Clicking
    /// empty space cancels a pending drag, same "click elsewhere backs
    /// out" convention [`Self::handle_connect_click`] already uses.
    fn handle_route_click(&mut self, board_pos: Point) {
        let Some(pad_id) = self.doc.pad_at(board_pos) else {
            self.routing = None;
            return;
        };

        match self.routing.take() {
            None => match RoutingDrag::start_with_options(&self.doc, pad_id, self.trace_width, self.via_diameter, self.via_drill) {
                Some(drag) => {
                    self.routing = Some(drag);
                    self.route_message = None;
                }
                None => self.route_message = Some("This pin has no net yet \u{2014} connect it to one first.".to_string()),
            },
            Some(drag) if pad_id == drag.from_pad => {
                self.routing = Some(drag); // clicked the start pin again: keep dragging
            }
            Some(mut drag) => {
                drag.update(&self.doc, board_pos);
                if drag.commit(&mut self.doc) {
                    self.route_message = None;
                } else {
                    self.route_message = drag.blocked_reason(&self.doc, board_pos).or(Some("can't connect these two pins".to_string()));
                    self.routing = Some(drag); // keep the session alive so the user can try elsewhere
                }
            }
        }
    }

    /// One click in [`Tool::Select`] mode, once no footprint drag is in
    /// progress: a footprint's pad takes priority (matching this tool's
    /// original, drag-to-move behaviour), otherwise falls back to
    /// hit-testing a bare `Item::Track`/`Item::Via` (see
    /// [`BoardDoc::track_at`]/[`BoardDoc::via_at`]) so it can be
    /// selected and deleted on its own, leaving its net intact. Missing
    /// both clears whatever was selected before, same "click elsewhere
    /// deselects" convention every other tool's click handler uses.
    fn handle_select_click(&mut self, board_pos: Point) {
        if let Some(fp_id) = self.doc.footprint_at(board_pos) {
            self.selected = Some(fp_id);
            self.selected_item = None;
            self.selected_silk_text = None;
            return;
        }
        if let Some(id) = self.doc.silk_text_at(board_pos) {
            self.clear_selection();
            self.selected_silk_text = Some(id);
            return;
        }
        if let Some(id) = self.doc.silk_dot_at(board_pos) {
            self.clear_selection();
            self.selected_silk_dot = Some(id);
            return;
        }
        let tolerance = self.snap_threshold_px(5.0);
        let item = self.doc.track_at(board_pos, tolerance).or_else(|| self.doc.via_at(board_pos, tolerance));
        self.clear_selection();
        self.selected_item = item;
    }

    /// One click in [`Tool::PlaceVia`] mode. Clicking a pin that already
    /// has a net (mirroring [`Self::handle_route_click`]'s own "start
    /// from a pin" gesture) always (re-)picks *that* net first, even if
    /// a net was already picked before -- this is deliberately checked
    /// unconditionally, not just while `via_net` is still `None`: a
    /// stitching run along a ground plane most often needs exactly one
    /// net the whole time, but if the user clicks a *different* net's
    /// pin mid-run, treating that as "place a via here instead" (the
    /// old behaviour) just produced a confusing `Collision` refusal --
    /// the pin obviously collides with itself -- instead of the more
    /// useful "switch nets" the click clearly meant. Only once the
    /// clicked point isn't on any pin at all does a click actually drop
    /// a via, on whichever net is currently picked -- and
    /// [`crate::board_doc::BoardDoc::try_add_stitching_via`] (not the
    /// plain `try_add_via`) additionally refuses a via that wouldn't
    /// touch any existing copper on that net at all, so a stray click
    /// far from the actual ground plane doesn't silently leave a
    /// pointless, disconnected via behind. Clicking empty space while no
    /// net is picked yet is a no-op (nothing to cancel); once a net *is*
    /// picked, it stays picked until the tool changes or Escape is
    /// pressed, even after a misplaced click.
    fn handle_place_via_click(&mut self, board_pos: Point) {
        if let Some(net) = self.doc.pad_at(board_pos).and_then(|id| self.doc.pad_net(id).ok().flatten()) {
            self.via_net = Some(net);
            self.via_message = None;
            return;
        }
        match self.via_net {
            None => self.via_message = Some("Click a pin that already has a net first, to pick which net to stitch.".to_string()),
            Some(net) => match self.doc.try_add_stitching_via(board_pos, net, self.via_diameter, self.via_drill) {
                Ok(_) => self.via_message = None,
                Err(e) => self.via_message = Some(e.to_string()),
            },
        }
    }

    /// A fixed screen-pixel radius converted to board units through
    /// `camera.pixels_per_mm`, so it stays the same comfortable on-
    /// screen size at any zoom level -- shared by [`Self::zone_close_threshold`]
    /// ("click back on the first zone vertex to close it") and
    /// [`Self::snap_matrix_center`] ("snap a dragged matrix onto the
    /// board's own center axis"), which are the same kind of
    /// screen-space hit-testing/snapping problem under different names.
    fn snap_threshold_px(&self, radius_px: f32) -> Unit {
        ((radius_px / self.camera.pixels_per_mm) * MM as f32) as Unit
    }

    /// The board-unit distance, at the current zoom level, under which a
    /// click on [`Self::zone_points`]'s first vertex is treated as
    /// "close the polygon" rather than "add another vertex" -- the same
    /// "click back on the start" gesture most vector-drawing tools use,
    /// so there's no need for a separate keyboard/button-only path.
    fn zone_close_threshold(&self) -> Unit {
        self.snap_threshold_px(10.0)
    }

    /// Snaps a candidate [`Tool::Place`] matrix center onto the board's
    /// own center axis, independently per axis -- within
    /// [`Self::snap_threshold_px`] of `board_center.x` alone snaps `x`
    /// (leaving `y` exactly where the mouse put it), and likewise for
    /// `y`; both can snap at once. Since [`BoardDoc::matrix_positions`]
    /// always builds the grid symmetrically around this center point,
    /// snapping it onto the board's center is exactly "make the left/
    /// right (or top/bottom) margins equal", without needing to know
    /// the matrix's own bounding size at all. Returns the snapped point
    /// plus which axes actually snapped, so the caller can draw a guide
    /// line only for those.
    fn snap_matrix_center(&self, candidate: Point) -> (Point, bool, bool) {
        let Some(bounds) = self.board_bounds() else { return (candidate, false, false) };
        let center = Point::new((bounds.min.x + bounds.max.x) / 2, (bounds.min.y + bounds.max.y) / 2);
        let threshold = self.snap_threshold_px(8.0) as f64;
        let snap_x = (candidate.x - center.x).abs() as f64 <= threshold;
        let snap_y = (candidate.y - center.y).abs() as f64 <= threshold;
        (Point::new(if snap_x { center.x } else { candidate.x }, if snap_y { center.y } else { candidate.y }), snap_x, snap_y)
    }

    /// The full set of ghost/commit positions for the currently
    /// configured [`Self::matrix_rows`]x[`Self::matrix_cols`] matrix,
    /// centered on (the already-snapped) `center` -- a thin unit
    /// conversion wrapper around [`BoardDoc::matrix_positions`], kept on
    /// `EditorState` since it's the one place that knows about the UI's
    /// millimetre-valued fields.
    fn matrix_ghost_positions(&self, center: Point) -> Vec<Point> {
        let pitch_x = (self.matrix_pitch_x_mm as f64 * MM as f64).round() as alladin_geom::Unit;
        let pitch_y = (self.matrix_pitch_y_mm as f64 * MM as f64).round() as alladin_geom::Unit;
        BoardDoc::matrix_positions(self.matrix_rows.max(1), self.matrix_cols.max(1), pitch_x, pitch_y, center)
    }

    /// One click in [`Tool::DrawZone`] mode: with fewer than three
    /// vertices placed, or the click landing away from the first vertex,
    /// just appends `board_pos` as a new polygon vertex. Once at least a
    /// triangle exists, a click back near the first vertex closes the
    /// outline instead of extending it -- the same "click back on the
    /// start" gesture most vector-drawing tools use, so there's no need
    /// for a separate keyboard/button-only path (though [`Self::
    /// finish_zone`] is also reachable via Enter/the side-panel button,
    /// for outlines where re-clicking the exact first pixel is fiddly).
    fn handle_draw_zone_click(&mut self, board_pos: Point) {
        if self.zone_points.len() >= 3 {
            if let Some(first) = self.zone_points.first() {
                if first.distance(board_pos) <= self.zone_close_threshold() as f64 {
                    self.finish_zone();
                    return;
                }
            }
        }
        self.zone_points.push(board_pos);
        self.zone_message = None;
    }

    /// Drains [`Self::zone_job`]'s result, if it's ready -- called once
    /// per frame (see [`PcbApp::ui`]), the same `poll()`-once-per-frame
    /// shape [`Self::poll_autoroute_job`] already uses for
    /// `autoroute_job`.
    fn poll_zone_job(&mut self) {
        let Some((_, job)) = &mut self.zone_job else { return };
        match job.poll() {
            JobPoll::Pending => {}
            JobPoll::Ready(apply) => {
                self.zone_job = None;
                apply(self);
            }
            JobPoll::Lost => {
                self.zone_job = None;
                self.zone_message = Some("The zone-fill background thread ended unexpectedly -- please try again.".to_string());
            }
        }
    }

    /// Starts `fill` (the actual, potentially slow zone-fill
    /// computation) on a `crate::background::BackgroundJob`, applying
    /// its result via `apply` once [`Self::poll_zone_job`] sees it
    /// resolve -- the shared plumbing behind [`Self::finish_zone`],
    /// [`Self::set_layer_plane`], and [`Self::refill_all_zones_in_background`].
    /// `label` is what [`PcbApp::ui`]'s busy status line shows while it
    /// runs; `running_message` is what `self.zone_message` is set to
    /// for the same duration. Refuses (reporting through
    /// `self.zone_message`, changing nothing else) if a zone job is
    /// already running -- this project's "Background heavy
    /// computations" plan (development log) never lets two computations
    /// race to mutate the same `self.doc`.
    fn start_zone_job<F, T>(&mut self, label: &'static str, running_message: &str, fill: F, apply: impl FnOnce(&mut EditorState, T) + Send + 'static)
    where
        F: FnOnce() -> T + Send + 'static,
        T: Send + 'static,
    {
        if let Some((running_label, _)) = &self.zone_job {
            self.zone_message = Some(format!("A zone job is already running ({running_label}) -- wait for it to finish first."));
            return;
        }
        let job = BackgroundJob::spawn(move || -> Box<dyn FnOnce(&mut EditorState) + Send> {
            let result = fill();
            Box::new(move |state: &mut EditorState| apply(state, result))
        });
        self.zone_job = Some((label, job));
        self.zone_message = Some(running_message.to_string());
    }

    /// Closes the in-progress [`Tool::DrawZone`] outline and starts its
    /// fill in the background (see [`Self::start_zone_job`]) -- refuses
    /// (leaving `zone_points` untouched, so the user doesn't lose their
    /// work) if there aren't enough vertices yet or no target net has
    /// been picked in the side panel; both are reported the same "shown
    /// until replaced" way as `via_message`/`route_message`.
    fn finish_zone(&mut self) {
        if self.zone_points.len() < 3 {
            self.zone_message = Some("Need at least 3 points to close a zone outline.".to_string());
            return;
        }
        let Some(net) = self.zone_net else {
            self.zone_message = Some("Pick a target net first.".to_string());
            return;
        };
        let outline = Polygon::new(std::mem::take(&mut self.zone_points));
        let layer = self.zone_layer;
        let node = self.doc.node.clone();
        let board_outline = self.doc.outline.clone();
        let resolver = self.doc.resolver();
        let filled_at_revision = node.obstacle_revision();
        let outline_for_fill = outline.clone();
        self.start_zone_job(
            "zone fill",
            "Filling zone...",
            move || zone_fill::fill_zone(&outline_for_fill, layer, net, &board_outline, &node, resolver),
            move |state, items| {
                let island_count = items.len();
                state.doc.insert_new_zone(outline, layer, net, items, filled_at_revision);
                state.zone_message = Some(if island_count > 0 {
                    format!("Zone filled into {island_count} island(s).")
                } else {
                    "Zone outline recorded, but the fill came back empty (fully off-board, or fully consumed by clearances) -- it can be refilled later once obstacles change.".to_string()
                });
            },
        );
    }

    /// (Re)creates or removes the whole-board-outline "solid plane"
    /// zone(s) on `layer` in the background (see
    /// [`Self::start_zone_job`]) -- the entire implementation behind
    /// the "Solid F.Cu/B.Cu plane" checkboxes and their net picker.
    /// Passing `net: None` just removes whatever plane currently exists
    /// on `layer`, if any (the "untick the checkbox" case), entirely
    /// synchronously -- there's nothing left to fill, so no background
    /// job is needed. Passing `Some` first removes any existing plane
    /// there too, so changing the net picker while a plane is already
    /// active re-fills on the new net instead of leaving a stale one
    /// behind *alongside* the new one. Handing the fill the board's own
    /// outline polygon(s) directly, rather than asking the user to
    /// trace a matching outline by hand with `Tool::DrawZone`, is the
    /// entire point of this shortcut -- the fill re-clips to the real
    /// board outline and every real obstacle's clearance anyway, so
    /// this can never "overfill" past what a painstakingly hand-traced
    /// matching outline would have.
    fn set_layer_plane(&mut self, layer: LayerId, net: Option<NetId>) {
        let old_zones = match layer {
            LayerId::FCu => std::mem::take(&mut self.front_plane_zones),
            LayerId::BCu => std::mem::take(&mut self.back_plane_zones),
        };
        for id in old_zones {
            self.doc.remove_zone(id);
        }
        let Some(net) = net else {
            match layer {
                LayerId::FCu => self.front_plane_zones = Vec::new(),
                LayerId::BCu => self.back_plane_zones = Vec::new(),
            }
            return;
        };
        let outlines = self.doc.outline.clone();
        let board_outline = self.doc.outline.clone();
        let node = self.doc.node.clone();
        let resolver = self.doc.resolver();
        let filled_at_revision = node.obstacle_revision();
        self.start_zone_job(
            "solid plane fill",
            "Filling solid plane...",
            move || {
                outlines
                    .into_iter()
                    .map(|outline| {
                        let items = zone_fill::fill_zone(&outline, layer, net, &board_outline, &node, resolver);
                        (outline, items)
                    })
                    .collect::<Vec<_>>()
            },
            move |state, results| {
                let new_zones = results
                    .into_iter()
                    .map(|(outline, items)| state.doc.insert_new_zone(outline, layer, net, items, filled_at_revision))
                    .collect();
                match layer {
                    LayerId::FCu => state.front_plane_zones = new_zones,
                    LayerId::BCu => state.back_plane_zones = new_zones,
                }
            },
        );
    }

    /// The "Refill zones" button's entire implementation -- see
    /// [`Self::start_zone_job`] and [`BoardDoc::clear_zone_fill`]/
    /// [`BoardDoc::insert_zone_refill`]'s own doc comments for why this
    /// mirrors [`BoardDoc::refill_all_zones`] (clear each zone's old
    /// fill synchronously, recompute every one in the background, then
    /// insert each result once it's ready) instead of just calling that
    /// method directly on a background thread -- `self.doc` itself
    /// never leaves the UI thread.
    fn refill_all_zones_in_background(&mut self) {
        let zones: Vec<(ZoneId, Polygon, LayerId, NetId)> = self.doc.zones.iter().map(|z| (z.id, z.outline.clone(), z.layer, z.net)).collect();
        for (id, ..) in &zones {
            self.doc.clear_zone_fill(*id);
        }
        let node = self.doc.node.clone();
        let board_outline = self.doc.outline.clone();
        let resolver = self.doc.resolver();
        let filled_at_revision = node.obstacle_revision();
        self.start_zone_job(
            "zone refill",
            "Refilling zones...",
            move || {
                zones
                    .into_iter()
                    .map(|(id, outline, layer, net)| {
                        let items = zone_fill::fill_zone(&outline, layer, net, &board_outline, &node, resolver);
                        (id, items)
                    })
                    .collect::<Vec<_>>()
            },
            move |state, results| {
                for (id, items) in results {
                    state.doc.insert_zone_refill(id, items, filled_at_revision);
                }
            },
        );
    }

    /// Drains [`Self::export_job`]'s result, if it's ready -- same
    /// once-per-frame shape as [`Self::poll_zone_job`].
    fn poll_export_job(&mut self) {
        let Some(job) = &mut self.export_job else { return };
        match job.poll() {
            JobPoll::Pending => {}
            JobPoll::Ready(result) => {
                self.export_job = None;
                self.io_message = Some(match result {
                    Ok(files) => format!(
                        "Exported {} + {} + {}",
                        files.gerber_zip.file_name().unwrap_or_default().to_string_lossy(),
                        files.position_csv.file_name().unwrap_or_default().to_string_lossy(),
                        files.bom_csv.file_name().unwrap_or_default().to_string_lossy()
                    ),
                    Err(e) => format!("Couldn't export manufacturing files: {e}"),
                });
            }
            JobPoll::Lost => {
                self.export_job = None;
                self.io_message = Some("The manufacturing-export background thread ended unexpectedly -- please try again.".to_string());
            }
        }
    }

    /// The GUI "Export manufacturing files..." button's entire
    /// implementation -- same native Gerber path as
    /// [`export_manufacturing_files_write`] (the MCP handler), driven by
    /// a folder-pick dialog and run on a
    /// `crate::background::BackgroundJob` so large boards don't freeze
    /// the UI. Refuses (via `self.io_message`) if an export is already
    /// running. Never touches `self.doc` on either thread.
    fn export_manufacturing_files_in_background(&mut self, out_dir: PathBuf, parts_db: &PartsDb) {
        if self.export_job.is_some() {
            self.io_message = Some("An export is already running -- wait for it to finish first.".to_string());
            return;
        }
        let doc = self.doc.clone();
        let templates = self.templates.clone();
        let file_path = self.file_path.clone();
        let bom_csv = crate::bom::to_csv(&crate::bom::build_bom_rows(&doc, &templates, &self.template_origin, parts_db));
        self.export_job = Some(BackgroundJob::spawn(move || export_manufacturing_files_to_dir(&doc, &templates, &file_path, &out_dir, &bom_csv)));
        self.io_message = Some("Exporting manufacturing files...".to_string());
    }

    /// Opens the "Autoroute (extern)" net-selection dialog, defaulting
    /// every net with more than one pad to checked -- the same
    /// "more than one pad" test [`draw_ratsnest`] already uses to
    /// decide whether a net has anything left it could possibly need a
    /// track for; a single-pad net (e.g. a mounting hole's own net)
    /// never does. This editor has no real board-wide connectivity
    /// pass to tell "already fully hand-routed" apart from "still
    /// needs a track" any more precisely than that (see
    /// `crate::ratsnest`'s own doc comment on why even the ratsnest
    /// display itself doesn't try) -- the external tool's own
    /// `check_connected.py`, surfaced after a run via
    /// `AutorouteReport::routed_nets`/`connected_ok`, is the real
    /// source of truth for that, not a guess made here before the user
    /// even clicks Route.
    fn open_autoroute_dialog(&mut self) {
        self.autoroute_selected_nets = self.doc.nets.iter().filter(|n| self.doc.pads_on_net(n.id).len() > 1).map(|n| n.id).collect();
        self.show_autoroute_dialog = true;
    }

    /// Starts a background external-autoroute run for `net_names` --
    /// refuses (via `io_message`) if one is already running rather
    /// than starting a second, since [`Self::autoroute_job`] only ever
    /// tracks one at a time.
    fn start_autoroute(&mut self, net_names: Vec<String>) {
        if matches!(&self.autoroute_job, Some(job) if job.result.is_none()) {
            self.io_message = Some("An autoroute job is already running -- wait for it to finish or cancel it first.".to_string());
            return;
        }
        match crate::external_router::run_autoroute(&self.doc, &self.templates, net_names, self.external_router_settings.clone()) {
            Ok(handle) => self.autoroute_job = Some(AutorouteJob { handle, log: Vec::new(), result: None }),
            Err(e) => self.io_message = Some(format!("Couldn't start the external autoroute: {e}")),
        }
    }

    /// Drains every event currently waiting on the running job's
    /// channel -- called once per frame (see [`PcbApp::ui`]), the same
    /// `try_recv()` idea `lcsc_fetch` already uses, just looped since a
    /// live subprocess log can produce far more lines per frame than a
    /// one-shot LCSC download ever would.
    fn poll_autoroute_job(&mut self) {
        let Some(job) = &mut self.autoroute_job else { return };
        loop {
            match job.handle.events.try_recv() {
                Ok(crate::external_router::AutorouteEvent::Log(line)) => job.log.push(line),
                Ok(crate::external_router::AutorouteEvent::Done(result)) => job.result = Some(result.map_err(|e| e.to_string())),
                Err(std::sync::mpsc::TryRecvError::Empty) => break,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    if job.result.is_none() {
                        job.result = Some(Err("the autoroute background thread ended unexpectedly".to_string()));
                    }
                    break;
                }
            }
        }
    }
}

pub struct PcbApp {
    screen: Screen,
    /// The user's parts library -- opened once for the process lifetime
    /// (unlike `screen`, it must survive "New board"/"Open" entirely,
    /// since it's independent of any one board file). See
    /// `crate::parts_db` for why this exists.
    parts_db: PartsDb,
    /// Pending requests from the embedded MCP server (see `crate::mcp`'s
    /// module doc comment), drained once per frame at the top of `fn ui`
    /// -- see [`handle_mcp_query`]. The sending half lives on
    /// `crate::mcp`'s own background thread, cloned into a fresh
    /// handler per HTTP request.
    mcp_rx: mpsc::Receiver<crate::mcp::McpQuery>,
    /// Whether this process was launched with `--allow-ai-write` (see
    /// `main.rs`) -- gates `crate::mcp`'s *write* tools (place a part,
    /// route, save, ...); the read-only ones are always available
    /// regardless. Shown as a small status label in the top panel (see
    /// [`Self::ui`]) so it's never a silent, invisible state. Fixed for
    /// the process's whole lifetime -- there's no in-GUI toggle on
    /// purpose, matching "an option at startup" rather than something
    /// that could be flipped on mid-session without a restart.
    allow_ai_write: bool,
    /// The one MCP-triggered background job [`Self::ui`] is tracking
    /// right now, if any -- zone-fill/refill, a routing search,
    /// net-continuity, manufacturing export, or a whole batch,
    /// whichever [`try_start_background_job`] most recently started
    /// (GUI-triggered zone/export jobs are tracked separately, see
    /// [`EditorState::zone_job`]/[`EditorState::export_job`]). Only one
    /// at a time on purpose: every *write* [`crate::mcp::McpQuery`] that
    /// arrives while this is `Some` is refused immediately with a busy
    /// message (see [`Self::ui`]'s own dispatch loop) rather than
    /// started as a second, concurrent one -- the only way to guarantee
    /// nothing else ever mutates `self.screen` between when this job's
    /// own snapshot was taken and when its result gets merged back into
    /// the live one. Read-only queries are entirely unaffected and
    /// always answer immediately regardless.
    pending_job: Option<PendingJob>,
    /// A board file being read from disk (and, for any of its own
    /// saved zones, re-filled -- see [`PendingBoardLoad::start`]'s doc
    /// comment) off the UI thread, if one is in flight: the launch-time
    /// "jump straight back into the last board" auto-open, or the
    /// user's own "Open board" menu action. `None` the rest of the
    /// time, including the entire "New board"/CLI/no-remembered-board
    /// path, which never needs this at all.
    board_load: Option<PendingBoardLoad>,
}

/// [`PcbApp::board_load`]: a board load in flight, and everything
/// needed to turn its result into a fresh [`Screen::Editor`] once it
/// resolves.
struct PendingBoardLoad {
    path: PathBuf,
    templates: Vec<FootprintTemplate>,
    template_origin: Vec<Option<i64>>,
    template_hover: Vec<Option<String>>,
    template_category: Vec<Option<String>>,
    job: BackgroundJob<Result<BoardDoc, String>>,
}

impl PendingBoardLoad {
    /// Starts reading and reconstructing `path` on a
    /// [`crate::background::BackgroundJob`] -- this project's
    /// "Background heavy computations" plan (development log) never
    /// actually named board *loading* as one of its hotspots, but
    /// [`load_from_path`] hits the exact same one: `from_json` re-runs
    /// `zone_fill::fill_zone` for every one of the board's own saved
    /// zones (see that function's doc comment on why the fill result
    /// itself is never persisted, only the outline), which is the same
    /// multi-second-per-fill cost as every other hotspot that plan
    /// covers -- except this one used to run synchronously inside
    /// [`PcbApp::new`] itself, *before* the very first frame is ever
    /// painted (launching straight back into the last board is the
    /// default -- see [`last_board_path`]), which is what made a real
    /// board with real zones look like the whole application had
    /// frozen at launch, not merely one background job being slow.
    /// `templates`/`template_origin`/`template_hover`/
    /// `template_category` are loaded synchronously up front (a plain
    /// `crate::parts_db` read, not the expensive part) so they're
    /// ready to hand `EditorState::new` the moment the load resolves.
    fn start(path: PathBuf, parts_db: &PartsDb) -> Self {
        let (templates, template_origin, template_hover, template_category) = load_templates(parts_db);
        let job_path = path.clone();
        let job_templates = templates.clone();
        let job = BackgroundJob::spawn(move || load_from_path(&job_path, &job_templates));
        Self { path, templates, template_origin, template_hover, template_category, job }
    }
}

/// What a [`crate::background::BackgroundJob`] started by
/// [`try_start_background_job`] hands back once it resolves: a closure
/// that applies whatever (if anything) needs to land on the *live*
/// [`Screen`] and returns the JSON string to reply with. A closure,
/// not just a plain `serde_json::Value`, because a couple of these
/// (zone fill, routing) still need to run their own cheap "insert the
/// already-computed result" step against whatever the live board looks
/// like *right now*, not the (possibly stale) clone the actual slow
/// computation ran against -- see this project's "Background heavy
/// computations" plan (development log) for the full reasoning.
type JobResult = Box<dyn FnOnce(&mut Screen) -> String + Send>;

/// One MCP-triggered background job in flight -- see
/// [`PcbApp::pending_job`]'s own doc comment for why [`PcbApp`] only
/// ever tracks one.
struct PendingJob {
    /// Shown by [`PcbApp::ui`]'s busy status line, and echoed back in
    /// [`busy_json`]'s refusal message for any write query that arrives
    /// while this is running.
    label: &'static str,
    job: BackgroundJob<JobResult>,
    reply: Option<oneshot::Sender<String>>,
}

impl PendingJob {
    /// A job that resolves on the very next poll without ever spawning
    /// a thread -- for a `spawn_*_job` function that discovers up front
    /// (cheap validation: no board open, an unknown net/pin name, a bad
    /// layer string, ...) that there's nothing to actually background,
    /// but still wants every caller to go through the exact same
    /// poll-then-reply path [`PcbApp::ui`] already has for a real job,
    /// rather than a separate synchronous-reply special case.
    fn immediate(label: &'static str, text: String, reply: oneshot::Sender<String>) -> Self {
        PendingJob { label, job: BackgroundJob::ready(Box::new(move |_: &mut Screen| text)), reply: Some(reply) }
    }
}

/// The JSON [`PcbApp::ui`]'s dispatch loop answers a *write*
/// [`crate::mcp::McpQuery`] with immediately, instead of queueing or
/// running it, while [`PcbApp::pending_job`] (`running`) is already
/// busy -- the caller's own retry (every MCP write tool's description
/// already tells an AI client to expect and handle this) picks up right
/// where the finished job leaves the board.
fn busy_json(running: &str) -> String {
    error_json(format!("a background job is already running ({running}) -- wait for it to finish, then retry")).to_string()
}

/// Opens the user's persistent parts library at its default per-user
/// path, falling back to a throwaway in-memory one (with a warning, not
/// a hard failure) if that's not possible -- shared between the GUI
/// (`PcbApp::default`) and `crate::cli`, so both ever open the database
/// exactly the same way.
pub(crate) fn open_parts_db() -> PartsDb {
    PartsDb::open_default().unwrap_or_else(|e| {
        eprintln!("warning: couldn't open the parts database ({e}); using a temporary in-memory one for this session");
        PartsDb::open_in_memory().expect("an in-memory sqlite database must always succeed")
    })
}

impl PcbApp {
    /// Jumps straight back into whichever board was last opened/saved
    /// (see [`last_board_path`]/[`remember_last_board`]) instead of
    /// always presenting [`Screen::NewBoard`] on every launch -- the
    /// "New board" form is still one click away via the top panel's
    /// "New board..." button, but a user who's already working on a
    /// board shouldn't have to re-navigate a form just to get back to
    /// it. Falls back to [`Screen::NewBoard`] exactly like before on a
    /// genuine first run, or if the remembered board can no longer be
    /// opened (moved/deleted/corrupted) -- never a hard failure, same
    /// "degrade gracefully" convention as [`load_templates`]'s own
    /// database-read fallback. `allow_ai_write` comes straight from
    /// `main.rs`'s `--allow-ai-write` flag -- see [`Self::allow_ai_write`].
    pub(crate) fn new(allow_ai_write: bool) -> Self {
        let parts_db = open_parts_db();
        // Starts loading in the background (see [`PendingBoardLoad::start`]'s
        // doc comment for why this can no longer just be a synchronous
        // `load_from_path` call right here) and opens on a plain "New
        // board" screen in the meantime -- [`Self::ui`]'s own poll of
        // `board_load` swaps in the real [`Screen::Editor`] the moment
        // it resolves, or leaves this screen as the fallback if there
        // was nothing to remember or the load fails, exactly like the
        // old synchronous version did.
        let board_load = last_board_path().map(|path| PendingBoardLoad::start(path, &parts_db));
        let screen = Screen::NewBoard(NewBoardParams::default());
        let (mcp_tx, mcp_rx) = mpsc::channel();
        crate::mcp::spawn_server(mcp_tx, crate::mcp::PORT, allow_ai_write);
        Self { screen, parts_db, mcp_rx, allow_ai_write, pending_job: None, board_load }
    }
}

impl Default for PcbApp {
    /// Equivalent to launching without `--allow-ai-write` -- used by
    /// tests and any caller that doesn't care about the MCP write gate.
    fn default() -> Self {
        Self::new(false)
    }
}

/// Where [`remember_last_board`] persists the most recently opened/
/// saved board's own file path, across process restarts -- same
/// "small file under the OS data dir" convention as
/// `PartsDb::default_path`, just a plain-text pointer file rather than
/// a database (there's only ever one value to remember).
fn last_board_pointer_path() -> PathBuf {
    let base = dirs::data_dir().unwrap_or_else(std::env::temp_dir);
    base.join("alladin-pcb").join("last_board.txt")
}

/// Reads back the path [`remember_last_board`] last wrote, if any --
/// `None` on a genuine first run or if the pointer file itself is
/// missing/unreadable/empty.
fn last_board_path() -> Option<PathBuf> {
    let text = std::fs::read_to_string(last_board_pointer_path()).ok()?;
    parse_last_board_pointer(&text)
}

/// The pure "what does this pointer file's content mean" half of
/// [`last_board_path`], split out so it's testable without touching the
/// real OS data dir. `None` for empty/all-whitespace content, so an
/// accidentally-truncated-to-empty pointer file degrades to "no
/// remembered board" rather than a bogus empty path.
fn parse_last_board_pointer(text: &str) -> Option<PathBuf> {
    let trimmed = text.trim();
    (!trimmed.is_empty()).then(|| PathBuf::from(trimmed))
}

/// Records `path` as the board [`PcbApp::default`] should jump straight
/// back into on the next launch. Called only once a board is actually
/// opened from or saved to a real file (never for a brand new,
/// still-unsaved board, which has no path yet to remember) -- see the
/// `open_requested`/`save_requested`/`save_as_requested` handlers.
/// Best-effort: a failure here (e.g. a read-only data dir) must never
/// block the save/open itself from succeeding, so errors are swallowed.
fn remember_last_board(path: &Path) {
    let pointer = last_board_pointer_path();
    if let Some(parent) = pointer.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let _ = std::fs::write(pointer, path.to_string_lossy().as_bytes());
}

pub(crate) fn save_to_path(doc: &BoardDoc, path: &std::path::Path) -> Result<(), String> {
    write_atomic(path, crate::persistence::to_json(doc).as_bytes())
}

/// Writes `contents` to `path` without a concurrent reader ever being
/// able to observe a half-written file: write to a sibling temp file
/// first, then `rename` it into place. This matters now that something
/// other than "the user clicks Save" can read this same path while a
/// write is in flight -- an AI/script driving the board headlessly
/// (see `crate::cli`) alongside a GUI that's polling the very same file
/// for external changes (see [`EditorState::reload_from_disk`]) -- a
/// plain `std::fs::write` lets a poll land mid-write and hand
/// `persistence::from_json` a truncated document. `rename` onto an
/// existing path is the standard way to make a multi-step write look
/// atomic to every other reader: they see either the complete old file
/// or the complete new one, never a partial one.
fn write_atomic(path: &std::path::Path, contents: &[u8]) -> Result<(), String> {
    let mut tmp_name = path.file_name().unwrap_or_default().to_os_string();
    tmp_name.push(".tmp");
    let tmp_path = path.with_file_name(tmp_name);
    std::fs::write(&tmp_path, contents).map_err(|e| e.to_string())?;
    std::fs::rename(&tmp_path, path).map_err(|e| e.to_string())
}

pub(crate) fn load_from_path(path: &std::path::Path, templates: &[FootprintTemplate]) -> Result<BoardDoc, String> {
    let json = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    crate::persistence::from_json(&json, templates).map_err(|e| e.to_string())
}

/// `path`'s current modification time, or `None` if it can't be
/// stat()ed at all (doesn't exist, permissions, ...) -- the one piece
/// [`EditorState::maybe_reload_from_disk`]'s live-watch polling needs,
/// factored out so both that call site and [`EditorState::set_file_path`]
/// read it the exact same way.
fn file_mtime(path: &std::path::Path) -> Option<std::time::SystemTime> {
    std::fs::metadata(path).ok()?.modified().ok()
}

fn board_file_dialog() -> rfd::FileDialog {
    rfd::FileDialog::new().add_filter("Aladin PCB board", &["json"]).set_file_name("board.json")
}

/// GUI counterpart of [`crate::app::export_manufacturing_files_write`]
/// (the MCP handler). Writes Gerber zip + CPL + BOM natively; see
/// [`crate::native_gerber`].
fn export_manufacturing_files_to_dir(
    doc: &BoardDoc,
    templates: &[FootprintTemplate],
    file_path: &Option<std::path::PathBuf>,
    out_dir: &std::path::Path,
    bom_csv_contents: &str,
) -> Result<crate::native_gerber::ManufacturingFiles, String> {
    let stem = file_path.as_ref().and_then(|p| p.file_stem()).and_then(|s| s.to_str()).unwrap_or("board");
    crate::native_gerber::export_manufacturing_files_native(doc, templates, stem, out_dir, bom_csv_contents).map_err(|e| e.to_string())
}

/// Writes a `*.before-autoroute.json` snapshot of `doc` exactly as it
/// stood right before merging in a completed
/// `crate::external_router::run_autoroute` run's tracks/vias, then adds
/// every item in `items` straight into `doc.node` -- see
/// `crate::external_router`'s own doc comment and the architecture
/// plan's "kein Undo-Stack" note for why a plain backup *file* is the
/// safety net here: this editor has no in-memory undo/redo stack
/// anywhere else either, so a bad or unwanted autoroute result needs
/// its own explicit way back, exactly like every other "point of no
/// return" action already gets one from a save-first habit. Named
/// after `file_path`'s own stem when the board has one (`board.json`
/// -> `board.before-autoroute.json`, right next to it); an
/// as-yet-unsaved board gets a same-shaped name in the OS temp
/// directory instead, since there's no board file path to sit next to
/// yet. A write failure here is reported back to the caller rather
/// than silently skipped -- merging without a successful backup first
/// would defeat the entire point.
pub(crate) fn merge_autoroute_items(doc: &mut BoardDoc, file_path: &Option<PathBuf>, items: Vec<Item>) -> Result<(), String> {
    let backup_path = match file_path {
        Some(p) => p.with_extension("before-autoroute.json"),
        None => std::env::temp_dir().join(format!("alladin_pcb_unsaved_board_{}.before-autoroute.json", std::process::id())),
    };
    std::fs::write(&backup_path, crate::persistence::to_json(doc)).map_err(|e| format!("couldn't write the safety backup {} before merging: {e}", backup_path.display()))?;
    for item in items {
        doc.node.add(item);
    }
    Ok(())
}

/// One green-check/red-cross checklist row in
/// [`draw_external_router_settings_window`]'s "Diagnose" results.
fn draw_diagnose_row(ui: &mut egui::Ui, ok: bool, label: &str) {
    let (color, mark) = if ok { (Color32::from_rgb(120, 200, 120), "\u{2713}") } else { (Color32::from_rgb(230, 90, 90), "\u{2717}") };
    ui.horizontal(|ui| {
        ui.colored_label(color, mark);
        ui.label(label);
    });
}

/// The exact, plain-text one-time setup a fresh machine needs before
/// [`crate::external_router::diagnose`] can ever report ready --
/// intentionally just text a user runs themselves (see
/// `crate::external_router`'s own doc comment for why Alladin never
/// runs any of this on its own).
fn external_router_setup_instructions() -> String {
    "git clone https://github.com/drandyhaas/KiCadRoutingTools.git\ncd KiCadRoutingTools\npip3 install numpy scipy shapely\npython3 build_router.py".to_string()
}

/// The "Autoroute (extern) settings" window: the tool folder/Python
/// binary/routing-parameter fields [`crate::external_router::ExternalRouterSettings`]
/// itself carries, a "Diagnose" checklist, and (only while not fully
/// ready) the plain-text one-time setup instructions with a "Copy"
/// button -- opened by the toolbar's gear-icon button, closed either
/// by its own titlebar close button or that same click again.
fn draw_external_router_settings_window(ctx: &egui::Context, state: &mut EditorState) {
    if !state.show_external_router_settings {
        return;
    }
    let mut open = true;
    egui::Window::new("Autoroute (extern) settings").open(&mut open).resizable(true).show(ctx, |ui| {
        egui::Grid::new("external_router_settings_grid").num_columns(2).show(ui, |ui| {
            ui.label("Tool folder");
            ui.horizontal(|ui| {
                ui.add(egui::TextEdit::singleline(&mut state.external_router_settings.tool_dir).desired_width(240.0)).on_hover_text("The cloned KiCadRoutingTools checkout -- must contain route.py directly.");
                if ui.button("Browse\u{2026}").clicked() {
                    if let Some(dir) = rfd::FileDialog::new().pick_folder() {
                        state.external_router_settings.tool_dir = dir.to_string_lossy().into_owned();
                    }
                }
            });
            ui.end_row();

            ui.label("Python");
            ui.text_edit_singleline(&mut state.external_router_settings.python_bin);
            ui.end_row();

            ui.label("Track width (mm)");
            ui.add(egui::DragValue::new(&mut state.external_router_settings.track_width_mm).range(JlcpcbDfm::MIN_TRACK_WIDTH as f64 / MM as f64..=5.0).speed(0.01));
            ui.end_row();

            ui.label("Via diameter (mm)");
            ui.add(egui::DragValue::new(&mut state.external_router_settings.via_diameter_mm).range(JlcpcbDfm::MIN_VIA_DIAMETER as f64 / MM as f64..=10.0).speed(0.01));
            ui.end_row();

            ui.label("Via drill (mm)");
            ui.add(egui::DragValue::new(&mut state.external_router_settings.via_drill_mm).range(JlcpcbDfm::MIN_VIA_HOLE as f64 / MM as f64..=5.0).speed(0.01));
            ui.end_row();

            // Not editable: unlike the three fields above, clearance has
            // no locally-stored value to drag at all any more -- see
            // `ExternalRouterSettings`'s own doc comment for why a
            // free-form clearance slider was a real JLCPCB-compliance
            // bug, not just a redundant control. `route.py` always
            // reads this exact number itself, straight off the fresh
            // `.kicad_pro` written right before every autoroute run.
            ui.label("Clearance (mm)");
            ui.label(format!("{:.2} (JLCPCB {} minimum, not editable)", state.doc.net_class_clearance() as f64 / MM as f64, state.doc.copper_weight));
            ui.end_row();

            ui.label("Extra arguments");
            ui.add(egui::TextEdit::singleline(&mut state.external_router_settings.extra_args).desired_width(240.0))
                .on_hover_text("Passed straight through to route.py's own argv, e.g. --bus --ordering mps -- see the tool's own --help for what it currently supports.");
            ui.end_row();
        });

        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if ui.button("Save").clicked() {
                match state.external_router_settings.save() {
                    Ok(()) => state.io_message = None,
                    Err(e) => state.io_message = Some(format!("Couldn't save Autoroute (extern) settings: {e}")),
                }
            }
            if ui.button("Diagnose").clicked() {
                state.external_router_diagnose = Some(crate::external_router::diagnose(&state.external_router_settings));
            }
            if ui
                .button("Reset")
                .on_hover_text("Resets track width/via diameter/via drill back to Alladin's own JLCPCB-safe defaults -- tool folder, Python interpreter, and extra arguments are left untouched.")
                .clicked()
            {
                let defaults = crate::external_router::ExternalRouterSettings::default();
                state.external_router_settings.track_width_mm = defaults.track_width_mm;
                state.external_router_settings.via_diameter_mm = defaults.via_diameter_mm;
                state.external_router_settings.via_drill_mm = defaults.via_drill_mm;
            }
        });

        ui.add_space(6.0);
        ui.separator();
        match &state.external_router_diagnose {
            None => {
                ui.weak("Click Diagnose to check Python/route.py/numpy/scipy/shapely on this machine.");
            }
            Some(report) => {
                draw_diagnose_row(ui, report.python_found, &format!("Python found{}", report.python_version.as_ref().map(|v| format!(" ({v})")).unwrap_or_default()));
                draw_diagnose_row(ui, report.script_found, "route.py found in the tool folder");
                draw_diagnose_row(ui, report.numpy_ok, "numpy importable");
                draw_diagnose_row(ui, report.scipy_ok, "scipy importable");
                draw_diagnose_row(ui, report.shapely_ok, "shapely importable");
                draw_diagnose_row(ui, report.help_ok, "route.py --help runs");

                if !report.is_ready() {
                    ui.add_space(6.0);
                    ui.label("Not fully set up yet -- one-time setup on this machine:");
                    let mut instructions = external_router_setup_instructions();
                    ui.add(egui::TextEdit::multiline(&mut instructions).desired_rows(4).font(egui::TextStyle::Monospace).interactive(false));
                    if ui.button("Copy setup instructions").clicked() {
                        ui.ctx().copy_text(instructions);
                    }
                }
            }
        }
    });
    if !open {
        state.show_external_router_settings = false;
    }
}

/// A modal "are you sure?" prompt for whatever "Place part" panel
/// delete button was last clicked ([`EditorState::pending_delete`]) --
/// without this, either delete button acted the instant it was
/// clicked, so one accidental click (especially on a category
/// header's, which can remove anywhere from one to several hundred
/// parts at once) was permanent with no way back. Only this function
/// ever actually calls [`crate::parts_db::PartsDb::delete_part`]/
/// [`crate::parts_db::PartsDb::delete_category_tree`] -- the panel's
/// own delete buttons (see `crate::app`'s `delete_part_requested`/
/// `delete_category_requested`) only ever stage a [`PendingDelete`]
/// here, never delete anything themselves. Draws nothing at all when
/// nothing is pending.
fn draw_delete_confirmation_window(ctx: &egui::Context, state: &mut EditorState, parts_db: &PartsDb) {
    let Some(pending) = &state.pending_delete else {
        return;
    };
    let (title, body) = match pending {
        PendingDelete::Part { name, .. } => ("Delete part?".to_string(), format!("Remove \"{name}\" from your parts database?\n\nThis cannot be undone.")),
        PendingDelete::Category { prefix, count } => {
            ("Delete category?".to_string(), format!("Delete all {count} part(s) under \"{prefix}\"?\n\nThis cannot be undone."))
        }
    };
    let mut confirmed = false;
    let mut cancelled = false;
    egui::Window::new(title).collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO).show(ctx, |ui| {
        ui.label(body);
        ui.add_space(8.0);
        ui.horizontal(|ui| {
            if ui.button("Cancel").clicked() {
                cancelled = true;
            }
            if ui.add(egui::Button::new("Yes, delete").fill(Color32::from_rgb(160, 50, 50))).clicked() {
                confirmed = true;
            }
        });
    });

    if confirmed {
        if let Some(pending) = state.pending_delete.take() {
            apply_confirmed_delete(state, parts_db, pending);
        }
    } else if cancelled {
        state.pending_delete = None;
    }
}

/// The two actions a confirmed [`PendingDelete`] actually performs --
/// split out from [`draw_delete_confirmation_window`] itself so this
/// part (the only part that ever calls
/// [`crate::parts_db::PartsDb::delete_part`]/`delete_category_tree`)
/// is plain Rust a test can call directly, with no `egui::Context` (a
/// real one only exists inside a running `eframe` frame) required at
/// all.
fn apply_confirmed_delete(state: &mut EditorState, parts_db: &PartsDb, pending: PendingDelete) {
    match pending {
        PendingDelete::Part { index, db_id, .. } => match parts_db.delete_part(db_id) {
            Ok(()) => {
                state.templates.remove(index);
                state.template_origin.remove(index);
                state.template_hover.remove(index);
                state.template_category.remove(index);
                // Any `Tool::Place(j)` index past `index` is now off
                // by one; simplest safe fix is to just drop out of
                // placement mode entirely.
                state.tool = Tool::Select;
            }
            Err(e) => state.io_message = Some(format!("Couldn't delete part: {e}")),
        },
        PendingDelete::Category { prefix, .. } => match parts_db.delete_category_tree(&prefix) {
            Ok(deleted) => {
                // Simplest safe way to keep `templates`/
                // `template_origin`/`template_hover`/
                // `template_category` in lock-step after an
                // arbitrary-sized bulk delete: just reload the whole
                // (now-shorter) database-backed list from scratch,
                // exactly like opening a board does -- same reasoning
                // as dropping out of placement mode after a
                // single-part delete above, just for a whole batch.
                let (templates, template_origin, template_hover, template_category) = load_templates(parts_db);
                state.templates = templates;
                state.template_origin = template_origin;
                state.template_hover = template_hover;
                state.template_category = template_category;
                state.tool = Tool::Select;
                state.io_message = Some(format!("Deleted {deleted} part(s) from \"{prefix}\"."));
            }
            Err(e) => state.io_message = Some(format!("Couldn't delete category \"{prefix}\": {e}")),
        },
    }
}

/// The "Autoroute (extern)" dialog: pick which nets to route, start/
/// cancel the background job, watch its live log, then -- only once it
/// finishes -- review the report and explicitly choose to merge its
/// tracks/vias into the live board or discard them. Opened by
/// [`EditorState::open_autoroute_dialog`]; closing the window (its
/// titlebar button) while a job is merely running leaves that job
/// running in the background (see `state.autoroute_job`, independent
/// of `show_autoroute_dialog`) rather than cancelling it, so switching
/// tools/panels to do something else meanwhile doesn't lose progress.
fn draw_autoroute_dialog_window(ctx: &egui::Context, state: &mut EditorState) {
    if !state.show_autoroute_dialog {
        return;
    }
    let mut open = true;
    let mut start_requested: Option<Vec<String>> = None;
    let mut cancel_requested = false;
    let mut merge_requested = false;
    let mut discard_requested = false;
    let mut close_requested = false;

    egui::Window::new("Autoroute (extern)").open(&mut open).resizable(true).default_width(420.0).show(ctx, |ui| {
        let busy = matches!(&state.autoroute_job, Some(job) if job.result.is_none());
        let finished = matches!(&state.autoroute_job, Some(job) if job.result.is_some());

        if !busy && !finished {
            ui.label("Nets to route (unchecked nets are left exactly as they are):");
            egui::ScrollArea::vertical().max_height(200.0).show(ui, |ui| {
                for net in &state.doc.nets {
                    let mut checked = state.autoroute_selected_nets.contains(&net.id);
                    if ui.checkbox(&mut checked, &net.name).changed() {
                        if checked {
                            state.autoroute_selected_nets.insert(net.id);
                        } else {
                            state.autoroute_selected_nets.remove(&net.id);
                        }
                    }
                }
            });
            ui.add_space(6.0);
            let selected_count = state.autoroute_selected_nets.len();
            ui.horizontal(|ui| {
                if ui.add_enabled(selected_count > 0, egui::Button::new(format!("Route {selected_count} net(s)"))).clicked() {
                    let names: Vec<String> = state.doc.nets.iter().filter(|n| state.autoroute_selected_nets.contains(&n.id)).map(|n| n.name.clone()).collect();
                    start_requested = Some(names);
                }
                if ui.button("Cancel").clicked() {
                    close_requested = true;
                }
            });
        }

        if let Some(job) = &state.autoroute_job {
            ui.add_space(8.0);
            ui.separator();
            ui.label(if busy { "Running\u{2026}" } else { "Finished." });
            if busy {
                ui.spinner();
                ui.ctx().request_repaint();
            }
            egui::ScrollArea::vertical().id_salt("autoroute_log_scroll").max_height(180.0).stick_to_bottom(true).show(ui, |ui| {
                for line in &job.log {
                    ui.monospace(line);
                }
            });
            if busy && ui.button("Cancel run").clicked() {
                cancel_requested = true;
            }

            if let Some(result) = &job.result {
                ui.add_space(6.0);
                match result {
                    Ok(report) => {
                        ui.colored_label(
                            Color32::from_rgb(120, 200, 120),
                            format!("{}/{} requested net(s) routed.", report.routed_nets.len(), report.requested_nets.len()),
                        );
                        let check_line = |ui: &mut egui::Ui, label: &str, ok: Option<bool>| match ok {
                            Some(true) => ui.colored_label(Color32::from_rgb(120, 200, 120), format!("{label}: passed")),
                            Some(false) => ui.colored_label(Color32::from_rgb(230, 90, 90), format!("{label}: FAILED -- see log above")),
                            None => ui.weak(format!("{label}: not available (script not found in tool folder)")),
                        };
                        check_line(ui, "DRC check", report.drc_ok);
                        check_line(ui, "Connectivity check", report.connected_ok);
                        ui.label(format!("{} track/via item(s) ready to merge.", report.items.len()));
                        ui.add_space(6.0);
                        ui.horizontal(|ui| {
                            if ui.add_enabled(!report.items.is_empty(), egui::Button::new("Merge into board")).clicked() {
                                merge_requested = true;
                            }
                            if ui.button("Discard").clicked() {
                                discard_requested = true;
                            }
                        });
                    }
                    Err(message) => {
                        ui.colored_label(Color32::from_rgb(230, 90, 90), message);
                        if ui.button("Close").clicked() {
                            discard_requested = true;
                        }
                    }
                }
            }
        }
    });

    if let Some(names) = start_requested {
        state.start_autoroute(names);
    }
    if cancel_requested {
        if let Some(job) = &state.autoroute_job {
            job.handle.cancel();
        }
    }
    if merge_requested {
        if let Some(job) = state.autoroute_job.take() {
            if let Some(Ok(report)) = job.result {
                let file_path = state.file_path.clone();
                match merge_autoroute_items(&mut state.doc, &file_path, report.items) {
                    Ok(()) => {
                        state.io_message = None;
                        state.show_autoroute_dialog = false;
                    }
                    Err(e) => state.io_message = Some(e),
                }
            }
        }
    }
    if discard_requested {
        state.autoroute_job = None;
    }
    if !open || close_requested {
        // Only actually closes the dialog once no job is running (see
        // this function's own doc comment) -- otherwise the window
        // stays put with its titlebar close button simply ignored, so
        // a live job's log/cancel button is never yanked out from
        // under the user by an accidental close click.
        let busy = matches!(&state.autoroute_job, Some(job) if job.result.is_none());
        if !busy {
            state.show_autoroute_dialog = false;
        }
    }
}

fn draw_ghost(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, items: &[Item], valid: bool) {
    let color = if valid {
        Color32::from_rgba_unmultiplied(80, 220, 120, 170)
    } else {
        Color32::from_rgba_unmultiplied(230, 90, 90, 170)
    };
    for item in items {
        match item {
            Item::Pad { shape, .. } => {
                // A circle sized to `bounding_radius()` (the AABB's own
                // half-diagonal for a non-round pad) rather than the pad's
                // true outline -- purely cosmetic, like the hover/selection
                // rings below: this is only a green/red placement-validity
                // preview, [`draw_pad_shape`] already draws every *placed*
                // pad's real shape once `check_placement` (which does use
                // the true outline, see `board_doc.rs`) accepts it.
                let center = camera.board_to_screen(rect, shape.center());
                let radius_px = (shape.bounding_radius() as f32 / MM as f32 * camera.pixels_per_mm).max(1.0);
                painter.circle_filled(center, radius_px, color);
                painter.circle_stroke(center, radius_px, Stroke::new(1.5, Color32::WHITE));
            }
            // A pure mounting-hole footprint (`world_items` only ever
            // produces `Item::Pad`/`Item::Hole`, see that function's own
            // doc comment) had no ghost at all here before -- placing or
            // dragging one showed literally nothing under the cursor
            // until the click/drop itself landed.
            Item::Hole { position, drill } => {
                let center = camera.board_to_screen(rect, *position);
                let radius_px = (*drill as f32 / 2.0 / MM as f32 * camera.pixels_per_mm).max(1.0);
                painter.circle_filled(center, radius_px, color);
                painter.circle_stroke(center, radius_px, Stroke::new(1.5, Color32::WHITE));
            }
            _ => {}
        }
    }
}

/// Where the exported Reference label's anchor sits in the
/// footprint's own local (unrotated) frame -- the exact mirror of
/// `alladin_kicad_io::writer`'s `footprint_vertical_extent`/`pad_reach`
/// + its 1mm margin (see `footprint_to_sexpr`'s `Reference` property):
/// one circumscribing reach above the topmost pad/hole. Mirrored here
/// (from `FootprintTemplate` rather than `WritePad`) so
/// [`draw_footprint_details`] can draw the designator at the very spot
/// and orientation the export will actually print it, instead of the
/// old fixed "2mm above the center, never rotated" guess that made
/// the preview and the Gerber disagree.
fn reference_label_local_y(template: &FootprintTemplate) -> Unit {
    let pad_reach = |p: &crate::footprint::PadTemplate| match p.shape {
        PadShapeKind::Circle => p.radius,
        PadShapeKind::Rect { width, height } | PadShapeKind::Oval { width, height } => {
            (((width as f64).powi(2) + (height as f64).powi(2)).sqrt() / 2.0).round() as Unit
        }
    };
    let min_y = template
        .pads
        .iter()
        .map(|p| p.offset.y - pad_reach(p))
        .chain(template.holes.iter().map(|h| h.offset.y - h.drill / 2))
        .min()
        .unwrap_or(-MM);
    min_y - MM
}

/// A [`crate::board_doc::SilkText`]'s real stroke width in on-screen
/// pixels -- scaled from its `line_width` the same way every other
/// board-space dimension here converts to pixels
/// (`camera.pixels_per_mm`), floored at a hairline so a label doesn't
/// vanish entirely at extreme zoom-out.
fn silk_stroke_width_px(text: &crate::board_doc::SilkText, camera: &Camera) -> f32 {
    (text.line_width as f32 / MM as f32 * camera.pixels_per_mm).max(1.0)
}

/// Draws `text` as its real, as-manufactured ink: the exact
/// embedded Hershey stroke segments DFM collision also checks
/// ([`crate::board_doc::SilkText::stroke_segments`]) and KiCad itself
/// re-renders after export -- the GUI preview, the legality check,
/// and the produced silkscreen are now literally the same geometry.
/// egui draws butt-capped lines, KiCad round-capped ones; small discs
/// on every segment end close that gap (and double as smooth joints
/// between a polyline's segments).
fn draw_silk_text_strokes(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, text: &crate::board_doc::SilkText, color: Color32) {
    let width_px = silk_stroke_width_px(text, camera);
    let stroke = Stroke::new(width_px, color);
    for segment in text.stroke_segments() {
        let a = camera.board_to_screen(rect, segment.a);
        let b = camera.board_to_screen(rect, segment.b);
        painter.line_segment([a, b], stroke);
        painter.circle_filled(a, width_px / 2.0, color);
        painter.circle_filled(b, width_px / 2.0, color);
    }
}

/// The screen-space corners of `text`'s ink bounding box
/// ([`crate::board_doc::SilkText::bounding_rect`] -- since the switch
/// to the embedded stroke font that *is* the tight box around the
/// real glyphs, so selection ring and ghost outline fit the visible
/// ink exactly, descenders included) -- mapped through the camera like
/// every other board-space shape, so it tracks pan/zoom/rotation for
/// free.
fn silk_text_outline_px(rect: egui::Rect, camera: &Camera, text: &crate::board_doc::SilkText) -> Vec<egui::Pos2> {
    text.bounding_rect().points.iter().map(|&p| camera.board_to_screen(rect, p)).collect()
}

/// Draws one already-placed [`crate::board_doc::SilkText`] -- just the
/// strokes themselves, no box: the yellow selection ring already
/// shows the outline on demand.
fn draw_silk_text(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, text: &crate::board_doc::SilkText) {
    draw_silk_text_strokes(painter, rect, camera, text, Color32::from_rgb(220, 220, 220));
}

/// [`draw_silk_text`]'s live, red/green placement-validity preview for
/// an in-progress [`Tool::PlaceSilkText`] session -- same green/red
/// convention [`draw_ghost`] already uses for a footprint/via ghost.
/// The glyph strokes themselves carry the validity color; the thin
/// outline around them is just a grab-frame affordance.
fn draw_silk_text_ghost(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, text: &crate::board_doc::SilkText, valid: bool) {
    let color = if valid {
        Color32::from_rgba_unmultiplied(80, 220, 120, 220)
    } else {
        Color32::from_rgba_unmultiplied(230, 90, 90, 220)
    };
    let points = silk_text_outline_px(rect, camera, text);
    painter.add(egui::Shape::closed_line(points, Stroke::new(1.0, color)));
    draw_silk_text_strokes(painter, rect, camera, text, color);
}

/// Draws one filled silkscreen circle (a placed
/// [`crate::board_doc::SilkDot`] or a footprint's pin-1 marker --
/// identical ink either way) in `color`, exactly the circle the export
/// prints (see `alladin_kicad_io::WriteSilkDot`): center + radius
/// through the camera, no cosmetic inflation.
fn draw_silk_dot_circle(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, circle: &alladin_geom::Circle, color: Color32) {
    let center = camera.board_to_screen(rect, circle.center);
    let radius_px = (circle.radius as f32 / MM as f32 * camera.pixels_per_mm).max(1.5);
    painter.circle_filled(center, radius_px, color);
}

/// [`draw_silk_dot_circle`]'s red/green placement-validity ghost for
/// [`Tool::PlaceSilkDot`] and an in-progress dot drag -- same color
/// convention as [`draw_silk_text_ghost`].
fn draw_silk_dot_ghost(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, circle: &alladin_geom::Circle, valid: bool) {
    let color = if valid {
        Color32::from_rgba_unmultiplied(80, 220, 120, 220)
    } else {
        Color32::from_rgba_unmultiplied(230, 90, 90, 220)
    };
    draw_silk_dot_circle(painter, rect, camera, circle, color);
}

/// A pad's ring size for the purely cosmetic hover/selection/pending-pin
/// indicators below: [`PadShape::bounding_radius`] for a non-round pad
/// (its AABB's half-diagonal, since these draw a circle regardless of
/// the pad's true shape) plus a fixed visual margin so the ring always
/// clears the pad's own edge -- never read for DRC/collision, only for
/// where to put a highlight ring on screen.
fn pad_ring_radius_px(shape: &PadShape, camera: &Camera) -> f32 {
    (shape.bounding_radius() as f32 / MM as f32 * camera.pixels_per_mm).max(1.0) + 4.0
}

fn draw_selection_ring(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, node: &Node, item_ids: &[ItemId]) {
    for &id in item_ids {
        match node.get(id) {
            Some(Item::Pad { shape, .. }) => {
                let center = camera.board_to_screen(rect, shape.center());
                let radius_px = pad_ring_radius_px(shape, camera);
                painter.circle_stroke(center, radius_px, Stroke::new(2.0, Color32::from_rgb(255, 220, 0)));
            }
            // A pure mounting-hole footprint (see `PlacedFootprint::hole_item_ids`'s
            // doc comment) has no pads for the arm above to ever match --
            // without this, selecting one (now possible at all since
            // `BoardDoc::footprint_at`'s own hole-hit-test fix) drew no
            // visible feedback whatsoever, even though the selection
            // itself, drag-to-move, and Delete all already worked.
            Some(Item::Hole { position, drill }) => {
                let center = camera.board_to_screen(rect, *position);
                let radius_px = (*drill as f32 / 2.0 / MM as f32 * camera.pixels_per_mm).max(1.0) + 3.0;
                painter.circle_stroke(center, radius_px, Stroke::new(2.0, Color32::from_rgb(255, 220, 0)));
            }
            _ => {}
        }
    }
}

/// The selection highlight for a bare [`EditorState::selected_item`]
/// (an `Item::Track` or `Item::Via`) -- [`draw_selection_ring`]'s
/// footprint-selection equivalent: a thick yellow outline drawn right
/// over every leg/via of [`BoardDoc::connected_wire`] (the *whole*
/// electrically-continuous wire the click landed on, not just that one
/// bent leg), so it's obvious at a glance exactly how much
/// Delete/Backspace is about to remove.
fn draw_item_selection_highlight(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, doc: &BoardDoc, id: ItemId) {
    let color = Color32::from_rgb(255, 220, 0);
    for wire_id in doc.connected_wire(id) {
        match doc.node.get(wire_id) {
            Some(Item::Track { shape, .. }) => {
                let a = camera.board_to_screen(rect, shape.a);
                let b = camera.board_to_screen(rect, shape.b);
                let width_px = (shape.width as f32 / MM as f32 * camera.pixels_per_mm).max(1.0) + 4.0;
                painter.line_segment([a, b], Stroke::new(width_px, color.gamma_multiply(0.5)));
            }
            Some(Item::Via { shape, .. }) => {
                let center = camera.board_to_screen(rect, shape.center);
                let radius_px = (shape.radius as f32 / MM as f32 * camera.pixels_per_mm).max(1.0) + 4.0;
                painter.circle_stroke(center, radius_px, Stroke::new(2.5, color));
            }
            _ => {}
        }
    }
}

/// A single pin, highlighted while it's the pending first half of a
/// [`Tool::Connect`] two-click gesture -- otherwise identical to
/// [`draw_selection_ring`] but visually distinct (cyan, not the
/// footprint-selection yellow) since the two mean different things.
fn draw_pending_pin(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, node: &Node, pad_id: ItemId) {
    if let Some(Item::Pad { shape, .. }) = node.get(pad_id) {
        let center = camera.board_to_screen(rect, shape.center());
        let radius_px = pad_ring_radius_px(shape, camera);
        painter.circle_stroke(center, radius_px, Stroke::new(2.5, Color32::from_rgb(80, 220, 255)));
    }
}

/// The live preview of an in-progress [`Tool::Route`] drag: the pin it
/// started from (cyan ring, same visual language as
/// [`draw_pending_pin`]), every corner already fixed (see
/// [`crate::routing::RoutingDrag::fixed_points`]) as a solid line in the
/// net's own colour -- these are settled, about to become real tracks
/// exactly as drawn -- and finally the live end: either the docked
/// walkaround/shove preview while hovering a same-net pin, or the
/// free-steered, snapped-angle leg(s) otherwise, drawn the same solid
/// colour while clear or dashed red while blocked so it's obvious at a
/// glance whether Space/click would actually do anything right now.
fn draw_routing_preview(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, doc: &BoardDoc, routing: &crate::routing::RoutingDrag) {
    draw_pending_pin(painter, rect, camera, &doc.node, routing.from_pad);
    if let Some(target) = routing.hover_target {
        draw_pending_pin(painter, rect, camera, &doc.node, target);
    }
    let net_color = alladin_render::net_color(Some(routing.net())).gamma_multiply(0.85);

    let fixed_path: Vec<Point> = std::iter::once(routing.origin()).chain(routing.fixed_points()).collect();
    for leg in fixed_path.windows(2) {
        let from = camera.board_to_screen(rect, leg[0]);
        let to = camera.board_to_screen(rect, leg[1]);
        painter.line_segment([from, to], Stroke::new(3.0, net_color));
    }

    let (live_legs, live_clear) = routing.live_end();
    if live_legs.is_empty() {
        return;
    }
    let live_color = if live_clear { net_color } else { Color32::from_rgb(230, 70, 70) };
    let mut last = camera.board_to_screen(rect, *fixed_path.last().unwrap());
    for &point in live_legs {
        let to = camera.board_to_screen(rect, point);
        painter.line_segment([last, to], Stroke::new(3.0, live_color));
        last = to;
    }
}

/// The live preview of an in-progress [`crate::routing::TraceDrag`]:
/// the replacement path drawn at the trace's own width, solid in the
/// net's colour while clear, dashed-look red (same "would collide"
/// language [`draw_routing_preview`] uses) while blocked -- the
/// original legs it would replace are hidden from the normal item
/// render for the duration (see [`crate::routing::TraceDrag::removed_ids`]),
/// so this is the only thing drawn for that wire until the drag ends.
fn draw_trace_drag_preview(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, drag: &crate::routing::TraceDrag) {
    let (path, clear) = drag.live();
    if path.len() < 2 {
        return;
    }
    let net_color = alladin_render::net_color(Some(drag.net())).gamma_multiply(0.85);
    let color = if clear { net_color } else { Color32::from_rgb(230, 70, 70) };
    for leg in path.windows(2) {
        let from = camera.board_to_screen(rect, leg[0]);
        let to = camera.board_to_screen(rect, leg[1]);
        painter.line_segment([from, to], Stroke::new(3.0, color));
    }
}

/// The live preview of an in-progress [`Tool::DrawZone`] outline: the
/// vertices placed so far, joined by solid lines, plus a dashed
/// "rubber band" segment from the last vertex to the current cursor
/// position so the user can see where the next click will land. The
/// first vertex is drawn as a small ring once at least a triangle's
/// worth of points exist, matching [`EditorState::handle_draw_zone_click`]'s
/// "click back here to close" gesture.
fn draw_zone_preview(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, points: &[Point], hover_board: Option<Point>) {
    if points.is_empty() {
        return;
    }
    let color = Color32::from_rgb(255, 200, 60);
    let screen_points: Vec<egui::Pos2> = points.iter().map(|&p| camera.board_to_screen(rect, p)).collect();
    for pair in screen_points.windows(2) {
        painter.line_segment([pair[0], pair[1]], Stroke::new(2.0, color));
    }
    for &p in &screen_points {
        painter.circle_filled(p, 3.0, color);
    }
    if let (Some(&last), Some(hover)) = (screen_points.last(), hover_board) {
        let hover_screen = camera.board_to_screen(rect, hover);
        painter.line_segment([last, hover_screen], Stroke::new(1.5, color.gamma_multiply(0.6)));
    }
    if points.len() >= 3 {
        painter.circle_stroke(screen_points[0], 6.0, Stroke::new(2.0, color));
    }
}

/// The dashed "you're centered" guide line(s) for an in-progress
/// [`Tool::Place`] matrix drag -- drawn across the *board's own* bounds
/// (not the screen), one full-height vertical line when
/// [`EditorState::snap_matrix_center`] snapped `x`, one full-width
/// horizontal line when it snapped `y`, either or both at once. Purely
/// visual feedback for "this axis is now centered" -- the actual
/// snapping already happened before this is called.
fn draw_matrix_snap_guides(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, bounds: alladin_geom::Aabb, snap_x: bool, snap_y: bool) {
    let color = Color32::from_rgba_unmultiplied(255, 220, 0, 180);
    let stroke = Stroke::new(1.5, color);
    if snap_x {
        let x = (bounds.min.x + bounds.max.x) / 2;
        let top = camera.board_to_screen(rect, Point::new(x, bounds.min.y));
        let bottom = camera.board_to_screen(rect, Point::new(x, bounds.max.y));
        painter.line_segment([top, bottom], stroke);
    }
    if snap_y {
        let y = (bounds.min.y + bounds.max.y) / 2;
        let left = camera.board_to_screen(rect, Point::new(bounds.min.x, y));
        let right = camera.board_to_screen(rect, Point::new(bounds.max.x, y));
        painter.line_segment([left, right], stroke);
    }
}

/// The thin "still needs a track here" lines for every net that has more
/// than one pad -- a minimum spanning tree over each net's pad centers
/// (see `ratsnest.rs`'s doc comment for why a spanning tree beats a star
/// or a complete graph here), drawn in that net's own colour so it reads
/// as "this is the same electrical net" at a glance.
fn draw_ratsnest(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, doc: &BoardDoc) {
    for net in &doc.nets {
        let pad_ids = doc.pads_on_net(net.id);
        let positions: Vec<Point> = pad_ids
            .iter()
            .filter_map(|&id| match doc.node.get(id) {
                Some(Item::Pad { shape, .. }) => Some(shape.center()),
                _ => None,
            })
            .collect();
        let color = alladin_render::net_color(Some(net.id)).gamma_multiply(0.7);
        for (a, b) in ratsnest::minimum_spanning_edges(&positions) {
            let from = camera.board_to_screen(rect, positions[a]);
            let to = camera.board_to_screen(rect, positions[b]);
            painter.line_segment([from, to], Stroke::new(1.0, color));
        }
    }
}

/// `offset` rotated by `rotation_deg` (same convention as
/// [`footprint::pad_world_position`]) and translated to `center`, in
/// board space -- the shared building block for both
/// [`rotated_rect_points`] and [`rotated_ellipse_points`].
fn rotate_and_place(offset: (f64, f64), center: Point, sin: f64, cos: f64) -> Point {
    let (x, y) = offset;
    let wx = x * cos - y * sin;
    let wy = x * sin + y * cos;
    Point::new(center.x + wx.round() as alladin_geom::Unit, center.y + wy.round() as alladin_geom::Unit)
}

fn rotated_rect_points(center: Point, width: alladin_geom::Unit, height: alladin_geom::Unit, rotation_deg: f64, camera: &Camera, rect: egui::Rect) -> Vec<egui::Pos2> {
    let (hw, hh) = (width as f64 / 2.0, height as f64 / 2.0);
    let rad = rotation_deg.to_radians();
    let (sin, cos) = rad.sin_cos();
    [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        .into_iter()
        .map(|corner| camera.board_to_screen(rect, rotate_and_place(corner, center, sin, cos)))
        .collect()
}

fn rotated_ellipse_points(center: Point, width: alladin_geom::Unit, height: alladin_geom::Unit, rotation_deg: f64, camera: &Camera, rect: egui::Rect) -> Vec<egui::Pos2> {
    const SEGMENTS: usize = 24;
    let (a, b) = (width as f64 / 2.0, height as f64 / 2.0);
    let rad = rotation_deg.to_radians();
    let (sin, cos) = rad.sin_cos();
    (0..SEGMENTS)
        .map(|i| {
            let t = (i as f64 / SEGMENTS as f64) * std::f64::consts::TAU;
            camera.board_to_screen(rect, rotate_and_place((a * t.cos(), b * t.sin()), center, sin, cos))
        })
        .collect()
}

/// A pad's real, already-placed geometry -- `radius` is the committed
/// [`Item::Pad`]'s collision circle (used as-is for `Circle`, ignored
/// otherwise), `shape`/`rotation_deg` come from the owning
/// [`crate::footprint::PadTemplate`] when known. Bundled together
/// purely to keep [`draw_pad_shape`]'s argument count sane.
struct PadGeometry {
    center: Point,
    radius: alladin_geom::Unit,
    shape: PadShapeKind,
    rotation_deg: f64,
}

/// How to paint a pad: its net-coloured fill, and whether it's pin 1
/// (drawn with a bright ring so a downloaded, or hand-added, part's
/// orientation is unambiguous at a glance -- the same convention real
/// boards use).
struct PadPaint {
    fill: Color32,
    highlight: bool,
}

/// The editor's canvas backdrop -- everything *outside* the board, so
/// the substrate fill below reads clearly as "the actual PCB" against
/// it.
const CANVAS_BACKGROUND: Color32 = Color32::from_rgb(20, 24, 20);

/// A plausible green FR-4 soldermask colour -- not sampled from any one
/// real board, but in the right ballpark of the "green PCB" look the
/// project's vision explicitly asks for (see the development log's
/// "Elfter"/"Zwölfter MVP-Slice" entries), which a stroke-only outline
/// (all `alladin_render::draw_board` gives you, by design -- see that
/// crate's module doc comment) can never provide on its own.
const SOLDERMASK_GREEN: Color32 = Color32::from_rgb(13, 90, 58);

/// The shoelace formula's doubled, unsigned area -- just enough to rank
/// `BoardDoc::outline`'s polygons by size (see
/// [`draw_board_substrate`]), not a general-purpose geometry primitive,
/// so it lives here rather than in `alladin_geom` alongside that
/// crate's own already-private `Polygon::signed_area`.
fn polygon_area_abs(poly: &Polygon) -> f64 {
    let points = &poly.points;
    if points.len() < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        sum += a.x as f64 * b.y as f64 - b.x as f64 * a.y as f64;
    }
    sum.abs()
}

/// Fills `outline` with a soldermask-green substrate before
/// `alladin_render::draw_board` strokes its edge on top -- purely
/// visual, no effect on any DRC/collision geometry (that's all still
/// `circle_within_outline`/`check_placement`, untouched by this).
/// Deliberately **not** added to `alladin_render` itself: that crate
/// never fills outlines/zones on purpose (see its module doc comment --
/// wrong results for a genuinely concave polygon, and real zone
/// polygons can have tens of thousands of vertices), a trade-off that's
/// right for `alladin-viewer`'s zone/outline rendering but not
/// acceptable to import wholesale here. This function narrows that
/// trade-off deliberately: it only ever fills `BoardDoc::outline`,
/// which today is always either `Polygon::rounded_rect` (convex, ~48
/// points) or a losslessly imported KiCad board outline of comparable
/// size -- ordinary boards, not copper pours. **Known limitation:** a
/// genuinely concave board outline (a real notch/cutout cut into the
/// main outline polygon, as opposed to a separate cutout polygon --
/// see below) would still render, just with a visibly wrong fill in
/// the concave area; this hasn't come up in practice yet since nothing
/// in this editor or its KiCad importer currently produces one.
///
/// Every polygon *other* than the largest by area is treated as a
/// cutout/hole (real boards commonly have both an outer outline and
/// separate hole polygons) and painted back over in the canvas
/// background colour, so a mounting-hole-sized cutout still reads as a
/// hole rather than vanishing into the green fill.
fn draw_board_substrate(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, outline: &[Polygon]) {
    let Some(board_index) = outline
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| polygon_area_abs(a).total_cmp(&polygon_area_abs(b)))
        .map(|(i, _)| i)
    else {
        return;
    };

    for (i, poly) in outline.iter().enumerate() {
        if poly.points.len() < 3 {
            continue;
        }
        let points: Vec<egui::Pos2> = poly.points.iter().map(|&p| camera.board_to_screen(rect, p)).collect();
        let fill = if i == board_index { SOLDERMASK_GREEN } else { CANVAS_BACKGROUND };
        painter.add(egui::Shape::convex_polygon(points, fill, Stroke::NONE));
    }
}

/// A light dot at every [`EditorState::grid_spacing`] intersection
/// across the visible viewport -- the visual half of "snap to grid":
/// without this, the grid a part actually snaps to (see
/// `snap_to_grid_point`) would be invisible until you'd already
/// dropped something onto it. Skipped entirely once dots would land
/// closer together on-screen than `MIN_DOT_SPACING_PX` (an extreme
/// zoom-out, or a very fine grid while zoomed out) -- packing
/// thousands of overlapping dots into a few visible pixels is both
/// meaningless and a real per-frame cost; `snap_to_grid_point` itself
/// keeps working exactly the same at that zoom level regardless, only
/// this visual aid drops out.
fn draw_placement_grid(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, spacing: Unit) {
    if spacing <= 0 {
        return;
    }
    const MIN_DOT_SPACING_PX: f32 = 6.0;
    let spacing_px = spacing as f32 / MM as f32 * camera.pixels_per_mm;
    if spacing_px < MIN_DOT_SPACING_PX {
        return;
    }
    let top_left = camera.screen_to_board(rect, rect.left_top());
    let bottom_right = camera.screen_to_board(rect, rect.right_bottom());
    let min_x = top_left.x.min(bottom_right.x);
    let max_x = top_left.x.max(bottom_right.x);
    let min_y = top_left.y.min(bottom_right.y);
    let max_y = top_left.y.max(bottom_right.y);
    let start_x = (min_x as f64 / spacing as f64).floor() as Unit * spacing;
    let start_y = (min_y as f64 / spacing as f64).floor() as Unit * spacing;
    let dot_color = Color32::from_rgba_unmultiplied(255, 255, 255, 45);

    let mut y = start_y;
    while y <= max_y {
        let mut x = start_x;
        while x <= max_x {
            painter.circle_filled(camera.board_to_screen(rect, Point::new(x, y)), 1.0, dot_color);
            x += spacing;
        }
        y += spacing;
    }
}

/// Draws one pad with its *true* shape rather than always a plain
/// circle -- see `footprint.rs`'s doc comment for why collision
/// geometry and rendered shape are allowed to differ like this.
fn draw_pad_shape(painter: &egui::Painter, rect: egui::Rect, camera: &Camera, geometry: &PadGeometry, paint: &PadPaint) {
    let (stroke_color, stroke_width) = if paint.highlight { (Color32::from_rgb(255, 210, 0), 2.5) } else { (Color32::from_rgb(20, 20, 20), 1.0) };
    match geometry.shape {
        PadShapeKind::Circle => {
            let center_px = camera.board_to_screen(rect, geometry.center);
            let radius_px = (geometry.radius as f32 / MM as f32 * camera.pixels_per_mm).max(1.0);
            painter.circle_filled(center_px, radius_px, paint.fill);
            painter.circle_stroke(center_px, radius_px, Stroke::new(stroke_width, stroke_color));
        }
        PadShapeKind::Rect { width, height } => {
            let points = rotated_rect_points(geometry.center, width, height, geometry.rotation_deg, camera, rect);
            painter.add(egui::Shape::convex_polygon(points, paint.fill, Stroke::new(stroke_width, stroke_color)));
        }
        PadShapeKind::Oval { width, height } => {
            let points = rotated_ellipse_points(geometry.center, width, height, geometry.rotation_deg, camera, rect);
            painter.add(egui::Shape::convex_polygon(points, paint.fill, Stroke::new(stroke_width, stroke_color)));
        }
    }
}

/// Every placed footprint's pads, with their real shape, pad number,
/// and a highlighted pin 1 -- plus its reference designator drawn above
/// it, silkscreen-style. Takes over pad rendering entirely from
/// `alladin_render::draw_board` (see the call site: `layers.pads` is
/// forced off there) since every pad here always belongs to some
/// footprint/template, so this can always do better than a plain
/// circle -- either the part's *real* shape (when its template is
/// still known) or, worst case, the exact same plain-circle fallback
/// `draw_board` used to draw (when a template was deleted from the
/// database after being placed -- a real, if rare, edge case).
fn draw_footprint_details(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    doc: &BoardDoc,
    templates: &[FootprintTemplate],
    layers: &LayerToggles,
    exclude: Option<FootprintId>,
    highlight_net: Option<NetId>,
) {
    if !layers.pads {
        return;
    }
    for fp in &doc.footprints {
        if Some(fp.id) == exclude {
            continue;
        }
        let template = templates.iter().find(|t| t.name == fp.template_name);
        for (index, &pad_id) in fp.pad_item_ids.iter().enumerate() {
            let Some(Item::Pad { shape, net, layer }) = doc.node.get(pad_id) else { continue };
            if *layer == LayerId::BCu && !layers.back_layer {
                continue;
            }
            let fill = alladin_render::net_highlight_dim(alladin_render::layer_tint(*layer, alladin_render::net_color(*net)), *net, highlight_net);
            let pad_template = template.and_then(|t| t.pads.get(index));
            let kind = pad_template.map(|p| p.shape).unwrap_or(PadShapeKind::Circle);
            let number = pad_template.map(|p| p.number.as_str()).unwrap_or("");
            let total_rotation = fp.rotation_deg + pad_template.map(|p| p.rotation_deg).unwrap_or(0.0);
            let is_pin_one = number == "1";
            // `bounding_radius()` only actually gets drawn for
            // `PadShapeKind::Circle` (see `draw_pad_shape`'s `match`) --
            // and a `PadShape::Circle`'s bounding radius is its exact
            // radius, so this is not an approximation for the case that
            // matters here. `Rect`/`Oval` pads instead use `kind`'s own
            // `width`/`height` below, never this field.
            let geometry = PadGeometry { center: shape.center(), radius: shape.bounding_radius(), shape: kind, rotation_deg: total_rotation };
            draw_pad_shape(painter, rect, camera, &geometry, &PadPaint { fill, highlight: is_pin_one });
            if !number.is_empty() {
                let center_px = camera.board_to_screen(rect, shape.center());
                painter.text(center_px, egui::Align2::CENTER_CENTER, number, egui::FontId::proportional(10.0), Color32::BLACK);
            }
        }

        if !fp.reference.is_empty() {
            // The exact geometry `alladin_kicad_io::writer` exports the
            // Reference property with (local offset above the part's
            // pad extent, rotating *with* the footprint, KiCad's
            // default 1.27mm size, DFM-floor stroke width), rendered
            // with the same embedded Hershey strokes the Gerber/KiCad export
            // uses -- so the preview's "U73" sits, rotates, and looks
            // exactly like the exported board's/Gerber's.
            let local_y = template.map(reference_label_local_y).unwrap_or(-2 * MM);
            let label = crate::board_doc::SilkText {
                id: crate::board_doc::SilkTextId(usize::MAX),
                text: fp.reference.clone(),
                position: Point::new(0, local_y).rotated(fp.rotation_deg).add(fp.position),
                rotation_deg: fp.rotation_deg,
                layer: LayerId::FCu,
                height: (1.27 * MM as f64) as Unit,
                line_width: JlcpcbDfm::MIN_SILK_LINE_WIDTH,
            };
            draw_silk_text_strokes(painter, rect, camera, &label, Color32::from_rgb(225, 225, 225));
        }

        // A faint dashed-looking (deliberately thin, low-alpha)
        // rectangle around the part's own real mechanical body/
        // courtyard (see `crate::board_doc::PlacedFootprint::courtyard`'s
        // own doc comment) -- lets a user actually *see* whether a
        // part's pins poke outside its own real body, and whether two
        // neighbouring bodies are crowding each other, without that
        // requiring the placement/DRC rejection to already have fired.
        alladin_render::draw_polygon_outline(painter, rect, camera, &fp.courtyard, Stroke::new(1.0, Color32::from_rgba_unmultiplied(180, 180, 190, 130)));
    }
}

impl eframe::App for PcbApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // Without this, egui/winit stops repainting once idle (no mouse/
        // keyboard input, e.g. an unfocused or minimized window) -- and
        // since `crate::mcp`'s query queue is only ever drained from
        // *inside* this very method (see its module doc comment), an
        // idle window would silently stop answering MCP calls at all,
        // eventually timing every one of them out. A cheap ~10Hz repaint
        // floor keeps this method running continuously regardless of
        // window focus, so an AI driving the GUI headlessly/in the
        // background is never at the mercy of "is anyone moving the
        // mouse right now".
        ui.ctx().request_repaint_after(std::time::Duration::from_millis(100));

        while let Ok(query) = self.mcp_rx.try_recv() {
            if let Some(pending) = &self.pending_job {
                if query.is_write() {
                    query.reply_now(busy_json(pending.label));
                    continue;
                }
            }
            match try_start_background_job(query, &mut self.screen, &self.parts_db) {
                Ok(pending) => self.pending_job = Some(pending),
                Err(query) => handle_mcp_query(query, &mut self.screen, &mut self.parts_db),
            }
        }
        // Poll (never blocks) whatever `try_start_background_job` above
        // most recently started -- once per frame, the same shape
        // [`EditorState::poll_autoroute_job`]/[`EditorState::poll_zone_job`]
        // already use for their own background jobs. Deliberately
        // outside the `match &mut self.screen` block below: `apply`
        // needs its own brief `&mut self.screen` borrow, which that
        // match's own (much longer-lived) borrow would otherwise be
        // holding for its whole body.
        let just_finished = match &mut self.pending_job {
            Some(pending) => match pending.job.poll() {
                JobPoll::Pending => None,
                JobPoll::Ready(apply) => Some(Ok(apply)),
                JobPoll::Lost => Some(Err(())),
            },
            None => None,
        };
        if let Some(outcome) = just_finished {
            let pending = self.pending_job.take().expect("`just_finished` is only `Some` when `self.pending_job` just matched `Some` above");
            let text = match outcome {
                Ok(apply) => apply(&mut self.screen),
                Err(()) => error_json("the background job ended unexpectedly (it may have panicked) -- please retry").to_string(),
            };
            if let Some(reply) = pending.reply {
                let _ = reply.send(text);
            }
        }
        // Same non-blocking once-per-frame poll as `pending_job` just
        // above, for [`PcbApp::board_load`] instead -- see
        // [`PendingBoardLoad::start`]'s doc comment for why loading a
        // board at all needs this. On success this is the one place a
        // still-`Screen::NewBoard` launch (or an "Open board" click
        // from an already-open [`Screen::Editor`]) ever turns into a
        // freshly loaded [`Screen::Editor`]; on failure/panic the
        // *current* screen is left exactly as it was -- silently for a
        // fresh launch (matching the old "fall back to New board" this
        // replaced), or with `io_message` set if the user was already
        // editing another board and asked to open a different one.
        let just_loaded = match &mut self.board_load {
            Some(pending) => match pending.job.poll() {
                JobPoll::Pending => None,
                JobPoll::Ready(result) => Some(Some(result)),
                JobPoll::Lost => Some(None),
            },
            None => None,
        };
        if let Some(outcome) = just_loaded {
            let pending = self.board_load.take().expect("`just_loaded` is only `Some` when `self.board_load` just matched `Some` above");
            let result = outcome.unwrap_or_else(|| Err("the board-load background thread ended unexpectedly -- please retry".to_string()));
            match result {
                Ok(doc) => {
                    remember_last_board(&pending.path);
                    let mut state = EditorState::new(doc, pending.templates, pending.template_origin, pending.template_hover, pending.template_category);
                    state.set_file_path(pending.path);
                    self.screen = Screen::Editor(state);
                }
                Err(e) => {
                    if let Screen::Editor(state) = &mut self.screen {
                        state.io_message = Some(format!("Couldn't open board: {e}"));
                    }
                }
            }
        }
        // Copied out up front so both `match` arms below can read it
        // freely without fighting the exclusive `&mut self.screen`
        // borrow the match itself holds for its whole body.
        let allow_ai_write = self.allow_ai_write;
        let board_loading = self.board_load.is_some();
        match &mut self.screen {
            Screen::NewBoard(params) => {
                let mut create_requested = false;
                egui::CentralPanel::default().show(ui, |ui| {
                    ui.vertical_centered(|ui| {
                        ui.add_space(40.0);
                        ui.heading("Alladin PCB \u{2014} New board");
                        if allow_ai_write {
                            ui.colored_label(Color32::from_rgb(255, 180, 60), "\u{1F513} AI-Schreibzugriff aktiv (MCP)");
                        }
                        // Only ever set here at launch, for the "jump
                        // straight back into the last board" auto-open
                        // (see [`PendingBoardLoad::start`]'s doc
                        // comment) -- this screen otherwise has no
                        // "Open board" button of its own to trigger a
                        // second case.
                        if board_loading {
                            ui.add_space(10.0);
                            ui.colored_label(Color32::from_rgb(120, 170, 255), "\u{23F3} Board wird geladen\u{2026}");
                        }
                        ui.add_space(20.0);
                    });

                    egui::Grid::new("new_board_grid").num_columns(2).spacing([12.0, 10.0]).show(ui, |ui| {
                        ui.label("Width (mm)");
                        ui.add(egui::DragValue::new(&mut params.width_mm).range(1.0..=500.0).speed(0.5));
                        ui.end_row();

                        ui.label("Height (mm)");
                        ui.add(egui::DragValue::new(&mut params.height_mm).range(1.0..=500.0).speed(0.5));
                        ui.end_row();

                        ui.label("Layers");
                        egui::ComboBox::from_id_salt("layer_count")
                            .selected_text(format!("{}", params.layer_count))
                            .show_ui(ui, |ui| {
                                for option in LayerCount::ALL {
                                    ui.selectable_value(&mut params.layer_count, option, format!("{option}"));
                                }
                            });
                        ui.end_row();

                        ui.label("Copper weight");
                        egui::ComboBox::from_id_salt("copper_weight")
                            .selected_text(format!("{}", params.copper_weight))
                            .show_ui(ui, |ui| {
                                for option in CopperWeight::ALL {
                                    ui.selectable_value(&mut params.copper_weight, option, format!("{option}"));
                                }
                            })
                            .response
                            .on_hover_text("Picks which real JLCPCB DFM/clearance rules this board enforces for its whole lifetime -- 2oz needs wider track spacing (0.16mm vs 1oz's 0.10mm).");
                        ui.end_row();

                        ui.label("Corner radius (mm)");
                        ui.add(egui::DragValue::new(&mut params.corner_radius_mm).range(0.0..=50.0).speed(0.1));
                        ui.end_row();
                    });

                    ui.add_space(20.0);
                    if !params.is_valid() {
                        ui.colored_label(
                            egui::Color32::from_rgb(230, 90, 90),
                            "Invalid dimensions: width/height must be positive and the corner radius must fit within the board.",
                        );
                    }
                    ui.add_enabled_ui(params.is_valid(), |ui| {
                        if ui.button("Create board").clicked() {
                            create_requested = true;
                        }
                    });
                });

                if create_requested {
                    let doc = params.create();
                    let (templates, template_origin, template_hover, template_category) = load_templates(&self.parts_db);
                    self.screen = Screen::Editor(EditorState::new(doc, templates, template_origin, template_hover, template_category));
                }
            }
            Screen::Editor(state) => {
                let mut new_board_requested = false;
                let mut open_requested = false;
                let mut save_requested = false;
                let mut save_as_requested = false;
                let mut export_manufacturing_requested = false;
                let mut create_part_requested = false;
                // Neither of these is acted on directly below (needs
                // `&self.parts_db`, only reachable outside this
                // closure) -- and even once it is reachable, still
                // doesn't delete anything itself, only stages
                // `state.pending_delete` for
                // `draw_delete_confirmation_window` to actually act on
                // once the user confirms. See [`PendingDelete`]'s own
                // doc comment for why.
                let mut delete_part_requested: Option<(usize, i64, String)> = None;
                // A category-tree header's own "delete" button's
                // exact prefix plus how many parts it would remove
                // (already known right here at the header's own render
                // site, see `group_templates_by_category`'s doc
                // comment for the tree shape this comes from).
                let mut delete_category_requested: Option<(String, usize)> = None;

                // Reactive `egui` only re-runs this whole method on
                // user input by default -- without this, an AI/script
                // changing the board file while the user's hands are
                // off the mouse would sit unnoticed until the next
                // click. Requesting a repaint a third of a second out
                // keeps this method (and so `maybe_reload_from_disk`'s
                // own poll right below) ticking on its own the whole
                // time the editor is open, at a deliberately low,
                // barely-perceptible-lag cadence rather than every
                // single frame.
                ui.ctx().request_repaint_after(std::time::Duration::from_millis(300));
                state.maybe_reload_from_disk(&self.parts_db, ui.input(|i| i.time));
                state.poll_autoroute_job();
                state.poll_zone_job();
                state.poll_export_job();

                if let Some(rx) = &state.lcsc_fetch {
                    match rx.try_recv() {
                        Ok(Ok(part)) => {
                            state.lcsc_fetch = None;
                            match self.parts_db.insert_part_categorized(
                                &part.name,
                                &part.reference_prefix,
                                &part.description,
                                Some(&part.lcsc_code),
                                &part.pads,
                                &[],
                                false,
                                part.explicit_courtyard,
                                part.category.as_deref(),
                            ) {
                                Ok(record) => {
                                    state.lcsc_message = Some((true, format!("{} ({}) added to your parts database.", record.template.name, part.lcsc_code)));
                                    let tooltip = format!("{}: {}", part.lcsc_code, part.description);
                                    state.templates.push(record.template);
                                    state.template_origin.push(Some(record.id));
                                    state.template_hover.push(Some(tooltip));
                                    state.template_category.push(record.category);
                                    state.tool = Tool::Place(state.templates.len() - 1);
                                    state.clear_selection();
                                }
                                Err(crate::parts_db::PartsDbError::DuplicateLcscCode(code)) => {
                                    // Already downloaded before -- select it for placing instead of
                                    // just reporting a dead-end error.
                                    let existing = state.template_origin.iter().position(|origin| {
                                        origin.and_then(|id| self.parts_db.find_by_lcsc_code(&code).ok().flatten().map(|r| r.id == id)).unwrap_or(false)
                                    });
                                    if let Some(index) = existing {
                                        state.tool = Tool::Place(index);
                                        state.clear_selection();
                                    }
                                    state.lcsc_message = Some((true, format!("{code} is already in your parts database \u{2014} selected for placing.")));
                                }
                                Err(e) => state.lcsc_message = Some((false, format!("Downloaded, but couldn't save to the database: {e}"))),
                            }
                        }
                        Ok(Err(e)) => {
                            state.lcsc_fetch = None;
                            state.lcsc_message = Some((false, e.to_string()));
                        }
                        Err(std::sync::mpsc::TryRecvError::Empty) => {}
                        Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                            state.lcsc_fetch = None;
                            state.lcsc_message = Some((false, "the download thread ended unexpectedly".to_string()));
                        }
                    }
                }

                if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                    state.tool = Tool::Select;
                    state.pending_connect = None;
                    state.routing = None;
                    state.trace_dragging = None;
                    state.via_net = None;
                    state.zone_points.clear();
                    state.zone_net = None;
                    if state.pending_pin_via.take().is_some() {
                        // Otherwise this leaves `begin_pin_via_relocation`'s
                        // "move the part until it turns green" message
                        // on screen forever, describing a mode that
                        // Escape just silently ended -- misleading
                        // about what a further click will now actually
                        // do (ordinary `Tool::Select` again, not "place
                        // the pending via").
                        state.io_message = None;
                    }
                }
                if ui.input(|i| i.key_pressed(egui::Key::R)) {
                    match state.tool {
                        Tool::Place(_) => state.place_rotation_deg = (state.place_rotation_deg + 90.0) % 360.0,
                        Tool::PlaceSilkText => state.silk_text_place_rotation_deg = (state.silk_text_place_rotation_deg + 90.0) % 360.0,
                        Tool::Select => state.rotate_selected(),
                        Tool::Connect | Tool::Route | Tool::PlaceVia | Tool::DrawZone | Tool::PlaceSilkDot => {}
                    }
                }
                if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    if let Tool::DrawZone = state.tool {
                        state.finish_zone();
                    }
                }
                if ui.input(|i| i.key_pressed(egui::Key::V)) {
                    if let (Tool::Route, Some(routing)) = (state.tool, &mut state.routing) {
                        state.route_message = match routing.drop_via_and_switch_layer(&mut state.doc) {
                            Ok(()) => None,
                            Err(e) => Some(e.to_string()),
                        };
                    }
                }
                if ui.input(|i| i.key_pressed(egui::Key::Space)) {
                    if let (Tool::Route, Some(routing)) = (state.tool, &mut state.routing) {
                        state.route_message = if routing.fix_corner() {
                            None
                        } else {
                            Some("can't fix a corner here \u{2014} move the mouse first, or this leg is blocked".to_string())
                        };
                    }
                }
                // Backspace un-fixes the last routing corner while an
                // in-progress `Tool::Route` drag exists (`state.selected`/
                // `state.selected_item` are always `None` in that tool,
                // see every `Tool::Route` switch-in's `clear_selection()`
                // call, so this never shadows the "delete selected
                // footprint/trace" gesture below); otherwise it falls
                // through to that gesture, same as Delete.
                if ui.input(|i| i.key_pressed(egui::Key::Backspace)) && matches!((state.tool, &state.routing), (Tool::Route, Some(_))) {
                    if let Some(routing) = &mut state.routing {
                        state.route_message = if routing.undo_last_corner() {
                            None
                        } else {
                            Some("no fixed corner to undo yet".to_string())
                        };
                    }
                } else if ui.input(|i| i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace)) {
                    if let Some(id) = state.selected.take() {
                        state.doc.remove_footprint(id);
                    } else if let Some(id) = state.selected_item.take() {
                        state.doc.remove_wire(id);
                    } else if let Some(id) = state.selected_silk_text.take() {
                        state.doc.remove_silk_text(id);
                    } else if let Some(id) = state.selected_silk_dot.take() {
                        state.doc.remove_silk_dot(id);
                    }
                }

                egui::Panel::top("top_panel").show(ui, |ui| {
                    // Several separate rows on purpose, not one long
                    // `ui.horizontal` -- a plain `ui.horizontal` never
                    // wraps in egui, so on a narrower window the tail end
                    // (e.g. the layer-visibility checkboxes, or later
                    // "Draw zone"/"Refill zones") silently got clipped
                    // behind the right-hand parts panel or the window
                    // edge, impossible to reach without resizing the
                    // whole window. This still doesn't fully fix that --
                    // this first row alone (board label through "Refill
                    // zones") already has too many buttons to reliably
                    // fit even a normal-width window -- so it uses
                    // `horizontal_wrapped` instead of `horizontal`:
                    // egui itself flows any button that doesn't fit onto
                    // a fresh line within the same row, rather than
                    // clipping or requiring a hardcoded second row here.
                    ui.horizontal_wrapped(|ui| {
                        ui.label(format!(
                            "Alladin PCB \u{2014} {}-layer, {} board",
                            state.doc.layer_count.as_u8(),
                            state.doc.copper_weight
                        ));
                        ui.separator();
                        // Always visible, never just implied -- see
                        // `PcbApp::allow_ai_write`'s doc comment for why
                        // this state should never be silent.
                        if allow_ai_write {
                            ui.colored_label(Color32::from_rgb(255, 180, 60), "\u{1F513} AI-Schreibzugriff aktiv (MCP)").on_hover_text(
                                "Dieser Prozess wurde mit --allow-ai-write gestartet: eine KI kann über MCP Bauteile platzieren, verbinden, routen und speichern.",
                            );
                        } else {
                            ui.weak("\u{1F512} AI-Schreibzugriff aus (nur lesen via MCP)").on_hover_text(
                                "Zum Aktivieren: alladin-pcb mit --allow-ai-write neu starten.",
                            );
                        }
                        // The MCP-driven background job, if any -- see
                        // `PcbApp::pending_job`'s own doc comment.
                        // Deliberately just a status line, not a modal:
                        // panning/inspecting the board (anything that
                        // doesn't itself try to start a second
                        // conflicting write) keeps working while this
                        // shows.
                        if let Some(pending) = &self.pending_job {
                            ui.separator();
                            ui.colored_label(Color32::from_rgb(120, 170, 255), format!("\u{23F3} {} (MCP)\u{2026}", pending.label));
                        }
                        // A different board being opened on top of this
                        // one -- see the "Open board..." handler below
                        // and [`PendingBoardLoad::start`]'s doc comment.
                        if board_loading {
                            ui.separator();
                            ui.colored_label(Color32::from_rgb(120, 170, 255), "\u{23F3} Board wird geöffnet\u{2026}");
                        }
                        ui.separator();
                        if ui.button("Fit to board").clicked() {
                            state.fitted = false;
                        }
                        ui.separator();
                        if ui.button("New board...").clicked() {
                            new_board_requested = true;
                        }
                        ui.separator();
                        if ui.button("Open...").clicked() {
                            open_requested = true;
                        }
                        if ui.button("Save").clicked() {
                            save_requested = true;
                        }
                        if ui.button("Save As...").clicked() {
                            save_as_requested = true;
                        }
                        ui.separator();
                        if ui
                            .add_enabled(state.export_job.is_none(), egui::Button::new(if state.export_job.is_some() { "Exporting\u{2026}" } else { "Export manufacturing files..." }))
                            .on_hover_text("Native Gerbers + drill (zip), JLCPCB CPL, and BOM CSV. No KiCad required.")
                            .clicked()
                        {
                            export_manufacturing_requested = true;
                        }
                        ui.separator();
                        let autoroute_busy = matches!(&state.autoroute_job, Some(job) if job.result.is_none());
                        let autoroute_ready = state.external_router_diagnose.as_ref().map(|d| d.is_ready()).unwrap_or(true);
                        if ui
                            .add_enabled(!autoroute_busy, egui::Button::new(if autoroute_busy { "Autoroute (extern) running\u{2026}" } else { "Autoroute (extern)\u{2026}" }))
                            .on_hover_text(if autoroute_ready {
                                "Autoroute currently unwired nets via the external KiCadRoutingTools (drandyhaas) -- a separate tool you install yourself, see the gear icon for setup."
                            } else {
                                "Not fully set up yet -- click the gear icon to configure/diagnose KiCadRoutingTools first."
                            })
                            .clicked()
                        {
                            state.open_autoroute_dialog();
                        }
                        if ui.button("\u{2699}").on_hover_text("Autoroute (extern) settings").clicked() {
                            state.external_router_diagnose = Some(crate::external_router::diagnose(&state.external_router_settings));
                            state.show_external_router_settings = true;
                        }
                        ui.label(match &state.file_path {
                            Some(path) => path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default(),
                            None => "(unsaved)".to_string(),
                        });
                        ui.separator();
                        if ui.selectable_label(state.tool == Tool::Connect, "Connect pins").clicked() {
                            state.tool = Tool::Connect;
                            state.clear_selection();
                        }
                        if ui.selectable_label(state.tool == Tool::Route, "Route traces").clicked() {
                            state.tool = Tool::Route;
                            state.clear_selection();
                        }
                        if ui.selectable_label(state.tool == Tool::PlaceVia, "Place vias").clicked() {
                            state.tool = Tool::PlaceVia;
                            state.clear_selection();
                            state.via_net = None;
                            state.via_message = None;
                        }
                        if ui.selectable_label(state.tool == Tool::DrawZone, "Draw zone").clicked() {
                            state.tool = Tool::DrawZone;
                            state.clear_selection();
                            state.zone_points.clear();
                            state.zone_message = None;
                        }
                        if ui.selectable_label(state.tool == Tool::PlaceSilkText, "Place silk text").clicked() {
                            state.tool = Tool::PlaceSilkText;
                            state.clear_selection();
                            state.silk_text_message = None;
                            state.silk_text_place_rotation_deg = 0.0;
                        }
                        if ui.selectable_label(state.tool == Tool::PlaceSilkDot, "Place silk dot").clicked() {
                            state.tool = Tool::PlaceSilkDot;
                            state.clear_selection();
                            state.silk_dot_message = None;
                        }
                        if ui
                            .add_enabled(state.zone_job.is_none(), egui::Button::new("Refill zones"))
                            .clicked()
                        {
                            state.refill_all_zones_in_background();
                        }
                        if let Some((label, _)) = &state.zone_job {
                            ui.colored_label(Color32::from_rgb(120, 170, 255), format!("\u{23F3} {label}\u{2026}"));
                        } else if state.doc.zones_are_stale() {
                            ui.colored_label(
                                Color32::from_rgb(255, 190, 60),
                                "\u{26A0} Zones may be stale (something moved/changed since the last fill) \u{2014} click Refill zones",
                            );
                        }
                    });
                    ui.horizontal(|ui| {
                        // These are the defaults every new `Tool::Route`
                        // drag ([`Self::handle_route_click`]), pin-stitching
                        // via ([`Self::add_pin_stitching_via_at`]), and
                        // [`Tool::PlaceVia`] click currently reads from --
                        // shown here as plain mm floats and converted back
                        // to `Unit` on change, since `EditorState`'s own
                        // fields stay in nanometre-precise `Unit`
                        // throughout (see the module doc comment on why).
                        // Lower bound on each of these three is the real
                        // JLCPCB DFM minimum, never a smaller round number
                        // -- a value here isn't just cosmetic, it's the
                        // exact width/diameter/drill the next manual route,
                        // via placement, or pin-stitching via commits to
                        // the board (see this struct's own `trace_width`/
                        // `via_diameter`/`via_drill` doc comments). Upper
                        // bound stays generous: nothing stops a
                        // deliberately wider power trace or a bigger
                        // through-hole via for current capacity, only
                        // *thinner/smaller-than-manufacturable* is refused.
                        ui.label("Trace width (mm):");
                        let mut trace_width_mm = state.trace_width as f64 / MM as f64;
                        if ui
                            .add(egui::DragValue::new(&mut trace_width_mm).range(JlcpcbDfm::MIN_TRACK_WIDTH as f64 / MM as f64..=5.0).speed(0.01))
                            .changed()
                        {
                            state.trace_width = (trace_width_mm * MM as f64).round() as Unit;
                        }
                        ui.label("Via diameter (mm):");
                        let mut via_diameter_mm = state.via_diameter as f64 / MM as f64;
                        if ui
                            .add(egui::DragValue::new(&mut via_diameter_mm).range(JlcpcbDfm::MIN_VIA_DIAMETER as f64 / MM as f64..=10.0).speed(0.01))
                            .changed()
                        {
                            state.via_diameter = (via_diameter_mm * MM as f64).round() as Unit;
                        }
                        ui.label("Via drill (mm):");
                        let mut via_drill_mm = state.via_drill as f64 / MM as f64;
                        if ui
                            .add(egui::DragValue::new(&mut via_drill_mm).range(JlcpcbDfm::MIN_VIA_HOLE as f64 / MM as f64..=5.0).speed(0.01))
                            .changed()
                        {
                            state.via_drill = (via_drill_mm * MM as f64).round() as Unit;
                        }
                        if ui
                            .button("Reset")
                            .on_hover_text("Resets trace width/via diameter/via drill back to Alladin's own JLCPCB-safe defaults (0.25/0.6/0.3mm) -- does not touch anything already placed on the board.")
                            .clicked()
                        {
                            state.trace_width = crate::routing::DEFAULT_TRACE_WIDTH;
                            state.via_diameter = DEFAULT_VIA_DIAMETER;
                            state.via_drill = DEFAULT_VIA_DRILL;
                        }
                        ui.separator();
                        ui.checkbox(&mut state.grid_snap_enabled, "Snap to grid").on_hover_text(
                            "Placing a new part (single or matrix) and dragging an already-placed one both round to the nearest grid intersection, so parts line up cleanly next to/under each other.",
                        );
                        ui.add_enabled_ui(state.grid_snap_enabled, |ui| {
                            ui.label("Grid (mm):");
                            let mut grid_mm = state.grid_spacing as f64 / MM as f64;
                            if ui.add(egui::DragValue::new(&mut grid_mm).range(0.05..=50.0).speed(0.01)).changed() {
                                state.grid_spacing = (grid_mm * MM as f64).round().max(1.0) as Unit;
                            }
                        });
                    });
                    ui.horizontal(|ui| {
                        ui.label("Show:");
                        ui.checkbox(&mut state.layers.outline, "Outline");
                        ui.checkbox(&mut state.layers.pads, "Pads");
                        ui.checkbox(&mut state.layers.tracks, "Tracks");
                        ui.checkbox(&mut state.layers.vias, "Vias");
                        ui.checkbox(&mut state.layers.zones, "Zones");
                        ui.checkbox(&mut state.layers.back_layer, "Show back copper (B.Cu)")
                            .on_hover_text("Hide everything on B.Cu (pads/tracks/zones) instead of just dimming it.");
                        ui.checkbox(&mut state.layers.holes, "Mounting holes");
                        ui.checkbox(&mut state.show_ratsnest, "Ratsnest")
                            .on_hover_text("The thin \"still needs a track here\" lines between same-net pads. A straight line between two distant, already-connected pads can visually cross an unrelated part placed in between -- hide this if that ever reads as a connection that isn't really there.");
                    });
                    if let Some(message) = &state.io_message {
                        ui.colored_label(Color32::from_rgb(230, 90, 90), message);
                    }
                });

                egui::Panel::right("parts_panel").min_size(300.0).show(ui, |ui| {
                    // The panel's own content (parts library, placed
                    // parts, nets, planes, the current tool's own
                    // section) easily outgrows a shorter window --
                    // without this, anything past the fold (originally
                    // "Power/ground planes" and everything below it) was
                    // simply clipped with no way to reach it at all, the
                    // exact same class of bug the top toolbar's own
                    // "Show back copper (B.Cu)" checkbox hit before.
                    egui::ScrollArea::vertical().id_salt("side_panel_scroll").show(ui, |ui| {
                    ui.heading("Place part");
                    let mut selection_changed = false;
                    // Built-ins first, ungrouped, exactly as before this
                    // feature -- they have no category to group by, and
                    // aren't the user's own data to organize/delete.
                    for i in 0..state.templates.len() {
                        if state.template_origin[i].is_some() {
                            continue;
                        }
                        if place_part_row(ui, i, &state.templates, &state.template_origin, &state.template_hover, &mut state.tool, &mut delete_part_requested) {
                            selection_changed = true;
                        }
                    }

                    let category_tree = group_templates_by_category(&state.template_origin, &state.template_category);
                    for (top, subs) in &category_tree {
                        let total: usize = subs.values().map(Vec::len).sum();
                        egui::CollapsingHeader::new(format!("{top} ({total})")).id_salt(("category_top", top)).show(ui, |ui| {
                            if ui.small_button("\u{1F5D1} Delete this whole category").on_hover_text(format!("Deletes all {total} part(s) under \"{top}\", including every sub-category")).clicked() {
                                delete_category_requested = Some((top.clone(), total));
                            }
                            ui.separator();
                            // Every index with no sub-category (a plain,
                            // one-level category, or "Uncategorized"
                            // itself) renders directly here -- see
                            // `group_templates_by_category`'s own doc
                            // comment for why `""` is that sentinel key.
                            if let Some(direct) = subs.get("") {
                                for &i in direct {
                                    if place_part_row(ui, i, &state.templates, &state.template_origin, &state.template_hover, &mut state.tool, &mut delete_part_requested) {
                                        selection_changed = true;
                                    }
                                }
                            }
                            for (sub, indices) in subs {
                                if sub.is_empty() {
                                    continue;
                                }
                                let full_category = format!("{top}/{sub}");
                                egui::CollapsingHeader::new(format!("{sub} ({})", indices.len())).id_salt(("category_sub", &full_category)).show(ui, |ui| {
                                    if ui
                                        .small_button("\u{1F5D1} Delete this category")
                                        .on_hover_text(format!("Deletes all {} part(s) under \"{full_category}\"", indices.len()))
                                        .clicked()
                                    {
                                        delete_category_requested = Some((full_category.clone(), indices.len()));
                                    }
                                    ui.separator();
                                    for &i in indices {
                                        if place_part_row(ui, i, &state.templates, &state.template_origin, &state.template_hover, &mut state.tool, &mut delete_part_requested) {
                                            selection_changed = true;
                                        }
                                    }
                                });
                            }
                        });
                    }
                    if selection_changed {
                        state.clear_selection();
                    }

                    ui.add_space(4.0);
                    ui.separator();
                    ui.heading("Download part (LCSC)");
                    ui.horizontal(|ui| {
                        ui.label("C-number:");
                        let text_response = ui.add_enabled(state.lcsc_fetch.is_none(), egui::TextEdit::singleline(&mut state.lcsc_input).hint_text("C2040"));
                        let enter_pressed = text_response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                        let can_fetch = state.lcsc_fetch.is_none() && !state.lcsc_input.trim().is_empty();
                        let clicked = ui.add_enabled(can_fetch, egui::Button::new(if state.lcsc_fetch.is_some() { "Downloading\u{2026}" } else { "Download" })).clicked();
                        if can_fetch && (clicked || enter_pressed) {
                            state.lcsc_fetch = Some(crate::lcsc::fetch_in_background(state.lcsc_input.trim().to_string()));
                            state.lcsc_message = None;
                        }
                    });
                    if state.lcsc_fetch.is_some() {
                        ui.spinner();
                        ui.ctx().request_repaint();
                    }
                    if let Some((ok, message)) = &state.lcsc_message {
                        let color = if *ok { Color32::from_rgb(120, 200, 120) } else { Color32::from_rgb(230, 90, 90) };
                        ui.colored_label(color, message);
                    }

                    ui.add_space(4.0);
                    if ui.button(if state.add_part_form.is_some() { "Cancel add part" } else { "Add part to database..." }).clicked() {
                        state.add_part_form = if state.add_part_form.is_some() { None } else { Some(AddPartForm::default()) };
                    }
                    if let Some(form) = &mut state.add_part_form {
                        ui.group(|ui| {
                            egui::Grid::new("add_part_grid").num_columns(2).show(ui, |ui| {
                                ui.label("Name");
                                ui.text_edit_singleline(&mut form.name);
                                ui.end_row();
                                ui.label("Ref. prefix");
                                ui.text_edit_singleline(&mut form.reference_prefix);
                                ui.end_row();
                                ui.label("Pins");
                                ui.add(egui::DragValue::new(&mut form.pin_count).range(1..=64));
                                ui.end_row();
                                ui.label("Pitch (mm)");
                                ui.add(egui::DragValue::new(&mut form.pitch_mm).range(0.1..=20.0).speed(0.01));
                                ui.end_row();
                                ui.label("Pad radius (mm)");
                                // Lower bound is JLCPCB's real
                                // `min_smd_pad_size` (0.25mm *diameter*,
                                // see `alladin_core::JlcpcbDfm::MIN_SMD_PAD_SIZE`)
                                // halved to a radius -- a hand-built part
                                // can't be given an unmanufacturable pad
                                // to begin with, rather than only being
                                // caught later by some future DRC pass.
                                ui.add(egui::DragValue::new(&mut form.pad_radius_mm).range(0.125..=5.0).speed(0.01));
                                ui.end_row();
                                ui.label("Description");
                                ui.text_edit_singleline(&mut form.description);
                                ui.end_row();
                                ui.label("Category");
                                ui.add(egui::TextEdit::singleline(&mut form.category).hint_text("Uncategorized"));
                                ui.end_row();
                            });
                            let valid = !form.name.trim().is_empty() && !form.reference_prefix.trim().is_empty();
                            ui.add_enabled_ui(valid, |ui| {
                                if ui.button("Save to parts database").clicked() {
                                    create_part_requested = true;
                                }
                            });
                        });
                    }

                    ui.separator();
                    if let Tool::Place(_) = state.tool {
                        ui.horizontal(|ui| {
                            ui.label(format!("Rotation: {:.0}\u{b0}", state.place_rotation_deg));
                            if ui.button("Rotate (R)").clicked() {
                                state.place_rotation_deg = (state.place_rotation_deg + 90.0) % 360.0;
                            }
                        });
                        if ui.button("Cancel placement (Esc)").clicked() {
                            state.tool = Tool::Select;
                        }
                        ui.label("Click on the board to place. Keep clicking to place more.");

                        ui.add_space(4.0);
                        ui.heading("Matrix");
                        egui::Grid::new("matrix_grid").num_columns(4).spacing([8.0, 6.0]).show(ui, |ui| {
                            ui.label("Rows");
                            ui.add(egui::DragValue::new(&mut state.matrix_rows).range(1..=100));
                            ui.label("Cols");
                            ui.add(egui::DragValue::new(&mut state.matrix_cols).range(1..=100));
                            ui.end_row();
                            ui.label("Pitch X (mm)");
                            ui.add(egui::DragValue::new(&mut state.matrix_pitch_x_mm).range(0.1..=200.0).speed(0.1));
                            ui.label("Pitch Y (mm)");
                            ui.add(egui::DragValue::new(&mut state.matrix_pitch_y_mm).range(0.1..=200.0).speed(0.1));
                            ui.end_row();
                        });
                        let matrix_count = state.matrix_rows as u64 * state.matrix_cols as u64;
                        if matrix_count > 1 {
                            ui.label(format!(
                                "Places {matrix_count} instances ({} x {}) as one unit -- drag near the board's center axis to snap.",
                                state.matrix_rows, state.matrix_cols
                            ));
                        }
                    }

                    ui.separator();
                    ui.heading(format!("Parts ({})", state.doc.footprints.len()));
                    let mut to_delete = None;
                    let mut to_select = None;
                    egui::ScrollArea::vertical().id_salt("parts_scroll").max_height(260.0).show(ui, |ui| {
                        for fp in &state.doc.footprints {
                            let is_selected = state.selected == Some(fp.id);
                            ui.horizontal(|ui| {
                                if ui.selectable_label(is_selected, format!("{} ({})", fp.reference, fp.template_name)).clicked() {
                                    to_select = Some(fp.id);
                                }
                                if ui.small_button("\u{2716}").on_hover_text("Delete").clicked() {
                                    to_delete = Some(fp.id);
                                }
                            });
                        }
                    });
                    if let Some(id) = to_select {
                        state.selected = Some(id);
                        state.selected_item = None;
                        state.tool = Tool::Select;
                    }
                    if let Some(id) = to_delete {
                        state.doc.remove_footprint(id);
                        if state.selected == Some(id) {
                            state.clear_selection();
                        }
                    }

                    if let Some(id) = state.selected {
                        if let Some(fp) = state.doc.footprints.iter().find(|f| f.id == id) {
                            ui.separator();
                            ui.label(format!("Selected: {}", fp.reference));
                            ui.label(format!(
                                "Position: ({:.2}, {:.2}) mm",
                                fp.position.x as f64 / MM as f64,
                                fp.position.y as f64 / MM as f64
                            ));
                            ui.label(format!("Rotation: {:.0}\u{b0}", fp.rotation_deg));
                            ui.label("Drag it on the board to move. R to rotate, Del to remove.");
                            let mut marker_on = fp.pin1_marker.is_some();
                            if ui
                                .checkbox(&mut marker_on, "Pin-1-Punkt (Silk)")
                                .on_hover_text("Druckt einen kleinen Punkt neben Pad 1 dieses Bauteils auf den Silkscreen \u{2014} wandert bei Verschieben/Drehen automatisch mit.")
                                .changed()
                            {
                                if marker_on {
                                    match state.template_for(id) {
                                        Some((template_index, _)) => {
                                            let template = state.templates[template_index].clone();
                                            match state.doc.try_enable_pin1_marker(id, &template) {
                                                Ok(()) => state.silk_dot_message = None,
                                                Err(e) => state.silk_dot_message = Some(format!("Pin-1-Punkt: {e}")),
                                            }
                                        }
                                        None => state.silk_dot_message = Some("Pin-1-Punkt: unbekanntes Template.".to_string()),
                                    }
                                } else {
                                    state.doc.disable_pin1_marker(id);
                                }
                            }
                            if let Some(message) = &state.silk_dot_message {
                                ui.colored_label(Color32::from_rgb(230, 90, 90), message);
                            }
                        }
                    }

                    if let Some(id) = state.selected_item {
                        let wire_len = state.doc.connected_wire(id).len();
                        match state.doc.node.get(id) {
                            Some(Item::Track { .. }) => {
                                ui.separator();
                                ui.label(format!("Selected: trace ({wire_len} segment{})", if wire_len == 1 { "" } else { "s" }));
                                ui.label("Del/Backspace removes the whole wire \u{2014} its net stays intact.");
                            }
                            Some(Item::Via { .. }) => {
                                ui.separator();
                                ui.label(format!("Selected: via ({wire_len} item{} in this wire)", if wire_len == 1 { "" } else { "s" }));
                                ui.label("Del/Backspace removes the whole wire \u{2014} its net stays intact.");
                            }
                            _ => {}
                        }
                    }

                    if let Some(id) = state.selected_silk_text {
                        if let Some(text) = state.doc.silk_texts.iter().find(|t| t.id == id) {
                            let (text_content, position, rotation_deg, layer, height) =
                                (text.text.clone(), text.position, text.rotation_deg, text.layer, text.height);
                            ui.separator();
                            let side = if layer == LayerId::FCu { "front" } else { "back" };
                            ui.label(format!("Selected silk text: \u{201c}{text_content}\u{201d} ({side})"));
                            ui.label(format!("Position: ({:.2}, {:.2}) mm", position.x as f64 / MM as f64, position.y as f64 / MM as f64));
                            ui.label(format!("Rotation: {:.0}\u{b0}", rotation_deg));
                            ui.horizontal(|ui| {
                                ui.label(format!("Size: {:.1}mm", height as f64 / MM as f64));
                                if ui.button("\u{2212}").on_hover_text("Smaller").clicked() {
                                    let smaller = silk_text_height_step(height, -1);
                                    let _ = state.doc.try_resize_silk_text(id, smaller);
                                }
                                if ui.button("+").on_hover_text("Bigger").clicked() {
                                    let bigger = silk_text_height_step(height, 1);
                                    let _ = state.doc.try_resize_silk_text(id, bigger);
                                }
                            });
                            ui.label("Drag it on the board to move. R to rotate, Del to remove.");
                        }
                    }

                    if let Some(id) = state.selected_silk_dot {
                        if let Some(dot) = state.doc.silk_dots.iter().find(|d| d.id == id) {
                            let (position, diameter, layer) = (dot.position, dot.diameter, dot.layer);
                            ui.separator();
                            let side = if layer == LayerId::FCu { "front" } else { "back" };
                            ui.label(format!("Selected silk dot ({side})"));
                            ui.label(format!("Position: ({:.2}, {:.2}) mm", position.x as f64 / MM as f64, position.y as f64 / MM as f64));
                            ui.horizontal(|ui| {
                                ui.label(format!("Diameter: {:.1}mm", diameter as f64 / MM as f64));
                                if ui.button("\u{2212}").on_hover_text("Smaller").clicked() {
                                    let _ = state.doc.try_resize_silk_dot(id, silk_dot_diameter_step(diameter, -1));
                                }
                                if ui.button("+").on_hover_text("Bigger").clicked() {
                                    let _ = state.doc.try_resize_silk_dot(id, silk_dot_diameter_step(diameter, 1));
                                }
                            });
                            ui.label("Drag it on the board to move. Del to remove.");
                        }
                    }

                    ui.separator();
                    ui.heading(format!("Nets ({})", state.doc.nets.len()));
                    if let Tool::Connect = state.tool {
                        ui.label("Click a pin, then another to connect them. Shift-click a pin to disconnect it. Esc to stop.");
                        if state.pending_connect.is_some() {
                            ui.colored_label(Color32::from_rgb(255, 220, 0), "First pin selected \u{2014} click the pin to connect it to.");
                        }
                    }
                    if let Some(message) = &state.net_message {
                        ui.colored_label(Color32::from_rgb(230, 90, 90), message);
                    }
                    if state.highlighted_net.is_some() {
                        ui.label("Highlighting a net \u{2014} click its \u{25C9} again (or anywhere else's \u{25CB}) to change/clear it.");
                    }
                    let mut net_to_remove = None;
                    // Renaming is committed (and validated -- non-empty,
                    // not already used by a different net, see
                    // `BoardDoc::rename_net`'s own doc comment) only once
                    // the field loses focus (which egui also triggers on
                    // Enter), not on every keystroke: the field still
                    // *feels* like it edits `net.name` directly while
                    // typing (same as before this validation existed),
                    // it just gets a chance to reject/revert the result
                    // right at the end instead of letting an in-progress,
                    // possibly-empty-or-duplicate keystroke ever become
                    // this net's real name.
                    let mut net_rename_to_commit: Option<(NetId, String)> = None;
                    // Snapshotted *before* this frame's text field can
                    // live-mutate `net.name` -- the only way to put a
                    // rejected rename's name back afterwards, since
                    // `BoardDoc::rename_net` itself never touches
                    // `self.nets` at all when it refuses.
                    let previous_names: std::collections::HashMap<NetId, String> = state.doc.nets.iter().map(|n| (n.id, n.name.clone())).collect();
                    let pin_counts: Vec<usize> = state.doc.nets.iter().map(|n| state.doc.pad_count_on_net(n.id)).collect();
                    egui::ScrollArea::vertical().id_salt("nets_scroll").max_height(200.0).show(ui, |ui| {
                        for (net, count) in state.doc.nets.iter_mut().zip(pin_counts) {
                            ui.horizontal(|ui| {
                                let is_highlighted = state.highlighted_net == Some(net.id);
                                let swatch_color = alladin_render::net_color(Some(net.id));
                                let dot = egui::RichText::new(if is_highlighted { "\u{25C9}" } else { "\u{25CB}" }).color(swatch_color);
                                if ui.selectable_label(is_highlighted, dot).on_hover_text("Highlight every pad/track/via/zone on this net on the board \u{2014} click again to clear.").clicked() {
                                    state.highlighted_net = if is_highlighted { None } else { Some(net.id) };
                                }
                                let response = ui.text_edit_singleline(&mut net.name).on_hover_text("Rename this net, e.g. to \"GND\"/\"5V\" -- press Enter or click elsewhere to apply.");
                                if response.lost_focus() {
                                    net_rename_to_commit = Some((net.id, net.name.clone()));
                                }
                                ui.label(format!("({count} pin{})", if count == 1 { "" } else { "s" }));
                                if ui.small_button("\u{2716}").on_hover_text("Delete net (disconnects its pins and deletes every trace/via on it)").clicked() {
                                    net_to_remove = Some(net.id);
                                }
                            });
                        }
                    });
                    if let Some((id, typed_name)) = net_rename_to_commit {
                        if let Err(e) = state.doc.rename_net(id, &typed_name) {
                            state.net_message = Some(format!("Couldn't rename net: {e}"));
                            if let Some(net) = state.doc.nets.iter_mut().find(|n| n.id == id) {
                                if let Some(previous) = previous_names.get(&id) {
                                    net.name = previous.clone();
                                }
                            }
                        } else {
                            state.net_message = None;
                        }
                    }
                    if let Some(id) = net_to_remove {
                        state.doc.remove_net(id);
                        if state.highlighted_net == Some(id) {
                            state.highlighted_net = None;
                        }
                    }

                    ui.separator();
                    ui.heading("Power/ground planes");
                    ui.label("One click turns a whole copper layer into a solid pour for one net (e.g. bottom = GND, top = 5V).");
                    let plane_nets: Vec<(NetId, String)> = state.doc.nets.iter().map(|n| (n.id, n.name.clone())).collect();
                    for (layer, label, id_salt) in [(LayerId::FCu, "Solid F.Cu plane", "front_plane_net"), (LayerId::BCu, "Solid B.Cu plane", "back_plane_net")] {
                        ui.horizontal(|ui| {
                            let active = match layer {
                                LayerId::FCu => &state.front_plane_zones,
                                LayerId::BCu => &state.back_plane_zones,
                            };
                            let mut enabled = !active.is_empty();
                            if ui.checkbox(&mut enabled, label).changed() {
                                if enabled {
                                    let picked = match layer {
                                        LayerId::FCu => state.front_plane_net,
                                        LayerId::BCu => state.back_plane_net,
                                    };
                                    match picked {
                                        Some(net) => state.set_layer_plane(layer, Some(net)),
                                        None => state.zone_message = Some(format!("Pick a net for the {label} first.")),
                                    }
                                } else {
                                    state.set_layer_plane(layer, None);
                                }
                            }

                            let net_field = match layer {
                                LayerId::FCu => &mut state.front_plane_net,
                                LayerId::BCu => &mut state.back_plane_net,
                            };
                            let current_name = net_field.and_then(|id| plane_nets.iter().find(|(n, _)| *n == id)).map(|(_, name)| name.as_str()).unwrap_or("(pick a net)");
                            let mut newly_picked = None;
                            egui::ComboBox::from_id_salt(id_salt).selected_text(current_name).show_ui(ui, |ui| {
                                for (id, name) in &plane_nets {
                                    if ui.selectable_value(net_field, Some(*id), name).changed() {
                                        newly_picked = Some(*id);
                                    }
                                }
                            });
                            // Changing the net while the plane is already
                            // active must re-fill it on the new net right
                            // away, not silently keep pouring the old one
                            // until the checkbox is toggled off and back on.
                            if let Some(net) = newly_picked {
                                let active = match layer {
                                    LayerId::FCu => &state.front_plane_zones,
                                    LayerId::BCu => &state.back_plane_zones,
                                };
                                if !active.is_empty() {
                                    state.set_layer_plane(layer, Some(net));
                                }
                            }
                        });
                    }

                    if let Tool::Route = state.tool {
                        ui.separator();
                        ui.heading("Route traces");
                        ui.label(
                            "Click a connected pin, then steer the trace with the mouse \u{2014} it snaps to \
                             clean 45\u{b0}/90\u{b0} angles automatically. Space fixes a corner where you are \
                             and continues from there; Backspace un-fixes the last one. Hover a same-net \
                             pin and click to finish the connection (auto-routed the rest of the way if \
                             needed). V drops a via at the cursor and switches copper layer. Esc to cancel.",
                        );
                        if let Some(routing) = &state.routing {
                            if routing.fixed_corner_count() > 0 {
                                ui.colored_label(Color32::from_rgb(140, 200, 255), format!("{} corner(s) fixed.", routing.fixed_corner_count()));
                            }
                        }
                        if let Some(message) = &state.route_message {
                            ui.colored_label(Color32::from_rgb(230, 90, 90), message);
                        }
                    }

                    if let Tool::PlaceVia = state.tool {
                        ui.separator();
                        ui.heading("Place vias");
                        ui.label("Click a pin to pick its net, then click anywhere to drop stitching vias on that net. Esc to pick a different net.");
                        if let Some(net) = state.via_net {
                            let name = state.doc.nets.iter().find(|n| n.id == net).map(|n| n.name.as_str()).unwrap_or("?");
                            ui.colored_label(Color32::from_rgb(255, 220, 0), format!("Stitching net \"{name}\" \u{2014} click to place a via."));
                        }
                        if let Some(message) = &state.via_message {
                            ui.colored_label(Color32::from_rgb(230, 90, 90), message);
                        }
                    }

                    if let Tool::DrawZone = state.tool {
                        ui.separator();
                        ui.heading("Draw zone");
                        ui.label(
                            "Click to place outline points, click back on the first point (or press Enter) to \
                             close it and fill. Esc to cancel.",
                        );
                        ui.horizontal(|ui| {
                            ui.label("Net:");
                            let current_name = state
                                .zone_net
                                .and_then(|id| state.doc.nets.iter().find(|n| n.id == id))
                                .map(|n| n.name.as_str())
                                .unwrap_or("(pick a net)");
                            egui::ComboBox::from_id_salt("zone_net").selected_text(current_name).show_ui(ui, |ui| {
                                for net in &state.doc.nets {
                                    ui.selectable_value(&mut state.zone_net, Some(net.id), &net.name);
                                }
                            });
                        });
                        ui.horizontal(|ui| {
                            ui.label("Layer:");
                            ui.selectable_value(&mut state.zone_layer, LayerId::FCu, "F.Cu");
                            ui.selectable_value(&mut state.zone_layer, LayerId::BCu, "B.Cu");
                        });
                        ui.label(format!("{} point(s) placed.", state.zone_points.len()));
                        ui.horizontal(|ui| {
                            if ui.add_enabled(state.zone_points.len() >= 3, egui::Button::new("Finish outline")).clicked() {
                                state.finish_zone();
                            }
                            if ui.button("Cancel").clicked() {
                                state.zone_points.clear();
                                state.zone_message = None;
                            }
                        });
                        if let Some(message) = &state.zone_message {
                            ui.colored_label(Color32::from_rgb(230, 90, 90), message);
                        }
                    }

                    if let Tool::PlaceSilkText = state.tool {
                        ui.separator();
                        ui.heading("Place silk text");
                        ui.label("Type the annotation below, then click on the board to place it. R rotates 90\u{b0} at a time.");
                        ui.horizontal(|ui| {
                            ui.label("Text:");
                            ui.text_edit_singleline(&mut state.silk_text_input);
                        });
                        ui.horizontal(|ui| {
                            ui.label("Side:");
                            ui.selectable_value(&mut state.silk_layer, LayerId::FCu, "Front (F.SilkS)");
                            ui.selectable_value(&mut state.silk_layer, LayerId::BCu, "Back (B.SilkS)");
                        });
                        ui.horizontal(|ui| {
                            ui.label(format!("Rotation: {:.0}\u{b0}", state.silk_text_place_rotation_deg));
                            if ui.button("Rotate 90\u{b0}").clicked() {
                                state.silk_text_place_rotation_deg = (state.silk_text_place_rotation_deg + 90.0) % 360.0;
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label(format!("Size: {:.1}mm", state.silk_text_height as f64 / MM as f64));
                            if ui.button("\u{2212}").on_hover_text("Smaller").clicked() {
                                state.silk_text_height = silk_text_height_step(state.silk_text_height, -1);
                            }
                            if ui.button("+").on_hover_text("Bigger").clicked() {
                                state.silk_text_height = silk_text_height_step(state.silk_text_height, 1);
                            }
                        });
                        if state.silk_text_input.trim().is_empty() {
                            ui.colored_label(Color32::from_rgb(255, 190, 60), "Type some text above before clicking on the board.");
                        }
                        if let Some(message) = &state.silk_text_message {
                            ui.colored_label(Color32::from_rgb(230, 90, 90), message);
                        }
                    }
                    if let Tool::PlaceSilkDot = state.tool {
                        ui.separator();
                        ui.heading("Place silk dot");
                        ui.label("Click on the board to place a filled dot (e.g. a polarity/orientation mark).");
                        ui.horizontal(|ui| {
                            ui.label("Side:");
                            ui.selectable_value(&mut state.silk_layer, LayerId::FCu, "Front (F.SilkS)");
                            ui.selectable_value(&mut state.silk_layer, LayerId::BCu, "Back (B.SilkS)");
                        });
                        ui.horizontal(|ui| {
                            ui.label(format!("Diameter: {:.1}mm", state.silk_dot_diameter as f64 / MM as f64));
                            if ui.button("\u{2212}").on_hover_text("Smaller").clicked() {
                                state.silk_dot_diameter = silk_dot_diameter_step(state.silk_dot_diameter, -1);
                            }
                            if ui.button("+").on_hover_text("Bigger").clicked() {
                                state.silk_dot_diameter = silk_dot_diameter_step(state.silk_dot_diameter, 1);
                            }
                        });
                        if let Some(message) = &state.silk_dot_message {
                            ui.colored_label(Color32::from_rgb(230, 90, 90), message);
                        }
                    }
                    });
                });

                draw_external_router_settings_window(ui.ctx(), state);
                draw_autoroute_dialog_window(ui.ctx(), state);
                draw_delete_confirmation_window(ui.ctx(), state, &self.parts_db);

                egui::CentralPanel::default().show(ui, |ui| {
                    let (rect, response) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());

                    if !state.fitted {
                        if let Some(bounds) = state.board_bounds() {
                            state.camera.fit(rect, bounds);
                        }
                        state.fitted = true;
                    }

                    let hover_board = response.hover_pos().map(|p| state.camera.screen_to_board(rect, p));
                    state.last_hover_board = hover_board;

                    let hover_pad_tooltip = hover_board.and_then(|p| state.doc.pad_at(p)).and_then(|pad_id| {
                        let footprint = state.doc.footprints.iter().find(|f| f.pad_item_ids.contains(&pad_id))?;
                        let index = footprint.pad_item_ids.iter().position(|&id| id == pad_id)?;
                        let pad_template = state.templates.iter().find(|t| t.name == footprint.template_name)?.pads.get(index)?;
                        Some(match &pad_template.pin_name {
                            Some(name) => format!("{}.{}  ({name})", footprint.reference, pad_template.number),
                            None => format!("{}.{}", footprint.reference, pad_template.number),
                        })
                    });
                    let response = match hover_pad_tooltip {
                        Some(text) => response.on_hover_text(text),
                        None => response,
                    };

                    // Captured once, right on the click that opens the
                    // menu, rather than re-derived from `hover_board`
                    // inside the `context_menu` closure below: once the
                    // popup itself has mouse focus, `response.hover_pos()`
                    // no longer reliably reports the pad that was
                    // actually right-clicked.
                    if response.secondary_clicked() {
                        state.context_menu_pad = hover_board.and_then(|p| state.doc.pad_at(p));
                    }
                    response.context_menu(|ui| {
                        if let Some(pad_id) = state.context_menu_pad {
                            if ui.button("Add via near pin").clicked() {
                                state.add_pin_stitching_via_at(pad_id);
                                ui.close();
                            }
                        } else {
                            ui.label("(nothing here)");
                        }
                    });

                    if state.pending_pin_via.is_some() {
                        // The footprint+via unit follows the cursor
                        // exactly like an ordinary `Dragging` move,
                        // but driven purely by hover (see
                        // `PendingPinVia`'s own doc comment for why):
                        // no mouse button is being held for this one.
                        if let Some(board_pos) = hover_board {
                            state.update_pending_pin_via(board_pos);
                        }
                        if response.clicked() {
                            state.finish_pending_pin_via();
                        }
                    } else {
                        match state.tool {
                        Tool::Place(index) => {
                            if response.dragged() {
                                state.camera.center_mm -= state.camera.screen_delta_to_board_mm(response.drag_delta());
                            }
                            if response.clicked() {
                                if let Some(board_pos) = hover_board {
                                    let board_pos = snap_to_grid_point(board_pos, state.grid_spacing, state.grid_snap_enabled);
                                    let template = &state.templates[index];
                                    if state.matrix_rows.max(1) * state.matrix_cols.max(1) > 1 {
                                        let (center, _, _) = state.snap_matrix_center(board_pos);
                                        let positions = state.matrix_ghost_positions(center);
                                        let _ = state.doc.place_matrix(template, &positions, state.place_rotation_deg);
                                    } else {
                                        let _ = state.doc.try_place_footprint(template, board_pos, state.place_rotation_deg);
                                    }
                                }
                            }
                        }
                        Tool::PlaceSilkText => {
                            if response.dragged() {
                                state.camera.center_mm -= state.camera.screen_delta_to_board_mm(response.drag_delta());
                            }
                            if response.clicked() {
                                if let Some(board_pos) = hover_board {
                                    let board_pos = snap_to_grid_point(board_pos, state.grid_spacing, state.grid_snap_enabled);
                                    match state.doc.try_place_silk_text(&state.silk_text_input, board_pos, state.silk_text_place_rotation_deg, state.silk_layer, state.silk_text_height) {
                                        Ok(_) => {
                                            state.silk_text_message = None;
                                            // 0deg is silk text's standard: a one-off
                                            // rotation applies to the text it was made
                                            // for, never silently to the next one too.
                                            state.silk_text_place_rotation_deg = 0.0;
                                        }
                                        Err(e) => state.silk_text_message = Some(e.to_string()),
                                    }
                                }
                            }
                        }
                        Tool::PlaceSilkDot => {
                            if response.dragged() {
                                state.camera.center_mm -= state.camera.screen_delta_to_board_mm(response.drag_delta());
                            }
                            if response.clicked() {
                                if let Some(board_pos) = hover_board {
                                    let board_pos = snap_to_grid_point(board_pos, state.grid_spacing, state.grid_snap_enabled);
                                    match state.doc.try_place_silk_dot(board_pos, state.silk_dot_diameter, state.silk_layer) {
                                        Ok(_) => state.silk_dot_message = None,
                                        Err(e) => state.silk_dot_message = Some(e.to_string()),
                                    }
                                }
                            }
                        }
                        Tool::Select => {
                            if response.drag_started() {
                                if let Some(board_pos) = hover_board {
                                    state.begin_drag(board_pos);
                                }
                            }
                            if state.dragging.is_some() || state.silk_text_dragging.is_some() {
                                if response.dragged() {
                                    if let Some(board_pos) = hover_board {
                                        state.update_drag(board_pos);
                                    }
                                }
                                if response.drag_stopped() {
                                    state.finish_drag();
                                }
                            } else if state.trace_dragging.is_some() {
                                if response.dragged() {
                                    if let Some(board_pos) = hover_board {
                                        state.update_trace_drag(board_pos);
                                    }
                                }
                                if response.drag_stopped() {
                                    state.finish_trace_drag();
                                }
                            } else if response.dragged() {
                                state.camera.center_mm -= state.camera.screen_delta_to_board_mm(response.drag_delta());
                            }
                            if response.clicked() {
                                if let Some(board_pos) = hover_board {
                                    state.handle_select_click(board_pos);
                                }
                            }
                        }
                        Tool::Connect => {
                            if response.dragged() {
                                state.camera.center_mm -= state.camera.screen_delta_to_board_mm(response.drag_delta());
                            }
                            if response.clicked() {
                                let pad_id = hover_board.and_then(|p| state.doc.pad_at(p));
                                let unassign = ui.input(|i| i.modifiers.shift);
                                state.handle_connect_click(pad_id, unassign);
                            }
                        }
                        Tool::Route => {
                            if response.dragged() && state.routing.is_none() {
                                state.camera.center_mm -= state.camera.screen_delta_to_board_mm(response.drag_delta());
                            }
                            if let (Some(routing), Some(board_pos)) = (&mut state.routing, hover_board) {
                                routing.update(&state.doc, board_pos);
                            }
                            if response.clicked() {
                                if let Some(board_pos) = hover_board {
                                    state.handle_route_click(board_pos);
                                } else {
                                    state.routing = None;
                                }
                            }
                        }
                        Tool::PlaceVia => {
                            if response.dragged() {
                                state.camera.center_mm -= state.camera.screen_delta_to_board_mm(response.drag_delta());
                            }
                            if response.clicked() {
                                if let Some(board_pos) = hover_board {
                                    state.handle_place_via_click(board_pos);
                                }
                            }
                        }
                        Tool::DrawZone => {
                            if response.dragged() {
                                state.camera.center_mm -= state.camera.screen_delta_to_board_mm(response.drag_delta());
                            }
                            if response.clicked() {
                                if let Some(board_pos) = hover_board {
                                    state.handle_draw_zone_click(board_pos);
                                }
                            }
                        }
                        }
                    }

                    let scroll = ui.input(|i| i.smooth_scroll_delta.y);
                    if scroll != 0.0 && response.hovered() {
                        state.camera.zoom_by((1.0 + scroll * 0.001).clamp(0.5, 2.0));
                    }

                    let painter = ui.painter_at(rect);
                    painter.rect_filled(rect, 0.0, CANVAS_BACKGROUND);
                    draw_board_substrate(&painter, rect, &state.camera, &state.doc.outline);
                    if state.grid_snap_enabled {
                        draw_placement_grid(&painter, rect, &state.camera, state.grid_spacing);
                    }

                    let mut hidden_ids: Vec<ItemId> = state
                        .dragging
                        .as_ref()
                        .and_then(|d| state.doc.footprints.iter().find(|f| f.id == d.id))
                        .map(|f| f.pad_item_ids.clone())
                        .unwrap_or_default();
                    if let Some(drag) = &state.trace_dragging {
                        hidden_ids.extend_from_slice(drag.removed_ids());
                    }
                    let items: Vec<Item> = state
                        .doc
                        .node
                        .iter_with_ids()
                        .filter(|(id, _)| !hidden_ids.contains(id))
                        .map(|(_, item)| item.clone())
                        .collect();
                    // Pads are drawn by `draw_footprint_details` below instead
                    // of generically here, so every pad gets its *real*
                    // shape/number/pin-1 marker rather than a plain circle --
                    // see that function's doc comment.
                    let board_layers = LayerToggles { pads: false, ..state.layers };
                    alladin_render::draw_board(&painter, rect, &state.camera, &state.doc.outline, &items, &board_layers, state.highlighted_net);
                    let dragging_id = state.dragging.as_ref().map(|d| d.id);
                    draw_footprint_details(&painter, rect, &state.camera, &state.doc, &state.templates, &state.layers, dragging_id, state.highlighted_net);
                    // The one currently being dragged (if any) is
                    // skipped here -- it's drawn instead by the
                    // red/green `draw_silk_text_ghost` below, following
                    // the live cursor, same "hide the real thing, show
                    // only the ghost" convention `hidden_ids`/`dragging_id`
                    // above already use for a footprint mid-drag.
                    let dragging_silk_text_id = state.silk_text_dragging.as_ref().map(|d| d.id);
                    for text in &state.doc.silk_texts {
                        if Some(text.id) != dragging_silk_text_id {
                            draw_silk_text(&painter, rect, &state.camera, text);
                        }
                    }
                    let dragging_silk_dot_id = state.silk_dot_dragging.as_ref().map(|d| d.id);
                    for dot in &state.doc.silk_dots {
                        if Some(dot.id) != dragging_silk_dot_id {
                            draw_silk_dot_circle(&painter, rect, &state.camera, &dot.circle(), Color32::from_rgb(220, 220, 220));
                        }
                    }
                    // Pin-1 markers: same silk-white ink as free dots --
                    // on the fabricated board they're indistinguishable,
                    // so the preview doesn't invent a difference either.
                    for fp in &state.doc.footprints {
                        if let Some(circle) = fp.pin1_marker_circle() {
                            draw_silk_dot_circle(&painter, rect, &state.camera, &circle, Color32::from_rgb(220, 220, 220));
                        }
                    }
                    if state.show_ratsnest {
                        draw_ratsnest(&painter, rect, &state.camera, &state.doc);
                    }

                    if let Some(pad_id) = state.pending_connect {
                        draw_pending_pin(&painter, rect, &state.camera, &state.doc.node, pad_id);
                    }
                    if let Some(routing) = &state.routing {
                        draw_routing_preview(&painter, rect, &state.camera, &state.doc, routing);
                    }
                    if let Tool::DrawZone = state.tool {
                        draw_zone_preview(&painter, rect, &state.camera, &state.zone_points, hover_board);
                    }
                    if let Some(drag) = &state.trace_dragging {
                        draw_trace_drag_preview(&painter, rect, &state.camera, drag);
                    }

                    if let Some(id) = state.selected {
                        if state.dragging.is_none() {
                            if let Some(fp) = state.doc.footprints.iter().find(|f| f.id == id) {
                                let ring_ids: Vec<ItemId> = fp.pad_item_ids.iter().chain(&fp.hole_item_ids).copied().collect();
                                draw_selection_ring(&painter, rect, &state.camera, &state.doc.node, &ring_ids);
                            }
                        }
                    }
                    if let Some(id) = state.selected_item {
                        if state.trace_dragging.is_none() {
                            draw_item_selection_highlight(&painter, rect, &state.camera, &state.doc, id);
                        }
                    }
                    if let Some(id) = state.selected_silk_text {
                        if state.silk_text_dragging.is_none() {
                            if let Some(text) = state.doc.silk_texts.iter().find(|t| t.id == id) {
                                let points = silk_text_outline_px(rect, &state.camera, text);
                                painter.add(egui::Shape::closed_line(points, Stroke::new(2.0, Color32::from_rgb(255, 220, 0))));
                            }
                        }
                    }
                    if let Some(id) = state.selected_silk_dot {
                        if state.silk_dot_dragging.is_none() {
                            if let Some(dot) = state.doc.silk_dots.iter().find(|d| d.id == id) {
                                let center = state.camera.board_to_screen(rect, dot.position);
                                let radius_px = (dot.diameter as f32 / 2.0 / MM as f32 * state.camera.pixels_per_mm).max(1.5) + 3.0;
                                painter.circle_stroke(center, radius_px, Stroke::new(2.0, Color32::from_rgb(255, 220, 0)));
                            }
                        }
                    }

                    if let Tool::Place(i) = state.tool {
                        if let Some(board_pos) = hover_board {
                            let board_pos = snap_to_grid_point(board_pos, state.grid_spacing, state.grid_snap_enabled);
                            let template = &state.templates[i];
                            if state.matrix_rows.max(1) * state.matrix_cols.max(1) > 1 {
                                let (center, snap_x, snap_y) = state.snap_matrix_center(board_pos);
                                let positions = state.matrix_ghost_positions(center);
                                let valid = state.doc.check_matrix_placement(template, &positions, state.place_rotation_deg).is_ok();
                                let ghost_items: Vec<Item> =
                                    positions.iter().flat_map(|&p| world_items(template, p, state.place_rotation_deg)).collect();
                                draw_ghost(&painter, rect, &state.camera, &ghost_items, valid);
                                if let Some(bounds) = state.board_bounds() {
                                    draw_matrix_snap_guides(&painter, rect, &state.camera, bounds, snap_x, snap_y);
                                }
                            } else {
                                let ghost_items = world_items(template, board_pos, state.place_rotation_deg);
                                let valid = state.doc.check_placement(template, board_pos, state.place_rotation_deg, None).is_ok();
                                draw_ghost(&painter, rect, &state.camera, &ghost_items, valid);
                            }
                        }
                    }
                    if let Tool::PlaceSilkText = state.tool {
                        if let Some(board_pos) = hover_board {
                            let board_pos = snap_to_grid_point(board_pos, state.grid_spacing, state.grid_snap_enabled);
                            let ghost = crate::board_doc::SilkText {
                                id: crate::board_doc::SilkTextId(0),
                                text: if state.silk_text_input.trim().is_empty() { "?".to_string() } else { state.silk_text_input.clone() },
                                position: board_pos,
                                rotation_deg: state.silk_text_place_rotation_deg,
                                layer: state.silk_layer,
                                height: state.silk_text_height,
                                line_width: crate::board_doc::DEFAULT_SILK_LINE_WIDTH,
                            };
                            let valid = !state.silk_text_input.trim().is_empty()
                                && state
                                    .doc
                                    .check_silk_text_placement(&state.silk_text_input, board_pos, state.silk_text_place_rotation_deg, state.silk_layer, state.silk_text_height)
                                    .is_ok();
                            draw_silk_text_ghost(&painter, rect, &state.camera, &ghost, valid);
                        }
                    }
                    if let Tool::PlaceSilkDot = state.tool {
                        if let Some(board_pos) = hover_board {
                            let board_pos = snap_to_grid_point(board_pos, state.grid_spacing, state.grid_snap_enabled);
                            let circle = alladin_geom::Circle::new(board_pos, state.silk_dot_diameter / 2);
                            let valid = state.doc.check_silk_dot_placement(board_pos, state.silk_dot_diameter, state.silk_layer).is_ok();
                            draw_silk_dot_ghost(&painter, rect, &state.camera, &circle, valid);
                        }
                    }
                    if let Some(dragging) = &state.dragging {
                        let template = &state.templates[dragging.template_index];
                        let ghost_items = world_items(template, dragging.candidate_position, dragging.rotation_deg);
                        draw_ghost(&painter, rect, &state.camera, &ghost_items, dragging.valid);
                    }
                    if let Some(drag) = &state.silk_text_dragging {
                        if let Some(original) = state.doc.silk_texts.iter().find(|t| t.id == drag.id) {
                            let ghost = crate::board_doc::SilkText {
                                id: drag.id,
                                text: original.text.clone(),
                                position: drag.candidate_position,
                                rotation_deg: drag.rotation_deg,
                                layer: original.layer,
                                height: original.height,
                                line_width: original.line_width,
                            };
                            draw_silk_text_ghost(&painter, rect, &state.camera, &ghost, drag.valid);
                        }
                    }
                    if let Some(drag) = &state.silk_dot_dragging {
                        if let Some(original) = state.doc.silk_dots.iter().find(|d| d.id == drag.id) {
                            let circle = alladin_geom::Circle::new(drag.candidate_position, original.diameter / 2);
                            draw_silk_dot_ghost(&painter, rect, &state.camera, &circle, drag.valid);
                        }
                    }
                    if let Some(pending) = &state.pending_pin_via {
                        let template = &state.templates[pending.template_index];
                        let mut ghost_items = world_items(template, pending.candidate_position, pending.rotation_deg);
                        let via_center = pending.candidate_position.add(pending.via_offset);
                        ghost_items.push(Item::Pad {
                            shape: PadShape::Circle(alladin_geom::Circle::new(via_center, pending.diameter / 2)),
                            net: None,
                            layer: LayerId::FCu,
                        });
                        draw_ghost(&painter, rect, &state.camera, &ghost_items, pending.valid);
                    }
                });

                // Neither delete button removes anything itself
                // anymore -- it only stages the request here, and
                // `draw_delete_confirmation_window` (below) is what
                // actually calls `self.parts_db.delete_part`/
                // `delete_category_tree` once the user explicitly
                // confirms. See [`PendingDelete`]'s own doc comment.
                if let Some((index, db_id, name)) = delete_part_requested {
                    state.pending_delete = Some(PendingDelete::Part { index, db_id, name });
                }
                if let Some((prefix, count)) = delete_category_requested {
                    state.pending_delete = Some(PendingDelete::Category { prefix, count });
                }
                if create_part_requested {
                    if let Some(form) = state.add_part_form.take() {
                        let template =
                            footprint::straight_row_template(form.name.clone(), form.reference_prefix.clone(), form.pin_count, form.pitch_mm as f64, form.pad_radius_mm as f64);
                        let category = (!form.category.trim().is_empty()).then(|| form.category.trim().to_string());
                        match self.parts_db.insert_part_categorized(
                            &form.name,
                            &form.reference_prefix,
                            &form.description,
                            None,
                            &template.pads,
                            &[],
                            false,
                            None,
                            category.as_deref(),
                        ) {
                            Ok(record) => {
                                state.templates.push(record.template);
                                state.template_origin.push(Some(record.id));
                                state.template_hover.push(if form.description.is_empty() { None } else { Some(form.description.clone()) });
                                state.template_category.push(record.category);
                            }
                            Err(e) => state.io_message = Some(format!("Couldn't save part: {e}")),
                        }
                    }
                }

                if open_requested {
                    if let Some(path) = board_file_dialog().pick_file() {
                        // Backgrounded via `self.board_load`, same as
                        // the launch-time auto-open (see
                        // [`PendingBoardLoad::start`]'s doc comment) --
                        // the currently-open board (`state`, this same
                        // `Screen::Editor`) is left fully intact and
                        // usable until the load resolves, which
                        // [`PcbApp::ui`]'s own poll of `board_load`
                        // then either replaces it with, or -- on
                        // failure -- reports onto via `io_message`,
                        // exactly like the old synchronous version did
                        // either way.
                        state.io_message = Some("Opening board...".to_string());
                        self.board_load = Some(PendingBoardLoad::start(path, &self.parts_db));
                    }
                }
                if save_requested || save_as_requested {
                    let path = if save_as_requested { None } else { state.file_path.clone() };
                    let path = path.or_else(|| board_file_dialog().save_file());
                    if let Some(path) = path {
                        match save_to_path(&state.doc, &path) {
                            Ok(()) => {
                                remember_last_board(&path);
                                state.set_file_path(path);
                                state.io_message = None;
                            }
                            Err(e) => state.io_message = Some(format!("Couldn't save board: {e}")),
                        }
                    }
                }

                if export_manufacturing_requested {
                    if let Some(dir) = rfd::FileDialog::new().pick_folder() {
                        state.export_manufacturing_files_in_background(dir, &self.parts_db);
                    }
                }

                if new_board_requested {
                    self.screen = Screen::NewBoard(NewBoardParams::default());
                }
            }
        }
    }
}

/// Answers one [`crate::mcp::McpQuery`] by building the JSON `crate::mcp`
/// asked for and firing it through that query's own reply channel. Lives
/// here, not in `crate::mcp` itself, because it needs direct access to
/// `Screen`/`EditorState`'s otherwise-private fields, which `crate::mcp`
/// deliberately knows nothing about (see that module's doc comment).
/// Ignores a failed send -- the HTTP request that wanted the answer
/// already timed out and hung up (see [`crate::mcp::spawn_server`]'s
/// `REPLY_TIMEOUT`) -- there's no meaningful way to react to that here.
/// Builds a throwaway [`Screen`] carrying a *clone* of `screen`'s own
/// board/templates -- for a background job that needs to run an
/// existing `&Screen`-taking function (like [`net_continuity_json`]/
/// [`export_manufacturing_files_write`]/[`run_batch_write`]) off the UI
/// thread, without that function itself needing a different signature.
/// `Screen::NewBoard` snapshots as another, fresh `Screen::NewBoard` --
/// there's no board to clone yet, and every caller here already treats
/// "no board open" as its own up-front, un-backgrounded error case
/// before ever reaching this. The returned `EditorState` is otherwise a
/// brand new one (default camera/tool/selection/...): nothing here ever
/// looks at those UI-only fields, only `doc`/`templates`/`file_path`
/// (see [`merge_screen_mutation`] for the other half of this, which
/// deliberately copies back only what a background write can actually
/// change, never touching the *live* `EditorState`'s own UI state).
fn snapshot_screen_for_background(screen: &Screen) -> Screen {
    let Screen::Editor(state) = screen else {
        return Screen::NewBoard(NewBoardParams::default());
    };
    let mut scratch = EditorState::new(
        state.doc.clone(),
        state.templates.clone(),
        state.template_origin.clone(),
        state.template_hover.clone(),
        state.template_category.clone(),
    );
    if let Some(path) = &state.file_path {
        scratch.set_file_path(path.clone());
    }
    Screen::Editor(scratch)
}

/// Copies back onto the *live* `screen` exactly the fields a background
/// job running against a [`snapshot_screen_for_background`] clone could
/// actually have changed (`doc`, `file_path`/`disk_mtime`, an
/// in-progress `routing` drag) -- never the live `EditorState`'s own
/// UI-only state (camera, tool, selection, ...), which this never
/// touches. If the live screen is still `Screen::NewBoard` (nothing was
/// open when the job started) and the finished job created a board
/// (`RunBatch`'s `create_board` op), the finished `Screen::Editor`
/// simply becomes the live screen outright -- there's no live
/// `EditorState` state to preserve in that case since none existed yet.
fn merge_screen_mutation(screen: &mut Screen, finished: Screen) {
    match (screen, finished) {
        (Screen::Editor(live), Screen::Editor(finished)) => {
            live.doc = finished.doc;
            live.file_path = finished.file_path;
            live.disk_mtime = finished.disk_mtime;
            live.routing = finished.routing;
        }
        (live @ Screen::NewBoard(_), finished @ Screen::Editor(_)) => *live = finished,
        // The job never created/found a board (every op that would
        // have failed, or ran zero operations) -- nothing to merge.
        _ => {}
    }
}

/// Tries to start a background job for `query`, matching this
/// project's "Background heavy computations" plan (development log):
/// zone fill/refill, a routing search, net-continuity, manufacturing
/// export, and `run_batch`. Every other -- fast, bounded-cost -- write,
/// and every read-only query, is hardly worth a whole background thread
/// for and gets handed back unchanged (`Err`) for [`handle_mcp_query`]
/// to answer exactly as before.
fn try_start_background_job(query: crate::mcp::McpQuery, screen: &mut Screen, parts_db: &PartsDb) -> Result<PendingJob, crate::mcp::McpQuery> {
    use crate::mcp::McpQuery;
    match query {
        McpQuery::AddZone { args, reply } => Ok(spawn_add_zone_job(screen, args, reply)),
        McpQuery::RefillZones { reply } => Ok(spawn_refill_zones_job(screen, reply)),
        McpQuery::RoutePins { args, reply } => Ok(spawn_route_pins_job(screen, args, reply)),
        McpQuery::CheckNetContinuity { args, reply } => Ok(spawn_check_net_continuity_job(screen, args, reply)),
        McpQuery::ExportManufacturingFiles { args, reply } => Ok(spawn_export_manufacturing_files_job(screen, parts_db, args, reply)),
        McpQuery::RunBatch { args, reply } => Ok(spawn_run_batch_job(screen, parts_db, args, reply)),
        other => Err(other),
    }
}

/// [`crate::mcp::McpQuery::AddZone`]'s background dispatch -- see
/// [`BoardDoc::insert_new_zone`]'s doc comment for the split this
/// relies on. Every validation [`add_zone_write`] does up front
/// (unknown layer/empty points/unknown net) is repeated here
/// synchronously, so a bad call is refused immediately rather than only
/// after a pointless background round-trip; only the actual
/// `zone_fill::fill_zone` call runs on the [`BackgroundJob`].
fn spawn_add_zone_job(screen: &mut Screen, args: crate::mcp::AddZoneArgs, reply: oneshot::Sender<String>) -> PendingJob {
    let Screen::Editor(state) = screen else {
        return PendingJob::immediate("zone fill", no_board_open_json_error().to_string(), reply);
    };
    let layer = match args.layer.as_str() {
        "front" => LayerId::FCu,
        "back" => LayerId::BCu,
        other => return PendingJob::immediate("zone fill", error_json(format!("invalid layer \"{other}\" -- must be \"front\" or \"back\"")).to_string(), reply),
    };
    if args.points.is_empty() {
        return PendingJob::immediate("zone fill", error_json("points must contain at least one point").to_string(), reply);
    }
    let Some(net) = state.doc.find_net_by_name(&args.net) else {
        return PendingJob::immediate(
            "zone fill",
            error_json(format!("unknown net \"{}\" -- only a net already created by a prior connect_pins call can be targeted", args.net)).to_string(),
            reply,
        );
    };
    let net_name = args.net;
    let layer_name = args.layer;
    let outline = Polygon::new(args.points.iter().map(|p| Point::new(mm_arg(p.x_mm), mm_arg(p.y_mm))).collect());
    let node = state.doc.node.clone();
    let board_outline = state.doc.outline.clone();
    let resolver = state.doc.resolver();
    let filled_at_revision = node.obstacle_revision();
    let outline_for_fill = outline.clone();
    let job = BackgroundJob::spawn(move || -> JobResult {
        let items = zone_fill::fill_zone(&outline_for_fill, layer, net, &board_outline, &node, resolver);
        Box::new(move |screen: &mut Screen| {
            let Screen::Editor(state) = screen else {
                return no_board_open_json_error().to_string();
            };
            let zone_id = state.doc.insert_new_zone(outline, layer, net, items, filled_at_revision);
            let island_count = state.doc.zones.iter().find(|z| z.id == zone_id).map(|z| z.item_ids.len()).unwrap_or(0);
            serde_json::json!({ "ok": true, "zone_id": zone_id.0, "net": net_name, "layer": layer_name, "filled_islands": island_count }).to_string()
        })
    });
    PendingJob { label: "zone fill", job, reply: Some(reply) }
}

/// [`crate::mcp::McpQuery::RefillZones`]'s background dispatch -- see
/// [`BoardDoc::clear_zone_fill`]/[`BoardDoc::insert_zone_refill`]'s doc
/// comments for the split this mirrors (clear every zone's old fill
/// synchronously, recompute every one in the background, insert each
/// result once ready), same shape as
/// [`EditorState::refill_all_zones_in_background`]'s GUI equivalent.
fn spawn_refill_zones_job(screen: &mut Screen, reply: oneshot::Sender<String>) -> PendingJob {
    let Screen::Editor(state) = screen else {
        return PendingJob::immediate("zone refill", no_board_open_json_error().to_string(), reply);
    };
    let zones: Vec<(ZoneId, Polygon, LayerId, NetId)> = state.doc.zones.iter().map(|z| (z.id, z.outline.clone(), z.layer, z.net)).collect();
    for (id, ..) in &zones {
        state.doc.clear_zone_fill(*id);
    }
    let node = state.doc.node.clone();
    let board_outline = state.doc.outline.clone();
    let resolver = state.doc.resolver();
    let filled_at_revision = node.obstacle_revision();
    let job = BackgroundJob::spawn(move || -> JobResult {
        let results: Vec<(ZoneId, Vec<Item>)> = zones
            .into_iter()
            .map(|(id, outline, layer, net)| (id, zone_fill::fill_zone(&outline, layer, net, &board_outline, &node, resolver)))
            .collect();
        Box::new(move |screen: &mut Screen| {
            let Screen::Editor(state) = screen else {
                return no_board_open_json_error().to_string();
            };
            for (id, items) in results {
                state.doc.insert_zone_refill(id, items, filled_at_revision);
            }
            serde_json::json!({ "ok": true, "zone_count": state.doc.zones.len() }).to_string()
        })
    });
    PendingJob { label: "zone refill", job, reply: Some(reply) }
}

/// [`crate::mcp::McpQuery::RoutePins`]'s background dispatch -- see
/// [`BoardDoc::route_plan`]/[`BoardDoc::path_still_valid`]'s doc
/// comments: every validation [`route_pins_write`] does up front (pin
/// lookup, shared-net/shared-layer check) runs synchronously here too,
/// but the actual `alladin_router::route_single_net` search runs on the
/// [`BackgroundJob`], and its result is re-validated against the *live*
/// board -- not just inserted on trust -- once that resolves, per this
/// project's "Background heavy computations" plan (development log).
fn spawn_route_pins_job(screen: &Screen, args: crate::mcp::RoutePinsArgs, reply: oneshot::Sender<String>) -> PendingJob {
    let Screen::Editor(state) = screen else {
        return PendingJob::immediate("routing search", no_board_open_json_error().to_string(), reply);
    };
    let Some(a) = state.doc.find_pad(&state.templates, &args.ref1, &args.pin1) else {
        return PendingJob::immediate("routing search", error_json(format!("no such pin: {} pin {}", args.ref1, args.pin1)).to_string(), reply);
    };
    let Some(b) = state.doc.find_pad(&state.templates, &args.ref2, &args.pin2) else {
        return PendingJob::immediate("routing search", error_json(format!("no such pin: {} pin {}", args.ref2, args.pin2)).to_string(), reply);
    };
    let plan = match state.doc.route_plan(a, b) {
        Ok(plan) => plan,
        Err(e) => {
            return PendingJob::immediate(
                "routing search",
                error_json(format!("couldn't route {}.{} -- {}.{}: {e}", args.ref1, args.pin1, args.ref2, args.pin2)).to_string(),
                reply,
            )
        }
    };
    let width = mm_arg(args.width_mm);
    let node = state.doc.node.clone();
    let outline = state.doc.outline.clone();
    let resolver = state.doc.resolver();
    let (ref1, pin1, ref2, pin2) = (args.ref1, args.pin1, args.ref2, args.pin2);
    let job = BackgroundJob::spawn(move || -> JobResult {
        let path = route_single_net(&node, plan.from, plan.to, width, plan.net, plan.layer, NetClass::C, resolver, &outline)
            .filter(|path| crate::routing::path_keeps_edge_clearance(path, width, &outline));
        Box::new(move |screen: &mut Screen| {
            let Screen::Editor(state) = screen else {
                return no_board_open_json_error().to_string();
            };
            let Some(path) = path else {
                return error_json(format!("couldn't route {ref1}.{pin1} -- {ref2}.{pin2}: {}", RouteError::NoDrcClearPath)).to_string();
            };
            if !state.doc.path_still_valid(&path, &plan, width) {
                return error_json(format!("couldn't route {ref1}.{pin1} -- {ref2}.{pin2}: {}", RouteError::BoardChangedDuringSearch)).to_string();
            }
            let segment_count = path.len() - 1;
            state.doc.add_track_path(&path, plan.net, plan.layer, width, NetClass::C);
            serde_json::json!({ "ok": true, "track_leg_count": segment_count }).to_string()
        })
    });
    PendingJob { label: "routing search", job, reply: Some(reply) }
}

/// [`crate::mcp::McpQuery::CheckNetContinuity`]'s background dispatch --
/// purely read-only w.r.t. `screen` (see [`net_continuity_json`]), so
/// this just runs that same, unmodified function against a
/// [`snapshot_screen_for_background`] clone on a [`BackgroundJob`] and
/// replies with its result; there's nothing to merge back.
fn spawn_check_net_continuity_job(screen: &Screen, args: crate::mcp::CheckNetContinuityArgs, reply: oneshot::Sender<String>) -> PendingJob {
    let scratch = snapshot_screen_for_background(screen);
    let job = BackgroundJob::spawn(move || -> JobResult {
        let text = net_continuity_json(&scratch, args).to_string();
        Box::new(move |_screen: &mut Screen| text)
    });
    PendingJob { label: "net continuity check", job, reply: Some(reply) }
}

/// [`crate::mcp::McpQuery::ExportManufacturingFiles`]'s background
/// dispatch -- same "run the unmodified function against a clone,
/// there's nothing to merge back" shape as
/// [`spawn_check_net_continuity_job`]: [`export_manufacturing_files_write`]
/// only ever reads `screen`, writing manufacturing files via the native
/// Gerber path (see that function's own doc comment).
fn spawn_export_manufacturing_files_job(
    screen: &Screen,
    parts_db: &PartsDb,
    args: crate::mcp::ExportManufacturingFilesArgs,
    reply: oneshot::Sender<String>,
) -> PendingJob {
    let scratch = snapshot_screen_for_background(screen);
    let bom_csv = match &scratch {
        Screen::Editor(state) => crate::bom::to_csv(&crate::bom::build_bom_rows(&state.doc, &state.templates, &state.template_origin, parts_db)),
        Screen::NewBoard(_) => crate::bom::to_csv(&[]),
    };
    let job = BackgroundJob::spawn(move || -> JobResult {
        let text = export_manufacturing_files_write(&scratch, &bom_csv, args).to_string();
        Box::new(move |_screen: &mut Screen| text)
    });
    PendingJob { label: "manufacturing export", job, reply: Some(reply) }
}

/// [`crate::mcp::McpQuery::RunBatch`]'s background dispatch -- the one
/// case this project's "Background heavy computations" plan
/// (development log) calls for a *whole-screen* clone-and-swap instead of
/// a narrow "compute, then cheaply insert against the live board" split
/// (unlike zone fill/routing above): a batch is an arbitrary *sequence*
/// of dependent operations, several of which (`register_part`,
/// manufacturing export's BOM/LCSC lookups) also touch the parts database, so it
/// runs the entire existing, unmodified [`run_batch_write`] loop
/// against a [`snapshot_screen_for_background`] clone and a *second*,
/// freshly-opened [`PartsDb`] connection to the same on-disk file (or,
/// best-effort, an in-memory one if that somehow fails -- see
/// `crate::app::open_parts_db`'s own doc comment for the same
/// fallback), then [`merge_screen_mutation`]s the finished clone's
/// `doc`/`file_path`/`routing` back onto the live screen once done.
/// `run_batch_write` itself needs no changes at all for this.
fn spawn_run_batch_job(screen: &mut Screen, parts_db: &PartsDb, args: crate::mcp::RunBatchArgs, reply: oneshot::Sender<String>) -> PendingJob {
    let _ = parts_db; // only the live `PartsDb`'s existence matters here, not its contents -- the background job opens its own connection
    let mut scratch_screen = snapshot_screen_for_background(screen);
    // An interactive GUI route already in progress on the live board
    // moves into the scratch for the batch's own duration (rather than
    // being cloned -- `RoutingDrag` isn't `Clone`, see its own
    // `dock_job` field) so a batch containing its own `route_to`/
    // `finish_route` steps (without a `start_route` of its own) can
    // still continue it, exactly like a synchronous call would have;
    // moved back by `merge_screen_mutation` once the batch finishes.
    if let (Screen::Editor(live), Screen::Editor(scratch)) = (screen, &mut scratch_screen) {
        scratch.routing = live.routing.take();
    }
    let job = BackgroundJob::spawn(move || -> JobResult {
        let mut scratch_parts_db =
            PartsDb::open(&PartsDb::default_path()).or_else(|_| PartsDb::open_in_memory()).expect("an in-memory sqlite database must always succeed");
        let text = run_batch_write(&mut scratch_screen, &mut scratch_parts_db, args).to_string();
        Box::new(move |screen: &mut Screen| {
            merge_screen_mutation(screen, scratch_screen);
            text
        })
    });
    PendingJob { label: "batch operation", job, reply: Some(reply) }
}

fn handle_mcp_query(query: crate::mcp::McpQuery, screen: &mut Screen, parts_db: &mut PartsDb) {
    use crate::mcp::McpQuery;
    match query {
        McpQuery::EditorState { reply } => {
            let _ = reply.send(editor_state_json(screen).to_string());
        }
        McpQuery::BoardOverview { reply } => {
            let _ = reply.send(board_overview_json(screen).to_string());
        }
        McpQuery::Nets { reply } => {
            let _ = reply.send(nets_json(screen).to_string());
        }
        McpQuery::Zones { reply } => {
            let _ = reply.send(zones_json(screen).to_string());
        }
        McpQuery::Footprints { reply } => {
            let _ = reply.send(footprints_json(screen).to_string());
        }
        McpQuery::CheckNetContinuity { args, reply } => {
            let _ = reply.send(net_continuity_json(screen, args).to_string());
        }
        McpQuery::CreateBoard { args, reply } => {
            let _ = reply.send(create_board_write(screen, parts_db, args).to_string());
        }
        McpQuery::PlaceFootprint { args, reply } => {
            let _ = reply.send(place_footprint_write(screen, args).to_string());
        }
        McpQuery::DownloadLcscPart { fetched, reply } => {
            let _ = reply.send(download_lcsc_part_write(screen, parts_db, fetched).to_string());
        }
        McpQuery::RegisterPart { args, reply } => {
            let _ = reply.send(register_part_write(screen, parts_db, args).to_string());
        }
        McpQuery::ConnectPins { args, reply } => {
            let _ = reply.send(connect_pins_write(screen, args).to_string());
        }
        McpQuery::RoutePins { args, reply } => {
            let _ = reply.send(route_pins_write(screen, args).to_string());
        }
        McpQuery::StartRoute { args, reply } => {
            let _ = reply.send(start_route_write(screen, args).to_string());
        }
        McpQuery::RouteTo { args, reply } => {
            let _ = reply.send(route_to_write(screen, args).to_string());
        }
        McpQuery::FixCorner { reply } => {
            let _ = reply.send(fix_corner_write(screen).to_string());
        }
        McpQuery::UndoLastCorner { reply } => {
            let _ = reply.send(undo_last_corner_write(screen).to_string());
        }
        McpQuery::FinishRoute { reply } => {
            let _ = reply.send(finish_route_write(screen).to_string());
        }
        McpQuery::CancelRoute { reply } => {
            let _ = reply.send(cancel_route_write(screen).to_string());
        }
        McpQuery::DropViaAndSwitchLayer { reply } => {
            let _ = reply.send(drop_via_and_switch_layer_write(screen).to_string());
        }
        McpQuery::AddVia { args, reply } => {
            let _ = reply.send(add_via_write(screen, args).to_string());
        }
        McpQuery::AddPinStitchingVia { args, reply } => {
            let _ = reply.send(add_pin_stitching_via_write(screen, args).to_string());
        }
        McpQuery::AddZone { args, reply } => {
            let _ = reply.send(add_zone_write(screen, args).to_string());
        }
        McpQuery::AddSilkText { args, reply } => {
            let _ = reply.send(add_silk_text_write(screen, args).to_string());
        }
        McpQuery::RefillZones { reply } => {
            let _ = reply.send(refill_zones_write(screen).to_string());
        }
        McpQuery::RenameNet { args, reply } => {
            let _ = reply.send(rename_net_write(screen, args).to_string());
        }
        McpQuery::SaveBoard { args, reply } => {
            let _ = reply.send(save_board_write(screen, args).to_string());
        }
        McpQuery::ExportManufacturingFiles { args, reply } => {
            let bom_csv = match screen {
                Screen::Editor(state) => crate::bom::to_csv(&crate::bom::build_bom_rows(&state.doc, &state.templates, &state.template_origin, parts_db)),
                Screen::NewBoard(_) => crate::bom::to_csv(&[]),
            };
            let _ = reply.send(export_manufacturing_files_write(screen, &bom_csv, args).to_string());
        }
        McpQuery::RunBatch { args, reply } => {
            let _ = reply.send(run_batch_write(screen, parts_db, args).to_string());
        }
        McpQuery::StartExternalAutoroute { args, reply } => {
            let _ = reply.send(start_external_autoroute_write(screen, args).to_string());
        }
        McpQuery::GetExternalAutorouteStatus { reply } => {
            let _ = reply.send(external_autoroute_status_json(screen).to_string());
        }
    }
}

/// A short JSON `{ "note": ... }` every `*_json` builder below returns
/// verbatim while still on [`Screen::NewBoard`] -- there's no board yet
/// for any of them to describe.
fn no_board_open_json() -> serde_json::Value {
    serde_json::json!({ "note": "no board is open yet -- still on the New Board screen" })
}

/// `Tool::Place`'s carried template index aside, every variant is a bare
/// name -- shared by [`editor_state_json`] and its own tests.
fn tool_name(tool: Tool) -> &'static str {
    match tool {
        Tool::Select => "Select",
        Tool::Place(_) => "Place",
        Tool::Connect => "Connect",
        Tool::PlaceVia => "PlaceVia",
        Tool::Route => "Route",
        Tool::DrawZone => "DrawZone",
        Tool::PlaceSilkText => "PlaceSilkText",
        Tool::PlaceSilkDot => "PlaceSilkDot",
    }
}

fn point_mm_json(p: Point) -> serde_json::Value {
    serde_json::json!({ "x_mm": p.x as f64 / MM as f64, "y_mm": p.y as f64 / MM as f64 })
}

/// A net's human-readable name, or `"?"` for an id that (shouldn't, but
/// in principle could) no longer resolve -- same fallback
/// `crate::cli::format_zones` already uses.
fn net_name(doc: &BoardDoc, id: NetId) -> &str {
    doc.nets.iter().find(|n| n.id == id).map(|n| n.name.as_str()).unwrap_or("?")
}

/// Resolves a pad's `alladin_core::ItemId` back to the `{footprint,
/// pin}` pair a human (or an AI) actually thinks in terms of -- the same
/// "reference + pad number" shape `crate::cli`'s `connect`/`add-via`
/// commands take as input, just resolved in the other direction. `None`
/// if `pad_id` doesn't belong to any placed footprint (shouldn't happen
/// for a pad this module was handed, but every caller here treats it as
/// possible rather than unwrapping).
fn describe_pad(doc: &BoardDoc, templates: &[FootprintTemplate], pad_id: ItemId) -> Option<serde_json::Value> {
    let footprint = doc.footprints.iter().find(|f| f.pad_item_ids.contains(&pad_id))?;
    let index = footprint.pad_item_ids.iter().position(|&id| id == pad_id)?;
    let pad_template = templates.iter().find(|t| t.name == footprint.template_name).and_then(|t| t.pads.get(index));
    let pin = pad_template.map(|p| p.number.clone()).unwrap_or_else(|| (index + 1).to_string());
    let pin_name = pad_template.and_then(|p| p.pin_name.clone());
    Some(serde_json::json!({ "footprint": footprint.reference, "pin": pin, "pin_name": pin_name }))
}

/// The `pending_pin_via`-shaped sibling of the same blind spot
/// [`routing_json`] documents: while this is `Some`, the footprint
/// named here (plus its not-yet-placed via) is glued to the cursor
/// instead of behaving like an ordinary [`Tool::Select`] drag --
/// surfaced explicitly so a stuck one (e.g. after clicking away from
/// the GUI mid-relocation) is visible over MCP instead of just
/// silently eating every click as "still not a valid spot yet".
fn pending_pin_via_json(doc: &BoardDoc, templates: &[FootprintTemplate], pending: &PendingPinVia) -> serde_json::Value {
    serde_json::json!({
        "footprint": doc.footprints.iter().find(|f| f.id == pending.footprint_id).map(|f| f.reference.clone()),
        "pin": describe_pad(doc, templates, pending.pad_id),
        "net": net_name(doc, pending.net),
        "candidate_position_mm": point_mm_json(pending.candidate_position),
        "valid": pending.valid,
    })
}

/// The exact blind spot this whole module exists to fix -- an
/// in-progress [`RoutingDrag`] is invisible to the board file and the
/// CLI, since [`RoutingDrag::commit`] hasn't run yet.
fn routing_json(doc: &BoardDoc, templates: &[FootprintTemplate], routing: &RoutingDrag) -> serde_json::Value {
    let (live_end, live_clear) = routing.live_end();
    serde_json::json!({
        "net": net_name(doc, routing.net()),
        "from_pad": describe_pad(doc, templates, routing.from_pad),
        "origin_mm": point_mm_json(routing.origin()),
        "fixed_corner_count": routing.fixed_corner_count(),
        "live_end_point_count": live_end.len(),
        "live_end_clear": live_clear,
        "hover_target_pad": routing.hover_target.and_then(|id| describe_pad(doc, templates, id)),
        "edge_clearance_violation": routing.edge_clearance_violation,
    })
}

fn editor_state_json(screen: &Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    serde_json::json!({
        "screen": "Editor",
        "file_path": state.file_path.as_ref().map(|p| p.to_string_lossy().to_string()),
        "tool": tool_name(state.tool),
        "hover_board_mm": state.last_hover_board.map(point_mm_json),
        "selected_footprint": state.selected.and_then(|id| state.doc.footprints.iter().find(|f| f.id == id)).map(|f| f.reference.clone()),
        "selected_item_id": state.selected_item.map(|id| id.0),
        "highlighted_net": state.highlighted_net.map(|id| net_name(&state.doc, id).to_string()),
        "pending_connect_pin": state.pending_connect.and_then(|id| describe_pad(&state.doc, &state.templates, id)),
        "routing": state.routing.as_ref().map(|r| routing_json(&state.doc, &state.templates, r)),
        "pending_pin_via": state.pending_pin_via.as_ref().map(|p| pending_pin_via_json(&state.doc, &state.templates, p)),
        "draw_zone": (!state.zone_points.is_empty()).then(|| serde_json::json!({ "point_count": state.zone_points.len() })),
        "place_via": (state.via_net.is_some() || state.via_message.is_some()).then(|| {
            serde_json::json!({
                "net": state.via_net.map(|id| net_name(&state.doc, id).to_string()),
                "last_message": state.via_message,
            })
        }),
        "messages": {
            "net": state.net_message,
            "route": state.route_message,
            "via": state.via_message,
            "zone": state.zone_message,
            "io": state.io_message,
        },
    })
}

fn board_overview_json(screen: &Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    let doc = &state.doc;
    let mut min = Point::new(Unit::MAX, Unit::MAX);
    let mut max = Point::new(Unit::MIN, Unit::MIN);
    for polygon in &doc.outline {
        for p in &polygon.points {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
        }
    }
    let (width_mm, height_mm) = if doc.outline.is_empty() { (0.0, 0.0) } else { ((max.x - min.x) as f64 / MM as f64, (max.y - min.y) as f64 / MM as f64) };
    serde_json::json!({
        "file_path": state.file_path.as_ref().map(|p| p.to_string_lossy().to_string()),
        "width_mm": width_mm,
        "height_mm": height_mm,
        "layer_count": doc.layer_count.as_u8(),
        "copper_weight_oz": doc.copper_weight.as_oz(),
        "net_count": doc.nets.len(),
        "footprint_count": doc.footprints.len(),
        "track_count": doc.node.iter().filter(|i| matches!(i, Item::Track { .. })).count(),
        "via_count": doc.node.iter().filter(|i| matches!(i, Item::Via { .. })).count(),
        "zone_count": doc.zones.len(),
        "zones_stale": doc.zones_are_stale(),
    })
}

fn nets_json(screen: &Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    let nets: Vec<_> = state
        .doc
        .nets
        .iter()
        .map(|net| {
            let pads: Vec<_> = state.doc.pads_on_net(net.id).into_iter().filter_map(|id| describe_pad(&state.doc, &state.templates, id)).collect();
            serde_json::json!({ "id": net.id.0, "name": net.name, "pin_count": pads.len(), "pads": pads })
        })
        .collect();
    serde_json::json!({ "nets": nets })
}

fn zones_json(screen: &Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    let zones: Vec<_> = state
        .doc
        .zones
        .iter()
        .map(|zone| {
            serde_json::json!({
                "id": zone.id.0,
                "net": net_name(&state.doc, zone.net),
                "layer": match zone.layer {
                    LayerId::FCu => "F.Cu",
                    LayerId::BCu => "B.Cu",
                },
                "outline_points": zone.outline.points.len(),
                "filled_islands": zone.item_ids.len(),
            })
        })
        .collect();
    serde_json::json!({ "zones": zones })
}

/// One [`alladin_core::Node::net_copper_components`] group, resolved
/// into the shape a human/AI actually cares about: how many pads it
/// holds (with their `footprint.pin` names, via [`describe_pad`]) plus
/// bare counts for the non-pad item kinds (tracks/vias/zone islands) --
/// those don't have a stable "name" the way a pad does, so a count is
/// all a report needs to say "this group also has 3 tracks and a via
/// holding it together", not e.g. list each one's raw coordinates.
fn net_component_json(doc: &BoardDoc, templates: &[FootprintTemplate], ids: &[ItemId]) -> serde_json::Value {
    let mut pads = Vec::new();
    let mut track_count = 0usize;
    let mut via_count = 0usize;
    let mut zone_island_count = 0usize;
    for &id in ids {
        match doc.node.get(id) {
            Some(Item::Pad { .. }) => {
                if let Some(pad) = describe_pad(doc, templates, id) {
                    pads.push(pad);
                }
            }
            Some(Item::Track { .. }) => track_count += 1,
            Some(Item::Via { .. }) => via_count += 1,
            Some(Item::Zone { .. }) => zone_island_count += 1,
            Some(Item::Hole { .. }) | None => {}
        }
    }
    serde_json::json!({
        "pad_count": pads.len(),
        "pads": pads,
        "track_count": track_count,
        "via_count": via_count,
        "zone_island_count": zone_island_count,
    })
}

/// [`crate::mcp::AlladinMcp::check_net_continuity`]'s actual report
/// builder: for each net under consideration, groups its copper into
/// physically-connected components (see
/// [`alladin_core::Node::net_copper_components`]) and reports whether
/// that came out as exactly one group (fully connected) or more than
/// one (some of that net's pads aren't actually copper-reachable from
/// the rest, despite every one of them sharing the net's name/id).
///
/// A specific `net_name` always gets its full component breakdown,
/// connected or not, so a caller can double-check one net right after
/// working on it. With no filter, every net with fewer than two pads is
/// skipped outright (nothing to disconnect), and a fully-connected
/// multi-pad net is reported only by its counts in `summary` -- not
/// with its own (trivial, one-group) entry in `problem_nets` -- so a
/// whole-board sweep stays a short, readable "here's what still needs
/// attention" list instead of repeating good news for every net that's
/// already fine.
fn net_continuity_json(screen: &Screen, args: crate::mcp::CheckNetContinuityArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    let doc = &state.doc;

    let checked_nets: Vec<_> = doc
        .nets
        .iter()
        .filter(|net| args.net_name.as_deref().map_or(true, |name| name == net.name))
        .filter(|net| args.net_name.is_some() || doc.pads_on_net(net.id).len() >= 2)
        .collect();

    if checked_nets.is_empty() {
        if let Some(name) = &args.net_name {
            return serde_json::json!({ "error": format!("no net named {name:?} found") });
        }
        return serde_json::json!({
            "summary": { "nets_checked": 0, "nets_fully_connected": 0, "nets_with_gaps": 0 },
            "problem_nets": [],
        });
    }

    let mut nets_fully_connected = 0usize;
    let mut problem_nets = Vec::new();
    let mut single_net_report = None;

    for net in &checked_nets {
        let components = doc.node.net_copper_components(net.id);
        let fully_connected = components.len() <= 1;
        if fully_connected {
            nets_fully_connected += 1;
        }

        let report = || {
            let component_jsons: Vec<_> = components.iter().map(|ids| net_component_json(doc, &state.templates, ids)).collect();
            serde_json::json!({
                "id": net.id.0,
                "name": net.name,
                "component_count": components.len(),
                "fully_connected": fully_connected,
                "components": component_jsons,
            })
        };

        if args.net_name.is_some() {
            single_net_report = Some(report());
        } else if !fully_connected {
            problem_nets.push(report());
        }
    }

    if let Some(report) = single_net_report {
        return report;
    }

    serde_json::json!({
        "summary": {
            "nets_checked": checked_nets.len(),
            "nets_fully_connected": nets_fully_connected,
            "nets_with_gaps": checked_nets.len() - nets_fully_connected,
        },
        "problem_nets": problem_nets,
    })
}

fn footprints_json(screen: &Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    let footprints: Vec<_> = state
        .doc
        .footprints
        .iter()
        .map(|fp| {
            let template = state.templates.iter().find(|t| t.name == fp.template_name);
            let pads: Vec<_> = fp
                .pad_item_ids
                .iter()
                .enumerate()
                .map(|(index, &pad_id)| {
                    let pad_template = template.and_then(|t| t.pads.get(index));
                    let pin = pad_template.map(|p| p.number.clone()).unwrap_or_else(|| (index + 1).to_string());
                    let pin_name = pad_template.and_then(|p| p.pin_name.clone());
                    let net = state.doc.pad_net(pad_id).ok().flatten().map(|id| net_name(&state.doc, id).to_string());
                    serde_json::json!({ "pin": pin, "pin_name": pin_name, "net": net })
                })
                .collect();
            // The footprint's own real mechanical body/courtyard (see
            // `crate::footprint::FootprintTemplate::courtyard`) --
            // the template's own local, rotation-independent
            // width/height (not this *placement*'s rotated on-board
            // extent, which `crate::board_doc::PlacedFootprint::courtyard`
            // itself is), so an AI driving `place_footprint` over MCP
            // reads the same "this part's body is WxH" fact
            // regardless of which way it happens to be rotated on
            // the board right now.
            let courtyard = template.map(|t| t.courtyard()).map(|c| {
                serde_json::json!({ "width_mm": c.width as f64 / MM as f64, "height_mm": c.height as f64 / MM as f64 })
            });
            serde_json::json!({
                "reference": fp.reference,
                "template": fp.template_name,
                "x_mm": fp.position.x as f64 / MM as f64,
                "y_mm": fp.position.y as f64 / MM as f64,
                "rotation_deg": fp.rotation_deg,
                "pads": pads,
                "courtyard": courtyard,
            })
        })
        .collect();
    serde_json::json!({ "footprints": footprints })
}

/// `{ "error": message }` -- the one JSON shape every write handler
/// below returns on failure, so an MCP client can reliably check for an
/// `"error"` key regardless of which tool it called.
fn error_json(message: impl Into<String>) -> serde_json::Value {
    serde_json::json!({ "error": message.into() })
}

/// [`crate::mcp::McpQuery::CreateBoard`]'s handler. Only succeeds while
/// still on [`Screen::NewBoard`] -- deliberately refuses to ever
/// silently replace an already-open board (see `crate::mcp`'s module
/// doc comment), matching exactly what the GUI's own "New board" button
/// requires (navigate there first).
fn create_board_write(screen: &mut Screen, parts_db: &PartsDb, args: crate::mcp::CreateBoardArgs) -> serde_json::Value {
    if matches!(screen, Screen::Editor(_)) {
        return error_json("a board is already open -- close it (or use the GUI's \"New board...\" button) before creating another one over MCP");
    }
    let layer_count = match args.layers {
        1 => LayerCount::One,
        2 => LayerCount::Two,
        other => return error_json(format!("invalid layers={other} -- must be 1 or 2")),
    };
    let copper_weight = match args.copper_weight_oz {
        1 => CopperWeight::OneOz,
        2 => CopperWeight::TwoOz,
        other => return error_json(format!("invalid copper_weight_oz={other} -- must be 1 or 2")),
    };
    let params =
        NewBoardParams { width_mm: args.width_mm as f32, height_mm: args.height_mm as f32, layer_count, copper_weight, corner_radius_mm: args.corner_radius_mm as f32 };
    if !params.is_valid() {
        return error_json(format!(
            "invalid board: {}x{}mm with a {}mm corner radius isn't physically sane",
            args.width_mm, args.height_mm, args.corner_radius_mm
        ));
    }
    let doc = params.create();
    let (templates, template_origin, template_hover, template_category) = load_templates(parts_db);
    *screen = Screen::Editor(EditorState::new(doc, templates, template_origin, template_hover, template_category));
    serde_json::json!({ "ok": true, "width_mm": args.width_mm, "height_mm": args.height_mm, "layers": args.layers, "copper_weight_oz": args.copper_weight_oz })
}

/// [`crate::mcp::McpQuery::PlaceFootprint`]'s handler -- same
/// template-lookup and placement logic as `crate::cli`'s `place-part`,
/// operating on the live board instead of a file.
fn place_footprint_write(screen: &mut Screen, args: crate::mcp::PlaceFootprintArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(template) = state.templates.iter().find(|t| t.name == args.template) else {
        return error_json(format!("unknown template \"{}\" -- call get_footprints or list-templates to see what's available", args.template));
    };
    let position = Point::new(mm_arg(args.x_mm), mm_arg(args.y_mm));
    match state.doc.try_place_footprint(template, position, args.rotation_deg) {
        Ok(id) => {
            let reference = state.doc.footprints.iter().find(|f| f.id == id).expect("just-placed footprint must exist").reference.clone();
            serde_json::json!({ "ok": true, "reference": reference, "template": args.template, "x_mm": args.x_mm, "y_mm": args.y_mm, "rotation_deg": args.rotation_deg })
        }
        Err(e) => error_json(format!("couldn't place {}: {e}", args.template)),
    }
}

/// [`crate::mcp::McpQuery::DownloadLcscPart`]'s handler -- the network
/// fetch already happened on the MCP thread (see
/// `AlladinMcp::download_lcsc_part`); this just inserts the result into
/// the parts database and refreshes the live template list, same
/// insert/duplicate-handling logic as the GUI's own background-download
/// success handler, minus the GUI-only "select it for placing" side
/// effect (an MCP call shouldn't reach into the human's active tool).
fn download_lcsc_part_write(screen: &mut Screen, parts_db: &PartsDb, fetched: Result<crate::lcsc::FetchedPart, crate::lcsc::FetchError>) -> serde_json::Value {
    let part = match fetched {
        Ok(part) => part,
        Err(e) => return error_json(e.to_string()),
    };
    match parts_db.insert_part_categorized(
        &part.name,
        &part.reference_prefix,
        &part.description,
        Some(&part.lcsc_code),
        &part.pads,
        &[],
        false,
        part.explicit_courtyard,
        part.category.as_deref(),
    ) {
        Ok(record) => {
            let template_name = record.template.name.clone();
            if let Screen::Editor(state) = screen {
                let tooltip = format!("{}: {}", part.lcsc_code, part.description);
                state.templates.push(record.template);
                state.template_origin.push(Some(record.id));
                state.template_hover.push(Some(tooltip));
                state.template_category.push(record.category);
            }
            serde_json::json!({ "ok": true, "template": template_name, "lcsc_code": part.lcsc_code, "pad_count": part.pads.len() })
        }
        Err(crate::parts_db::PartsDbError::DuplicateLcscCode(code)) => {
            error_json(format!("{code} is already in your parts database -- use its existing template name with place_footprint"))
        }
        Err(e) => error_json(format!("downloaded, but couldn't save to the database: {e}")),
    }
}

/// [`crate::mcp::McpQuery::RegisterPart`]'s handler -- same
/// mutually-exclusive `pin_count`/`hole_diameter_mm` validation as
/// `crate::cli`'s `register-part`.
fn register_part_write(screen: &mut Screen, parts_db: &PartsDb, args: crate::mcp::RegisterPartArgs) -> serde_json::Value {
    let (pads, holes) = match (args.pin_count, args.hole_diameter_mm) {
        (Some(_), Some(_)) => return error_json("pin_count and hole_diameter_mm are mutually exclusive -- give exactly one"),
        (None, None) => return error_json("give exactly one of pin_count (a row of solder pads) or hole_diameter_mm (a mounting hole)"),
        (Some(pin_count), None) => {
            let template = footprint::straight_row_template(args.name.clone(), args.reference_prefix.clone(), pin_count, args.pitch_mm, args.pad_radius_mm);
            (template.pads, Vec::new())
        }
        (None, Some(hole_diameter_mm)) => (Vec::new(), vec![crate::footprint::HoleTemplate { offset: Point::new(0, 0), drill: mm_arg(hole_diameter_mm) }]),
    };
    let category = (!args.category.trim().is_empty()).then(|| args.category.trim().to_string());
    match parts_db.insert_part_categorized(&args.name, &args.reference_prefix, &args.description, None, &pads, &holes, args.exclude_from_bom, None, category.as_deref()) {
        Ok(record) => {
            let template_name = record.template.name.clone();
            if let Screen::Editor(state) = screen {
                state.templates.push(record.template);
                state.template_origin.push(Some(record.id));
                state.template_hover.push(Some(args.description.clone()));
                state.template_category.push(record.category);
            }
            serde_json::json!({ "ok": true, "template": template_name })
        }
        Err(e) => error_json(format!("couldn't register \"{}\": {e}", args.name)),
    }
}

/// [`crate::mcp::McpQuery::ConnectPins`]'s handler.
fn connect_pins_write(screen: &mut Screen, args: crate::mcp::ConnectPinsArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(a) = state.doc.find_pad(&state.templates, &args.ref1, &args.pin1) else {
        return error_json(format!("no such pin: {} pin {}", args.ref1, args.pin1));
    };
    let Some(b) = state.doc.find_pad(&state.templates, &args.ref2, &args.pin2) else {
        return error_json(format!("no such pin: {} pin {}", args.ref2, args.pin2));
    };
    match state.doc.connect_pads(a, b) {
        Ok(net) => {
            let name = net_name(&state.doc, net).to_string();
            serde_json::json!({ "ok": true, "net": name })
        }
        Err(e) => error_json(format!("couldn't connect {}.{} to {}.{}: {e}", args.ref1, args.pin1, args.ref2, args.pin2)),
    }
}

/// [`crate::mcp::McpQuery::RoutePins`]'s handler.
fn route_pins_write(screen: &mut Screen, args: crate::mcp::RoutePinsArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(a) = state.doc.find_pad(&state.templates, &args.ref1, &args.pin1) else {
        return error_json(format!("no such pin: {} pin {}", args.ref1, args.pin1));
    };
    let Some(b) = state.doc.find_pad(&state.templates, &args.ref2, &args.pin2) else {
        return error_json(format!("no such pin: {} pin {}", args.ref2, args.pin2));
    };
    match state.doc.try_route_pads_with_width(a, b, mm_arg(args.width_mm)) {
        Ok(leg_count) => serde_json::json!({ "ok": true, "track_leg_count": leg_count }),
        Err(e) => error_json(format!("couldn't route {}.{} -- {}.{}: {e}", args.ref1, args.pin1, args.ref2, args.pin2)),
    }
}

/// [`crate::mcp::McpQuery::StartRoute`]'s handler -- see
/// `crate::app::handle_route_click`'s `None` branch for the GUI mouse
/// click this mirrors. Refuses outright (never touching
/// `state.routing`) if a drag is already in progress, rather than
/// silently discarding it -- unlike a human's mouse, an MCP call has no
/// "click on empty space to abandon" gesture of its own, so overwriting
/// here would make a stuck/abandoned drag impossible to notice from the
/// caller's side.
fn start_route_write(screen: &mut Screen, args: crate::mcp::StartRouteArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    if state.routing.is_some() {
        return error_json("a route is already in progress -- call finish_route or cancel_route first");
    }
    let Some(pad_id) = state.doc.find_pad(&state.templates, &args.reference, &args.pin) else {
        return error_json(format!("no such pin: {} pin {}", args.reference, args.pin));
    };
    match RoutingDrag::start_with_options(&state.doc, pad_id, mm_arg(args.width_mm), mm_arg(args.via_diameter_mm), mm_arg(args.via_drill_mm)) {
        Some(drag) => {
            let info = routing_json(&state.doc, &state.templates, &drag);
            state.routing = Some(drag);
            let mut result = info;
            if let Some(obj) = result.as_object_mut() {
                obj.insert("ok".to_string(), serde_json::json!(true));
            }
            result
        }
        None => error_json("this pin has no net yet -- call connect_pins first"),
    }
}

/// [`crate::mcp::McpQuery::RouteTo`]'s handler -- always succeeds as a
/// call (see [`crate::mcp::AlladinMcp::route_to`]'s tool description);
/// `blocked_reason`/`live_end_clear` in the response are what tell the
/// caller whether the resulting leg is actually usable right now.
fn route_to_write(screen: &mut Screen, args: crate::mcp::RouteToArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(drag) = state.routing.as_mut() else {
        return error_json("no route in progress -- call start_route first");
    };
    let cursor = Point::new(mm_arg(args.x_mm), mm_arg(args.y_mm));
    drag.update(&state.doc, cursor);
    let mut result = routing_json(&state.doc, &state.templates, drag);
    let blocked = drag.blocked_reason(&state.doc, cursor);
    if let Some(obj) = result.as_object_mut() {
        obj.insert("ok".to_string(), serde_json::json!(true));
        obj.insert("blocked_reason".to_string(), serde_json::json!(blocked));
    }
    result
}

/// [`crate::mcp::McpQuery::FixCorner`]'s handler.
fn fix_corner_write(screen: &mut Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(drag) = state.routing.as_mut() else {
        return error_json("no route in progress -- call start_route first");
    };
    if drag.fix_corner() {
        let mut result = routing_json(&state.doc, &state.templates, drag);
        if let Some(obj) = result.as_object_mut() {
            obj.insert("ok".to_string(), serde_json::json!(true));
        }
        result
    } else {
        error_json("can't fix a corner here -- move towards a point first (route_to), the leg must be clear, and it must not be docked onto a pad (finish_route instead)")
    }
}

/// [`crate::mcp::McpQuery::UndoLastCorner`]'s handler.
fn undo_last_corner_write(screen: &mut Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(drag) = state.routing.as_mut() else {
        return error_json("no route in progress -- call start_route first");
    };
    if drag.undo_last_corner() {
        let mut result = routing_json(&state.doc, &state.templates, drag);
        if let Some(obj) = result.as_object_mut() {
            obj.insert("ok".to_string(), serde_json::json!(true));
        }
        result
    } else {
        error_json("no fixed corner to undo yet")
    }
}

/// [`crate::mcp::McpQuery::FinishRoute`]'s handler -- see
/// `crate::app::handle_route_click`'s final match arm for the GUI mouse
/// click this mirrors, including "keep the drag alive on failure so the
/// caller can keep steering" (`state.routing` is only left `None` on
/// success, matching that code exactly).
fn finish_route_write(screen: &mut Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(drag) = state.routing.take() else {
        return error_json("no route in progress -- call start_route first");
    };
    if drag.commit(&mut state.doc) {
        serde_json::json!({ "ok": true, "net": net_name(&state.doc, drag.net()) })
    } else {
        let reason = drag.blocked_reason(&state.doc, drag.origin());
        state.routing = Some(drag); // keep the session alive so the caller can try elsewhere
        error_json(reason.unwrap_or_else(|| {
            "can't finish here -- route_to the target pin first (its position must be reachable and docked, i.e. hover_target set) before calling finish_route".to_string()
        }))
    }
}

/// [`crate::mcp::McpQuery::CancelRoute`]'s handler -- unlike every other
/// handler here, this never fails: abandoning nothing is as valid a
/// no-op as abandoning something.
fn cancel_route_write(screen: &mut Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let was_active = state.routing.take().is_some();
    serde_json::json!({ "ok": true, "was_active": was_active })
}

/// [`crate::mcp::McpQuery::DropViaAndSwitchLayer`]'s handler -- the drag
/// stays alive (on the other layer) both on success and on failure, so
/// this never needs `state.routing.take()` the way [`finish_route_write`]
/// does.
fn drop_via_and_switch_layer_write(screen: &mut Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(drag) = state.routing.as_mut() else {
        return error_json("no route in progress -- call start_route first");
    };
    if let Err(e) = drag.drop_via_and_switch_layer(&mut state.doc) {
        return error_json(e.to_string());
    }
    let mut result = routing_json(&state.doc, &state.templates, drag);
    if let Some(obj) = result.as_object_mut() {
        obj.insert("ok".to_string(), serde_json::json!(true));
    }
    result
}

/// [`crate::mcp::McpQuery::AddVia`]'s handler.
fn add_via_write(screen: &mut Screen, args: crate::mcp::AddViaArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(net) = state.doc.find_net_by_name(&args.net) else {
        return error_json(format!("unknown net \"{}\" -- only a net already created by a prior connect_pins call can be targeted", args.net));
    };
    let position = Point::new(mm_arg(args.x_mm), mm_arg(args.y_mm));
    match state.doc.try_add_stitching_via(position, net, mm_arg(args.diameter_mm), mm_arg(args.drill_mm)) {
        Ok(id) => serde_json::json!({ "ok": true, "via_id": id.0, "net": args.net, "x_mm": args.x_mm, "y_mm": args.y_mm }),
        Err(e) => error_json(format!("couldn't place a via at ({}, {})mm: {e}", args.x_mm, args.y_mm)),
    }
}

/// [`crate::mcp::McpQuery::AddPinStitchingVia`]'s handler -- the
/// AI-driven equivalent of `EditorState::add_pin_stitching_via_at`'s
/// first, "natural spot worked" branch; unlike that GUI action, there's
/// no drag-fallback ghost here for the caller to steer (see
/// `AddPinStitchingViaArgs`'s own doc comment for why), so a refusal is
/// just reported back as a plain error.
fn add_pin_stitching_via_write(screen: &mut Screen, args: crate::mcp::AddPinStitchingViaArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(pad_id) = state.doc.find_pad(&state.templates, &args.reference, &args.pin) else {
        return error_json(format!("no such pin: {} pin {}", args.reference, args.pin));
    };
    match state.doc.try_add_pin_stitching_via(pad_id, mm_arg(args.diameter_mm), mm_arg(args.drill_mm), mm_arg(args.stub_width_mm)) {
        Ok(result) => {
            let mut json = point_mm_json(result.center);
            json["ok"] = serde_json::json!(true);
            json["via_id"] = serde_json::json!(result.via_id.0);
            json
        }
        Err(e) => error_json(format!("couldn't place a stitching via near {}.{}: {e}", args.reference, args.pin)),
    }
}

/// [`crate::mcp::McpQuery::AddZone`]'s handler.
fn add_zone_write(screen: &mut Screen, args: crate::mcp::AddZoneArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let layer = match args.layer.as_str() {
        "front" => LayerId::FCu,
        "back" => LayerId::BCu,
        other => return error_json(format!("invalid layer \"{other}\" -- must be \"front\" or \"back\"")),
    };
    if args.points.is_empty() {
        return error_json("points must contain at least one point");
    }
    let Some(net) = state.doc.find_net_by_name(&args.net) else {
        return error_json(format!("unknown net \"{}\" -- only a net already created by a prior connect_pins call can be targeted", args.net));
    };
    let outline = Polygon::new(args.points.iter().map(|p| Point::new(mm_arg(p.x_mm), mm_arg(p.y_mm))).collect());
    let zone_id = state.doc.add_zone(outline, layer, net);
    let island_count = state.doc.zones.iter().find(|z| z.id == zone_id).map(|z| z.item_ids.len()).unwrap_or(0);
    serde_json::json!({ "ok": true, "zone_id": zone_id.0, "net": args.net, "layer": args.layer, "filled_islands": island_count })
}

/// [`crate::mcp::McpQuery::AddSilkText`]'s handler.
fn add_silk_text_write(screen: &mut Screen, args: crate::mcp::AddSilkTextArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let layer = match args.layer.as_str() {
        "front" => LayerId::FCu,
        "back" => LayerId::BCu,
        other => return error_json(format!("invalid layer \"{other}\" -- must be \"front\" or \"back\"")),
    };
    let position = Point::new(mm_arg(args.x_mm), mm_arg(args.y_mm));
    let height = mm_arg(args.height_mm);
    match state.doc.try_place_silk_text(&args.text, position, args.rotation_deg, layer, height) {
        Ok(id) => serde_json::json!({ "ok": true, "silk_text_id": id.0, "text": args.text, "layer": args.layer, "height_mm": args.height_mm }),
        Err(e) => error_json(format!("couldn't place silk text {:?}: {e}", args.text)),
    }
}

/// [`crate::mcp::McpQuery::RefillZones`]'s handler.
fn refill_zones_write(screen: &mut Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    state.doc.refill_all_zones();
    serde_json::json!({ "ok": true, "zone_count": state.doc.zones.len() })
}

/// [`crate::mcp::McpQuery::RenameNet`]'s handler.
fn rename_net_write(screen: &mut Screen, args: crate::mcp::RenameNetArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(net) = state.doc.find_net_by_name(&args.net) else {
        return error_json(format!("unknown net \"{}\"", args.net));
    };
    match state.doc.rename_net(net, &args.new_name) {
        Ok(()) => serde_json::json!({ "ok": true, "old_name": args.net, "new_name": args.new_name.trim() }),
        Err(e) => error_json(format!("couldn't rename \"{}\": {e}", args.net)),
    }
}

/// [`crate::mcp::McpQuery::SaveBoard`]'s handler -- same
/// `save_to_path`/`remember_last_board`/`set_file_path` sequence as the
/// GUI's own "Save"/"Save As" buttons.
fn save_board_write(screen: &mut Screen, args: crate::mcp::SaveBoardArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let path = match args.path.map(PathBuf::from).or_else(|| state.file_path.clone()) {
        Some(path) => path,
        None => return error_json("no path given, and this board has never been saved before -- give a path"),
    };
    match save_to_path(&state.doc, &path) {
        Ok(()) => {
            remember_last_board(&path);
            state.set_file_path(path.clone());
            serde_json::json!({ "ok": true, "path": path.to_string_lossy() })
        }
        Err(e) => error_json(format!("couldn't save board: {e}")),
    }
}

/// [`crate::mcp::McpQuery::ExportManufacturingFiles`]'s handler --
/// native Gerber/Excellon zip + JLCPCB CPL + BOM, no KiCad involved.
/// `bom_csv_contents` is pre-built by the caller (needs `PartsDb`).
fn export_manufacturing_files_write(screen: &Screen, bom_csv_contents: &str, args: crate::mcp::ExportManufacturingFilesArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };

    let out_dir = std::path::Path::new(&args.out_dir);
    if let Err(e) = std::fs::create_dir_all(out_dir) {
        return error_json(format!("couldn't create {}: {e}", out_dir.display()));
    }
    let stem = state.file_path.as_ref().and_then(|p| p.file_stem()).and_then(|s| s.to_str()).unwrap_or("board");
    match crate::native_gerber::export_manufacturing_files_native(&state.doc, &state.templates, stem, out_dir, bom_csv_contents) {
        Ok(files) => serde_json::json!({
            "ok": true,
            "backend": "native",
            "gerber_zip_path": files.gerber_zip.to_string_lossy(),
            "cpl_csv_path": files.position_csv.to_string_lossy(),
            "bom_csv_path": files.bom_csv.to_string_lossy(),
        }),
        Err(e) => error_json(format!("native manufacturing export failed: {e}")),
    }
}

/// [`crate::mcp::McpQuery::StartExternalAutoroute`]'s handler --
/// mirrors [`EditorState::start_autoroute`] (the GUI dialog's own
/// "Route" button), but reports success/failure as JSON instead of a
/// GUI toast message, and supports a one-off `extra_args` override on
/// top of whatever's persisted in [`EditorState::external_router_settings`].
/// Deliberately never merges the result onto the board itself -- see
/// `start_external_autoroute`'s own tool description for why that
/// stays a manual GUI step.
fn start_external_autoroute_write(screen: &mut Screen, args: crate::mcp::StartExternalAutorouteArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    if matches!(&state.autoroute_job, Some(job) if job.result.is_none()) {
        return error_json("an autoroute job is already running -- poll get_external_autoroute_status and wait for it to finish (or cancel it in the GUI's Autoroute (extern) dialog) first");
    }
    let net_names = if args.nets.is_empty() { state.doc.multi_item_net_names() } else { args.nets };
    let mut settings = state.external_router_settings.clone();
    if let Some(extra) = args.extra_args {
        settings.extra_args = extra;
    }
    match crate::external_router::run_autoroute(&state.doc, &state.templates, net_names.clone(), settings) {
        Ok(handle) => {
            state.autoroute_job = Some(AutorouteJob { handle, log: Vec::new(), result: None });
            serde_json::json!({ "ok": true, "status": "running", "requested_nets": net_names })
        }
        Err(e) => error_json(format!("couldn't start the external autoroute: {e}")),
    }
}

/// [`crate::mcp::McpQuery::GetExternalAutorouteStatus`]'s handler --
/// polls the job the same way [`EditorState::poll_autoroute_job`]
/// already does each GUI frame (so this reports accurately even if
/// called before that frame-driven poll gets to it) and reports its
/// current state as plain JSON. Never merges anything onto the board;
/// see [`start_external_autoroute_write`]'s own doc comment for why.
fn external_autoroute_status_json(screen: &mut Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    state.poll_autoroute_job();
    let Some(job) = &state.autoroute_job else {
        return serde_json::json!({ "status": "idle" });
    };
    match &job.result {
        None => serde_json::json!({ "status": "running", "log": job.log }),
        Some(Ok(report)) => serde_json::json!({
            "status": "done",
            "ok": true,
            "requested_nets": report.requested_nets,
            "routed_nets": report.routed_nets,
            "drc_ok": report.drc_ok,
            "connected_ok": report.connected_ok,
            "item_count": report.items.len(),
            "note": "these routed tracks/vias are not on the board yet -- merging (or discarding) them is a manual step in the GUI's Autoroute (extern) dialog",
        }),
        Some(Err(message)) => serde_json::json!({ "status": "failed", "ok": false, "error": message }),
    }
}

/// The `tool` name [`run_batch_write`] tags a `{"skipped": true}` entry
/// with, for an operation it never got to attempt because an earlier
/// one already failed with `stop_on_error` (the default) -- kept as
/// its own tiny function so it can be called on a `&BatchOp` without
/// having to move/execute it the way the dispatch match below does.
fn batch_op_name(op: &crate::mcp::BatchOp) -> &'static str {
    use crate::mcp::BatchOp;
    match op {
        BatchOp::CreateBoard(_) => "create_board",
        BatchOp::PlaceFootprint(_) => "place_footprint",
        BatchOp::RegisterPart(_) => "register_part",
        BatchOp::ConnectPins(_) => "connect_pins",
        BatchOp::RoutePins(_) => "route_pins",
        BatchOp::StartRoute(_) => "start_route",
        BatchOp::RouteTo(_) => "route_to",
        BatchOp::FixCorner => "fix_corner",
        BatchOp::UndoLastCorner => "undo_last_corner",
        BatchOp::FinishRoute => "finish_route",
        BatchOp::CancelRoute => "cancel_route",
        BatchOp::DropViaAndSwitchLayer => "drop_via_and_switch_layer",
        BatchOp::AddVia(_) => "add_via",
        BatchOp::AddPinStitchingVia(_) => "add_pin_stitching_via",
        BatchOp::AddZone(_) => "add_zone",
        BatchOp::AddSilkText(_) => "add_silk_text",
        BatchOp::RefillZones => "refill_zones",
        BatchOp::RenameNet(_) => "rename_net",
        BatchOp::SaveBoard(_) => "save_board",
        BatchOp::ExportManufacturingFiles(_) => "export_manufacturing_files",
    }
}

/// [`crate::mcp::McpQuery::RunBatch`]'s handler -- runs every operation
/// in `args.operations` against the live board in order, each through
/// the exact same `*_write` function its own single-call tool would
/// use, collecting one JSON result per operation instead of forcing an
/// MCP client to spend one whole round-trip (and, worse, one whole
/// slice of ever-growing conversation history) per operation. Stops at
/// the first operation whose result carries an `"error"` key when
/// `args.stop_on_error` is set (the default) -- every operation after
/// that point is reported back as `{"skipped": true}` rather than
/// attempted, since e.g. a `connect_pins` after a failed
/// `place_footprint` would just fail again for the same reason and add
/// nothing but noise.
fn run_batch_write(screen: &mut Screen, parts_db: &mut PartsDb, args: crate::mcp::RunBatchArgs) -> serde_json::Value {
    use crate::mcp::BatchOp;

    let mut results = Vec::with_capacity(args.operations.len());
    let mut stopped = false;
    let mut ok_count = 0usize;
    let mut error_count = 0usize;

    for (index, op) in args.operations.into_iter().enumerate() {
        let tool = batch_op_name(&op);
        if stopped {
            results.push(serde_json::json!({ "index": index, "tool": tool, "skipped": true }));
            continue;
        }
        let mut result = match op {
            BatchOp::CreateBoard(op_args) => create_board_write(screen, parts_db, op_args),
            BatchOp::PlaceFootprint(op_args) => place_footprint_write(screen, op_args),
            BatchOp::RegisterPart(op_args) => register_part_write(screen, parts_db, op_args),
            BatchOp::ConnectPins(op_args) => connect_pins_write(screen, op_args),
            BatchOp::RoutePins(op_args) => route_pins_write(screen, op_args),
            BatchOp::StartRoute(op_args) => start_route_write(screen, op_args),
            BatchOp::RouteTo(op_args) => route_to_write(screen, op_args),
            BatchOp::FixCorner => fix_corner_write(screen),
            BatchOp::UndoLastCorner => undo_last_corner_write(screen),
            BatchOp::FinishRoute => finish_route_write(screen),
            BatchOp::CancelRoute => cancel_route_write(screen),
            BatchOp::DropViaAndSwitchLayer => drop_via_and_switch_layer_write(screen),
            BatchOp::AddVia(op_args) => add_via_write(screen, op_args),
            BatchOp::AddPinStitchingVia(op_args) => add_pin_stitching_via_write(screen, op_args),
            BatchOp::AddZone(op_args) => add_zone_write(screen, op_args),
            BatchOp::AddSilkText(op_args) => add_silk_text_write(screen, op_args),
            BatchOp::RefillZones => refill_zones_write(screen),
            BatchOp::RenameNet(op_args) => rename_net_write(screen, op_args),
            BatchOp::SaveBoard(op_args) => save_board_write(screen, op_args),
            BatchOp::ExportManufacturingFiles(op_args) => {
                let bom_csv = match screen {
                    Screen::Editor(state) => crate::bom::to_csv(&crate::bom::build_bom_rows(&state.doc, &state.templates, &state.template_origin, parts_db)),
                    Screen::NewBoard(_) => crate::bom::to_csv(&[]),
                };
                export_manufacturing_files_write(screen, &bom_csv, op_args)
            }
        };
        if result.get("error").is_some() {
            error_count += 1;
            stopped = args.stop_on_error;
        } else {
            ok_count += 1;
        }
        if let Some(obj) = result.as_object_mut() {
            obj.insert("index".to_string(), serde_json::json!(index));
            obj.insert("tool".to_string(), serde_json::json!(tool));
        }
        results.push(result);
    }

    serde_json::json!({
        "ok": error_count == 0,
        "operation_count": results.len(),
        "ok_count": ok_count,
        "error_count": error_count,
        "stopped_early": stopped,
        "results": results,
    })
}

/// [`no_board_open_json`]'s error-shaped twin -- every write handler
/// above needs the `"error"` key (see [`error_json`]) rather than the
/// bare `"note"` the read-only `*_json` builders use, so an MCP client
/// can check for `"error"` consistently across every tool.
fn no_board_open_json_error() -> serde_json::Value {
    error_json("no board is open yet -- create one first (create_board, or open one in the GUI)")
}

/// Steps [`EditorState::silk_text_height`] (or an already-selected
/// text's own height) one entry forward/backward through
/// [`SILK_TEXT_HEIGHT_STEPS_MM`], clamped at either end rather than
/// wrapping -- the "bigger"/"smaller" size buttons' shared arithmetic.
/// Falls back to whichever step is numerically closest if `current`
/// doesn't land exactly on one (e.g. an older board saved before this
/// stepper existed, at some other height entirely) instead of doing
/// nothing.
fn silk_text_height_step(current: Unit, delta: i32) -> Unit {
    let steps: Vec<Unit> = SILK_TEXT_HEIGHT_STEPS_MM.iter().map(|&mm_value| mm_arg(mm_value)).collect();
    let current_index = steps.iter().position(|&s| s == current).unwrap_or_else(|| {
        steps.iter().enumerate().min_by_key(|(_, &s)| (s - current).abs()).map(|(i, _)| i).unwrap_or(0)
    });
    let new_index = (current_index as i32 + delta).clamp(0, steps.len() as i32 - 1) as usize;
    steps[new_index]
}

/// [`silk_text_height_step`]'s exact counterpart for a silk dot's
/// diameter, over [`SILK_DOT_DIAMETER_STEPS_MM`] -- same clamped,
/// closest-step-fallback arithmetic.
fn silk_dot_diameter_step(current: Unit, delta: i32) -> Unit {
    let steps: Vec<Unit> = SILK_DOT_DIAMETER_STEPS_MM.iter().map(|&mm_value| mm_arg(mm_value)).collect();
    let current_index = steps.iter().position(|&s| s == current).unwrap_or_else(|| {
        steps.iter().enumerate().min_by_key(|(_, &s)| (s - current).abs()).map(|(i, _)| i).unwrap_or(0)
    });
    let new_index = (current_index as i32 + delta).clamp(0, steps.len() as i32 - 1) as usize;
    steps[new_index]
}

/// mm -> internal [`Unit`] conversion for MCP write-tool arguments,
/// taking `f64` (JSON has no separate float-width concept) rather than
/// `crate::cli`'s own private `f64`-taking `mm` helper -- kept as its
/// own copy here since that one is private to `cli.rs`.
fn mm_arg(v: f64) -> Unit {
    (v * MM as f64).round() as Unit
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_last_board_pointer_recovers_a_plain_path() {
        assert_eq!(parse_last_board_pointer("/home/user/board.json"), Some(PathBuf::from("/home/user/board.json")));
    }

    #[test]
    fn parse_last_board_pointer_trims_a_trailing_newline() {
        // `remember_last_board` never writes one itself, but a pointer
        // file is plain, hand-editable text -- must not choke on one.
        assert_eq!(parse_last_board_pointer("/home/user/board.json\n"), Some(PathBuf::from("/home/user/board.json")));
    }

    #[test]
    fn parse_last_board_pointer_treats_empty_or_whitespace_only_content_as_no_remembered_board() {
        assert_eq!(parse_last_board_pointer(""), None);
        assert_eq!(parse_last_board_pointer("   \n"), None);
    }

    /// Snapshots [`last_board_pointer_path`]'s real, global, on-disk
    /// content before a test that (transitively, via `save_board_write`
    /// -> [`remember_last_board`]) writes to it, and restores it
    /// verbatim on drop -- runs even if the test panics on an `assert!`
    /// first, unlike a plain cleanup statement at the end of the test
    /// body. Without this, a single `cargo test` run permanently
    /// clobbers *this machine's own* "reopen the last real board on GUI
    /// startup" pointer with a temp test path that's deleted moments
    /// later -- not a hypothetical, this exact thing happened on a real
    /// dev machine and had to be manually restored.
    struct LastBoardPointerGuard {
        path: PathBuf,
        original: Option<String>,
    }

    impl LastBoardPointerGuard {
        fn capture() -> Self {
            let path = last_board_pointer_path();
            let original = std::fs::read_to_string(&path).ok();
            Self { path, original }
        }
    }

    impl Drop for LastBoardPointerGuard {
        fn drop(&mut self) {
            match &self.original {
                Some(content) => {
                    if let Some(parent) = self.path.parent() {
                        let _ = std::fs::create_dir_all(parent);
                    }
                    let _ = std::fs::write(&self.path, content);
                }
                None => {
                    let _ = std::fs::remove_file(&self.path);
                }
            }
        }
    }

    // `crate::mcp`'s JSON builders: exercised here (not in `mcp.rs`
    // itself) since they need direct access to `Screen`/`EditorState`'s
    // otherwise-private fields -- see `handle_mcp_query`'s doc comment.

    fn mm(v: f64) -> Unit {
        (v * MM as f64).round() as Unit
    }

    fn test_board() -> BoardDoc {
        NewBoardParams { width_mm: 40.0, height_mm: 40.0, layer_count: LayerCount::Two, copper_weight: CopperWeight::OneOz, corner_radius_mm: 0.0 }.create()
    }

    fn two_pin_template() -> FootprintTemplate {
        footprint::builtin_templates().remove(0)
    }

    fn mounting_hole_template() -> FootprintTemplate {
        footprint::builtin_templates().into_iter().find(|t| t.name.starts_with("Mounting hole (M3")).unwrap()
    }

    fn test_editor_state() -> EditorState {
        let templates = footprint::builtin_templates();
        let origin = vec![None; templates.len()];
        let hover = vec![None; templates.len()];
        let category = vec![None; templates.len()];
        EditorState::new(test_board(), templates, origin, hover, category)
    }

    #[test]
    fn snap_to_grid_point_rounds_both_axes_to_the_nearest_grid_intersection() {
        let p = Point::new(mm(5.37), mm(3.63));
        let snapped = snap_to_grid_point(p, MM, true);
        assert_eq!(snapped, Point::new(mm(5.0), mm(4.0)));
    }

    #[test]
    fn snap_to_grid_point_is_a_no_op_when_disabled_or_given_a_non_positive_spacing() {
        let p = Point::new(mm(5.37), mm(3.63));
        assert_eq!(snap_to_grid_point(p, MM, false), p, "disabled must pass the point through untouched");
        assert_eq!(snap_to_grid_point(p, 0, true), p, "a non-positive spacing must never divide by zero or otherwise mangle the point");
    }

    #[test]
    fn update_drag_snaps_the_dragged_footprints_candidate_position_to_the_grid() {
        let mut state = test_editor_state();
        let template = two_pin_template();
        state.doc.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let pad_position = match state.doc.node.get(state.doc.footprints[0].pad_item_ids[0]).unwrap() {
            Item::Pad { shape, .. } => shape.center(),
            _ => panic!("expected a pad"),
        };
        state.grid_spacing = MM;
        state.grid_snap_enabled = true;

        state.begin_drag(pad_position);
        assert!(state.dragging.is_some(), "clicking right on a pad must start a drag");

        // Move the footprint by an off-grid delta (0.37mm/0.63mm past
        // the nearest 1mm lines in x/y) -- the resulting candidate
        // position must land exactly on the grid, not wherever the raw
        // cursor happened to be.
        let cursor = pad_position.add(Point::new(mm(5.37), mm(3.63)));
        state.update_drag(cursor);
        let candidate = state.dragging.as_ref().unwrap().candidate_position;
        assert_eq!(candidate, Point::new(mm(5.0), mm(4.0)));
    }

    #[test]
    fn update_drag_leaves_the_candidate_position_off_grid_when_snapping_is_disabled() {
        let mut state = test_editor_state();
        let template = two_pin_template();
        state.doc.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let pad_position = match state.doc.node.get(state.doc.footprints[0].pad_item_ids[0]).unwrap() {
            Item::Pad { shape, .. } => shape.center(),
            _ => panic!("expected a pad"),
        };
        state.grid_snap_enabled = false;

        state.begin_drag(pad_position);
        let cursor = pad_position.add(Point::new(mm(5.37), mm(3.63)));
        state.update_drag(cursor);
        let candidate = state.dragging.as_ref().unwrap().candidate_position;
        assert_eq!(candidate, Point::new(mm(5.37), mm(3.63)), "with snapping off the exact cursor-driven position must be kept");
    }

    /// Regression test for a real, user-reported bug: a placed silk
    /// text could never be clicked, selected, or deleted -- `EditorState`
    /// had no selection slot for one at all, and `handle_select_click`
    /// never even tried `BoardDoc::silk_text_at`.
    #[test]
    fn handle_select_click_selects_a_placed_silk_text_and_delete_removes_it() {
        let mut state = test_editor_state();
        let id = state.doc.try_place_silk_text("REV A", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap();

        state.handle_select_click(Point::new(0, 0));
        assert_eq!(state.selected_silk_text, Some(id));
        assert_eq!(state.selected, None, "selecting a silk text must not leave a stale footprint selection");

        // What the real `Delete`/`Backspace` key handler does once a
        // silk text is the live selection.
        let taken = state.selected_silk_text.take().unwrap();
        state.doc.remove_silk_text(taken);
        assert!(state.doc.silk_texts.is_empty());
    }

    /// Regression test for the other half of the same bug report: a
    /// placed silk text also couldn't be dragged to a new position --
    /// `begin_drag`/`update_drag`/`finish_drag` only ever knew about
    /// `Dragging` (a footprint), never a silk text.
    #[test]
    fn begin_drag_and_finish_drag_move_a_selected_silk_text_to_a_new_position() {
        let mut state = test_editor_state();
        let id = state.doc.try_place_silk_text("REV A", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap();

        state.begin_drag(Point::new(0, 0));
        assert!(state.silk_text_dragging.is_some(), "clicking right on a placed silk text must start a drag");
        assert_eq!(state.selected_silk_text, Some(id));

        state.update_drag(Point::new(mm(5.0), mm(3.0)));
        assert!(state.silk_text_dragging.as_ref().unwrap().valid, "open board space must accept the move");

        state.finish_drag();
        assert!(state.silk_text_dragging.is_none());
        assert_eq!(state.doc.silk_texts[0].position, Point::new(mm(5.0), mm(3.0)));
    }

    /// Regression test for the mounting-hole half of the same bug
    /// report: a pure mounting-hole footprint (no pads at all) could
    /// be placed but never again selected/dragged/deleted by clicking
    /// on it, since `BoardDoc::footprint_at` used to only ever
    /// hit-test pads. Exercised here (not just in `board_doc.rs`'s own
    /// `footprint_at_finds_a_pure_mounting_hole_footprint_...` test)
    /// because the actual user-facing bug was in the GUI's
    /// click-to-select path, `handle_select_click`, not `footprint_at`
    /// in isolation.
    #[test]
    fn handle_select_click_selects_a_pure_mounting_hole_footprint_by_its_hole() {
        let mut state = test_editor_state();
        let template = mounting_hole_template();
        let id = state.doc.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let hole_position = match state.doc.node.get(state.doc.footprints[0].hole_item_ids[0]).unwrap() {
            Item::Hole { position, .. } => *position,
            _ => panic!("expected a hole"),
        };

        state.handle_select_click(hole_position);
        assert_eq!(state.selected, Some(id));

        state.doc.remove_footprint(id);
        assert!(state.doc.footprints.is_empty());
    }

    #[test]
    fn editor_state_json_on_the_new_board_screen_reports_no_board_open_rather_than_panicking() {
        let screen = Screen::NewBoard(NewBoardParams::default());
        let json = editor_state_json(&screen);
        assert!(json["note"].as_str().unwrap().contains("no board is open"));
    }

    #[test]
    fn editor_state_json_reports_the_select_tool_and_nothing_in_progress_on_a_fresh_board() {
        let screen = Screen::Editor(test_editor_state());
        let json = editor_state_json(&screen);
        assert_eq!(json["tool"], "Select");
        assert!(json["routing"].is_null());
        assert!(json["draw_zone"].is_null());
        assert!(json["place_via"].is_null());
        assert!(json["hover_board_mm"].is_null());
    }

    #[test]
    fn editor_state_json_surfaces_a_stray_in_progress_route_exactly_the_blind_spot_this_module_fixes() {
        let mut state = test_editor_state();
        let template = two_pin_template();
        let p1 = state.doc.try_place_footprint(&template, Point::new(-mm(5.0), 0), 0.0).unwrap();
        let p2 = state.doc.try_place_footprint(&template, Point::new(mm(5.0), 0), 0.0).unwrap();
        let pad_a = state.doc.footprints.iter().find(|f| f.id == p1).unwrap().pad_item_ids[0];
        let pad_b = state.doc.footprints.iter().find(|f| f.id == p2).unwrap().pad_item_ids[0];
        state.doc.connect_pads(pad_a, pad_b).unwrap();

        // Started from "Route traces" (see `Tool::Route`), then the
        // tool was switched away without cancelling -- reproduces the
        // exact stray-preview scenario this whole feature was built to
        // make visible from outside the GUI.
        state.routing = RoutingDrag::start(&state.doc, pad_a);
        state.tool = Tool::Connect;

        let screen = Screen::Editor(state);
        let json = editor_state_json(&screen);
        assert_eq!(json["tool"], "Connect", "the tool really did change");
        let routing = &json["routing"];
        assert!(!routing.is_null(), "the abandoned route must still be visible even though the tool moved on");
        assert_eq!(routing["net"], "Net1");
        assert_eq!(routing["from_pad"]["footprint"], "P1");
        assert_eq!(routing["from_pad"]["pin"], "1");
    }

    #[test]
    fn editor_state_json_reports_an_in_progress_draw_zone_outline_regardless_of_the_active_tool() {
        let mut state = test_editor_state();
        state.zone_points = vec![Point::new(0, 0), Point::new(mm(5.0), 0)];
        state.tool = Tool::Select; // switched away, same as the routing test above

        let screen = Screen::Editor(state);
        let json = editor_state_json(&screen);
        assert_eq!(json["draw_zone"]["point_count"], 2);
    }

    #[test]
    fn board_overview_json_on_the_new_board_screen_reports_no_board_open() {
        let screen = Screen::NewBoard(NewBoardParams::default());
        let json = board_overview_json(&screen);
        assert!(json["note"].as_str().unwrap().contains("no board is open"));
    }

    #[test]
    fn board_overview_json_reports_the_boards_own_size_and_item_counts() {
        let mut state = test_editor_state();
        let template = two_pin_template();
        state.doc.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();

        let screen = Screen::Editor(state);
        let json = board_overview_json(&screen);
        assert_eq!(json["width_mm"], 40.0);
        assert_eq!(json["height_mm"], 40.0);
        assert_eq!(json["layer_count"], 2);
        assert_eq!(json["footprint_count"], 1);
        assert_eq!(json["zone_count"], 0);
    }

    #[test]
    fn nets_json_lists_every_pad_on_each_net_by_footprint_and_pin() {
        let mut state = test_editor_state();
        let template = two_pin_template();
        let p1 = state.doc.try_place_footprint(&template, Point::new(-mm(5.0), 0), 0.0).unwrap();
        let p2 = state.doc.try_place_footprint(&template, Point::new(mm(5.0), 0), 0.0).unwrap();
        let pad_a = state.doc.footprints.iter().find(|f| f.id == p1).unwrap().pad_item_ids[0];
        let pad_b = state.doc.footprints.iter().find(|f| f.id == p2).unwrap().pad_item_ids[0];
        state.doc.connect_pads(pad_a, pad_b).unwrap();

        let screen = Screen::Editor(state);
        let json = nets_json(&screen);
        let nets = json["nets"].as_array().unwrap();
        assert_eq!(nets.len(), 1);
        assert_eq!(nets[0]["name"], "Net1");
        assert_eq!(nets[0]["pin_count"], 2);
        let refs: Vec<&str> = nets[0]["pads"].as_array().unwrap().iter().map(|p| p["footprint"].as_str().unwrap()).collect();
        assert_eq!(refs, vec!["P1", "P2"]);
    }

    #[test]
    fn zones_json_mirrors_the_cli_list_zones_shape() {
        let mut state = test_editor_state();
        let template = two_pin_template();
        let p1 = state.doc.try_place_footprint(&template, Point::new(-mm(5.0), 0), 0.0).unwrap();
        let p2 = state.doc.try_place_footprint(&template, Point::new(mm(5.0), 0), 0.0).unwrap();
        let pad_a = state.doc.footprints.iter().find(|f| f.id == p1).unwrap().pad_item_ids[0];
        let pad_b = state.doc.footprints.iter().find(|f| f.id == p2).unwrap().pad_item_ids[0];
        let net = state.doc.connect_pads(pad_a, pad_b).unwrap();
        let outline = Polygon::new(vec![Point::new(-mm(15.0), -mm(15.0)), Point::new(mm(15.0), -mm(15.0)), Point::new(mm(15.0), mm(15.0)), Point::new(-mm(15.0), mm(15.0))]);
        state.doc.add_zone(outline, LayerId::FCu, net);

        let screen = Screen::Editor(state);
        let json = zones_json(&screen);
        let zones = json["zones"].as_array().unwrap();
        assert_eq!(zones.len(), 1);
        assert_eq!(zones[0]["net"], "Net1");
        assert_eq!(zones[0]["layer"], "F.Cu");
        assert_eq!(zones[0]["outline_points"], 4);
        assert!(zones[0]["filled_islands"].as_u64().unwrap() >= 1);
    }

    #[test]
    fn net_continuity_json_flags_a_net_that_is_connected_but_never_actually_routed() {
        // `connect_pins_write` only declares the net -- no copper --
        // so the two pads are exactly the "logically one net, zero
        // physical bridge" gap this whole feature exists to catch.
        let screen = two_connected_pins_20mm_apart();

        let json = net_continuity_json(&screen, crate::mcp::CheckNetContinuityArgs { net_name: None });
        assert_eq!(json["summary"]["nets_checked"], 1);
        assert_eq!(json["summary"]["nets_fully_connected"], 0);
        assert_eq!(json["summary"]["nets_with_gaps"], 1);
        let problems = json["problem_nets"].as_array().unwrap();
        assert_eq!(problems.len(), 1);
        assert_eq!(problems[0]["name"], "Net1");
        assert_eq!(problems[0]["fully_connected"], false);
        assert_eq!(problems[0]["component_count"], 2);
        let components = problems[0]["components"].as_array().unwrap();
        assert_eq!(components.len(), 2);
        assert_eq!(components[0]["pad_count"], 1);
        assert_eq!(components[1]["pad_count"], 1);
    }

    #[test]
    fn net_continuity_json_reports_no_gaps_once_the_net_is_actually_routed() {
        let mut screen = two_connected_pins_20mm_apart();
        let json = route_pins_write(&mut screen, crate::mcp::RoutePinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string(), width_mm: 0.25 });
        assert_eq!(json["ok"], true, "unexpected response: {json}");

        let json = net_continuity_json(&screen, crate::mcp::CheckNetContinuityArgs { net_name: None });
        assert_eq!(json["summary"]["nets_checked"], 1);
        assert_eq!(json["summary"]["nets_fully_connected"], 1);
        assert_eq!(json["summary"]["nets_with_gaps"], 0);
        assert_eq!(json["problem_nets"].as_array().unwrap().len(), 0, "a fully-connected net must not show up in the problem list");
    }

    #[test]
    fn net_continuity_json_with_a_net_name_always_reports_full_detail_even_when_fully_connected() {
        let mut screen = two_connected_pins_20mm_apart();
        route_pins_write(&mut screen, crate::mcp::RoutePinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string(), width_mm: 0.25 });

        let json = net_continuity_json(&screen, crate::mcp::CheckNetContinuityArgs { net_name: Some("Net1".to_string()) });
        assert_eq!(json["name"], "Net1");
        assert_eq!(json["fully_connected"], true);
        assert_eq!(json["component_count"], 1);
        let components = json["components"].as_array().unwrap();
        assert_eq!(components.len(), 1);
        assert_eq!(components[0]["pad_count"], 2);
        assert!(components[0]["track_count"].as_u64().unwrap() >= 1);
    }

    #[test]
    fn net_continuity_json_reports_an_error_for_an_unknown_net_name() {
        let screen = two_connected_pins_20mm_apart();
        let json = net_continuity_json(&screen, crate::mcp::CheckNetContinuityArgs { net_name: Some("NoSuchNet".to_string()) });
        assert!(json["error"].as_str().unwrap().contains("NoSuchNet"), "unexpected response: {json}");
    }

    #[test]
    fn footprints_json_reports_each_pads_net_or_null_when_unconnected() {
        let mut state = test_editor_state();
        let template = two_pin_template();
        let p1 = state.doc.try_place_footprint(&template, Point::new(-mm(5.0), 0), 0.0).unwrap();
        let p2 = state.doc.try_place_footprint(&template, Point::new(mm(5.0), 0), 0.0).unwrap();
        let pad_a = state.doc.footprints.iter().find(|f| f.id == p1).unwrap().pad_item_ids[0];
        let pad_b = state.doc.footprints.iter().find(|f| f.id == p2).unwrap().pad_item_ids[0];
        state.doc.connect_pads(pad_a, pad_b).unwrap();

        let screen = Screen::Editor(state);
        let json = footprints_json(&screen);
        let footprints = json["footprints"].as_array().unwrap();
        assert_eq!(footprints.len(), 2);
        assert_eq!(footprints[0]["reference"], "P1");
        assert_eq!(footprints[0]["pads"][0]["net"], "Net1", "pin 1 is connected");
        assert!(footprints[0]["pads"][1]["net"].is_null(), "pin 2 was never connected");
    }

    #[test]
    fn footprints_json_reports_a_pads_schematic_pin_name_when_the_template_has_one() {
        let mut template = two_pin_template();
        template.pads[0].pin_name = Some("GND".to_string());
        let templates = vec![template.clone()];
        let mut state = EditorState::new(test_board(), templates, vec![None], vec![None], vec![None]);
        state.doc.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();

        let screen = Screen::Editor(state);
        let json = footprints_json(&screen);
        let pads = &json["footprints"][0]["pads"];
        assert_eq!(pads[0]["pin_name"], "GND", "a pad with a known schematic pin function must report it");
        assert!(pads[1]["pin_name"].is_null(), "a pad with no known pin function must report null, not a made-up value");
    }

    #[test]
    fn footprints_json_reports_the_templates_own_courtyard_dimensions() {
        // `two_pin_template()` has no `explicit_courtyard` at all, so
        // this must be the pad bounding-box fallback: two 0.45mm-radius
        // pads at +/-1.27mm apart -> 3.44mm wide, 0.9mm tall.
        let mut state = test_editor_state();
        let template = two_pin_template();
        state.doc.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();

        let screen = Screen::Editor(state);
        let json = footprints_json(&screen);
        let courtyard = &json["footprints"][0]["courtyard"];
        assert!((courtyard["width_mm"].as_f64().unwrap() - 3.44).abs() < 0.01);
        assert!((courtyard["height_mm"].as_f64().unwrap() - 0.9).abs() < 0.01);
    }

    // -- Write handlers (`crate::mcp`'s write tools) below --

    fn test_parts_db() -> PartsDb {
        PartsDb::open_in_memory().expect("an in-memory sqlite database must always succeed")
    }

    #[test]
    fn create_board_write_switches_from_new_board_to_editor() {
        let mut screen = Screen::NewBoard(NewBoardParams::default());
        let parts_db = test_parts_db();
        let json = create_board_write(
            &mut screen,
            &parts_db,
            crate::mcp::CreateBoardArgs { width_mm: 40.0, height_mm: 30.0, layers: 2, copper_weight_oz: 2, corner_radius_mm: 1.0 },
        );
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        match &screen {
            Screen::Editor(state) => {
                assert_eq!(state.doc.layer_count, LayerCount::Two);
                assert_eq!(state.doc.copper_weight, CopperWeight::TwoOz, "the requested copper weight must reach the created board");
                assert!(!state.templates.is_empty(), "built-in templates must be loaded for the freshly created board");
            }
            Screen::NewBoard(_) => panic!("expected create_board_write to switch to Screen::Editor"),
        }
    }

    #[test]
    fn create_board_write_refuses_to_replace_an_already_open_board() {
        let mut screen = Screen::Editor(test_editor_state());
        let parts_db = test_parts_db();
        let json = create_board_write(
            &mut screen,
            &parts_db,
            crate::mcp::CreateBoardArgs { width_mm: 40.0, height_mm: 30.0, layers: 2, copper_weight_oz: 1, corner_radius_mm: 1.0 },
        );
        assert!(json["error"].as_str().unwrap().contains("already open"), "unexpected response: {json}");
        assert!(matches!(screen, Screen::Editor(_)), "the already-open board must be left untouched");
    }

    #[test]
    fn create_board_write_rejects_an_invalid_layer_count() {
        let mut screen = Screen::NewBoard(NewBoardParams::default());
        let parts_db = test_parts_db();
        let json = create_board_write(
            &mut screen,
            &parts_db,
            crate::mcp::CreateBoardArgs { width_mm: 40.0, height_mm: 30.0, layers: 3, copper_weight_oz: 1, corner_radius_mm: 1.0 },
        );
        assert!(json["error"].as_str().unwrap().contains("must be 1 or 2"), "unexpected response: {json}");
    }

    #[test]
    fn create_board_write_rejects_an_invalid_copper_weight() {
        let mut screen = Screen::NewBoard(NewBoardParams::default());
        let parts_db = test_parts_db();
        let json = create_board_write(
            &mut screen,
            &parts_db,
            crate::mcp::CreateBoardArgs { width_mm: 40.0, height_mm: 30.0, layers: 2, copper_weight_oz: 3, corner_radius_mm: 1.0 },
        );
        assert!(json["error"].as_str().unwrap().contains("copper_weight_oz=3"), "unexpected response: {json}");
    }

    #[test]
    fn place_footprint_write_places_and_returns_the_new_reference() {
        let mut screen = Screen::Editor(test_editor_state());
        let template_name = two_pin_template().name;
        let json = place_footprint_write(&mut screen, crate::mcp::PlaceFootprintArgs { template: template_name, x_mm: 0.0, y_mm: 0.0, rotation_deg: 0.0 });
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["reference"], "P1");
        let Screen::Editor(state) = &screen else { panic!("board must still be open") };
        assert_eq!(state.doc.footprints.len(), 1);
    }

    #[test]
    fn place_footprint_write_rejects_an_unknown_template() {
        let mut screen = Screen::Editor(test_editor_state());
        let json = place_footprint_write(&mut screen, crate::mcp::PlaceFootprintArgs { template: "no-such-template".to_string(), x_mm: 0.0, y_mm: 0.0, rotation_deg: 0.0 });
        assert!(json["error"].as_str().unwrap().contains("unknown template"), "unexpected response: {json}");
    }

    #[test]
    fn place_footprint_write_reports_no_board_open_on_the_new_board_screen() {
        let mut screen = Screen::NewBoard(NewBoardParams::default());
        let json = place_footprint_write(&mut screen, crate::mcp::PlaceFootprintArgs { template: "anything".to_string(), x_mm: 0.0, y_mm: 0.0, rotation_deg: 0.0 });
        assert!(json["error"].as_str().unwrap().contains("no board is open"), "unexpected response: {json}");
    }

    #[test]
    fn register_part_write_inserts_a_mounting_hole_and_it_becomes_placeable() {
        let mut screen = Screen::Editor(test_editor_state());
        let parts_db = test_parts_db();
        let before = if let Screen::Editor(state) = &screen { state.templates.len() } else { 0 };
        let json = register_part_write(
            &mut screen,
            &parts_db,
            crate::mcp::RegisterPartArgs {
                name: "Test mounting hole".to_string(),
                reference_prefix: "H".to_string(),
                description: String::new(),
                pin_count: None,
                pitch_mm: 2.54,
                pad_radius_mm: 0.45,
                hole_diameter_mm: Some(3.2),
                exclude_from_bom: true,
                category: String::new(),
            },
        );
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["template"], "Test mounting hole");
        let Screen::Editor(state) = &screen else { panic!("board must still be open") };
        assert_eq!(state.templates.len(), before + 1, "the new template must be appended for immediate place_footprint use");
    }

    #[test]
    fn group_templates_by_category_puts_built_ins_and_uncategorized_db_parts_apart_from_real_categories() {
        let origin = vec![None, Some(1), Some(2), Some(3)];
        let category = vec![None, None, Some("Resistors".to_string()), Some("Imported files/board_a".to_string())];
        let tree = group_templates_by_category(&origin, &category);

        assert!(!tree.contains_key(""), "a built-in template (origin == None) must never show up in the category tree at all");
        assert_eq!(tree["Uncategorized"][""], vec![1], "a database-backed part with no category must land under a plain \"Uncategorized\" bucket");
        assert_eq!(tree["Resistors"][""], vec![2], "a plain, one-level category has no sub-bucket");
        assert_eq!(tree["Imported files"]["board_a"], vec![3], "a \"Top/Sub\" category must split into a nested sub-bucket");
    }

    #[test]
    fn register_part_write_files_the_new_template_under_its_given_category() {
        let mut screen = Screen::Editor(test_editor_state());
        let parts_db = test_parts_db();
        let json = register_part_write(
            &mut screen,
            &parts_db,
            crate::mcp::RegisterPartArgs {
                name: "Categorized Part".to_string(),
                reference_prefix: "U".to_string(),
                description: String::new(),
                pin_count: Some(2),
                pitch_mm: 2.54,
                pad_radius_mm: 0.45,
                hole_diameter_mm: None,
                exclude_from_bom: false,
                category: "Custom".to_string(),
            },
        );
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        let Screen::Editor(state) = &screen else { panic!("board must still be open") };
        let index = state.templates.iter().position(|t| t.name == "Categorized Part").expect("the new template must be in the live list");
        assert_eq!(state.template_category[index], Some("Custom".to_string()));
        assert_eq!(parts_db.list_categories().unwrap(), vec!["Custom".to_string()]);
    }

    #[test]
    fn apply_confirmed_delete_part_removes_it_from_both_the_database_and_the_live_template_list() {
        let mut screen = Screen::Editor(test_editor_state());
        let parts_db = test_parts_db();
        register_part_write(
            &mut screen,
            &parts_db,
            crate::mcp::RegisterPartArgs {
                name: "Doomed Part".to_string(),
                reference_prefix: "U".to_string(),
                description: String::new(),
                pin_count: Some(2),
                pitch_mm: 2.54,
                pad_radius_mm: 0.45,
                hole_diameter_mm: None,
                exclude_from_bom: false,
                category: String::new(),
            },
        );
        let Screen::Editor(state) = &mut screen else { panic!("board must still be open") };
        let index = state.templates.iter().position(|t| t.name == "Doomed Part").expect("the new template must be in the live list");
        let db_id = state.template_origin[index].expect("a register_part-created template is always database-backed");
        let before = state.templates.len();

        apply_confirmed_delete(state, &parts_db, PendingDelete::Part { index, db_id, name: "Doomed Part".to_string() });

        assert_eq!(state.templates.len(), before - 1, "the deleted template must be gone from the live list too, not just the database");
        assert!(state.templates.iter().all(|t| t.name != "Doomed Part"));
        assert!(parts_db.get_part(db_id).is_err(), "the row must actually be gone from the database");
    }

    #[test]
    fn apply_confirmed_delete_category_removes_every_part_under_it_and_reloads_the_template_list() {
        let mut screen = Screen::Editor(test_editor_state());
        let parts_db = test_parts_db();
        for name in ["Res A", "Res B"] {
            register_part_write(
                &mut screen,
                &parts_db,
                crate::mcp::RegisterPartArgs {
                    name: name.to_string(),
                    reference_prefix: "R".to_string(),
                    description: String::new(),
                    pin_count: Some(2),
                    pitch_mm: 2.54,
                    pad_radius_mm: 0.45,
                    hole_diameter_mm: None,
                    exclude_from_bom: false,
                    category: "Resistors".to_string(),
                },
            );
        }
        let Screen::Editor(state) = &mut screen else { panic!("board must still be open") };

        apply_confirmed_delete(state, &parts_db, PendingDelete::Category { prefix: "Resistors".to_string(), count: 2 });

        assert!(state.templates.iter().all(|t| t.name != "Res A" && t.name != "Res B"), "every part under the deleted category must be gone from the live list");
        assert!(parts_db.list_categories().unwrap().is_empty(), "the category itself must be gone from the database too");
        assert!(state.io_message.as_deref().unwrap_or_default().contains('2'), "the confirmation message must say how many parts were actually deleted");
    }

    #[test]
    fn register_part_write_rejects_when_both_shape_flags_are_given() {
        let mut screen = Screen::Editor(test_editor_state());
        let parts_db = test_parts_db();
        let json = register_part_write(
            &mut screen,
            &parts_db,
            crate::mcp::RegisterPartArgs {
                name: "Bad part".to_string(),
                reference_prefix: "U".to_string(),
                description: String::new(),
                pin_count: Some(2),
                pitch_mm: 2.54,
                pad_radius_mm: 0.45,
                hole_diameter_mm: Some(3.2),
                exclude_from_bom: false,
                category: String::new(),
            },
        );
        assert!(json["error"].as_str().unwrap().contains("mutually exclusive"), "unexpected response: {json}");
    }

    #[test]
    fn connect_pins_write_joins_two_pads_and_returns_the_net_name() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(-mm(5.0), 0), 0.0).unwrap();
            state.doc.try_place_footprint(&template, Point::new(mm(5.0), 0), 0.0).unwrap();
        }
        let json = connect_pins_write(&mut screen, crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string() });
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["net"], "Net1");
    }

    #[test]
    fn connect_pins_write_rejects_an_unknown_pin() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        }
        let json = connect_pins_write(&mut screen, crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "99".to_string(), ref2: "P1".to_string(), pin2: "1".to_string() });
        assert!(json["error"].as_str().unwrap().contains("no such pin"), "unexpected response: {json}");
    }

    #[test]
    fn route_pins_write_routes_two_connected_pins_and_reports_a_positive_leg_count() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            // Rotated 90 degrees for the same reason as
            // `crate::cli`'s own `route_pins_succeeds_...` test: keeps
            // each part's second, unconnected pad off the straight line
            // between the two connected first pads.
            state.doc.try_place_footprint(&template, Point::new(-mm(10.0), 0), 90.0).unwrap();
            state.doc.try_place_footprint(&template, Point::new(mm(10.0), 0), 90.0).unwrap();
        }
        connect_pins_write(&mut screen, crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string() });

        let json = route_pins_write(&mut screen, crate::mcp::RoutePinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string(), width_mm: 0.25 });
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert!(json["track_leg_count"].as_u64().unwrap() >= 1);
    }

    #[test]
    fn route_pins_write_rejects_pins_that_are_not_yet_connected() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(-mm(10.0), 0), 0.0).unwrap();
            state.doc.try_place_footprint(&template, Point::new(mm(10.0), 0), 0.0).unwrap();
        }
        let json = route_pins_write(&mut screen, crate::mcp::RoutePinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string(), width_mm: 0.25 });
        assert!(json["error"].as_str().unwrap().contains("couldn't route"), "unexpected response: {json}");
    }

    // -- Manual routing drag (start_route/route_to/fix_corner/undo_last_corner/finish_route/cancel_route) --

    /// Two connected pins 20mm apart, rotated 90 degrees like
    /// `route_pins_write`'s own tests, so the second, unconnected pad on
    /// each footprint never sits on the straight line between the two
    /// connected first pads.
    fn two_connected_pins_20mm_apart() -> Screen {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(-mm(10.0), 0), 90.0).unwrap();
            state.doc.try_place_footprint(&template, Point::new(mm(10.0), 0), 90.0).unwrap();
        }
        connect_pins_write(&mut screen, crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string() });
        screen
    }

    #[test]
    fn start_route_write_starts_a_drag_from_a_connected_pin() {
        let mut screen = two_connected_pins_20mm_apart();
        let json = start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["net"], "Net1");
        assert_eq!(json["from_pad"]["footprint"], "P1");
        match &screen {
            Screen::Editor(state) => assert!(state.routing.is_some(), "the drag must now be live in EditorState"),
            Screen::NewBoard(_) => panic!("expected Screen::Editor"),
        }
    }

    #[test]
    fn start_route_write_rejects_a_pin_with_no_net_yet() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        }
        let json = start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        assert!(json["error"].as_str().unwrap().contains("no net"), "unexpected response: {json}");
    }

    #[test]
    fn start_route_write_refuses_when_a_route_is_already_in_progress() {
        let mut screen = two_connected_pins_20mm_apart();
        start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        let json = start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P2".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        assert!(json["error"].as_str().unwrap().contains("already in progress"), "unexpected response: {json}");
    }

    #[test]
    fn route_to_write_rejects_when_no_route_in_progress() {
        let mut screen = two_connected_pins_20mm_apart();
        let json = route_to_write(&mut screen, crate::mcp::RouteToArgs { x_mm: 0.0, y_mm: 0.0 });
        assert!(json["error"].as_str().unwrap().contains("no route in progress"), "unexpected response: {json}");
    }

    #[test]
    fn route_to_write_reports_a_clear_live_leg_towards_an_open_point() {
        let mut screen = two_connected_pins_20mm_apart();
        start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        let json = route_to_write(&mut screen, crate::mcp::RouteToArgs { x_mm: -5.0, y_mm: 0.0 });
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["live_end_clear"], true, "unexpected response: {json}");
        assert!(json["blocked_reason"].is_null(), "unexpected response: {json}");
    }

    #[test]
    fn fix_corner_write_rejects_when_no_route_in_progress() {
        let mut screen = two_connected_pins_20mm_apart();
        let json = fix_corner_write(&mut screen);
        assert!(json["error"].as_str().unwrap().contains("no route in progress"), "unexpected response: {json}");
    }

    #[test]
    fn fix_corner_write_fixes_a_clear_leg_and_increments_the_fixed_corner_count() {
        let mut screen = two_connected_pins_20mm_apart();
        start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        route_to_write(&mut screen, crate::mcp::RouteToArgs { x_mm: -5.0, y_mm: 0.0 });
        let json = fix_corner_write(&mut screen);
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["fixed_corner_count"], 1);
    }

    #[test]
    fn fix_corner_write_rejects_a_leg_that_has_not_moved_from_the_last_fixed_point_yet() {
        let mut screen = two_connected_pins_20mm_apart();
        start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        // No `route_to` at all -- the live leg is still empty.
        let json = fix_corner_write(&mut screen);
        assert!(json["error"].as_str().unwrap().contains("can't fix a corner"), "unexpected response: {json}");
    }

    #[test]
    fn undo_last_corner_write_rejects_when_nothing_is_fixed_yet() {
        let mut screen = two_connected_pins_20mm_apart();
        start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        let json = undo_last_corner_write(&mut screen);
        assert!(json["error"].as_str().unwrap().contains("no fixed corner"), "unexpected response: {json}");
    }

    #[test]
    fn undo_last_corner_write_undoes_a_previously_fixed_corner() {
        let mut screen = two_connected_pins_20mm_apart();
        start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        route_to_write(&mut screen, crate::mcp::RouteToArgs { x_mm: -5.0, y_mm: 0.0 });
        fix_corner_write(&mut screen);

        let json = undo_last_corner_write(&mut screen);
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["fixed_corner_count"], 0);
    }

    #[test]
    fn finish_route_write_rejects_when_not_docked_onto_a_same_net_target_and_keeps_the_drag_alive() {
        let mut screen = two_connected_pins_20mm_apart();
        start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        route_to_write(&mut screen, crate::mcp::RouteToArgs { x_mm: -5.0, y_mm: 0.0 });

        let json = finish_route_write(&mut screen);
        assert!(json["error"].is_string(), "unexpected response: {json}");
        match &screen {
            Screen::Editor(state) => assert!(state.routing.is_some(), "a failed finish_route must keep the drag alive"),
            Screen::NewBoard(_) => panic!("expected Screen::Editor"),
        }
    }

    #[test]
    fn finish_route_write_commits_a_route_docked_onto_the_target_pin() {
        let mut screen = two_connected_pins_20mm_apart();
        start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        // Straight onto P2's pin 1 (offset -1.27mm off the footprint's own
        // placement y by the 90-degree rotation, same as P1's) -- same
        // net, so `route_to` docks onto it.
        route_to_write(&mut screen, crate::mcp::RouteToArgs { x_mm: 10.0, y_mm: -1.27 });

        let json = finish_route_write(&mut screen);
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["net"], "Net1");
        match &screen {
            Screen::Editor(state) => {
                assert!(state.routing.is_none(), "a successful finish_route must clear the drag");
                assert!(state.doc.node.iter().any(|item| matches!(item, Item::Track { .. })), "a real track must now exist on the board");
            }
            Screen::NewBoard(_) => panic!("expected Screen::Editor"),
        }
    }

    #[test]
    fn finish_route_write_rejects_when_no_route_in_progress() {
        let mut screen = two_connected_pins_20mm_apart();
        let json = finish_route_write(&mut screen);
        assert!(json["error"].as_str().unwrap().contains("no route in progress"), "unexpected response: {json}");
    }

    #[test]
    fn cancel_route_write_clears_an_in_progress_route() {
        let mut screen = two_connected_pins_20mm_apart();
        start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        let json = cancel_route_write(&mut screen);
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["was_active"], true);
        match &screen {
            Screen::Editor(state) => assert!(state.routing.is_none()),
            Screen::NewBoard(_) => panic!("expected Screen::Editor"),
        }
    }

    #[test]
    fn cancel_route_write_is_a_harmless_no_op_with_nothing_in_progress() {
        let mut screen = two_connected_pins_20mm_apart();
        let json = cancel_route_write(&mut screen);
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["was_active"], false);
    }

    #[test]
    fn drop_via_and_switch_layer_write_rejects_when_no_route_in_progress() {
        let mut screen = two_connected_pins_20mm_apart();
        let json = drop_via_and_switch_layer_write(&mut screen);
        assert!(json["error"].as_str().unwrap().contains("no route in progress"), "unexpected response: {json}");
    }

    #[test]
    fn drop_via_and_switch_layer_write_rejects_when_the_live_leg_has_not_moved_yet() {
        let mut screen = two_connected_pins_20mm_apart();
        start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        // No `route_to` at all -- no usable live leg to drop a via onto.
        let json = drop_via_and_switch_layer_write(&mut screen);
        assert!(json["error"].as_str().unwrap().contains("no clear route"), "unexpected response: {json}");
    }

    #[test]
    fn drop_via_and_switch_layer_write_commits_a_track_adds_a_via_and_flips_the_drags_layer() {
        let mut screen = two_connected_pins_20mm_apart();
        start_route_write(&mut screen, crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 });
        route_to_write(&mut screen, crate::mcp::RouteToArgs { x_mm: -5.0, y_mm: -1.27 });

        let json = drop_via_and_switch_layer_write(&mut screen);
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        match &screen {
            Screen::Editor(state) => {
                assert!(state.routing.is_some(), "the drag must continue on the other layer");
                assert!(state.doc.node.iter().any(|item| matches!(item, Item::Via { .. })), "a real via must now exist on the board");
                assert!(state.doc.node.iter().any(|item| matches!(item, Item::Track { .. })), "the leg up to the via must be committed as a track");
            }
            Screen::NewBoard(_) => panic!("expected Screen::Editor"),
        }
    }

    #[test]
    fn run_batch_write_drags_a_manual_route_start_to_finish_across_several_operations() {
        let mut screen = two_connected_pins_20mm_apart();
        let mut parts_db = test_parts_db();

        let json = run_batch_write(
            &mut screen,
            &mut parts_db,
            crate::mcp::RunBatchArgs {
                operations: vec![
                    crate::mcp::BatchOp::StartRoute(crate::mcp::StartRouteArgs { reference: "P1".to_string(), pin: "1".to_string(), width_mm: 0.25, via_diameter_mm: 0.6, via_drill_mm: 0.3 }),
                    crate::mcp::BatchOp::RouteTo(crate::mcp::RouteToArgs { x_mm: -5.0, y_mm: -1.27 }),
                    crate::mcp::BatchOp::FixCorner,
                    crate::mcp::BatchOp::RouteTo(crate::mcp::RouteToArgs { x_mm: 10.0, y_mm: -1.27 }),
                    crate::mcp::BatchOp::FinishRoute,
                ],
                stop_on_error: true,
            },
        );

        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["ok_count"], 5);
        assert_eq!(json["error_count"], 0);
        match &screen {
            Screen::Editor(state) => {
                assert!(state.routing.is_none());
                assert!(state.doc.node.iter().any(|item| matches!(item, Item::Track { .. })), "a real track must now exist on the board");
            }
            Screen::NewBoard(_) => panic!("expected Screen::Editor"),
        }
    }

    #[test]
    fn add_via_write_touching_its_own_net_succeeds() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(-mm(10.0), 0), 0.0).unwrap();
            state.doc.try_place_footprint(&template, Point::new(mm(10.0), 0), 0.0).unwrap();
        }
        connect_pins_write(&mut screen, crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string() });

        // Right on top of P1's own pin "1" pad, same geometry as
        // `crate::cli`'s `add_via_touching_its_own_net_succeeds_...`.
        let json = add_via_write(&mut screen, crate::mcp::AddViaArgs { net: "Net1".to_string(), x_mm: -11.27, y_mm: 0.0, diameter_mm: 0.6, drill_mm: 0.3 });
        assert_eq!(json["ok"], true, "unexpected response: {json}");
    }

    #[test]
    fn add_via_write_rejects_an_unknown_net() {
        let mut screen = Screen::Editor(test_editor_state());
        let json = add_via_write(&mut screen, crate::mcp::AddViaArgs { net: "no-such-net".to_string(), x_mm: 0.0, y_mm: 0.0, diameter_mm: 0.6, drill_mm: 0.3 });
        assert!(json["error"].as_str().unwrap().contains("unknown net"), "unexpected response: {json}");
    }

    #[test]
    fn add_pin_stitching_via_write_places_a_via_and_stub_next_to_a_connected_pin() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(-mm(10.0), 0), 0.0).unwrap();
            state.doc.try_place_footprint(&template, Point::new(mm(10.0), 0), 0.0).unwrap();
        }
        connect_pins_write(&mut screen, crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string() });

        let json = add_pin_stitching_via_write(
            &mut screen,
            crate::mcp::AddPinStitchingViaArgs { reference: "P1".to_string(), pin: "1".to_string(), diameter_mm: 0.6, drill_mm: 0.3, stub_width_mm: 0.25 },
        );
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert!(json["via_id"].is_u64(), "unexpected response: {json}");
        // P1 pin "1" sits at world x = -11.27mm; the via must land
        // further out (more negative x), away from the part's body.
        assert!(json["x_mm"].as_f64().unwrap() < -11.27, "unexpected response: {json}");
    }

    #[test]
    fn add_pin_stitching_via_write_rejects_an_unknown_pin() {
        let mut screen = Screen::Editor(test_editor_state());
        let json =
            add_pin_stitching_via_write(&mut screen, crate::mcp::AddPinStitchingViaArgs { reference: "P1".to_string(), pin: "1".to_string(), diameter_mm: 0.6, drill_mm: 0.3, stub_width_mm: 0.25 });
        assert!(json["error"].as_str().unwrap().contains("no such pin"), "unexpected response: {json}");
    }

    #[test]
    fn add_zone_write_fills_at_least_one_island() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(-mm(15.0), -mm(15.0)), 0.0).unwrap();
            state.doc.try_place_footprint(&template, Point::new(-mm(15.0), mm(15.0)), 0.0).unwrap();
        }
        connect_pins_write(&mut screen, crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string() });

        let points = vec![
            crate::mcp::PointMmArg { x_mm: -10.0, y_mm: -10.0 },
            crate::mcp::PointMmArg { x_mm: 10.0, y_mm: -10.0 },
            crate::mcp::PointMmArg { x_mm: 10.0, y_mm: 10.0 },
            crate::mcp::PointMmArg { x_mm: -10.0, y_mm: 10.0 },
        ];
        let json = add_zone_write(&mut screen, crate::mcp::AddZoneArgs { net: "Net1".to_string(), layer: "front".to_string(), points });
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert!(json["filled_islands"].as_u64().unwrap() >= 1);
        let Screen::Editor(state) = &screen else { panic!("board must still be open") };
        assert_eq!(state.doc.zones.len(), 1);
    }

    #[test]
    fn add_zone_write_rejects_an_invalid_layer_name() {
        let mut screen = Screen::Editor(test_editor_state());
        let points = vec![crate::mcp::PointMmArg { x_mm: 0.0, y_mm: 0.0 }, crate::mcp::PointMmArg { x_mm: 1.0, y_mm: 0.0 }, crate::mcp::PointMmArg { x_mm: 1.0, y_mm: 1.0 }];
        let json = add_zone_write(&mut screen, crate::mcp::AddZoneArgs { net: "Net1".to_string(), layer: "top".to_string(), points });
        assert!(json["error"].as_str().unwrap().contains("must be \"front\" or \"back\""), "unexpected response: {json}");
    }

    #[test]
    fn add_silk_text_write_places_a_text_in_open_space() {
        let mut screen = Screen::Editor(test_editor_state());
        let json = add_silk_text_write(&mut screen, crate::mcp::AddSilkTextArgs { text: "REV A".to_string(), x_mm: 0.0, y_mm: 0.0, rotation_deg: 0.0, layer: "front".to_string(), height_mm: 1.0 });
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert!(json["silk_text_id"].is_u64(), "unexpected response: {json}");
        let Screen::Editor(state) = &screen else { panic!("board must still be open") };
        assert_eq!(state.doc.silk_texts.len(), 1);
        assert_eq!(state.doc.silk_texts[0].text, "REV A");
    }

    #[test]
    fn add_silk_text_write_honours_an_explicit_height_mm() {
        let mut screen = Screen::Editor(test_editor_state());
        let json = add_silk_text_write(
            &mut screen,
            crate::mcp::AddSilkTextArgs { text: "BIG".to_string(), x_mm: 0.0, y_mm: 0.0, rotation_deg: 0.0, layer: "front".to_string(), height_mm: 3.0 },
        );
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        let Screen::Editor(state) = &screen else { panic!("board must still be open") };
        assert_eq!(state.doc.silk_texts[0].height, mm(3.0));
    }

    #[test]
    fn add_silk_text_write_rejects_an_invalid_layer_name() {
        let mut screen = Screen::Editor(test_editor_state());
        let json = add_silk_text_write(&mut screen, crate::mcp::AddSilkTextArgs { text: "X".to_string(), x_mm: 0.0, y_mm: 0.0, rotation_deg: 0.0, layer: "top".to_string(), height_mm: 1.0 });
        assert!(json["error"].as_str().unwrap().contains("must be \"front\" or \"back\""), "unexpected response: {json}");
    }

    #[test]
    fn add_silk_text_write_reports_a_pad_collision_instead_of_placing_anyway() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        }
        let json = add_silk_text_write(&mut screen, crate::mcp::AddSilkTextArgs { text: "X".to_string(), x_mm: 1.27, y_mm: 0.0, rotation_deg: 0.0, layer: "front".to_string(), height_mm: 1.0 });
        assert!(json["error"].as_str().unwrap().contains("couldn't place silk text"), "unexpected response: {json}");
        let Screen::Editor(state) = &screen else { panic!("board must still be open") };
        assert!(state.doc.silk_texts.is_empty(), "a refused placement must not be added");
    }

    #[test]
    fn add_silk_text_write_refuses_with_no_board_open() {
        let mut screen = Screen::NewBoard(NewBoardParams::default());
        let json = add_silk_text_write(&mut screen, crate::mcp::AddSilkTextArgs { text: "X".to_string(), x_mm: 0.0, y_mm: 0.0, rotation_deg: 0.0, layer: "front".to_string(), height_mm: 1.0 });
        assert!(json["error"].is_string(), "unexpected response: {json}");
    }

    // -- Background job dispatch (`try_start_background_job`) --
    //
    // Exercises the actual `crate::background::BackgroundJob` thread
    // hop these go through -- unlike every `*_write` test above (which
    // call that function directly, synchronously), these route through
    // `try_start_background_job` exactly like `PcbApp::ui`'s own MCP
    // dispatch does, so a bug in the clone/spawn/re-validate/merge-back
    // plumbing itself (not just the underlying `*_write` logic, which
    // is untouched) would actually be caught here.

    /// Polls `pending.job` in a tight sleep loop until it resolves --
    /// standing in for `PcbApp::ui`'s own once-per-frame poll,
    /// compressed into a single blocking call since these tests have
    /// no event loop of their own. Panics if it's still pending after a
    /// generous timeout: every job here is microseconds of real work on
    /// a tiny test board, so taking anywhere near this long must mean a
    /// genuine bug, not just a slow CI machine.
    fn wait_for_job_result(pending: &mut PendingJob) -> JobResult {
        for _ in 0..2000 {
            match pending.job.poll() {
                JobPoll::Ready(apply) => return apply,
                JobPoll::Pending => std::thread::sleep(std::time::Duration::from_millis(5)),
                JobPoll::Lost => panic!("background job ended unexpectedly -- the worker thread must have panicked"),
            }
        }
        panic!("background job never resolved within the timeout");
    }

    #[test]
    fn try_start_background_job_runs_add_zone_off_thread_and_its_result_lands_on_the_board_once_applied() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(-mm(15.0), -mm(15.0)), 0.0).unwrap();
            state.doc.try_place_footprint(&template, Point::new(-mm(15.0), mm(15.0)), 0.0).unwrap();
        }
        connect_pins_write(&mut screen, crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string() });

        let points = vec![
            crate::mcp::PointMmArg { x_mm: -10.0, y_mm: -10.0 },
            crate::mcp::PointMmArg { x_mm: 10.0, y_mm: -10.0 },
            crate::mcp::PointMmArg { x_mm: 10.0, y_mm: 10.0 },
            crate::mcp::PointMmArg { x_mm: -10.0, y_mm: 10.0 },
        ];
        let (reply, mut rx) = oneshot::channel();
        let query = crate::mcp::McpQuery::AddZone { args: crate::mcp::AddZoneArgs { net: "Net1".to_string(), layer: "front".to_string(), points }, reply };
        let parts_db = PartsDb::open_in_memory().unwrap();
        // `McpQuery` has no `Debug` impl (it embeds a `oneshot::Sender`),
        // so this can't just be a `.expect(...)` on the `Result`.
        let Ok(mut pending) = try_start_background_job(query, &mut screen, &parts_db) else { panic!("AddZone must always background") };
        assert_eq!(pending.label, "zone fill");

        let apply = wait_for_job_result(&mut pending);
        let text = apply(&mut screen);
        let json: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert!(json["filled_islands"].as_u64().unwrap() >= 1);
        let Screen::Editor(state) = &screen else { panic!("board must still be open") };
        assert_eq!(state.doc.zones.len(), 1);

        pending.reply.take().unwrap().send(text).unwrap();
        assert!(rx.try_recv().is_ok(), "the oneshot reply channel must have received the same JSON text");
    }

    #[test]
    fn try_start_background_job_reports_an_unknown_net_immediately_without_ever_spawning_a_thread() {
        let mut screen = Screen::Editor(test_editor_state());
        let points = vec![crate::mcp::PointMmArg { x_mm: 0.0, y_mm: 0.0 }, crate::mcp::PointMmArg { x_mm: 1.0, y_mm: 0.0 }, crate::mcp::PointMmArg { x_mm: 1.0, y_mm: 1.0 }];
        let (reply, _rx) = oneshot::channel();
        let query = crate::mcp::McpQuery::AddZone { args: crate::mcp::AddZoneArgs { net: "NoSuchNet".to_string(), layer: "front".to_string(), points }, reply };
        let parts_db = PartsDb::open_in_memory().unwrap();
        let Ok(mut pending) = try_start_background_job(query, &mut screen, &parts_db) else { panic!("AddZone must always background (even a validation failure)") };

        // `PendingJob::immediate` resolves on the very first poll --
        // no sleep/retry loop needed, unlike a real fill above. `JobPoll`
        // has no `Debug` impl either (it wraps the same non-`Debug`
        // `JobResult` closure type), hence the `panic!`-without-`{:?}`
        // arm below rather than a `.expect(...)`/`unwrap()`.
        let apply = match pending.job.poll() {
            JobPoll::Ready(apply) => apply,
            JobPoll::Pending => panic!("an upfront validation failure must resolve immediately, not stay Pending"),
            JobPoll::Lost => panic!("an upfront validation failure must resolve immediately, not report Lost"),
        };
        let text = apply(&mut screen);
        assert!(text.contains("unknown net"), "unexpected response: {text}");
    }

    #[test]
    fn try_start_background_job_runs_route_pins_off_thread_and_commits_the_track_once_applied() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(-mm(10.0), 0), 90.0).unwrap();
            state.doc.try_place_footprint(&template, Point::new(mm(10.0), 0), 90.0).unwrap();
        }
        connect_pins_write(&mut screen, crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string() });

        let (reply, _rx) = oneshot::channel();
        let args = crate::mcp::RoutePinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string(), width_mm: 0.25 };
        let query = crate::mcp::McpQuery::RoutePins { args, reply };
        let parts_db = PartsDb::open_in_memory().unwrap();
        let Ok(mut pending) = try_start_background_job(query, &mut screen, &parts_db) else { panic!("RoutePins must always background") };
        assert_eq!(pending.label, "routing search");

        let apply = wait_for_job_result(&mut pending);
        let text = apply(&mut screen);
        let json: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert!(json["track_leg_count"].as_u64().unwrap() >= 1);
        assert!(matches!(screen, Screen::Editor(_)), "board must still be open");
    }

    #[test]
    fn try_start_background_job_runs_run_batch_off_thread_and_merges_the_finished_board_back() {
        let mut screen = Screen::NewBoard(NewBoardParams::default());
        let create_args = crate::mcp::CreateBoardArgs { width_mm: 40.0, height_mm: 40.0, layers: 2, copper_weight_oz: 1, corner_radius_mm: 0.0 };
        let operations = vec![crate::mcp::BatchOp::CreateBoard(create_args)];
        let (reply, _rx) = oneshot::channel();
        let query = crate::mcp::McpQuery::RunBatch { args: crate::mcp::RunBatchArgs { operations, stop_on_error: true }, reply };
        let parts_db = PartsDb::open_in_memory().unwrap();
        let Ok(mut pending) = try_start_background_job(query, &mut screen, &parts_db) else { panic!("RunBatch must always background") };
        assert_eq!(pending.label, "batch operation");

        let apply = wait_for_job_result(&mut pending);
        let text = apply(&mut screen);
        let json: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert!(matches!(screen, Screen::Editor(_)), "the still-on-NewBoard live screen must have picked up the batch-created board");
    }

    #[test]
    fn rename_net_write_renames_a_net_findable_by_its_new_name_afterwards() {
        let mut screen = Screen::Editor(test_editor_state());
        let template = two_pin_template();
        if let Screen::Editor(state) = &mut screen {
            state.doc.try_place_footprint(&template, Point::new(-mm(15.0), -mm(15.0)), 0.0).unwrap();
            state.doc.try_place_footprint(&template, Point::new(-mm(15.0), mm(15.0)), 0.0).unwrap();
        }
        connect_pins_write(&mut screen, crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string() });

        let json = rename_net_write(&mut screen, crate::mcp::RenameNetArgs { net: "Net1".to_string(), new_name: "GND".to_string() });
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["new_name"], "GND");

        if let Screen::Editor(state) = &screen {
            assert!(state.doc.find_net_by_name("GND").is_some());
        }
    }

    #[test]
    fn rename_net_write_reports_an_unknown_net_name_as_an_error() {
        let mut screen = Screen::Editor(test_editor_state());
        let json = rename_net_write(&mut screen, crate::mcp::RenameNetArgs { net: "NoSuchNet".to_string(), new_name: "GND".to_string() });
        assert!(json["error"].as_str().unwrap().contains("unknown net"), "unexpected response: {json}");
    }

    #[test]
    fn refill_zones_write_reports_the_boards_zone_count() {
        let mut screen = Screen::Editor(test_editor_state());
        let json = refill_zones_write(&mut screen);
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["zone_count"], 0);
    }

    #[test]
    fn save_board_write_requires_a_path_the_first_time() {
        let mut screen = Screen::Editor(test_editor_state());
        let json = save_board_write(&mut screen, crate::mcp::SaveBoardArgs { path: None });
        assert!(json["error"].as_str().unwrap().contains("no path given"), "unexpected response: {json}");
    }

    #[test]
    fn save_board_write_saves_to_a_given_path_and_remembers_it_for_a_pathless_save() {
        let _pointer_guard = LastBoardPointerGuard::capture(); // restores this machine's real last-board pointer, however this test ends
        let dir = std::env::temp_dir().join(format!("alladin_pcb_mcp_write_test_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("board.alladinpcb.json");

        let mut screen = Screen::Editor(test_editor_state());
        let json = save_board_write(&mut screen, crate::mcp::SaveBoardArgs { path: Some(path.to_string_lossy().to_string()) });
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert!(path.exists(), "save_board_write must actually write the file");

        // A second, pathless save must reuse the just-remembered path.
        let json2 = save_board_write(&mut screen, crate::mcp::SaveBoardArgs { path: None });
        assert_eq!(json2["ok"], true, "unexpected response: {json2}");

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn export_manufacturing_files_write_produces_gerber_zip_cpl_and_bom_from_a_real_board() {
        let dir = std::env::temp_dir().join(format!("alladin_pcb_mcp_write_test_mfg_{}", std::process::id()));
        let out_dir = dir.join("out");

        let screen = Screen::Editor(test_editor_state());
        let bom = "Comment,Designator,Footprint,LCSC Part #\n";
        let json = export_manufacturing_files_write(
            &screen,
            bom,
            crate::mcp::ExportManufacturingFilesArgs { out_dir: out_dir.to_string_lossy().to_string() },
        );
        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["backend"], "native");
        assert!(std::path::Path::new(json["gerber_zip_path"].as_str().unwrap()).exists());
        assert!(std::path::Path::new(json["cpl_csv_path"].as_str().unwrap()).exists());
        assert!(std::path::Path::new(json["bom_csv_path"].as_str().unwrap()).exists());

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn start_external_autoroute_write_reports_no_board_open_on_the_new_board_screen() {
        let mut screen = Screen::NewBoard(NewBoardParams::default());
        let json = start_external_autoroute_write(&mut screen, crate::mcp::StartExternalAutorouteArgs { nets: Vec::new(), extra_args: None });
        assert!(json["error"].as_str().unwrap().contains("no board is open"), "unexpected response: {json}");
    }

    #[test]
    fn start_external_autoroute_write_refuses_an_unconfigured_tool_without_touching_the_board() {
        let mut screen = Screen::Editor(test_editor_state());
        // Force the in-memory default (empty `tool_dir`) regardless of
        // whatever `~/.config/alladin-pcb/external_router.json` this
        // *particular machine* happens to have on disk -- `EditorState::new`
        // loads real, persisted settings (see its own doc comment), so a
        // dev machine that's actually configured KiCadRoutingTools for
        // real (to test the feature end-to-end) would otherwise make
        // this exact "not configured yet" refusal path unreachable here.
        let Screen::Editor(state) = &mut screen else { unreachable!() };
        state.external_router_settings = crate::external_router::ExternalRouterSettings::default();
        let json = start_external_autoroute_write(&mut screen, crate::mcp::StartExternalAutorouteArgs { nets: Vec::new(), extra_args: None });
        assert!(json["error"].as_str().unwrap().contains("isn't configured"), "unexpected response: {json}");
        let Screen::Editor(state) = &screen else { panic!("board must still be open") };
        assert!(state.autoroute_job.is_none(), "a refused start must never leave a job behind");
    }

    #[test]
    fn external_autoroute_status_json_reports_idle_when_no_job_was_ever_started() {
        let mut screen = Screen::Editor(test_editor_state());
        let json = external_autoroute_status_json(&mut screen);
        assert_eq!(json["status"], "idle");
    }

    #[test]
    fn external_autoroute_status_json_on_the_new_board_screen_reports_no_board_open() {
        let mut screen = Screen::NewBoard(NewBoardParams::default());
        let json = external_autoroute_status_json(&mut screen);
        assert!(json["note"].as_str().unwrap().contains("no board is open"), "unexpected response: {json}");
    }

    /// End-to-end through the MCP handler layer, needing a real,
    /// locally configured `KiCadRoutingTools` checkout -- skipped,
    /// not failed, everywhere else, exactly like
    /// `crate::external_router::tests::run_autoroute_end_to_end_against_a_real_locally_configured_tool`
    /// (same `ALLADIN_KICAD_ROUTING_TOOLS_DIR` env var). Confirms the
    /// two MCP tools' own JSON shapes end to end, on top of what that
    /// lower-level test already covers for `run_autoroute` itself.
    #[test]
    fn start_and_poll_external_autoroute_write_reach_a_done_status_against_a_real_tool() {
        let Ok(tool_dir) = std::env::var("ALLADIN_KICAD_ROUTING_TOOLS_DIR") else {
            eprintln!("skipping: ALLADIN_KICAD_ROUTING_TOOLS_DIR not set");
            return;
        };
        let mut settings = crate::external_router::ExternalRouterSettings::default();
        settings.tool_dir = tool_dir;
        if !crate::external_router::diagnose(&settings).is_ready() {
            eprintln!("skipping: KiCadRoutingTools at {} isn't fully set up (see diagnose())", settings.tool_dir);
            return;
        }

        let mut state = test_editor_state();
        state.external_router_settings = settings;
        let template = two_pin_template();
        state.doc.try_place_footprint(&template, Point::new(-mm(10.0), 0), 0.0).unwrap();
        state.doc.try_place_footprint(&template, Point::new(mm(10.0), 0), 0.0).unwrap();
        let mut screen = Screen::Editor(state);
        connect_pins_write(&mut screen, crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string() });

        let start_json = start_external_autoroute_write(&mut screen, crate::mcp::StartExternalAutorouteArgs { nets: vec!["Net1".to_string()], extra_args: None });
        assert_eq!(start_json["ok"], true, "unexpected response: {start_json}");

        let status_json = loop {
            let json = external_autoroute_status_json(&mut screen);
            if json["status"] != "running" {
                break json;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        };
        assert_eq!(status_json["status"], "done", "unexpected response: {status_json}");
        assert_eq!(status_json["routed_nets"].as_array().unwrap(), &vec![serde_json::json!("Net1")]);
        assert!(status_json["item_count"].as_u64().unwrap() >= 1);
    }

    #[test]
    fn download_lcsc_part_write_reports_a_network_error_without_touching_the_board() {
        let mut screen = Screen::Editor(test_editor_state());
        let parts_db = test_parts_db();
        let json = download_lcsc_part_write(&mut screen, &parts_db, Err(crate::lcsc::FetchError::NotFound("C999999".to_string())));
        assert!(json["error"].is_string(), "unexpected response: {json}");
        let Screen::Editor(state) = &screen else { panic!("board must still be open") };
        assert!(state.template_origin.iter().all(Option::is_none), "no part must have been inserted on a failed fetch");
    }

    #[test]
    fn run_batch_write_places_two_footprints_and_connects_them_in_one_call() {
        let mut screen = Screen::Editor(test_editor_state());
        let mut parts_db = test_parts_db();
        let template_name = two_pin_template().name;

        let json = run_batch_write(
            &mut screen,
            &mut parts_db,
            crate::mcp::RunBatchArgs {
                operations: vec![
                    crate::mcp::BatchOp::PlaceFootprint(crate::mcp::PlaceFootprintArgs { template: template_name.clone(), x_mm: -5.0, y_mm: 0.0, rotation_deg: 0.0 }),
                    crate::mcp::BatchOp::PlaceFootprint(crate::mcp::PlaceFootprintArgs { template: template_name, x_mm: 5.0, y_mm: 0.0, rotation_deg: 0.0 }),
                    crate::mcp::BatchOp::ConnectPins(crate::mcp::ConnectPinsArgs { ref1: "P1".to_string(), pin1: "1".to_string(), ref2: "P2".to_string(), pin2: "1".to_string() }),
                    crate::mcp::BatchOp::RefillZones,
                ],
                stop_on_error: true,
            },
        );

        assert_eq!(json["ok"], true, "unexpected response: {json}");
        assert_eq!(json["ok_count"], 4);
        assert_eq!(json["error_count"], 0);
        assert_eq!(json["stopped_early"], false);
        let results = json["results"].as_array().expect("results must be an array");
        assert_eq!(results.len(), 4);
        assert_eq!(results[0]["tool"], "place_footprint");
        assert_eq!(results[0]["reference"], "P1");
        assert_eq!(results[2]["tool"], "connect_pins");
        assert_eq!(results[2]["net"], "Net1");
        let Screen::Editor(state) = &screen else { panic!("board must still be open") };
        assert_eq!(state.doc.footprints.len(), 2);
    }

    #[test]
    fn run_batch_write_stops_at_the_first_error_by_default_and_marks_the_rest_skipped() {
        let mut screen = Screen::Editor(test_editor_state());
        let mut parts_db = test_parts_db();

        let json = run_batch_write(
            &mut screen,
            &mut parts_db,
            crate::mcp::RunBatchArgs {
                operations: vec![
                    crate::mcp::BatchOp::PlaceFootprint(crate::mcp::PlaceFootprintArgs { template: "no-such-template".to_string(), x_mm: 0.0, y_mm: 0.0, rotation_deg: 0.0 }),
                    crate::mcp::BatchOp::RefillZones,
                ],
                stop_on_error: true,
            },
        );

        assert_eq!(json["ok"], false, "unexpected response: {json}");
        assert_eq!(json["ok_count"], 0);
        assert_eq!(json["error_count"], 1);
        assert_eq!(json["stopped_early"], true);
        let results = json["results"].as_array().expect("results must be an array");
        assert!(results[0]["error"].is_string(), "unexpected response: {json}");
        assert_eq!(results[1]["skipped"], true);
        assert_eq!(results[1]["tool"], "refill_zones");
    }

    #[test]
    fn run_batch_write_with_stop_on_error_false_runs_every_operation_and_collects_every_result() {
        let mut screen = Screen::Editor(test_editor_state());
        let mut parts_db = test_parts_db();

        let json = run_batch_write(
            &mut screen,
            &mut parts_db,
            crate::mcp::RunBatchArgs {
                operations: vec![
                    crate::mcp::BatchOp::PlaceFootprint(crate::mcp::PlaceFootprintArgs { template: "no-such-template".to_string(), x_mm: 0.0, y_mm: 0.0, rotation_deg: 0.0 }),
                    crate::mcp::BatchOp::RefillZones,
                ],
                stop_on_error: false,
            },
        );

        assert_eq!(json["ok"], false, "unexpected response: {json}");
        assert_eq!(json["ok_count"], 1);
        assert_eq!(json["error_count"], 1);
        assert_eq!(json["stopped_early"], false);
        let results = json["results"].as_array().expect("results must be an array");
        assert!(results[0]["error"].is_string(), "unexpected response: {json}");
        assert_eq!(results[1]["ok"], true, "the second operation must still have run: {json}");
        assert_eq!(results[1]["tool"], "refill_zones");
    }
}
