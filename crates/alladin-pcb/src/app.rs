//! The `eframe::App`: a tiny screen state machine -- [`Screen::NewBoard`]
//! (a params form) and [`Screen::Editor`] (the open board: place and
//! drag footprints, assign nets by pin click, draw manual 45° traces,
//! vias, zones, and silk). Every placement/move is hard-gated by
//! [`BoardDoc::check_placement`] so a part can never end up off-board
//! or overlapping another one -- "correct-by-construction" rather than
//! "flag it after the fact".

use std::collections::{BTreeMap, VecDeque};
use std::path::{Path, PathBuf};
use std::sync::mpsc;
#[cfg(not(target_arch = "wasm32"))]
use std::thread;

/// Max document snapshots kept for Ctrl+Z / Ctrl+Y. Each entry is a full
/// [`BoardDoc`] clone — enough for a working session without unbounded RAM.
const UNDO_LIMIT: usize = 40;

use alladin_core::{Item, ItemId, JlcpcbDfm, LayerId, NetId, Node, PadShape, ZoneConnection};
use alladin_geom::{Aabb, Point, Polygon, Unit, MM};
use alladin_render::{Camera, LayerToggles};
use eframe::egui::{self, Color32, Stroke};

use crate::board_doc::{
    BoardDoc, CopperWeight, FootprintId, LayerCount, NewBoardParams, SilkDotId, SilkTextId, ZoneId,
    ZoneRecord, DEFAULT_SILK_DOT_DIAMETER, DEFAULT_SILK_TEXT_HEIGHT, DEFAULT_VIA_DIAMETER,
    DEFAULT_VIA_DRILL, SILK_DOT_DIAMETER_STEPS_MM, SILK_TEXT_HEIGHT_STEPS_MM,
};
use crate::footprint::{self, world_items, FootprintTemplate, PadShapeKind};
use crate::parts_db::PartsDb;
use crate::ratsnest;
use crate::routing::{RoutingDrag, TraceDrag};

/// Progress / completion messages from the native zone-refill worker.
#[cfg(not(target_arch = "wasm32"))]
enum ZoneRefillEvent {
    Progress { done: usize, total: usize },
    Finished { doc: BoardDoc, errors: Vec<String> },
}

/// In-flight "Refill zones" job. While active, board writes (GUI and
/// MCP) are locked — the desktop worker fills a clone and assigns it
/// back, so a concurrent edit would be overwritten. Disk reload is
/// paused the same way. A generation check on finish is the safety
/// net if a write slips through: the filled clone is discarded.
/// Desktop fills on a worker thread; WASM advances one zone per egui
/// frame so the single-threaded event loop can breathe.
enum ZoneRefillJob {
    /// One [`BoardDoc::refill_zone`] per frame — WASM path.
    #[cfg(target_arch = "wasm32")]
    Cooperative {
        before: BoardDoc,
        remaining: Vec<ZoneId>,
        done: usize,
        total: usize,
        errors: Vec<String>,
    },
    /// Full refill on a background thread — desktop path.
    #[cfg(not(target_arch = "wasm32"))]
    Background {
        before: BoardDoc,
        started_at_generation: u64,
        rx: mpsc::Receiver<ZoneRefillEvent>,
        done: usize,
        total: usize,
    },
}

impl ZoneRefillJob {
    fn progress(&self) -> (usize, usize) {
        match self {
            #[cfg(target_arch = "wasm32")]
            ZoneRefillJob::Cooperative { done, total, .. } => (*done, *total),
            #[cfg(not(target_arch = "wasm32"))]
            ZoneRefillJob::Background { done, total, .. } => (*done, *total),
        }
    }
}

pub(crate) enum Screen {
    NewBoard(NewBoardParams),
    Editor(EditorState),
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tool {
    Select,
    Place(usize),
    /// Direct pin-to-net assignment -- no schematic: click one pin then
    /// another to join them onto the same net.
    Connect,
    /// Freehand via placement (stitching vias) -- click a pin to pick
    /// (or switch to) which net to stitch (see [`EditorState::via_net`]),
    /// then every click that doesn't land on a pin drops a via there on
    /// the currently picked net, until the tool changes or Escape is
    /// pressed. Deliberately not tied to an in-progress [`RoutingDrag`]
    /// -- see `crate::routing::RoutingDrag::drop_via_and_switch_layer`
    /// for the mid-route via/layer-switch case instead.
    PlaceVia,
    /// Manual 45° routing between two same-net pins (see
    /// `crate::routing`) -- click one pin, steer with live snapped legs,
    /// click a same-net pin to commit.
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
    Part {
        index: usize,
        db_id: i64,
        name: String,
    },
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

pub(crate) struct EditorState {
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
    /// Non-blocking "Refill zones" job, if any — see [`ZoneRefillJob`].
    zone_refill: Option<ZoneRefillJob>,
    /// Bumped on every successful board mutation (and undo/redo/reload)
    /// so a finishing desktop refill can refuse to assign its clone
    /// over a board that changed while the worker ran.
    edit_generation: u64,
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
    lcsc_fetch: Option<
        std::sync::mpsc::Receiver<Result<crate::lcsc::FetchedPart, crate::lcsc::FetchError>>,
    >,
    /// The outcome of the *last* download/save attempt (`true` = ok),
    /// shown until the next attempt replaces it -- same convention as
    /// `net_message`/`route_message`/`io_message`.
    lcsc_message: Option<(bool, String)>,
    /// The board-space position the mouse was over as of the last
    /// frame's `hover_board` computation (`None` while the pointer is
    /// outside the canvas) -- used as a fallback cursor when a gesture
    /// needs board coords and the pointer is momentarily off-canvas.
    last_hover_board: Option<Point>,
    /// Document undo history: snapshots of [`Self::doc`] *before* each
    /// successful board mutation. Camera, tool, banners and parts-DB
    /// state are intentionally excluded. Cleared on Open / external
    /// reload. Depth capped at [`UNDO_LIMIT`].
    undo_stack: VecDeque<BoardDoc>,
    /// Snapshots popped by undo, waiting for redo. Cleared on every new
    /// successful mutation (branching history is discarded).
    redo_stack: VecDeque<BoardDoc>,
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
    /// Plated drill in mm; `0` means SMD (no hole).
    hole_diameter_mm: f32,
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
            hole_diameter_mm: 0.0,
            description: String::new(),
            category: String::new(),
        }
    }
}

/// Every session built-in (placeable pad set plus load-only demo ghosts)
/// plus every part currently in `parts_db`, as one flat list ready for
/// [`EditorState::templates`], alongside a parallel `template_origin`
/// (see that field's doc comment) recording which entries came from the
/// database. A `parts_db` read failure is swallowed on purpose -- a
/// broken/missing parts file should degrade to "just the built-ins
/// available this session", not stop the editor from opening at all.
pub(crate) fn load_templates(
    parts_db: &PartsDb,
) -> (
    Vec<FootprintTemplate>,
    Vec<Option<i64>>,
    Vec<Option<String>>,
    Vec<Option<String>>,
) {
    let mut templates = footprint::session_builtin_templates();
    let mut origin: Vec<Option<i64>> = vec![None; templates.len()];
    let mut hover: Vec<Option<String>> = vec![None; templates.len()];
    let mut category: Vec<Option<String>> = vec![None; templates.len()];
    if let Ok(parts) = parts_db.list_parts() {
        for part in parts {
            let tooltip = match &part.lcsc_code {
                Some(code) if !part.description.is_empty() => {
                    Some(format!("{code}: {}", part.description))
                }
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
/// hand-added part, every older saved footprint) still
/// shows up somewhere. The inner map's `""` key holds every index with
/// *no* sub-category (a plain `"Resistors"`-style category, or
/// "Uncategorized" itself) -- rendered directly under the top-level
/// header rather than one more empty-titled nested header.
fn group_templates_by_category(
    template_origin: &[Option<i64>],
    template_category: &[Option<String>],
) -> BTreeMap<String, BTreeMap<String, Vec<usize>>> {
    let mut tree: BTreeMap<String, BTreeMap<String, Vec<usize>>> = BTreeMap::new();
    for (i, origin) in template_origin.iter().enumerate() {
        if origin.is_none() {
            continue;
        }
        let full_category = template_category[i]
            .as_deref()
            .unwrap_or(crate::parts_db::UNCATEGORIZED_LABEL);
        let (top, sub) = full_category
            .split_once('/')
            .map_or((full_category, ""), |(top, sub)| (top, sub));
        tree.entry(top.to_string())
            .or_default()
            .entry(sub.to_string())
            .or_default()
            .push(i);
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
            if ui
                .small_button("\u{2716}")
                .on_hover_text("Remove from parts database")
                .clicked()
            {
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
    let matches: Vec<&ZoneRecord> = doc
        .zones
        .iter()
        .filter(|z| z.layer == layer && doc.outline.iter().any(|o| *o == z.outline))
        .collect();
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
            zone_refill: None,
            edit_generation: 0,
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
            undo_stack: VecDeque::new(),
            redo_stack: VecDeque::new(),
        }
    }

    /// Records `before` (a clone of [`Self::doc`] taken *before* a
    /// successful mutation) onto the undo stack and clears redo.
    fn record_undo(&mut self, before: BoardDoc) {
        self.bump_edit_generation();
        self.redo_stack.clear();
        self.undo_stack.push_back(before);
        while self.undo_stack.len() > UNDO_LIMIT {
            self.undo_stack.pop_front();
        }
    }

    fn bump_edit_generation(&mut self) {
        self.edit_generation = self.edit_generation.wrapping_add(1);
    }

    /// Runs `f` on the live doc; if it returns `Ok`, pushes the pre-
    /// mutation snapshot onto the undo stack. On `Err` the doc is
    /// restored from the pre-mutation snapshot so a fallible batch that
    /// mutated mid-flight cannot leave a half-applied board (individual
    /// BoardDoc mutators still preferably leave state untouched on
    /// failure; this restore is the safety net).
    fn try_mutate_doc<E>(
        &mut self,
        f: impl FnOnce(&mut BoardDoc) -> Result<(), E>,
    ) -> Result<(), E> {
        let before = self.doc.clone();
        match f(&mut self.doc) {
            Ok(()) => {
                self.record_undo(before);
                Ok(())
            }
            Err(e) => {
                self.doc = before;
                Err(e)
            }
        }
    }

    /// Like [`Self::try_mutate_doc`] for fallible ops that return a
    /// value on success (e.g. `connect_pads` → `NetId`).
    fn try_mutate_doc_ok<T, E>(
        &mut self,
        f: impl FnOnce(&mut BoardDoc) -> Result<T, E>,
    ) -> Result<T, E> {
        let before = self.doc.clone();
        match f(&mut self.doc) {
            Ok(v) => {
                self.record_undo(before);
                Ok(v)
            }
            Err(e) => {
                self.doc = before;
                Err(e)
            }
        }
    }

    /// Unconditional mutation (BoardDoc methods that always succeed).
    fn mutate_doc(&mut self, f: impl FnOnce(&mut BoardDoc)) {
        if self.zone_refill_active() {
            return;
        }
        let before = self.doc.clone();
        f(&mut self.doc);
        self.record_undo(before);
    }

    /// Drops in-progress gestures so undo/redo never leaves a ghost
    /// referring to ids that no longer exist on the restored doc.
    fn cancel_transient_gestures(&mut self) {
        self.dragging = None;
        self.silk_text_dragging = None;
        self.silk_dot_dragging = None;
        self.trace_dragging = None;
        self.routing = None;
        self.pending_connect = None;
        self.pending_pin_via = None;
        self.via_net = None;
        self.zone_points.clear();
        self.context_menu_pad = None;
    }

    fn sync_plane_zones_from_doc(&mut self) {
        let (front_plane_zones, front_plane_net) = detect_plane_zones(&self.doc, LayerId::FCu);
        let (back_plane_zones, back_plane_net) = detect_plane_zones(&self.doc, LayerId::BCu);
        self.front_plane_zones = front_plane_zones;
        self.back_plane_zones = back_plane_zones;
        // Keep the user's net picker if still valid; otherwise adopt
        // whatever the restored plane zones use.
        if front_plane_net.is_some() {
            self.front_plane_net = front_plane_net;
        }
        if back_plane_net.is_some() {
            self.back_plane_net = back_plane_net;
        }
    }

    fn clear_undo_history(&mut self) {
        self.undo_stack.clear();
        self.redo_stack.clear();
    }

    fn undo(&mut self) -> bool {
        let Some(prev) = self.undo_stack.pop_back() else {
            return false;
        };
        self.cancel_transient_gestures();
        self.clear_selection();
        self.bump_edit_generation();
        let current = std::mem::replace(&mut self.doc, prev);
        self.redo_stack.push_back(current);
        while self.redo_stack.len() > UNDO_LIMIT {
            self.redo_stack.pop_front();
        }
        self.sync_plane_zones_from_doc();
        true
    }

    fn redo(&mut self) -> bool {
        let Some(next) = self.redo_stack.pop_back() else {
            return false;
        };
        self.cancel_transient_gestures();
        self.clear_selection();
        self.bump_edit_generation();
        let current = std::mem::replace(&mut self.doc, next);
        self.undo_stack.push_back(current);
        while self.undo_stack.len() > UNDO_LIMIT {
            self.undo_stack.pop_front();
        }
        self.sync_plane_zones_from_doc();
        true
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
        let name = &self
            .doc
            .footprints
            .iter()
            .find(|f| f.id == id)?
            .template_name;
        self.templates
            .iter()
            .enumerate()
            .find(|(_, t)| &t.name == name)
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
            let Some((template_index, _)) = self.template_for(id) else {
                return;
            };
            let position = self
                .doc
                .footprints
                .iter()
                .find(|f| f.id == id)
                .unwrap()
                .position;
            let rotation_deg = self
                .doc
                .footprints
                .iter()
                .find(|f| f.id == id)
                .unwrap()
                .rotation_deg;
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
            let position = self
                .doc
                .silk_dots
                .iter()
                .find(|d| d.id == id)
                .unwrap()
                .position;
            self.silk_dot_dragging = Some(SilkDotDrag {
                id,
                grab_offset: position.sub(board_pos),
                candidate_position: position,
                valid: true,
            });
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
            let candidate = snap_to_grid_point(
                board_pos.add(dragging.grab_offset),
                grid_spacing,
                grid_snap_enabled,
            );
            dragging.candidate_position = candidate;
            let template = &self.templates[dragging.template_index];
            dragging.valid = self
                .doc
                .check_placement(
                    template,
                    candidate,
                    dragging.rotation_deg,
                    Some(dragging.id),
                )
                .is_ok();
            return;
        }
        if let Some(drag) = &mut self.silk_text_dragging {
            let candidate = snap_to_grid_point(
                board_pos.add(drag.grab_offset),
                grid_spacing,
                grid_snap_enabled,
            );
            drag.candidate_position = candidate;
            drag.valid = self
                .doc
                .check_silk_text_move(drag.id, candidate, drag.rotation_deg)
                .is_ok();
            return;
        }
        let Some(drag) = &mut self.silk_dot_dragging else {
            return;
        };
        let candidate = snap_to_grid_point(
            board_pos.add(drag.grab_offset),
            grid_spacing,
            grid_snap_enabled,
        );
        drag.candidate_position = candidate;
        drag.valid = self.doc.check_silk_dot_move(drag.id, candidate).is_ok();
    }

    fn finish_drag(&mut self) {
        if let Some(dragging) = self.dragging.take() {
            let unchanged = self
                .doc
                .footprints
                .iter()
                .find(|f| f.id == dragging.id)
                .is_some_and(|f| {
                    f.position == dragging.candidate_position
                        && (f.rotation_deg - dragging.rotation_deg).abs() < f64::EPSILON
                });
            if unchanged {
                return;
            }
            let template = self.templates[dragging.template_index].clone();
            let _ = self.try_mutate_doc(|doc| {
                doc.try_move_footprint(
                    dragging.id,
                    &template,
                    dragging.candidate_position,
                    dragging.rotation_deg,
                )
            });
            return;
        }
        if let Some(drag) = self.silk_text_dragging.take() {
            let unchanged = self
                .doc
                .silk_texts
                .iter()
                .find(|t| t.id == drag.id)
                .is_some_and(|t| {
                    t.position == drag.candidate_position
                        && (t.rotation_deg - drag.rotation_deg).abs() < f64::EPSILON
                });
            if unchanged {
                return;
            }
            let _ = self.try_mutate_doc(|doc| {
                doc.try_move_silk_text(drag.id, drag.candidate_position, drag.rotation_deg)
            });
            return;
        }
        let Some(drag) = self.silk_dot_dragging.take() else {
            return;
        };
        let unchanged = self
            .doc
            .silk_dots
            .iter()
            .find(|d| d.id == drag.id)
            .is_some_and(|d| d.position == drag.candidate_position);
        if unchanged {
            return;
        }
        let _ = self.try_mutate_doc(|doc| doc.try_move_silk_dot(drag.id, drag.candidate_position));
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
        let (diameter, drill, stub_width) = (self.via_diameter, self.via_drill, self.trace_width);
        match self.try_mutate_doc_ok(|doc| {
            doc.try_add_pin_stitching_via(pad_id, diameter, drill, stub_width)
        }) {
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
    fn begin_pin_via_relocation(
        &mut self,
        pad_id: ItemId,
        reason: crate::board_doc::PinStitchingViaError,
    ) {
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
        let Some(footprint) = self
            .doc
            .footprints
            .iter()
            .find(|f| f.pad_item_ids.contains(&pad_id))
        else {
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
        let Some(via_candidate) = self
            .doc
            .pin_stitching_via_candidate(pad_id, self.via_diameter)
        else {
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
        let Some(pending) = &mut self.pending_pin_via else {
            return;
        };
        let candidate = board_pos.add(pending.grab_offset);
        pending.candidate_position = candidate;
        let template = &self.templates[pending.template_index];
        let footprint_ok = self
            .doc
            .check_placement(
                template,
                candidate,
                pending.rotation_deg,
                Some(pending.footprint_id),
            )
            .is_ok();
        let via_ok = self.doc.via_would_fit(
            candidate.add(pending.via_offset),
            pending.net,
            pending.diameter,
        );
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
        let Some(pending) = &self.pending_pin_via else {
            return;
        };
        if !pending.valid {
            return;
        }
        let pending = self.pending_pin_via.take().unwrap();
        let template = &self.templates[pending.template_index];
        let before = self.doc.clone();
        if self
            .doc
            .try_move_footprint(
                pending.footprint_id,
                template,
                pending.candidate_position,
                pending.rotation_deg,
            )
            .is_err()
        {
            self.io_message =
                Some("Couldn't place the part there after all -- try again.".to_string());
            return;
        }
        match self.doc.try_add_pin_stitching_via(
            pending.pad_id,
            pending.diameter,
            pending.drill,
            pending.stub_width,
        ) {
            Ok(_) => {
                self.record_undo(before);
                self.io_message = None;
            }
            Err(e) => {
                // Shouldn't normally happen right after both live
                // checks above passed, but if the board changed out
                // from under this exact frame, re-enter relocation
                // rather than silently dropping the user's request.
                // Restore the pre-move board so we don't leave a half
                // committed footprint move without a via / undo entry.
                self.doc = before;
                self.begin_pin_via_relocation(pending.pad_id, e);
            }
        }
    }

    fn update_trace_drag(&mut self, board_pos: Point) {
        let Some(drag) = &mut self.trace_dragging else {
            return;
        };
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
        let Some(drag) = self.trace_dragging.take() else {
            return;
        };
        let before = self.doc.clone();
        if drag.commit(&mut self.doc) {
            self.record_undo(before);
        }
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
        let Some(path) = self.file_path.clone() else {
            return;
        };
        if file_mtime(&path) != self.disk_mtime {
            // Bumped before reload so a slow zone-heavy load can't
            // re-trigger the same mtime change on every following frame.
            self.disk_mtime = file_mtime(&path);
            self.reload_from_disk(&path, parts_db);
        }
    }

    /// Reloads the board from `path` on the UI thread.
    fn reload_from_disk(&mut self, path: &Path, parts_db: &PartsDb) {
        let (templates, _, _, _) = load_templates(parts_db);
        let Ok((doc, _)) = load_from_path(path, &templates, parts_db) else {
            self.zone_message = Some("Reload from disk failed -- keeping the last good board (will retry on the next external change).".to_string());
            return;
        };
        let (templates, template_origin, template_hover, template_category) =
            load_templates(parts_db);
        self.dragging = None;
        self.trace_dragging = None;
        self.routing = None;
        self.pending_connect = None;
        self.clear_selection();
        self.bump_edit_generation();
        self.doc = doc;
        self.templates = templates;
        self.template_origin = template_origin;
        self.template_hover = template_hover;
        self.template_category = template_category;
        self.clear_undo_history();
        self.sync_plane_zones_from_doc();
        self.disk_mtime = file_mtime(path);
        self.zone_message = Some("Board reloaded from disk.".to_string());
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
            let Some((template_index, _)) = self.template_for(id) else {
                return;
            };
            let footprint = self.doc.footprints.iter().find(|f| f.id == id).unwrap();
            let position = footprint.position;
            let new_rotation = (footprint.rotation_deg + 90.0) % 360.0;
            let template = self.templates[template_index].clone();
            let _ = self.try_mutate_doc(|doc| {
                doc.try_move_footprint(id, &template, position, new_rotation)
            });
            return;
        }
        if let Some(id) = self.selected_silk_text {
            let Some(text) = self.doc.silk_texts.iter().find(|t| t.id == id) else {
                return;
            };
            let position = text.position;
            let new_rotation = (text.rotation_deg + 90.0) % 360.0;
            let _ = self.try_mutate_doc(|doc| doc.try_move_silk_text(id, position, new_rotation));
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
            let _ = self.try_mutate_doc(|doc| doc.disconnect_pad(pad_id).map(|_| ()));
            self.pending_connect = None;
            return;
        }
        match self.pending_connect.take() {
            None => self.pending_connect = Some(pad_id),
            Some(first) if first == pad_id => {} // clicked the same pin twice: no-op
            Some(first) => match self.try_mutate_doc_ok(|doc| doc.connect_pads(first, pad_id)) {
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
            None => match RoutingDrag::start_with_options(
                &self.doc,
                pad_id,
                self.trace_width,
                self.via_diameter,
                self.via_drill,
            ) {
                Some(drag) => {
                    self.routing = Some(drag);
                    self.route_message = None;
                }
                None => {
                    self.route_message = Some(
                        "This pin has no net yet \u{2014} connect it to one first.".to_string(),
                    )
                }
            },
            Some(drag) if pad_id == drag.from_pad => {
                self.routing = Some(drag); // clicked the start pin again: keep dragging
            }
            Some(mut drag) => {
                drag.update(&self.doc, board_pos);
                let before = self.doc.clone();
                if drag.commit(&mut self.doc) {
                    self.record_undo(before);
                    self.route_message = None;
                } else {
                    self.route_message = drag
                        .blocked_reason(&self.doc, board_pos)
                        .or(Some("can't connect these two pins".to_string()));
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
        let item = self
            .doc
            .track_at(board_pos, tolerance)
            .or_else(|| self.doc.via_at(board_pos, tolerance));
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
        if let Some(net) = self
            .doc
            .pad_at(board_pos)
            .and_then(|id| self.doc.pad_net(id).ok().flatten())
        {
            self.via_net = Some(net);
            self.via_message = None;
            return;
        }
        match self.via_net {
            None => {
                self.via_message = Some(
                    "Click a pin that already has a net first, to pick which net to stitch."
                        .to_string(),
                )
            }
            Some(net) => {
                let (diameter, drill) = (self.via_diameter, self.via_drill);
                match self.try_mutate_doc_ok(|doc| {
                    doc.try_add_stitching_via(board_pos, net, diameter, drill)
                }) {
                    Ok(_) => self.via_message = None,
                    Err(e) => self.via_message = Some(e.to_string()),
                }
            }
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
        let Some(bounds) = self.board_bounds() else {
            return (candidate, false, false);
        };
        let center = Point::new(
            (bounds.min.x + bounds.max.x) / 2,
            (bounds.min.y + bounds.max.y) / 2,
        );
        let threshold = self.snap_threshold_px(8.0) as f64;
        let snap_x = (candidate.x - center.x).abs() as f64 <= threshold;
        let snap_y = (candidate.y - center.y).abs() as f64 <= threshold;
        (
            Point::new(
                if snap_x { center.x } else { candidate.x },
                if snap_y { center.y } else { candidate.y },
            ),
            snap_x,
            snap_y,
        )
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
        BoardDoc::matrix_positions(
            self.matrix_rows.max(1),
            self.matrix_cols.max(1),
            pitch_x,
            pitch_y,
            center,
        )
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

    /// Closes the in-progress zone outline and fills it synchronously.
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
        let before = self.doc.clone();
        match self.doc.add_zone(outline.clone(), layer, net) {
            Ok(id) => {
                let island_count = self
                    .doc
                    .zones
                    .iter()
                    .find(|z| z.id == id)
                    .map(|z| z.item_ids.len())
                    .unwrap_or(0);
                self.record_undo(before);
                self.zone_message = Some(if island_count > 0 {
                    format!("Zone filled into {island_count} island(s).")
                } else {
                    "Zone outline recorded, but the fill came back empty (fully off-board, or fully consumed by clearances) -- it can be refilled later once obstacles change.".to_string()
                });
            }
            Err(e) => {
                self.zone_points = outline.points;
                self.zone_message = Some(format!("Zone fill refused: {e}"));
            }
        }
    }

    /// (Re)creates or removes the whole-board solid plane on `layer` synchronously.
    fn set_layer_plane(&mut self, layer: LayerId, net: Option<NetId>) {
        if self.zone_refill_active() {
            self.zone_message = Some(
                "Can't change planes while zones are refilling.".to_string(),
            );
            return;
        }
        let before = self.doc.clone();
        let old_zones = match layer {
            LayerId::FCu => std::mem::take(&mut self.front_plane_zones),
            LayerId::BCu => std::mem::take(&mut self.back_plane_zones),
        };
        for &id in &old_zones {
            self.doc.remove_zone(id);
        }
        let Some(net) = net else {
            match layer {
                LayerId::FCu => self.front_plane_zones = Vec::new(),
                LayerId::BCu => self.back_plane_zones = Vec::new(),
            }
            self.record_undo(before);
            return;
        };
        let outlines = self.doc.outline.clone();
        let mut new_zones = Vec::new();
        for outline in outlines {
            match self.doc.add_zone(outline, layer, net) {
                Ok(id) => new_zones.push(id),
                Err(e) => {
                    self.doc = before;
                    match layer {
                        LayerId::FCu => self.front_plane_zones = old_zones,
                        LayerId::BCu => self.back_plane_zones = old_zones,
                    }
                    self.zone_message = Some(format!("Solid plane refused: {e}"));
                    return;
                }
            }
        }
        match layer {
            LayerId::FCu => self.front_plane_zones = new_zones,
            LayerId::BCu => self.back_plane_zones = new_zones,
        }
        self.record_undo(before);
    }

    /// Starts a non-blocking refill of every zone. Desktop: worker thread.
    /// WASM: one zone per egui frame. Progress shows next to "Refill zones".
    fn start_refill_all_zones(&mut self) {
        if self.zone_refill.is_some() {
            return;
        }
        let ids: Vec<ZoneId> = self.doc.zones.iter().map(|z| z.id).collect();
        if ids.is_empty() {
            self.zone_message = Some("No zones to refill.".to_string());
            return;
        }
        self.cancel_transient_gestures();
        let total = ids.len();
        let before = self.doc.clone();
        self.zone_message = Some(format!("Refilling zones\u{2026} 0/{total}"));

        #[cfg(not(target_arch = "wasm32"))]
        {
            let mut doc = self.doc.clone();
            let (tx, rx) = mpsc::channel();
            thread::spawn(move || {
                let mut errors = Vec::new();
                for (i, id) in ids.into_iter().enumerate() {
                    if let Err(e) = doc.refill_zone(id) {
                        errors.push(e.to_string());
                    }
                    let _ = tx.send(ZoneRefillEvent::Progress { done: i + 1, total });
                }
                let _ = tx.send(ZoneRefillEvent::Finished { doc, errors });
            });
            self.zone_refill = Some(ZoneRefillJob::Background {
                before,
                started_at_generation: self.edit_generation,
                rx,
                done: 0,
                total,
            });
        }
        #[cfg(target_arch = "wasm32")]
        {
            self.zone_refill = Some(ZoneRefillJob::Cooperative {
                before,
                remaining: ids,
                done: 0,
                total,
                errors: Vec::new(),
            });
        }
    }

    /// Advances / completes [`Self::zone_refill`]. Call once per frame while
    /// a job is active; requests a repaint so progress keeps moving without
    /// user input.
    fn poll_zone_refill(&mut self, ctx: &egui::Context) {
        let Some(job) = self.zone_refill.take() else {
            return;
        };
        ctx.request_repaint();

        #[cfg(target_arch = "wasm32")]
        {
            let ZoneRefillJob::Cooperative {
                before,
                mut remaining,
                mut done,
                total,
                mut errors,
            } = job;
            if let Some(id) = remaining.first().copied() {
                remaining.remove(0);
                if let Err(e) = self.doc.refill_zone(id) {
                    errors.push(e.to_string());
                }
                done += 1;
                self.zone_message = Some(format!("Refilling zones\u{2026} {done}/{total}"));
            }
            if remaining.is_empty() {
                self.record_undo(before);
                self.zone_message = Some(if errors.is_empty() {
                    "Zones refilled.".to_string()
                } else {
                    format!(
                        "Zones refilled with {} thermal error(s): {}",
                        errors.len(),
                        errors[0]
                    )
                });
            } else {
                self.zone_refill = Some(ZoneRefillJob::Cooperative {
                    before,
                    remaining,
                    done,
                    total,
                    errors,
                });
            }
        }

        #[cfg(not(target_arch = "wasm32"))]
        {
            let ZoneRefillJob::Background {
                before,
                started_at_generation,
                rx,
                mut done,
                mut total,
            } = job;
            loop {
                match rx.try_recv() {
                    Ok(ZoneRefillEvent::Progress { done: d, total: t }) => {
                        done = d;
                        total = t;
                        self.zone_message = Some(format!("Refilling zones\u{2026} {done}/{total}"));
                    }
                    Ok(ZoneRefillEvent::Finished { doc, errors }) => {
                        if self.edit_generation != started_at_generation {
                            self.zone_message = Some(
                                "Zone refill discarded \u{2014} the board changed during the fill. Click Refill zones again.".to_string(),
                            );
                            break;
                        }
                        self.record_undo(before);
                        self.doc = doc;
                        self.sync_plane_zones_from_doc();
                        self.zone_message = Some(if errors.is_empty() {
                            "Zones refilled.".to_string()
                        } else {
                            format!(
                                "Zones refilled with {} thermal error(s): {}",
                                errors.len(),
                                errors[0]
                            )
                        });
                        break;
                    }
                    Err(mpsc::TryRecvError::Empty) => {
                        self.zone_refill = Some(ZoneRefillJob::Background {
                            before,
                            started_at_generation,
                            rx,
                            done,
                            total,
                        });
                        break;
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        if self.edit_generation == started_at_generation {
                            self.doc = before;
                            self.sync_plane_zones_from_doc();
                        }
                        self.zone_message =
                            Some("Zone refill failed (worker ended unexpectedly).".to_string());
                        break;
                    }
                }
            }
        }
    }

    fn zone_refill_active(&self) -> bool {
        self.zone_refill.is_some()
    }
}

/// Live board + parts DB shared by the GUI frame and the desktop MCP
/// pump thread. MCP queries are handled on that pump (blocking
/// `recv`), not inside [`PcbApp::ui`] -- a native file dialog can
/// freeze the UI thread for minutes without starving the AI.
struct McpWorld {
    screen: Screen,
    parts_db: PartsDb,
}

pub struct PcbApp {
    world: std::sync::Arc<std::sync::Mutex<McpWorld>>,
    /// Whether this process was launched with `--allow-ai-write`.
    /// Only displayed/used on desktop (the web build has no MCP), hence
    /// dead on wasm32.
    #[cfg_attr(target_arch = "wasm32", allow(dead_code))]
    allow_ai_write: bool,
    /// In-flight browser file picks (WASM only).
    #[cfg(target_arch = "wasm32")]
    wasm_pending: WasmPending,
    /// Outline loaded from DXF on the New Board screen; when set, Create
    /// uses this polygon instead of the width/height/corner-radius rect.
    new_board_dxf: Option<crate::dxf_outline::DxfOutline>,
    new_board_dxf_label: Option<String>,
    new_board_dxf_message: Option<(bool, String)>,
    /// In-flight native file dialog / manufacturing write. The worker
    /// owns the blocking `rfd` call so [`PcbApp::ui`] keeps pumping
    /// and GNOME does not mark the window "not responding".
    #[cfg(not(target_arch = "wasm32"))]
    desktop_io: Option<mpsc::Receiver<DesktopIoResult>>,
}

#[cfg(target_arch = "wasm32")]
#[derive(Default)]
struct WasmPending {
    open_board: Option<mpsc::Receiver<crate::web_io::PickedFile>>,
    import_parts: Option<mpsc::Receiver<crate::web_io::PickedFile>>,
    import_dxf: Option<mpsc::Receiver<crate::web_io::PickedFile>>,
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
        let (templates, _, _, _) = load_templates(&parts_db);
        #[cfg(not(target_arch = "wasm32"))]
        let screen = match last_board_path() {
            Some(path) => match load_from_path(&path, &templates, &parts_db) {
                Ok((doc, _)) => {
                    remember_last_board(&path);
                    let (templates, template_origin, template_hover, template_category) =
                        load_templates(&parts_db);
                    let mut state = EditorState::new(
                        doc,
                        templates,
                        template_origin,
                        template_hover,
                        template_category,
                    );
                    state.set_file_path(path);
                    Screen::Editor(state)
                }
                Err(_) => Screen::NewBoard(NewBoardParams::default()),
            },
            None => Screen::NewBoard(NewBoardParams::default()),
        };
        #[cfg(target_arch = "wasm32")]
        let screen = {
            let _ = templates;
            Screen::NewBoard(NewBoardParams::default())
        };
        let world = std::sync::Arc::new(std::sync::Mutex::new(McpWorld { screen, parts_db }));
        #[cfg(not(target_arch = "wasm32"))]
        {
            let (mcp_tx, mcp_rx) = mpsc::channel();
            crate::mcp::spawn_server(mcp_tx, crate::mcp::PORT, allow_ai_write);
            spawn_mcp_pump(mcp_rx, world.clone());
        }
        Self {
            world,
            allow_ai_write,
            #[cfg(target_arch = "wasm32")]
            wasm_pending: WasmPending::default(),
            new_board_dxf: None,
            new_board_dxf_label: None,
            new_board_dxf_message: None,
            #[cfg(not(target_arch = "wasm32"))]
            desktop_io: None,
        }
    }

    fn lock_world(&self) -> std::sync::MutexGuard<'_, McpWorld> {
        self.world
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    fn clear_new_board_dxf(&mut self) {
        self.new_board_dxf = None;
        self.new_board_dxf_label = None;
        self.new_board_dxf_message = None;
    }

    fn apply_dxf_bytes(&mut self, name: &str, bytes: &[u8]) {
        match crate::dxf_outline::parse_dxf_outline(bytes) {
            Ok(outline) => {
                self.new_board_dxf_message = Some((
                    true,
                    format!(
                        "Outline from {name}: {:.1}×{:.1} mm, {} vertices ({})",
                        outline.width_mm,
                        outline.height_mm,
                        outline.vertex_count,
                        outline.source_kind
                    ),
                ));
                self.new_board_dxf_label = Some(name.to_string());
                self.new_board_dxf = Some(outline);
            }
            Err(e) => {
                self.new_board_dxf = None;
                self.new_board_dxf_label = None;
                self.new_board_dxf_message = Some((false, format!("Couldn't import DXF: {e}")));
            }
        }
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
#[cfg(not(target_arch = "wasm32"))]
fn last_board_pointer_path() -> PathBuf {
    let base = dirs::data_dir().unwrap_or_else(std::env::temp_dir);
    base.join("alladin-pcb").join("last_board.txt")
}

/// Reads back the path [`remember_last_board`] last wrote, if any --
/// `None` on a genuine first run or if the pointer file itself is
/// missing/unreadable/empty.
#[cfg(not(target_arch = "wasm32"))]
fn last_board_path() -> Option<PathBuf> {
    let text = std::fs::read_to_string(last_board_pointer_path()).ok()?;
    parse_last_board_pointer(&text)
}

/// The pure "what does this pointer file's content mean" half of
/// [`last_board_path`], split out so it's testable without touching the
/// real OS data dir. `None` for empty/all-whitespace content, so an
/// accidentally-truncated-to-empty pointer file degrades to "no
/// remembered board" rather than a bogus empty path.
#[cfg(not(target_arch = "wasm32"))]
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
#[cfg(not(target_arch = "wasm32"))]
fn remember_last_board(path: &Path) {
    let pointer = last_board_pointer_path();
    if let Some(parent) = pointer.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let _ = std::fs::write(pointer, path.to_string_lossy().as_bytes());
}

pub(crate) fn save_to_path(
    doc: &BoardDoc,
    path: &std::path::Path,
    templates: &[FootprintTemplate],
    template_origin: &[Option<i64>],
    parts_db: &PartsDb,
) -> Result<(), String> {
    let embedded =
        crate::parts_transfer::snapshots_used_on_board(doc, templates, template_origin, parts_db)
            .map_err(|e| e.to_string())?;
    write_atomic(path, crate::persistence::to_json(doc, &embedded).as_bytes())
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

/// Opens a board JSON string: reconstructs geometry (preferring any
/// embedded part snapshots), then merges those snapshots into `parts_db`
/// (LCSC / name dedupe). Returns the board plus optional
/// `(imported, skipped)` merge counts when the file carried embeds.
pub(crate) fn load_board_json(
    json: &str,
    templates: &[FootprintTemplate],
    parts_db: &PartsDb,
) -> Result<(BoardDoc, Option<(usize, usize)>), String> {
    let (mut doc, embedded) =
        crate::persistence::from_json(json, templates).map_err(|e| e.to_string())?;
    let merge = if embedded.is_empty() {
        None
    } else {
        let (imported, skipped) =
            crate::parts_transfer::merge_snapshots_into_db(parts_db, &embedded)
                .map_err(|e| e.to_string())?;
        Some((imported, skipped))
    };
    // Prefer embedded geometry for courtyards of parts not yet in the
    // session template list; then session templates for everything else.
    let mut sync_templates: Vec<FootprintTemplate> = templates.to_vec();
    for snap in &embedded {
        if !sync_templates.iter().any(|t| t.name == snap.name) {
            sync_templates.push(crate::parts_transfer::template_from_snapshot(snap));
        }
    }
    doc.sync_courtyards(&sync_templates);
    Ok((doc, merge))
}

pub(crate) fn load_from_path(
    path: &std::path::Path,
    templates: &[FootprintTemplate],
    parts_db: &PartsDb,
) -> Result<(BoardDoc, Option<(usize, usize)>), String> {
    let json = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    load_board_json(&json, templates, parts_db)
}

/// `path`'s current modification time, or `None` if it can't be
/// stat()ed at all (doesn't exist, permissions, ...) -- the one piece
/// [`EditorState::maybe_reload_from_disk`]'s live-watch polling needs,
/// factored out so both that call site and [`EditorState::set_file_path`]
/// read it the exact same way.
fn file_mtime(path: &std::path::Path) -> Option<std::time::SystemTime> {
    std::fs::metadata(path).ok()?.modified().ok()
}

#[cfg(target_arch = "wasm32")]
fn remember_last_board(_path: &std::path::Path) {}

#[cfg(target_arch = "wasm32")]
fn last_board_path() -> Option<std::path::PathBuf> {
    None
}

/// Native file dialogs + heavy folder writes. Scheduled from [`PcbApp::ui`]
/// and executed on `alladin-desktop-io` so the egui loop never blocks
/// in `rfd` or Gerber export.
#[cfg(not(target_arch = "wasm32"))]
enum DesktopFileJob {
    OpenBoard,
    SaveBoardAs,
    ExportManufacturing,
    ImportDxf,
    ExportParts { json: String },
    ImportParts,
}

/// Result of one [`DesktopFileJob`]. `None` / cancelled means the user
/// dismissed the dialog -- apply is a no-op.
#[cfg(not(target_arch = "wasm32"))]
enum DesktopIoResult {
    OpenBoard(Option<PathBuf>),
    SaveBoardAs(Option<PathBuf>),
    ExportManufacturing { message: Option<String> },
    ImportDxf(Option<Result<(String, Vec<u8>), String>>),
    ExportParts { path: Option<PathBuf>, json: String },
    ImportParts(Option<Result<String, String>>),
}

#[cfg(not(target_arch = "wasm32"))]
fn board_file_dialog() -> rfd::FileDialog {
    rfd::FileDialog::new()
        .add_filter("Alladin PCB board", &["json"])
        .set_file_name("board.json")
}

#[cfg(not(target_arch = "wasm32"))]
fn editor_from_path(path: PathBuf, parts_db: &PartsDb) -> Result<EditorState, String> {
    let (templates, _, _, _) = load_templates(parts_db);
    let (doc, merge) = load_from_path(&path, &templates, parts_db).map_err(|e| e.to_string())?;
    remember_last_board(&path);
    let (templates, template_origin, template_hover, template_category) = load_templates(parts_db);
    let mut opened = EditorState::new(
        doc,
        templates,
        template_origin,
        template_hover,
        template_category,
    );
    opened.set_file_path(path);
    if let Some((n, skip)) = merge {
        if n > 0 || skip > 0 {
            opened.lcsc_message = Some((
                true,
                format!("Board parts: imported {n}, already had {skip}."),
            ));
        }
    }
    Ok(opened)
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
    let stem = file_path
        .as_ref()
        .and_then(|p| p.file_stem())
        .and_then(|s| s.to_str())
        .unwrap_or("board");
    crate::native_gerber::export_manufacturing_files_native(
        doc,
        templates,
        stem,
        out_dir,
        bom_csv_contents,
    )
    .map_err(|e| e.to_string())
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
fn draw_delete_confirmation_window(
    ctx: &egui::Context,
    state: &mut EditorState,
    parts_db: &PartsDb,
) {
    let Some(pending) = &state.pending_delete else {
        return;
    };
    let (title, body) = match pending {
        PendingDelete::Part { name, .. } => (
            "Delete part?".to_string(),
            format!("Remove \"{name}\" from your parts database?\n\nThis cannot be undone."),
        ),
        PendingDelete::Category { prefix, count } => (
            "Delete category?".to_string(),
            format!("Delete all {count} part(s) under \"{prefix}\"?\n\nThis cannot be undone."),
        ),
    };
    let mut confirmed = false;
    let mut cancelled = false;
    egui::Window::new(title)
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
        .show(ctx, |ui| {
            ui.label(body);
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    cancelled = true;
                }
                if ui
                    .add(egui::Button::new("Yes, delete").fill(Color32::from_rgb(160, 50, 50)))
                    .clicked()
                {
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
                let (templates, template_origin, template_hover, template_category) =
                    load_templates(parts_db);
                state.templates = templates;
                state.template_origin = template_origin;
                state.template_hover = template_hover;
                state.template_category = template_category;
                state.tool = Tool::Select;
                state.io_message = Some(format!("Deleted {deleted} part(s) from \"{prefix}\"."));
            }
            Err(e) => {
                state.io_message = Some(format!("Couldn't delete category \"{prefix}\": {e}"))
            }
        },
    }
}

fn draw_ghost(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    items: &[Item],
    valid: bool,
) {
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
                let radius_px =
                    (shape.bounding_radius() as f32 / MM as f32 * camera.pixels_per_mm).max(1.0);
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
/// footprint's own local (unrotated) frame -- one circumscribing
/// reach above the topmost pad/hole plus a 1mm margin. Mirrored here
/// (from `FootprintTemplate`) so
/// [`draw_footprint_details`] can draw the designator at the very spot
/// and orientation manufacturing export will actually print it, instead
/// of the old fixed "2mm above the center, never rotated" guess that
/// made the preview and the Gerber disagree.
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
/// ([`crate::board_doc::SilkText::stroke_segments`]) and native Gerber
/// strokes -- the GUI preview, the legality check, and the produced
/// silkscreen are now literally the same geometry.
/// egui draws butt-capped lines, fabs expect round-capped ones; small
/// discs on every segment end close that gap (and double as smooth
/// joints between a polyline's segments).
fn draw_silk_text_strokes(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    text: &crate::board_doc::SilkText,
    color: Color32,
) {
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
fn silk_text_outline_px(
    rect: egui::Rect,
    camera: &Camera,
    text: &crate::board_doc::SilkText,
) -> Vec<egui::Pos2> {
    text.bounding_rect()
        .points
        .iter()
        .map(|&p| camera.board_to_screen(rect, p))
        .collect()
}

/// Draws one already-placed [`crate::board_doc::SilkText`] -- just the
/// strokes themselves, no box: the yellow selection ring already
/// shows the outline on demand.
fn draw_silk_text(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    text: &crate::board_doc::SilkText,
) {
    draw_silk_text_strokes(
        painter,
        rect,
        camera,
        text,
        Color32::from_rgb(220, 220, 220),
    );
}

/// [`draw_silk_text`]'s live, red/green placement-validity preview for
/// an in-progress [`Tool::PlaceSilkText`] session -- same green/red
/// convention [`draw_ghost`] already uses for a footprint/via ghost.
/// The glyph strokes themselves carry the validity color; the thin
/// outline around them is just a grab-frame affordance.
fn draw_silk_text_ghost(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    text: &crate::board_doc::SilkText,
    valid: bool,
) {
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
/// prints: center + radius through the camera, no cosmetic inflation.
fn draw_silk_dot_circle(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    circle: &alladin_geom::Circle,
    color: Color32,
) {
    let center = camera.board_to_screen(rect, circle.center);
    let radius_px = (circle.radius as f32 / MM as f32 * camera.pixels_per_mm).max(1.5);
    painter.circle_filled(center, radius_px, color);
}

/// [`draw_silk_dot_circle`]'s red/green placement-validity ghost for
/// [`Tool::PlaceSilkDot`] and an in-progress dot drag -- same color
/// convention as [`draw_silk_text_ghost`].
fn draw_silk_dot_ghost(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    circle: &alladin_geom::Circle,
    valid: bool,
) {
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

fn draw_selection_ring(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    node: &Node,
    item_ids: &[ItemId],
) {
    for &id in item_ids {
        match node.get(id) {
            Some(Item::Pad { shape, .. }) => {
                let center = camera.board_to_screen(rect, shape.center());
                let radius_px = pad_ring_radius_px(shape, camera);
                painter.circle_stroke(
                    center,
                    radius_px,
                    Stroke::new(2.0, Color32::from_rgb(255, 220, 0)),
                );
            }
            // A pure mounting-hole footprint (see `PlacedFootprint::hole_item_ids`'s
            // doc comment) has no pads for the arm above to ever match --
            // without this, selecting one (now possible at all since
            // `BoardDoc::footprint_at`'s own hole-hit-test fix) drew no
            // visible feedback whatsoever, even though the selection
            // itself, drag-to-move, and Delete all already worked.
            Some(Item::Hole { position, drill }) => {
                let center = camera.board_to_screen(rect, *position);
                let radius_px =
                    (*drill as f32 / 2.0 / MM as f32 * camera.pixels_per_mm).max(1.0) + 3.0;
                painter.circle_stroke(
                    center,
                    radius_px,
                    Stroke::new(2.0, Color32::from_rgb(255, 220, 0)),
                );
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
fn draw_item_selection_highlight(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    doc: &BoardDoc,
    id: ItemId,
) {
    let color = Color32::from_rgb(255, 220, 0);
    for wire_id in doc.connected_wire(id) {
        match doc.node.get(wire_id) {
            Some(Item::Track { shape, .. }) => {
                let a = camera.board_to_screen(rect, shape.a);
                let b = camera.board_to_screen(rect, shape.b);
                let width_px =
                    (shape.width as f32 / MM as f32 * camera.pixels_per_mm).max(1.0) + 4.0;
                painter.line_segment([a, b], Stroke::new(width_px, color.gamma_multiply(0.5)));
            }
            Some(Item::Via { shape, .. }) => {
                let center = camera.board_to_screen(rect, shape.center);
                let radius_px =
                    (shape.radius as f32 / MM as f32 * camera.pixels_per_mm).max(1.0) + 4.0;
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
fn draw_pending_pin(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    node: &Node,
    pad_id: ItemId,
) {
    if let Some(Item::Pad { shape, .. }) = node.get(pad_id) {
        let center = camera.board_to_screen(rect, shape.center());
        let radius_px = pad_ring_radius_px(shape, camera);
        painter.circle_stroke(
            center,
            radius_px,
            Stroke::new(2.5, Color32::from_rgb(80, 220, 255)),
        );
    }
}

/// The live preview of an in-progress [`Tool::Route`] drag: the pin it
/// started from (cyan ring), every corner already fixed as a solid line
/// in the net's colour, and the free-steered snapped-angle live end --
/// solid while clear, dashed red while blocked.
fn draw_routing_preview(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    doc: &BoardDoc,
    routing: &crate::routing::RoutingDrag,
) {
    draw_pending_pin(painter, rect, camera, &doc.node, routing.from_pad);
    if let Some(target) = routing.hover_target {
        draw_pending_pin(painter, rect, camera, &doc.node, target);
    }
    let net_color = alladin_render::net_color(Some(routing.net())).gamma_multiply(0.85);

    let fixed_path: Vec<Point> = std::iter::once(routing.origin())
        .chain(routing.fixed_points())
        .collect();
    for leg in fixed_path.windows(2) {
        let from = camera.board_to_screen(rect, leg[0]);
        let to = camera.board_to_screen(rect, leg[1]);
        painter.line_segment([from, to], Stroke::new(3.0, net_color));
    }

    let (live_legs, live_clear) = routing.live_end();
    if live_legs.is_empty() {
        return;
    }
    let live_color = if live_clear {
        net_color
    } else {
        Color32::from_rgb(230, 70, 70)
    };
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
fn draw_trace_drag_preview(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    drag: &crate::routing::TraceDrag,
) {
    let (path, clear) = drag.live();
    if path.len() < 2 {
        return;
    }
    let net_color = alladin_render::net_color(Some(drag.net())).gamma_multiply(0.85);
    let color = if clear {
        net_color
    } else {
        Color32::from_rgb(230, 70, 70)
    };
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
fn draw_zone_preview(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    points: &[Point],
    hover_board: Option<Point>,
) {
    if points.is_empty() {
        return;
    }
    let color = Color32::from_rgb(255, 200, 60);
    let screen_points: Vec<egui::Pos2> = points
        .iter()
        .map(|&p| camera.board_to_screen(rect, p))
        .collect();
    for pair in screen_points.windows(2) {
        painter.line_segment([pair[0], pair[1]], Stroke::new(2.0, color));
    }
    for &p in &screen_points {
        painter.circle_filled(p, 3.0, color);
    }
    if let (Some(&last), Some(hover)) = (screen_points.last(), hover_board) {
        let hover_screen = camera.board_to_screen(rect, hover);
        painter.line_segment(
            [last, hover_screen],
            Stroke::new(1.5, color.gamma_multiply(0.6)),
        );
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
fn draw_matrix_snap_guides(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    bounds: alladin_geom::Aabb,
    snap_x: bool,
    snap_y: bool,
) {
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
    Point::new(
        center.x + wx.round() as alladin_geom::Unit,
        center.y + wy.round() as alladin_geom::Unit,
    )
}

fn rotated_rect_points(
    center: Point,
    width: alladin_geom::Unit,
    height: alladin_geom::Unit,
    rotation_deg: f64,
    camera: &Camera,
    rect: egui::Rect,
) -> Vec<egui::Pos2> {
    let (hw, hh) = (width as f64 / 2.0, height as f64 / 2.0);
    let rad = rotation_deg.to_radians();
    let (sin, cos) = rad.sin_cos();
    [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        .into_iter()
        .map(|corner| camera.board_to_screen(rect, rotate_and_place(corner, center, sin, cos)))
        .collect()
}

fn rotated_ellipse_points(
    center: Point,
    width: alladin_geom::Unit,
    height: alladin_geom::Unit,
    rotation_deg: f64,
    camera: &Camera,
    rect: egui::Rect,
) -> Vec<egui::Pos2> {
    const SEGMENTS: usize = 24;
    let (a, b) = (width as f64 / 2.0, height as f64 / 2.0);
    let rad = rotation_deg.to_radians();
    let (sin, cos) = rad.sin_cos();
    (0..SEGMENTS)
        .map(|i| {
            let t = (i as f64 / SEGMENTS as f64) * std::f64::consts::TAU;
            camera.board_to_screen(
                rect,
                rotate_and_place((a * t.cos(), b * t.sin()), center, sin, cos),
            )
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
/// real board, but in the right ballpark of the "green PCB" look a
/// stroke-only outline (all `alladin_render::draw_board` gives you, by
/// design -- see that crate's module doc comment) can never provide on
/// its own.
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
/// never fills outlines/zones on purpose (zone meshes can be huge; see
/// its module doc). Board outlines stay small, so we ear-clip them here
/// and paint a triangle mesh — correct for concave DXF imports, not only
/// convex rounded rectangles.
///
/// Every polygon *other* than the largest by area is treated as a
/// cutout/hole (real boards commonly have both an outer outline and
/// separate hole polygons) and painted back over in the canvas
/// background colour, so a mounting-hole-sized cutout still reads as a
/// hole rather than vanishing into the green fill.
fn draw_board_substrate(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    outline: &[Polygon],
) {
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
        let fill = if i == board_index {
            SOLDERMASK_GREEN
        } else {
            CANVAS_BACKGROUND
        };
        fill_polygon_mesh(painter, rect, camera, poly, fill);
    }
}

/// Paint a (possibly concave) polygon fill via ear-clipped triangles.
fn fill_polygon_mesh(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    poly: &Polygon,
    fill: Color32,
) {
    let tris = alladin_geom::triangulate::triangulate_simple(poly);
    if tris.is_empty() {
        // Degenerate / untriangulable: last-resort convex fill (may be wrong
        // for concave shapes, but better than drawing nothing).
        let points: Vec<egui::Pos2> = poly
            .points
            .iter()
            .map(|&p| camera.board_to_screen(rect, p))
            .collect();
        painter.add(egui::Shape::convex_polygon(points, fill, Stroke::NONE));
        return;
    }
    let mut mesh = egui::Mesh::default();
    for &p in &poly.points {
        mesh.colored_vertex(camera.board_to_screen(rect, p), fill);
    }
    for [a, b, c] in tris {
        mesh.add_triangle(a as u32, b as u32, c as u32);
    }
    painter.add(egui::Shape::mesh(mesh));
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
            painter.circle_filled(
                camera.board_to_screen(rect, Point::new(x, y)),
                1.0,
                dot_color,
            );
            x += spacing;
        }
        y += spacing;
    }
}

/// Draws one pad with its *true* shape rather than always a plain
/// circle -- see `footprint.rs`'s doc comment for why collision
/// geometry and rendered shape are allowed to differ like this.
fn draw_pad_shape(
    painter: &egui::Painter,
    rect: egui::Rect,
    camera: &Camera,
    geometry: &PadGeometry,
    paint: &PadPaint,
) {
    let (stroke_color, stroke_width) = if paint.highlight {
        (Color32::from_rgb(255, 210, 0), 2.5)
    } else {
        (Color32::from_rgb(20, 20, 20), 1.0)
    };
    match geometry.shape {
        PadShapeKind::Circle => {
            let center_px = camera.board_to_screen(rect, geometry.center);
            let radius_px = (geometry.radius as f32 / MM as f32 * camera.pixels_per_mm).max(1.0);
            painter.circle_filled(center_px, radius_px, paint.fill);
            painter.circle_stroke(
                center_px,
                radius_px,
                Stroke::new(stroke_width, stroke_color),
            );
        }
        PadShapeKind::Rect { width, height } => {
            let points = rotated_rect_points(
                geometry.center,
                width,
                height,
                geometry.rotation_deg,
                camera,
                rect,
            );
            painter.add(egui::Shape::convex_polygon(
                points,
                paint.fill,
                Stroke::new(stroke_width, stroke_color),
            ));
        }
        PadShapeKind::Oval { width, height } => {
            let points = rotated_ellipse_points(
                geometry.center,
                width,
                height,
                geometry.rotation_deg,
                camera,
                rect,
            );
            painter.add(egui::Shape::convex_polygon(
                points,
                paint.fill,
                Stroke::new(stroke_width, stroke_color),
            ));
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
            let Some(Item::Pad {
                shape,
                net,
                layer,
                hole_diameter,
                ..
            }) = doc.node.get(pad_id)
            else {
                continue;
            };
            let is_pth = hole_diameter.is_some();
            if *layer == LayerId::BCu && !layers.back_layer && !is_pth {
                continue;
            }
            let fill = alladin_render::net_highlight_dim(
                alladin_render::layer_tint(*layer, alladin_render::net_color(*net)),
                *net,
                highlight_net,
            );
            let pad_template = template.and_then(|t| t.pads.get(index));
            let kind = pad_template
                .map(|p| p.shape)
                .unwrap_or(PadShapeKind::Circle);
            let number = pad_template.map(|p| p.number.as_str()).unwrap_or("");
            let total_rotation =
                fp.rotation_deg + pad_template.map(|p| p.rotation_deg).unwrap_or(0.0);
            let is_pin_one = number == "1";
            // `bounding_radius()` only actually gets drawn for
            // `PadShapeKind::Circle` (see `draw_pad_shape`'s `match`) --
            // and a `PadShape::Circle`'s bounding radius is its exact
            // radius, so this is not an approximation for the case that
            // matters here. `Rect`/`Oval` pads instead use `kind`'s own
            // `width`/`height` below, never this field.
            let geometry = PadGeometry {
                center: shape.center(),
                radius: shape.bounding_radius(),
                shape: kind,
                rotation_deg: total_rotation,
            };
            draw_pad_shape(
                painter,
                rect,
                camera,
                &geometry,
                &PadPaint {
                    fill,
                    highlight: is_pin_one,
                },
            );
            if let Some(drill) = hole_diameter {
                let center_px = camera.board_to_screen(rect, shape.center());
                let radius_px = (*drill as f32 / 2.0 / MM as f32 * camera.pixels_per_mm).max(1.0);
                painter.circle_stroke(
                    center_px,
                    radius_px,
                    Stroke::new(1.2, Color32::from_gray(40)),
                );
            }
            if !number.is_empty() {
                let center_px = camera.board_to_screen(rect, shape.center());
                painter.text(
                    center_px,
                    egui::Align2::CENTER_CENTER,
                    number,
                    egui::FontId::proportional(10.0),
                    Color32::BLACK,
                );
            }
        }

        if !fp.reference.is_empty() {
            // Reference label geometry (local offset above the part's
            // pad extent, rotating *with* the footprint, KiCad-compatible
            // 1.27mm size, DFM-floor stroke width), rendered with the
            // same embedded Hershey strokes the Gerber export uses --
            // so the preview's "U73" sits, rotates, and looks exactly
            // like the exported Gerber.
            let local_y = template.map(reference_label_local_y).unwrap_or(-2 * MM);
            let label = crate::board_doc::SilkText {
                id: crate::board_doc::SilkTextId(usize::MAX),
                text: fp.reference.clone(),
                position: Point::new(0, local_y)
                    .rotated(fp.rotation_deg)
                    .add(fp.position),
                rotation_deg: fp.rotation_deg,
                layer: LayerId::FCu,
                height: (1.27 * MM as f64) as Unit,
                line_width: JlcpcbDfm::MIN_SILK_LINE_WIDTH,
            };
            draw_silk_text_strokes(
                painter,
                rect,
                camera,
                &label,
                Color32::from_rgb(225, 225, 225),
            );
        }

        // A faint dashed-looking (deliberately thin, low-alpha)
        // rectangle around the part's own real mechanical body/
        // courtyard (see `crate::board_doc::PlacedFootprint::courtyard`'s
        // own doc comment) -- lets a user actually *see* whether a
        // part's pins poke outside its own real body, and whether two
        // neighbouring bodies are crowding each other, without that
        // requiring the placement/DRC rejection to already have fired.
        alladin_render::draw_polygon_outline(
            painter,
            rect,
            camera,
            &fp.courtyard,
            Stroke::new(1.0, Color32::from_rgba_unmultiplied(180, 180, 190, 130)),
        );
    }
}

impl eframe::App for PcbApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // Disk-reload / zone-refill / desktop-io completion still need
        // a tick while the window is idle. File dialogs and Gerber
        // writes run on `alladin-desktop-io`, so this method stays
        // short and GNOME does not mark the window unresponsive.
        ui.ctx()
            .request_repaint_after(std::time::Duration::from_millis(100));

        #[cfg(not(target_arch = "wasm32"))]
        if let Some(rx) = &self.desktop_io {
            match rx.try_recv() {
                Ok(result) => {
                    self.desktop_io = None;
                    apply_desktop_io_result(self, result);
                }
                Err(mpsc::TryRecvError::Empty) => {}
                Err(mpsc::TryRecvError::Disconnected) => self.desktop_io = None,
            }
        }

        let mut world = self
            .world
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        let board_loading = false;
        let mut pending_screen: Option<Screen> = None;
        let mut reset_new_board_dxf = false;
        #[cfg(target_arch = "wasm32")]
        let mut pending_dxf_file: Option<(String, Vec<u8>)> = None;
        #[cfg(target_arch = "wasm32")]
        let mut pending_dxf_read_err: Option<String> = None;
        #[cfg(not(target_arch = "wasm32"))]
        let mut desktop_file_job: Option<DesktopFileJob> = None;

        #[cfg(target_arch = "wasm32")]
        {
            if let Some(rx) = self.wasm_pending.open_board.take() {
                match rx.try_recv() {
                    Ok(crate::web_io::PickedFile::Ok { name, bytes }) => {
                        if matches!(&world.screen, Screen::Editor(s) if s.zone_refill_active()) {
                            if let Screen::Editor(state) = &mut world.screen {
                                state.io_message = Some(
                                    "Can't open a board while zones are refilling.".to_string(),
                                );
                            }
                        } else {
                            let (templates, _, _, _) = load_templates(&world.parts_db);
                            match String::from_utf8(bytes) {
                                Ok(json) => {
                                    match load_board_json(&json, &templates, &world.parts_db) {
                                        Ok((doc, merge)) => {
                                            let (
                                                templates,
                                                template_origin,
                                                template_hover,
                                                template_category,
                                            ) = load_templates(&world.parts_db);
                                            let mut opened = EditorState::new(
                                                doc,
                                                templates,
                                                template_origin,
                                                template_hover,
                                                template_category,
                                            );
                                            opened.set_file_path(PathBuf::from(name));
                                            if let Some((n, skip)) = merge {
                                                if n > 0 || skip > 0 {
                                                    opened.lcsc_message = Some((
                                                        true,
                                                        format!(
                                                            "Board parts: imported {n}, already had {skip}."
                                                        ),
                                                    ));
                                                }
                                            }
                                            pending_screen = Some(Screen::Editor(opened));
                                        }
                                        Err(e) => {
                                            if let Screen::Editor(state) = &mut world.screen {
                                                state.io_message =
                                                    Some(format!("Couldn't open board: {e}"));
                                            }
                                        }
                                    }
                                }
                                Err(e) => {
                                    if let Screen::Editor(state) = &mut world.screen {
                                        state.io_message =
                                            Some(format!("Couldn't open board: {e}"));
                                    }
                                }
                            }
                        }
                    }
                    Ok(crate::web_io::PickedFile::Err(e)) => {
                        if let Screen::Editor(state) = &mut world.screen {
                            state.io_message = Some(format!("Couldn't open board: {e}"));
                        }
                    }
                    Ok(crate::web_io::PickedFile::Cancelled) => {}
                    Err(std::sync::mpsc::TryRecvError::Empty) => {
                        self.wasm_pending.open_board = Some(rx);
                        ui.ctx().request_repaint();
                    }
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {}
                }
            }
            if let Some(rx) = self.wasm_pending.import_parts.take() {
                match rx.try_recv() {
                    Ok(crate::web_io::PickedFile::Ok { bytes, .. }) => {
                        match String::from_utf8(bytes) {
                            Ok(json) => match crate::parts_transfer::import_library_json(
                                &world.parts_db,
                                &json,
                            ) {
                                Ok((n, skip)) => {
                                    if let Screen::Editor(state) = &mut world.screen {
                                        let (templates, origin, hover, category) =
                                            load_templates(&world.parts_db);
                                        state.templates = templates;
                                        state.template_origin = origin;
                                        state.template_hover = hover;
                                        state.template_category = category;
                                        state.lcsc_message =
                                        Some((true, format!("Imported {n} part(s), skipped {skip} duplicate(s).")));
                                    }
                                }
                                Err(e) => {
                                    if let Screen::Editor(state) = &mut world.screen {
                                        state.lcsc_message =
                                            Some((false, format!("Couldn't import parts: {e}")));
                                    }
                                }
                            },
                            Err(e) => {
                                if let Screen::Editor(state) = &mut world.screen {
                                    state.lcsc_message =
                                        Some((false, format!("Couldn't import parts: {e}")));
                                }
                            }
                        }
                    }
                    Ok(crate::web_io::PickedFile::Err(e)) => {
                        if let Screen::Editor(state) = &mut world.screen {
                            state.lcsc_message =
                                Some((false, format!("Couldn't import parts: {e}")));
                        }
                    }
                    Ok(crate::web_io::PickedFile::Cancelled) => {}
                    Err(std::sync::mpsc::TryRecvError::Empty) => {
                        self.wasm_pending.import_parts = Some(rx);
                        ui.ctx().request_repaint();
                    }
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {}
                }
            }
            if let Some(rx) = self.wasm_pending.import_dxf.take() {
                match rx.try_recv() {
                    Ok(crate::web_io::PickedFile::Ok { name, bytes }) => {
                        pending_dxf_file = Some((name, bytes));
                    }
                    Ok(crate::web_io::PickedFile::Err(e)) => {
                        pending_dxf_read_err = Some(format!("Couldn't import DXF: {e}"));
                    }
                    Ok(crate::web_io::PickedFile::Cancelled) => {}
                    Err(std::sync::mpsc::TryRecvError::Empty) => {
                        self.wasm_pending.import_dxf = Some(rx);
                        ui.ctx().request_repaint();
                    }
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {}
                }
            }
        }

        {
            let McpWorld { screen, parts_db } = &mut *world;
            match screen {
                Screen::NewBoard(params) => {
                    let mut create_requested = false;
                    let mut open_from_new = false;
                    let mut import_dxf = false;
                    let mut clear_dxf = false;
                    let dxf_loaded = self.new_board_dxf.is_some();
                    egui::CentralPanel::default().show(ui, |ui| {
                    ui.vertical_centered(|ui| {
                        ui.add_space(40.0);
                        ui.heading("Alladin PCB \u{2014} New board");
                        #[cfg(not(target_arch = "wasm32"))]
                        if self.allow_ai_write {
                            ui.colored_label(Color32::from_rgb(255, 180, 60), "\u{1F513} AI-Schreibzugriff aktiv (MCP)");
                        }
                        if ui.button("Open existing board…").clicked() {
                            open_from_new = true;
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
                        ui.label("Outline");
                        ui.horizontal(|ui| {
                            if ui.button("Import outline DXF…").clicked() {
                                import_dxf = true;
                            }
                            if dxf_loaded && ui.button("Clear DXF").clicked() {
                                clear_dxf = true;
                            }
                        });
                        ui.end_row();

                        if let Some(label) = &self.new_board_dxf_label {
                            ui.label("DXF file");
                            ui.label(label.as_str());
                            ui.end_row();
                        }

                        ui.label("Width (mm)");
                        if let Some(outline) = &self.new_board_dxf {
                            ui.label(format!("{:.2} (from DXF)", outline.width_mm));
                        } else {
                            ui.add(egui::DragValue::new(&mut params.width_mm).range(1.0..=500.0).speed(0.5));
                        }
                        ui.end_row();

                        ui.label("Height (mm)");
                        if let Some(outline) = &self.new_board_dxf {
                            ui.label(format!("{:.2} (from DXF)", outline.height_mm));
                        } else {
                            ui.add(egui::DragValue::new(&mut params.height_mm).range(1.0..=500.0).speed(0.5));
                        }
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
                        if dxf_loaded {
                            ui.label("(unused — DXF outline)");
                        } else {
                            ui.add(egui::DragValue::new(&mut params.corner_radius_mm).range(0.0..=50.0).speed(0.1));
                        }
                        ui.end_row();
                    });

                    if let Some((ok, msg)) = &self.new_board_dxf_message {
                        ui.add_space(8.0);
                        let color = if *ok { Color32::from_rgb(120, 200, 140) } else { Color32::from_rgb(230, 90, 90) };
                        ui.colored_label(color, msg);
                    }

                    ui.add_space(20.0);
                    let can_create = dxf_loaded || params.is_valid();
                    if !can_create {
                        ui.colored_label(
                            egui::Color32::from_rgb(230, 90, 90),
                            "Invalid dimensions: width/height must be positive and the corner radius must fit within the board.",
                        );
                    }
                    ui.add_enabled_ui(can_create, |ui| {
                        if ui.button("Create board").clicked() {
                            create_requested = true;
                        }
                    });
                });

                    if clear_dxf {
                        reset_new_board_dxf = true;
                    }
                    #[cfg(not(target_arch = "wasm32"))]
                    if import_dxf {
                        desktop_file_job = Some(DesktopFileJob::ImportDxf);
                    }
                    #[cfg(target_arch = "wasm32")]
                    if import_dxf {
                        self.wasm_pending.import_dxf =
                            Some(crate::web_io::pick_file("DXF outline", &["dxf"]));
                    }

                    if create_requested {
                        let doc = if let Some(outline) = self.new_board_dxf.take() {
                            self.new_board_dxf_label = None;
                            self.new_board_dxf_message = None;
                            params.create_with_outline(outline.polygon)
                        } else {
                            params.create()
                        };
                        let (templates, template_origin, template_hover, template_category) =
                            load_templates(&parts_db);
                        pending_screen = Some(Screen::Editor(EditorState::new(
                            doc,
                            templates,
                            template_origin,
                            template_hover,
                            template_category,
                        )));
                    }
                    #[cfg(not(target_arch = "wasm32"))]
                    if open_from_new {
                        desktop_file_job = Some(DesktopFileJob::OpenBoard);
                    }
                    #[cfg(target_arch = "wasm32")]
                    if open_from_new {
                        self.wasm_pending.open_board =
                            Some(crate::web_io::pick_file("Alladin PCB board", &["json"]));
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
                    // `&parts_db`, only reachable outside this
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
                    ui.ctx()
                        .request_repaint_after(std::time::Duration::from_millis(300));
                    if !state.zone_refill_active() {
                        state.maybe_reload_from_disk(&parts_db, ui.input(|i| i.time));
                    }
                    state.poll_zone_refill(ui.ctx());

                    if let Some(rx) = &state.lcsc_fetch {
                        match rx.try_recv() {
                            Ok(Ok(part)) => {
                                state.lcsc_fetch = None;
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
                                        state.lcsc_message = Some((
                                            true,
                                            format!(
                                                "{} ({}) added to your parts database.",
                                                record.template.name, part.lcsc_code
                                            ),
                                        ));
                                        let tooltip =
                                            format!("{}: {}", part.lcsc_code, part.description);
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
                                        let existing =
                                            state.template_origin.iter().position(|origin| {
                                                origin
                                                    .and_then(|id| {
                                                        parts_db
                                                            .find_by_lcsc_code(&code)
                                                            .ok()
                                                            .flatten()
                                                            .map(|r| r.id == id)
                                                    })
                                                    .unwrap_or(false)
                                            });
                                        if let Some(index) = existing {
                                            state.tool = Tool::Place(index);
                                            state.clear_selection();
                                        }
                                        state.lcsc_message = Some((true, format!("{code} is already in your parts database \u{2014} selected for placing.")));
                                    }
                                    Err(e) => {
                                        state.lcsc_message = Some((
                                            false,
                                            format!(
                                            "Downloaded, but couldn't save to the database: {e}"
                                        ),
                                        ))
                                    }
                                }
                            }
                            Ok(Err(e)) => {
                                state.lcsc_fetch = None;
                                state.lcsc_message = Some((false, e.to_string()));
                            }
                            Err(std::sync::mpsc::TryRecvError::Empty) => {}
                            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                                state.lcsc_fetch = None;
                                state.lcsc_message = Some((
                                    false,
                                    "the download thread ended unexpectedly".to_string(),
                                ));
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
                    // Document undo/redo. Skip while a text field has focus
                    // so Ctrl+Z doesn't steal egui's own text undo. Mid-
                    // route Backspace still undoes corners (below); Ctrl+Z
                    // undoes the last *committed* board change.
                    if !ui.ctx().text_edit_focused() {
                        let (undo_pressed, redo_pressed) = ui.input(|i| {
                            let ctrl = i.modifiers.command; // Ctrl on Linux/Win, Cmd on macOS
                            let z = i.key_pressed(egui::Key::Z);
                            let y = i.key_pressed(egui::Key::Y);
                            let undo = ctrl && z && !i.modifiers.shift;
                            let redo = (ctrl && y) || (ctrl && z && i.modifiers.shift);
                            (undo, redo)
                        });
                        if state.zone_refill_active() {
                            // Don't tear the board out from under a running refill.
                        } else if undo_pressed {
                            if state.routing.is_some() {
                                state.routing = None;
                                state.route_message = Some(
                                "Route cancelled \u{2014} Ctrl+Z undoes committed board changes."
                                    .to_string(),
                            );
                            } else if !state.undo() {
                                state.io_message = Some("Nothing to undo.".to_string());
                            } else {
                                state.io_message = None;
                            }
                        } else if redo_pressed {
                            if !state.redo() {
                                state.io_message = Some("Nothing to redo.".to_string());
                            } else {
                                state.io_message = None;
                            }
                        }
                    }
                    if !state.zone_refill_active() && ui.input(|i| i.key_pressed(egui::Key::R)) {
                        match state.tool {
                            Tool::Place(_) => {
                                state.place_rotation_deg = (state.place_rotation_deg + 90.0) % 360.0
                            }
                            Tool::PlaceSilkText => {
                                state.silk_text_place_rotation_deg =
                                    (state.silk_text_place_rotation_deg + 90.0) % 360.0
                            }
                            Tool::Select => state.rotate_selected(),
                            Tool::Connect
                            | Tool::Route
                            | Tool::PlaceVia
                            | Tool::DrawZone
                            | Tool::PlaceSilkDot => {}
                        }
                    }
                    if !state.zone_refill_active() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                        if let Tool::DrawZone = state.tool {
                            state.finish_zone();
                        }
                    }
                    if !state.zone_refill_active() && ui.input(|i| i.key_pressed(egui::Key::V)) {
                        if matches!(state.tool, Tool::Route) {
                            let outcome = state.routing.as_mut().map(|routing| {
                                let before = state.doc.clone();
                                match routing.drop_via_and_switch_layer(&mut state.doc) {
                                    Ok(()) => Ok(before),
                                    Err(e) => Err(e),
                                }
                            });
                            match outcome {
                                Some(Ok(before)) => {
                                    state.record_undo(before);
                                    state.route_message = None;
                                }
                                Some(Err(e)) => state.route_message = Some(e.to_string()),
                                None => {}
                            }
                        }
                    }
                    if !state.zone_refill_active() && ui.input(|i| i.key_pressed(egui::Key::Space)) {
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
                    if !state.zone_refill_active()
                        && ui.input(|i| i.key_pressed(egui::Key::Backspace))
                        && matches!((state.tool, &state.routing), (Tool::Route, Some(_)))
                    {
                        if let Some(routing) = &mut state.routing {
                            state.route_message = if routing.undo_last_corner() {
                                None
                            } else {
                                Some("no fixed corner to undo yet".to_string())
                            };
                        }
                    } else if !state.zone_refill_active()
                        && ui.input(|i| {
                            i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace)
                        })
                    {
                        if let Some(id) = state.selected.take() {
                            state.mutate_doc(|doc| {
                                doc.remove_footprint(id);
                            });
                        } else if let Some(id) = state.selected_item.take() {
                            state.mutate_doc(|doc| {
                                doc.remove_wire(id);
                            });
                        } else if let Some(id) = state.selected_silk_text.take() {
                            state.mutate_doc(|doc| {
                                doc.remove_silk_text(id);
                            });
                        } else if let Some(id) = state.selected_silk_dot.take() {
                            state.mutate_doc(|doc| {
                                doc.remove_silk_dot(id);
                            });
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
                        // Desktop only — WASM has no MCP server.
                        #[cfg(not(target_arch = "wasm32"))]
                        {
                            // Always visible, never just implied -- see
                            // `PcbApp::allow_ai_write`'s doc comment for why
                            // this state should never be silent.
                            if self.allow_ai_write {
                                ui.colored_label(Color32::from_rgb(255, 180, 60), "\u{1F513} AI-Schreibzugriff aktiv (MCP)").on_hover_text(
                                    "Dieser Prozess wurde mit --allow-ai-write gestartet: eine KI kann über MCP Bauteile platzieren, verbinden, routen und speichern.",
                                );
                            } else {
                                ui.weak("\u{1F512} AI-Schreibzugriff aus (nur lesen via MCP)").on_hover_text(
                                    "Zum Aktivieren: alladin-pcb mit --allow-ai-write neu starten.",
                                );
                            }
                            ui.separator();
                        }
                        if ui.button("Fit to board").clicked() {
                            state.fitted = false;
                        }
                        ui.separator();
                        if ui
                            .add_enabled(!state.zone_refill_active(), egui::Button::new("New board..."))
                            .on_hover_text("Board writes are paused while zones refill.")
                            .clicked()
                        {
                            new_board_requested = true;
                        }
                        ui.separator();
                        if ui
                            .add_enabled(!state.zone_refill_active(), egui::Button::new("Open..."))
                            .on_hover_text("Board writes are paused while zones refill.")
                            .clicked()
                        {
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
                            .add(egui::Button::new("Export manufacturing files..."))
                            .on_hover_text("Native Gerbers + drill (zip), JLCPCB CPL, and BOM CSV. No KiCad required.")
                            .clicked()
                        {
                            export_manufacturing_requested = true;
                        }
                        ui.separator();
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
                        if state.zone_refill_active() {
                            let (done, total) = state.zone_refill.as_ref().map(|j| j.progress()).unwrap_or((0, 1));
                            let frac = if total == 0 { 1.0 } else { done as f32 / total as f32 };
                            ui.add(
                                egui::ProgressBar::new(frac)
                                    .desired_width(140.0)
                                    .text(format!("Zones {done}/{total}")),
                            )
                            .on_hover_text("Refilling copper pours — board edits are paused until this finishes.");
                        } else if ui
                            .add_enabled(!state.zone_refill_active(), egui::Button::new("Refill zones"))
                            .on_hover_text("Recompute every copper pour against the current board (non-blocking).")
                            .clicked()
                        {
                            state.start_refill_all_zones();
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
                        // Independent DragValue floors can still form a
                        // 0.25/0.15 pair with only 0.05 mm annular ring --
                        // raise diameter so `JlcpcbDfm::check_via` passes.
                        let min_dia_for_annular = state.via_drill + 2 * JlcpcbDfm::MIN_VIA_ANNULAR_RING;
                        if state.via_diameter < min_dia_for_annular {
                            state.via_diameter = min_dia_for_annular;
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
                        if footprint::is_legacy_demo_template(&state.templates[i].name) {
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
                    ui.heading("Parts library file");
                    ui.label("Transfer parts as JSON between desktop and web (no LCSC proxy).");
                    ui.horizontal(|ui| {
                        if ui.button("Export parts…").clicked() {
                            match crate::parts_transfer::export_library_json(&parts_db) {
                                Ok(json) => {
                                    #[cfg(not(target_arch = "wasm32"))]
                                    {
                                        desktop_file_job =
                                            Some(DesktopFileJob::ExportParts { json });
                                    }
                                    #[cfg(target_arch = "wasm32")]
                                    {
                                        crate::web_io::download_bytes("alladin-parts.json", json.into_bytes());
                                        state.lcsc_message = Some((true, "Parts download started: alladin-parts.json".into()));
                                    }
                                }
                                Err(e) => state.lcsc_message = Some((false, format!("Couldn't export parts: {e}"))),
                            }
                        }
                        if ui.button("Import parts…").clicked() {
                            #[cfg(not(target_arch = "wasm32"))]
                            {
                                desktop_file_job = Some(DesktopFileJob::ImportParts);
                            }
                            #[cfg(target_arch = "wasm32")]
                            {
                                self.wasm_pending.import_parts = Some(crate::web_io::pick_file("Alladin parts", &["json"]));
                            }
                        }
                    });
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        ui.add_space(4.0);
                        ui.heading("Download part (LCSC)");
                        ui.horizontal(|ui| {
                            ui.label("C-number:");
                            let text_response = ui.add_enabled(state.lcsc_fetch.is_none(), egui::TextEdit::singleline(&mut state.lcsc_input).hint_text("C2040"));
                            let enter_pressed = text_response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                            let can_fetch = state.lcsc_fetch.is_none() && !state.lcsc_input.trim().is_empty();
                            let clicked = ui.add_enabled(can_fetch, egui::Button::new(if state.lcsc_fetch.is_some() { "Downloading…" } else { "Download" })).clicked();
                            if can_fetch && (clicked || enter_pressed) {
                                state.lcsc_fetch = Some(crate::lcsc::fetch_in_background(state.lcsc_input.trim().to_string()));
                                state.lcsc_message = None;
                            }
                        });
                        if state.lcsc_fetch.is_some() {
                            ui.spinner();
                            ui.ctx().request_repaint();
                        }
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
                                ui.label("Hole Ø (mm)");
                                ui.add(egui::DragValue::new(&mut form.hole_diameter_mm).range(0.0..=4.0).speed(0.05))
                                    .on_hover_text("0 = SMD (no drill). >0 = plated through-hole on both copper layers.");
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
                    if ui
                        .button("Pin-1 auf allen Teilen")
                        .on_hover_text(
                            "Setzt auf jedem Bauteil mit Pads einen Silkscreen-Punkt an Pin 1 — \
                             dieselben JLCPCB-Silk-Regeln wie die Einzel-Checkbox (Abstand zu Pads/Kante/Body). \
                             Teile ohne Platz oder ohne Pads werden \u{fc}bersprungen. Ein Ctrl+Z macht den Batch r\u{fc}ckg\u{e4}ngig.",
                        )
                        .clicked()
                    {
                        let templates = state.templates.clone();
                        let mut report = crate::board_doc::Pin1BatchReport::default();
                        state.mutate_doc(|doc| {
                            report = doc.try_enable_pin1_markers_for_all(&templates);
                        });
                        state.silk_dot_message = Some(report.summary_line());
                    }
                    if state.selected.is_none() {
                        if let Some(message) = &state.silk_dot_message {
                            ui.label(message);
                        }
                    }
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
                        if !state.zone_refill_active() {
                            state.mutate_doc(|doc| {
                                doc.remove_footprint(id);
                            });
                        }
                        if state.selected == Some(id) {
                            state.clear_selection();
                        }
                    }

                    if let Some(id) = state.selected {
                        let selected_fp = state.doc.footprints.iter().find(|f| f.id == id).map(|fp| {
                            let pad_conns: Vec<ZoneConnection> = fp
                                .pad_item_ids
                                .iter()
                                .filter_map(|&pid| match state.doc.node.get(pid) {
                                    Some(Item::Pad { zone_connection, .. }) => Some(*zone_connection),
                                    _ => None,
                                })
                                .collect();
                            (fp.reference.clone(), fp.position, fp.rotation_deg, fp.pin1_marker.is_some(), pad_conns)
                        });
                        if let Some((reference, position, rotation_deg, marker_was_on, pad_conns)) = selected_fp {
                            ui.separator();
                            ui.label(format!("Selected: {reference}"));
                            ui.label(format!(
                                "Position: ({:.2}, {:.2}) mm",
                                position.x as f64 / MM as f64,
                                position.y as f64 / MM as f64
                            ));
                            ui.label(format!("Rotation: {:.0}\u{b0}", rotation_deg));
                            ui.label("Drag it on the board to move. R to rotate, Del to remove.");
                            if !pad_conns.is_empty() {
                                let mixed = pad_conns.windows(2).any(|w| w[0] != w[1]);
                                let current = pad_conns[0];
                                ui.horizontal(|ui| {
                                    ui.label("Pour:");
                                    if ui
                                        .selectable_label(!mixed && current == ZoneConnection::Thermal, "Thermal")
                                        .on_hover_text("Annular gap + spokes into a same-net plane (easier to solder). PTH applies this on F.Cu and B.Cu.")
                                        .clicked()
                                    {
                                        let _ = state.try_mutate_doc(|doc| doc.set_footprint_zone_connection(id, ZoneConnection::Thermal));
                                    }
                                    if ui
                                        .selectable_label(!mixed && current == ZoneConnection::Solid, "Solid")
                                        .on_hover_text("Full copper flood into a same-net plane (better for higher current).")
                                        .clicked()
                                    {
                                        let _ = state.try_mutate_doc(|doc| doc.set_footprint_zone_connection(id, ZoneConnection::Solid));
                                    }
                                    if mixed {
                                        ui.label("(mixed)");
                                    }
                                });
                            }
                            let mut marker_on = marker_was_on;
                            if ui
                                .checkbox(&mut marker_on, "Pin-1-Punkt (Silk)")
                                .on_hover_text("Druckt einen kleinen Punkt neben Pad 1 dieses Bauteils auf den Silkscreen \u{2014} wandert bei Verschieben/Drehen automatisch mit.")
                                .changed()
                            {
                                if marker_on {
                                    match state.template_for(id) {
                                        Some((template_index, _)) => {
                                            let template = state.templates[template_index].clone();
                                            match state.try_mutate_doc(|doc| doc.try_enable_pin1_marker(id, &template)) {
                                                Ok(()) => state.silk_dot_message = None,
                                                Err(e) => state.silk_dot_message = Some(format!("Pin-1-Punkt: {e}")),
                                            }
                                        }
                                        None => state.silk_dot_message = Some("Pin-1-Punkt: unbekanntes Template.".to_string()),
                                    }
                                } else {
                                    state.mutate_doc(|doc| {
                                        doc.disable_pin1_marker(id);
                                    });
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
                                    let _ = state.try_mutate_doc(|doc| doc.try_resize_silk_text(id, smaller));
                                }
                                if ui.button("+").on_hover_text("Bigger").clicked() {
                                    let bigger = silk_text_height_step(height, 1);
                                    let _ = state.try_mutate_doc(|doc| doc.try_resize_silk_text(id, bigger));
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
                                    let _ = state.try_mutate_doc(|doc| doc.try_resize_silk_dot(id, silk_dot_diameter_step(diameter, -1)));
                                }
                                if ui.button("+").on_hover_text("Bigger").clicked() {
                                    let _ = state.try_mutate_doc(|doc| doc.try_resize_silk_dot(id, silk_dot_diameter_step(diameter, 1)));
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
                        if state.zone_refill_active() {
                            state.net_message = Some(
                                "Can't rename a net while zones are refilling.".to_string(),
                            );
                        } else {
                            let previous = previous_names.get(&id).cloned().unwrap_or_default();
                            if typed_name.trim() == previous {
                                if let Some(net) = state.doc.nets.iter_mut().find(|n| n.id == id) {
                                    net.name = previous;
                                }
                            } else {
                                let mut before = state.doc.clone();
                                if let Some(net) = before.nets.iter_mut().find(|n| n.id == id) {
                                    net.name = previous.clone();
                                }
                                if let Err(e) = state.doc.rename_net(id, &typed_name) {
                                    state.net_message = Some(format!("Couldn't rename net: {e}"));
                                    if let Some(net) = state.doc.nets.iter_mut().find(|n| n.id == id)
                                    {
                                        net.name = previous;
                                    }
                                } else {
                                    state.record_undo(before);
                                    state.net_message = None;
                                }
                            }
                        }
                    }
                    if let Some(id) = net_to_remove {
                        state.mutate_doc(|doc| {
                            doc.remove_net(id);
                        });
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
                             pin and click to finish the connection. V drops a via at the cursor and \
                             switches copper layer. Esc to cancel.",
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

                    draw_delete_confirmation_window(ui.ctx(), state, &parts_db);

                    egui::CentralPanel::default().show(ui, |ui| {
                        let (rect, response) = ui.allocate_exact_size(
                            ui.available_size(),
                            egui::Sense::click_and_drag(),
                        );

                        if !state.fitted {
                            if let Some(bounds) = state.board_bounds() {
                                state.camera.fit(rect, bounds);
                            }
                            state.fitted = true;
                        }

                        let hover_board = response
                            .hover_pos()
                            .map(|p| state.camera.screen_to_board(rect, p));
                        state.last_hover_board = hover_board;

                        let hover_pad_tooltip = hover_board
                            .and_then(|p| state.doc.pad_at(p))
                            .and_then(|pad_id| {
                                let footprint = state
                                    .doc
                                    .footprints
                                    .iter()
                                    .find(|f| f.pad_item_ids.contains(&pad_id))?;
                                let index =
                                    footprint.pad_item_ids.iter().position(|&id| id == pad_id)?;
                                let pad_template = state
                                    .templates
                                    .iter()
                                    .find(|t| t.name == footprint.template_name)?
                                    .pads
                                    .get(index)?;
                                Some(match &pad_template.pin_name {
                                    Some(name) => format!(
                                        "{}.{}  ({name})",
                                        footprint.reference, pad_template.number
                                    ),
                                    None => {
                                        format!("{}.{}", footprint.reference, pad_template.number)
                                    }
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
                        let board_locked = state.zone_refill_active();
                        if !board_locked && response.secondary_clicked() {
                            state.context_menu_pad = hover_board.and_then(|p| state.doc.pad_at(p));
                        }
                        if !board_locked {
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
                        }

                        if board_locked {
                            // Still allow pan while pours recompute.
                            if response.dragged() {
                                state.camera.center_mm -=
                                    state.camera.screen_delta_to_board_mm(response.drag_delta());
                            }
                        } else if state.pending_pin_via.is_some() {
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
                                        state.camera.center_mm -= state
                                            .camera
                                            .screen_delta_to_board_mm(response.drag_delta());
                                    }
                                    if response.clicked() {
                                        if let Some(board_pos) = hover_board {
                                            let board_pos = snap_to_grid_point(
                                                board_pos,
                                                state.grid_spacing,
                                                state.grid_snap_enabled,
                                            );
                                            let template = state.templates[index].clone();
                                            let rotation = state.place_rotation_deg;
                                            if state.matrix_rows.max(1) * state.matrix_cols.max(1)
                                                > 1
                                            {
                                                let (center, _, _) =
                                                    state.snap_matrix_center(board_pos);
                                                let positions =
                                                    state.matrix_ghost_positions(center);
                                                let _ = state.try_mutate_doc(|doc| {
                                                    doc.place_matrix(
                                                        &template, &positions, rotation,
                                                    )
                                                    .map(|_| ())
                                                });
                                            } else {
                                                let _ = state.try_mutate_doc(|doc| {
                                                    doc.try_place_footprint(
                                                        &template, board_pos, rotation,
                                                    )
                                                    .map(|_| ())
                                                });
                                            }
                                        }
                                    }
                                }
                                Tool::PlaceSilkText => {
                                    if response.dragged() {
                                        state.camera.center_mm -= state
                                            .camera
                                            .screen_delta_to_board_mm(response.drag_delta());
                                    }
                                    if response.clicked() {
                                        if let Some(board_pos) = hover_board {
                                            let board_pos = snap_to_grid_point(
                                                board_pos,
                                                state.grid_spacing,
                                                state.grid_snap_enabled,
                                            );
                                            let text = state.silk_text_input.clone();
                                            let (rot, layer, height) = (
                                                state.silk_text_place_rotation_deg,
                                                state.silk_layer,
                                                state.silk_text_height,
                                            );
                                            match state.try_mutate_doc_ok(|doc| {
                                                doc.try_place_silk_text(
                                                    &text, board_pos, rot, layer, height,
                                                )
                                            }) {
                                                Ok(_) => {
                                                    state.silk_text_message = None;
                                                    // 0deg is silk text's standard: a one-off
                                                    // rotation applies to the text it was made
                                                    // for, never silently to the next one too.
                                                    state.silk_text_place_rotation_deg = 0.0;
                                                }
                                                Err(e) => {
                                                    state.silk_text_message = Some(e.to_string())
                                                }
                                            }
                                        }
                                    }
                                }
                                Tool::PlaceSilkDot => {
                                    if response.dragged() {
                                        state.camera.center_mm -= state
                                            .camera
                                            .screen_delta_to_board_mm(response.drag_delta());
                                    }
                                    if response.clicked() {
                                        if let Some(board_pos) = hover_board {
                                            let board_pos = snap_to_grid_point(
                                                board_pos,
                                                state.grid_spacing,
                                                state.grid_snap_enabled,
                                            );
                                            let (diameter, layer) =
                                                (state.silk_dot_diameter, state.silk_layer);
                                            match state.try_mutate_doc_ok(|doc| {
                                                doc.try_place_silk_dot(board_pos, diameter, layer)
                                            }) {
                                                Ok(_) => state.silk_dot_message = None,
                                                Err(e) => {
                                                    state.silk_dot_message = Some(e.to_string())
                                                }
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
                                    if state.dragging.is_some()
                                        || state.silk_text_dragging.is_some()
                                    {
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
                                        state.camera.center_mm -= state
                                            .camera
                                            .screen_delta_to_board_mm(response.drag_delta());
                                    }
                                    if response.clicked() {
                                        if let Some(board_pos) = hover_board {
                                            state.handle_select_click(board_pos);
                                        }
                                    }
                                }
                                Tool::Connect => {
                                    if response.dragged() {
                                        state.camera.center_mm -= state
                                            .camera
                                            .screen_delta_to_board_mm(response.drag_delta());
                                    }
                                    if response.clicked() {
                                        let pad_id = hover_board.and_then(|p| state.doc.pad_at(p));
                                        let unassign = ui.input(|i| i.modifiers.shift);
                                        state.handle_connect_click(pad_id, unassign);
                                    }
                                }
                                Tool::Route => {
                                    if response.dragged() && state.routing.is_none() {
                                        state.camera.center_mm -= state
                                            .camera
                                            .screen_delta_to_board_mm(response.drag_delta());
                                    }
                                    if let (Some(routing), Some(board_pos)) =
                                        (&mut state.routing, hover_board)
                                    {
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
                                        state.camera.center_mm -= state
                                            .camera
                                            .screen_delta_to_board_mm(response.drag_delta());
                                    }
                                    if response.clicked() {
                                        if let Some(board_pos) = hover_board {
                                            state.handle_place_via_click(board_pos);
                                        }
                                    }
                                }
                                Tool::DrawZone => {
                                    if response.dragged() {
                                        state.camera.center_mm -= state
                                            .camera
                                            .screen_delta_to_board_mm(response.drag_delta());
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
                        let board_layers = LayerToggles {
                            pads: false,
                            ..state.layers
                        };
                        alladin_render::draw_board(
                            &painter,
                            rect,
                            &state.camera,
                            &state.doc.outline,
                            &items,
                            &board_layers,
                            state.highlighted_net,
                        );
                        let dragging_id = state.dragging.as_ref().map(|d| d.id);
                        draw_footprint_details(
                            &painter,
                            rect,
                            &state.camera,
                            &state.doc,
                            &state.templates,
                            &state.layers,
                            dragging_id,
                            state.highlighted_net,
                        );
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
                                draw_silk_dot_circle(
                                    &painter,
                                    rect,
                                    &state.camera,
                                    &dot.circle(),
                                    Color32::from_rgb(220, 220, 220),
                                );
                            }
                        }
                        // Pin-1 markers: same silk-white ink as free dots --
                        // on the fabricated board they're indistinguishable,
                        // so the preview doesn't invent a difference either.
                        for fp in &state.doc.footprints {
                            if let Some(circle) = fp.pin1_marker_circle() {
                                draw_silk_dot_circle(
                                    &painter,
                                    rect,
                                    &state.camera,
                                    &circle,
                                    Color32::from_rgb(220, 220, 220),
                                );
                            }
                        }
                        if state.show_ratsnest {
                            draw_ratsnest(&painter, rect, &state.camera, &state.doc);
                        }

                        if let Some(pad_id) = state.pending_connect {
                            draw_pending_pin(
                                &painter,
                                rect,
                                &state.camera,
                                &state.doc.node,
                                pad_id,
                            );
                        }
                        if let Some(routing) = &state.routing {
                            draw_routing_preview(
                                &painter,
                                rect,
                                &state.camera,
                                &state.doc,
                                routing,
                            );
                        }
                        if let Tool::DrawZone = state.tool {
                            draw_zone_preview(
                                &painter,
                                rect,
                                &state.camera,
                                &state.zone_points,
                                hover_board,
                            );
                        }
                        if let Some(drag) = &state.trace_dragging {
                            draw_trace_drag_preview(&painter, rect, &state.camera, drag);
                        }

                        if let Some(id) = state.selected {
                            if state.dragging.is_none() {
                                if let Some(fp) = state.doc.footprints.iter().find(|f| f.id == id) {
                                    let ring_ids: Vec<ItemId> = fp
                                        .pad_item_ids
                                        .iter()
                                        .chain(&fp.hole_item_ids)
                                        .copied()
                                        .collect();
                                    draw_selection_ring(
                                        &painter,
                                        rect,
                                        &state.camera,
                                        &state.doc.node,
                                        &ring_ids,
                                    );
                                }
                            }
                        }
                        if let Some(id) = state.selected_item {
                            if state.trace_dragging.is_none() {
                                draw_item_selection_highlight(
                                    &painter,
                                    rect,
                                    &state.camera,
                                    &state.doc,
                                    id,
                                );
                            }
                        }
                        if let Some(id) = state.selected_silk_text {
                            if state.silk_text_dragging.is_none() {
                                if let Some(text) = state.doc.silk_texts.iter().find(|t| t.id == id)
                                {
                                    let points = silk_text_outline_px(rect, &state.camera, text);
                                    painter.add(egui::Shape::closed_line(
                                        points,
                                        Stroke::new(2.0, Color32::from_rgb(255, 220, 0)),
                                    ));
                                }
                            }
                        }
                        if let Some(id) = state.selected_silk_dot {
                            if state.silk_dot_dragging.is_none() {
                                if let Some(dot) = state.doc.silk_dots.iter().find(|d| d.id == id) {
                                    let center = state.camera.board_to_screen(rect, dot.position);
                                    let radius_px = (dot.diameter as f32 / 2.0 / MM as f32
                                        * state.camera.pixels_per_mm)
                                        .max(1.5)
                                        + 3.0;
                                    painter.circle_stroke(
                                        center,
                                        radius_px,
                                        Stroke::new(2.0, Color32::from_rgb(255, 220, 0)),
                                    );
                                }
                            }
                        }

                        if let Tool::Place(i) = state.tool {
                            if let Some(board_pos) = hover_board {
                                let board_pos = snap_to_grid_point(
                                    board_pos,
                                    state.grid_spacing,
                                    state.grid_snap_enabled,
                                );
                                let template = &state.templates[i];
                                if state.matrix_rows.max(1) * state.matrix_cols.max(1) > 1 {
                                    let (center, snap_x, snap_y) =
                                        state.snap_matrix_center(board_pos);
                                    let positions = state.matrix_ghost_positions(center);
                                    let valid = state
                                        .doc
                                        .check_matrix_placement(
                                            template,
                                            &positions,
                                            state.place_rotation_deg,
                                        )
                                        .is_ok();
                                    let ghost_items: Vec<Item> = positions
                                        .iter()
                                        .flat_map(|&p| {
                                            world_items(template, p, state.place_rotation_deg)
                                        })
                                        .collect();
                                    draw_ghost(&painter, rect, &state.camera, &ghost_items, valid);
                                    if let Some(bounds) = state.board_bounds() {
                                        draw_matrix_snap_guides(
                                            &painter,
                                            rect,
                                            &state.camera,
                                            bounds,
                                            snap_x,
                                            snap_y,
                                        );
                                    }
                                } else {
                                    let ghost_items =
                                        world_items(template, board_pos, state.place_rotation_deg);
                                    let valid = state
                                        .doc
                                        .check_placement(
                                            template,
                                            board_pos,
                                            state.place_rotation_deg,
                                            None,
                                        )
                                        .is_ok();
                                    draw_ghost(&painter, rect, &state.camera, &ghost_items, valid);
                                }
                            }
                        }
                        if let Tool::PlaceSilkText = state.tool {
                            if let Some(board_pos) = hover_board {
                                let board_pos = snap_to_grid_point(
                                    board_pos,
                                    state.grid_spacing,
                                    state.grid_snap_enabled,
                                );
                                let ghost = crate::board_doc::SilkText {
                                    id: crate::board_doc::SilkTextId(0),
                                    text: if state.silk_text_input.trim().is_empty() {
                                        "?".to_string()
                                    } else {
                                        state.silk_text_input.clone()
                                    },
                                    position: board_pos,
                                    rotation_deg: state.silk_text_place_rotation_deg,
                                    layer: state.silk_layer,
                                    height: state.silk_text_height,
                                    line_width: crate::board_doc::DEFAULT_SILK_LINE_WIDTH,
                                };
                                let valid = !state.silk_text_input.trim().is_empty()
                                    && state
                                        .doc
                                        .check_silk_text_placement(
                                            &state.silk_text_input,
                                            board_pos,
                                            state.silk_text_place_rotation_deg,
                                            state.silk_layer,
                                            state.silk_text_height,
                                        )
                                        .is_ok();
                                draw_silk_text_ghost(&painter, rect, &state.camera, &ghost, valid);
                            }
                        }
                        if let Tool::PlaceSilkDot = state.tool {
                            if let Some(board_pos) = hover_board {
                                let board_pos = snap_to_grid_point(
                                    board_pos,
                                    state.grid_spacing,
                                    state.grid_snap_enabled,
                                );
                                let circle = alladin_geom::Circle::new(
                                    board_pos,
                                    state.silk_dot_diameter / 2,
                                );
                                let valid = state
                                    .doc
                                    .check_silk_dot_placement(
                                        board_pos,
                                        state.silk_dot_diameter,
                                        state.silk_layer,
                                    )
                                    .is_ok();
                                draw_silk_dot_ghost(&painter, rect, &state.camera, &circle, valid);
                            }
                        }
                        if let Some(dragging) = &state.dragging {
                            let template = &state.templates[dragging.template_index];
                            let ghost_items = world_items(
                                template,
                                dragging.candidate_position,
                                dragging.rotation_deg,
                            );
                            draw_ghost(&painter, rect, &state.camera, &ghost_items, dragging.valid);
                        }
                        if let Some(drag) = &state.silk_text_dragging {
                            if let Some(original) =
                                state.doc.silk_texts.iter().find(|t| t.id == drag.id)
                            {
                                let ghost = crate::board_doc::SilkText {
                                    id: drag.id,
                                    text: original.text.clone(),
                                    position: drag.candidate_position,
                                    rotation_deg: drag.rotation_deg,
                                    layer: original.layer,
                                    height: original.height,
                                    line_width: original.line_width,
                                };
                                draw_silk_text_ghost(
                                    &painter,
                                    rect,
                                    &state.camera,
                                    &ghost,
                                    drag.valid,
                                );
                            }
                        }
                        if let Some(drag) = &state.silk_dot_dragging {
                            if let Some(original) =
                                state.doc.silk_dots.iter().find(|d| d.id == drag.id)
                            {
                                let circle = alladin_geom::Circle::new(
                                    drag.candidate_position,
                                    original.diameter / 2,
                                );
                                draw_silk_dot_ghost(
                                    &painter,
                                    rect,
                                    &state.camera,
                                    &circle,
                                    drag.valid,
                                );
                            }
                        }
                        if let Some(pending) = &state.pending_pin_via {
                            let template = &state.templates[pending.template_index];
                            let mut ghost_items = world_items(
                                template,
                                pending.candidate_position,
                                pending.rotation_deg,
                            );
                            let via_center = pending.candidate_position.add(pending.via_offset);
                            ghost_items.push(Item::Pad {
                                shape: PadShape::Circle(alladin_geom::Circle::new(
                                    via_center,
                                    pending.diameter / 2,
                                )),
                                net: None,
                                layer: LayerId::FCu,
                                zone_connection: ZoneConnection::Thermal,
                                hole_diameter: None,
                            });
                            draw_ghost(&painter, rect, &state.camera, &ghost_items, pending.valid);
                        }
                    });

                    // Neither delete button removes anything itself
                    // anymore -- it only stages the request here, and
                    // `draw_delete_confirmation_window` (below) is what
                    // actually calls `parts_db.delete_part`/
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
                            let hole = (form.hole_diameter_mm > 0.0)
                                .then_some(form.hole_diameter_mm as f64);
                            let template = footprint::straight_row_template_with_hole(
                                form.name.clone(),
                                form.reference_prefix.clone(),
                                form.pin_count,
                                form.pitch_mm as f64,
                                form.pad_radius_mm as f64,
                                hole,
                            );
                            let category = (!form.category.trim().is_empty())
                                .then(|| form.category.trim().to_string());
                            match parts_db.insert_part_categorized(
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
                                    state.template_hover.push(if form.description.is_empty() {
                                        None
                                    } else {
                                        Some(form.description.clone())
                                    });
                                    state.template_category.push(record.category);
                                }
                                Err(e) => {
                                    state.io_message = Some(format!("Couldn't save part: {e}"))
                                }
                            }
                        }
                    }

                    #[cfg(not(target_arch = "wasm32"))]
                    if open_requested && !state.zone_refill_active() {
                        desktop_file_job = Some(DesktopFileJob::OpenBoard);
                    }
                    #[cfg(target_arch = "wasm32")]
                    if open_requested && !state.zone_refill_active() {
                        self.wasm_pending.open_board =
                            Some(crate::web_io::pick_file("Alladin PCB board", &["json"]));
                    }
                    #[cfg(not(target_arch = "wasm32"))]
                    if save_requested || save_as_requested {
                        let existing = if save_as_requested {
                            None
                        } else {
                            state.file_path.clone()
                        };
                        if let Some(path) = existing {
                            match save_to_path(
                                &state.doc,
                                &path,
                                &state.templates,
                                &state.template_origin,
                                &parts_db,
                            ) {
                                Ok(()) => {
                                    remember_last_board(&path);
                                    state.set_file_path(path);
                                    state.io_message = None;
                                }
                                Err(e) => {
                                    state.io_message = Some(format!("Couldn't save board: {e}"))
                                }
                            }
                        } else {
                            desktop_file_job = Some(DesktopFileJob::SaveBoardAs);
                        }
                    }
                    #[cfg(target_arch = "wasm32")]
                    if save_requested || save_as_requested {
                        let name = state
                            .file_path
                            .as_ref()
                            .and_then(|p| p.file_name())
                            .map(|n| n.to_string_lossy().into_owned())
                            .unwrap_or_else(|| "board.json".to_string());
                        match crate::parts_transfer::snapshots_used_on_board(
                            &state.doc,
                            &state.templates,
                            &state.template_origin,
                            &parts_db,
                        ) {
                            Ok(embedded) => {
                                let json = crate::persistence::to_json(&state.doc, &embedded);
                                crate::web_io::download_bytes(&name, json.into_bytes());
                                state.set_file_path(PathBuf::from(&name));
                                state.io_message = Some(format!("Download started: {name}"));
                            }
                            Err(e) => state.io_message = Some(format!("Couldn't save board: {e}")),
                        }
                    }

                    #[cfg(not(target_arch = "wasm32"))]
                    if export_manufacturing_requested {
                        desktop_file_job = Some(DesktopFileJob::ExportManufacturing);
                    }
                    #[cfg(target_arch = "wasm32")]
                    if export_manufacturing_requested {
                        let stem = state
                            .file_path
                            .as_ref()
                            .and_then(|p| p.file_stem())
                            .and_then(|s| s.to_str())
                            .unwrap_or("board");
                        let bom_csv = crate::bom::to_csv(&crate::bom::build_bom_rows(
                            &state.doc,
                            &state.templates,
                            &state.template_origin,
                            &parts_db,
                        ));
                        match crate::native_gerber::export_manufacturing_zip_bytes(
                            &state.doc,
                            &state.templates,
                            stem,
                            &bom_csv,
                        ) {
                            Ok(bytes) => {
                                crate::web_io::download_bytes(
                                    &format!("{stem}_manufacturing.zip"),
                                    bytes,
                                );
                                state.io_message =
                                    Some(format!("Download started: {stem}_manufacturing.zip"));
                            }
                            Err(e) => {
                                state.io_message =
                                    Some(format!("Couldn't export manufacturing files: {e}"))
                            }
                        }
                    }

                    if new_board_requested && !state.zone_refill_active() {
                        reset_new_board_dxf = true;
                        pending_screen = Some(Screen::NewBoard(NewBoardParams::default()));
                    }
                }
            }
        }
        if let Some(screen) = pending_screen {
            world.screen = screen;
        }
        drop(world);

        if reset_new_board_dxf {
            self.clear_new_board_dxf();
        }
        #[cfg(target_arch = "wasm32")]
        if let Some(err) = pending_dxf_read_err {
            self.clear_new_board_dxf();
            self.new_board_dxf_message = Some((false, err));
        }
        #[cfg(target_arch = "wasm32")]
        if let Some((name, bytes)) = pending_dxf_file {
            self.apply_dxf_bytes(&name, &bytes);
        }

        #[cfg(not(target_arch = "wasm32"))]
        if let Some(job) = desktop_file_job {
            if self.desktop_io.is_none() {
                self.desktop_io = Some(spawn_desktop_file_job(job, self.world.clone()));
            }
        }
    }
}

/// Drains MCP queries on their own OS thread so [`PcbApp::ui`] can
/// block in a native file dialog without silencing the AI. Takes
/// [`McpWorld`]'s mutex only for the duration of one handler.
#[cfg(not(target_arch = "wasm32"))]
fn spawn_mcp_pump(
    rx: mpsc::Receiver<crate::mcp::McpQuery>,
    world: std::sync::Arc<std::sync::Mutex<McpWorld>>,
) {
    std::thread::Builder::new()
        .name("alladin-mcp-pump".into())
        .spawn(move || {
            while let Ok(query) = rx.recv() {
                let mut world = world.lock().unwrap_or_else(|p| p.into_inner());
                let McpWorld { screen, parts_db } = &mut *world;
                handle_mcp_query(query, screen, parts_db);
            }
        })
        .expect("alladin-mcp-pump thread");
}

/// Runs one native file dialog (and manufacturing write) on a worker
/// thread. [`PcbApp::ui`] keeps pumping; the result is applied on a
/// later frame via [`apply_desktop_io_result`].
#[cfg(not(target_arch = "wasm32"))]
fn spawn_desktop_file_job(
    job: DesktopFileJob,
    world: std::sync::Arc<std::sync::Mutex<McpWorld>>,
) -> mpsc::Receiver<DesktopIoResult> {
    let (tx, rx) = mpsc::channel();
    thread::Builder::new()
        .name("alladin-desktop-io".into())
        .spawn(move || {
            let result = match job {
                DesktopFileJob::OpenBoard => {
                    DesktopIoResult::OpenBoard(board_file_dialog().pick_file())
                }
                DesktopFileJob::SaveBoardAs => {
                    DesktopIoResult::SaveBoardAs(board_file_dialog().save_file())
                }
                DesktopFileJob::ExportManufacturing => {
                    let snapshot = {
                        let world = world.lock().unwrap_or_else(|p| p.into_inner());
                        export_snapshot(&world)
                    };
                    let dir = rfd::FileDialog::new().pick_folder();
                    let message = match (snapshot, dir) {
                        (Some(snap), Some(dir)) => Some(match export_manufacturing_files_to_dir(
                            &snap.doc,
                            &snap.templates,
                            &snap.file_path,
                            &dir,
                            &snap.bom_csv,
                        ) {
                            Ok(files) => format!(
                                "Wrote manufacturing files:\n  {}\n  {}\n  {}",
                                files.gerber_zip.display(),
                                files.position_csv.display(),
                                files.bom_csv.display(),
                            ),
                            Err(e) => format!("Couldn't export manufacturing files: {e}"),
                        }),
                        _ => None,
                    };
                    DesktopIoResult::ExportManufacturing { message }
                }
                DesktopFileJob::ImportDxf => {
                    let picked = rfd::FileDialog::new()
                        .add_filter("DXF outline", &["dxf"])
                        .pick_file();
                    DesktopIoResult::ImportDxf(picked.map(|path| {
                        let name = path
                            .file_name()
                            .and_then(|s| s.to_str())
                            .unwrap_or("outline.dxf")
                            .to_string();
                        std::fs::read(&path)
                            .map(|bytes| (name, bytes))
                            .map_err(|e| format!("Couldn't read DXF: {e}"))
                    }))
                }
                DesktopFileJob::ExportParts { json } => DesktopIoResult::ExportParts {
                    path: rfd::FileDialog::new()
                        .add_filter("Alladin parts", &["json"])
                        .set_file_name("alladin-parts.json")
                        .save_file(),
                    json,
                },
                DesktopFileJob::ImportParts => {
                    let path = rfd::FileDialog::new()
                        .add_filter("Alladin parts", &["json"])
                        .pick_file();
                    DesktopIoResult::ImportParts(path.map(|path| {
                        std::fs::read_to_string(&path)
                            .map_err(|e| format!("Couldn't read parts file: {e}"))
                    }))
                }
            };
            let _ = tx.send(result);
        })
        .expect("alladin-desktop-io thread");
    rx
}

#[cfg(not(target_arch = "wasm32"))]
struct ExportSnapshot {
    doc: BoardDoc,
    templates: Vec<FootprintTemplate>,
    file_path: Option<PathBuf>,
    bom_csv: String,
}

#[cfg(not(target_arch = "wasm32"))]
fn export_snapshot(world: &McpWorld) -> Option<ExportSnapshot> {
    let Screen::Editor(state) = &world.screen else {
        return None;
    };
    let bom_csv = crate::bom::to_csv(&crate::bom::build_bom_rows(
        &state.doc,
        &state.templates,
        &state.template_origin,
        &world.parts_db,
    ));
    Some(ExportSnapshot {
        doc: state.doc.clone(),
        templates: state.templates.clone(),
        file_path: state.file_path.clone(),
        bom_csv,
    })
}

#[cfg(not(target_arch = "wasm32"))]
fn apply_desktop_io_result(app: &mut PcbApp, result: DesktopIoResult) {
    match result {
        DesktopIoResult::OpenBoard(path) => {
            let mut world = app.lock_world();
            if matches!(&world.screen, Screen::Editor(s) if s.zone_refill_active()) {
                if let Screen::Editor(state) = &mut world.screen {
                    state.io_message = Some(
                        "Can't open a board while zones are refilling.".to_string(),
                    );
                }
                return;
            }
            if let Some(path) = path {
                match editor_from_path(path, &world.parts_db) {
                    Ok(opened) => world.screen = Screen::Editor(opened),
                    Err(e) => {
                        if let Screen::Editor(state) = &mut world.screen {
                            state.io_message = Some(format!("Couldn't open board: {e}"));
                        } else {
                            eprintln!("Couldn't open board: {e}");
                        }
                    }
                }
            }
        }
        DesktopIoResult::SaveBoardAs(path) => {
            let Some(path) = path else {
                return;
            };
            let mut world = app.lock_world();
            let McpWorld { screen, parts_db } = &mut *world;
            let Screen::Editor(state) = screen else {
                return;
            };
            match save_to_path(
                &state.doc,
                &path,
                &state.templates,
                &state.template_origin,
                parts_db,
            ) {
                Ok(()) => {
                    remember_last_board(&path);
                    state.set_file_path(path);
                    state.io_message = None;
                }
                Err(e) => state.io_message = Some(format!("Couldn't save board: {e}")),
            }
        }
        DesktopIoResult::ExportManufacturing { message } => {
            if let Some(message) = message {
                let mut world = app.lock_world();
                if let Screen::Editor(state) = &mut world.screen {
                    state.io_message = Some(message);
                }
            }
        }
        DesktopIoResult::ImportDxf(picked) => match picked {
            None => {}
            Some(Ok((name, bytes))) => app.apply_dxf_bytes(&name, &bytes),
            Some(Err(e)) => {
                app.clear_new_board_dxf();
                app.new_board_dxf_message = Some((false, e));
            }
        },
        DesktopIoResult::ExportParts { path, json } => {
            let mut world = app.lock_world();
            if let (Some(path), Screen::Editor(state)) = (path, &mut world.screen) {
                match std::fs::write(&path, json.as_bytes()) {
                    Ok(()) => {
                        state.lcsc_message =
                            Some((true, format!("Exported parts to {}", path.display())))
                    }
                    Err(e) => {
                        state.lcsc_message = Some((false, format!("Couldn't export parts: {e}")))
                    }
                }
            }
        }
        DesktopIoResult::ImportParts(read) => {
            let mut world = app.lock_world();
            let McpWorld { screen, parts_db } = &mut *world;
            let Screen::Editor(state) = screen else {
                return;
            };
            match read {
                None => {}
                Some(Err(e)) => {
                    state.lcsc_message = Some((false, e));
                }
                Some(Ok(json)) => match crate::parts_transfer::import_library_json(parts_db, &json)
                {
                    Ok((n, skip)) => {
                        let (templates, origin, hover, category) = load_templates(parts_db);
                        state.templates = templates;
                        state.template_origin = origin;
                        state.template_hover = hover;
                        state.template_category = category;
                        state.lcsc_message = Some((
                            true,
                            format!("Imported {n} part(s), skipped {skip} duplicate(s)."),
                        ));
                    }
                    Err(e) => {
                        state.lcsc_message = Some((false, format!("Couldn't import parts: {e}")));
                    }
                },
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn handle_mcp_query(query: crate::mcp::McpQuery, screen: &mut Screen, parts_db: &mut PartsDb) {
    use crate::mcp::McpQuery;
    match query {
        McpQuery::Nets { reply } => {
            let _ = reply.send(nets_json(screen).to_string());
        }
        McpQuery::Footprints { reply } => {
            let _ = reply.send(footprints_json(screen).to_string());
        }
        McpQuery::BoardSummary { reply } => {
            let _ = reply.send(board_summary_json(screen).to_string());
        }
        McpQuery::DownloadLcscPart { fetched, reply } => {
            let _ = reply.send(download_lcsc_part_write(screen, parts_db, fetched).to_string());
        }
        McpQuery::ConnectPins { args, reply } => {
            let _ = reply.send(connect_pins_write(screen, args).to_string());
        }
        McpQuery::SaveBoard { args, reply } => {
            let _ = reply.send(save_board_write(screen, parts_db, args).to_string());
        }
        McpQuery::ListParts { reply } => {
            let _ = reply.send(list_parts_json(screen, parts_db).to_string());
        }
        McpQuery::CheckBoard { reply } => {
            let _ = reply.send(check_board_json(screen).to_string());
        }
        McpQuery::PlaceFootprint { args, reply } => {
            let _ = reply.send(place_footprint_write(screen, args).to_string());
        }
        McpQuery::SetZoneConnection { args, reply } => {
            let _ = reply.send(set_zone_connection_write(screen, args).to_string());
        }
        McpQuery::MoveFootprint { args, reply } => {
            let _ = reply.send(move_footprint_write(screen, args).to_string());
        }
        McpQuery::ProbePlacement { args, reply } => {
            let _ = reply.send(probe_placement_json(screen, args).to_string());
        }
        McpQuery::PlaceParts { args, reply } => {
            let _ = reply.send(place_parts_write(screen, args).to_string());
        }
        McpQuery::MoveParts { args, reply } => {
            let _ = reply.send(move_parts_write(screen, args).to_string());
        }
        McpQuery::RemoveFootprint { args, reply } => {
            let _ = reply.send(remove_footprint_write(screen, args).to_string());
        }
        McpQuery::DisconnectPin { args, reply } => {
            let _ = reply.send(disconnect_pin_write(screen, args).to_string());
        }
        McpQuery::AddPinStitchingVia { args, reply } => {
            let _ = reply.send(add_pin_stitching_via_write(screen, args).to_string());
        }
        McpQuery::RenameNet { args, reply } => {
            let _ = reply.send(rename_net_write(screen, args).to_string());
        }
        McpQuery::NewBoard { args, reply } => {
            let _ = reply.send(new_board_write(screen, parts_db, args).to_string());
        }
        McpQuery::GetRoutingScene { reply } => {
            let _ = reply.send(get_routing_scene_json(screen).to_string());
        }
        McpQuery::ProbeRoute { args, reply } => {
            let _ = reply.send(probe_route_json(screen, args).to_string());
        }
        McpQuery::CommitRoute { args, reply } => {
            let _ = reply.send(commit_route_write(screen, args).to_string());
        }
        McpQuery::RipupWire { args, reply } => {
            let _ = reply.send(ripup_wire_write(screen, args).to_string());
        }
        McpQuery::SuggestRoute { args, reply } => {
            let _ = reply.send(suggest_route_handle(screen, args).to_string());
        }
    }
}

/// A short JSON `{ "note": ... }` every `*_json` builder below returns
/// verbatim while still on [`Screen::NewBoard`] -- there's no board yet
/// for any of them to describe.
#[cfg(not(target_arch = "wasm32"))]
fn no_board_open_json() -> serde_json::Value {
    serde_json::json!({ "note": "no board is open yet -- still on the New Board screen" })
}

/// A net's human-readable name, or `"?"` if the id no longer resolves.
#[cfg(not(target_arch = "wasm32"))]
fn net_name(doc: &BoardDoc, id: NetId) -> &str {
    doc.nets
        .iter()
        .find(|n| n.id == id)
        .map(|n| n.name.as_str())
        .unwrap_or("?")
}

/// Resolves a pad `ItemId` back to `{footprint, pin, pin_name}`.
#[cfg(not(target_arch = "wasm32"))]
fn describe_pad(
    doc: &BoardDoc,
    templates: &[FootprintTemplate],
    pad_id: ItemId,
) -> Option<serde_json::Value> {
    let footprint = doc
        .footprints
        .iter()
        .find(|f| f.pad_item_ids.contains(&pad_id))?;
    let index = footprint.pad_item_ids.iter().position(|&id| id == pad_id)?;
    let pad_template = templates
        .iter()
        .find(|t| t.name == footprint.template_name)
        .and_then(|t| t.pads.get(index));
    let pin = pad_template
        .map(|p| p.number.clone())
        .unwrap_or_else(|| (index + 1).to_string());
    let pin_name = pad_template.and_then(|p| p.pin_name.clone());
    Some(serde_json::json!({ "footprint": footprint.reference, "pin": pin, "pin_name": pin_name }))
}

/// Compact board size/counts overview for MCP `board_summary`.
#[cfg(not(target_arch = "wasm32"))]
fn board_overview_json(screen: &Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    let doc = &state.doc;
    let mut min_x = Unit::MAX;
    let mut min_y = Unit::MAX;
    let mut max_x = Unit::MIN;
    let mut max_y = Unit::MIN;
    for poly in &doc.outline {
        for p in &poly.points {
            min_x = min_x.min(p.x);
            min_y = min_y.min(p.y);
            max_x = max_x.max(p.x);
            max_y = max_y.max(p.y);
        }
    }
    if min_x == Unit::MAX {
        min_x = 0;
        min_y = 0;
        max_x = 0;
        max_y = 0;
    }
    serde_json::json!({
        "file_path": state.file_path.as_ref().map(|p| p.to_string_lossy().to_string()),
        "width_mm": (max_x - min_x) as f64 / MM as f64,
        "height_mm": (max_y - min_y) as f64 / MM as f64,
        "layer_count": doc.layer_count.as_u8(),
        "copper_weight_oz": match doc.copper_weight { crate::board_doc::CopperWeight::OneOz => 1, crate::board_doc::CopperWeight::TwoOz => 2 },
        "footprint_count": doc.footprints.len(),
        "net_count": doc.nets.len(),
        "track_count": doc.node.iter().filter(|i| matches!(i, Item::Track { .. })).count(),
        "via_count": doc.node.iter().filter(|i| matches!(i, Item::Via { .. })).count(),
        "zone_count": doc.zones.len(),
        "zones_stale": doc.zones_are_stale(),
    })
}

/// mm -> internal [`Unit`] conversion for MCP write-tool arguments,
/// taking `f64` (JSON has no separate float-width concept) rather than
/// `crate::cli`'s own private `f64`-taking `mm` helper -- kept as its
/// own copy here since that one is private to `cli.rs`.
fn mm_arg(v: f64) -> Unit {
    (v * MM as f64).round() as Unit
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
    let steps: Vec<Unit> = SILK_TEXT_HEIGHT_STEPS_MM
        .iter()
        .map(|&mm_value| mm_arg(mm_value))
        .collect();
    let current_index = steps.iter().position(|&s| s == current).unwrap_or_else(|| {
        steps
            .iter()
            .enumerate()
            .min_by_key(|(_, &s)| (s - current).abs())
            .map(|(i, _)| i)
            .unwrap_or(0)
    });
    let new_index = (current_index as i32 + delta).clamp(0, steps.len() as i32 - 1) as usize;
    steps[new_index]
}

/// [`silk_text_height_step`]'s exact counterpart for a silk dot's
/// diameter, over [`SILK_DOT_DIAMETER_STEPS_MM`] -- same clamped,
/// closest-step-fallback arithmetic.
fn silk_dot_diameter_step(current: Unit, delta: i32) -> Unit {
    let steps: Vec<Unit> = SILK_DOT_DIAMETER_STEPS_MM
        .iter()
        .map(|&mm_value| mm_arg(mm_value))
        .collect();
    let current_index = steps.iter().position(|&s| s == current).unwrap_or_else(|| {
        steps
            .iter()
            .enumerate()
            .min_by_key(|(_, &s)| (s - current).abs())
            .map(|(i, _)| i)
            .unwrap_or(0)
    });
    let new_index = (current_index as i32 + delta).clamp(0, steps.len() as i32 - 1) as usize;
    steps[new_index]
}

#[cfg(not(target_arch = "wasm32"))]
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

#[cfg(not(target_arch = "wasm32"))]
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
                    let zone_connection = match state.doc.node.get(pad_id) {
                        Some(Item::Pad { zone_connection, .. }) => {
                            Some(crate::mcp::zone_connection_name(*zone_connection))
                        }
                        _ => None,
                    };
                    serde_json::json!({ "pin": pin, "pin_name": pin_name, "net": net, "zone_connection": zone_connection })
                })
                .collect();
            // The footprint's own real mechanical body/courtyard (see
            // `crate::footprint::FootprintTemplate::courtyard`) --
            // the template's own local, rotation-independent
            // width/height (not this *placement*'s rotated on-board
            // extent, which `crate::board_doc::PlacedFootprint::courtyard`
            // itself is), so an AI reading footprints over MCP gets the
            // same "this part's body is WxH" fact regardless of which
            // way it happens to be rotated on the board right now.
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

/// [`crate::mcp::AlladinMcp::board_summary`]'s report builder: the
/// one-call working picture an AI needs before (and after) touching the
/// board. `overview` is [`board_overview_json`] verbatim; `rules` are
/// the DFM numbers every action gets validated against, surfaced so a
/// caller plans with facts instead of guessing them; `todo` lists pads
/// not assigned to any net yet, plus every multi-pad net whose copper
/// isn't physically one piece (via
/// [`alladin_core::Node::net_copper_components`], boiled down to name +
/// counts). `pins_without_net` is capped at 40 entries
/// (`pins_without_net_count` always holds the real total) so a
/// barely-started 200-pin board still gets a readable answer.
#[cfg(not(target_arch = "wasm32"))]
fn board_summary_json(screen: &Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    let doc = &state.doc;

    let mut pins_without_net = Vec::new();
    for fp in &doc.footprints {
        for &pad_id in &fp.pad_item_ids {
            if doc.pad_net(pad_id).ok().flatten().is_none() {
                if let Some(pad) = describe_pad(doc, &state.templates, pad_id) {
                    pins_without_net.push(format!(
                        "{}.{}",
                        pad["footprint"].as_str().unwrap_or("?"),
                        pad["pin"].as_str().unwrap_or("?")
                    ));
                }
            }
        }
    }
    let pins_without_net_count = pins_without_net.len();
    pins_without_net.truncate(40);

    let open_nets: Vec<_> = doc
        .nets
        .iter()
        .filter(|net| doc.pads_on_net(net.id).len() >= 2)
        .filter_map(|net| {
            let pieces = doc.node.net_copper_components(net.id).len();
            (pieces > 1).then(|| {
                serde_json::json!({
                    "name": net.name,
                    "pad_count": doc.pads_on_net(net.id).len(),
                    "copper_pieces": pieces,
                })
            })
        })
        .collect();
    let nothing_left_to_route = pins_without_net_count == 0 && open_nets.is_empty();

    serde_json::json!({
        "overview": board_overview_json(screen),
        "rules": {
            "min_copper_clearance_mm": doc.pad_to_pad_clearance() as f64 / MM as f64,
            "copper_to_board_edge_mm": JlcpcbDfm::COPPER_TO_ROUTED_EDGE as f64 / MM as f64,
            "default_trace_width_mm": crate::routing::DEFAULT_TRACE_WIDTH as f64 / MM as f64,
        },
        "todo": {
            "pins_without_net_count": pins_without_net_count,
            "pins_without_net": pins_without_net,
            "open_nets": open_nets,
            "nothing_left_to_route": nothing_left_to_route,
        },
    })
}

/// [`crate::mcp::McpQuery::DownloadLcscPart`]'s handler -- the network
/// fetch already happened on the MCP thread (see
/// `AlladinMcp::download_lcsc_part`); this just inserts the result into
/// the parts database and refreshes the live template list, same
/// insert/duplicate-handling logic as the GUI's own background-download
/// success handler, minus the GUI-only "select it for placing" side
/// effect (an MCP call shouldn't reach into the human's active tool).
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn download_lcsc_part_write(
    screen: &mut Screen,
    parts_db: &PartsDb,
    fetched: Result<crate::lcsc::FetchedPart, crate::lcsc::FetchError>,
) -> serde_json::Value {
    let part = match fetched {
        Ok(part) => part,
        Err(e) => return error_json(e.to_string()),
    };
    // Hard scalar-DFM gate at the boundary where foreign geometry
    // enters the system -- a part JLCPCB physically can't drill must
    // never even reach the parts database (the placement gate would
    // refuse it anyway; refusing here names the problem at download
    // time instead of mysteriously later). Report-only findings
    // (PTH annular ring / fine-pitch SMD pad floor -- see
    // `template_dfm_hard_violations`) still get surfaced without
    // blocking.
    let hard = crate::footprint::template_dfm_hard_violations(&part.pads, &[]);
    if !hard.is_empty() {
        let listed: Vec<String> = hard
            .iter()
            .map(|(label, v)| format!("{label}: {v}"))
            .collect();
        return error_json(format!(
            "{} refused -- its geometry violates JLCPCB DFM: {}",
            part.lcsc_code,
            listed.join("; ")
        ));
    }
    // Report-only findings still get surfaced, just without blocking.
    let dfm_warnings: Vec<String> = crate::footprint::template_dfm_violations(&part.pads, &[])
        .iter()
        .map(|(label, v)| format!("{label}: {v}"))
        .collect();
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
            serde_json::json!({ "ok": true, "template": template_name, "lcsc_code": part.lcsc_code, "pad_count": part.pads.len(), "dfm_warnings": dfm_warnings })
        }
        Err(crate::parts_db::PartsDbError::DuplicateLcscCode(code)) => {
            error_json(format!("{code} is already in your parts database -- place it from the existing library template"))
        }
        Err(e) => error_json(format!("downloaded, but couldn't save to the database: {e}")),
    }
}

/// [`crate::mcp::McpQuery::ConnectPins`]'s handler.
#[cfg(not(target_arch = "wasm32"))]
fn connect_pins_write(screen: &mut Screen, args: crate::mcp::ConnectPinsArgs) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let pads_a = state
        .doc
        .find_pads(&state.templates, &args.ref1, &args.pin1);
    if pads_a.is_empty() {
        return error_json(format!("no such pin: {} pin {}", args.ref1, args.pin1));
    }
    let pads_b = state
        .doc
        .find_pads(&state.templates, &args.ref2, &args.pin2);
    if pads_b.is_empty() {
        return error_json(format!("no such pin: {} pin {}", args.ref2, args.pin2));
    }
    // A multi-pad pin (e.g. a thermal pad split into a grid of same-numbered
    // paste pads) is one electrical pin: every sibling pad joins the net,
    // not just the first. Pre-check all of them so the mutation below is
    // all-or-nothing -- `try_mutate_doc_ok` records undo but doesn't roll
    // back a half-applied closure on error.
    let mut existing: Option<crate::board_doc::NetRecord> = None;
    for &pad in pads_a.iter().chain(&pads_b) {
        let Ok(net) = state.doc.pad_net(pad) else {
            return error_json("internal error: pin resolved to a non-pad item");
        };
        if let Some(net) = net {
            let record = state.doc.nets.iter().find(|n| n.id == net).cloned();
            match (&existing, record) {
                (None, Some(r)) => existing = Some(r),
                (Some(e), Some(r)) if e.id != r.id => {
                    return error_json(format!(
                        "couldn't connect {}.{} to {}.{}: pads already sit on two different nets ({} and {})",
                        args.ref1, args.pin1, args.ref2, args.pin2, e.name, r.name
                    ));
                }
                _ => {}
            }
        }
    }
    let anchor = pads_a[0];
    let result = state.try_mutate_doc_ok(|doc| {
        let net = doc.connect_pads(anchor, pads_b[0])?;
        for &extra in pads_a[1..].iter().chain(&pads_b[1..]) {
            doc.connect_pads(extra, anchor)?;
        }
        Ok(net)
    });
    match result {
        Ok(net) => {
            let name = net_name(&state.doc, net).to_string();
            serde_json::json!({ "ok": true, "net": name, "pads_joined": pads_a.len() + pads_b.len() })
        }
        Err(e) => {
            let e: crate::board_doc::NetError = e;
            error_json(format!(
                "couldn't connect {}.{} to {}.{}: {e}",
                args.ref1, args.pin1, args.ref2, args.pin2
            ))
        }
    }
}

/// [`crate::mcp::McpQuery::SaveBoard`]'s handler -- same
/// `save_to_path`/`remember_last_board`/`set_file_path` sequence as the
/// GUI's own "Save"/"Save As" buttons.
#[cfg(not(target_arch = "wasm32"))]
fn save_board_write(
    screen: &mut Screen,
    parts_db: &PartsDb,
    args: crate::mcp::SaveBoardArgs,
) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let path = match args
        .path
        .map(PathBuf::from)
        .or_else(|| state.file_path.clone())
    {
        Some(path) => path,
        None => {
            return error_json(
                "no path given, and this board has never been saved before -- give a path",
            )
        }
    };
    match save_to_path(
        &state.doc,
        &path,
        &state.templates,
        &state.template_origin,
        parts_db,
    ) {
        Ok(()) => {
            remember_last_board(&path);
            state.set_file_path(path.clone());
            serde_json::json!({ "ok": true, "path": path.to_string_lossy() })
        }
        Err(e) => error_json(format!("couldn't save board: {e}")),
    }
}

/// [`crate::mcp::McpQuery::ListParts`]'s handler -- every placeable
/// template, from the live editor's list when one is open (it also
/// carries parts downloaded this session) or straight from the parts
/// database + builtins when still on the New Board screen.
#[cfg(not(target_arch = "wasm32"))]
fn list_parts_json(screen: &Screen, parts_db: &PartsDb) -> serde_json::Value {
    let loaded;
    let (templates, hover, category): (&[FootprintTemplate], &[Option<String>], &[Option<String>]) =
        match screen {
            Screen::Editor(state) => (
                &state.templates,
                &state.template_hover,
                &state.template_category,
            ),
            Screen::NewBoard(_) => {
                loaded = load_templates(parts_db);
                (&loaded.0, &loaded.2, &loaded.3)
            }
        };
    let parts: Vec<_> = templates
        .iter()
        .enumerate()
        .filter(|(_, t)| !footprint::is_legacy_demo_template(&t.name))
        .map(|(i, t)| {
            let c = t.courtyard();
            serde_json::json!({
                "name": t.name,
                "reference_prefix": t.reference_prefix,
                "pad_count": t.pads.len(),
                "body_width_mm": c.width as f64 / MM as f64,
                "body_height_mm": c.height as f64 / MM as f64,
                "category": category.get(i).and_then(|c| c.clone()),
                "description": hover.get(i).and_then(|h| h.clone()),
            })
        })
        .collect();
    serde_json::json!({ "parts": parts })
}

/// [`crate::mcp::McpQuery::CheckBoard`]'s handler -- the
/// self-verification report an AI runs after a batch of changes.
/// Placement/clearance/edge rules need no scan here: every write path
/// (GUI gesture or MCP tool) already enforces them, so what's left to
/// *check* is exactly what write-time gates can't promise -- is the
/// netlist complete, is the copper physically connected, are zone
/// fills current -- plus the report-only template DFM findings that
/// deliberately never block (see
/// [`crate::footprint::template_dfm_violations`]).
#[cfg(not(target_arch = "wasm32"))]
fn check_board_json(screen: &Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    let doc = &state.doc;

    let mut pins_without_net = Vec::new();
    for fp in &doc.footprints {
        for &pad_id in &fp.pad_item_ids {
            if doc.pad_net(pad_id).ok().flatten().is_none() {
                if let Some(pad) = describe_pad(doc, &state.templates, pad_id) {
                    pins_without_net.push(format!(
                        "{}.{}",
                        pad["footprint"].as_str().unwrap_or("?"),
                        pad["pin"].as_str().unwrap_or("?")
                    ));
                }
            }
        }
    }
    let pins_without_net_count = pins_without_net.len();
    pins_without_net.truncate(200);

    let open_nets: Vec<_> = doc
        .nets
        .iter()
        .filter(|net| doc.pads_on_net(net.id).len() >= 2)
        .filter_map(|net| {
            let pieces = doc.node.net_copper_components(net.id).len();
            (pieces > 1).then(|| {
                serde_json::json!({
                    "name": net.name,
                    "pad_count": doc.pads_on_net(net.id).len(),
                    "copper_pieces": pieces,
                })
            })
        })
        .collect();

    // Report-only DFM findings, once per distinct placed template --
    // 16 identical LEDs shouldn't repeat the same finding 16 times.
    let mut seen = std::collections::BTreeSet::new();
    let mut dfm_warnings = Vec::new();
    for fp in &doc.footprints {
        if !seen.insert(fp.template_name.clone()) {
            continue;
        }
        if let Some(t) = state.templates.iter().find(|t| t.name == fp.template_name) {
            for (label, v) in crate::footprint::template_dfm_violations(&t.pads, &t.holes) {
                dfm_warnings.push(format!("{}: {label}: {v}", fp.template_name));
            }
        }
    }

    let zones_stale = doc.zones_are_stale();
    let ok = pins_without_net_count == 0 && open_nets.is_empty() && !zones_stale;
    serde_json::json!({
        "ok": ok,
        "pins_without_net_count": pins_without_net_count,
        "pins_without_net": pins_without_net,
        "open_nets": open_nets,
        "zones_stale": zones_stale,
        "dfm_warnings": dfm_warnings,
    })
}

/// [`crate::mcp::McpQuery::PlaceFootprint`]'s handler -- same
/// [`BoardDoc::try_place_footprint`] (and therefore the same DFM
/// gates) as the GUI's own click-to-place, through the undo helpers so
/// Ctrl+Z takes it back.
#[cfg(not(target_arch = "wasm32"))]
fn place_footprint_write(
    screen: &mut Screen,
    args: crate::mcp::PlaceFootprintArgs,
) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(template) = state
        .templates
        .iter()
        .find(|t| t.name == args.template)
        .cloned()
    else {
        return error_json(format!(
            "no template named \"{}\" in the parts library -- call list_parts for the exact names, or download_lcsc_part to add it",
            args.template
        ));
    };
    let position = Point::new(mm_arg(args.x_mm), mm_arg(args.y_mm));
    let rotation = args.rotation_deg.unwrap_or(0.0);
    let zone = match args.zone_connection.as_deref() {
        None => None,
        Some(s) => match crate::mcp::parse_zone_connection(s) {
            Ok(conn) => Some(conn),
            Err(e) => return error_json(e),
        },
    };
    match state.try_mutate_doc_ok(|doc| -> Result<_, String> {
        let id = doc
            .try_place_footprint(&template, position, rotation)
            .map_err(|e| e.to_string())?;
        if let Some(conn) = zone {
            doc.set_footprint_zone_connection(id, conn)
                .map_err(|e| e.to_string())?;
        }
        Ok(id)
    }) {
        Ok(id) => {
            let reference = state
                .doc
                .footprints
                .iter()
                .find(|f| f.id == id)
                .map(|f| f.reference.clone())
                .unwrap_or_default();
            let zone_connection = state
                .doc
                .footprints
                .iter()
                .find(|f| f.id == id)
                .and_then(|fp| fp.pad_item_ids.first().copied())
                .and_then(|pad_id| match state.doc.node.get(pad_id) {
                    Some(Item::Pad {
                        zone_connection, ..
                    }) => Some(crate::mcp::zone_connection_name(*zone_connection)),
                    _ => None,
                });
            serde_json::json!({ "ok": true, "reference": reference, "template": args.template, "x_mm": args.x_mm, "y_mm": args.y_mm, "rotation_deg": rotation, "zone_connection": zone_connection })
        }
        Err(e) => error_json(format!(
            "couldn't place {} at ({}, {}): {e}",
            args.template, args.x_mm, args.y_mm
        )),
    }
}

/// [`crate::mcp::McpQuery::SetZoneConnection`]'s handler -- same
/// [`BoardDoc::set_footprint_zone_connection`] as the GUI's Pour control.
#[cfg(not(target_arch = "wasm32"))]
fn set_zone_connection_write(
    screen: &mut Screen,
    args: crate::mcp::SetZoneConnectionArgs,
) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let conn = match crate::mcp::parse_zone_connection(&args.zone_connection) {
        Ok(conn) => conn,
        Err(e) => return error_json(e),
    };
    let Some(fp) = state
        .doc
        .footprints
        .iter()
        .find(|f| f.reference == args.reference)
    else {
        return error_json(format!(
            "no footprint with reference \"{}\" on the board",
            args.reference
        ));
    };
    if fp.pad_item_ids.is_empty() {
        return error_json(format!(
            "{} has no pads -- mounting holes have no pour connection",
            args.reference
        ));
    }
    let id = fp.id;
    match state.try_mutate_doc_ok(|doc| doc.set_footprint_zone_connection(id, conn)) {
        Ok(()) => serde_json::json!({
            "ok": true,
            "reference": args.reference,
            "zone_connection": crate::mcp::zone_connection_name(conn),
        }),
        Err(e) => error_json(format!(
            "couldn't set zone_connection on {}: {e}",
            args.reference
        )),
    }
}

/// [`crate::mcp::McpQuery::MoveFootprint`]'s handler -- same
/// [`BoardDoc::try_move_footprint`] as the GUI's own drag-drop commit;
/// on refusal the part stays exactly where it was.
#[cfg(not(target_arch = "wasm32"))]
fn move_footprint_write(
    screen: &mut Screen,
    args: crate::mcp::MoveFootprintArgs,
) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(fp) = state
        .doc
        .footprints
        .iter()
        .find(|f| f.reference == args.reference)
    else {
        return error_json(format!(
            "no footprint with reference \"{}\" on the board",
            args.reference
        ));
    };
    let (id, template_name, current_rotation) = (fp.id, fp.template_name.clone(), fp.rotation_deg);
    let Some(template) = state
        .templates
        .iter()
        .find(|t| t.name == template_name)
        .cloned()
    else {
        return error_json(format!(
            "{}'s template \"{template_name}\" is missing from the parts library",
            args.reference
        ));
    };
    // The AI moving a part out from under the human's own live drag of
    // that same part would leave the drag committing stale geometry --
    // cancel transient gestures first, exactly like undo does.
    state.cancel_transient_gestures();
    let position = Point::new(mm_arg(args.x_mm), mm_arg(args.y_mm));
    let rotation = args.rotation_deg.unwrap_or(current_rotation);
    match state.try_mutate_doc(|doc| doc.try_move_footprint(id, &template, position, rotation)) {
        Ok(()) => {
            serde_json::json!({ "ok": true, "reference": args.reference, "x_mm": args.x_mm, "y_mm": args.y_mm, "rotation_deg": rotation })
        }
        Err(e) => error_json(format!(
            "couldn't move {} to ({}, {}): {e} -- it stays where it was",
            args.reference, args.x_mm, args.y_mm
        )),
    }
}

/// [`crate::mcp::McpQuery::ProbePlacement`]'s handler -- read-only DFM probe.
#[cfg(not(target_arch = "wasm32"))]
fn probe_placement_json(
    screen: &Screen,
    args: crate::mcp::ProbePlacementArgs,
) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    crate::mcp_placement::probe_placement_json(&state.doc, &state.templates, &args)
}

/// [`crate::mcp::McpQuery::PlaceParts`]'s handler -- atomic multi-place.
#[cfg(not(target_arch = "wasm32"))]
fn place_parts_write(screen: &mut Screen, args: crate::mcp::PlacePartsArgs) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    state.cancel_transient_gestures();
    let templates = state.templates.clone();
    match state.try_mutate_doc_ok(|doc| {
        crate::mcp_placement::place_parts_on_doc(doc, &templates, &args.parts)
    }) {
        Ok(v) => v,
        Err(e) => error_json(format!("couldn't place_parts: {e}")),
    }
}

/// [`crate::mcp::McpQuery::MoveParts`]'s handler -- atomic multi-move.
#[cfg(not(target_arch = "wasm32"))]
fn move_parts_write(screen: &mut Screen, args: crate::mcp::MovePartsArgs) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    state.cancel_transient_gestures();
    let templates = state.templates.clone();
    match state.try_mutate_doc_ok(|doc| {
        crate::mcp_placement::move_parts_on_doc(doc, &templates, &args.parts)
    }) {
        Ok(v) => v,
        Err(e) => error_json(format!("couldn't move_parts: {e}")),
    }
}

/// [`crate::mcp::McpQuery::RemoveFootprint`]'s handler -- same
/// [`BoardDoc::remove_footprint`] as the GUI's Delete key, with the
/// same gesture/selection cleanup undo does: the removed part's
/// `FootprintId` must not survive anywhere in transient UI state.
#[cfg(not(target_arch = "wasm32"))]
fn remove_footprint_write(
    screen: &mut Screen,
    args: crate::mcp::RemoveFootprintArgs,
) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(id) = state
        .doc
        .footprints
        .iter()
        .find(|f| f.reference == args.reference)
        .map(|f| f.id)
    else {
        return error_json(format!(
            "no footprint with reference \"{}\" on the board",
            args.reference
        ));
    };
    state.cancel_transient_gestures();
    state.clear_selection();
    state.mutate_doc(|doc| doc.remove_footprint(id));
    serde_json::json!({ "ok": true, "removed": args.reference, "zones_stale": state.doc.zones_are_stale() })
}

/// [`crate::mcp::McpQuery::DisconnectPin`]'s handler --
/// [`crate::mcp::McpQuery::ConnectPins`]'s inverse, via
/// [`BoardDoc::disconnect_pad`].
#[cfg(not(target_arch = "wasm32"))]
fn disconnect_pin_write(
    screen: &mut Screen,
    args: crate::mcp::DisconnectPinArgs,
) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let pads = state
        .doc
        .find_pads(&state.templates, &args.reference, &args.pin);
    if pads.is_empty() {
        return error_json(format!("no such pin: {} pin {}", args.reference, args.pin));
    }
    // Multi-pad pins come off the net as one electrical pin, mirroring
    // connect_pins.
    match state.try_mutate_doc(|doc| pads.iter().try_for_each(|&pad| doc.disconnect_pad(pad))) {
        Ok(()) => {
            serde_json::json!({ "ok": true, "disconnected": format!("{}.{}", args.reference, args.pin), "pads": pads.len() })
        }
        Err(e) => error_json(format!(
            "couldn't disconnect {}.{}: {e}",
            args.reference, args.pin
        )),
    }
}

/// [`crate::mcp::McpQuery::AddPinStitchingVia`]'s handler -- the MCP
/// face of the GUI's right-click "Add via near pin"
/// ([`EditorState::add_pin_stitching_via_at`] /
/// [`BoardDoc::try_add_pin_stitching_via`]): the spot next to the pin
/// is picked automatically, no coordinates involved. Two modes:
/// reference+pin stitches that one pad; net stitches every pad on the
/// net in a single undo step, skipping pads that already have a
/// same-net via right next to them so re-running after a partial
/// failure doesn't double-stitch the pads that already worked.
#[cfg(not(target_arch = "wasm32"))]
fn add_pin_stitching_via_write(
    screen: &mut Screen,
    args: crate::mcp::AddPinStitchingViaArgs,
) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let diameter = args
        .via_diameter_mm
        .map(mm_arg)
        .unwrap_or(state.via_diameter);
    let drill = args.via_drill_mm.map(mm_arg).unwrap_or(state.via_drill);
    let stub_width = state.trace_width;

    match (&args.reference, &args.pin, &args.net) {
        (Some(reference), Some(pin), None) => {
            let Some(pad) = state.doc.find_pad(&state.templates, reference, pin) else {
                return error_json(format!("no such pin: {reference} pin {pin}"));
            };
            match state.try_mutate_doc_ok(|doc| doc.try_add_pin_stitching_via(pad, diameter, drill, stub_width)) {
                Ok(via) => serde_json::json!({
                    "ok": true,
                    "pad": format!("{reference}.{pin}"),
                    "via_x_mm": via.center.x as f64 / MM as f64,
                    "via_y_mm": via.center.y as f64 / MM as f64,
                }),
                Err(e) => error_json(format!("couldn't stitch {reference}.{pin}: {e}")),
            }
        }
        (None, None, Some(net_name)) => {
            let Some(net) = state.doc.nets.iter().find(|n| n.name == *net_name).map(|n| n.id) else {
                return error_json(format!("no net named \"{net_name}\" -- get_nets lists the current names"));
            };
            // A pad counts as already stitched when a same-net via sits
            // within stitching reach of it -- snapshotted BEFORE any of
            // this batch's own vias exist, so a via placed for pad N
            // can't make close-by pad N+1 look "already done".
            let near = mm_arg(1.5) + diameter;
            let existing_vias: Vec<Point> = state
                .doc
                .node
                .iter()
                .filter_map(|item| match item {
                    Item::Via { shape, net: n, .. } if *n == Some(net) => Some(shape.center),
                    _ => None,
                })
                .collect();
            let jobs: Vec<(ItemId, String, Point)> = state
                .doc
                .pads_on_net(net)
                .into_iter()
                .filter_map(|id| {
                    let center = state.doc.pad_center(id)?;
                    let (fp, pin) = crate::mcp_routing::pad_label(&state.doc, &state.templates, id)
                        .unwrap_or_else(|| ("?".into(), "?".into()));
                    Some((id, format!("{fp}.{pin}"), center))
                })
                .collect();
            let already_stitched = |center: Point| {
                existing_vias.iter().any(|v| {
                    let (dx, dy) = ((v.x - center.x) as f64, (v.y - center.y) as f64);
                    dx * dx + dy * dy <= (near as f64) * (near as f64)
                })
            };
            let result = state.try_mutate_doc_ok(|doc| {
                let (mut placed, mut skipped, mut failed) = (0usize, 0usize, Vec::new());
                for (pad, label, center) in &jobs {
                    if already_stitched(*center) {
                        skipped += 1;
                        continue;
                    }
                    match doc.try_add_pin_stitching_via(*pad, diameter, drill, stub_width) {
                        Ok(_) => placed += 1,
                        Err(e) => failed.push(serde_json::json!({ "pad": label, "error": e.to_string() })),
                    }
                }
                if placed == 0 && !failed.is_empty() {
                    return Err(format!(
                        "no via could be placed for any of {} pad(s) on {net_name} -- first failure: {}",
                        failed.len(),
                        failed[0]["error"].as_str().unwrap_or("?")
                    ));
                }
                failed.truncate(20);
                Ok(serde_json::json!({
                    "ok": true,
                    "net": net_name,
                    "pads": jobs.len(),
                    "placed": placed,
                    "skipped_already_stitched": skipped,
                    "failed": failed,
                }))
            });
            match result {
                Ok(v) => v,
                Err(e) => error_json(format!("couldn't stitch net {net_name}: {e}")),
            }
        }
        _ => error_json("give either reference+pin (one via) or net (stitch every pad on that net), not both/neither"),
    }
}

/// [`crate::mcp::McpQuery::RenameNet`]'s handler -- looks the net up by
/// its current name (the stable thing an MCP client actually has) and
/// delegates to [`BoardDoc::rename_net`].
#[cfg(not(target_arch = "wasm32"))]
fn rename_net_write(screen: &mut Screen, args: crate::mcp::RenameNetArgs) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(id) = state
        .doc
        .nets
        .iter()
        .find(|n| n.name == args.net)
        .map(|n| n.id)
    else {
        return error_json(format!(
            "no net named \"{}\" -- get_nets lists the current names",
            args.net
        ));
    };
    match state.try_mutate_doc(|doc| doc.rename_net(id, &args.new_name)) {
        Ok(()) => serde_json::json!({ "ok": true, "net": args.new_name }),
        Err(e) => error_json(format!("couldn't rename \"{}\": {e}", args.net)),
    }
}

/// [`crate::mcp::McpQuery::GetRoutingScene`]'s handler.
#[cfg(not(target_arch = "wasm32"))]
fn get_routing_scene_json(screen: &Screen) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    crate::mcp_routing::routing_scene_json(&state.doc, &state.templates)
}

/// [`crate::mcp::McpQuery::ProbeRoute`]'s handler -- read-only batch
/// clearance check; never mutates the board.
#[cfg(not(target_arch = "wasm32"))]
fn probe_route_json(screen: &Screen, args: crate::mcp::ProbeRouteArgs) -> serde_json::Value {
    let Screen::Editor(state) = screen else {
        return no_board_open_json();
    };
    if args.candidates.is_empty() {
        return error_json("candidates must not be empty");
    }
    if args.candidates.len() > 50 {
        return error_json("at most 50 candidates per probe_route call");
    }
    crate::mcp_routing::probe_routes_json(&state.doc, &args.candidates)
}

/// [`crate::mcp::McpQuery::CommitRoute`]'s handler -- same gates as
/// [`probe_route_json`], then `add_track_path` / `try_add_via` through undo.
#[cfg(not(target_arch = "wasm32"))]
fn commit_route_write(screen: &mut Screen, args: crate::mcp::CommitRouteArgs) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let route = match crate::mcp_routing::parse_route_candidate(&state.doc, &args.route) {
        Ok(route) => route,
        Err(e) => return error_json(e),
    };
    state.cancel_transient_gestures();
    match state.try_mutate_doc_ok(|doc| crate::mcp_routing::commit_route(doc, &route)) {
        Ok(mut v) => {
            if let Some(obj) = v.as_object_mut() {
                obj.insert("ok".into(), serde_json::json!(true));
                obj.insert("net".into(), serde_json::json!(route.net_name));
            }
            v
        }
        Err(e) => error_json(format!("couldn't commit route on {}: {e}", route.net_name)),
    }
}

/// [`crate::mcp::McpQuery::RipupWire`]'s handler.
#[cfg(not(target_arch = "wasm32"))]
fn ripup_wire_write(screen: &mut Screen, args: crate::mcp::RipupWireArgs) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    state.cancel_transient_gestures();
    if let Some(net) = args.net.as_deref() {
        return match state.try_mutate_doc_ok(|doc| crate::mcp_routing::ripup_net_copper(doc, net)) {
            Ok(v) => v,
            Err(e) => error_json(e),
        };
    }
    let (Some(x_mm), Some(y_mm)) = (args.x_mm, args.y_mm) else {
        return error_json(
            "pass net=\"...\" to rip a whole net, or x_mm and y_mm to rip the wire nearest a point",
        );
    };
    match state.try_mutate_doc_ok(|doc| crate::mcp_routing::ripup_wire_near(doc, x_mm, y_mm)) {
        Ok(v) => v,
        Err(e) => error_json(e),
    }
}

/// [`crate::mcp::McpQuery::SuggestRoute`]'s handler -- server-side
/// octilinear A* (see [`crate::mcp_routing::suggest_route`]). Read-only
/// unless `commit=true`, in which case the found path goes through the
/// exact same [`crate::mcp_routing::commit_route`] gates as the
/// commit_route tool (undo-recorded, connectivity-verified).
#[cfg(not(target_arch = "wasm32"))]
fn suggest_route_handle(
    screen: &mut Screen,
    args: crate::mcp::SuggestRouteArgs,
) -> serde_json::Value {
    use crate::mcp_routing::{self, SuggestOptions};
    if args.commit == Some(true) {
        if let Some(err) = board_write_lock_error(screen) {
            return err;
        }
    }
    let Screen::Editor(state) = screen else {
        return no_board_open_json_error();
    };
    let Some(net_record) = state.doc.nets.iter().find(|n| n.name == args.net).cloned() else {
        return error_json(format!(
            "no net named \"{}\" -- get_nets lists current names",
            args.net
        ));
    };
    let net = net_record.id;

    let resolve =
        |pin: &Option<String>, point: &Option<Vec<f64>>, which: &str| -> Result<Point, String> {
            if let Some(spec) = pin {
                let (reference, number) = spec.split_once('.').ok_or_else(|| {
                    format!(
                    "{which}_pin must look like \"U1.14\" (footprint reference, dot, pad number)"
                )
                })?;
                let pad = state
                    .doc
                    .find_pad(&state.templates, reference, number)
                    .ok_or_else(|| format!("no such pin: {reference} pin {number}"))?;
                let (center, _, pad_net) = state
                    .doc
                    .pad_endpoint(pad)
                    .ok_or_else(|| format!("{spec} is not a live pad"))?;
                if pad_net != Some(net) {
                    return Err(format!(
                    "{spec} is not on net {} -- connect_pins first, or route from the right pin",
                    args.net
                ));
                }
                Ok(center)
            } else if let Some(p) = point {
                if p.len() != 2 {
                    return Err(format!("{which}_mm must be [x_mm, y_mm]"));
                }
                Ok(Point::new(
                    (p[0] * MM as f64).round() as Unit,
                    (p[1] * MM as f64).round() as Unit,
                ))
            } else {
                Err(format!(
                    "give {which}_pin (\"REF.PIN\") or {which}_mm ([x, y])"
                ))
            }
        };
    let start = match resolve(&args.start_pin, &args.start_mm, "start") {
        Ok(p) => p,
        Err(e) => return error_json(e),
    };
    let goal = match resolve(&args.end_pin, &args.end_mm, "end") {
        Ok(p) => p,
        Err(e) => return error_json(e),
    };

    let layer = match mcp_routing::parse_layer(args.layer.as_deref().unwrap_or("FCu")) {
        Ok(l) => l,
        Err(e) => return error_json(e),
    };
    let width = args
        .width_mm
        .map(|v| (v * MM as f64).round() as Unit)
        .unwrap_or(crate::routing::DEFAULT_TRACE_WIDTH);
    let edge_margin = match args.edge_margin_mm {
        Some(v) => {
            let margin = (v * MM as f64).round() as Unit;
            if margin < JlcpcbDfm::COPPER_TO_ROUTED_EDGE {
                return error_json(format!(
                    "edge_margin_mm {v} is below JLCPCB's hard copper-to-edge minimum of 0.2mm"
                ));
            }
            margin
        }
        None => mcp_routing::EDGE_COMFORT_MARGIN,
    };
    let opts = SuggestOptions {
        layer,
        width,
        edge_margin,
        step: (args.step_mm.unwrap_or(0.5).clamp(0.1, 2.0) * MM as f64).round() as Unit,
        bend_penalty: (args.bend_penalty_mm.unwrap_or(0.4).max(0.0) * MM as f64).round() as Unit,
        max_expansions: args.max_expansions.unwrap_or(80_000).min(400_000) as usize,
    };

    let found = match mcp_routing::suggest_route(&state.doc, net, start, goal, &opts) {
        Ok(s) => s,
        Err(e) => return error_json(e),
    };

    let points_mm: Vec<serde_json::Value> = found
        .points
        .iter()
        .map(|p| serde_json::json!([p.x as f64 / MM as f64, p.y as f64 / MM as f64]))
        .collect();
    let layer_str = match layer {
        LayerId::FCu => "FCu",
        LayerId::BCu => "BCu",
    };
    let candidate = serde_json::json!({
        "net": net_record.name,
        "width_mm": width as f64 / MM as f64,
        "edge_margin_mm": edge_margin as f64 / MM as f64,
        "segments": [{ "layer": layer_str, "points_mm": points_mm }],
    });
    let length_mm: f64 = found
        .points
        .windows(2)
        .map(|leg| leg[0].distance(leg[1]) / MM as f64)
        .sum();
    let mut result = serde_json::json!({
        "ok": true,
        "net": net_record.name,
        "layer": layer_str,
        "points_mm": candidate["segments"][0]["points_mm"].clone(),
        "length_mm": length_mm,
        "bends": found.points.len().saturating_sub(2),
        "expansions": found.expansions,
        "route_candidate": candidate,
        "committed": false,
    });
    if args.commit == Some(true) {
        let route = match mcp_routing::parse_route_candidate(&state.doc, &candidate) {
            Ok(r) => r,
            Err(e) => {
                return error_json(format!(
                    "internal: the suggested route failed to re-parse: {e}"
                ))
            }
        };
        state.cancel_transient_gestures();
        match state.try_mutate_doc_ok(|doc| mcp_routing::commit_route(doc, &route)) {
            Ok(v) => {
                result["committed"] = serde_json::json!(true);
                result["commit"] = v;
            }
            Err(e) => {
                return error_json(format!(
                    "found a path but couldn't commit it on {}: {e}",
                    net_record.name
                ))
            }
        }
    }
    result
}

/// [`crate::mcp::McpQuery::NewBoard`]'s handler -- the same
/// [`NewBoardParams::create`] + [`load_templates`] + [`EditorState::new`]
/// sequence as the GUI's own "Create" button, guarded so an AI can't
/// silently discard the human's open board (`replace_current` must be
/// explicitly true for that).
#[cfg(not(target_arch = "wasm32"))]
fn new_board_write(
    screen: &mut Screen,
    parts_db: &PartsDb,
    args: crate::mcp::NewBoardArgs,
) -> serde_json::Value {
    if let Some(err) = board_write_lock_error(screen) {
        return err;
    }
    if matches!(screen, Screen::Editor(_)) && !args.replace_current.unwrap_or(false) {
        return error_json(
            "a board is already open -- pass replace_current=true to discard it (ask the human first: unsaved work and undo history would be gone)",
        );
    }
    match args.layer_count {
        None | Some(2) => {}
        Some(n) => {
            return error_json(format!(
                "layer_count {n} is not supported -- Alladin is 2-layer only"
            ))
        }
    }
    let copper_weight = match args.copper_weight_oz {
        None | Some(1) => CopperWeight::OneOz,
        Some(2) => CopperWeight::TwoOz,
        Some(n) => {
            return error_json(format!(
                "copper_weight_oz {n} is not a JLCPCB profile -- use 1 or 2"
            ))
        }
    };
    let params = NewBoardParams {
        width_mm: args.width_mm as f32,
        height_mm: args.height_mm as f32,
        layer_count: LayerCount::Two,
        copper_weight,
        corner_radius_mm: args.corner_radius_mm.unwrap_or(1.0) as f32,
    };
    if !params.is_valid() {
        return error_json(format!(
            "{}x{} mm with corner radius {} mm is not a physically sane board",
            params.width_mm, params.height_mm, params.corner_radius_mm
        ));
    }
    let (templates, template_origin, template_hover, template_category) = load_templates(parts_db);
    *screen = Screen::Editor(EditorState::new(
        params.create(),
        templates,
        template_origin,
        template_hover,
        template_category,
    ));
    serde_json::json!({ "ok": true, "width_mm": args.width_mm, "height_mm": args.height_mm, "layer_count": 2, "copper_weight_oz": match copper_weight { CopperWeight::OneOz => 1, CopperWeight::TwoOz => 2 } })
}

/// Shared MCP/GUI copy while a desktop zone-refill worker owns the board.
#[cfg(not(target_arch = "wasm32"))]
const ZONE_REFILL_LOCK_MSG: &str =
    "zone refill is running -- board writes are locked until it finishes (retry in a few seconds)";

/// `Some(error_json)` while [`EditorState::zone_refill_active`], so a
/// write cannot land on the live doc and then be stomped by the
/// worker's finished clone.
#[cfg(not(target_arch = "wasm32"))]
fn board_write_lock_error(screen: &Screen) -> Option<serde_json::Value> {
    match screen {
        Screen::Editor(state) if state.zone_refill_active() => {
            Some(error_json(ZONE_REFILL_LOCK_MSG))
        }
        _ => None,
    }
}

/// `{ "error": message }` -- the one JSON shape every write handler
/// below returns on failure, so an MCP client can reliably check for an
/// `"error"` key regardless of which tool it called.
#[cfg(not(target_arch = "wasm32"))]
fn error_json(message: impl Into<String>) -> serde_json::Value {
    serde_json::json!({ "error": message.into() })
}

/// [`no_board_open_json`]'s error-shaped twin -- every write handler
/// above needs the `"error"` key (see [`error_json`]) rather than the
/// bare `"note"` the read-only `*_json` builders use, so an MCP client
/// can check for `"error"` consistently across every tool.
#[cfg(not(target_arch = "wasm32"))]
fn no_board_open_json_error() -> serde_json::Value {
    error_json("no board is open yet -- create one first (new_board, or open one in the GUI)")
}

#[cfg(test)]
mod undo_tests {
    use super::*;
    use alladin_geom::Point;

    fn empty_editor() -> EditorState {
        EditorState::new(
            NewBoardParams::default().create(),
            footprint::builtin_templates(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
        )
    }

    #[test]
    fn undo_restores_footprint_after_place() {
        let mut state = empty_editor();
        let template = state.templates[0].clone();
        assert!(state.doc.footprints.is_empty());
        state
            .try_mutate_doc(|doc| {
                doc.try_place_footprint(&template, Point::new(0, 0), 0.0)
                    .map(|_| ())
            })
            .unwrap();
        assert_eq!(state.doc.footprints.len(), 1);
        assert_eq!(state.undo_stack.len(), 1);
        assert!(state.undo());
        assert!(state.doc.footprints.is_empty());
        assert!(state.redo());
        assert_eq!(state.doc.footprints.len(), 1);
    }

    #[test]
    fn new_mutation_clears_redo() {
        let mut state = empty_editor();
        let template = state.templates[0].clone();
        state
            .try_mutate_doc(|doc| {
                doc.try_place_footprint(&template, Point::new(0, 0), 0.0)
                    .map(|_| ())
            })
            .unwrap();
        state.undo();
        assert_eq!(state.redo_stack.len(), 1);
        state
            .try_mutate_doc(|doc| {
                doc.try_place_footprint(&template, Point::new(5 * MM, 0), 0.0)
                    .map(|_| ())
            })
            .unwrap();
        assert!(state.redo_stack.is_empty());
        assert!(!state.redo());
    }

    #[test]
    fn undo_stack_is_capped() {
        let mut state = empty_editor();
        for _ in 0..(UNDO_LIMIT + 5) {
            state.mutate_doc(|doc| {
                let _ = doc.create_net();
            });
        }
        assert_eq!(state.undo_stack.len(), UNDO_LIMIT);
    }
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod mcp_handler_tests {
    use super::*;
    use alladin_geom::Point;

    fn editor_screen() -> Screen {
        Screen::Editor(EditorState::new(
            NewBoardParams::default().create(),
            footprint::builtin_templates(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
        ))
    }

    /// Name of some builtin template with at least `pads` pads -- the
    /// tests below don't care which part it is, only that it's real.
    fn a_template_with_pads(screen: &Screen, pads: usize) -> String {
        let Screen::Editor(state) = screen else {
            unreachable!()
        };
        state
            .templates
            .iter()
            .find(|t| t.pads.len() >= pads && t.holes.is_empty())
            .expect("some builtin multi-pad template")
            .name
            .to_string()
    }

    fn place(screen: &mut Screen, template: &str, x_mm: f64, y_mm: f64) -> serde_json::Value {
        place_footprint_write(
            screen,
            crate::mcp::PlaceFootprintArgs {
                template: template.to_string(),
                x_mm,
                y_mm,
                rotation_deg: None,
                zone_connection: None,
            },
        )
    }

    #[test]
    fn place_footprint_returns_a_reference_and_undo_takes_it_back() {
        let mut screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        let result = place(&mut screen, &template, 0.0, 0.0);
        assert_eq!(result["ok"], true, "unexpected: {result}");
        let reference = result["reference"].as_str().unwrap().to_string();
        assert!(!reference.is_empty());
        let Screen::Editor(state) = &mut screen else {
            unreachable!()
        };
        assert_eq!(state.doc.footprints.len(), 1);
        assert!(state.undo());
        assert!(state.doc.footprints.is_empty());
    }

    #[test]
    fn place_and_set_zone_connection_toggle_thermal_and_solid() {
        let mut screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        let placed = place_footprint_write(
            &mut screen,
            crate::mcp::PlaceFootprintArgs {
                template: template.clone(),
                x_mm: 0.0,
                y_mm: 0.0,
                rotation_deg: None,
                zone_connection: Some("solid".into()),
            },
        );
        assert_eq!(placed["ok"], true, "unexpected: {placed}");
        assert_eq!(placed["zone_connection"], "solid");
        let reference = placed["reference"].as_str().unwrap().to_string();
        {
            let Screen::Editor(state) = &screen else {
                unreachable!()
            };
            let pad = state.doc.footprints[0].pad_item_ids[0];
            match state.doc.node.get(pad) {
                Some(Item::Pad {
                    zone_connection, ..
                }) => assert_eq!(*zone_connection, ZoneConnection::Solid),
                other => panic!("expected pad, got {other:?}"),
            }
        }
        let toggled = set_zone_connection_write(
            &mut screen,
            crate::mcp::SetZoneConnectionArgs {
                reference,
                zone_connection: "thermal".into(),
            },
        );
        assert_eq!(toggled["ok"], true, "unexpected: {toggled}");
        assert_eq!(toggled["zone_connection"], "thermal");
        let Screen::Editor(state) = &screen else {
            unreachable!()
        };
        let pad = state.doc.footprints[0].pad_item_ids[0];
        match state.doc.node.get(pad) {
            Some(Item::Pad {
                zone_connection, ..
            }) => assert_eq!(*zone_connection, ZoneConnection::Thermal),
            other => panic!("expected pad, got {other:?}"),
        }
    }

    #[test]
    fn probe_placement_reports_illegal_pose_without_mutating() {
        let screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        let result = probe_placement_json(
            &screen,
            crate::mcp::ProbePlacementArgs {
                template: Some(template),
                reference: None,
                x_mm: 500.0,
                y_mm: 500.0,
                rotation_deg: None,
                search_radius_mm: None,
                search_step_mm: None,
            },
        );
        assert_eq!(result["ok"], true, "unexpected: {result}");
        assert_eq!(result["legal"], false, "unexpected: {result}");
        let Screen::Editor(state) = &screen else {
            unreachable!()
        };
        assert!(state.doc.footprints.is_empty());
        assert!(state.undo_stack.is_empty());
    }

    #[test]
    fn probe_placement_search_finds_a_legal_spot_near_a_collision() {
        let mut screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        assert_eq!(place(&mut screen, &template, 0.0, 0.0)["ok"], true);
        let result = probe_placement_json(
            &screen,
            crate::mcp::ProbePlacementArgs {
                template: Some(template),
                reference: None,
                x_mm: 0.0,
                y_mm: 0.0,
                rotation_deg: None,
                search_radius_mm: Some(8.0),
                search_step_mm: Some(0.5),
            },
        );
        assert_eq!(result["ok"], true, "unexpected: {result}");
        assert_eq!(result["legal"], true, "unexpected: {result}");
        assert!(result["suggested"].is_object(), "unexpected: {result}");
        let Screen::Editor(state) = &screen else {
            unreachable!()
        };
        assert_eq!(state.doc.footprints.len(), 1, "probe must not place");
    }

    fn two_pin_test_template() -> crate::footprint::FootprintTemplate {
        crate::footprint::straight_row_template("test-2pin".into(), "P".into(), 2, 2.54, 0.45)
    }

    fn editor_screen_with_two_pin() -> Screen {
        let mut templates = footprint::builtin_templates();
        templates.insert(0, two_pin_test_template());
        Screen::Editor(EditorState::new(
            NewBoardParams::default().create(),
            templates,
            Vec::new(),
            Vec::new(),
            Vec::new(),
        ))
    }

    #[test]
    fn place_parts_atomic_with_pin_nets_and_one_undo() {
        let mut screen = editor_screen_with_two_pin();
        let template = a_template_with_pads(&screen, 2);
        let mut pins = std::collections::BTreeMap::new();
        pins.insert("1".into(), "3V3".into());
        pins.insert("2".into(), "GND".into());
        let result = place_parts_write(
            &mut screen,
            crate::mcp::PlacePartsArgs {
                parts: vec![
                    crate::mcp::PlacePartSpec {
                        template: template.clone(),
                        x_mm: -8.0,
                        y_mm: 0.0,
                        rotation_deg: None,
                        pins: Some(pins.clone()),
                        zone_connection: None,
                    },
                    crate::mcp::PlacePartSpec {
                        template: template.clone(),
                        x_mm: 8.0,
                        y_mm: 0.0,
                        rotation_deg: None,
                        pins: Some(pins),
                        zone_connection: None,
                    },
                ],
            },
        );
        assert_eq!(result["ok"], true, "unexpected: {result}");
        assert_eq!(result["placed"].as_array().unwrap().len(), 2);
        assert!(result["open_bridges"]["count"].is_number());
        {
            let Screen::Editor(state) = &mut screen else {
                unreachable!()
            };
            assert_eq!(state.doc.footprints.len(), 2);
            assert!(state.doc.find_net_by_name("3V3").is_some());
            assert!(state.doc.find_net_by_name("GND").is_some());
            assert_eq!(state.undo_stack.len(), 1, "batch must be one undo frame");
            assert!(state.undo());
            assert!(state.doc.footprints.is_empty());
        }
    }

    #[test]
    fn place_parts_overlapping_batch_commits_nothing() {
        let mut screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        let result = place_parts_write(
            &mut screen,
            crate::mcp::PlacePartsArgs {
                parts: vec![
                    crate::mcp::PlacePartSpec {
                        template: template.clone(),
                        x_mm: 0.0,
                        y_mm: 0.0,
                        rotation_deg: None,
                        pins: None,
                        zone_connection: None,
                    },
                    crate::mcp::PlacePartSpec {
                        template,
                        x_mm: 0.0,
                        y_mm: 0.0,
                        rotation_deg: None,
                        pins: None,
                        zone_connection: None,
                    },
                ],
            },
        );
        assert!(result["error"].is_string(), "unexpected: {result}");
        let Screen::Editor(state) = &screen else {
            unreachable!()
        };
        assert!(state.doc.footprints.is_empty());
        assert!(state.undo_stack.is_empty());
    }

    #[test]
    fn move_parts_relocates_both_and_one_undo_restores() {
        let mut screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        let placed = place_parts_write(
            &mut screen,
            crate::mcp::PlacePartsArgs {
                parts: vec![
                    crate::mcp::PlacePartSpec {
                        template: template.clone(),
                        x_mm: -8.0,
                        y_mm: 0.0,
                        rotation_deg: None,
                        pins: None,
                        zone_connection: None,
                    },
                    crate::mcp::PlacePartSpec {
                        template,
                        x_mm: 8.0,
                        y_mm: 0.0,
                        rotation_deg: None,
                        pins: None,
                        zone_connection: None,
                    },
                ],
            },
        );
        let r1 = placed["placed"][0]["reference"]
            .as_str()
            .unwrap()
            .to_string();
        let r2 = placed["placed"][1]["reference"]
            .as_str()
            .unwrap()
            .to_string();
        let moved = move_parts_write(
            &mut screen,
            crate::mcp::MovePartsArgs {
                parts: vec![
                    crate::mcp::MovePartSpec {
                        reference: r1,
                        x_mm: -8.0,
                        y_mm: 5.0,
                        rotation_deg: None,
                    },
                    crate::mcp::MovePartSpec {
                        reference: r2,
                        x_mm: 8.0,
                        y_mm: 5.0,
                        rotation_deg: Some(90.0),
                    },
                ],
            },
        );
        assert_eq!(moved["ok"], true, "unexpected: {moved}");
        {
            let Screen::Editor(state) = &mut screen else {
                unreachable!()
            };
            assert_eq!(state.doc.footprints[0].position.y, 5 * MM);
            assert_eq!(state.doc.footprints[1].rotation_deg, 90.0);
            assert!(state.undo());
            assert_eq!(state.doc.footprints[0].position.y, 0);
            assert_eq!(state.doc.footprints[1].rotation_deg, 0.0);
        }
    }

    #[test]
    fn place_footprint_off_board_reports_an_error_and_places_nothing() {
        let mut screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        let result = place(&mut screen, &template, 500.0, 500.0);
        assert!(result["error"].is_string(), "unexpected: {result}");
        let Screen::Editor(state) = &screen else {
            unreachable!()
        };
        assert!(state.doc.footprints.is_empty());
        assert!(
            state.undo_stack.is_empty(),
            "a refused placement must not burn an undo step"
        );
    }

    #[test]
    fn place_footprint_with_unknown_template_names_the_problem() {
        let mut screen = editor_screen();
        let result = place(&mut screen, "NoSuchPart", 0.0, 0.0);
        assert!(result["error"].as_str().unwrap().contains("NoSuchPart"));
    }

    #[test]
    fn move_footprint_relocates_and_keeps_rotation() {
        let mut screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        let reference = place(&mut screen, &template, 0.0, 0.0)["reference"]
            .as_str()
            .unwrap()
            .to_string();
        let result = move_footprint_write(
            &mut screen,
            crate::mcp::MoveFootprintArgs {
                reference: reference.clone(),
                x_mm: 5.0,
                y_mm: 5.0,
                rotation_deg: None,
            },
        );
        assert_eq!(result["ok"], true, "unexpected: {result}");
        let Screen::Editor(state) = &screen else {
            unreachable!()
        };
        let fp = &state.doc.footprints[0];
        assert_eq!(fp.position, Point::new(5 * MM, 5 * MM));
        assert_eq!(fp.rotation_deg, 0.0);
    }

    #[test]
    fn remove_footprint_deletes_by_reference() {
        let mut screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        let reference = place(&mut screen, &template, 0.0, 0.0)["reference"]
            .as_str()
            .unwrap()
            .to_string();
        let result =
            remove_footprint_write(&mut screen, crate::mcp::RemoveFootprintArgs { reference });
        assert_eq!(result["ok"], true, "unexpected: {result}");
        let Screen::Editor(state) = &screen else {
            unreachable!()
        };
        assert!(state.doc.footprints.is_empty());
    }

    #[test]
    fn connect_then_disconnect_then_rename_round_trip() {
        let mut screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        let r1 = place(&mut screen, &template, -8.0, 0.0)["reference"]
            .as_str()
            .unwrap()
            .to_string();
        let r2 = place(&mut screen, &template, 8.0, 0.0)["reference"]
            .as_str()
            .unwrap()
            .to_string();

        let connected = connect_pins_write(
            &mut screen,
            crate::mcp::ConnectPinsArgs {
                ref1: r1.clone(),
                pin1: "1".into(),
                ref2: r2.clone(),
                pin2: "1".into(),
            },
        );
        assert_eq!(connected["ok"], true, "unexpected: {connected}");
        let auto_name = connected["net"].as_str().unwrap().to_string();

        let renamed = rename_net_write(
            &mut screen,
            crate::mcp::RenameNetArgs {
                net: auto_name,
                new_name: "DATA".into(),
            },
        );
        assert_eq!(renamed["ok"], true, "unexpected: {renamed}");
        {
            let Screen::Editor(state) = &screen else {
                unreachable!()
            };
            assert!(state.doc.nets.iter().any(|n| n.name == "DATA"));
        }

        let disconnected = disconnect_pin_write(
            &mut screen,
            crate::mcp::DisconnectPinArgs {
                reference: r2,
                pin: "1".into(),
            },
        );
        assert_eq!(disconnected["ok"], true, "unexpected: {disconnected}");
        let check = check_board_json(&screen);
        // r2.1 is off the net again, so the board can't be "done".
        assert_eq!(check["ok"], false);
    }

    #[test]
    fn add_pin_stitching_via_stitches_one_pin_then_the_batch_skips_it() {
        let mut screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        let r1 = place(&mut screen, &template, -8.0, 0.0)["reference"]
            .as_str()
            .unwrap()
            .to_string();
        let r2 = place(&mut screen, &template, 8.0, 0.0)["reference"]
            .as_str()
            .unwrap()
            .to_string();
        let connected = connect_pins_write(
            &mut screen,
            crate::mcp::ConnectPinsArgs {
                ref1: r1.clone(),
                pin1: "1".into(),
                ref2: r2,
                pin2: "1".into(),
            },
        );
        assert_eq!(connected["ok"], true, "{connected}");
        let net = connected["net"].as_str().unwrap().to_string();

        let one = add_pin_stitching_via_write(
            &mut screen,
            crate::mcp::AddPinStitchingViaArgs {
                reference: Some(r1),
                pin: Some("1".into()),
                net: None,
                via_diameter_mm: None,
                via_drill_mm: None,
            },
        );
        assert_eq!(one["ok"], true, "{one}");
        assert!(
            one["via_x_mm"].is_f64() && one["via_y_mm"].is_f64(),
            "{one}"
        );

        // Batch over the same net: the pad stitched above must be
        // skipped, only the other one gets a fresh via.
        let batch = add_pin_stitching_via_write(
            &mut screen,
            crate::mcp::AddPinStitchingViaArgs {
                reference: None,
                pin: None,
                net: Some(net),
                via_diameter_mm: None,
                via_drill_mm: None,
            },
        );
        assert_eq!(batch["ok"], true, "{batch}");
        assert_eq!(batch["pads"], 2, "{batch}");
        assert_eq!(batch["placed"], 1, "{batch}");
        assert_eq!(batch["skipped_already_stitched"], 1, "{batch}");
        assert_eq!(batch["failed"].as_array().unwrap().len(), 0, "{batch}");

        let Screen::Editor(state) = &mut screen else {
            unreachable!()
        };
        assert_eq!(
            state
                .doc
                .node
                .iter()
                .filter(|i| matches!(i, Item::Via { .. }))
                .count(),
            2
        );
        // The batch was one undo step: one Ctrl+Z removes exactly the
        // batch's via, the next removes the single-pin one.
        assert!(state.undo());
        assert_eq!(
            state
                .doc
                .node
                .iter()
                .filter(|i| matches!(i, Item::Via { .. }))
                .count(),
            1
        );
        assert!(state.undo());
        assert_eq!(
            state
                .doc
                .node
                .iter()
                .filter(|i| matches!(i, Item::Via { .. }))
                .count(),
            0
        );
    }

    #[test]
    fn check_board_reports_ok_only_once_everything_is_wired() {
        let mut screen = editor_screen();
        let check = check_board_json(&screen);
        assert_eq!(
            check["ok"], true,
            "an empty board has nothing unfinished: {check}"
        );

        let template = a_template_with_pads(&screen, 1);
        place(&mut screen, &template, 0.0, 0.0);
        let check = check_board_json(&screen);
        assert_eq!(
            check["ok"], false,
            "an unwired pin must fail verification: {check}"
        );
    }

    #[test]
    fn mcp_routing_scene_probe_commit_and_ripup_round_trip() {
        let mut screen = editor_screen();
        // Exactly one pad: a multi-pad part's unused pins would block a
        // straight same-net track between pin 1 of each footprint.
        let template = {
            let Screen::Editor(state) = &screen else {
                unreachable!()
            };
            state
                .templates
                .iter()
                .find(|t| t.pads.len() == 1 && t.holes.is_empty())
                .expect("single-pad template")
                .name
                .clone()
        };
        let r1 = place(&mut screen, &template, -8.0, 0.0)["reference"]
            .as_str()
            .unwrap()
            .to_string();
        let r2 = place(&mut screen, &template, 8.0, 0.0)["reference"]
            .as_str()
            .unwrap()
            .to_string();
        let connected = connect_pins_write(
            &mut screen,
            crate::mcp::ConnectPinsArgs {
                ref1: r1,
                pin1: "1".into(),
                ref2: r2,
                pin2: "1".into(),
            },
        );
        assert_eq!(connected["ok"], true, "{connected}");
        let net = connected["net"].as_str().unwrap().to_string();

        let scene = get_routing_scene_json(&screen);
        let bridges = scene["open_bridges"].as_array().unwrap();
        assert_eq!(bridges.len(), 1, "{scene}");
        let a = &bridges[0]["a"];
        let b = &bridges[0]["b"];
        // Prefer the pad's own copper layer from the scene pads list.
        let layer = scene["pads"]
            .as_array()
            .unwrap()
            .iter()
            .find(|p| p["ref"] == a["ref"] && p["pin"] == a["pin"])
            .and_then(|p| p["layer"].as_str())
            .unwrap_or("FCu");
        let route = serde_json::json!({
            "net": net,
            "segments": [{
                "layer": layer,
                "points_mm": [
                    [a["x_mm"].as_f64().unwrap(), a["y_mm"].as_f64().unwrap()],
                    [b["x_mm"].as_f64().unwrap(), b["y_mm"].as_f64().unwrap()]
                ]
            }]
        });
        let probed = probe_route_json(
            &screen,
            crate::mcp::ProbeRouteArgs {
                candidates: vec![route.clone()],
            },
        );
        assert_eq!(probed["results"][0]["ok"], true, "{probed}");
        let committed = commit_route_write(&mut screen, crate::mcp::CommitRouteArgs { route });
        assert_eq!(committed["ok"], true, "{committed}");
        assert!(get_routing_scene_json(&screen)["open_bridges"]
            .as_array()
            .unwrap()
            .is_empty());
        let ripped = ripup_wire_write(
            &mut screen,
            crate::mcp::RipupWireArgs {
                x_mm: Some(0.0),
                y_mm: Some(0.0),
                net: None,
            },
        );
        assert_eq!(ripped["ok"], true, "{ripped}");
        assert_eq!(
            get_routing_scene_json(&screen)["open_bridges"]
                .as_array()
                .unwrap()
                .len(),
            1
        );
    }

    #[test]
    fn new_board_refuses_to_discard_an_open_board_unless_told_to() {
        let parts_db = PartsDb::open_in_memory().unwrap();
        let mut screen = editor_screen();
        let args = |replace: Option<bool>| crate::mcp::NewBoardArgs {
            width_mm: 40.0,
            height_mm: 40.0,
            layer_count: None,
            copper_weight_oz: None,
            corner_radius_mm: None,
            replace_current: replace,
        };
        let refused = new_board_write(&mut screen, &parts_db, args(None));
        assert!(refused["error"]
            .as_str()
            .unwrap()
            .contains("replace_current"));

        let created = new_board_write(&mut screen, &parts_db, args(Some(true)));
        assert_eq!(created["ok"], true, "unexpected: {created}");
        let Screen::Editor(state) = &screen else {
            panic!("new_board must land in the editor")
        };
        assert!(state.doc.footprints.is_empty());
    }

    #[test]
    fn list_parts_names_every_builtin_template() {
        let parts_db = PartsDb::open_in_memory().unwrap();
        let screen = editor_screen();
        let listed = list_parts_json(&screen, &parts_db);
        let parts = listed["parts"].as_array().unwrap();
        assert_eq!(parts.len(), footprint::builtin_templates().len());
        assert!(parts
            .iter()
            .all(|p| p["name"].is_string() && p["pad_count"].is_number()));
    }

    #[test]
    fn mcp_pump_answers_list_parts_without_running_ui() {
        let parts_db = PartsDb::open_in_memory().unwrap();
        let world = std::sync::Arc::new(std::sync::Mutex::new(super::McpWorld {
            screen: editor_screen(),
            parts_db,
        }));
        let (tx, rx) = std::sync::mpsc::channel();
        super::spawn_mcp_pump(rx, world);
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
            .unwrap();
        let json = rt.block_on(async {
            let (reply, reply_rx) = tokio::sync::oneshot::channel();
            tx.send(crate::mcp::McpQuery::ListParts { reply }).unwrap();
            tokio::time::timeout(std::time::Duration::from_secs(2), reply_rx)
                .await
                .expect("pump must answer without PcbApp::ui")
                .expect("reply channel")
        });
        let listed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert!(
            listed["parts"].as_array().unwrap().len() >= 1,
            "unexpected: {listed}"
        );
    }

    fn hang_zone_refill(screen: &mut Screen) {
        let Screen::Editor(state) = screen else {
            panic!("expected editor");
        };
        let (_tx, rx) = std::sync::mpsc::channel();
        state.zone_refill = Some(ZoneRefillJob::Background {
            before: state.doc.clone(),
            started_at_generation: state.edit_generation,
            rx,
            done: 0,
            total: 1,
        });
    }

    #[test]
    fn mcp_board_writes_are_refused_while_zone_refill_runs() {
        let mut screen = editor_screen();
        let template = a_template_with_pads(&screen, 1);
        hang_zone_refill(&mut screen);

        let placed = place(&mut screen, &template, 0.0, 0.0);
        assert!(
            placed["error"]
                .as_str()
                .unwrap()
                .contains("zone refill is running"),
            "unexpected: {placed}"
        );
        let Screen::Editor(state) = &screen else {
            panic!("expected editor");
        };
        assert!(
            state.doc.footprints.is_empty(),
            "a refused place must not land on the live doc"
        );

        let summary = board_summary_json(&screen);
        assert!(
            summary.get("error").is_none(),
            "reads must keep working during refill: {summary}"
        );

        let parts_db = PartsDb::open_in_memory().unwrap();
        let created = new_board_write(
            &mut screen,
            &parts_db,
            crate::mcp::NewBoardArgs {
                width_mm: 40.0,
                height_mm: 40.0,
                layer_count: None,
                copper_weight_oz: None,
                corner_radius_mm: None,
                replace_current: Some(true),
            },
        );
        assert!(
            created["error"]
                .as_str()
                .unwrap()
                .contains("zone refill is running"),
            "unexpected: {created}"
        );
    }

    #[test]
    fn finishing_zone_refill_discards_clone_if_the_board_changed() {
        let ctx = egui::Context::default();
        let Screen::Editor(mut state) = editor_screen() else {
            panic!("expected editor");
        };
        let (tx, rx) = std::sync::mpsc::channel();
        let started_at_generation = state.edit_generation;
        state.zone_refill = Some(ZoneRefillJob::Background {
            before: state.doc.clone(),
            started_at_generation,
            rx,
            done: 0,
            total: 1,
        });
        // A write that slipped past the lock (or a future missed path)
        // must not be overwritten by the worker's clone.
        state.bump_edit_generation();
        let mut filled = state.doc.clone();
        filled.copper_weight = CopperWeight::TwoOz;
        tx.send(ZoneRefillEvent::Finished {
            doc: filled,
            errors: Vec::new(),
        })
        .unwrap();
        state.poll_zone_refill(&ctx);
        assert!(state.zone_refill.is_none());
        assert_eq!(
            state.doc.copper_weight,
            CopperWeight::OneOz,
            "changed board must keep the live doc, not the worker clone"
        );
        assert!(
            state
                .zone_message
                .as_deref()
                .unwrap_or("")
                .contains("discarded"),
            "unexpected: {:?}",
            state.zone_message
        );
    }
}
