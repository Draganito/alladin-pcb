//! The in-memory PCB document `alladin-pcb` edits: an outline plus an
//! `alladin_core::Node` world of placed items (pads/tracks/vias/zones) --
//! the single source of geometric truth for DRC-aware placement and
//! manual routing.

use alladin_core::{DfmViolation, Item, ItemId, Jlcpcb2Layer2Oz, JlcpcbClearance, JlcpcbDfm, LayerId, NetClass, NetId, Node, PadShape, RuleResolver};
use alladin_geom::{
    circle_polygon_collides, circle_within_outline, dist_segment_to_segment, polygon_polygon_collides, polygon_within_outline_with_clearance,
    segment_polygon_collides, segment_within_outline_with_clearance, Aabb, Circle, Point, Polygon, Segment, Unit, MM,
};

use crate::footprint::{world_assembly_drills, world_courtyard, world_items, FootprintTemplate};
use crate::routing::path_keeps_edge_clearance;
use crate::zone_fill;

/// Board layer count `alladin-pcb`'s "New board" dialog offers. Only 1/2
/// layers today -- a hobbyist-focused subset of what real fab houses
/// support, not a technical ceiling in `alladin_core::LayerId` (which is
/// itself only `FCu`/`BCu`; >2 layers needs inner-layer modelling first).
/// A `Four` variant existed here before, but was never wired beyond its
/// own outline/label -- picking "4" produced a board electrically
/// identical to a 2-layer one under a misleading label, so it was
/// removed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LayerCount {
    One,
    Two,
}

impl LayerCount {
    pub fn as_u8(self) -> u8 {
        match self {
            LayerCount::One => 1,
            LayerCount::Two => 2,
        }
    }

    pub const ALL: [LayerCount; 2] = [LayerCount::One, LayerCount::Two];
}

impl std::fmt::Display for LayerCount {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.as_u8())
    }
}

/// Round-trips with [`Display`](std::fmt::Display) above -- what
/// `crate::cli`'s `new-board --layers` flag parses against, so an AI or
/// script driving the CLI can pass the exact same `"1"`/`"2"` a human
/// sees in the GUI's own layer-count dropdown.
impl std::str::FromStr for LayerCount {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "1" => Ok(LayerCount::One),
            "2" => Ok(LayerCount::Two),
            other => Err(format!("invalid layer count \"{other}\" -- must be 1 or 2")),
        }
    }
}

/// Copper weight (thickness) of the outer layers `alladin-pcb`'s "New
/// board" dialog offers -- directly picks which real JLCPCB DFM/
/// clearance profile [`BoardDoc::resolver`] enforces for the whole
/// board's lifetime (see that method's own doc comment). Only the two
/// profiles [`alladin_core`] has actually ported so far
/// (`JlcpcbClearance`/[`Jlcpcb2Layer2Oz`]) -- JLCPCB's heavier 2.5/3.5/
/// 4.5oz options (2-layer only) have no Rust-side DFM data yet, so
/// aren't offered here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CopperWeight {
    OneOz,
    TwoOz,
}

impl CopperWeight {
    pub fn as_oz(self) -> u8 {
        match self {
            CopperWeight::OneOz => 1,
            CopperWeight::TwoOz => 2,
        }
    }

    pub const ALL: [CopperWeight; 2] = [CopperWeight::OneOz, CopperWeight::TwoOz];
}

impl std::fmt::Display for CopperWeight {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}oz", self.as_oz())
    }
}

/// Round-trips with [`Display`](std::fmt::Display) above -- what
/// `crate::cli`'s `new-board --copper-oz` flag parses against, same
/// convention as [`LayerCount`]'s own `FromStr`.
impl std::str::FromStr for CopperWeight {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "1" => Ok(CopperWeight::OneOz),
            "2" => Ok(CopperWeight::TwoOz),
            other => Err(format!("invalid copper weight \"{other}\" -- must be 1 or 2 (oz)")),
        }
    }
}

/// The parameters `alladin-pcb`'s "New board" dialog collects -- plain
/// millimetre `f32`s (GUI-friendly), converted to internal nanometre
/// `Unit`s only once, in [`NewBoardParams::create`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NewBoardParams {
    pub width_mm: f32,
    pub height_mm: f32,
    /// **Not yet wired to anything beyond the outline/label** -- real
    /// >2-layer support (an extra `alladin_core::LayerId` variant per
    /// inner layer, plus via span/blind/buried modelling) is not
    /// implemented; collected here so the dialog already asks for layer
    /// count without a UI rework once it is.
    pub layer_count: LayerCount,
    /// Which real JLCPCB DFM/clearance profile [`Self::create`]'s
    /// resulting [`BoardDoc`] enforces for its whole lifetime -- see
    /// [`CopperWeight`]'s own doc comment. Defaults to `OneOz`, JLCPCB's
    /// own standard/no-extra-cost option, matching every board created
    /// before this field existed.
    pub copper_weight: CopperWeight,
    pub corner_radius_mm: f32,
}

impl Default for NewBoardParams {
    fn default() -> Self {
        Self { width_mm: 50.0, height_mm: 30.0, layer_count: LayerCount::Two, copper_weight: CopperWeight::OneOz, corner_radius_mm: 1.0 }
    }
}

fn mm_to_unit(mm: f32) -> Unit {
    (mm as f64 * MM as f64).round() as Unit
}

impl NewBoardParams {
    /// Whether these params describe a physically sane board -- the
    /// dialog disables its "Create" button unless this holds, so a
    /// degenerate (zero/negative-size, or corner radius that would
    /// swallow the whole board) board can never actually be created.
    pub fn is_valid(&self) -> bool {
        self.width_mm > 0.0
            && self.height_mm > 0.0
            && self.corner_radius_mm >= 0.0
            && self.corner_radius_mm * 2.0 <= self.width_mm.min(self.height_mm)
    }

    pub fn create(&self) -> BoardDoc {
        let width = mm_to_unit(self.width_mm);
        let height = mm_to_unit(self.height_mm);
        let corner_radius = mm_to_unit(self.corner_radius_mm);
        let outline = Polygon::rounded_rect(width, height, corner_radius, 12);
        Self::board_from_outline(outline, self.layer_count, self.copper_weight)
    }

    /// Empty board whose outline is an already-built polygon (e.g. from a
    /// DXF import). Width/height/corner-radius of [`NewBoardParams`] are
    /// ignored -- the polygon is the board.
    pub fn create_with_outline(&self, outline: Polygon) -> BoardDoc {
        Self::board_from_outline(outline, self.layer_count, self.copper_weight)
    }

    fn board_from_outline(outline: Polygon, layer_count: LayerCount, copper_weight: CopperWeight) -> BoardDoc {
        BoardDoc {
            outline: vec![outline],
            layer_count,
            copper_weight,
            node: Node::new(),
            footprints: Vec::new(),
            next_footprint_serial: 0,
            nets: Vec::new(),
            next_net_serial: 0,
            zones: Vec::new(),
            next_zone_serial: 0,
            silk_texts: Vec::new(),
            next_silk_text_serial: 0,
            silk_dots: Vec::new(),
            next_silk_dot_serial: 0,
        }
    }
}

/// Identifies one placed footprint within a [`BoardDoc`]. Distinct from
/// `alladin_core::ItemId` (which identifies one *pad*, not the whole
/// part) -- a footprint owns several pad `ItemId`s, see
/// [`PlacedFootprint::pad_item_ids`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FootprintId(pub(crate) usize);

/// A part actually placed on the board: which built-in template it came
/// from (by name -- see `crate::footprint::builtin_templates`; matched
/// back to the template's real pad layout by the caller, since `BoardDoc`
/// itself deliberately holds no static template registry -- that becomes
/// the parts database's job once it exists), where, and which live
/// `Node` pads are its own (so moving/removing it can find and update
/// exactly those, nothing else). `Clone`: needed so [`BoardDoc`] itself
/// can be cloned (see that struct's own doc comment).
#[derive(Debug, Clone)]
pub struct PlacedFootprint {
    pub id: FootprintId,
    pub reference: String,
    pub template_name: String,
    pub position: Point,
    pub rotation_deg: f64,
    pub pad_item_ids: Vec<ItemId>,
    /// The `Item::Hole` counterpart of [`Self::pad_item_ids`] -- the
    /// live `Node` ids of this footprint's own mechanical holes (see
    /// `crate::footprint::HoleTemplate`/`crate::footprint::FootprintTemplate::holes`),
    /// in the same order as `template.holes`. Kept as a genuinely
    /// separate list rather than folded into `pad_item_ids` (even
    /// though `crate::footprint::world_items` happens to emit pads
    /// then holes back-to-back) so every caller that means "this
    /// footprint's *pads*" (`Self::find_pad`, `Self::footprint_at`,
    /// Gerber/BOM per-pad net resolution, all of which
    /// zip 1:1 against `template.pads`) keeps working unmodified and
    /// correctly, instead of silently zipping a hole in where a pad
    /// was expected the moment a template gains its first hole. Empty
    /// for every footprint whose template has no holes -- everything
    /// electrical, still the overwhelming majority of placed parts.
    pub hole_item_ids: Vec<ItemId>,
    /// This footprint's own mechanical body/courtyard (see
    /// [`crate::footprint::world_courtyard`]), already placed in
    /// world space at this exact `position`/`rotation_deg` --
    /// recomputed on every [`BoardDoc::try_place_footprint`]/
    /// [`BoardDoc::try_move_footprint`] call, kept here (rather than
    /// recomputed on demand from `template_name`) purely so
    /// [`BoardDoc::check_placement`]'s own body-vs-body overlap check
    /// never needs a template registry to compare a *new* candidate
    /// against every *already-placed* footprint -- `BoardDoc` itself
    /// deliberately holds no such registry (see this struct's own doc
    /// comment).
    pub courtyard: Polygon,
    /// Plated + mechanical drills of this footprint in world space
    /// (see [`crate::footprint::world_assembly_drills`]) -- kept beside
    /// [`Self::courtyard`] so lead-to-hole assembly checks can see PTH
    /// holes that are *not* separate `Item::Hole`s in the `Node` (LCSC
    /// through-hole pads stay `Item::Pad` only). Empty for pure SMD.
    pub assembly_drills: Vec<Circle>,
    /// Where this part's optional pin-1 marker dot sits, in the
    /// footprint's own **local, pre-rotation** frame -- `None` means
    /// "no marker" (the default; markers are opt-in per part, in
    /// keeping with the "only deliberately placed silk ever prints"
    /// contract). Stored as the local offset (computed
    /// once, template in hand, by [`BoardDoc::try_enable_pin1_marker`]'s
    /// legality sweep) rather than a world position, so the dot
    /// rides along with every later move/rotate of the part for free
    /// -- world center is `position + offset.rotated(rotation_deg)`,
    /// the exact composition every pad already uses. Diameter is the
    /// fixed [`PIN1_MARKER_DIAMETER`]; side is always the front (the
    /// same "components are front-side" assumption the courtyard
    /// checks already make).
    pub pin1_marker: Option<Point>,
}

impl PlacedFootprint {
    /// The pin-1 marker's printed ink in world space, if enabled --
    /// the one shape its rendering, DFM collision, and export all
    /// share, mirroring [`SilkDot::circle`]'s role for free dots.
    pub fn pin1_marker_circle(&self) -> Option<Circle> {
        self.pin1_marker.map(|offset| Circle::new(offset.rotated(self.rotation_deg).add(self.position), PIN1_MARKER_DIAMETER / 2))
    }
}

/// Identifies one placed [`SilkText`] within a [`BoardDoc`] -- own id
/// space, distinct from [`ItemId`]/[`FootprintId`]: a silk text is
/// neither a `Node` item (see [`SilkText`]'s own doc comment for why)
/// nor owned by any footprint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SilkTextId(pub usize);

/// A user-placed, free-standing silkscreen annotation -- a project
/// title, a polarity mark, a warning label, anything beyond the
/// per-footprint reference/value text manufacturing export already
/// emits at a fixed offset (those are not modelled as `SilkText` and
/// get no collision checking today -- a separate gap this type does
/// not attempt to close).
///
/// Deliberately **not** an [`alladin_core::Item`] variant, so never
/// lives in [`BoardDoc::node`]/its `Node` -- exactly the same
/// reasoning that already keeps [`PlacedFootprint::courtyard`] outside
/// `Node`: `Node`'s whole model (same-net exemptions, a via/hole
/// spanning both copper layers, obstacle search) is built around
/// *electrical copper*. Silk has no net, isn't electrically continuous
/// with anything, and must never become a routing obstacle a track has
/// to detour around -- folding it into `Item` would force every one of
/// `Node`'s copper-specific match arms to grow a meaningless arm just
/// to stay total, and would risk treating a label as real copper by
/// mistake. Collision against real copper (the one place a silk text
/// *does* need to know about pads, see [`BoardDoc::silk_text_fits`])
/// instead reuses the same plain geometry helpers
/// (`circle_polygon_collides`/`polygon_polygon_collides`) `Node`'s own
/// collision model is built on, just called directly against
/// `self.node`'s pads rather than going through it.
#[derive(Debug, Clone, PartialEq)]
pub struct SilkText {
    pub id: SilkTextId,
    pub text: String,
    pub position: Point,
    pub rotation_deg: f64,
    /// Which side this prints on -- `FCu` means F.SilkS (front, over
    /// the front copper/pads), `BCu` means B.SilkS. Reuses
    /// [`LayerId`] (a copper-layer enum) purely as "which side of the
    /// board", the same convenient reuse [`Self::silk_text_fits`]'s
    /// same-side pad comparison depends on -- a silk text has no
    /// copper of its own and is never itself a `Node` item (see this
    /// struct's own doc comment).
    pub layer: LayerId,
    /// Character height. Stroke-font rendering is what this actually
    /// becomes ink-wise; [`Self::bounding_rect`] only ever needs it for
    /// a conservative collision rectangle.
    pub height: Unit,
    /// Stroke width -- clamped to at least
    /// [`JlcpcbDfm::MIN_SILK_LINE_WIDTH`] at every entry point that
    /// creates or edits a `SilkText` (see [`BoardDoc::try_place_silk_text`]),
    /// the same "never below the real DFM floor" contract
    /// [`crate::routing::DEFAULT_TRACE_WIDTH`] already documents for tracks.
    pub line_width: Unit,
}


/// [`SilkText::height`]/[`SilkText::line_width`] for every new text a
/// caller doesn't pick sizes for itself -- height is a plain,
/// readable-on-a-real-board default (matching common hand-soldering-
/// friendly labels), *not* a JLCPCB minimum (there is no official
/// minimum text height, only [`JlcpcbDfm::MIN_SILK_LINE_WIDTH`] for
/// the stroke itself); line width sits comfortably above that real
/// floor (0.16mm) rather than exactly on it, for the same "default
/// above the minimum, not pinned to it" reasoning
/// `crate::routing::DEFAULT_TRACE_WIDTH` already documents for track
/// width.
pub const DEFAULT_SILK_TEXT_HEIGHT: Unit = 1_000_000;
pub const DEFAULT_SILK_LINE_WIDTH: Unit = 200_000;

/// The GUI's own size-stepper choices for [`SilkText::height`]: the
/// default 1.0mm plus four progressively larger steps, cycled through
/// by `EditorState`'s "bigger"/"smaller" buttons rather than a free-form
/// numeric field -- a handful of sane, always-legible-per-JLCPCB sizes
/// (see [`DEFAULT_SILK_TEXT_HEIGHT`]'s own doc comment: there is no
/// official minimum text height, so any of these is already fine) beats
/// letting someone dial in an oddly-specific 1.37mm by accident.
pub const SILK_TEXT_HEIGHT_STEPS_MM: [f64; 5] = [1.0, 1.5, 2.0, 2.5, 3.0];

impl SilkText {
    /// Every drawn stroke of this text as world-space capsule segments
    /// (each carrying [`Self::line_width`]) -- the *real*, as-printed
    /// ink, laid out by `crate::stroke_font` from the embedded Hershey
    /// Futural glyphs. This is the one shape the GUI draws, DFM
    /// collision checks (see [`BoardDoc::silk_text_fits`]/
    /// [`BoardDoc::check_placement`]), and native Gerber strokes -- so
    /// preview, placement, and fabrication never disagree. Rotated about
    /// [`Self::position`] by [`Self::rotation_deg`] the same way every
    /// other rotated shape in this crate is (see [`Point::rotated`]).
    pub fn stroke_segments(&self) -> Vec<Segment> {
        let to_world = |(x, y): (f64, f64)| Point::new(x.round() as Unit, y.round() as Unit).rotated(self.rotation_deg).add(self.position);
        let mut segments = Vec::new();
        for polyline in crate::stroke_font::layout_polylines(&self.text, self.height as f64, self.line_width as f64) {
            for pair in polyline.windows(2) {
                segments.push(Segment::new(to_world(pair[0]), to_world(pair[1]), self.line_width));
            }
        }
        segments
    }

    /// The tight, axis-aligned-then-rotated rectangle around this
    /// text's real ink (stroke centerlines from the same
    /// `crate::stroke_font` layout as [`Self::stroke_segments`],
    /// inflated by half the stroke width) -- the click target for
    /// [`BoardDoc::silk_text_at`], the GUI's selection ring, and the
    /// conservative board-edge check in [`BoardDoc::silk_text_fits`].
    /// Deliberately *not* what pad/text/body collision runs against
    /// (that's the per-stroke segments): clicking the gap between two
    /// words should still select the text, but nothing may be refused
    /// because of that same blank gap. Never empty even for a
    /// whitespace-only string -- see `stroke_font::layout_bounds`'s
    /// own fallback.
    pub fn bounding_rect(&self) -> Polygon {
        let (min_x, min_y, max_x, max_y) = crate::stroke_font::layout_bounds(&self.text, self.height as f64, self.line_width as f64);
        let corners = [
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
        ];
        Polygon::new(
            corners
                .iter()
                .map(|&(x, y)| Point::new(x.round() as Unit, y.round() as Unit).rotated(self.rotation_deg).add(self.position))
                .collect(),
        )
    }
}

/// Whether one silk stroke capsule (see [`SilkText::stroke_segments`])
/// comes closer than `margin` to a pad's copper -- the one distance
/// formula both directions of the silk-over-copper rule share:
/// [`BoardDoc::silk_text_fits`] (new/moved silk vs existing pads) and
/// [`BoardDoc::check_placement`] (new/moved pads vs existing silk),
/// so the two can never drift apart numerically.
fn stroke_hits_pad(seg: &Segment, shape: &PadShape, margin: Unit) -> bool {
    match shape {
        PadShape::Circle(c) => dist_segment_to_segment((seg.a, seg.b), (c.center, c.center)) < (c.radius + seg.width / 2 + margin) as f64,
        PadShape::Polygon { outline, .. } => segment_polygon_collides(seg, outline, margin),
    }
}

/// Whether two filled circles come closer than `clearance` -- the one
/// distance formula every dot-vs-dot and dot-vs-round-pad rule here
/// shares (a [`SilkDot`]/pin-1 marker against a circular pad, another
/// dot, another marker).
fn pad_shape_hits_circle(shape: &PadShape, circle: &Circle, clearance: Unit) -> bool {
    match shape {
        PadShape::Circle(c) => circles_touch(c, circle, clearance),
        PadShape::Polygon { outline, .. } => circle_polygon_collides(circle, outline, clearance),
    }
}

fn circles_touch(a: &Circle, b: &Circle, clearance: Unit) -> bool {
    ((a.center.x - b.center.x) as f64).hypot((a.center.y - b.center.y) as f64) < (a.radius + b.radius + clearance) as f64
}

/// How far a pad template's copper plausibly reaches from its own
/// center in any direction -- the circumscribing radius
/// [`BoardDoc::try_enable_pin1_marker`]'s sweep clears before adding
/// [`JlcpcbDfm::SILK_TO_PAD`]. Same deliberately-generous formula as
/// `crate::app`'s reference-label `pad_reach` mirror: a dot a hair
/// further out than strictly necessary is harmless, one clipping the
/// pad is a real DRC defect.
fn pad_template_reach(pad: &crate::footprint::PadTemplate) -> Unit {
    match pad.shape {
        crate::footprint::PadShapeKind::Circle => pad.radius,
        crate::footprint::PadShapeKind::Rect { width, height } | crate::footprint::PadShapeKind::Oval { width, height } => {
            (((width as f64).powi(2) + (height as f64).powi(2)).sqrt() / 2.0).round() as Unit
        }
    }
}

/// Why [`BoardDoc::try_place_silk_text`]/[`BoardDoc::try_move_silk_text`]
/// was refused -- deliberately its own type rather than reusing
/// [`PlacementError`]: every variant here is about a *silk* rule
/// specifically (silk-to-pad, silk-to-silk, silk-to-edge), none of
/// which map onto `PlacementError`'s copper/body-oriented variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SilkTextError {
    /// `text` was empty or whitespace-only -- a label with no visible
    /// content isn't a placement Alladin will create.
    EmptyText,
    /// Closer to a same-side copper pad than
    /// [`JlcpcbDfm::SILK_TO_PAD`] allows -- a real "silk over copper"
    /// DRC violation if placed anyway (see [`SilkText`]'s own doc
    /// comment for the fixed/per-footprint reference text this
    /// deliberately does *not* also check).
    TooCloseToPad,
    /// Overlaps another already-placed [`SilkText`] on the same side
    /// -- no official JLCPCB numeric rule for this (unlike
    /// `TooCloseToPad`), but two overlapping labels are unreadable on
    /// the real board regardless, so this is refused outright (zero
    /// tolerance, not a numeric clearance) rather than only
    /// documented.
    OverlapsAnotherText,
    /// Would leave the board (or come closer to the edge than a
    /// same-side pad already has to, via
    /// [`JlcpcbDfm::COPPER_TO_ROUTED_EDGE`] reused here as the same
    /// "don't hug the cut line" margin).
    OffBoard,
    /// `id` (for [`BoardDoc::try_move_silk_text`]/[`BoardDoc::remove_silk_text`])
    /// doesn't refer to a currently-placed silk text.
    NotFound,
    /// The text's real ink would end up underneath an already-placed
    /// footprint's body/courtyard -- a label hidden under a soldered
    /// part is unreadable on the real board, so this is refused
    /// outright (zero tolerance, same reasoning as
    /// [`Self::OverlapsAnotherText`]) even though it isn't a numeric
    /// JLCPCB rule the way [`Self::TooCloseToPad`] is. The exact
    /// mirror of [`PlacementError::OverSilkText`]'s own body arm, so
    /// the outcome never depends on whether the part or the text was
    /// placed first.
    UnderComponentBody,
    /// The text's ink would touch an already-placed [`SilkDot`] (or a
    /// footprint's pin-1 marker) on the same side -- same "overlapping
    /// silk is unreadable, zero tolerance" reasoning as
    /// [`Self::OverlapsAnotherText`], just against the round kind of
    /// deliberate silk instead of the written kind.
    OverlapsDot,
}

impl std::fmt::Display for SilkTextError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SilkTextError::EmptyText => write!(f, "silk text can't be empty"),
            SilkTextError::TooCloseToPad => write!(f, "too close to a pad on this side -- would print silk over copper"),
            SilkTextError::OverlapsAnotherText => write!(f, "overlaps another silk text on this side"),
            SilkTextError::OffBoard => write!(f, "would leave the board, or hug its edge too closely"),
            SilkTextError::NotFound => write!(f, "no such silk text"),
            SilkTextError::UnderComponentBody => write!(f, "would print the text underneath a component's body, where it can never be read"),
            SilkTextError::OverlapsDot => write!(f, "overlaps a silkscreen dot on this side"),
        }
    }
}

/// Identifies one placed [`SilkDot`] within a [`BoardDoc`] -- own id
/// space, same reasoning as [`SilkTextId`]'s.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SilkDotId(pub usize);

/// A deliberately placed, filled silkscreen dot -- a polarity mark, an
/// orientation cue, any small round annotation the user wants printed.
/// The round sibling of [`SilkText`], and like it deliberately **not**
/// an [`alladin_core::Item`] (see that struct's doc comment for the
/// full "silk is not copper" reasoning). Exports as a *filled circle*
/// (`gr_circle` with `fill`), which KiCad/Gerber reproduce exactly --
/// no font involved at all, so preview and fabrication output are
/// trivially the same shape.
#[derive(Debug, Clone, PartialEq)]
pub struct SilkDot {
    pub id: SilkDotId,
    pub position: Point,
    /// Full diameter of the printed dot -- clamped to at least
    /// [`JlcpcbDfm::MIN_SILK_LINE_WIDTH`] at every creating/resizing
    /// entry point (a dot thinner than the thinnest printable silk
    /// stroke would simply vanish in fabrication), same clamping
    /// contract as [`SilkText::line_width`]'s.
    pub diameter: Unit,
    /// Which side this prints on -- same [`LayerId`]-as-board-side
    /// reuse as [`SilkText::layer`].
    pub layer: LayerId,
}

impl SilkDot {
    /// The dot's printed ink, as the one geometric shape every check
    /// and every renderer of a dot shares.
    pub fn circle(&self) -> Circle {
        Circle::new(self.position, self.diameter / 2)
    }
}

/// Why a [`SilkDot`] placement/move/resize (or enabling a footprint's
/// pin-1 marker, which is geometrically the same dot) was refused --
/// the round counterpart of [`SilkTextError`], its own type for the
/// same reason that one is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SilkDotError {
    /// Closer to a same-side copper pad than
    /// [`JlcpcbDfm::SILK_TO_PAD`] allows -- identical rule (and
    /// margin) to [`SilkTextError::TooCloseToPad`].
    TooCloseToPad,
    /// Would touch another silk dot, a pin-1 marker, or a silk text's
    /// real ink on the same side -- unreadable regardless of any
    /// numeric rule, same zero-tolerance reasoning as
    /// [`SilkTextError::OverlapsAnotherText`].
    OverlapsSilk,
    /// Would leave the board or hug the cut line closer than
    /// [`JlcpcbDfm::COPPER_TO_ROUTED_EDGE`] -- same edge rule as
    /// [`SilkTextError::OffBoard`].
    OffBoard,
    /// Would end up underneath a placed footprint's body, where it
    /// can never be seen -- same rule as
    /// [`SilkTextError::UnderComponentBody`].
    UnderComponentBody,
    /// `id` doesn't refer to a currently-placed silk dot (or, for the
    /// pin-1 marker entry points, the [`FootprintId`] doesn't exist).
    NotFound,
    /// Enabling a pin-1 marker found no legal spot anywhere around
    /// pad 1 -- every candidate in [`BoardDoc::try_enable_pin1_marker`]'s
    /// sweep violated one of the rules above (typical on a very dense
    /// board where neighbouring pads/parts crowd the pin-1 corner).
    NoRoomNearPin1,
}

impl std::fmt::Display for SilkDotError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SilkDotError::TooCloseToPad => write!(f, "too close to a pad on this side -- would print silk over copper"),
            SilkDotError::OverlapsSilk => write!(f, "overlaps other silkscreen (a text, dot, or pin-1 marker) on this side"),
            SilkDotError::OffBoard => write!(f, "would leave the board, or hug its edge too closely"),
            SilkDotError::UnderComponentBody => write!(f, "would print the dot underneath a component's body, where it can never be seen"),
            SilkDotError::NotFound => write!(f, "no such silk dot"),
            SilkDotError::NoRoomNearPin1 => write!(f, "no legal spot for a pin-1 dot anywhere around this part's pad 1"),
        }
    }
}

/// [`SilkDot::diameter`] for every new dot a caller doesn't size
/// itself -- comfortably legible next to typical 0603..SOIC parts and
/// far above [`JlcpcbDfm::MIN_SILK_LINE_WIDTH`], same "default above
/// the floor, not pinned to it" reasoning as
/// [`DEFAULT_SILK_LINE_WIDTH`]'s.
pub const DEFAULT_SILK_DOT_DIAMETER: Unit = 400_000;

/// The GUI's size-stepper choices for [`SilkDot::diameter`] -- same
/// "handful of sane sizes beats a free-form field" reasoning as
/// [`SILK_TEXT_HEIGHT_STEPS_MM`]. The smallest step sits at 0.3mm,
/// still nearly double JLCPCB's thinnest printable silk stroke.
pub const SILK_DOT_DIAMETER_STEPS_MM: [f64; 5] = [0.3, 0.4, 0.5, 0.8, 1.0];

/// Diameter of every footprint's optional pin-1 marker dot (see
/// [`PlacedFootprint::pin1_marker`]) -- one fixed, deliberately small
/// size rather than per-part configuration: the marker's whole job is
/// "this corner is pin 1", and a uniform dot reads more consistently
/// across a board than per-part sizes would.
pub const PIN1_MARKER_DIAMETER: Unit = 400_000;

/// Identifies one user-drawn copper zone/pour within a [`BoardDoc`].
/// Distinct from any single `alladin_core::ItemId` -- a zone typically
/// fills into several disjoint `Item::Zone` islands (see
/// [`ZoneRecord::item_ids`]), and re-filling it (`Self::refill_zone`)
/// replaces however many of those the current board state produces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ZoneId(pub(crate) usize);

/// A zone/pour as the user actually defined it: the outline they drew,
/// its target layer/net, and which live `Item::Zone` island(s) in
/// `self.node` are its current fill result. Kept as its own
/// source-of-truth record -- separate from those `Item::Zone`s
/// themselves -- for exactly the reason `crate::zone_fill`'s own module
/// doc comment gives for not just storing the raw drawn outline as
/// copper: the *filled* result depends on the rest of the board (other
/// items' clearances) at fill time, and can go stale the moment a new
/// track is routed underneath it. [`BoardDoc::refill_zone`]/
/// [`BoardDoc::refill_all_zones`] are how a stale fill gets brought back
/// in sync with the current board -- by re-running the fill from this
/// record's own `outline`/`layer`/`net`, never by trying to patch the
/// old filled islands directly. `Clone`: needed so [`BoardDoc`] itself
/// can be cloned (see that struct's own doc comment).
#[derive(Debug, Clone)]
pub struct ZoneRecord {
    pub id: ZoneId,
    /// The polygon the user actually drew (in [`Tool::DrawZone`]'s UI
    /// terms) -- board-outline clipping and obstacle clearance are
    /// re-derived from this every time, not stored.
    pub outline: Polygon,
    pub layer: LayerId,
    pub net: NetId,
    /// The `Item::Zone`(s) in `self.node` this record's most recent fill
    /// produced -- zero if the last fill came back empty (outline fully
    /// outside the board, or fully consumed by obstacle clearances).
    pub item_ids: Vec<ItemId>,
    /// `self.node.obstacle_revision()` as of this record's most recent
    /// fill -- compared against the *current* `obstacle_revision` by
    /// [`BoardDoc::zones_are_stale`] to tell a still-current fill from
    /// one that's fallen behind a footprint move/new track/new via
    /// since (see that method's own doc comment for the full picture).
    pub filled_at_revision: u64,
}

/// Why a placement/move was refused -- surfaced to the UI so a rejected
/// drag can explain itself instead of just silently snapping back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlacementError {
    /// At least one pad would land outside the board outline (or inside
    /// a cutout/hole -- see [`alladin_geom::circle_within_outline`]), or
    /// closer to the board edge than JLCPCB's manufacturable
    /// `copper_to_routed_edge` minimum allows (see
    /// [`alladin_core::JlcpcbDfm::COPPER_TO_ROUTED_EDGE`]) -- both are
    /// surfaced identically since a script/AI driving [`crate::cli`]
    /// only needs to know "move it further from the edge/outline",
    /// not which of the two geometric reasons applied.
    OffBoard,
    /// At least one pad would collide with an existing item, under the
    /// active [`alladin_core::RuleResolver`]'s clearance rules. Carries
    /// the number of colliding pad/item pairs found, purely for a more
    /// informative message -- not meant to be parsed.
    Collision(usize),
    /// This footprint's own mechanical body/courtyard (see
    /// [`crate::footprint::FootprintTemplate::courtyard`]) would
    /// overlap an already-placed footprint's body -- distinct from
    /// [`Self::Collision`] (a *copper* clearance violation): two real
    /// components' plastic/metal cases can never physically occupy the
    /// same space even where their pads happen not to collide (e.g. a
    /// small part placed underneath a big module's footprint), while
    /// a routed `Item::Track`/`Item::Via` legitimately *can* run
    /// underneath a body -- this check only ever compares footprint
    /// bodies against each other.
    BodyOverlap,
    /// This footprint's own mechanical body/courtyard would come
    /// closer to the board edge than JLCPCB's real, official
    /// *assembly* rule allows -- distinct from [`Self::OffBoard`]
    /// (which only ever inflates a *pad's* copper by the much smaller
    /// `copper_to_routed_edge` *fabrication* margin, see
    /// [`alladin_core::JlcpcbDfm::COMPONENT_BODY_TO_EDGE`]'s own doc
    /// comment for the exact source and number): a part's real
    /// physical body can, and routinely does, overhang its own
    /// copper pads (gull-wing leads, a big module's plastic shell over
    /// small edge pads, ...), so passing the pad-only `OffBoard` check
    /// is not sufficient on its own to guarantee the *assembled* board
    /// is actually manufacturable.
    BodyOffBoard,
    /// At least one pad would land on (or within
    /// [`alladin_core::JlcpcbDfm::SILK_TO_PAD`] of) an existing
    /// [`SilkText`]'s real printed ink on the same side of the board
    /// -- the exact mirror of [`SilkTextError::TooCloseToPad`], which
    /// already refuses printing new silk over an existing pad: the
    /// same JLCPCB rule (silkscreen must not be printed over exposed
    /// copper) is violated either way, regardless of which of the two
    /// was placed first, so placing/moving a component onto existing
    /// silk has to be refused just as symmetrically. Checked against
    /// the text's per-character ink shape ([`SilkText::ink_cells`]),
    /// so a pad genuinely fitting into a string's blank space (a wide
    /// gap, the band above lowercase letters) is still legal.
    OverSilkText,
    /// Would land a pad on (or the part's body over) a deliberately
    /// placed [`SilkDot`] or another part's pin-1 marker -- the
    /// dot-shaped counterpart of [`Self::OverSilkText`], same
    /// both-directions reasoning.
    OverSilkDot,
    /// An SMD pad ("lead") would come closer to another footprint's
    /// plated or mechanical drill than
    /// [`alladin_core::JlcpcbDfm::COMPONENT_LEAD_TO_HOLE`] allows --
    /// JLCPCB assembly DFM "Lead to hole distance". Distinct from
    /// [`Self::Collision`] (copper clearance against items already in
    /// the `Node`) because PTH drills are not always separate
    /// `Item::Hole`s.
    LeadToHole,
    /// The item's own scalar geometry already violates a JLCPCB DFM
    /// floor (track too thin, via pad/drill/annular ring too small) --
    /// distinct from every other variant here, which are about where
    /// the item would *sit* relative to the board or other items. See
    /// [`DfmViolation`].
    Dfm(DfmViolation),
}

impl std::fmt::Display for PlacementError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlacementError::OffBoard => write!(f, "would leave the board outline, or come closer to its edge than JLCPCB can manufacture"),
            PlacementError::Collision(n) => write!(f, "would collide with {n} existing pad/item(s)"),
            PlacementError::BodyOverlap => write!(
                f,
                "would place bodies closer than JLCPCB's required {:.2}mm assembly clearance",
                JlcpcbDfm::COMPONENT_BODY_CLEARANCE as f64 / MM as f64
            ),
            PlacementError::BodyOffBoard => {
                write!(f, "would place the component's body within JLCPCB's required 2.5mm assembly clearance of the board edge")
            }
            PlacementError::OverSilkText => {
                write!(f, "would put a pad on top of (or too close to) existing silkscreen text -- JLCPCB does not print silk over exposed copper")
            }
            PlacementError::OverSilkDot => {
                write!(f, "would put a pad or the part's body on top of a silkscreen dot/pin-1 marker")
            }
            PlacementError::LeadToHole => {
                write!(
                    f,
                    "would place an SMD pad within JLCPCB's required {:.2}mm lead-to-hole clearance of another part's drill",
                    alladin_core::JlcpcbDfm::COMPONENT_LEAD_TO_HOLE as f64 / MM as f64
                )
            }
            PlacementError::Dfm(v) => write!(f, "{v}"),
        }
    }
}

/// Why [`BoardDoc::try_add_stitching_via`] was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViaError {
    /// The plain geometric placement itself already failed -- see
    /// [`PlacementError`].
    Placement(PlacementError),
    /// The via is geometrically valid (no collision, on-board) but
    /// doesn't touch any existing copper on its own net, so it would be
    /// an electrically pointless, dangling standalone via -- see
    /// [`alladin_core::Node::touches_same_net`]'s doc comment.
    Dangling,
}

impl std::fmt::Display for ViaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ViaError::Placement(e) => write!(f, "{e}"),
            ViaError::Dangling => write!(f, "wouldn't touch any existing pad/track/zone on that net -- place it directly on top of one, or route a track to it first"),
        }
    }
}

/// The result of a successful [`BoardDoc::try_add_pin_stitching_via`] --
/// both new items, so a caller (the GUI's right-click menu, the
/// `add_pin_stitching_via` MCP tool) can report exactly what was
/// placed without re-deriving the via's center from `pad_id` itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinStitchingVia {
    pub via_id: ItemId,
    pub center: Point,
}

/// Why [`BoardDoc::try_add_pin_stitching_via`] was refused. Distinct
/// from [`ViaError`] (which is about a via placed at an *explicit*
/// point the caller already chose): every variant here can only
/// happen because this method picked the point itself, so each one
/// tells the caller something about *why the pin's own neighbourhood*
/// didn't work, not just "here's a `PlacementError`, go figure out
/// why".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinStitchingViaError {
    /// `pad_id` isn't a live pad at all.
    NotAPad,
    /// The pad exists but isn't on any net yet -- [`BoardDoc::connect_pads`]
    /// (or a schematic-derived net assignment) has to run first.
    NoNet,
    /// The natural candidate point -- just outside the pad's own
    /// copper, radially away from the footprint's own body center --
    /// already fails a plain via placement:
    /// [`PlacementError::OffBoard`] (too close to the board edge in
    /// that direction) or [`PlacementError::Collision`] (something
    /// else already occupies that exact spot). Carries the underlying
    /// reason so a caller can tell a user "move the part first"
    /// without having to guess why.
    Via(PlacementError),
    /// The via itself placed cleanly, but the short straight stub
    /// track needed to actually connect it back to the pad doesn't --
    /// rolled the via straight back out again (same "touches nothing
    /// on `Err`" contract as every other placement primitive here).
    /// Typically means the pad's immediate neighbourhood is too tight
    /// (e.g. a copper pour right up against the pad on a different
    /// net) even though the via's own footprint had room a fraction of
    /// a millimetre further out.
    NoRoomForStub,
}

impl std::fmt::Display for PinStitchingViaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PinStitchingViaError::NotAPad => write!(f, "not a pad"),
            PinStitchingViaError::NoNet => write!(f, "this pin isn't on a net yet -- connect it first"),
            PinStitchingViaError::Via(e) => write!(f, "{e}"),
            PinStitchingViaError::NoRoomForStub => {
                write!(f, "a via would fit there, but not the short track needed to connect it back to the pin -- move the part and try again")
            }
        }
    }
}

/// Why [`BoardDoc::set_outline`] was refused -- every existing item
/// that would fall foul of the *new* outline stays exactly where it
/// is, and the outline itself is left unchanged, same "trial first,
/// commit only on success" contract as [`PlacementError`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(not(test), allow(dead_code))]
pub enum SetOutlineError {
    /// At least one already-placed footprint's pad would end up
    /// off-board (or too close to the new edge) under the new outline.
    FootprintOffBoard(FootprintId),
    /// At least one already-routed `Item::Track` would end up off-board.
    TrackOffBoard,
    /// At least one already-placed `Item::Via` would end up off-board.
    ViaOffBoard,
}

impl std::fmt::Display for SetOutlineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SetOutlineError::FootprintOffBoard(id) => write!(f, "footprint {id:?} would fall outside the new outline"),
            SetOutlineError::TrackOffBoard => write!(f, "an existing track would fall outside the new outline"),
            SetOutlineError::ViaOffBoard => write!(f, "an existing via would fall outside the new outline"),
        }
    }
}

/// One user-named electrical net. `alladin_core::NetId` (already what
/// every `Item::net()` carries) is the real identity; this just remembers
/// a human-readable name for it, which `Item` itself has no room for --
/// same net-id / display-name split used throughout the editor.
/// `Clone`: needed so [`BoardDoc`] itself can be cloned (see that
/// struct's own doc comment).
#[derive(Debug, Clone)]
pub struct NetRecord {
    pub id: NetId,
    pub name: String,
}

/// Why a pin-to-net operation was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetError {
    /// The targeted item isn't a pad at all (tracks/vias/zones have no
    /// "assign this to a net by clicking it" UI yet).
    NotAPad,
    /// [`BoardDoc::connect_pads`] was asked to join two pads that already
    /// each belong to a *different* existing net. Net **merging** (making
    /// every pad of one net join the other) is deliberately not
    /// implemented, to keep "which net wins, and does the loser's name
    /// just vanish" out of scope rather than picking an arbitrary answer.
    AlreadyOnDifferentNets,
}

impl std::fmt::Display for NetError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NetError::NotAPad => write!(f, "not a pad"),
            NetError::AlreadyOnDifferentNets => {
                write!(f, "both pins already belong to different nets (merging isn't supported yet)")
            }
        }
    }
}

/// Why [`BoardDoc::rename_net`] was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RenameNetError {
    /// No net with that [`NetId`] exists (e.g. stale id from before an
    /// undo, or a typo'd id over MCP).
    NotFound,
    /// The name was empty (or all whitespace) after trimming.
    EmptyName,
    /// Some *other* net already has this exact name -- see
    /// [`BoardDoc::rename_net`]'s own doc comment for why uniqueness is
    /// enforced, not just a cosmetic nicety.
    NameAlreadyUsed,
}

impl std::fmt::Display for RenameNetError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RenameNetError::NotFound => write!(f, "no net with that id exists"),
            RenameNetError::EmptyName => write!(f, "net name can't be empty"),
            RenameNetError::NameAlreadyUsed => write!(f, "another net already has that name"),
        }
    }
}

/// Hobby-friendly default via size (0.6mm outer diameter / 0.3mm drill --
/// the same pairing KiCad itself defaults new projects to), comfortably
/// above JLCPCB's real minimums (see [`JlcpcbDfm::MIN_VIA_DIAMETER`]/
/// [`JlcpcbDfm::MIN_VIA_HOLE`]/[`JlcpcbDfm::MIN_VIA_ANNULAR_RING`] --
/// `(600_000 - 300_000) / 2 = 150_000`, well past the 100_000 minimum
/// annular ring). No per-via size picker yet; same defaulting idea as
/// [`crate::routing::DEFAULT_TRACE_WIDTH`].
pub const DEFAULT_VIA_DIAMETER: Unit = 600_000;
pub const DEFAULT_VIA_DRILL: Unit = 300_000;

/// A board actually open for editing: outline plus the live item world.
/// Starts life with an empty `node` (a freshly created board has no
/// footprints on it yet) -- populated by manual part placement,
/// pin-to-net connect, and interactive routing.
///
/// `Clone`: every field is plain owned data (`Node` already carries its
/// own manual `Clone` impl, see that type's own doc comment for why it
/// isn't `#[derive(Clone)]`) -- useful for snapshots (e.g. zone fill
/// working on a cloned `Node` while the live board stays editable).
/// `Send`/`Sync` follow automatically from every field already being
/// `Send`/`Sync` (see `Node`'s own doc comment on why *it* has to be).
#[derive(Clone)]
pub struct BoardDoc {
    pub outline: Vec<Polygon>,
    pub layer_count: LayerCount,
    /// Which real JLCPCB DFM/clearance profile [`Self::resolver`]
    /// enforces for every placement/routing/zone-fill check on this
    /// board -- see [`CopperWeight`]'s own doc comment.
    pub copper_weight: CopperWeight,
    pub node: Node,
    pub footprints: Vec<PlacedFootprint>,
    pub(crate) next_footprint_serial: usize,
    /// Every net the user has created so far, purely by directly
    /// connecting pads to each other in the layout -- see
    /// [`Self::connect_pads`]'s doc comment for why there's deliberately
    /// no schematic/netlist-import step behind this.
    pub nets: Vec<NetRecord>,
    pub(crate) next_net_serial: u32,
    /// Every zone/pour the user has drawn so far (see [`ZoneRecord`]'s
    /// doc comment) -- empty on a freshly created board, same as
    /// `footprints`/`nets` above.
    pub zones: Vec<ZoneRecord>,
    pub(crate) next_zone_serial: usize,
    /// Every free-standing silk annotation the user has placed so far
    /// (see [`SilkText`]'s own doc comment) -- empty on a freshly
    /// created board, same as `footprints`/`nets`/`zones` above.
    pub silk_texts: Vec<SilkText>,
    pub(crate) next_silk_text_serial: usize,
    /// Every deliberately placed [`SilkDot`] -- same source-of-truth
    /// role (and same not-in-`node` reasoning) as [`Self::silk_texts`].
    pub silk_dots: Vec<SilkDot>,
    pub(crate) next_silk_dot_serial: usize,
}

impl BoardDoc {
    /// Every clearance/collision check goes through this one resolver --
    /// the same real JLCPCB DFM rules used everywhere, not a placeholder.
    /// Picks between the two profiles `alladin_core` has ported so far
    /// based on this board's own [`CopperWeight`] (set once, at
    /// [`NewBoardParams::create`] time, and persisted -- see
    /// [`crate::persistence`]'s own `copper_weight` field). Both
    /// `JlcpcbClearance`/`Jlcpcb2Layer2Oz` are zero-sized types, so
    /// returning a `'static` reference to either is free -- no
    /// allocation, no `Box<dyn _>`.
    pub(crate) fn resolver(&self) -> &'static dyn RuleResolver {
        match self.copper_weight {
            CopperWeight::OneOz => &JlcpcbClearance,
            CopperWeight::TwoOz => &Jlcpcb2Layer2Oz,
        }
    }

    /// This board's own pad-to-pad JLCPCB minimum, copper-weight-aware
    /// so [`Self::pin_stitching_via_candidate`] never hardcodes a
    /// specific profile's constant. Both ported profiles happen to
    /// define the same `PAD_TO_PAD` value today
    /// (`Jlcpcb2Layer2Oz::PAD_TO_PAD` is a direct alias of
    /// `JlcpcbClearance::PAD_TO_PAD`), so this makes no observable
    /// difference *yet* -- it's here so a future profile that diverges
    /// on `PAD_TO_PAD` doesn't assume one fixed profile.
    pub(crate) fn pad_to_pad_clearance(&self) -> Unit {
        match self.copper_weight {
            CopperWeight::OneOz => JlcpcbClearance::PAD_TO_PAD,
            CopperWeight::TwoOz => Jlcpcb2Layer2Oz::PAD_TO_PAD,
        }
    }

    /// Whether `template` placed at `position`/`rotation_deg` is
    /// geometrically legal *right now*: every pad stays fully on the
    /// board outline (holes/cutouts included) and none collides with any
    /// existing item. `moving` excludes one already-placed footprint's
    /// own pads from the collision check -- used when dragging it to a
    /// new spot so it never "collides with itself"; `None` when placing
    /// a brand new footprint. This is the one gate both
    /// [`Self::try_place_footprint`] and [`Self::try_move_footprint`]
    /// share, and what the UI calls every frame while a placement/drag
    /// ghost is live, so "correct-by-construction" placement never
    /// depends on the two call sites staying in sync by hand.
    pub fn check_placement(
        &self,
        template: &FootprintTemplate,
        position: Point,
        rotation_deg: f64,
        moving: Option<FootprintId>,
    ) -> Result<(), PlacementError> {
        let excluding = moving.map(|id| vec![id]).unwrap_or_default();
        self.check_placement_excluding(template, position, rotation_deg, moving, &excluding)
    }

    /// [`Self::check_placement`] with an explicit set of footprints whose
    /// *current* pads/holes/bodies are ignored as obstacles -- used when
    /// dragging so a footprint never collides with its own seat.
    /// `moving` still supplies the candidate's own pad nets (same zone
    /// same-net fast-path reason as [`Self::check_placement`]).
    fn check_placement_excluding(
        &self,
        template: &FootprintTemplate,
        position: Point,
        rotation_deg: f64,
        moving: Option<FootprintId>,
        excluding: &[FootprintId],
    ) -> Result<(), PlacementError> {
        let exclude_ids: Vec<ItemId> = self
            .footprints
            .iter()
            .filter(|f| excluding.contains(&f.id))
            .flat_map(|f| f.pad_item_ids.iter().chain(&f.hole_item_ids).copied())
            .collect();

        // `world_items` always builds fresh `Item::Pad`s with `net:
        // None` (nets are assigned after placement, not part of the
        // static template -- see that function's own doc comment), so
        // for an *already-placed* footprint being dragged to a new
        // spot, this candidate would otherwise always look net-less
        // here even on a pad that's actually long since been wired up.
        // That silently defeats `Node::query_colliding`'s own
        // same-net fast path against a same-net `Item::Zone` (a solid
        // ground/power plane), since a real net and `None` are never
        // equal -- every single candidate position a routed part could
        // ever be dragged to, including the one it's already
        // sitting at, would then spuriously "collide" with its own
        // plane and never validate, freezing the part in place. Fixed
        // the same way `Self::try_move_footprint`'s own commit loop
        // already carries real nets across a move: look each pad's
        // *current* net up by its existing `ItemId` before checking.
        let existing_pad_nets: Vec<Option<NetId>> = moving
            .and_then(|id| self.footprints.iter().find(|f| f.id == id))
            .map(|f| f.pad_item_ids.iter().map(|&id| self.node.get(id).and_then(Item::net)).collect())
            .unwrap_or_default();

        let resolver = self.resolver();
        let mut colliding = 0usize;
        for (index, mut item) in world_items(template, position, rotation_deg).into_iter().enumerate() {
            if let Item::Pad { net, .. } = &mut item {
                *net = existing_pad_nets.get(index).copied().flatten();
            }
            if let Item::Pad { shape, .. } = &item {
                // Inflating the pad's own extent by JLCPCB's real
                // `copper_to_routed_edge` minimum (not just checking
                // "is the pad's own copper on-board") is what actually
                // makes this a DFM gate rather than a bare geometry
                // check -- a pad flush against the outline passes a
                // bare on-board check alone but would be rejected by
                // every real fab as too close to the cut line. See
                // `JlcpcbDfm::COPPER_TO_ROUTED_EDGE`'s doc comment.
                // `PadShape::Polygon` (a non-round pad's true, possibly
                // rotated outline) is checked exactly, edge-to-edge,
                // via `polygon_within_outline_with_clearance` --
                // `circle_within_outline`'s own boundary-sampling
                // wouldn't see a rotated rectangular pad's corner
                // poking past the edge between samples.
                let on_board = match shape {
                    PadShape::Circle(c) => circle_within_outline(c.center, c.radius + JlcpcbDfm::COPPER_TO_ROUTED_EDGE, &self.outline),
                    PadShape::Polygon { outline, .. } => {
                        polygon_within_outline_with_clearance(outline, JlcpcbDfm::COPPER_TO_ROUTED_EDGE, &self.outline)
                    }
                };
                if !on_board {
                    return Err(PlacementError::OffBoard);
                }
            }
            if let Item::Hole { position, drill } = &item {
                // A mechanical hole has no copper, so `COPPER_TO_ROUTED_EDGE`
                // (a *copper*-to-edge rule) doesn't literally apply --
                // but a drill that breaks through the routed edge is
                // just as unmanufacturable as copper that does, so this
                // still needs its own edge margin rather than a bare
                // on-board check. `MIN_NPTH_HOLE` (the smallest hole
                // this fab will drill at all post-plating, see that
                // constant's own doc comment) is reused here as that
                // margin -- there's no dedicated "npth_to_edge" DFM
                // constant to reach for instead, and it's already in
                // the right ballpark (0.5mm, more conservative than
                // `COPPER_TO_ROUTED_EDGE`'s 0.2mm), which is exactly
                // the "same spirit, not the same number" choice made
                // for `Item::Hole`'s clearance rules in
                // `alladin_core::JlcpcbClearance` too.
                let on_board = circle_within_outline(*position, drill / 2 + JlcpcbDfm::MIN_NPTH_HOLE, &self.outline);
                if !on_board {
                    return Err(PlacementError::OffBoard);
                }
            }
            // A copper pour is never itself a placement/move obstacle
            // here -- `Node::item_collides`'s own doc comment already
            // covers why (a fill is a point-in-time snapshot that can
            // go stale, `Self::refill_zone`/`Self::refill_all_zones`
            // exist precisely to recompute it against the board as it
            // looks *now*) -- so `query_colliding` never even hands
            // back a `Zone` hit for this to need filtering out.
            colliding += self.node.query_colliding(&item, resolver).into_iter().filter(|hit| !exclude_ids.contains(hit)).count();
        }

        if colliding > 0 {
            return Err(PlacementError::Collision(colliding));
        }

        // The exact mirror of `Self::silk_text_fits`'s own pad check
        // (see `PlacementError::OverSilkText`'s doc comment): silk may
        // not be printed over a pad, so a pad may not be placed/moved
        // onto existing silk either -- same `SILK_TO_PAD` margin, same
        // real stroke-ink shape (via the shared `stroke_hits_pad`),
        // same same-side-only scope.
        for item in world_items(template, position, rotation_deg) {
            let Item::Pad { shape, layer, .. } = &item else { continue };
            for text in self.silk_texts.iter().filter(|t| t.layer == *layer) {
                if text.stroke_segments().iter().any(|seg| stroke_hits_pad(seg, shape, JlcpcbDfm::SILK_TO_PAD)) {
                    return Err(PlacementError::OverSilkText);
                }
            }
            // Same rule for round silk (free dots + pin-1 markers) --
            // every excluded footprint's own marker rides along with
            // the batch and must not block its own (or a sibling's)
            // destination, or a multi-move would self-collide.
            let over_dot = self.silk_dot_circles_on(*layer, None, excluding).iter().any(|c| match shape {
                PadShape::Circle(p) => circles_touch(c, p, JlcpcbDfm::SILK_TO_PAD),
                PadShape::Polygon { outline, .. } => circle_polygon_collides(c, outline, JlcpcbDfm::SILK_TO_PAD),
            });
            if over_dot {
                return Err(PlacementError::OverSilkDot);
            }
        }

        // JLCPCB assembly "Lead to hole" before body-vs-body: for an
        // NPTH/mounting-hole courtyard floored to the drill AABB, the
        // two gates share the same 0.3 mm number on a concentric
        // approach, and pad-vs-hole copper clearance is only 0.15 mm --
        // reporting the more specific lead-to-hole when both fire.
        if self.lead_to_hole_violated(template, position, rotation_deg, excluding) {
            return Err(PlacementError::LeadToHole);
        }

        // Copper/pad clearance is clean at this point -- still reject
        // if this template's own *body* would come within
        // [`JlcpcbDfm::COMPONENT_BODY_CLEARANCE`] of an already-placed
        // footprint's body (see `PlacementError::BodyOverlap`). Bodies
        // of every `excluding` footprint are skipped here; new-vs-new
        // body spacing for multi-place flows is
        // [`Self::check_matrix_placement`]'s scratch-courtyard pass.
        let candidate_courtyard = world_courtyard(template, position, rotation_deg);
        let overlaps_another_body = self.footprints.iter().filter(|fp| !excluding.contains(&fp.id)).any(|fp| {
            polygon_polygon_collides(&candidate_courtyard, &fp.courtyard, JlcpcbDfm::COMPONENT_BODY_CLEARANCE)
        });
        if overlaps_another_body {
            return Err(PlacementError::BodyOverlap);
        }

        // The body arm of `PlacementError::OverSilkText` (the pad arm
        // ran above): a part dropped *on top of* an existing label
        // hides it just as thoroughly as printing the label under the
        // part would -- which `SilkTextError::UnderComponentBody`
        // already refuses -- so this direction is refused too, or the
        // outcome would depend on placement order. Front-side silk
        // only, same reasoning as `silk_text_fits`'s own body check.
        let covers_a_text = self
            .silk_texts
            .iter()
            .filter(|t| t.layer == LayerId::FCu)
            .any(|t| t.stroke_segments().iter().any(|seg| segment_polygon_collides(seg, &candidate_courtyard, 0)));
        if covers_a_text {
            return Err(PlacementError::OverSilkText);
        }

        // The body arm for round silk, mirroring `covers_a_text` just
        // above -- a part dropped onto a dot/marker hides it exactly
        // like it would hide a label.
        let covers_a_dot =
            self.silk_dot_circles_on(LayerId::FCu, None, excluding).iter().any(|c| circle_polygon_collides(c, &candidate_courtyard, 0));
        if covers_a_dot {
            return Err(PlacementError::OverSilkDot);
        }

        // Real, official JLCPCB *assembly* rule (see
        // `PlacementError::BodyOffBoard`'s own doc comment): the
        // part's real body, not just its pads, must clear the board
        // edge by a full 2.5mm.
        if !polygon_within_outline_with_clearance(&candidate_courtyard, JlcpcbDfm::COMPONENT_BODY_TO_EDGE, &self.outline) {
            return Err(PlacementError::BodyOffBoard);
        }

        Ok(())
    }

    /// Replaces the board's own outline (see [`Self::outline`] --
    /// arbitrary polygons, not just [`NewBoardParams::create`]'s rounded
    /// rect: chamfers, notches, cutouts, multiple pieces, all already
    /// supported end-to-end by every placement/DRC/zone-fill/export path
    /// that reads `self.outline`, see `crate::cli`'s `set-outline`
    /// subcommand). Unlike [`Self::check_placement`]'s per-placement
    /// gate, this re-validates *everything already on the board*
    /// against the *new* shape first -- every placed footprint's pads
    /// (via `templates`, the same lookup [`Self::find_pad`] needs),
    /// every `Item::Track`, every `Item::Via` -- and touches nothing at
    /// all if any of them would fall off-board or too close to the new
    /// edge, exactly the "trial first, commit only on success" contract
    /// every other mutating method here already follows. A footprint
    /// whose `template_name` no longer resolves against `templates` is
    /// silently skipped (same stale-reference tolerance as
    /// [`Self::find_pad`]) rather than treated as an error -- there's
    /// nothing more specific to check without its real pad geometry.
    ///
    /// On success, every current zone is [`Self::refill_all_zones`]d
    /// against the new outline too -- deliberately *not* a rejection
    /// condition like footprints/tracks/vias above: a zone's fill is
    /// already expected to shrink/grow as the rest of the board changes
    /// (see [`ZoneRecord`]'s own doc comment), so re-clipping it to a
    /// smaller board is exactly what a stale fill's normal "catch up"
    /// path already does, not a new failure mode to invent here.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn set_outline(&mut self, new_outline: Vec<Polygon>, templates: &[FootprintTemplate]) -> Result<(), SetOutlineError> {
        for fp in &self.footprints {
            let Some(template) = templates.iter().find(|t| t.name == fp.template_name) else { continue };
            for item in world_items(template, fp.position, fp.rotation_deg) {
                if let Item::Pad { shape, .. } = &item {
                    let on_board = match shape {
                        PadShape::Circle(c) => circle_within_outline(c.center, c.radius + JlcpcbDfm::COPPER_TO_ROUTED_EDGE, &new_outline),
                        PadShape::Polygon { outline, .. } => {
                            polygon_within_outline_with_clearance(outline, JlcpcbDfm::COPPER_TO_ROUTED_EDGE, &new_outline)
                        }
                    };
                    if !on_board {
                        return Err(SetOutlineError::FootprintOffBoard(fp.id));
                    }
                }
                if let Item::Hole { position, drill } = &item {
                    // Same `MIN_NPTH_HOLE`-as-edge-margin choice as
                    // `Self::check_placement`'s own `Item::Hole` arm --
                    // see that arm's doc comment for why.
                    if !circle_within_outline(*position, drill / 2 + JlcpcbDfm::MIN_NPTH_HOLE, &new_outline) {
                        return Err(SetOutlineError::FootprintOffBoard(fp.id));
                    }
                }
            }
            // Same real assembly body-to-edge rule as
            // `Self::check_placement`'s own `PlacementError::BodyOffBoard`
            // check -- a shrunk/reshaped outline can just as easily
            // strand an already-placed body too close to the *new*
            // edge as it can a pad.
            let courtyard = world_courtyard(template, fp.position, fp.rotation_deg);
            if !polygon_within_outline_with_clearance(&courtyard, JlcpcbDfm::COMPONENT_BODY_TO_EDGE, &new_outline) {
                return Err(SetOutlineError::FootprintOffBoard(fp.id));
            }
        }

        for item in self.node.iter() {
            match item {
                Item::Track { shape, .. } => {
                    if !segment_within_outline_with_clearance(shape.a, shape.b, shape.width, JlcpcbDfm::COPPER_TO_ROUTED_EDGE, &new_outline) {
                        return Err(SetOutlineError::TrackOffBoard);
                    }
                }
                Item::Via { shape, .. } => {
                    if !circle_within_outline(shape.center, shape.radius + JlcpcbDfm::COPPER_TO_ROUTED_EDGE, &new_outline) {
                        return Err(SetOutlineError::ViaOffBoard);
                    }
                }
                _ => {}
            }
        }

        self.outline = new_outline;
        self.refill_all_zones();
        Ok(())
    }

    /// Places a brand new `template` instance, hard-gated by
    /// [`Self::check_placement`] -- the `Node` and `self.footprints` are
    /// left completely untouched on `Err`, exactly like every other
    /// "trial first, commit only on success" pattern in this workspace.
    /// Auto-generates a reference designator from
    /// `template.reference_prefix` plus a monotonically increasing
    /// counter (`P1`, `P2`, ... -- never reused, even after a removal,
    /// so a reference never silently gets reassigned to a different part).
    pub fn try_place_footprint(
        &mut self,
        template: &FootprintTemplate,
        position: Point,
        rotation_deg: f64,
    ) -> Result<FootprintId, PlacementError> {
        // Template-geometry DFM (hard drill floors -- see
        // `crate::footprint::template_dfm_hard_violations` for why the
        // PTH ring and SMD pad floor stay report-only) gates *new*
        // placements only, not `check_placement` itself: that one also
        // runs for every frame of a move-drag, and a part already on
        // the board must stay movable even if its template predates a
        // rule.
        if let Some((_, violation)) = crate::footprint::template_dfm_hard_violations(&template.pads, &template.holes).into_iter().next() {
            return Err(PlacementError::Dfm(violation));
        }
        // Same new-placements-only reasoning for the drill-spacing rule:
        // an NPTH mounting hole must not land nearer to an existing
        // via/hole than the drill process allows.
        for item in world_items(template, position, rotation_deg) {
            if let Item::Hole { position: hole_position, drill } = item {
                if self.violates_hole_to_hole(hole_position, drill, None) {
                    return Err(PlacementError::Dfm(DfmViolation::HoleToHoleBelowMin));
                }
            }
        }
        self.check_placement(template, position, rotation_deg, None)?;

        let mut pad_item_ids = Vec::new();
        let mut hole_item_ids = Vec::new();
        for item in world_items(template, position, rotation_deg) {
            let is_hole = matches!(item, Item::Hole { .. });
            let item_id = self.node.add(item);
            if is_hole {
                hole_item_ids.push(item_id);
            } else {
                pad_item_ids.push(item_id);
            }
        }

        self.next_footprint_serial += 1;
        let id = FootprintId(self.next_footprint_serial);
        self.footprints.push(PlacedFootprint {
            id,
            reference: format!("{}{}", template.reference_prefix, self.next_footprint_serial),
            template_name: template.name.to_string(),
            position,
            rotation_deg,
            pad_item_ids,
            hole_item_ids,
            courtyard: world_courtyard(template, position, rotation_deg),
            assembly_drills: world_assembly_drills(template, position, rotation_deg),
            pin1_marker: None,
        });
        Ok(id)
    }

    /// The centered grid of positions a `rows`x`cols` matrix placement
    /// (see [`Self::check_matrix_placement`]/[`Self::place_matrix`])
    /// resolves to: `center` is the geometric center of the *whole*
    /// grid, not its first cell -- so dragging a matrix ghost by its
    /// center is what makes `crate::app::EditorState`'s board-center/
    /// symmetric-margin snapping meaningful (snapping this `center`
    /// point to the board's own center automatically makes left/right
    /// *and* top/bottom margins equal at once, since the grid is
    /// symmetric around it by construction). A 1x1 "matrix" degenerates
    /// to the single point `center`, so callers don't need a separate
    /// code path for an ordinary single-part placement.
    pub fn matrix_positions(rows: u32, cols: u32, pitch_x: Unit, pitch_y: Unit, center: Point) -> Vec<Point> {
        let mut positions = Vec::with_capacity((rows as usize) * (cols as usize));
        for row in 0..rows {
            for col in 0..cols {
                let dx = (col as f64 - (cols as f64 - 1.0) / 2.0) * pitch_x as f64;
                let dy = (row as f64 - (rows as f64 - 1.0) / 2.0) * pitch_y as f64;
                positions.push(Point::new(center.x + dx.round() as Unit, center.y + dy.round() as Unit));
            }
        }
        positions
    }

    /// The dry-run half of matrix placement (see [`Self::place_matrix`]
    /// for the committing half, same "trial first, commit only on
    /// success" split as [`Self::check_placement`]/[`Self::try_place_footprint`]):
    /// **every** `positions` entry must be individually legal against
    /// the *current* board (off-board/edge-clearance/collision, exactly
    /// [`Self::check_placement`]'s own rules), and no two matrix members
    /// may collide with *each other* either -- checked by replaying
    /// every position's pads into a scratch [`Node`] (starting empty,
    /// **not** a clone of `self.node`: the real board was already
    /// checked in the loop just above, so re-including it here would
    /// only cost time, not catch anything new) and rejecting the first
    /// position whose own pads would collide with an earlier position's
    /// already-added ones. Chosen deliberately over rejecting just the
    /// individually-colliding cells: a matrix is placed as one indivisible
    /// unit -- silently placing 140 of 143 LEDs because 3 didn't fit
    /// would leave a confusing, hard-to-notice gap in a regular-looking
    /// grid.
    pub fn check_matrix_placement(&self, template: &FootprintTemplate, positions: &[Point], rotation_deg: f64) -> Result<(), PlacementError> {
        for &position in positions {
            self.check_placement(template, position, rotation_deg, None)?;
        }

        let resolver = self.resolver();
        let mut scratch = Node::new();
        // `check_placement` above already caught every position that
        // would overlap an *already-placed* footprint's body -- this
        // second pass catches bodies overlapping *each other* within
        // the very same new batch (e.g. too tight a pitch for a real
        // LED's actual body size), the same "new-vs-new" gap
        // `scratch`/`is_colliding` below already closes for pads.
        let mut scratch_courtyards: Vec<Polygon> = Vec::with_capacity(positions.len());
        for &position in positions {
            let courtyard = world_courtyard(template, position, rotation_deg);
            if scratch_courtyards.iter().any(|other| polygon_polygon_collides(&courtyard, other, JlcpcbDfm::COMPONENT_BODY_CLEARANCE)) {
                return Err(PlacementError::BodyOverlap);
            }
            scratch_courtyards.push(courtyard);

            for item in world_items(template, position, rotation_deg) {
                if scratch.is_colliding(&item, resolver) {
                    return Err(PlacementError::Collision(1));
                }
                scratch.add(item);
            }
        }
        Ok(())
    }

    /// Commits a whole `rows`x`cols` matrix of `template` instances at
    /// `positions` (see [`Self::matrix_positions`]) in one indivisible
    /// step: [`Self::check_matrix_placement`] gates the *entire* batch
    /// first, so a board that fails the check is left completely
    /// untouched -- no partial grid ever gets committed. Each placed
    /// instance still goes through the ordinary [`Self::try_place_footprint`]
    /// (auto-generated reference designators, sequential and never
    /// reused, exactly like placing the same template one click at a
    /// time would produce).
    pub fn place_matrix(&mut self, template: &FootprintTemplate, positions: &[Point], rotation_deg: f64) -> Result<Vec<FootprintId>, PlacementError> {
        self.check_matrix_placement(template, positions, rotation_deg)?;
        let mut ids = Vec::with_capacity(positions.len());
        for &position in positions {
            ids.push(self.try_place_footprint(template, position, rotation_deg)?);
        }
        Ok(ids)
    }

    /// Places `template` **without** [`Self::check_placement`]'s
    /// collision/off-board gate (routing test helpers / already-validated
    /// geometry). Pads take `pad_nets[i]` (or `None` past the slice),
    /// same as `crate::persistence::from_json`. `reference` is kept as given.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn insert_footprint_unchecked(
        &mut self,
        template: &FootprintTemplate,
        reference: String,
        position: Point,
        rotation_deg: f64,
        pad_nets: &[Option<NetId>],
    ) -> FootprintId {
        let mut pad_nets = pad_nets.iter().chain(std::iter::repeat(&None));
        let mut pad_item_ids = Vec::new();
        let mut hole_item_ids = Vec::new();
        for item in world_items(template, position, rotation_deg) {
            match item {
                Item::Pad { shape, layer, .. } => {
                    let &net = pad_nets.next().unwrap_or(&None);
                    pad_item_ids.push(self.node.add(Item::Pad { shape, layer, net }));
                }
                Item::Hole { .. } => {
                    hole_item_ids.push(self.node.add(item));
                }
                other => {
                    // world_items only ever produces Item::Pad/Item::Hole.
                    pad_item_ids.push(self.node.add(other));
                }
            }
        }

        self.next_footprint_serial += 1;
        let id = FootprintId(self.next_footprint_serial);
        self.footprints.push(PlacedFootprint {
            id,
            reference,
            template_name: template.name.clone(),
            position,
            rotation_deg,
            pad_item_ids,
            hole_item_ids,
            courtyard: world_courtyard(template, position, rotation_deg),
            assembly_drills: world_assembly_drills(template, position, rotation_deg),
            pin1_marker: None,
        });
        id
    }

    /// Every routed wire (a whole [`Self::connected_wire`], not just the
    /// one leg that happens to touch) currently touching any pad in
    /// `pad_item_ids` at that pad's *current* position -- what
    /// [`Self::try_move_footprint`]/[`Self::remove_footprint`] must
    /// delete before actually relocating/deleting those pads.
    /// `Item::Track`/`Item::Via` store a fixed [`Point`], never a live
    /// reference to whichever pad they happen to touch (see
    /// `alladin_core::Item`'s own doc comment), so without this a moved
    /// or deleted pad would strand same-net copper at its *old* spot --
    /// still tagged with a real net, still drawn, but no longer
    /// touching anything -- exactly the "nonsensical net connection" a
    /// move/delete must never leave behind.
    fn wires_touching_pads(&self, pad_item_ids: &[ItemId]) -> Vec<ItemId> {
        let mut entry_points = Vec::new();
        for &pad_id in pad_item_ids {
            let Some(Item::Pad { shape, layer, net }) = self.node.get(pad_id) else { continue };
            entry_points.push((shape.center(), *layer, *net));
        }

        let mut wires = std::collections::HashSet::new();
        for (item_id, item) in self.node.iter_with_ids() {
            let touches_a_pad = entry_points.iter().any(|&(center, layer, net)| match item {
                Item::Track { shape, layer: track_layer, .. } => {
                    item.net() == net && *track_layer == layer && (shape.a == center || shape.b == center)
                }
                Item::Via { shape, .. } => item.net() == net && shape.center == center,
                _ => false,
            });
            if touches_a_pad {
                wires.extend(self.connected_wire(item_id));
            }
        }
        wires.into_iter().collect()
    }

    /// Moves (and/or rotates) an already-placed footprint, hard-gated by
    /// [`Self::check_placement`] exactly like placement -- on `Err`,
    /// `self.footprints`/`self.node` are left completely unchanged, so
    /// the caller's "reject the drag" is simply "do nothing", not a
    /// separate rollback step. Reuses each pad's existing `ItemId` via
    /// `Node::replace` rather than remove-then-add, so nothing else ever
    /// has to learn about a new id for the same physical pad -- and
    /// carries each pad's existing net across that replace too (see the
    /// loop below): [`world_items`] always builds a fresh `Item::Pad`
    /// with `net: None` (nets are assigned after placement, not part of
    /// the static template), so without this the net a `connect_pads`/
    /// import/load call already assigned would silently vanish on every
    /// single drag.
    ///
    /// Also deletes every wire [`Self::wires_touching_pads`] finds
    /// touching one of this footprint's pads *before* moving them (see
    /// that method's own doc comment for why) -- the net itself is
    /// untouched (pad-net membership is carried across the move right
    /// below), only the now-geometrically-invalid copper is removed, so
    /// the user re-routes with the interactive router rather than the
    /// board silently accumulating stranded, same-colored copper stubs
    /// every time a routed part gets nudged.
    ///
    /// # Panics
    /// If `id` doesn't name a currently-placed footprint -- always a
    /// caller bug (the UI only ever calls this with an id it just read
    /// back from `self.footprints`).
    pub fn try_move_footprint(
        &mut self,
        id: FootprintId,
        template: &FootprintTemplate,
        new_position: Point,
        new_rotation_deg: f64,
    ) -> Result<(), PlacementError> {
        self.check_placement(template, new_position, new_rotation_deg, Some(id))?;
        self.commit_move_footprint(id, template, new_position, new_rotation_deg);
        Ok(())
    }

    /// Commits a footprint relocate without re-running
    /// [`Self::check_placement`] -- call only after a successful gate.
    /// Same pad-net / wire-delete / courtyard update behaviour as the
    /// commit half of [`Self::try_move_footprint`].
    ///
    /// # Panics
    /// If `id` doesn't name a currently-placed footprint.
    pub(crate) fn commit_move_footprint(&mut self, id: FootprintId, template: &FootprintTemplate, new_position: Point, new_rotation_deg: f64) {
        let index = self.footprints.iter().position(|f| f.id == id).expect("commit_move_footprint: unknown FootprintId");
        // `world_items` always emits pads then holes (see its own doc
        // comment), so chaining `pad_item_ids` then `hole_item_ids`
        // lines the two sequences back up 1:1, same order both sides.
        let item_ids: Vec<ItemId> =
            self.footprints[index].pad_item_ids.iter().chain(&self.footprints[index].hole_item_ids).copied().collect();
        for wire_id in self.wires_touching_pads(&self.footprints[index].pad_item_ids.clone()) {
            self.remove_item(wire_id);
        }
        for (item_id, mut item) in item_ids.into_iter().zip(world_items(template, new_position, new_rotation_deg)) {
            if let Item::Pad { net, .. } = &mut item {
                *net = self.node.get(item_id).and_then(Item::net);
            }
            self.node.replace(item_id, item);
        }
        self.footprints[index].position = new_position;
        self.footprints[index].rotation_deg = new_rotation_deg;
        self.footprints[index].courtyard = world_courtyard(template, new_position, new_rotation_deg);
        self.footprints[index].assembly_drills = world_assembly_drills(template, new_position, new_rotation_deg);
    }

    /// Recomputes every placed footprint's [`PlacedFootprint::courtyard`]
    /// and [`PlacedFootprint::assembly_drills`] from `templates` (silk ∪
    /// pad-bbox floor, PTH drills). Call after loading a board JSON so
    /// older files pick up current assembly geometry without a re-place.
    /// Footprints whose `template_name` no longer resolves are left as
    /// stored (same tolerance as [`Self::find_pad`]).
    pub fn sync_courtyards(&mut self, templates: &[FootprintTemplate]) {
        for fp in &mut self.footprints {
            let Some(template) = templates.iter().find(|t| t.name == fp.template_name) else { continue };
            fp.courtyard = world_courtyard(template, fp.position, fp.rotation_deg);
            fp.assembly_drills = world_assembly_drills(template, fp.position, fp.rotation_deg);
        }
    }

    /// Whether placing `template` at `position`/`rotation_deg` would
    /// put an SMD pad within [`JlcpcbDfm::COMPONENT_LEAD_TO_HOLE`] of
    /// another footprint's drill, or one of this template's drills
    /// within that clearance of another footprint's SMD pad.
    fn lead_to_hole_violated(
        &self,
        template: &FootprintTemplate,
        position: Point,
        rotation_deg: f64,
        excluding: &[FootprintId],
    ) -> bool {
        let clearance = JlcpcbDfm::COMPONENT_LEAD_TO_HOLE;
        let candidate_drills = world_assembly_drills(template, position, rotation_deg);
        // world_items emits pads then holes; zip 1:1 with template.pads.
        let items = world_items(template, position, rotation_deg);
        let candidate_smd_shapes: Vec<&PadShape> = template
            .pads
            .iter()
            .zip(items.iter())
            .filter_map(|(pad, item)| match (pad.hole_diameter, item) {
                (None, Item::Pad { shape, .. }) => Some(shape),
                _ => None,
            })
            .collect();

        for fp in self.footprints.iter().filter(|fp| !excluding.contains(&fp.id)) {
            for drill in &fp.assembly_drills {
                for shape in &candidate_smd_shapes {
                    if pad_shape_hits_circle(shape, drill, clearance) {
                        return true;
                    }
                }
            }
            // Reverse: candidate drills vs foreign SMD pads (skip pads
            // that sit on that footprint's own PTH drill centers).
            for drill in &candidate_drills {
                for &pad_id in &fp.pad_item_ids {
                    let Some(Item::Pad { shape, .. }) = self.node.get(pad_id) else { continue };
                    let is_pth = fp.assembly_drills.iter().any(|d| d.center.distance(shape.center()) < 1.0);
                    if is_pth {
                        continue;
                    }
                    if pad_shape_hits_circle(shape, drill, clearance) {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Removes a placed footprint and every one of its pads from the
    /// `Node`. A no-op (not a panic) if `id` is already gone -- the UI's
    /// "delete" button can't race against anything else in this
    /// single-threaded editor, but staying tolerant here costs nothing
    /// and avoids a needless footgun if that ever changes. Any net left
    /// with zero remaining pads by this removal is pruned too (see
    /// [`Self::prune_empty_nets`]).
    ///
    /// Also deletes every wire [`Self::wires_touching_pads`] finds
    /// touching one of this footprint's pads, for the same reason
    /// [`Self::try_move_footprint`] does -- without this, deleting a
    /// routed part would leave its tracks/vias behind as copper on a
    /// net whose only pad(s) just vanished.
    pub fn remove_footprint(&mut self, id: FootprintId) {
        if let Some(index) = self.footprints.iter().position(|f| f.id == id) {
            for wire_id in self.wires_touching_pads(&self.footprints[index].pad_item_ids.clone()) {
                self.remove_item(wire_id);
            }
            let footprint = self.footprints.remove(index);
            for item_id in footprint.pad_item_ids.into_iter().chain(footprint.hole_item_ids) {
                self.node.remove(item_id);
            }
            self.prune_empty_nets();
        }
    }

    /// The `ItemId` of the pad whose boundary contains `point`, if any --
    /// the hit-test both [`Self::footprint_at`] and pin-to-net assignment
    /// build on. Deliberately scans every live pad in `self.node` rather
    /// than only ones already tracked by `self.footprints`, so it stays
    /// correct even for a pad that ends up on the board some other way
    /// later (e.g. a manually-added test pad, or a future parts-database
    /// footprint) -- there's nothing footprint-specific about "which pad
    /// is under the cursor".
    pub fn pad_at(&self, point: Point) -> Option<ItemId> {
        self.node.iter_with_ids().find_map(|(id, item)| match item {
            Item::Pad { shape: PadShape::Circle(c), .. } if c.center.distance(point) <= c.radius as f64 => Some(id),
            // A non-round pad's true, possibly rotated outline -- an
            // exact point-in-polygon test, not the bounding circle this
            // used to fall back to, so e.g. clicking just past a
            // rotated rectangular pad's short side (inside its old,
            // larger bounding circle but outside its real copper) no
            // longer selects it.
            Item::Pad { shape: PadShape::Polygon { outline, .. }, .. } if outline.contains_point(point) => Some(id),
            _ => None,
        })
    }

    /// The `Item::Hole` at `point`, if any -- [`Self::pad_at`]'s
    /// counterpart for a mounting hole, needed because a pure-hole
    /// footprint (see [`PlacedFootprint::hole_item_ids`]) has *no* pads
    /// at all for [`Self::pad_at`] to ever find, which otherwise made
    /// [`Self::footprint_at`] silently unable to select one no matter
    /// where it was clicked.
    fn hole_at(&self, point: Point) -> Option<ItemId> {
        self.node.iter_with_ids().find_map(|(id, item)| match item {
            Item::Hole { position, drill } if position.distance(point) <= *drill as f64 / 2.0 => Some(id),
            _ => None,
        })
    }

    /// The footprint that owns the pad *or mounting hole* at `point`, if
    /// any -- the hit-test behind click-to-select and drag-to-move in
    /// the editor. Checks pads first (the overwhelmingly common case,
    /// and cheaper: [`Self::pad_at`] is a single early-return scan) and
    /// only falls back to [`Self::hole_at`] if that misses, so a normal
    /// component footprint's hit-test cost is unchanged -- see that
    /// method's own doc comment for why a pad-only check used to make a
    /// pure-mounting-hole footprint (`mounting_hole_template`, no pads
    /// at all) permanently unclickable regardless of tool.
    pub fn footprint_at(&self, point: Point) -> Option<FootprintId> {
        if let Some(pad_id) = self.pad_at(point) {
            return self.footprints.iter().find(|f| f.pad_item_ids.contains(&pad_id)).map(|f| f.id);
        }
        let hole_id = self.hole_at(point)?;
        self.footprints.iter().find(|f| f.hole_item_ids.contains(&hole_id)).map(|f| f.id)
    }

    /// Finds the `ItemId` of pin `pad_number` on the footprint whose
    /// reference designator is `reference` (exact, case-sensitive match
    /// -- real KiCad references are, and `crate::cli` reads both
    /// straight from the AI/script's own command-line arguments) --
    /// what a human or script identifies a pin by ("U1 pin 3"), unlike
    /// [`Self::pad_at`]'s screen-coordinate hit-test. Needs `templates`
    /// to resolve which of a footprint's `pad_item_ids` entries
    /// corresponds to which pad number, for the same reason
    /// [`Self::check_placement`] needs a `&FootprintTemplate` --
    /// `BoardDoc` itself deliberately holds no template registry (see
    /// [`PlacedFootprint`]'s doc comment).
    pub fn find_pad(&self, templates: &[FootprintTemplate], reference: &str, pad_number: &str) -> Option<ItemId> {
        let footprint = self.footprints.iter().find(|f| f.reference == reference)?;
        let template = templates.iter().find(|t| t.name == footprint.template_name)?;
        let index = template.pads.iter().position(|p| p.number == pad_number)?;
        footprint.pad_item_ids.get(index).copied()
    }

    /// `pub(crate)`, not `pub`: outside callers (the UI) only ever need
    /// to know a pad's net to *decide something* (can this pin be
    /// routed? can these two be connected?), which the higher-level
    /// [`Self::connect_pads`]/`crate::routing::RoutingDrag` methods
    /// already answer -- exposing the raw net lookup itself outside the
    /// crate would just invite bypassing those.
    pub(crate) fn pad_net(&self, id: ItemId) -> Result<Option<NetId>, NetError> {
        match self.node.get(id) {
            Some(Item::Pad { net, .. }) => Ok(*net),
            _ => Err(NetError::NotAPad),
        }
    }

    /// A pad's world-space center, layer, and current net in one call --
    /// exactly what starting or live-updating an interactive routing drag
    /// needs (see `crate::routing::RoutingDrag`). `None` if `id` isn't a
    /// live pad.
    pub(crate) fn pad_endpoint(&self, id: ItemId) -> Option<(Point, LayerId, Option<NetId>)> {
        match self.node.get(id) {
            Some(Item::Pad { shape, layer, net }) => Some((shape.center(), *layer, *net)),
            _ => None,
        }
    }

    /// A pad's world-space center alone -- used to snap a routing
    /// preview's last point exactly onto a hovered same-net target pad
    /// rather than stopping a pad-radius short of it.
    pub(crate) fn pad_center(&self, id: ItemId) -> Option<Point> {
        match self.node.get(id) {
            Some(Item::Pad { shape, .. }) => Some(shape.center()),
            _ => None,
        }
    }

    /// Commits a routed polyline as one `Item::Track` per leg -- the only
    /// place `alladin-pcb` adds tracks to `self.node`. This function
    /// itself does no validation, trusting the caller already has a
    /// legal path (see `crate::routing`).
    pub(crate) fn add_track_path(&mut self, path: &[Point], net: NetId, layer: LayerId, width: Unit, class: NetClass) {
        for leg in path.windows(2) {
            self.node.add(Item::Track { shape: Segment::new(leg[0], leg[1], width), net: Some(net), layer, class });
        }
    }

    /// Places a through-hole via (always FCu<->BCu, see
    /// [`alladin_core::Item::Via`]'s own doc comment) centered at
    /// `center`, hard-gated exactly like [`Self::check_placement`]'s pad
    /// branch: the via's own scalar DFM (diameter / drill / annular
    /// ring -- see [`JlcpcbDfm::check_via`]) must clear JLCPCB's floors,
    /// its full outer copper diameter must clear the board edge by
    /// JLCPCB's real `copper_to_routed_edge` minimum, and it must not
    /// collide with any existing item under the active
    /// [`RuleResolver`](alladin_core::RuleResolver). Touches nothing on
    /// `Err`, same "trial first, commit only on success" contract as
    /// every other placement primitive in this module.
    pub fn try_add_via(&mut self, center: Point, net: NetId, diameter: Unit, drill: Unit) -> Result<ItemId, PlacementError> {
        if let Err(v) = JlcpcbDfm::check_via(diameter, drill) {
            return Err(PlacementError::Dfm(v));
        }
        let radius = diameter / 2;
        if !circle_within_outline(center, radius + JlcpcbDfm::COPPER_TO_ROUTED_EDGE, &self.outline) {
            return Err(PlacementError::OffBoard);
        }
        if self.violates_hole_to_hole(center, drill, Some(net)) {
            return Err(PlacementError::Dfm(DfmViolation::HoleToHoleBelowMin));
        }

        let resolver = self.resolver();
        let candidate = Item::Via { shape: Circle::new(center, radius), drill, net: Some(net) };
        let colliding = self.node.query_colliding(&candidate, resolver).len();
        if colliding > 0 {
            return Err(PlacementError::Collision(colliding));
        }

        Ok(self.node.add(candidate))
    }

    /// Whether a new drilled hole (a via barrel or an NPTH mechanical
    /// hole) at `center` with diameter `drill` would sit closer,
    /// wall-to-wall, to any existing drilled hole on the board than
    /// [`JlcpcbDfm::required_hole_to_hole`] allows for the two holes'
    /// net pair -- the drill-bit spacing rule the copper-shape
    /// [`RuleResolver`](alladin_core::RuleResolver) physically cannot
    /// express (two vias' *copper* rings can satisfy 0.15mm while their
    /// *holes* still sit under the 0.5mm drill rule). Only sees holes
    /// that live in the [`Node`](alladin_core::Node) (vias, NPTH) --
    /// through-hole *pad* drills exist only in templates and are
    /// covered by the board-wide DFM report instead.
    pub(crate) fn violates_hole_to_hole(&self, center: Point, drill: Unit, net: Option<NetId>) -> bool {
        self.node.iter().any(|item| {
            let (other_center, other_drill, other_net) = match item {
                Item::Via { shape, drill, net } => (shape.center, *drill, *net),
                Item::Hole { position, drill } => (*position, *drill, None),
                _ => return false,
            };
            let required = JlcpcbDfm::required_hole_to_hole(net.map(|n| n.0), other_net.map(|n| n.0));
            let dx = (center.x - other_center.x) as f64;
            let dy = (center.y - other_center.y) as f64;
            dx.hypot(dy) < ((drill + other_drill) / 2 + required) as f64
        })
    }

    /// Whether an `Item::Track`/`Item::Via` with exactly this geometry,
    /// net, and layer is already live on the board. Tracks match in
    /// either endpoint order (`a->b` == `b->a`: same capsule).
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) fn has_identical_routed_item(&self, candidate: &Item) -> bool {
        self.node.iter().any(|existing| match (existing, candidate) {
            (Item::Track { shape: e, net: en, layer: el, .. }, Item::Track { shape: c, net: cn, layer: cl, .. }) => {
                en == cn && el == cl && e.width == c.width && ((e.a == c.a && e.b == c.b) || (e.a == c.b && e.b == c.a))
            }
            (Item::Via { shape: e, drill: ed, net: en }, Item::Via { shape: c, drill: cd, net: cn }) => {
                en == cn && ed == cd && e.center == c.center && e.radius == c.radius
            }
            _ => false,
        })
    }

    /// Read-only equivalent of [`Self::try_add_via`]'s own two gates
    /// (board edge, collision) -- never mutates `self.node`. Purely for
    /// a *live* per-frame ghost preview (the GUI's pin-via drag-fallback,
    /// see `app::EditorState::update_pending_pin_via`) to colour itself
    /// red/green without actually placing (and immediately having to
    /// roll back) a real via on every single frame just to ask "would
    /// this work right now".
    pub(crate) fn via_would_fit(&self, center: Point, net: NetId, diameter: Unit) -> bool {
        let radius = diameter / 2;
        if !circle_within_outline(center, radius + JlcpcbDfm::COPPER_TO_ROUTED_EDGE, &self.outline) {
            return false;
        }
        let resolver = self.resolver();
        // `drill` genuinely doesn't matter here -- collision only ever
        // looks at `shape` (the outer copper circle), see `Item::Via::drill`'s
        // own doc comment -- so `0` is a safe placeholder, never a real via.
        let candidate = Item::Via { shape: Circle::new(center, radius), drill: 0, net: Some(net) };
        !self.node.is_colliding(&candidate, resolver)
    }

    /// The natural candidate via center [`Self::try_add_pin_stitching_via`]
    /// itself would try *first* for `pad_id` -- i.e. just
    /// [`Self::pin_stitching_via_candidates`]' first element -- exposed
    /// separately so a caller that needs to *show* that one ideal point
    /// before actually attempting anything (the GUI's drag-fallback
    /// ghost, started right after every automatic attempt, including
    /// the angular sweep, was refused) doesn't need its own,
    /// possibly-drifting copy of this radial-direction math. `None`
    /// only if `pad_id` isn't a live pad.
    pub(crate) fn pin_stitching_via_candidate(&self, pad_id: ItemId, diameter: Unit) -> Option<Point> {
        self.pin_stitching_via_candidates(pad_id, diameter).into_iter().next()
    }

    /// Every via center [`Self::try_add_pin_stitching_via`] is willing
    /// to try for `pad_id`, in trial order: first the one "natural"
    /// point -- just outside the pad's own copper, along the radial
    /// line from the owning footprint's own body/courtyard center
    /// through the pin (the "just outside this pin, pointing away from
    /// the part" spot a human routing engineer reaches for by hand; +X
    /// fallback if that radial direction has zero length, see
    /// [`Self::try_add_pin_stitching_via`]'s own doc comment for the
    /// full rationale) -- then, if that exact spot turns out to be
    /// occupied, increasingly wide deviations off to alternating sides
    /// of it (+15°, -15°, +30°, -30°, ... up to +/-90°), always on the
    /// *same* radius circle centered on the pad itself.
    ///
    /// The radius is deliberately never allowed to grow: a stitching
    /// via always sits the same distance from its pin regardless of
    /// which candidate in this list ends up used, so a board's vias
    /// stay visually and electrically consistent however crowded a
    /// given pin's neighbourhood is. Only the *angle* varies, and only
    /// within +/-90° of the natural direction -- wide enough to step
    /// around a single obstacle sitting right on the natural line, but
    /// capped well short of a full circle so a via can never end up on
    /// the *far* side of the footprint, behind the part, where a
    /// stitching via has no business being even if that spot happens
    /// to be geometrically free.
    ///
    /// Empty only if `pad_id` isn't a live pad.
    pub(crate) fn pin_stitching_via_candidates(&self, pad_id: ItemId, diameter: Unit) -> Vec<Point> {
        const STEP_DEG: f64 = 15.0;
        const MAX_DEG: f64 = 90.0;

        let Some(Item::Pad { shape, .. }) = self.node.get(pad_id) else { return Vec::new() };
        let pad_center = shape.center();
        let pad_radius = shape.bounding_radius();

        let body_center = self.footprints.iter().find(|f| f.pad_item_ids.contains(&pad_id)).map(|f| {
            let b = Aabb::from_polygon(&f.courtyard);
            Point::new((b.min.x + b.max.x) / 2, (b.min.y + b.max.y) / 2)
        });
        let outward = body_center.map(|c| pad_center.sub(c)).filter(|v| v.x != 0 || v.y != 0).unwrap_or(Point::new(1, 0));
        let outward_len = outward.length();
        let (ux, uy) = (outward.x as f64 / outward_len, outward.y as f64 / outward_len);

        let gap = pad_radius + self.pad_to_pad_clearance() + diameter / 2 + 1_000;

        let mut deviations_deg = vec![0.0];
        let mut deg = STEP_DEG;
        while deg <= MAX_DEG + f64::EPSILON {
            deviations_deg.push(deg);
            deviations_deg.push(-deg);
            deg += STEP_DEG;
        }

        deviations_deg
            .into_iter()
            .map(|deg| {
                let (sin, cos) = deg.to_radians().sin_cos();
                let (rux, ruy) = (ux * cos - uy * sin, ux * sin + uy * cos);
                Point::new(pad_center.x + (rux * gap as f64).round() as Unit, pad_center.y + (ruy * gap as f64).round() as Unit)
            })
            .collect()
    }

    /// Like [`Self::try_add_via`], but additionally refuses -- rolling
    /// the via straight back out again on refusal, same "trial first,
    /// commit only on success" contract -- if the via wouldn't actually
    /// *touch* any existing copper on its own net (see
    /// [`alladin_core::Node::touches_same_net`]'s doc comment for why
    /// the plain collision check alone can never catch this: same-net
    /// pairs are always exempt there). Deliberately a separate entry
    /// point rather than baked into `try_add_via` itself:
    /// [`crate::routing::RoutingDrag::drop_via_and_switch_layer`]'s
    /// mid-route via is placed *before* the track leg that will
    /// actually connect it is committed, so it would always -- wrongly
    /// -- fail this extra check there; that caller keeps using the
    /// plain, unchecked `try_add_via`. This one is for the two
    /// "standalone" via callers instead, where nothing else is about to
    /// connect it a moment later: the GUI's "Place vias" tool
    /// (`app::EditorState::handle_place_via_click`) and the CLI's
    /// `add-via` command.
    pub fn try_add_stitching_via(&mut self, center: Point, net: NetId, diameter: Unit, drill: Unit) -> Result<ItemId, ViaError> {
        let id = self.try_add_via(center, net, diameter, drill).map_err(ViaError::Placement)?;
        // `exclude: Some(id)` is required, not optional here -- the via
        // is already live in `self.node` at this point (added by the
        // `try_add_via` call above), so without excluding its own id the
        // query would trivially find that just-inserted copy as a "same
        // net, perfectly overlapping" match every time, making this
        // whole check a no-op. See `Node::touches_same_net`'s doc
        // comment.
        let placed = self.node.get(id).expect("just-added via must still be live");
        if !self.node.touches_same_net(placed, Some(id)) {
            self.node.remove(id);
            return Err(ViaError::Dangling);
        }
        Ok(id)
    }

    /// Places a stitching via a hair outside `pad_id`'s own copper,
    /// along the radial line from the owning footprint's own
    /// body/courtyard center through the pin -- the "just outside this
    /// pin, pointing away from the part" spot a human routing engineer
    /// reaches for by hand -- plus the short straight `Item::Track`
    /// stub that actually connects the two; a bare via touching
    /// nothing would just be [`ViaError::Dangling`] with extra steps.
    /// This is what the GUI's right-click "Add via near pin" menu
    /// (`app::EditorState::add_pin_stitching_via_at`) and the
    /// `add_pin_stitching_via` MCP tool both call.
    ///
    /// The exit direction is deliberately *radial from the footprint's
    /// own courtyard center*, not derived from the pad's rotation or
    /// which side of the package it's on: it's the one definition
    /// that's correct for every footprint shape this crate knows about
    /// without a per-template "which way does this pin face" table --
    /// a corner pin on a QFN, a two-pin 0402, and a WROOM module's
    /// castellated edge pad all naturally point "away from the part"
    /// under this rule, exactly where a stitching via belongs. The one
    /// degenerate case -- a pin sitting exactly on the footprint's own
    /// center, so the radial direction has zero length -- falls back
    /// to a fixed `+X` direction rather than failing outright; a via
    /// placed slightly off in an arbitrary direction is still useful,
    /// an outright refusal for a case this rare wouldn't be. A pad
    /// with no owning footprint at all (shouldn't happen for any real
    /// pad, every one comes from `Self::try_place_footprint`) falls
    /// back the same way.
    ///
    /// Trial-first, same contract as every other placement primitive
    /// here: on any `Err`, neither the via nor the stub track exist --
    /// no partial state for the caller to clean up or roll back by
    /// hand.
    ///
    /// Tries every point [`Self::pin_stitching_via_candidates`] offers,
    /// in order, and commits at the first one where *both* the via
    /// itself and its stub track come back clean -- so a single
    /// obstacle sitting exactly on the natural line (another
    /// already-placed via, a neighbouring footprint's copper, ...)
    /// no longer has to mean an outright refusal, the way it used to
    /// when this method only ever tried that one point. If every
    /// candidate fails, the error reported back is always the one from
    /// that first, "natural" point specifically -- not whichever
    /// candidate happened to be tried last -- since that is the one
    /// failure reason a caller showing a message to a human actually
    /// wants to hear ("the pin's immediate neighbourhood is too
    /// tight", not "attempt number eleven, at 90 degrees, failed for
    /// this unrelated reason").
    pub fn try_add_pin_stitching_via(&mut self, pad_id: ItemId, diameter: Unit, drill: Unit, stub_width: Unit) -> Result<PinStitchingVia, PinStitchingViaError> {
        // The via's own diameter/drill/annular gates live in the
        // `try_add_via` call below; the stub track's width has no such
        // downstream gate (`add_track_path` trusts its caller), so it's
        // checked here before anything is placed.
        if let Err(v) = JlcpcbDfm::check_track_width(stub_width) {
            return Err(PinStitchingViaError::Via(PlacementError::Dfm(v)));
        }
        let Some(Item::Pad { shape, net, layer }) = self.node.get(pad_id).cloned() else {
            return Err(PinStitchingViaError::NotAPad);
        };
        let Some(net) = net else { return Err(PinStitchingViaError::NoNet) };
        let pad_center = shape.center();

        let candidates = self.pin_stitching_via_candidates(pad_id, diameter);
        let resolver = self.resolver();
        let mut first_err = None;
        for via_center in candidates {
            let via_id = match self.try_add_via(via_center, net, diameter, drill) {
                Ok(id) => id,
                Err(e) => {
                    first_err.get_or_insert(PinStitchingViaError::Via(e));
                    continue;
                }
            };

            let stub_clear = self.node.path_is_clear(pad_center, via_center, stub_width, Some(net), layer, NetClass::C, resolver)
                && path_keeps_edge_clearance(&[pad_center, via_center], stub_width, &self.outline);
            if !stub_clear {
                self.node.remove(via_id);
                first_err.get_or_insert(PinStitchingViaError::NoRoomForStub);
                continue;
            }

            self.add_track_path(&[pad_center, via_center], net, layer, stub_width, NetClass::C);
            return Ok(PinStitchingVia { via_id, center: via_center });
        }
        Err(first_err.expect("pin_stitching_via_candidates never returns empty for a pad_id already confirmed live above"))
    }

    /// Whether `candidate` (already carrying whatever position/
    /// rotation/height is being tried) would be a legal silk placement
    /// -- shared by [`Self::check_silk_text_placement`] (read-only,
    /// for a GUI ghost preview) and [`Self::try_place_silk_text`]/
    /// [`Self::try_move_silk_text`] (the actual mutating commits).
    /// Pad, body, and text-to-text collision all run against the
    /// string's *real printed ink* -- the embedded Hershey stroke
    /// capsules of [`SilkText::stroke_segments`], the same geometry
    /// the GUI draws and native Gerber strokes -- so "there's
    /// visibly nothing overlapping there" and "this placement is
    /// accepted" agree, exactly. Only the board-*edge* check stays on
    /// the single [`SilkText::bounding_rect`] hull (itself now the
    /// tight ink box), where the difference is nil. `ignore` excludes
    /// one already-placed [`SilkTextId`] from the same-side overlap
    /// check -- needed so moving/rotating a text doesn't spuriously
    /// "collide" with its own, not-yet-updated old position (see
    /// [`Self::try_move_silk_text`]).
    fn silk_text_fits(&self, candidate: &SilkText, ignore: Option<SilkTextId>) -> Result<(), SilkTextError> {
        let layer = candidate.layer;
        // Same margin `Self::check_placement` already uses for a
        // pad's own copper-to-edge clearance -- silk ink flush against
        // the cut line is exactly as unmanufacturable as copper would
        // be there, and there is no separate, dedicated "silk to
        // routed edge" JLCPCB constant to reach for instead.
        if !polygon_within_outline_with_clearance(&candidate.bounding_rect(), JlcpcbDfm::COPPER_TO_ROUTED_EDGE, &self.outline) {
            return Err(SilkTextError::OffBoard);
        }

        let segments = candidate.stroke_segments();
        for item in self.node.iter() {
            let Item::Pad { shape, layer: pad_layer, .. } = item else { continue };
            // Only a same-*side* pad's copper can ever actually be
            // printed over -- a back-side pad sits on the opposite
            // face of the board from front-side silk, physically
            // nowhere near the ink.
            if *pad_layer != layer {
                continue;
            }
            if segments.iter().any(|seg| stroke_hits_pad(seg, shape, JlcpcbDfm::SILK_TO_PAD)) {
                return Err(SilkTextError::TooCloseToPad);
            }
        }

        // A label printed underneath a part's body can never be read
        // (see `SilkTextError::UnderComponentBody`). Bodies only exist
        // on the front side (this editor places every component for
        // JLCPCB's front-side assembly), so back-side silk is never
        // affected by them.
        if layer == LayerId::FCu {
            let under_a_body = self
                .footprints
                .iter()
                .any(|fp| segments.iter().any(|seg| segment_polygon_collides(seg, &fp.courtyard, 0)));
            if under_a_body {
                return Err(SilkTextError::UnderComponentBody);
            }
        }

        // Two strokes overlap once their centerlines come closer than
        // the two half-widths combined -- real ink touching real ink,
        // zero extra tolerance (unreadable regardless of any numeric
        // rule, see `SilkTextError::OverlapsAnotherText`).
        let overlaps_another_text = self.silk_texts.iter().filter(|t| Some(t.id) != ignore && t.layer == layer).any(|t| {
            let other = t.stroke_segments();
            segments.iter().any(|seg| {
                other
                    .iter()
                    .any(|o| dist_segment_to_segment((seg.a, seg.b), (o.a, o.b)) < ((seg.width + o.width) / 2) as f64)
            })
        });
        if overlaps_another_text {
            return Err(SilkTextError::OverlapsAnotherText);
        }

        // Round silk (free dots + pin-1 markers) is ink too -- same
        // zero-tolerance touch rule as text-on-text above.
        let overlaps_a_dot = self.silk_dot_circles_on(layer, None, &[]).iter().any(|c| {
            segments
                .iter()
                .any(|seg| dist_segment_to_segment((seg.a, seg.b), (c.center, c.center)) < (seg.width / 2 + c.radius) as f64)
        });
        if overlaps_a_dot {
            return Err(SilkTextError::OverlapsDot);
        }

        Ok(())
    }

    /// Read-only counterpart of [`Self::try_place_silk_text`] -- same
    /// "check first, so a GUI ghost can show red/green before any real
    /// commit" split every other placement primitive here already has
    /// (see e.g. [`Self::check_placement`]). `height` matters: a bigger
    /// character height means a bigger [`SilkText::bounding_rect`], so
    /// a placement one size would accept can still be legitimately
    /// refused at a larger one (e.g. now too close to a pad it used to
    /// clear) -- pass [`DEFAULT_SILK_TEXT_HEIGHT`] for the common case,
    /// or the GUI's own size-stepper value for a live preview that
    /// tracks it exactly.
    pub fn check_silk_text_placement(&self, text: &str, position: Point, rotation_deg: f64, layer: LayerId, height: Unit) -> Result<(), SilkTextError> {
        if text.trim().is_empty() {
            return Err(SilkTextError::EmptyText);
        }
        let candidate = SilkText { id: SilkTextId(0), text: text.to_string(), position, rotation_deg, layer, height, line_width: DEFAULT_SILK_LINE_WIDTH };
        self.silk_text_fits(&candidate, None)
    }

    /// Places a new free-standing [`SilkText`] -- the GUI's "Place silk
    /// text" tool (driven by its own size stepper, see
    /// `EditorState::silk_text_height`) and the `add_silk_text` MCP
    /// tool's entry point (`height_mm`, defaulting to
    /// [`DEFAULT_SILK_TEXT_HEIGHT`]). Trial-first, same contract as
    /// every other placement primitive here: on any `Err`, nothing is
    /// added.
    pub fn try_place_silk_text(&mut self, text: &str, position: Point, rotation_deg: f64, layer: LayerId, height: Unit) -> Result<SilkTextId, SilkTextError> {
        if text.trim().is_empty() {
            return Err(SilkTextError::EmptyText);
        }
        let id = SilkTextId(self.next_silk_text_serial);
        let candidate = SilkText { id, text: text.to_string(), position, rotation_deg, layer, height, line_width: DEFAULT_SILK_LINE_WIDTH };
        self.silk_text_fits(&candidate, None)?;
        self.next_silk_text_serial += 1;
        self.silk_texts.push(candidate);
        Ok(id)
    }

    /// Moves/rotates an already-placed [`SilkText`] to a new
    /// position/rotation, refusing (leaving it exactly where it was)
    /// under the same rules [`Self::try_place_silk_text`] applies to a
    /// brand new one -- its own current placement is excluded from the
    /// same-side overlap check (see [`Self::silk_text_fits`]'s `ignore`
    /// parameter), or moving it even a fraction of a millimetre would
    /// always spuriously collide with its own, not-yet-updated old
    /// rectangle.
    pub fn try_move_silk_text(&mut self, id: SilkTextId, position: Point, rotation_deg: f64) -> Result<(), SilkTextError> {
        let Some(existing) = self.silk_texts.iter().find(|t| t.id == id) else { return Err(SilkTextError::NotFound) };
        let mut candidate = existing.clone();
        candidate.position = position;
        candidate.rotation_deg = rotation_deg;
        self.silk_text_fits(&candidate, Some(id))?;
        let slot = self.silk_texts.iter_mut().find(|t| t.id == id).expect("just found above");
        slot.position = position;
        slot.rotation_deg = rotation_deg;
        Ok(())
    }

    /// Read-only counterpart of [`Self::try_move_silk_text`] -- what a
    /// live drag-to-move preview (the GUI's own [`Tool::Select`]
    /// dragging a selected [`SilkText`]) checks every frame against,
    /// same "check first" split [`Self::check_silk_text_placement`]
    /// already gives new placements. Reuses `id`'s own already-stored
    /// text/layer/height (a move never changes those) and excludes it
    /// from the same-side overlap check exactly like the real commit
    /// does, so hovering back over its own current spot never shows a
    /// false "invalid" preview.
    pub fn check_silk_text_move(&self, id: SilkTextId, position: Point, rotation_deg: f64) -> Result<(), SilkTextError> {
        let Some(existing) = self.silk_texts.iter().find(|t| t.id == id) else { return Err(SilkTextError::NotFound) };
        let mut candidate = existing.clone();
        candidate.position = position;
        candidate.rotation_deg = rotation_deg;
        self.silk_text_fits(&candidate, Some(id))
    }

    /// Resizes an already-placed [`SilkText`]'s character height in
    /// place (position/rotation untouched) -- the GUI's size-stepper
    /// "bigger"/"smaller" buttons for an already-selected text, refused
    /// under the exact same rules a brand new placement at the larger
    /// size would be (a bigger character height grows
    /// [`SilkText::bounding_rect`], which can newly collide with a pad
    /// or another text that the smaller size used to clear).
    pub fn try_resize_silk_text(&mut self, id: SilkTextId, height: Unit) -> Result<(), SilkTextError> {
        let Some(existing) = self.silk_texts.iter().find(|t| t.id == id) else { return Err(SilkTextError::NotFound) };
        let mut candidate = existing.clone();
        candidate.height = height;
        self.silk_text_fits(&candidate, Some(id))?;
        let slot = self.silk_texts.iter_mut().find(|t| t.id == id).expect("just found above");
        slot.height = height;
        Ok(())
    }

    /// The placed [`SilkText`] whose [`SilkText::bounding_rect`]
    /// contains `point`, if any -- the hit-test behind click-to-select,
    /// drag-to-move, and Delete for a placed silk text in
    /// [`Tool::Select`], mirroring [`Self::footprint_at`]/[`Self::track_at`]'s
    /// own role for footprints/tracks. Iterated back-to-front (most
    /// recently placed first) purely for a deterministic pick if two
    /// texts' generous rectangles ever happen to overlap on *opposite*
    /// sides of the board (same-side overlap is already refused by
    /// [`Self::silk_text_fits`], so that's the only way this can
    /// happen at all) -- functionally arbitrary either way, since only
    /// one is ever wanted per click.
    pub fn silk_text_at(&self, point: Point) -> Option<SilkTextId> {
        self.silk_texts.iter().rev().find(|t| t.bounding_rect().contains_point(point)).map(|t| t.id)
    }

    /// Deletes a placed [`SilkText`] -- always succeeds if `id` exists
    /// (there's no other item that can ever depend on a silk text the
    /// way a footprint's pads depend on it, so there is nothing else
    /// to clean up), `false` if it doesn't.
    pub fn remove_silk_text(&mut self, id: SilkTextId) -> bool {
        let before = self.silk_texts.len();
        self.silk_texts.retain(|t| t.id != id);
        self.silk_texts.len() != before
    }

    /// Every printed round silk shape on `layer` -- free [`SilkDot`]s
    /// plus every footprint's enabled pin-1 marker -- as plain
    /// [`Circle`]s, with up to one of each excludable: the one shared
    /// enumeration behind every "does X hit a dot?" check
    /// ([`Self::silk_text_fits`], [`Self::check_placement`],
    /// [`Self::silk_circle_fits`] itself), so no rule ever sees a
    /// different set of dots than another. Markers only exist on the
    /// front side (see [`PlacedFootprint::pin1_marker`]).
    fn silk_dot_circles_on(&self, layer: LayerId, ignore_dot: Option<SilkDotId>, ignore_markers: &[FootprintId]) -> Vec<Circle> {
        let mut circles: Vec<Circle> =
            self.silk_dots.iter().filter(|d| d.layer == layer && Some(d.id) != ignore_dot).map(|d| d.circle()).collect();
        if layer == LayerId::FCu {
            circles.extend(self.footprints.iter().filter(|fp| !ignore_markers.contains(&fp.id)).filter_map(|fp| fp.pin1_marker_circle()));
        }
        circles
    }

    /// Whether a filled silk circle (a free [`SilkDot`] or a pin-1
    /// marker -- geometrically the same thing) would be a legal print
    /// at this exact spot: on the board with the usual edge margin,
    /// [`JlcpcbDfm::SILK_TO_PAD`] clear of every same-side pad, not
    /// under any component body, and not touching any other silk ink
    /// (text strokes, other dots, other markers). The one shared rule
    /// set behind [`Self::try_place_silk_dot`]/[`Self::try_move_silk_dot`]/
    /// [`Self::try_resize_silk_dot`] and [`Self::try_enable_pin1_marker`]'s
    /// sweep -- mirroring [`Self::silk_text_fits`]'s role for texts.
    fn silk_circle_fits(
        &self,
        circle: &Circle,
        layer: LayerId,
        ignore_dot: Option<SilkDotId>,
        ignore_marker_of: Option<FootprintId>,
    ) -> Result<(), SilkDotError> {
        if !circle_within_outline(circle.center, circle.radius + JlcpcbDfm::COPPER_TO_ROUTED_EDGE, &self.outline) {
            return Err(SilkDotError::OffBoard);
        }

        for item in self.node.iter() {
            let Item::Pad { shape, layer: pad_layer, .. } = item else { continue };
            if *pad_layer != layer {
                continue;
            }
            let too_close = match shape {
                PadShape::Circle(p) => circles_touch(circle, p, JlcpcbDfm::SILK_TO_PAD),
                PadShape::Polygon { outline, .. } => circle_polygon_collides(circle, outline, JlcpcbDfm::SILK_TO_PAD),
            };
            if too_close {
                return Err(SilkDotError::TooCloseToPad);
            }
        }

        if layer == LayerId::FCu && self.footprints.iter().any(|fp| circle_polygon_collides(circle, &fp.courtyard, 0)) {
            return Err(SilkDotError::UnderComponentBody);
        }

        let touches_text = self.silk_texts.iter().filter(|t| t.layer == layer).any(|t| {
            t.stroke_segments()
                .iter()
                .any(|seg| dist_segment_to_segment((seg.a, seg.b), (circle.center, circle.center)) < (seg.width / 2 + circle.radius) as f64)
        });
        if touches_text {
            return Err(SilkDotError::OverlapsSilk);
        }

        let ignore_markers = ignore_marker_of.as_ref().map(std::slice::from_ref).unwrap_or(&[]);
        if self.silk_dot_circles_on(layer, ignore_dot, ignore_markers).iter().any(|other| circles_touch(circle, other, 0)) {
            return Err(SilkDotError::OverlapsSilk);
        }

        Ok(())
    }

    /// Read-only legality check for the GUI's dot-placement ghost --
    /// same check-first split as [`Self::check_silk_text_placement`].
    pub fn check_silk_dot_placement(&self, position: Point, diameter: Unit, layer: LayerId) -> Result<(), SilkDotError> {
        let candidate = SilkDot { id: SilkDotId(0), position, diameter: diameter.max(JlcpcbDfm::MIN_SILK_LINE_WIDTH), layer };
        self.silk_circle_fits(&candidate.circle(), layer, None, None)
    }

    /// Places a new free-standing [`SilkDot`] -- trial-first, nothing
    /// added on `Err`, same contract as [`Self::try_place_silk_text`].
    pub fn try_place_silk_dot(&mut self, position: Point, diameter: Unit, layer: LayerId) -> Result<SilkDotId, SilkDotError> {
        let id = SilkDotId(self.next_silk_dot_serial);
        let candidate = SilkDot { id, position, diameter: diameter.max(JlcpcbDfm::MIN_SILK_LINE_WIDTH), layer };
        self.silk_circle_fits(&candidate.circle(), layer, None, None)?;
        self.next_silk_dot_serial += 1;
        self.silk_dots.push(candidate);
        Ok(id)
    }

    /// Read-only counterpart of [`Self::try_move_silk_dot`] -- the
    /// live drag preview's per-frame check, excluding the dot's own
    /// current spot exactly like the real commit does.
    pub fn check_silk_dot_move(&self, id: SilkDotId, position: Point) -> Result<(), SilkDotError> {
        let Some(existing) = self.silk_dots.iter().find(|d| d.id == id) else { return Err(SilkDotError::NotFound) };
        let candidate = Circle::new(position, existing.diameter / 2);
        self.silk_circle_fits(&candidate, existing.layer, Some(id), None)
    }

    /// Moves an already-placed [`SilkDot`], refusing (dot untouched)
    /// under the same rules as a fresh placement -- its own current
    /// spot excluded from the overlap check, mirroring
    /// [`Self::try_move_silk_text`].
    pub fn try_move_silk_dot(&mut self, id: SilkDotId, position: Point) -> Result<(), SilkDotError> {
        self.check_silk_dot_move(id, position)?;
        let slot = self.silk_dots.iter_mut().find(|d| d.id == id).expect("checked above");
        slot.position = position;
        Ok(())
    }

    /// Changes an already-placed [`SilkDot`]'s diameter in place --
    /// the GUI's size stepper for a selected dot, refused if the
    /// bigger dot would newly collide, mirroring
    /// [`Self::try_resize_silk_text`].
    pub fn try_resize_silk_dot(&mut self, id: SilkDotId, diameter: Unit) -> Result<(), SilkDotError> {
        let Some(existing) = self.silk_dots.iter().find(|d| d.id == id) else { return Err(SilkDotError::NotFound) };
        let clamped = diameter.max(JlcpcbDfm::MIN_SILK_LINE_WIDTH);
        let candidate = Circle::new(existing.position, clamped / 2);
        self.silk_circle_fits(&candidate, existing.layer, Some(id), None)?;
        let slot = self.silk_dots.iter_mut().find(|d| d.id == id).expect("just found above");
        slot.diameter = clamped;
        Ok(())
    }

    /// The placed [`SilkDot`] under `point`, if any -- the hit-test
    /// behind click-to-select/drag/Delete, mirroring
    /// [`Self::silk_text_at`]. The clickable radius is floored at
    /// 0.25mm so the smallest legal dots stay selectable at all.
    pub fn silk_dot_at(&self, point: Point) -> Option<SilkDotId> {
        self.silk_dots
            .iter()
            .rev()
            .find(|d| {
                let hit_radius = (d.diameter / 2).max(MM / 4);
                ((d.position.x - point.x) as f64).hypot((d.position.y - point.y) as f64) < hit_radius as f64
            })
            .map(|d| d.id)
    }

    /// Deletes a placed [`SilkDot`] -- same "nothing else can depend
    /// on it" reasoning as [`Self::remove_silk_text`].
    pub fn remove_silk_dot(&mut self, id: SilkDotId) -> bool {
        let before = self.silk_dots.len();
        self.silk_dots.retain(|d| d.id != id);
        self.silk_dots.len() != before
    }

    /// Enables `id`'s pin-1 marker dot: finds pad "1" in `template`
    /// (the same template registry lookup every other footprint
    /// operation already routes through the caller), then sweeps 12
    /// candidate directions around that pad -- starting outward, away
    /// from the part's own center, the natural "pin-1 corner" spot --
    /// at a distance that clears the pad's own reach plus
    /// [`JlcpcbDfm::SILK_TO_PAD`], committing the first spot
    /// [`Self::silk_circle_fits`] accepts (the part's *own* body
    /// included: a marker hidden under its own part would be
    /// pointless). The winning spot is stored as a footprint-local
    /// offset so it rides along with every later move/rotate (see
    /// [`PlacedFootprint::pin1_marker`]'s doc comment). Refused with
    /// [`SilkDotError::NoRoomNearPin1`] if every candidate is illegal
    /// -- the flag stays off, nothing changes.
    pub fn try_enable_pin1_marker(&mut self, id: FootprintId, template: &FootprintTemplate) -> Result<(), SilkDotError> {
        let Some(fp_index) = self.footprints.iter().position(|f| f.id == id) else { return Err(SilkDotError::NotFound) };
        let fp = &self.footprints[fp_index];
        let Some(pad1) = template.pads.iter().find(|p| p.number == "1").or_else(|| template.pads.first()) else {
            return Err(SilkDotError::NoRoomNearPin1);
        };
        let radius = PIN1_MARKER_DIAMETER / 2;
        let reach = pad_template_reach(pad1);
        // 0.05mm on top of the DFM minimum, so the committed spot
        // never sits *exactly* on the refusal threshold where a later
        // rounding wiggle could flip it illegal.
        let dist = (reach + JlcpcbDfm::SILK_TO_PAD + radius + MM / 20) as f64;
        let base_angle = if pad1.offset.x == 0 && pad1.offset.y == 0 {
            // A single-pad-at-origin part has no "outward": default to
            // up-left, the classic pin-1 corner.
            (-135.0f64).to_radians()
        } else {
            (pad1.offset.y as f64).atan2(pad1.offset.x as f64)
        };
        for step_deg in [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0, 150.0, -150.0, 180.0] {
            let angle = base_angle + (step_deg as f64).to_radians();
            let local = Point::new(pad1.offset.x + (dist * angle.cos()).round() as Unit, pad1.offset.y + (dist * angle.sin()).round() as Unit);
            let world = Circle::new(local.rotated(fp.rotation_deg).add(fp.position), radius);
            if self.silk_circle_fits(&world, LayerId::FCu, None, Some(id)).is_ok() {
                self.footprints[fp_index].pin1_marker = Some(local);
                return Ok(());
            }
        }
        Err(SilkDotError::NoRoomNearPin1)
    }

    /// Turns `id`'s pin-1 marker back off -- always succeeds if the
    /// footprint exists (`false` if it doesn't), nothing else to
    /// clean up.
    pub fn disable_pin1_marker(&mut self, id: FootprintId) -> bool {
        match self.footprints.iter_mut().find(|f| f.id == id) {
            Some(fp) => {
                fp.pin1_marker = None;
                true
            }
            None => false,
        }
    }

    /// Fills `outline` on `layer` for `net` (see `crate::zone_fill::fill_zone`'s
    /// own doc comment for the full board-clip/obstacle-buffer/union/
    /// difference pipeline), adds whatever `Item::Zone` island(s) that
    /// produces to `self.node`, and records the result as a new
    /// [`ZoneRecord`] so it can later be [`Self::refill_zone`]d. Always
    /// succeeds and records a `ZoneRecord` -- even an outline that
    /// currently fills to nothing (fully off-board, or fully consumed by
    /// obstacle clearances) is kept, with an empty `item_ids`, so a
    /// later `refill_zone` (once the board around it changes) can still
    /// find it and try again, rather than the user having to redraw it
    /// from scratch.
    pub fn add_zone(&mut self, outline: Polygon, layer: LayerId, net: NetId) -> ZoneId {
        // Read *before* computing the fill -- not that it matters here,
        // since inserting the resulting `Item::Zone`(s) never itself
        // bumps `obstacle_revision` (see that field's own doc comment),
        // but this is the actual "board state this fill was computed
        // against" moment, and reading it any later would just be
        // relying on that same fact to still hold rather than stating
        // the real intent directly.
        let filled_at_revision = self.node.obstacle_revision();
        let resolver = self.resolver();
        let items = zone_fill::fill_zone(&outline, layer, net, &self.outline, &self.node, resolver);
        self.insert_new_zone(outline, layer, net, items, filled_at_revision)
    }

    /// The insertion half of [`Self::add_zone`] -- adds the
    /// already-computed `items` (exactly what `zone_fill::fill_zone`
    /// returned for `outline`/`layer`/`net`) to `self.node` and records
    /// the result as a new [`ZoneRecord`]. Split from the fill itself so
    /// callers can compute islands on a cloned `Node`/outline/resolver
    /// and then commit only this cheap half against the live board.
    /// `filled_at_revision` is the caller's own
    /// `self.node.obstacle_revision()` read from *before* it computed
    /// `items`, exactly like [`Self::add_zone`] reads it above -- not
    /// re-read here, since the live revision may have moved on.
    pub(crate) fn insert_new_zone(&mut self, outline: Polygon, layer: LayerId, net: NetId, items: Vec<Item>, filled_at_revision: u64) -> ZoneId {
        self.next_zone_serial += 1;
        let id = ZoneId(self.next_zone_serial);
        let item_ids = items.into_iter().map(|item| self.node.add(item)).collect();
        self.zones.push(ZoneRecord { id, outline, layer, net, item_ids, filled_at_revision });
        id
    }

    /// Removes `id`'s current fill island(s) from `self.node`, if any --
    /// the first half of [`Self::refill_zone`], split for the same reason
    /// as [`Self::insert_new_zone`]: clear stale copper before computing
    /// a fresh fill so the pour never treats its own previous islands as
    /// obstacles. A no-op if `id` doesn't name a currently-recorded zone.
    pub(crate) fn clear_zone_fill(&mut self, id: ZoneId) {
        let Some(index) = self.zones.iter().position(|z| z.id == id) else { return };
        for item_id in std::mem::take(&mut self.zones[index].item_ids) {
            self.node.remove(item_id);
        }
    }

    /// The insertion half of [`Self::refill_zone`] -- see
    /// [`Self::clear_zone_fill`]/[`Self::insert_new_zone`]'s doc
    /// comments for the companion halves and why this three-way split
    /// exists. `items` must be exactly what `zone_fill::fill_zone`
    /// returned for `id`'s own recorded `outline`/`layer`/`net`. A
    /// no-op if `id` no longer names a live zone.
    pub(crate) fn insert_zone_refill(&mut self, id: ZoneId, items: Vec<Item>, filled_at_revision: u64) {
        let Some(index) = self.zones.iter().position(|z| z.id == id) else { return };
        self.zones[index].item_ids = items.into_iter().map(|item| self.node.add(item)).collect();
        self.zones[index].filled_at_revision = filled_at_revision;
    }

    /// Re-runs `id`'s fill from its own recorded `outline`/`layer`/`net`
    /// against the *current* board state -- what to call after routing a
    /// new track (or moving a part) underneath an existing pour, since
    /// [`Self::add_zone`]'s fill result is a point-in-time snapshot, not
    /// a live-updating one (see [`ZoneRecord`]'s own doc comment for
    /// why). The zone's previous fill islands are removed from
    /// `self.node` first, so a shrinking pour never leaves stale copper
    /// behind. A no-op if `id` doesn't name a currently-recorded zone.
    pub fn refill_zone(&mut self, id: ZoneId) {
        self.clear_zone_fill(id);
        let Some(index) = self.zones.iter().position(|z| z.id == id) else { return };
        let (outline, layer, net) = (self.zones[index].outline.clone(), self.zones[index].layer, self.zones[index].net);
        let filled_at_revision = self.node.obstacle_revision();
        let resolver = self.resolver();
        let items = zone_fill::fill_zone(&outline, layer, net, &self.outline, &self.node, resolver);
        self.insert_zone_refill(id, items, filled_at_revision);
    }

    /// Whether any current [`ZoneRecord`]'s fill predates the board's
    /// own `obstacle_revision` -- i.e. a `Pad`/`Via`/`Track`/`Hole` has
    /// moved, appeared, or disappeared since that zone was last
    /// (re)filled, so its on-screen copper may no longer match reality
    /// (stale clearance holes around a footprint's *old* position,
    /// still-solid copper where a pad now sits that didn't before,
    /// etc.) until [`Self::refill_zone`]/[`Self::refill_all_zones`]
    /// catches it up. Surfaced by the GUI as a one-line reminder rather
    /// than auto-refilling on every edit -- an always-fresh fill would
    /// be too expensive during interactive drag.
    pub fn zones_are_stale(&self) -> bool {
        let current = self.node.obstacle_revision();
        self.zones.iter().any(|z| z.filled_at_revision != current)
    }

    /// [`Self::refill_zone`] for every zone currently on the board -- the
    /// "Refill zones" UI action, for when several pours need to catch up
    /// with the board at once rather than one at a time.
    pub fn refill_all_zones(&mut self) {
        for id in self.zones.iter().map(|z| z.id).collect::<Vec<_>>() {
            self.refill_zone(id);
        }
    }

    /// Deletes `id` outright: removes its current fill island(s) from
    /// `self.node` *and* forgets the [`ZoneRecord`] itself -- unlike
    /// [`Self::refill_zone`], which keeps the record around and just
    /// re-runs its fill. The first real "delete a zone" primitive this
    /// editor has; every zone so far could only ever be re-filled, never
    /// removed. Needed by `app.rs`'s "Solid F.Cu/B.Cu plane" checkboxes
    /// (unticking one must make the plane actually go away, not just
    /// stop tracking it) but generically useful beyond that. A no-op if
    /// `id` doesn't name a currently-recorded zone.
    pub fn remove_zone(&mut self, id: ZoneId) {
        let Some(index) = self.zones.iter().position(|z| z.id == id) else { return };
        let record = self.zones.remove(index);
        for item_id in record.item_ids {
            self.node.remove(item_id);
        }
    }

    fn set_pad_net(&mut self, id: ItemId, net: Option<NetId>) -> Result<(), NetError> {
        match self.node.get(id).cloned() {
            Some(Item::Pad { shape, layer, .. }) => {
                self.node.replace(id, Item::Pad { shape, net, layer });
                Ok(())
            }
            _ => Err(NetError::NotAPad),
        }
    }

    /// Creates a brand new, still-empty net (no pads assigned yet) with
    /// an auto-generated name (`Net1`, `Net2`, ... -- never reused, same
    /// "monotonic counter, no id recycling" convention as
    /// [`Self::try_place_footprint`]'s reference designators). Exposed
    /// mainly for the "New net" UI action; [`Self::connect_pads`] calls
    /// this itself when neither pad already has one.
    pub fn create_net(&mut self) -> NetId {
        self.next_net_serial += 1;
        let id = NetId(self.next_net_serial);
        self.nets.push(NetRecord { id, name: format!("Net{}", self.next_net_serial) });
        id
    }

    /// Resolves a human-typed net name (as shown in the GUI net list)
    /// back to its [`NetId`] -- same exact-match style as
    /// [`Self::find_pad`]. Useful for tests and any caller that only
    /// knows the display name.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn find_net_by_name(&self, name: &str) -> Option<NetId> {
        self.nets.iter().find(|n| n.name == name).map(|n| n.id)
    }

    /// Gives an existing net a human name -- turning [`Self::create_net`]'s
    /// auto-generated `"Net7"` into `"GND"`/`"5V"`/whatever the schematic
    /// (or the user's own head) actually calls it, everywhere that name
    /// is shown or exported (GUI net list, ratsnest labels, `get_nets`/MCP).
    /// Trims surrounding whitespace first so a stray leading/trailing
    /// space from a GUI text field can't silently create a
    /// visually-identical-but-technically-different net name.
    pub fn rename_net(&mut self, net: NetId, new_name: &str) -> Result<(), RenameNetError> {
        let trimmed = new_name.trim();
        if trimmed.is_empty() {
            return Err(RenameNetError::EmptyName);
        }
        // Uniqueness matters beyond cosmetics: `Self::find_net_by_name`
        // (the CLI/MCP's own name -> `NetId` lookup) can only ever
        // return one match, so two same-named nets would make one of
        // them permanently unreachable by name.
        if self.nets.iter().any(|n| n.id != net && n.name == trimmed) {
            return Err(RenameNetError::NameAlreadyUsed);
        }
        let record = self.nets.iter_mut().find(|n| n.id == net).ok_or(RenameNetError::NotFound)?;
        record.name = trimmed.to_string();
        Ok(())
    }

    /// Every item currently on `id` -- despite the name, **not** just
    /// pads: `Item::net()` is shared by pads, tracks, vias and zones
    /// alike, and every one of this method's own callers (net-emptiness
    /// pruning, [`Self::remove_net`]'s pad-disconnect loop, the
    /// ratsnest's own re-filter down to just pads) genuinely wants all
    /// of them, not only pads. A UI pin-*count* display wants
    /// [`Self::pad_count_on_net`] instead.
    pub fn pads_on_net(&self, id: NetId) -> Vec<ItemId> {
        self.node.iter_with_ids().filter(|(_, item)| item.net() == Some(id)).map(|(item_id, _)| item_id).collect()
    }

    /// How many actual `Item::Pad`s (not tracks/vias/zones -- see
    /// [`Self::pads_on_net`]'s doc comment) sit on `id` -- what a "N
    /// pin(s)" UI label should count.
    pub fn pad_count_on_net(&self, id: NetId) -> usize {
        self.node.iter().filter(|item| matches!(item, Item::Pad { .. }) && item.net() == Some(id)).count()
    }

    /// Direct pin-to-net assignment (no schematic): joins pads `a` and
    /// `b` onto the same net, creating one if neither has one yet, or
    /// extending whichever one of them already does. Refuses (leaving
    /// both pads' nets untouched) if they already belong to two
    /// different existing nets -- see
    /// [`NetError::AlreadyOnDifferentNets`]'s doc comment for why that's
    /// a deliberate scope cut, not an oversight. Connecting a pad to
    /// itself, or two pads already on the same net, is a harmless no-op
    /// that still returns that net's id.
    pub fn connect_pads(&mut self, a: ItemId, b: ItemId) -> Result<NetId, NetError> {
        let net_a = self.pad_net(a)?;
        let net_b = self.pad_net(b)?;

        let net = match (net_a, net_b) {
            (Some(x), Some(y)) if x == y => x,
            (Some(_), Some(_)) => return Err(NetError::AlreadyOnDifferentNets),
            (Some(x), None) => x,
            (None, Some(y)) => y,
            (None, None) => self.create_net(),
        };

        self.set_pad_net(a, Some(net))?;
        self.set_pad_net(b, Some(net))?;
        Ok(net)
    }

    /// Removes `id` from whatever net it's on (a no-op if it isn't on
    /// one). If that was the net's last remaining pad, the now-empty
    /// [`NetRecord`] is removed too, so disconnecting pads never leaves
    /// silently-orphaned net names behind.
    pub fn disconnect_pad(&mut self, id: ItemId) -> Result<(), NetError> {
        let old_net = self.pad_net(id)?;
        self.set_pad_net(id, None)?;
        if let Some(net) = old_net {
            if self.pads_on_net(net).is_empty() {
                self.nets.retain(|n| n.id != net);
            }
        }
        Ok(())
    }

    /// Disconnects every pad on `id`, **deletes every `Item::Track`/
    /// `Item::Via` still on it** (unlike a pad, a track/via has no
    /// "net-less" fallback state to drop back to -- a trace with no net
    /// is just dead copper nobody could ever route to again), and
    /// removes the (now guaranteed empty) [`NetRecord`] -- the "delete
    /// net" UI action. A single trace/via can still be deleted on its
    /// own, leaving the net itself and its other traces intact, via
    /// [`Self::remove_item`] instead.
    pub fn remove_net(&mut self, id: NetId) {
        let stale_copper: Vec<ItemId> = self
            .node
            .iter_with_ids()
            .filter(|(_, item)| matches!(item, Item::Track { .. } | Item::Via { .. }) && item.net() == Some(id))
            .map(|(item_id, _)| item_id)
            .collect();
        for item_id in stale_copper {
            self.node.remove(item_id);
        }
        for pad_id in self.pads_on_net(id) {
            let _ = self.disconnect_pad(pad_id);
        }
    }

    /// The `ItemId` of the `Item::Track` whose centerline (plus its own
    /// half-width and `tolerance`) contains `point`, if any -- the
    /// click-to-select hit-test behind [`Self::remove_item`]'s "delete
    /// this one trace" gesture. `tolerance` is caller-supplied board
    /// units (see `app.rs`'s `snap_threshold_px`) so a thin, e.g.
    /// 0.25mm-wide trace stays comfortably clickable at any zoom level
    /// rather than demanding a pixel-perfect click on its bare
    /// centerline.
    pub fn track_at(&self, point: Point, tolerance: Unit) -> Option<ItemId> {
        self.node.iter_with_ids().find_map(|(id, item)| match item {
            Item::Track { shape, .. } if alladin_geom::dist_point_to_line(point, shape.a, shape.b) <= (shape.width / 2 + tolerance) as f64 => Some(id),
            _ => None,
        })
    }

    /// The `ItemId` of the `Item::Via` whose copper (plus `tolerance`)
    /// contains `point`, if any -- the via equivalent of
    /// [`Self::track_at`], for the same "delete this one via" gesture.
    pub fn via_at(&self, point: Point, tolerance: Unit) -> Option<ItemId> {
        self.node.iter_with_ids().find_map(|(id, item)| match item {
            Item::Via { shape, .. } if shape.center.distance(point) <= (shape.radius + tolerance) as f64 => Some(id),
            _ => None,
        })
    }

    /// Deletes a single, free-standing `Item::Track` or `Item::Via` --
    /// the one-leg-at-a-time primitive [`Self::remove_wire`] builds on
    /// top of. Returns `false` (touching nothing) for anything else at
    /// `id`, or if `id` doesn't exist: a pad is only ever removed via
    /// [`Self::remove_footprint`] (it doesn't own itself), and a zone's
    /// fill islands are `alladin-pcb`'s own bookkeeping in `self.zones`,
    /// not a bare `Node` item a stray UI click should be able to delete
    /// out from under it.
    fn remove_item(&mut self, id: ItemId) -> bool {
        match self.node.get(id) {
            Some(Item::Track { .. }) | Some(Item::Via { .. }) => {
                self.node.remove(id);
                true
            }
            _ => false,
        }
    }

    /// Every `Item::Track`/`Item::Via` electrically continuous with the
    /// one at `id` -- found by repeatedly following *exact* endpoint/
    /// center-point matches on the same copper layer (an `Item::Via`
    /// bridges both FCu and BCu, so it connects a track on either).
    /// Always includes `id` itself. A single routed connection between
    /// two pins is almost never just one `Item::Track` -- one is added
    /// per corner (see `crate::routing`) and per via-hop -- so this is
    /// what "select/delete the whole wire" has to walk to find every
    /// one of them, not just the one leg that happened to be clicked.
    ///
    /// Deliberately stops at pads: a pad is never included (or treated
    /// as a bridge to some *other* wire that happens to end at the same
    /// pin) since it belongs to its footprint, not to any one wire --
    /// [`Self::remove_footprint`] is the only way to remove one of
    /// those. Exact-point matching (rather than any clearance/overlap
    /// test) is deliberate too: it's exactly the precision every path
    /// this editor itself ever creates already shares (a leg's own
    /// endpoint, a via's center, a pad's center are always the very
    /// same `Point`, never merely close) -- see e.g.
    /// `RoutingDrag::commit`'s `fixed_path`/`dock_path` join.
    pub fn connected_wire(&self, id: ItemId) -> Vec<ItemId> {
        let net = match self.node.get(id) {
            Some(item @ (Item::Track { .. } | Item::Via { .. })) => item.net(),
            _ => return Vec::new(),
        };

        // Every (point, layer) any candidate item on this net touches,
        // paired with the item that touches it -- the adjacency table
        // the walk below repeatedly probes.
        let mut touch_points: Vec<(Point, LayerId, ItemId)> = Vec::new();
        for (item_id, item) in self.node.iter_with_ids() {
            if item.net() != net {
                continue;
            }
            match item {
                Item::Track { shape, layer, .. } => {
                    touch_points.push((shape.a, *layer, item_id));
                    touch_points.push((shape.b, *layer, item_id));
                }
                Item::Via { shape, .. } => {
                    touch_points.push((shape.center, LayerId::FCu, item_id));
                    touch_points.push((shape.center, LayerId::BCu, item_id));
                }
                _ => {}
            }
        }

        let mut visited = std::collections::HashSet::new();
        let mut stack = vec![id];
        while let Some(current) = stack.pop() {
            if !visited.insert(current) {
                continue;
            }
            let own_touches: Vec<(Point, LayerId)> = match self.node.get(current) {
                Some(Item::Track { shape, layer, .. }) => vec![(shape.a, *layer), (shape.b, *layer)],
                Some(Item::Via { shape, .. }) => vec![(shape.center, LayerId::FCu), (shape.center, LayerId::BCu)],
                _ => Vec::new(),
            };
            for &(point, layer) in &own_touches {
                for &(p, l, other_id) in &touch_points {
                    if p == point && l == layer && !visited.contains(&other_id) {
                        stack.push(other_id);
                    }
                }
            }
        }
        visited.into_iter().collect()
    }

    /// Atomically replaces `remove` (a set of `Item::Track`/`Item::Via`
    /// ids -- see `crate::routing::TraceDrag`, whose commit step this
    /// exists for) with a fresh `Item::Track` path: the "drag a trace
    /// segment, its immediate neighbors re-route to follow, everything
    /// further away stays exactly where it was" commit. `remove` is not
    /// required to be contiguous or even electrically connected --
    /// callers (just [`crate::routing::TraceDrag`] today) are trusted to
    /// have already picked a sensible set.
    pub(crate) fn replace_wire_segment(&mut self, remove: &[ItemId], new_path: &[Point], net: NetId, layer: LayerId, width: Unit, class: NetClass) {
        for &id in remove {
            self.remove_item(id);
        }
        self.add_track_path(new_path, net, layer, width, class);
    }

    /// Deletes the *entire* electrically-continuous wire at `id` (see
    /// [`Self::connected_wire`]) -- every corner-leg and via-hop of one
    /// routed connection between two pins, all at once -- the "delete
    /// this whole trace, leave the rest of the net alone" counterpart
    /// to [`Self::remove_net`]'s "delete the whole net, including every
    /// wire on it". Returns `false` (touching nothing) if `id` isn't a
    /// `Item::Track`/`Item::Via` to begin with.
    pub fn remove_wire(&mut self, id: ItemId) -> bool {
        let wire = self.connected_wire(id);
        if wire.is_empty() {
            return false;
        }
        for item_id in wire {
            self.remove_item(item_id);
        }
        true
    }

    fn prune_empty_nets(&mut self) {
        let ids: Vec<NetId> = self.nets.iter().map(|n| n.id).collect();
        for id in ids {
            if self.pads_on_net(id).is_empty() {
                self.nets.retain(|n| n.id != id);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_builds_a_single_outline_polygon_around_the_origin() {
        let params = NewBoardParams { width_mm: 40.0, height_mm: 20.0, layer_count: LayerCount::Two, copper_weight: CopperWeight::OneOz, corner_radius_mm: 0.5 };
        let doc = params.create();
        assert_eq!(doc.outline.len(), 1);
        assert!(doc.outline[0].contains_point(alladin_geom::Point::new(0, 0)));
        assert_eq!(doc.node.iter().count(), 0, "a freshly created board must start with no items");
    }

    #[test]
    fn is_valid_rejects_a_non_positive_size() {
        let mut params = NewBoardParams::default();
        params.width_mm = 0.0;
        assert!(!params.is_valid());
    }

    #[test]
    fn is_valid_rejects_a_corner_radius_that_would_swallow_the_board() {
        let mut params = NewBoardParams::default();
        params.width_mm = 10.0;
        params.height_mm = 10.0;
        params.corner_radius_mm = 6.0; // > half the shorter side
        assert!(!params.is_valid());
    }

    #[test]
    fn is_valid_accepts_sane_defaults() {
        assert!(NewBoardParams::default().is_valid());
    }

    #[test]
    fn pad_to_pad_clearance_matches_each_profiles_own_constant() {
        let one_oz = NewBoardParams { width_mm: 40.0, height_mm: 40.0, layer_count: LayerCount::Two, copper_weight: CopperWeight::OneOz, corner_radius_mm: 0.0 }.create();
        assert_eq!(one_oz.pad_to_pad_clearance(), JlcpcbClearance::PAD_TO_PAD);

        let two_oz = NewBoardParams { width_mm: 40.0, height_mm: 40.0, layer_count: LayerCount::Two, copper_weight: CopperWeight::TwoOz, corner_radius_mm: 0.0 }.create();
        assert_eq!(two_oz.pad_to_pad_clearance(), Jlcpcb2Layer2Oz::PAD_TO_PAD, "must go through resolver()-style copper-weight dispatch, not a hardcoded single profile");
    }

    fn test_board() -> BoardDoc {
        NewBoardParams { width_mm: 40.0, height_mm: 40.0, layer_count: LayerCount::Two, copper_weight: CopperWeight::OneOz, corner_radius_mm: 0.0 }.create()
    }

    fn two_pin_template() -> crate::footprint::FootprintTemplate {
        crate::footprint::builtin_templates().remove(0)
    }

    #[test]
    fn try_place_silk_text_succeeds_in_open_space_and_is_recorded() {
        let mut board = test_board();
        let id = board
            .try_place_silk_text("HELLO", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT)
            .expect("center of an empty board must be legal");
        assert_eq!(board.silk_texts.len(), 1);
        assert_eq!(board.silk_texts[0].id, id);
        assert_eq!(board.silk_texts[0].text, "HELLO");
    }

    #[test]
    fn try_place_silk_text_rejects_empty_or_whitespace_only_text() {
        let mut board = test_board();
        assert_eq!(board.try_place_silk_text("", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT), Err(SilkTextError::EmptyText));
        assert_eq!(board.try_place_silk_text("   ", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT), Err(SilkTextError::EmptyText));
        assert!(board.silk_texts.is_empty());
    }

    #[test]
    fn try_place_silk_text_rejects_off_board() {
        let mut board = test_board();
        let err = board.try_place_silk_text("X", Point::new(1_000 * MM, 1_000 * MM), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap_err();
        assert_eq!(err, SilkTextError::OffBoard);
        assert!(board.silk_texts.is_empty());
    }

    #[test]
    fn try_place_silk_text_rejects_printing_over_a_same_side_pad() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        // Template's own pad sits at world x = +1.27mm on `FCu` (see
        // `try_place_footprint_succeeds_in_open_space_...`'s sibling
        // tests for this same template's real geometry) -- centering
        // text right on top of it must collide.
        let err = board.try_place_silk_text("X", Point::new((1.27 * MM as f64) as Unit, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap_err();
        assert_eq!(err, SilkTextError::TooCloseToPad);
    }

    #[test]
    fn try_place_silk_text_ignores_a_pad_on_the_opposite_side() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        // Same world position as the previous test's colliding case,
        // but on `BCu` -- the pad is physically on the opposite face
        // of the board, so back-side silk must be unaffected by it.
        board
            .try_place_silk_text("X", Point::new((1.27 * MM as f64) as Unit, 0), 0.0, LayerId::BCu, DEFAULT_SILK_TEXT_HEIGHT)
            .expect("a front-side pad must not block back-side silk");
    }

    #[test]
    fn try_place_silk_text_rejects_overlapping_another_text_on_the_same_side() {
        let mut board = test_board();
        board.try_place_silk_text("FIRST", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap();
        let err = board.try_place_silk_text("SECOND", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap_err();
        assert_eq!(err, SilkTextError::OverlapsAnotherText);
        assert_eq!(board.silk_texts.len(), 1, "the rejected second text must not be added");
    }

    #[test]
    fn try_place_silk_text_allows_the_same_spot_on_the_opposite_side() {
        let mut board = test_board();
        board.try_place_silk_text("FIRST", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap();
        board
            .try_place_silk_text("SECOND", Point::new(0, 0), 0.0, LayerId::BCu, DEFAULT_SILK_TEXT_HEIGHT)
            .expect("front and back silk never overlap each other physically");
        assert_eq!(board.silk_texts.len(), 2);
    }

    #[test]
    fn try_place_silk_text_allows_another_text_inside_the_first_ones_whitespace_gap() {
        let mut board = test_board();
        // "A B"'s overall bounding rect spans the space between the
        // two letters, but its real ink (see `SilkText::ink_cells`)
        // doesn't -- a small "." (a dot hugging the baseline) centered
        // in that gap overlaps nothing actually printed, so the old
        // whole-rectangle check's rejection here was exactly the
        // false positive per-character collision exists to fix. The
        // host text is 2mm tall so its space is genuinely wider than
        // the 1mm dot's own cell -- merely *touching* cells still
        // count as overlapping.
        board.try_place_silk_text("A B", Point::new(0, 0), 0.0, LayerId::FCu, mm_to_unit(2.0)).unwrap();
        board
            .try_place_silk_text(".", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT)
            .expect("a dot in the blank gap between \"A\" and \"B\" overlaps no real ink");
        assert_eq!(board.silk_texts.len(), 2);
    }

    #[test]
    fn try_place_silk_text_rejects_printing_under_a_component_body() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        // Dead center between the template's two pads: clears both
        // pads' own `SILK_TO_PAD` margin, but sits squarely under the
        // part's body/courtyard -- unreadable on the real board.
        let err = board.try_place_silk_text("X", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap_err();
        assert_eq!(err, SilkTextError::UnderComponentBody);
        assert!(board.silk_texts.is_empty());
    }

    #[test]
    fn try_place_footprint_rejects_landing_on_existing_silk_text() {
        let mut board = test_board();
        board.try_place_silk_text("LABEL", Point::new(5 * MM, 5 * MM), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap();
        let template = two_pin_template();
        // The exact mirror of `try_place_silk_text_rejects_printing_over_a_same_side_pad`:
        // the outcome must not depend on whether the part or the text
        // came first.
        let err = board.try_place_footprint(&template, Point::new(5 * MM, 5 * MM), 0.0).unwrap_err();
        assert_eq!(err, PlacementError::OverSilkText);
        assert!(board.footprints.is_empty());
        // Well away from the text, the same part still places fine.
        board.try_place_footprint(&template, Point::new(-10 * MM, -10 * MM), 0.0).expect("clear of the text, placement must succeed");
    }

    #[test]
    fn stroke_segments_skip_whitespace_and_confine_a_descender_to_its_own_character() {
        let text = SilkText {
            id: SilkTextId(0),
            text: "a g".to_string(),
            position: Point::new(0, 0),
            rotation_deg: 0.0,
            layer: LayerId::FCu,
            height: DEFAULT_SILK_TEXT_HEIGHT,
            line_width: DEFAULT_SILK_LINE_WIDTH,
        };
        let segments = text.stroke_segments();
        assert!(!segments.is_empty());
        // The space between "a" and "g" prints nothing: a vertical
        // band around the anchor's x must be entirely stroke-free.
        let gap_half_width = DEFAULT_SILK_TEXT_HEIGHT / 8;
        assert!(
            !segments.iter().any(|s| s.a.x.min(s.b.x) < gap_half_width && s.a.x.max(s.b.x) > -gap_half_width),
            "the blank space in \"a g\" must contain no stroke at all"
        );
        // Only the right half (the "g") may reach below the baseline
        // -- its descender is its own, not the "a"'s.
        let lowest_left = segments.iter().filter(|s| s.a.x < 0).map(|s| s.a.y.max(s.b.y)).max().unwrap();
        let lowest_right = segments.iter().filter(|s| s.a.x > 0).map(|s| s.a.y.max(s.b.y)).max().unwrap();
        assert!(lowest_right > lowest_left, "only the \"g\"'s own strokes may reach below the \"a\"'s baseline");
    }

    #[test]
    fn try_move_silk_text_can_move_back_onto_its_own_current_spot() {
        let mut board = test_board();
        let id = board.try_place_silk_text("X", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap();
        // Must not spuriously "collide" with its own, not-yet-updated
        // rectangle -- see `silk_text_fits`'s `ignore` parameter.
        board.try_move_silk_text(id, Point::new(0, 0), 90.0).expect("a text must be able to rotate in place");
        assert_eq!(board.silk_texts[0].rotation_deg, 90.0);
    }

    #[test]
    fn try_move_silk_text_reports_not_found_for_an_unknown_id() {
        let mut board = test_board();
        assert_eq!(board.try_move_silk_text(SilkTextId(999), Point::new(0, 0), 0.0), Err(SilkTextError::NotFound));
    }

    #[test]
    fn remove_silk_text_removes_an_existing_one_and_reports_false_for_an_unknown_id() {
        let mut board = test_board();
        let id = board.try_place_silk_text("X", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap();
        assert!(!board.remove_silk_text(SilkTextId(999)));
        assert!(board.remove_silk_text(id));
        assert!(board.silk_texts.is_empty());
    }

    #[test]
    fn silk_text_at_finds_a_placed_text_by_a_point_inside_its_bounding_rect_but_not_far_away() {
        let mut board = test_board();
        let id = board.try_place_silk_text("HELLO", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap();
        assert_eq!(board.silk_text_at(Point::new(0, 0)), Some(id), "dead center must always be inside its own bounding rect");
        assert_eq!(board.silk_text_at(Point::new(19 * MM, 19 * MM)), None);
    }

    #[test]
    fn bounding_rect_hugs_the_real_ink_and_grows_only_downward_for_a_descender() {
        let no_descender = SilkText {
            id: SilkTextId(0),
            text: "HELLO".to_string(),
            position: Point::new(0, 0),
            rotation_deg: 0.0,
            layer: LayerId::FCu,
            height: DEFAULT_SILK_TEXT_HEIGHT,
            line_width: DEFAULT_SILK_LINE_WIDTH,
        };
        let top = no_descender.bounding_rect().points[0].y;
        let bottom = no_descender.bounding_rect().points[2].y;
        assert!(top < 0 && bottom > 0, "the anchor must sit inside the ink box (KiCad centers its text on the anchor)");
        // Real Hershey ink: roughly one cap height tall plus the
        // stroke width's own padding -- nowhere near the old
        // guessed-rectangle's fixed proportions.
        let ink_height = bottom - top;
        assert!(
            ink_height > DEFAULT_SILK_TEXT_HEIGHT && ink_height < DEFAULT_SILK_TEXT_HEIGHT + 2 * DEFAULT_SILK_LINE_WIDTH,
            "a no-descender string's ink box must be cap height plus stroke padding, got {ink_height}"
        );

        let with_descender = SilkText { text: "HELLg".to_string(), ..no_descender };
        let top2 = with_descender.bounding_rect().points[0].y;
        let bottom2 = with_descender.bounding_rect().points[2].y;
        assert_eq!(top2, top, "the *top* edge must be unaffected by a descender -- only the bottom grows");
        assert!(bottom2 > bottom, "a real \"g\" must push the bottom edge further down than a descender-free string's");
    }

    #[test]
    fn bounding_rect_is_narrower_for_a_string_of_narrow_characters_than_the_same_count_of_normal_ones() {
        let narrow = SilkText {
            id: SilkTextId(0),
            text: "iiii".to_string(),
            position: Point::new(0, 0),
            rotation_deg: 0.0,
            layer: LayerId::FCu,
            height: DEFAULT_SILK_TEXT_HEIGHT,
            line_width: DEFAULT_SILK_LINE_WIDTH,
        };
        let normal = SilkText { text: "abcd".to_string(), ..narrow };
        let wide = SilkText { text: "MMMM".to_string(), ..narrow };
        let half_width = |t: &SilkText| t.bounding_rect().points[1].x;
        assert!(half_width(&narrow) < half_width(&normal), "narrow marks like \"iiii\" must claim less width than ordinary letters");
        assert!(half_width(&wide) > half_width(&normal), "genuinely wide glyphs like \"MMMM\" must claim more width than ordinary letters");
    }

    #[test]
    fn try_resize_silk_text_grows_the_bounding_rect_and_can_newly_collide_with_a_pad() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        // A "T" directly above the template's pad at (+1.27mm, 0) --
        // same geometry as `check_silk_text_placement_can_reject_a_bigger_size...`:
        // the stem's bottom tip points straight at the pad, clearing
        // its `SILK_TO_PAD` margin (and the part's own body) at 1.0mm
        // and 1.5mm, but reaching inside it once grown to 3.0mm.
        let id = board
            .try_place_silk_text("T", Point::new((1.27 * MM as f64) as Unit, (-1.6 * MM as f64) as Unit), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT)
            .expect("must clear the pad at the default size");

        board.try_resize_silk_text(id, mm_to_unit(1.5)).expect("a modest resize must still clear the pad");
        assert_eq!(board.silk_texts[0].height, mm_to_unit(1.5));

        let err = board.try_resize_silk_text(id, mm_to_unit(3.0)).unwrap_err();
        assert_eq!(err, SilkTextError::TooCloseToPad);
        // Refused resize must leave the previous, successfully-applied size untouched.
        assert_eq!(board.silk_texts[0].height, mm_to_unit(1.5));
    }

    #[test]
    fn try_resize_silk_text_reports_not_found_for_an_unknown_id() {
        let mut board = test_board();
        assert_eq!(board.try_resize_silk_text(SilkTextId(999), mm_to_unit(2.0)), Err(SilkTextError::NotFound));
    }

    #[test]
    fn check_silk_text_move_ignores_the_texts_own_current_position_but_still_checks_other_collisions() {
        let mut board = test_board();
        let id = board.try_place_silk_text("X", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap();
        // Must not spuriously "collide" with its own current rectangle,
        // exactly like `try_move_silk_text` itself.
        assert!(board.check_silk_text_move(id, Point::new(0, 0), 0.0).is_ok());
        // But a real off-board destination must still be refused.
        assert_eq!(board.check_silk_text_move(id, Point::new(1_000 * MM, 1_000 * MM), 0.0), Err(SilkTextError::OffBoard));
        // A read-only check must never actually move anything.
        assert_eq!(board.silk_texts[0].position, Point::new(0, 0));
    }

    #[test]
    fn check_silk_text_placement_can_reject_a_bigger_size_that_a_smaller_one_would_accept() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        // A "T" directly above the template's pad at (+1.27mm, 0):
        // its stem's bottom tip points straight at the pad -- well
        // clear of the `SILK_TO_PAD` margin (and the part's own body)
        // at 1.0mm, but a 3.0mm-tall "T"'s stem reaches down to
        // within that margin of the pad's copper.
        let position = Point::new((1.27 * MM as f64) as Unit, (-1.6 * MM as f64) as Unit);
        assert!(board.check_silk_text_placement("T", position, 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).is_ok());
        assert_eq!(board.check_silk_text_placement("T", position, 0.0, LayerId::FCu, mm_to_unit(3.0)), Err(SilkTextError::TooCloseToPad));
    }

    #[test]
    fn silk_dot_place_move_resize_hit_test_and_remove_round_trip() {
        let mut board = test_board();
        let id = board.try_place_silk_dot(Point::new(0, 0), DEFAULT_SILK_DOT_DIAMETER, LayerId::FCu).expect("center of an empty board must be legal");
        assert_eq!(board.silk_dots.len(), 1);
        assert_eq!(board.silk_dot_at(Point::new(100_000, 0)), Some(id), "a click within the (floored) hit radius must select the dot");
        board.try_move_silk_dot(id, Point::new(5 * MM, 5 * MM)).expect("moving into open space must succeed");
        assert_eq!(board.silk_dots[0].position, Point::new(5 * MM, 5 * MM));
        board.try_resize_silk_dot(id, mm_to_unit(1.0)).expect("growing in open space must succeed");
        assert_eq!(board.silk_dots[0].diameter, mm_to_unit(1.0));
        assert!(board.remove_silk_dot(id));
        assert!(board.silk_dots.is_empty());
        assert!(!board.remove_silk_dot(id), "removing an already-removed dot must report false");
    }

    #[test]
    fn silk_dot_placement_rejects_pad_proximity_off_board_and_opposite_side_is_independent() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        // Right on top of a pad: silk over copper, refused.
        assert_eq!(
            board.check_silk_dot_placement(Point::new((1.27 * MM as f64) as Unit, 0), DEFAULT_SILK_DOT_DIAMETER, LayerId::FCu),
            Err(SilkDotError::TooCloseToPad)
        );
        // Same spot on the *back* silk: the pad and body are front-side
        // only, so this must be legal.
        board
            .check_silk_dot_placement(Point::new((1.27 * MM as f64) as Unit, 0), DEFAULT_SILK_DOT_DIAMETER, LayerId::BCu)
            .expect("the back side has no pads/body here");
        // Hugging the board edge (40mm board -> outline at +20mm).
        assert_eq!(
            board.check_silk_dot_placement(Point::new(20 * MM, 0), DEFAULT_SILK_DOT_DIAMETER, LayerId::FCu),
            Err(SilkDotError::OffBoard)
        );
    }

    #[test]
    fn silk_dot_and_silk_text_refuse_to_overlap_in_both_directions() {
        let mut board = test_board();
        board.try_place_silk_text("HELLO", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).unwrap();
        // A dot dropped into the middle of the text's ink.
        assert_eq!(board.check_silk_dot_placement(Point::new(0, 0), DEFAULT_SILK_DOT_DIAMETER, LayerId::FCu), Err(SilkDotError::OverlapsSilk));
        // And the reverse: text over an existing dot, on a fresh board
        // so only the dot can be the reason.
        let mut board = test_board();
        board.try_place_silk_dot(Point::new(0, 0), DEFAULT_SILK_DOT_DIAMETER, LayerId::FCu).unwrap();
        assert_eq!(
            board.check_silk_text_placement("HELLO", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT),
            Err(SilkTextError::OverlapsDot)
        );
    }

    #[test]
    fn footprint_placement_refuses_to_land_on_a_dot_but_the_dots_own_footprint_move_ignores_its_marker() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_silk_dot(Point::new(0, 0), DEFAULT_SILK_DOT_DIAMETER, LayerId::FCu).unwrap();
        // Dropping the part right on the dot: refused, same
        // order-independence contract as `OverSilkText`.
        assert_eq!(board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap_err(), PlacementError::OverSilkDot);
        // Well away from the dot it still places fine.
        board.try_place_footprint(&template, Point::new(-10 * MM, -10 * MM), 0.0).expect("clear of the dot, placement must succeed");
    }

    #[test]
    fn pin1_marker_enables_next_to_pad_1_rides_along_with_a_move_and_disables_cleanly() {
        let mut board = test_board();
        let template = two_pin_template();
        let id = board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        board.try_enable_pin1_marker(id, &template).expect("an empty 40mm board has room for a pin-1 dot");
        let first = board.footprints[0].pin1_marker_circle().expect("marker must be set");
        // The committed spot must itself be DFM-legal against the
        // part's own pads (the sweep clears pad reach + SILK_TO_PAD).
        for item in board.node.iter() {
            if let Item::Pad { shape, layer: LayerId::FCu, .. } = item {
                let too_close = match shape {
                    PadShape::Circle(p) => circles_touch(&first, p, JlcpcbDfm::SILK_TO_PAD),
                    PadShape::Polygon { outline, .. } => circle_polygon_collides(&first, outline, JlcpcbDfm::SILK_TO_PAD),
                };
                assert!(!too_close, "the marker must clear every pad by the full SILK_TO_PAD margin");
            }
        }
        // Moving the part carries the marker along rigidly.
        board.try_move_footprint(id, &template, Point::new(5 * MM, 5 * MM), 0.0).expect("open space");
        let moved = board.footprints[0].pin1_marker_circle().unwrap();
        assert_eq!(moved.center, first.center.add(Point::new(5 * MM, 5 * MM)));
        // And the marker is real silk: a dot may not be placed on it.
        assert_eq!(board.check_silk_dot_placement(moved.center, DEFAULT_SILK_DOT_DIAMETER, LayerId::FCu), Err(SilkDotError::OverlapsSilk));
        assert!(board.disable_pin1_marker(id));
        assert!(board.footprints[0].pin1_marker.is_none());
    }

    #[test]
    fn check_silk_text_placement_is_read_only_and_also_rejects_empty_text() {
        let board = test_board();
        assert_eq!(
            board.check_silk_text_placement("", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT),
            Err(SilkTextError::EmptyText)
        );
        assert!(board.check_silk_text_placement("X", Point::new(0, 0), 0.0, LayerId::FCu, DEFAULT_SILK_TEXT_HEIGHT).is_ok());
        // A read-only check must never actually place anything.
        assert!(board.silk_texts.is_empty());
    }

    #[test]
    fn try_place_footprint_succeeds_in_open_space_and_adds_its_pads_to_the_node() {
        let mut board = test_board();
        let template = two_pin_template();
        let id = board.try_place_footprint(&template, Point::new(0, 0), 0.0).expect("open space must succeed");

        assert_eq!(board.node.iter().count(), template.pads.len());
        let placed = board.footprints.iter().find(|f| f.id == id).unwrap();
        assert_eq!(placed.reference, "P1");
        assert_eq!(placed.pad_item_ids.len(), template.pads.len());
    }

    #[test]
    fn try_place_footprint_off_board_is_rejected_and_touches_nothing() {
        let mut board = test_board();
        let template = two_pin_template();
        let far_away = Point::new(1_000 * MM, 1_000 * MM);

        let err = board.try_place_footprint(&template, far_away, 0.0).unwrap_err();
        assert_eq!(err, PlacementError::OffBoard);
        assert_eq!(board.node.iter().count(), 0, "a rejected placement must not add anything");
        assert!(board.footprints.is_empty());
    }

    #[test]
    fn try_place_footprint_rejects_a_pad_too_close_to_the_board_edge_even_though_it_is_technically_on_board() {
        // 40mm board -> outline at x = ±20mm. Positioned so the far pad
        // (template offset +1.27mm, radius 0.45mm) sits only 0.05mm
        // from the edge -- comfortably *on*-board (would pass a bare
        // `circle_within_outline` check), but well inside JLCPCB's real
        // 0.20mm `copper_to_routed_edge` minimum (see
        // `JlcpcbDfm::COPPER_TO_ROUTED_EDGE`). This is the exact
        // scenario a plain outline-containment check would wrongly
        // accept.
        let mut board = test_board();
        let template = two_pin_template();
        let gap_from_edge = 50_000; // 0.05mm -- less than the 0.20mm minimum
        let far_pad_center_x = 20 * MM - gap_from_edge - mm_to_unit(0.45);
        let position = Point::new(far_pad_center_x - mm_to_unit(1.27), 0);

        let err = board.try_place_footprint(&template, position, 0.0).unwrap_err();
        assert_eq!(err, PlacementError::OffBoard);
        assert!(board.footprints.is_empty());
    }

    #[test]
    fn try_place_footprint_accepts_a_pad_that_clears_the_edge_by_the_full_dfm_minimum() {
        // Same geometry as above, but with a comfortable 3mm gap --
        // past *both* real edge minimums that now apply here: the
        // 0.20mm copper `OffBoard` one this test originally targeted,
        // and the newer, much stricter 2.5mm *body* `BodyOffBoard`
        // one (`JlcpcbDfm::COMPONENT_BODY_TO_EDGE`) that a two-pad
        // template's own fallback courtyard -- exactly its own pads'
        // bounding box, no extra margin -- is just as subject to.
        // Makes sure the new edge check doesn't over-reject placements
        // that are actually fine.
        let mut board = test_board();
        let template = two_pin_template();
        let gap_from_edge = 3 * MM;
        let far_pad_center_x = 20 * MM - gap_from_edge - mm_to_unit(0.45);
        let position = Point::new(far_pad_center_x - mm_to_unit(1.27), 0);

        board.try_place_footprint(&template, position, 0.0).expect("a well-cleared edge placement must succeed");
    }

    fn mounting_hole_template() -> crate::footprint::FootprintTemplate {
        crate::footprint::builtin_templates().into_iter().find(|t| t.name.starts_with("Mounting hole (M3")).unwrap()
    }

    #[test]
    fn try_place_footprint_splits_pads_and_holes_into_their_own_id_lists() {
        let mut board = test_board();
        let template = mounting_hole_template();
        let id = board.try_place_footprint(&template, Point::new(0, 0), 0.0).expect("open space must succeed");

        let placed = board.footprints.iter().find(|f| f.id == id).unwrap();
        assert!(placed.pad_item_ids.is_empty(), "a pure mounting hole has no pads");
        assert_eq!(placed.hole_item_ids.len(), 1);
        assert!(matches!(board.node.get(placed.hole_item_ids[0]), Some(Item::Hole { .. })));
    }

    #[test]
    fn try_place_footprint_rejects_a_mounting_hole_too_close_to_the_board_edge() {
        // 40mm board -> outline at x = ±20mm. A hole placed only
        // 0.1mm from its own drill radius to the edge is well inside
        // `MIN_NPTH_HOLE`'s 0.5mm margin (see `check_placement`'s own
        // `Item::Hole` arm) -- must be rejected exactly like a pad
        // that violates `COPPER_TO_ROUTED_EDGE` is.
        let mut board = test_board();
        let template = mounting_hole_template();
        let drill_radius = template.holes[0].drill / 2;
        let gap_from_edge = 100_000; // 0.1mm -- less than the 0.5mm minimum
        let position = Point::new(20 * MM - gap_from_edge - drill_radius, 0);

        let err = board.try_place_footprint(&template, position, 0.0).unwrap_err();
        assert_eq!(err, PlacementError::OffBoard);
        assert!(board.footprints.is_empty());
    }

    #[test]
    fn try_move_footprint_moves_both_the_pads_and_the_holes_of_a_mixed_template() {
        let mut board = test_board();
        let template = crate::footprint::FootprintTemplate {
            name: "mixed".to_string(),
            reference_prefix: "T".to_string(),
            pads: vec![crate::footprint::PadTemplate {
                offset: Point::new(-mm_to_unit(1.0), 0),
                radius: mm_to_unit(0.45),
                layer: LayerId::FCu,
                number: "1".to_string(),
                shape: crate::footprint::PadShapeKind::Circle,
                rotation_deg: 0.0,
                hole_diameter: None,
                pin_name: None,
            }],
            holes: vec![crate::footprint::HoleTemplate { offset: Point::new(mm_to_unit(1.0), 0), drill: mm_to_unit(2.2) }],
            exclude_from_bom: true,
            explicit_courtyard: None,
        };
        let id = board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let new_position = Point::new(5 * MM, 5 * MM);
        board.try_move_footprint(id, &template, new_position, 0.0).expect("open space must succeed");

        let placed = board.footprints.iter().find(|f| f.id == id).unwrap();
        let pad_center = match board.node.get(placed.pad_item_ids[0]).unwrap() {
            Item::Pad { shape, .. } => shape.center(),
            _ => panic!("expected a pad"),
        };
        let hole_position = match board.node.get(placed.hole_item_ids[0]).unwrap() {
            Item::Hole { position, .. } => *position,
            _ => panic!("expected a hole"),
        };
        assert_eq!(pad_center, Point::new(new_position.x - mm_to_unit(1.0), new_position.y));
        assert_eq!(hole_position, Point::new(new_position.x + mm_to_unit(1.0), new_position.y));
    }

    #[test]
    fn remove_footprint_removes_both_its_pads_and_its_holes_from_the_node() {
        let mut board = test_board();
        let template = mounting_hole_template();
        let id = board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        assert_eq!(board.node.iter().count(), 1);

        board.remove_footprint(id);
        assert_eq!(board.node.iter().count(), 0, "removing a mounting hole footprint must remove its hole too");
        assert!(board.footprints.is_empty());
    }

    #[test]
    fn matrix_positions_is_symmetric_around_the_given_center() {
        let positions = BoardDoc::matrix_positions(2, 3, 2 * MM, 3 * MM, Point::new(0, 0));
        assert_eq!(positions.len(), 6);
        let sum_x: Unit = positions.iter().map(|p| p.x).sum();
        let sum_y: Unit = positions.iter().map(|p| p.y).sum();
        assert_eq!(sum_x, 0, "an even column count must average out exactly to the center on x");
        assert_eq!(sum_y, 0, "an even row count must average out exactly to the center on y");
        assert!(positions.contains(&Point::new(-2 * MM, -1_500_000)), "leftmost column, top row");
        assert!(positions.contains(&Point::new(2 * MM, 1_500_000)), "rightmost column, bottom row");
    }

    #[test]
    fn matrix_positions_degenerates_to_a_single_point_for_a_1x1_matrix() {
        let center = Point::new(5 * MM, -3 * MM);
        assert_eq!(BoardDoc::matrix_positions(1, 1, 10 * MM, 10 * MM, center), vec![center]);
    }

    #[test]
    fn check_matrix_placement_succeeds_for_a_well_spaced_grid_in_open_space() {
        let board = test_board();
        let template = two_pin_template();
        let positions = BoardDoc::matrix_positions(2, 2, 10 * MM, 10 * MM, Point::new(0, 0));
        board.check_matrix_placement(&template, &positions, 0.0).expect("a well-spaced grid in open space must be legal");
    }

    #[test]
    fn check_matrix_placement_rejects_a_pitch_too_tight_for_the_templates_own_pads() {
        // The template's own two pads sit 2.54mm apart -- a 0.5mm grid
        // pitch means neighbouring matrix cells' pads land almost
        // exactly on top of each other, even though nothing else is on
        // the board at all.
        let board = test_board();
        let template = two_pin_template();
        let positions = BoardDoc::matrix_positions(1, 3, 500_000, 10 * MM, Point::new(0, 0));
        assert!(
            board.check_matrix_placement(&template, &positions, 0.0).is_err(),
            "matrix members that would collide with each other must be rejected, not just individually-off-board/colliding-with-the-real-board cells"
        );
    }

    #[test]
    fn check_matrix_placement_rejects_the_whole_grid_when_any_single_cell_collides_with_an_existing_part() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap(); // occupies the grid's center cell

        let positions = BoardDoc::matrix_positions(1, 3, 10 * MM, 10 * MM, Point::new(0, 0));
        assert!(board.check_matrix_placement(&template, &positions, 0.0).is_err());
    }

    #[test]
    fn place_matrix_commits_every_cell_with_sequential_references() {
        let mut board = test_board();
        let template = two_pin_template();
        let positions = BoardDoc::matrix_positions(2, 2, 10 * MM, 10 * MM, Point::new(0, 0));
        let ids = board.place_matrix(&template, &positions, 0.0).expect("a well-spaced grid in open space must commit");

        assert_eq!(ids.len(), 4);
        assert_eq!(board.footprints.len(), 4);
        let mut references: Vec<&str> = board.footprints.iter().map(|f| f.reference.as_str()).collect();
        references.sort();
        assert_eq!(references, vec!["P1", "P2", "P3", "P4"]);
        assert_eq!(board.node.iter().count(), 4 * template.pads.len());
    }

    #[test]
    fn place_matrix_commits_nothing_when_the_grid_is_rejected() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let footprints_before = board.footprints.len();
        let node_items_before = board.node.iter().count();

        let positions = BoardDoc::matrix_positions(1, 3, 10 * MM, 10 * MM, Point::new(0, 0));
        assert!(board.place_matrix(&template, &positions, 0.0).is_err());
        assert_eq!(board.footprints.len(), footprints_before, "a rejected matrix must not partially commit");
        assert_eq!(board.node.iter().count(), node_items_before);
    }

    #[test]
    fn try_place_footprint_rejects_overlapping_a_previously_placed_one() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).expect("first placement must succeed");

        let err = board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap_err();
        assert!(matches!(err, PlacementError::Collision(_)));
        assert_eq!(board.footprints.len(), 1, "the second, rejected placement must not be recorded");
    }

    /// A single tiny pad (so it can never *itself* collide with a
    /// same-shaped neighbour a few mm away) sitting inside a large,
    /// explicit 4mm x 4mm courtyard -- the minimal template needed to
    /// exercise `PlacementError::BodyOverlap` on its own, decoupled
    /// from ordinary pad/copper collision.
    fn wide_courtyard_template() -> crate::footprint::FootprintTemplate {
        crate::footprint::FootprintTemplate {
            name: "wide-courtyard-test".to_string(),
            reference_prefix: "T".to_string(),
            pads: vec![crate::footprint::PadTemplate {
                offset: Point::new(0, 0),
                radius: mm_to_unit(0.2),
                layer: LayerId::FCu,
                number: "1".to_string(),
                shape: crate::footprint::PadShapeKind::Circle,
                rotation_deg: 0.0,
                hole_diameter: None,
                pin_name: None,
            }],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: Some(crate::footprint::Courtyard { center: Point::new(0, 0), width: mm_to_unit(4.0), height: mm_to_unit(4.0) }),
        }
    }

    #[test]
    fn try_place_footprint_rejects_a_body_overlap_even_though_the_pads_themselves_stay_clear() {
        let mut board = test_board();
        let template = wide_courtyard_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).expect("first placement must succeed");

        // 3mm apart: the two tiny (0.2mm-radius) pads are nowhere near
        // each other, but the two 4mm-wide courtyards (half-width
        // 2mm each) overlap by a full 1mm.
        let err = board.try_place_footprint(&template, Point::new(mm_to_unit(3.0), 0), 0.0).unwrap_err();
        assert_eq!(err, PlacementError::BodyOverlap);
        assert_eq!(board.footprints.len(), 1, "the rejected placement must not be recorded");
    }

    #[test]
    fn try_place_footprint_accepts_two_bodies_placed_clear_of_each_other() {
        let mut board = test_board();
        let template = wide_courtyard_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).expect("first placement must succeed");

        // 5mm apart: comfortably clears two 4mm-wide (half-width 2mm)
        // courtyards plus the 0.3mm assembly body clearance.
        board.try_place_footprint(&template, Point::new(mm_to_unit(5.0), 0), 0.0).expect("well-separated bodies must not be rejected");
        assert_eq!(board.footprints.len(), 2);
    }

    #[test]
    fn try_place_footprint_rejects_bodies_closer_than_0_3mm_assembly_clearance() {
        // Tiny pad, tiny 1x1mm courtyard -- place first at 0, second at
        // 1.25mm: body gap = 1.25 - 0.5 - 0.5 = 0.25mm < 0.3mm.
        let mut board = test_board();
        let template = crate::footprint::FootprintTemplate {
            name: "tiny-body".into(),
            reference_prefix: "T".into(),
            pads: vec![crate::footprint::PadTemplate {
                offset: Point::new(0, 0),
                radius: mm_to_unit(0.15),
                layer: LayerId::FCu,
                number: "1".into(),
                shape: crate::footprint::PadShapeKind::Circle,
                rotation_deg: 0.0,
                hole_diameter: None,
                pin_name: None,
            }],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: Some(crate::footprint::Courtyard {
                center: Point::new(0, 0),
                width: mm_to_unit(1.0),
                height: mm_to_unit(1.0),
            }),
        };
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let err = board.try_place_footprint(&template, Point::new(mm_to_unit(1.25), 0), 0.0).unwrap_err();
        assert_eq!(err, PlacementError::BodyOverlap);
    }

    #[test]
    fn try_place_footprint_accepts_bodies_just_over_0_3mm_assembly_clearance() {
        let mut board = test_board();
        let template = crate::footprint::FootprintTemplate {
            name: "tiny-body".into(),
            reference_prefix: "T".into(),
            pads: vec![crate::footprint::PadTemplate {
                offset: Point::new(0, 0),
                radius: mm_to_unit(0.15),
                layer: LayerId::FCu,
                number: "1".into(),
                shape: crate::footprint::PadShapeKind::Circle,
                rotation_deg: 0.0,
                hole_diameter: None,
                pin_name: None,
            }],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: Some(crate::footprint::Courtyard {
                center: Point::new(0, 0),
                width: mm_to_unit(1.0),
                height: mm_to_unit(1.0),
            }),
        };
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        // gap = 1.35 - 0.5 - 0.5 = 0.35mm > 0.3mm
        board.try_place_footprint(&template, Point::new(mm_to_unit(1.35), 0), 0.0).expect("0.35mm body gap must clear 0.3mm floor");
    }

    #[test]
    fn try_place_footprint_rejects_smd_pad_too_close_to_a_foreign_pth_drill() {
        // NPTH mounting hole (no copper annulus): Node pad-vs-hole uses
        // only 0.15mm, while lead-to-hole wants 0.3mm -- place in that
        // window so copper clears and assembly lead-to-hole is the gate.
        let mut board = test_board();
        let th = crate::footprint::FootprintTemplate {
            name: "npth".into(),
            reference_prefix: "H".into(),
            pads: Vec::new(),
            holes: vec![crate::footprint::HoleTemplate {
                offset: Point::new(0, 0),
                drill: mm_to_unit(1.0), // r=0.5
            }],
            exclude_from_bom: true,
            explicit_courtyard: None,
        };
        let smd = crate::footprint::FootprintTemplate {
            name: "smd".into(),
            reference_prefix: "R".into(),
            pads: vec![crate::footprint::PadTemplate {
                offset: Point::new(0, 0),
                radius: mm_to_unit(0.2),
                layer: LayerId::FCu,
                number: "1".into(),
                shape: crate::footprint::PadShapeKind::Circle,
                rotation_deg: 0.0,
                hole_diameter: None,
                pin_name: None,
            }],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: None,
        };
        board.try_place_footprint(&th, Point::new(0, 0), 0.0).unwrap();
        // Lead/body need: 0.5+0.2+0.3 = 1.0mm. Pad-vs-hole copper: +0.15 → 0.85.
        // At 0.92mm copper passes; lead-to-hole (checked before body) rejects.
        let err = board.try_place_footprint(&smd, Point::new(mm_to_unit(0.92), 0), 0.0).unwrap_err();
        assert_eq!(err, PlacementError::LeadToHole, "got {err}");
    }

    #[test]
    fn try_place_footprint_rejects_a_body_within_the_2_5mm_assembly_margin_of_the_board_edge() {
        // 40mm board -> outline at x = +20mm. `wide_courtyard_template`'s
        // tiny 0.2mm-radius pad clears JLCPCB's *copper* edge margin
        // (0.2mm) by a wide margin here (2.8mm), but its 4mm-wide
        // (half-width 2mm) courtyard sits only 1mm from the edge --
        // inside the real, stricter 2.5mm *body* assembly margin
        // (`JlcpcbDfm::COMPONENT_BODY_TO_EDGE`). Isolates
        // `PlacementError::BodyOffBoard` from the pre-existing
        // pad-only `OffBoard` check.
        let mut board = test_board();
        let template = wide_courtyard_template();
        let position = Point::new(mm_to_unit(17.0), 0);

        let err = board.try_place_footprint(&template, position, 0.0).unwrap_err();
        assert_eq!(err, PlacementError::BodyOffBoard);
        assert!(board.footprints.is_empty(), "the rejected placement must not be recorded");
    }

    #[test]
    fn try_place_footprint_accepts_a_body_that_clears_the_2_5mm_assembly_margin() {
        // Same template, but pulled in to a 3mm clearance -- past the
        // 2.5mm assembly minimum -- to confirm the new check isn't
        // over-eager.
        let mut board = test_board();
        let template = wide_courtyard_template();
        let position = Point::new(mm_to_unit(15.0), 0);

        board.try_place_footprint(&template, position, 0.0).expect("a body well clear of the edge must be accepted");
    }

    #[test]
    fn set_outline_refuses_and_touches_nothing_when_a_placed_body_would_end_up_too_close_to_a_shrunk_edge() {
        // A body that comfortably clears the *original* 40mm board's
        // edge (see the "accepts" test above) is stranded within the
        // 2.5mm assembly margin once the board is shrunk to 32mm
        // (outline at x = ±16mm): the very same 3mm clearance the
        // template enjoyed against the old edge becomes -1mm (a real
        // overhang) against the new one.
        let mut board = test_board();
        let template = wide_courtyard_template();
        let position = Point::new(mm_to_unit(15.0), 0);
        board.try_place_footprint(&template, position, 0.0).unwrap();

        let smaller = vec![Polygon::rounded_rect(mm_to_unit(32.0), mm_to_unit(32.0), 0, 12)];
        let err = board.set_outline(smaller, &[template]).unwrap_err();
        assert!(matches!(err, SetOutlineError::FootprintOffBoard(_)));
        assert_eq!(
            board.outline,
            vec![Polygon::rounded_rect(mm_to_unit(40.0), mm_to_unit(40.0), 0, 12)],
            "a refused outline change must leave the board's own outline untouched"
        );
    }

    #[test]
    fn try_move_footprint_rejects_a_body_overlap_and_leaves_position_unchanged() {
        let mut board = test_board();
        let template = wide_courtyard_template();
        let moving = board.try_place_footprint(&template, Point::new(mm_to_unit(-5.0), 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();

        let err = board.try_move_footprint(moving, &template, Point::new(mm_to_unit(3.0), 0), 0.0).unwrap_err();
        assert_eq!(err, PlacementError::BodyOverlap);
        let placed = board.footprints.iter().find(|f| f.id == moving).unwrap();
        assert_eq!(placed.position, Point::new(mm_to_unit(-5.0), 0), "a rejected move must leave the footprint exactly where it was");
    }

    #[test]
    fn check_matrix_placement_rejects_a_grid_whose_own_members_bodies_would_overlap_each_other() {
        let board = test_board();
        let template = wide_courtyard_template();
        // Pads 3mm apart clear each other (see the template's own doc
        // comment), but the grid's own two members' 4mm-wide bodies
        // still overlap each other, entirely within this one new
        // batch -- must be caught even though neither cell collides
        // with anything already on the board.
        let positions = vec![Point::new(0, 0), Point::new(mm_to_unit(3.0), 0)];
        let err = board.check_matrix_placement(&template, &positions, 0.0).unwrap_err();
        assert_eq!(err, PlacementError::BodyOverlap);
    }

    #[test]
    fn try_move_footprint_to_a_clear_spot_succeeds_and_updates_pad_positions() {
        let mut board = test_board();
        let template = two_pin_template();
        let id = board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();

        let new_position = Point::new(5 * MM, 5 * MM);
        board.try_move_footprint(id, &template, new_position, 0.0).expect("moving to open space must succeed");

        let placed = board.footprints.iter().find(|f| f.id == id).unwrap();
        assert_eq!(placed.position, new_position);
        for &pad_id in &placed.pad_item_ids {
            let Item::Pad { shape, .. } = board.node.get(pad_id).unwrap() else { panic!("expected a pad") };
            assert!(shape.center().distance(new_position) < (3 * MM) as f64, "pad should have moved along with the footprint");
        }
    }

    #[test]
    fn try_move_footprint_does_not_collide_with_its_own_previous_position() {
        // Moving a footprint back onto (roughly) its own current spot
        // must not be refused as "colliding with itself".
        let mut board = test_board();
        let template = two_pin_template();
        let id = board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();

        board.try_move_footprint(id, &template, Point::new(0, 0), 0.0).expect("moving onto its own spot must succeed");
    }

    #[test]
    fn try_move_footprint_into_another_footprint_is_rejected_and_leaves_position_unchanged() {
        let mut board = test_board();
        let template = two_pin_template();
        let a = board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();

        let err = board.try_move_footprint(a, &template, Point::new(10 * MM, 0), 0.0).unwrap_err();
        assert!(matches!(err, PlacementError::Collision(_)));
        let placed = board.footprints.iter().find(|f| f.id == a).unwrap();
        assert_eq!(placed.position, Point::new(-10 * MM, 0), "a rejected move must leave the footprint where it was");
    }

    #[test]
    fn try_move_footprint_preserves_each_pads_net_membership() {
        // Regression test: `world_items` always builds a fresh pad with
        // `net: None` (see its own doc comment -- nets are assigned
        // after placement, not part of the static template), and
        // `try_move_footprint` used to swap that net-less item straight
        // in via `Node::replace`, silently disconnecting every pad on
        // every single drag.
        let mut board = test_board();
        let template = two_pin_template();
        let moving = board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let net = board.connect_pads(a, b).expect("connecting two unassigned pads must succeed");
        assert_eq!(board.pads_on_net(net).len(), 2);

        let new_position = Point::new(-10 * MM, 15 * MM);
        board.try_move_footprint(moving, &template, new_position, 90.0).expect("moving to open space must succeed");

        assert_eq!(board.pads_on_net(net).len(), 2, "both pads must still share the net after the move");
        let Item::Pad { net: moved_pad_net, .. } = board.node.get(a).unwrap() else { panic!("expected a pad") };
        assert_eq!(*moved_pad_net, Some(net), "the moved footprint's own pad must keep its net assignment");
        let placed = board.footprints.iter().find(|f| f.id == moving).unwrap();
        assert_eq!(placed.position, new_position, "the move itself must still have taken effect");
    }

    #[test]
    fn try_move_footprint_does_not_collide_with_a_same_net_solid_plane_it_is_already_sitting_under() {
        // Regression test: `check_placement`'s candidate pads used to
        // always carry `net: None` (straight from `world_items`, which
        // has no way to know a *placed* footprint's real, already-
        // assigned net -- see this fix's own comment above), so even a
        // pad on the exact same net as an existing full-board `Item::Zone`
        // pour would still register as a different-net clearance
        // violation against it -- freezing every routed footprint that
        // a solid ground/power plane happened to cover in place, unable
        // to move anywhere at all, not even back onto its own spot.
        let mut board = test_board();
        let template = two_pin_template();
        let moving = board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let net = board.connect_pads(a, b).expect("connecting two unassigned pads must succeed");
        // `two_pin_template` has a *second* pin too -- join it onto the
        // same net so the plane below covers the whole footprint, not
        // just one of its two pads (an actually-unconnected second pad
        // would then legitimately collide with a different-net plane,
        // which isn't the bug this test is about).
        board.connect_pads(pad_ids_of(&board, 0)[1], b).expect("extending the net to the footprint's other pad must succeed");

        // A solid plane covering the *entire* board on that same net --
        // exactly what the GUI's "Solid F.Cu/B.Cu plane" checkbox
        // produces.
        let board_outline = board.outline.clone();
        board.add_zone(board_outline[0].clone(), LayerId::FCu, net);
        assert!(board.node.iter().any(|item| matches!(item, Item::Zone { .. })), "the plane must have actually filled");

        let new_position = Point::new(-10 * MM, 15 * MM);
        board.try_move_footprint(moving, &template, new_position, 0.0).expect("moving a same-net pad under its own plane must succeed");
    }

    #[test]
    fn try_move_footprint_is_not_blocked_by_a_different_net_solid_plane_it_has_no_net_yet_to_match() {
        // A copper pour is never a placement/move obstacle (see
        // `check_placement`'s own comment on that filter) -- otherwise
        // a full-board plane on net A would permanently freeze every
        // footprint with even one still-unconnected (or different-net)
        // pad, i.e. in practice almost every real part, since there'd
        // be nowhere left on the whole board it could ever move to.
        let mut board = test_board();
        let template = two_pin_template();
        let moving = board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        let other = board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let net = board.connect_pads(pad_ids_of(&board, 1)[0], pad_ids_of(&board, 1)[1]).unwrap();
        let _ = other;

        let board_outline = board.outline.clone();
        board.add_zone(board_outline[0].clone(), LayerId::FCu, net);
        assert!(board.node.iter().any(|item| matches!(item, Item::Zone { .. })), "the plane must have actually filled");

        // `moving`'s own two pads are still net-less -- neither one
        // shares the plane's net.
        let new_position = Point::new(-10 * MM, 15 * MM);
        board.try_move_footprint(moving, &template, new_position, 0.0).expect("a copper pour of an unrelated net must never block a move");
    }

    #[test]
    fn refilling_a_zone_never_assigns_a_net_to_a_new_unconnected_footprint_sitting_under_it() {
        // Regression test for a reported bug: place a brand new part on
        // top of an already-poured plane, hit "Refill zones", and its
        // pads must stay exactly as net-less as they were the instant
        // before -- a `refill_zone`/`refill_all_zones` call only ever
        // reruns `zone_fill::fill_zone` (produces `Item::Zone` fill
        // islands) and never touches, reads back, or mutates any
        // `Item::Pad`'s own `net` field. Net assignment must only ever
        // happen through an explicit `Self::connect_pads` call -- never
        // as a side effect of geometry recomputation.
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        let net = board.connect_pads(pad_ids_of(&board, 0)[0], pad_ids_of(&board, 0)[1]).unwrap();

        let board_outline = board.outline.clone();
        board.add_zone(board_outline[0].clone(), LayerId::FCu, net);

        // The new, still-unconnected part, placed squarely on top of
        // the just-poured plane.
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let new_pads = pad_ids_of(&board, 1);
        for &pad in &new_pads {
            assert_eq!(board.pad_net(pad), Ok(None), "a freshly placed pad must start net-less");
        }

        board.refill_all_zones();

        for &pad in &new_pads {
            assert_eq!(board.pad_net(pad), Ok(None), "refilling zones must never wire an unrelated pad onto the plane's net");
        }
    }

    #[test]
    fn zones_are_stale_flags_a_fill_that_predates_a_later_footprint_move_and_clears_once_refilled() {
        let mut board = test_board();
        let template = two_pin_template();
        let moving = board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 0)[1];
        let net = board.connect_pads(a, b).unwrap();

        let board_outline = board.outline.clone();
        board.add_zone(board_outline[0].clone(), LayerId::FCu, net);
        assert!(!board.zones_are_stale(), "a zone must never be stale immediately after its own fill");

        board.try_move_footprint(moving, &template, Point::new(-10 * MM, 10 * MM), 0.0).expect("moving under its own same-net plane must succeed");
        assert!(board.zones_are_stale(), "the plane's fill still reflects the footprint's old position, not its new one");

        board.refill_all_zones();
        assert!(!board.zones_are_stale(), "refilling must catch every zone back up to the board's current state");
    }

    #[test]
    fn try_move_footprint_deletes_a_wire_touching_one_of_its_pads_so_none_are_left_stranded() {
        // Regression test: `Item::Track`/`Item::Via` store a fixed
        // `Point`, never a live reference to whichever pad they happen
        // to touch (see `alladin_core::Item`'s own doc comment) --
        // without `wires_touching_pads`, moving a routed footprint
        // would leave its track(s) behind at the *old* pad position,
        // still tagged with the pad's net but no longer touching
        // anything real: exactly the "sinnlose Netzverbindung" a move
        // must never leave behind.
        let mut board = test_board();
        let template = two_pin_template();
        let moving = board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let net = board.connect_pads(a, b).unwrap();
        let (a_center, b_center) = (board.pad_center(a).unwrap(), board.pad_center(b).unwrap());
        board.node.add(Item::Track { shape: Segment::new(a_center, b_center, 250_000), net: Some(net), layer: LayerId::FCu, class: NetClass::C });
        assert_eq!(board.node.iter().filter(|item| matches!(item, Item::Track { .. })).count(), 1);

        board.try_move_footprint(moving, &template, Point::new(-10 * MM, 15 * MM), 0.0).expect("open space must accept the move");

        assert_eq!(
            board.node.iter().filter(|item| matches!(item, Item::Track { .. })).count(),
            0,
            "the stale track must be deleted, not left stranded at the old pad position"
        );
        assert_eq!(board.pads_on_net(net).len(), 2, "the net's own pad membership must be unaffected by deleting the stale copper");
    }

    #[test]
    fn try_move_footprint_leaves_a_wire_touching_other_footprints_pads_alone() {
        let mut board = test_board();
        let template = two_pin_template();
        let moving = board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(0, 15 * MM), 0.0).unwrap();
        let b = pad_ids_of(&board, 1)[0];
        let c = pad_ids_of(&board, 2)[0];
        let unrelated_net = board.connect_pads(b, c).unwrap();
        let (b_center, c_center) = (board.pad_center(b).unwrap(), board.pad_center(c).unwrap());
        board.node.add(Item::Track { shape: Segment::new(b_center, c_center, 250_000), net: Some(unrelated_net), layer: LayerId::FCu, class: NetClass::C });

        board.try_move_footprint(moving, &template, Point::new(-10 * MM, -15 * MM), 0.0).expect("open space must accept the move");

        assert_eq!(
            board.node.iter().filter(|item| matches!(item, Item::Track { .. })).count(),
            1,
            "a wire that doesn't touch the moved footprint's own pads must survive the move"
        );
    }

    #[test]
    fn try_move_footprint_rejected_by_collision_leaves_its_wires_untouched() {
        let mut board = test_board();
        let template = two_pin_template();
        let moving = board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let net = board.connect_pads(a, a).unwrap(); // self-connect trick: gives `a` a net without needing a second real pad
        let a_center = board.pad_center(a).unwrap();
        board.node.add(Item::Track {
            shape: Segment::new(a_center, Point::new(-5 * MM, 5 * MM), 250_000),
            net: Some(net),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let err = board.try_move_footprint(moving, &template, Point::new(10 * MM, 0), 0.0).unwrap_err();

        assert!(matches!(err, PlacementError::Collision(_)));
        assert_eq!(
            board.node.iter().filter(|item| matches!(item, Item::Track { .. })).count(),
            1,
            "a rejected move must leave everything untouched, including any wire on the pad that was about to move"
        );
    }

    #[test]
    fn remove_footprint_deletes_a_wire_touching_one_of_its_pads_too() {
        let mut board = test_board();
        let template = two_pin_template();
        let removing = board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let net = board.connect_pads(a, b).unwrap();
        let (a_center, b_center) = (board.pad_center(a).unwrap(), board.pad_center(b).unwrap());
        board.node.add(Item::Track { shape: Segment::new(a_center, b_center, 250_000), net: Some(net), layer: LayerId::FCu, class: NetClass::C });

        board.remove_footprint(removing);

        assert_eq!(
            board.node.iter().filter(|item| matches!(item, Item::Track { .. })).count(),
            0,
            "the wire touching the removed footprint's own pad must be deleted too, not left as copper on a now-padless net"
        );
    }

    #[test]
    fn remove_footprint_deletes_its_pads_from_the_node() {
        let mut board = test_board();
        let template = two_pin_template();
        let id = board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        assert_eq!(board.node.iter().count(), template.pads.len());

        board.remove_footprint(id);
        assert_eq!(board.node.iter().count(), 0);
        assert!(board.footprints.is_empty());
    }

    fn pad_ids_of(board: &BoardDoc, footprint_index: usize) -> Vec<ItemId> {
        board.footprints[footprint_index].pad_item_ids.clone()
    }

    #[test]
    fn connect_pads_creates_a_new_net_when_neither_pad_has_one() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];

        let net = board.connect_pads(a, b).expect("connecting two unassigned pads must succeed");
        assert_eq!(board.nets.len(), 1);
        assert_eq!(board.pads_on_net(net).len(), 2);
    }

    #[test]
    fn connect_pads_extends_an_existing_net_instead_of_making_a_new_one() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(0, 15 * MM), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let c = pad_ids_of(&board, 2)[0];

        let net_ab = board.connect_pads(a, b).unwrap();
        let net_bc = board.connect_pads(b, c).unwrap();

        assert_eq!(net_ab, net_bc, "joining a third pad to an already-connected one must reuse the same net");
        assert_eq!(board.nets.len(), 1);
        assert_eq!(board.pads_on_net(net_ab).len(), 3);
    }

    #[test]
    fn connect_pads_refuses_to_merge_two_different_existing_nets() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(0, 15 * MM), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(0, -15 * MM), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let c = pad_ids_of(&board, 2)[0];
        let d = pad_ids_of(&board, 3)[0];
        board.connect_pads(a, b).unwrap();
        board.connect_pads(c, d).unwrap();

        let err = board.connect_pads(a, c).unwrap_err();
        assert_eq!(err, NetError::AlreadyOnDifferentNets);
        assert_eq!(board.nets.len(), 2, "a refused merge must leave both nets exactly as they were");
    }

    #[test]
    fn disconnect_pad_removes_the_net_once_it_has_no_pads_left() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let net = board.connect_pads(a, b).unwrap();

        board.disconnect_pad(a).unwrap();
        assert_eq!(board.pads_on_net(net).len(), 1, "the net itself must survive with its remaining pad");
        assert_eq!(board.nets.len(), 1);

        board.disconnect_pad(b).unwrap();
        assert!(board.nets.is_empty(), "a net with zero pads left must be pruned");
    }

    #[test]
    fn remove_net_disconnects_every_one_of_its_pads() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let net = board.connect_pads(a, b).unwrap();

        board.remove_net(net);
        assert!(board.nets.is_empty());
        assert_eq!(board.pad_net(a), Ok(None));
        assert_eq!(board.pad_net(b), Ok(None));
    }

    #[test]
    fn remove_net_also_deletes_every_track_and_via_still_on_it() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let net = board.connect_pads(a, b).unwrap();
        board.add_track_path(&[Point::new(-10 * MM, 0), Point::new(0, 0)], net, LayerId::FCu, 250_000, NetClass::C);
        board.try_add_via(Point::new(5 * MM, 0), net, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL).unwrap();
        let other_net_track_before = board.node.iter().filter(|i| matches!(i, Item::Track { .. })).count();
        assert!(other_net_track_before > 0, "test setup: the track must actually have been added");

        board.remove_net(net);

        assert!(board.nets.is_empty());
        assert!(!board.node.iter().any(|item| matches!(item, Item::Track { .. })), "every track on the deleted net must be gone too");
        assert!(!board.node.iter().any(|item| matches!(item, Item::Via { .. })), "every via on the deleted net must be gone too");
    }

    #[test]
    fn track_at_finds_a_track_near_its_centerline_but_not_far_from_it() {
        let mut board = test_board();
        board.node.add(Item::Track { shape: Segment::new(Point::new(0, 0), Point::new(10 * MM, 0), 250_000), net: None, layer: LayerId::FCu, class: NetClass::C });
        let id = board.node.iter_with_ids().next().unwrap().0;

        assert_eq!(board.track_at(Point::new(5 * MM, 0), 0), Some(id), "dead center of the track must hit");
        assert_eq!(board.track_at(Point::new(5 * MM, 100_000), 0), Some(id), "within the track's own half-width must still hit");
        assert_eq!(board.track_at(Point::new(5 * MM, MM), 0), None, "1mm off a 0.25mm-wide track must miss with no tolerance");
        assert_eq!(board.track_at(Point::new(5 * MM, MM), MM), Some(id), "but a generous tolerance must catch it");
    }

    #[test]
    fn via_at_finds_a_via_within_its_radius_plus_tolerance() {
        let mut board = test_board();
        board.node.add(Item::Via { shape: Circle::new(Point::new(0, 0), 300_000), drill: 300_000, net: None });
        let id = board.node.iter_with_ids().next().unwrap().0;

        assert_eq!(board.via_at(Point::new(0, 0), 0), Some(id));
        assert_eq!(board.via_at(Point::new(500_000, 0), 0), None, "outside the via's own radius must miss with no tolerance");
        assert_eq!(board.via_at(Point::new(500_000, 0), 300_000), Some(id), "but a generous tolerance must catch it");
    }

    #[test]
    fn remove_item_deletes_a_track_or_via_but_refuses_a_pad() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let pad = pad_ids_of(&board, 0)[0];
        board.node.add(Item::Track { shape: Segment::new(Point::new(0, 0), Point::new(10 * MM, 0), 250_000), net: None, layer: LayerId::FCu, class: NetClass::C });
        let track = board.node.iter_with_ids().find(|(_, i)| matches!(i, Item::Track { .. })).unwrap().0;

        assert!(!board.remove_item(pad), "a pad must not be deletable through this path");
        assert!(board.node.get(pad).is_some());

        assert!(board.remove_item(track));
        assert!(board.node.get(track).is_none());
    }

    #[test]
    fn connected_wire_gathers_every_leg_of_a_multi_corner_route() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let a_center = board.pad_center(a).unwrap();
        let b_center = board.pad_center(b).unwrap();
        let net = board.connect_pads(a, b).unwrap();
        // One route, but bent through two intermediate corners -- three
        // separate `Item::Track` legs, exactly like a real
        // `RoutingDrag::commit` with two fixed corners would add.
        board.add_track_path(
            &[a_center, Point::new(-5 * MM, 0), Point::new(-5 * MM, 5 * MM), Point::new(5 * MM, 5 * MM), b_center],
            net,
            LayerId::FCu,
            250_000,
            NetClass::C,
        );
        let track_ids: Vec<ItemId> = board.node.iter_with_ids().filter(|(_, i)| matches!(i, Item::Track { .. })).map(|(id, _)| id).collect();
        assert_eq!(track_ids.len(), 4, "test setup: four legs for the four-point path above");

        for &start in &track_ids {
            let wire = board.connected_wire(start);
            assert_eq!(wire.len(), 4, "every leg must gather all four, no matter which one the click landed on");
            for &id in &track_ids {
                assert!(wire.contains(&id));
            }
        }
    }

    #[test]
    fn connected_wire_does_not_pull_in_an_unrelated_wire_that_only_shares_a_pad() {
        let mut board = test_board();
        let template = crate::footprint::builtin_templates().remove(1); // 4-pin header, room for two separate nets
        board.try_place_footprint(&template, Point::new(-12 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(12 * MM, 0), 0.0).unwrap();
        let pads_left = pad_ids_of(&board, 0);
        let pads_right = pad_ids_of(&board, 1);
        let net_a = board.connect_pads(pads_left[0], pads_right[0]).unwrap();
        let net_b = board.connect_pads(pads_left[1], pads_right[1]).unwrap();
        board.add_track_path(&[board.pad_center(pads_left[0]).unwrap(), board.pad_center(pads_right[0]).unwrap()], net_a, LayerId::FCu, 250_000, NetClass::C);
        board.add_track_path(&[board.pad_center(pads_left[1]).unwrap(), board.pad_center(pads_right[1]).unwrap()], net_b, LayerId::FCu, 250_000, NetClass::C);

        let track_a = board.node.iter_with_ids().find(|(_, i)| matches!(i, Item::Track { net, .. } if *net == Some(net_a))).unwrap().0;
        let wire = board.connected_wire(track_a);
        assert_eq!(wire, vec![track_a], "a different net's wire, even on the same two footprints, must never be pulled in");
    }

    #[test]
    fn connected_wire_crosses_a_via_onto_the_other_layer() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let a_center = board.pad_center(a).unwrap();
        let b_center = board.pad_center(b).unwrap();
        let net = board.connect_pads(a, b).unwrap();
        let via_point = Point::new(0, 0);
        board.add_track_path(&[a_center, via_point], net, LayerId::FCu, 250_000, NetClass::C);
        board.try_add_via(via_point, net, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL).unwrap();
        board.add_track_path(&[via_point, b_center], net, LayerId::BCu, 250_000, NetClass::C);

        let leg_on_fcu = board.node.iter_with_ids().find(|(_, i)| matches!(i, Item::Track { layer: LayerId::FCu, .. })).unwrap().0;
        let wire = board.connected_wire(leg_on_fcu);
        assert_eq!(wire.len(), 3, "both legs plus the via bridging them must all be gathered");
        assert!(wire.iter().any(|&id| matches!(board.node.get(id), Some(Item::Via { .. }))));
        assert!(wire.iter().any(|&id| matches!(board.node.get(id), Some(Item::Track { layer: LayerId::BCu, .. }))));
    }

    #[test]
    fn remove_wire_deletes_every_leg_at_once_but_leaves_the_net_and_pads_intact() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let a_center = board.pad_center(a).unwrap();
        let b_center = board.pad_center(b).unwrap();
        let net = board.connect_pads(a, b).unwrap();
        board.add_track_path(&[a_center, Point::new(0, 5 * MM), b_center], net, LayerId::FCu, 250_000, NetClass::C);
        let first_leg = board.node.iter_with_ids().find(|(_, i)| matches!(i, Item::Track { .. })).unwrap().0;

        assert!(board.remove_wire(first_leg));

        assert!(!board.node.iter().any(|i| matches!(i, Item::Track { .. })), "every leg of the wire must be gone");
        assert_eq!(board.nets.len(), 1, "the net itself must survive");
        assert_eq!(board.pad_net(a), Ok(Some(net)), "pads must stay connected");
        assert_eq!(board.pad_net(b), Ok(Some(net)), "pads must stay connected");
    }

    #[test]
    fn remove_wire_refuses_for_a_pad() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let pad = pad_ids_of(&board, 0)[0];
        assert!(!board.remove_wire(pad));
        assert!(board.node.get(pad).is_some());
    }

    #[test]
    fn remove_footprint_prunes_a_net_left_with_no_pads() {
        // Both ends of the net must disappear together: connect the two
        // pads of the *same* footprint (a harmless, if unusual, net), so
        // removing that one footprint removes every pad the net has.
        let mut board = test_board();
        let template = two_pin_template();
        let id = board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let pads = pad_ids_of(&board, 0);
        board.connect_pads(pads[0], pads[1]).unwrap();
        assert_eq!(board.nets.len(), 1);

        board.remove_footprint(id);
        assert!(board.nets.is_empty(), "removing every pad a net has must prune that net too");
    }

    #[test]
    fn layer_count_from_str_round_trips_with_display() {
        for count in LayerCount::ALL {
            assert_eq!(count.to_string().parse::<LayerCount>(), Ok(count));
        }
    }

    #[test]
    fn layer_count_from_str_rejects_anything_else() {
        assert!("3".parse::<LayerCount>().is_err());
        assert!("".parse::<LayerCount>().is_err());
    }

    #[test]
    fn find_pad_resolves_a_reference_and_pad_number_to_the_right_item_id() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let expected = board.footprints[0].pad_item_ids[1];

        assert_eq!(board.find_pad(std::slice::from_ref(&template), "P1", "2"), Some(expected));
        assert_eq!(board.find_pad(std::slice::from_ref(&template), "P1", "no-such-pin"), None);
        assert_eq!(board.find_pad(&[template], "no-such-reference", "1"), None);
    }

    #[test]
    fn find_net_by_name_resolves_an_exact_match_and_misses_an_unknown_name() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(&board, 0)[0];
        let b = pad_ids_of(&board, 1)[0];
        let net = board.connect_pads(a, b).unwrap();

        assert_eq!(board.find_net_by_name("Net1"), Some(net));
        assert_eq!(board.find_net_by_name("no-such-net"), None);
    }

    #[test]
    fn rename_net_gives_an_auto_named_net_a_real_name_findable_by_that_new_name() {
        let mut board = test_board();
        let net = board.create_net();

        board.rename_net(net, "GND").unwrap();

        assert_eq!(board.nets.iter().find(|n| n.id == net).unwrap().name, "GND");
        assert_eq!(board.find_net_by_name("GND"), Some(net));
        assert_eq!(board.find_net_by_name("Net1"), None, "the old auto-generated name must no longer resolve");
    }

    #[test]
    fn rename_net_trims_surrounding_whitespace() {
        let mut board = test_board();
        let net = board.create_net();

        board.rename_net(net, "  5V  ").unwrap();

        assert_eq!(board.nets.iter().find(|n| n.id == net).unwrap().name, "5V");
    }

    #[test]
    fn rename_net_rejects_an_empty_or_all_whitespace_name() {
        let mut board = test_board();
        let net = board.create_net();

        assert_eq!(board.rename_net(net, ""), Err(RenameNetError::EmptyName));
        assert_eq!(board.rename_net(net, "   "), Err(RenameNetError::EmptyName));
    }

    #[test]
    fn rename_net_rejects_an_unknown_net_id() {
        let mut board = test_board();
        let bogus = NetId(999);
        assert_eq!(board.rename_net(bogus, "GND"), Err(RenameNetError::NotFound));
    }

    #[test]
    fn rename_net_rejects_a_name_already_used_by_a_different_net_but_allows_renaming_to_its_own_current_name() {
        let mut board = test_board();
        let gnd = board.create_net();
        board.rename_net(gnd, "GND").unwrap();
        let other = board.create_net();

        assert_eq!(board.rename_net(other, "GND"), Err(RenameNetError::NameAlreadyUsed));
        // Renaming a net to the exact name it already has is a harmless no-op, not a self-conflict.
        assert!(board.rename_net(gnd, "GND").is_ok());
    }

    #[test]
    fn footprint_at_hits_a_pad_and_misses_empty_space() {
        let mut board = test_board();
        let template = two_pin_template();
        let id = board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();

        let pad_position = match board.node.get(board.footprints[0].pad_item_ids[0]).unwrap() {
            Item::Pad { shape, .. } => shape.center(),
            _ => panic!("expected a pad"),
        };
        assert_eq!(board.footprint_at(pad_position), Some(id));
        assert_eq!(board.footprint_at(Point::new(19 * MM, 19 * MM)), None);
    }

    /// Regression test for a real bug: a pure mounting-hole footprint
    /// (`mounting_hole_template`, no pads at all -- see
    /// `try_place_footprint_splits_pads_and_holes_into_their_own_id_lists`)
    /// was permanently unclickable in the GUI, because `footprint_at`
    /// only ever hit-tested `pad_at`, and a hole-only footprint has no
    /// pad for that to find. Users could place one but never again
    /// select, move, or delete it by clicking on it.
    #[test]
    fn footprint_at_finds_a_pure_mounting_hole_footprint_by_its_hole_not_just_a_pad() {
        let mut board = test_board();
        let template = mounting_hole_template();
        let id = board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();

        let hole_position = match board.node.get(board.footprints[0].hole_item_ids[0]).unwrap() {
            Item::Hole { position, .. } => *position,
            _ => panic!("expected a hole"),
        };
        assert_eq!(board.footprint_at(hole_position), Some(id), "clicking dead center of the hole must select its footprint");
        assert_eq!(board.footprint_at(Point::new(19 * MM, 19 * MM)), None);
    }

    fn one_pad_rect_template(width: Unit, height: Unit, rotation_deg: f64) -> crate::footprint::FootprintTemplate {
        crate::footprint::FootprintTemplate {
            name: "rect-test".to_string(),
            reference_prefix: "P".to_string(),
            pads: vec![crate::footprint::PadTemplate {
                offset: Point::new(0, 0),
                radius: width.min(height) / 2,
                layer: LayerId::FCu,
                number: "1".to_string(),
                shape: crate::footprint::PadShapeKind::Rect { width, height },
                rotation_deg,
                hole_diameter: None,
                pin_name: None,
            }],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: None,
        }
    }

    #[test]
    fn pad_at_hits_a_rotated_rectangular_pads_true_corner_not_just_its_bounding_circle() {
        // A 4mm x 1mm pad rotated 45 degrees: its bounding circle (radius
        // ~2.06mm) covers plenty of empty space a true point-in-polygon
        // hit-test must reject.
        let mut board = test_board();
        let template = one_pad_rect_template(4 * MM, MM, 45.0);
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();

        // Just off the pad's own long axis (rotated 45 degrees), well
        // inside the true rectangle.
        let on_axis = Point::new(mm_to_unit(1.0), mm_to_unit(1.0));
        assert_eq!(board.pad_at(on_axis), Some(board.footprints[0].pad_item_ids[0]), "must hit real copper on the rotated pad's long axis");

        // Same distance from the center, but off-axis (e.g. straight
        // along unrotated +X) -- inside the old bounding circle, but
        // outside the true rotated rectangle.
        let off_axis = Point::new(mm_to_unit(1.9), 0);
        assert_eq!(board.pad_at(off_axis), None, "must miss the bounding circle's corner that isn't real copper");
    }

    /// The rotated pad's true rightmost extent along X (its AABB
    /// half-width, not the corner-to-corner half-diagonal, since a
    /// rotated corner is generally not the point furthest along a given
    /// axis) -- computed straight from the real, already-tested
    /// [`crate::footprint::world_items`] outline rather than
    /// re-deriving the rotation trig by hand in the test itself.
    fn rightmost_extent_of_a_single_rect_pad(width: Unit, height: Unit, rotation_deg: f64) -> Unit {
        let template = one_pad_rect_template(width, height, rotation_deg);
        let items = crate::footprint::world_items(&template, Point::new(0, 0), 0.0);
        let Item::Pad { shape: PadShape::Polygon { outline, .. }, .. } = &items[0] else { panic!("expected a polygon pad") };
        outline.points.iter().map(|p| p.x).max().unwrap()
    }

    #[test]
    fn check_placement_rejects_a_rotated_rect_pads_true_corner_too_close_to_the_board_edge() {
        // 40mm board -> outline at x = +20mm. A 4mm x 1mm pad rotated 45
        // degrees has a true rightmost extent well past its own radius
        // (0.5mm) -- placed so that extent sits inside JLCPCB's 0.20mm
        // minimum, even though the pad's *short*-axis bounding circle
        // alone wouldn't reach that close.
        let mut board = test_board();
        let (width, height, rotation_deg) = (4 * MM, MM, 45.0);
        let extent = rightmost_extent_of_a_single_rect_pad(width, height, rotation_deg);
        let template = one_pad_rect_template(width, height, rotation_deg);
        let gap_from_edge = 50_000; // 0.05mm -- less than the 0.20mm minimum
        let position = Point::new(20 * MM - gap_from_edge - extent, 0);

        let err = board.try_place_footprint(&template, position, 0.0).unwrap_err();
        assert_eq!(err, PlacementError::OffBoard);
    }

    #[test]
    fn check_placement_accepts_a_rotated_rect_pad_that_clears_the_edge_by_the_full_dfm_minimum() {
        // 3mm gap -- comfortably past both the 0.20mm copper `OffBoard`
        // minimum this test originally targeted *and* the stricter
        // 2.5mm body `BodyOffBoard` one (`JlcpcbDfm::COMPONENT_BODY_TO_EDGE`):
        // this template's only pad *is* its fallback courtyard, so the
        // same rotated-rect geometry is subject to both checks at once.
        let mut board = test_board();
        let (width, height, rotation_deg) = (4 * MM, MM, 45.0);
        let extent = rightmost_extent_of_a_single_rect_pad(width, height, rotation_deg);
        let template = one_pad_rect_template(width, height, rotation_deg);
        let gap_from_edge = 3 * MM;
        let position = Point::new(20 * MM - gap_from_edge - extent, 0);

        board.try_place_footprint(&template, position, 0.0).expect("a well-cleared rotated rect pad must be accepted");
    }

    fn net_for_a_connected_pair(board: &mut BoardDoc) -> NetId {
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let a = pad_ids_of(board, 0)[0];
        let b = pad_ids_of(board, 1)[0];
        board.connect_pads(a, b).unwrap()
    }

    #[test]
    fn try_place_footprint_accepts_a_template_with_a_sub_minimum_smd_pad() {
        // Fine-pitch QFN pads sit under JLCPCB's 0.25mm "good" floor but
        // must still place -- fine-pitch pads are report-only warnings;
        // hard-refusing would block every real MCU footprint (see
        // `template_dfm_hard_violations`).
        let mut board = test_board();
        let mut template = two_pin_template();
        template.pads[0].radius = 100_000; // 0.2mm diameter, under the 0.25mm SMD floor
        template.pads[0].shape = crate::footprint::PadShapeKind::Circle;
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).expect("a sub-minimum SMD pad must not hard-block placement");
        assert_eq!(board.footprints.len(), 1);
        assert!(
            !crate::footprint::template_dfm_violations(&template.pads, &[]).is_empty(),
            "the same geometry must still show up as a report-only DFM finding"
        );
    }

    #[test]
    fn try_add_via_enforces_jlcpcbs_hole_to_hole_spacing_and_relaxes_it_for_a_shared_net() {
        let mut board = test_board();
        let net_a = net_for_a_connected_pair(&mut board);
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 12 * MM), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 12 * MM), 0.0).unwrap();
        let c = pad_ids_of(&board, 2)[0];
        let d = pad_ids_of(&board, 3)[0];
        let net_b = board.connect_pads(c, d).unwrap();

        // Two 0.6/0.3 vias, centres 0.75mm apart: their *copper* gap is
        // 0.15mm -- clean under the pairwise resolver -- but their drill
        // walls sit only 0.45mm apart, under JLCPCB's 0.5mm
        // different-net hole-to-hole rule. Exactly the class of refusal
        // only a drill-aware check can produce.
        board.try_add_via(Point::new(0, -12 * MM), net_a, 600_000, 300_000).unwrap();
        let err = board.try_add_via(Point::new(750_000, -12 * MM), net_b, 600_000, 300_000).unwrap_err();
        assert_eq!(err, PlacementError::Dfm(DfmViolation::HoleToHoleBelowMin));

        // The identical geometry on the *same* net falls under the
        // relaxed 0.254mm rule instead -- a legal stitching pair.
        board.try_add_via(Point::new(750_000, -12 * MM), net_a, 600_000, 300_000).expect("a same-net via pair with a 0.45mm wall gap must stay legal");
    }

    #[test]
    fn try_place_footprint_refuses_a_mounting_hole_whose_drill_crowds_an_existing_via() {
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);
        let mounting = crate::footprint::builtin_templates().into_iter().find(|t| !t.holes.is_empty()).expect("a mounting-hole builtin must exist");
        let drill = mounting.holes[0].drill;

        let via_center = Point::new(0, -12 * MM);
        board.try_add_via(via_center, net, 600_000, 300_000).unwrap();
        // Wall gap 0.35mm (under the 0.5mm different-net rule), while
        // the copper edges still clear each other comfortably.
        let distance = drill / 2 + 150_000 + 350_000;
        let err = board.try_place_footprint(&mounting, Point::new(via_center.x + distance, via_center.y), 0.0).unwrap_err();
        assert_eq!(err, PlacementError::Dfm(DfmViolation::HoleToHoleBelowMin));

        // The same hole a full 0.5mm away must place cleanly.
        let distance = drill / 2 + 150_000 + 500_000;
        board.try_place_footprint(&mounting, Point::new(via_center.x + distance, via_center.y), 0.0).expect("a hole at exactly the 0.5mm wall gap must be legal");
    }

    #[test]
    fn try_add_via_rejects_a_via_whose_annular_ring_is_below_jlcpcb_minimum() {
        // Diameter and drill each sit on their own JLCPCB floor, but the
        // resulting ring is only 0.05 mm -- half of MIN_VIA_ANNULAR_RING.
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);
        let before = board.node.iter().count();
        let err = board
            .try_add_via(Point::new(0, 5 * MM), net, JlcpcbDfm::MIN_VIA_DIAMETER, JlcpcbDfm::MIN_VIA_HOLE)
            .expect_err("sub-minimum annular ring must be refused");
        assert_eq!(err, PlacementError::Dfm(DfmViolation::ViaAnnularRingBelowMin));
        assert_eq!(board.node.iter().count(), before, "a refused via must not be added");
    }

    #[test]
    fn has_identical_routed_item_matches_a_track_in_either_endpoint_order_but_never_a_near_miss() {
        // Exact copy -- even with swapped endpoints, same capsule --
        // must be recognised, while any real difference (width, layer,
        // net) must NOT be.
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);
        let (a, b) = (Point::new(-5 * MM, 0), Point::new(5 * MM, 0));
        board.add_track_path(&[a, b], net, LayerId::FCu, 250_000, NetClass::C);

        let same = |a, b, width, net, layer| Item::Track { shape: Segment::new(a, b, width), net, layer, class: NetClass::C };
        assert!(board.has_identical_routed_item(&same(a, b, 250_000, Some(net), LayerId::FCu)));
        assert!(board.has_identical_routed_item(&same(b, a, 250_000, Some(net), LayerId::FCu)), "swapped endpoints are the same capsule");
        assert!(!board.has_identical_routed_item(&same(a, b, 300_000, Some(net), LayerId::FCu)), "a different width is different copper");
        assert!(!board.has_identical_routed_item(&same(a, b, 250_000, Some(net), LayerId::BCu)), "a different layer is different copper");
        assert!(!board.has_identical_routed_item(&same(a, b, 250_000, None, LayerId::FCu)), "a different net is different copper");
    }

    #[test]
    fn has_identical_routed_item_matches_a_via_only_on_exact_center_size_drill_and_net() {
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);
        let center = Point::new(0, 5 * MM);
        board.try_add_via(center, net, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL).unwrap();

        let via = |center, diameter: Unit, drill, net| Item::Via { shape: Circle::new(center, diameter / 2), drill, net };
        assert!(board.has_identical_routed_item(&via(center, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL, Some(net))));
        assert!(!board.has_identical_routed_item(&via(Point::new(MM, 5 * MM), DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL, Some(net))), "a different center is a different via");
        assert!(!board.has_identical_routed_item(&via(center, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL + 50_000, Some(net))), "a different drill is a different via");
        assert!(!board.has_identical_routed_item(&via(center, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL, None)), "a different net is a different via");
    }

    #[test]
    fn try_add_via_succeeds_in_open_space_and_adds_it_to_the_node() {
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);

        let id = board.try_add_via(Point::new(0, 5 * MM), net, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL).expect("open space must accept a via");

        match board.node.get(id) {
            Some(Item::Via { shape, drill, net: via_net }) => {
                assert_eq!(shape.center, Point::new(0, 5 * MM));
                assert_eq!(shape.radius, DEFAULT_VIA_DIAMETER / 2);
                assert_eq!(*drill, DEFAULT_VIA_DRILL);
                assert_eq!(*via_net, Some(net));
            }
            other => panic!("expected a via, got {other:?}"),
        }
    }

    #[test]
    fn try_add_via_rejects_a_collision_with_a_different_nets_pad_and_adds_nothing() {
        // Same-net items never collide (see `Node::query_colliding`'s own
        // doc comment), so the pad this via must be rejected against has
        // to belong to a *different* net (here: no net at all) -- placing
        // it on the connected pair's own net would legitimately be a
        // no-op collision-wise, not the case this test means to cover.
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 15 * MM), 0.0).unwrap();
        let unconnected_pad_center = match board.node.get(pad_ids_of(&board, 2)[0]).unwrap() {
            Item::Pad { shape, .. } => shape.center(),
            _ => panic!("expected a pad"),
        };
        let before = board.node.iter().count();

        let err = board.try_add_via(unconnected_pad_center, net, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL).unwrap_err();
        assert!(matches!(err, PlacementError::Collision(_)));
        assert_eq!(board.node.iter().count(), before, "a rejected via must not be added");
    }

    #[test]
    fn try_add_via_rejects_a_center_too_close_to_the_board_edge() {
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);
        let before = board.node.iter().count();
        // 40mm board -> outline at x = +20mm. Placed so the via's own
        // outer copper edge sits only 0.05mm from the board edge --
        // comfortably on-board (would pass a bare on-board check) but
        // well inside JLCPCB's 0.20mm `copper_to_routed_edge` minimum.
        let gap_from_edge = 50_000; // 0.05mm
        let center = Point::new(20 * MM - gap_from_edge - DEFAULT_VIA_DIAMETER / 2, 15 * MM);

        let err = board.try_add_via(center, net, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL).unwrap_err();
        assert_eq!(err, PlacementError::OffBoard);
        assert_eq!(board.node.iter().count(), before, "a rejected via must not be added");
    }

    #[test]
    fn try_add_stitching_via_succeeds_when_it_overlaps_a_same_net_pad() {
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);
        let pad_a_center = match board.node.get(pad_ids_of(&board, 0)[0]).unwrap() {
            Item::Pad { shape, .. } => shape.center(),
            _ => panic!("expected a pad"),
        };

        let id = board
            .try_add_stitching_via(pad_a_center, net, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL)
            .expect("a via placed right on top of its own net's pad must be accepted, not just geometrically legal");
        assert!(board.node.get(id).is_some());
    }

    #[test]
    fn try_add_stitching_via_rejects_a_dangling_via_and_rolls_it_back() {
        // Same net as `try_add_stitching_via_succeeds_when_it_overlaps_a_same_net_pad`,
        // but far from both of that net's own pads (at x = +-10mm, y=0) --
        // geometrically legal on its own (no collision, well on-board)
        // but touches nothing on this net, so it would be an
        // electrically pointless, dangling via -- exactly the case
        // `try_add_via` itself (deliberately) can't catch.
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);
        let before = board.node.iter().count();

        let err = board.try_add_stitching_via(Point::new(0, 15 * MM), net, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL).unwrap_err();
        assert_eq!(err, ViaError::Dangling);
        assert_eq!(board.node.iter().count(), before, "a rejected dangling via must be rolled back, not left on the board");
    }

    #[test]
    fn try_add_stitching_via_still_reports_a_plain_placement_error_first() {
        // The ordinary geometric refusal (too close to the board edge,
        // same fixture as `try_add_via_rejects_a_center_too_close_to_the_board_edge`)
        // must still surface as `ViaError::Placement`, not get
        // reinterpreted as `Dangling` -- `try_add_via` itself is never
        // even reached far enough to add anything for `touches_same_net`
        // to check in the first place.
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);
        let before = board.node.iter().count();
        let gap_from_edge = 50_000; // 0.05mm, see the `try_add_via` sibling test
        let center = Point::new(20 * MM - gap_from_edge - DEFAULT_VIA_DIAMETER / 2, 15 * MM);

        let err = board.try_add_stitching_via(center, net, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL).unwrap_err();
        assert_eq!(err, ViaError::Placement(PlacementError::OffBoard));
        assert_eq!(board.node.iter().count(), before, "a rejected via must not be added");
    }

    #[test]
    fn try_add_pin_stitching_via_places_the_via_radially_away_from_the_footprints_own_body_and_connects_it_with_a_stub() {
        // `two_pin_template()`'s pad "1" sits at local x = -1.27mm, and
        // this symmetric two-pad footprint's own fallback courtyard is
        // centered on its `position` -- so the radial-outward direction
        // for this pin must point further in -x, away from the part,
        // never back across it.
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let pad_id = pad_ids_of(&board, 0)[0];
        board.connect_pads(pad_id, pad_ids_of(&board, 1)[0]).unwrap();

        let items_before = board.node.iter().count();
        let pad_center = board.pad_center(pad_id).unwrap();

        let result = board
            .try_add_pin_stitching_via(pad_id, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL, crate::routing::DEFAULT_TRACE_WIDTH)
            .expect("open space next to the pin must accept a stitching via");

        assert!(result.center.x < pad_center.x, "the via must land further from the part than the pin itself, not on top of it or back across the body");
        assert_eq!(result.center.y, pad_center.y);
        assert!(matches!(board.node.get(result.via_id), Some(Item::Via { .. })), "the via itself must actually be live");
        assert_eq!(board.node.iter().count(), items_before + 2, "exactly one new via and one new stub track leg must be added");
        assert!(
            board.node.iter().any(|item| matches!(item, Item::Track { shape, .. } if shape.a == pad_center && shape.b == result.center)),
            "a stub track from the pin straight to the new via must have been committed"
        );
    }

    fn wire_pad_template() -> crate::footprint::FootprintTemplate {
        crate::footprint::builtin_templates().into_iter().find(|t| t.name == "Wire pad (solder, 2mm)").unwrap()
    }

    #[test]
    fn try_add_pin_stitching_via_falls_back_to_a_fixed_direction_when_the_pin_sits_on_the_footprints_own_center() {
        // `wire_pad_template()` has exactly one pad, at the footprint's
        // own local origin -- the one case where "radially away from
        // the body" is undefined (zero-length direction). Must still
        // succeed, picking the documented +X fallback, rather than
        // panicking on a division by zero or refusing outright.
        let mut board = test_board();
        let template = wire_pad_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let pad_id = pad_ids_of(&board, 0)[0];
        board.connect_pads(pad_id, pad_ids_of(&board, 1)[0]).unwrap();

        let result = board.try_add_pin_stitching_via(pad_id, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL, crate::routing::DEFAULT_TRACE_WIDTH).expect("the fixed fallback direction must still be a legal placement");

        assert!(result.center.x > 0, "the documented +X fallback must have been used");
        assert_eq!(result.center.y, 0);
    }

    #[test]
    fn try_add_pin_stitching_via_rejects_a_non_pad_item_id() {
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);
        let via_id = board.try_add_via(Point::new(0, 5 * MM), net, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL).unwrap();

        let err = board.try_add_pin_stitching_via(via_id, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL, crate::routing::DEFAULT_TRACE_WIDTH).unwrap_err();
        assert_eq!(err, PinStitchingViaError::NotAPad);
    }

    #[test]
    fn try_add_pin_stitching_via_rejects_a_pad_with_no_net_yet() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let pad_id = pad_ids_of(&board, 0)[0];
        let before = board.node.iter().count();

        let err = board.try_add_pin_stitching_via(pad_id, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL, crate::routing::DEFAULT_TRACE_WIDTH).unwrap_err();
        assert_eq!(err, PinStitchingViaError::NoNet);
        assert_eq!(board.node.iter().count(), before, "a refused pin must not have anything added for it");
    }

    #[test]
    fn try_add_pin_stitching_via_rolls_the_via_back_when_only_the_stub_track_has_no_room() {
        // A bare pad with no owning footprint (fallback direction: +X,
        // see the fixed-direction test above) with a JLCPCB-legal via
        // right next to it, and a wide stub. An obstacle on a different
        // net, placed close enough to block only the *wide* stub's own
        // clearance corridor but far enough from the via's own much
        // smaller footprint to still let the via placement itself
        // succeed cleanly -- exactly the split this error variant
        // exists for. (Via sizes must clear `JlcpcbDfm::check_via` --
        // a sub-minimum pair would now be refused as `Via(Dfm(...))`
        // before the stub is ever tried.)
        let mut board = test_board();
        let net_a = board.create_net();
        let net_b = board.create_net();
        let pad_radius = mm_to_unit(0.1);
        let pad_id = board.node.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), pad_radius)), net: Some(net_a), layer: LayerId::FCu });
        // Smallest JLCPCB-legal via (0.35/0.15 -- annular ring exactly
        // at MIN_VIA_ANNULAR_RING once drill is at MIN_VIA_HOLE).
        let via_diameter = mm_to_unit(0.35);
        let via_drill = mm_to_unit(0.15);
        let wide_stub_width = mm_to_unit(2.0);
        // Just past the via, clear of it (via radius 0.175 + VIA_TO_TRACK
        // 0.20 = 0.375 exclusion; obstacle at y=0.45 clears that), but
        // well within the 2mm-wide stub's own much larger corridor.
        board.node.add(Item::Track {
            shape: Segment::new(Point::new(mm_to_unit(0.3), mm_to_unit(0.45)), Point::new(mm_to_unit(0.5), mm_to_unit(0.45)), mm_to_unit(0.1)),
            net: Some(net_b),
            layer: LayerId::FCu,
            class: NetClass::C,
        });
        let items_before = board.node.iter().count();

        let err = board.try_add_pin_stitching_via(pad_id, via_diameter, via_drill, wide_stub_width).unwrap_err();
        assert_eq!(err, PinStitchingViaError::NoRoomForStub);
        assert_eq!(board.node.iter().count(), items_before, "the via must be rolled straight back out again, not left dangling on the board");
    }

    #[test]
    fn try_add_pin_stitching_via_sweeps_to_a_nearby_angle_when_the_natural_spot_is_occupied() {
        // A bare pad (fallback direction: +X, see the fixed-direction
        // test above) whose one "natural" via spot is occupied by an
        // unrelated via on a different net. Before the angular sweep
        // existed this was a flat refusal; now that exact spot is
        // still tried first, but a small deviation around the pad --
        // at the *same* distance from it -- finds a free spot instead.
        let mut board = test_board();
        let net_a = board.create_net();
        let net_b = board.create_net();
        let pad_radius = mm_to_unit(0.1);
        let pad_id = board.node.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), pad_radius)), net: Some(net_a), layer: LayerId::FCu });

        let ideal = board.pin_stitching_via_candidates(pad_id, DEFAULT_VIA_DIAMETER)[0];
        // A tiny via, but sitting exactly on the one point being
        // tested -- guaranteed to block that single spot without
        // reaching far enough around the circle to block every
        // fallback angle too (see the full-sweep-blocked test below
        // for that case).
        board.node.add(Item::Via { shape: Circle::new(ideal, mm_to_unit(0.05)), drill: 0, net: Some(net_b) });
        let items_before = board.node.iter().count();

        let result = board
            .try_add_pin_stitching_via(pad_id, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL, crate::routing::DEFAULT_TRACE_WIDTH)
            .expect("a small angular deviation must find a free spot next to the blocked one");

        assert_ne!(result.center, ideal, "must not have reused the blocked, already-occupied spot");
        let ideal_distance = (ideal.x as f64).hypot(ideal.y as f64);
        let actual_distance = (result.center.x as f64).hypot(result.center.y as f64);
        assert!(
            (actual_distance - ideal_distance).abs() < 100.0,
            "the distance to the pin must stay the same however far the angle had to move to dodge the obstacle: ideal={ideal_distance}, actual={actual_distance}"
        );
        assert_eq!(
            board.node.iter().count(),
            items_before + 2,
            "exactly one new via and one new stub track, and nothing left over from the earlier, rejected angles that were tried first"
        );
    }

    #[test]
    fn try_add_pin_stitching_via_reports_the_natural_points_own_error_when_the_whole_sweep_is_blocked() {
        // One big obstacle centered exactly on the pad itself reaches
        // every point on the sweep's candidate circle equally -- they
        // all sit at the very same fixed distance from the pad, by
        // construction -- so the entire +/-90 degree sweep is blocked
        // at once. Even though the *last* candidate actually tried is
        // +/-90 degrees away, not the natural, 0-degree one, the error
        // reported back must still be the natural point's own: the one
        // reason a human asking "why did this fail" actually wants.
        let mut board = test_board();
        let net_a = board.create_net();
        let net_b = board.create_net();
        let pad_radius = mm_to_unit(0.1);
        let pad_id = board.node.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), pad_radius)), net: Some(net_a), layer: LayerId::FCu });
        board.node.add(Item::Via { shape: Circle::new(Point::new(0, 0), mm_to_unit(2.0)), drill: 0, net: Some(net_b) });
        let items_before = board.node.iter().count();

        let err = board.try_add_pin_stitching_via(pad_id, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL, crate::routing::DEFAULT_TRACE_WIDTH).unwrap_err();

        assert!(matches!(err, PinStitchingViaError::Via(_)), "expected every candidate to fail as a plain via placement collision, got {err:?}");
        assert_eq!(
            board.node.iter().count(),
            items_before,
            "every rejected attempt across the whole sweep must roll its own via straight back out, leaving nothing dangling"
        );
    }

    fn chamfered_outline(width: Unit, height: Unit, chamfer: Unit) -> Polygon {
        let (hw, hh) = (width / 2, height / 2);
        Polygon::new(vec![
            Point::new(-hw + chamfer, -hh),
            Point::new(hw, -hh),
            Point::new(hw, hh - chamfer),
            Point::new(hw - chamfer, hh),
            Point::new(-hw, hh),
            Point::new(-hw, -hh + chamfer),
        ])
    }

    #[test]
    fn set_outline_accepts_an_arbitrary_polygon_that_still_clears_every_existing_item() {
        let mut board = test_board(); // 40x40mm rounded rect
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();

        let new_outline = vec![chamfered_outline(40 * MM, 40 * MM, 5 * MM)];
        board.set_outline(new_outline.clone(), &[template]).expect("a well-clear chamfered outline must be accepted");
        assert_eq!(board.outline, new_outline);
    }

    #[test]
    fn set_outline_rejects_a_shape_that_would_leave_an_existing_footprint_off_board_and_touches_nothing() {
        let mut board = test_board();
        let template = two_pin_template();
        board.try_place_footprint(&template, Point::new(15 * MM, 0), 0.0).unwrap();
        let original_outline = board.outline.clone();

        // A much smaller board -- the already-placed part no longer fits.
        let tiny_outline = vec![Polygon::rounded_rect(4 * MM, 4 * MM, 0, 8)];
        let err = board.set_outline(tiny_outline, &[template]).unwrap_err();
        assert!(matches!(err, SetOutlineError::FootprintOffBoard(_)));
        assert_eq!(board.outline, original_outline, "a rejected set_outline must leave the outline untouched");
    }

    #[test]
    fn set_outline_rejects_a_shape_that_would_leave_an_existing_track_off_board() {
        let mut board = test_board();
        let net = net_for_a_connected_pair(&mut board);
        board.add_track_path(&[Point::new(-10 * MM, 0), Point::new(10 * MM, 0)], net, LayerId::FCu, 250_000, NetClass::C);
        let original_outline = board.outline.clone();

        let tiny_outline = vec![Polygon::rounded_rect(4 * MM, 4 * MM, 0, 8)];
        let err = board.set_outline(tiny_outline, &[]).unwrap_err();
        assert_eq!(err, SetOutlineError::TrackOffBoard);
        assert_eq!(board.outline, original_outline);
    }

    #[test]
    fn set_outline_rejects_a_shape_that_would_leave_an_existing_mounting_hole_off_board() {
        let mut board = test_board(); // 40x40mm
        let template = mounting_hole_template();
        board.try_place_footprint(&template, Point::new(15 * MM, 0), 0.0).unwrap();
        let original_outline = board.outline.clone();

        // A much smaller board -- the already-placed hole no longer fits.
        let tiny_outline = vec![Polygon::rounded_rect(4 * MM, 4 * MM, 0, 8)];
        let err = board.set_outline(tiny_outline, &[template]).unwrap_err();
        assert!(matches!(err, SetOutlineError::FootprintOffBoard(_)));
        assert_eq!(board.outline, original_outline, "a rejected set_outline must leave the outline untouched");
    }

    #[test]
    fn set_outline_reclips_an_existing_zone_to_the_smaller_shape() {
        let mut board = test_board(); // 40x40mm
        let net = board.create_net();
        board.add_zone(chamfered_outline(30 * MM, 30 * MM, 2 * MM), LayerId::FCu, net);
        assert!(!board.zones[0].item_ids.is_empty(), "test setup: the zone must have filled to something on the full board");

        let smaller_outline = vec![Polygon::rounded_rect(10 * MM, 10 * MM, 0, 8)];
        board.set_outline(smaller_outline, &[]).expect("shrinking the board must not itself be rejected because of a zone");

        // A tiny slack for the clip's own boundary-edge float rounding --
        // this is checking "did the refill actually re-clip to the
        // smaller board", not exact geometry, so a few nanometres of
        // margin is harmless.
        let half_extent = 5 * MM + 1_000;
        for &item_id in &board.zones[0].item_ids {
            let Some(Item::Zone { outline, .. }) = board.node.get(item_id) else { panic!("expected a zone island") };
            for &p in &outline.points {
                assert!(p.x.abs() <= half_extent && p.y.abs() <= half_extent, "every refilled zone vertex must now lie within the smaller 10x10mm board, got {p:?}");
            }
        }
    }

    #[test]
    fn remove_zone_deletes_its_fill_islands_and_forgets_the_record() {
        let mut board = test_board(); // 40x40mm
        let net = board.create_net();
        let id = board.add_zone(chamfered_outline(30 * MM, 30 * MM, 2 * MM), LayerId::FCu, net);
        let island_count = board.zones[0].item_ids.len();
        assert!(island_count > 0, "test setup: an obstacle-free pour must fill to at least one island");
        let node_count_before = board.node.iter().count();

        board.remove_zone(id);

        assert!(board.zones.is_empty(), "the ZoneRecord itself must be gone, not just re-filled empty");
        assert_eq!(board.node.iter().count(), node_count_before - island_count, "every one of the zone's fill islands must be gone from the node too");
    }

    #[test]
    fn remove_zone_is_a_no_op_for_an_id_that_is_no_longer_recorded() {
        let mut board = test_board();
        let net = board.create_net();
        let id = board.add_zone(chamfered_outline(30 * MM, 30 * MM, 2 * MM), LayerId::FCu, net);
        board.remove_zone(id);
        let node_count = board.node.iter().count();

        board.remove_zone(id); // already gone -- must not panic or touch anything
        assert_eq!(board.node.iter().count(), node_count);
        assert!(board.zones.is_empty());
    }
}
