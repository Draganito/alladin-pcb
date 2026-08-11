//! Alladin core: the "world" a route lives in.
//!
//! This module is the Rust equivalent of the algorithm shape reverse-
//! engineered from KiCad's `pcbnew/router/pns_node.{h,cpp}` and
//! `pns_item.h` -- **the algorithm's shape, not its code**. Concretely:
//!
//! | KiCad PNS concept          | Alladin equivalent          |
//! |-----------------------------|------------------------------|
//! | `PNS::NODE` (R-tree world) | [`Node`] (rstar R-tree)      |
//! | `PNS::ITEM` / `SOLID`/`SEGMENT`/`VIA` | [`Item`]           |
//! | `PNS::RULE_RESOLVER`        | [`RuleResolver`] trait       |
//! | `PNS::NET_HANDLE`           | [`NetId`]                    |
//! | Net-priority classes (A/B/C from the original architecture note) | [`NetClass`] |
//!
//! Deliberately **not** reused: KiCad's `BOARD`/`PAD`/`PCB_TRACK` classes
//! (structurally tied to `KIGFX::VIEW_ITEM` + `wxPropertyGrid`).
//! Alladin's `Item` is a plain data enum with zero rendering /
//! property-panel baggage.

use alladin_geom::{
    circle_circle_collides, circle_polygon_collides, circle_polygon_collides_indexed,
    circle_segment_collides, polygon_polygon_collides, polygon_polygon_collides_indexed,
    segment_polygon_collides, segment_polygon_collides_indexed, segment_segment_collides, Aabb,
    Circle, Point, PolygonEdgeIndex, Polygon, Segment, Unit,
};
use rstar::{RTree, RTreeObject, AABB};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// How a pad joins a same-net copper pour ([`Item::Zone`]).
///
/// Stored on every [`Item::Pad`] (and on the footprint template / parts
/// DB) so zone fill and placement keepout share one source of truth:
/// [`ZoneConnection::Thermal`] punches a gap and restores thin spokes;
/// [`ZoneConnection::Solid`] leaves the pad fully flooded (exposed pads,
/// large power pads).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ZoneConnection {
    /// Annular clearance + spokes into the pour (default for ordinary pads).
    #[default]
    Thermal,
    /// Full copper flood — no thermal gap (EP / large power pads).
    Solid,
}

/// Geometry constants shared by zone-fill thermals and placement keepout.
/// Values sit in the same band as JLCPCB pad/track clearances so a
/// thermal ring is fab-legal and still large enough for reflow.
pub mod thermal {
    use alladin_geom::Unit;

    /// Clearance ring between pad copper and pour copper (0.20 mm).
    pub const GAP: Unit = 200_000;
    /// Width of each of the four thermal spokes (0.20 mm).
    pub const SPOKE_WIDTH: Unit = 200_000;
    /// Longest pad side at or above this → heuristic picks [`super::ZoneConnection::Solid`] (2.0 mm).
    pub const SOLID_MIN_SIDE: Unit = 2_000_000;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NetId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ItemId(pub usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LayerId {
    FCu,
    BCu,
}

/// Strategic net-priority classes from the Alladin architecture note
/// (2026-07-29 concept): routing order and "how frozen" an item is once
/// placed.
///
/// `PartialOrd`/`Ord` follow declaration order (`A < B < C`), matching
/// routing priority -- class-`A` nets get first claim on open space,
/// exactly as the doc comments below describe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum NetClass {
    /// USB data pairs, crystal/oscillator lines: routed first, frozen
    /// immediately (hard island as soon as placed).
    A,
    /// VCC/GND power: wide corridors, routed early.
    B,
    /// Ordinary GPIO/signal: lowest routing priority.
    C,
}

/// A pad's real, DFM-exact collision shape -- everything the router's
/// collision model needs to never let a route cross real copper, not a
/// compromise approximation of it. Round footprints use
/// [`PadShape::Circle`]; rectangular/oval (possibly rotated) pads use
/// [`PadShape::Polygon`] with the true filled outline. [`Item::Via`]
/// keeps a plain [`Circle`] because vias are always round.
#[derive(Debug, Clone, PartialEq)]
pub enum PadShape {
    /// A round pad or via footprint -- identical to every collision
    /// check this crate used before this type existed.
    Circle(Circle),
    /// A non-round (rectangular/oval, possibly rotated) pad's true
    /// filled outline. `center` is stored explicitly rather than
    /// derived from `outline`'s own vertex-average centroid -- cheap,
    /// exact, and immune to the rounding error a polygon-average would
    /// otherwise introduce for hit-testing/rendering call sites that
    /// just need "the pad's own center", not its geometric shape.
    Polygon { outline: Polygon, center: Point },
}

impl PadShape {
    pub fn center(&self) -> Point {
        match self {
            PadShape::Circle(c) => c.center,
            PadShape::Polygon { center, .. } => *center,
        }
    }

    fn aabb(&self) -> Aabb {
        match self {
            PadShape::Circle(c) => Aabb::from_circle(c),
            PadShape::Polygon { outline, .. } => Aabb::from_polygon(outline),
        }
    }

    /// Half-diagonal of this shape's own axis-aligned bounding box -- a
    /// purely cosmetic "how big a circle would visually cover this
    /// shape" helper for `alladin-pcb`'s hover/selection rings. Never
    /// used for any DRC/collision decision (those always go through
    /// [`Self::collides_with`]/[`Self::collides_with_circle`]/
    /// [`Self::collides_with_segment`] below, against the exact shape),
    /// so an approximation here is explicitly fine.
    pub fn bounding_radius(&self) -> Unit {
        match self {
            PadShape::Circle(c) => c.radius,
            PadShape::Polygon { .. } => {
                let b = self.aabb();
                let dx = (b.max.x - b.min.x) as f64;
                let dy = (b.max.y - b.min.y) as f64;
                ((dx * dx + dy * dy).sqrt() / 2.0).round() as Unit
            }
        }
    }

    fn collides_with_circle(&self, other: &Circle, clearance: Unit) -> bool {
        match self {
            PadShape::Circle(c) => circle_circle_collides(c, other, clearance),
            PadShape::Polygon { outline, .. } => circle_polygon_collides(other, outline, clearance),
        }
    }

    fn collides_with_segment(&self, seg: &Segment, clearance: Unit) -> bool {
        match self {
            PadShape::Circle(c) => circle_segment_collides(c, seg, clearance),
            PadShape::Polygon { outline, .. } => segment_polygon_collides(seg, outline, clearance),
        }
    }

    fn collides_with(&self, other: &PadShape, clearance: Unit) -> bool {
        match (self, other) {
            (PadShape::Circle(a), PadShape::Circle(b)) => circle_circle_collides(a, b, clearance),
            (PadShape::Circle(c), PadShape::Polygon { outline, .. })
            | (PadShape::Polygon { outline, .. }, PadShape::Circle(c)) => {
                circle_polygon_collides(c, outline, clearance)
            }
            (PadShape::Polygon { outline: o1, .. }, PadShape::Polygon { outline: o2, .. }) => {
                polygon_polygon_collides(o1, o2, clearance)
            }
        }
    }
}

/// **Not** `Copy` (unlike its predecessor before [`Item::Zone`] existed):
/// [`Item::Zone`]'s `outline: Polygon` owns a `Vec<Point>`, which can
/// never implement `Copy`. Every call site that previously relied on
/// `Item` being trivially copyable now clones explicitly instead (see
/// [`Node::remove`] and SHOVE-style track relocation, the only two spots
/// that ever pulled an owned `Item` out of a `&Item`).
#[derive(Debug, Clone)]
pub enum Item {
    /// A pad or via footprint -- a "hard island" per the water/island
    /// model. Unlike [`Item::Via`], a pad's `shape` can be non-round
    /// (see [`PadShape`]).
    Pad {
        shape: PadShape,
        net: Option<NetId>,
        layer: LayerId,
        /// Pour join style — see [`ZoneConnection`]. Set from the
        /// footprint template via `world_items` (parts DB / embedded
        /// snapshot); older templates without the field use Thermal.
        zone_connection: ZoneConnection,
    },
    /// A routed track segment (capsule shape).
    Track {
        shape: Segment,
        net: Option<NetId>,
        layer: LayerId,
        class: NetClass,
    },
    /// A via: present on both copper layers.
    Via {
        /// Outer copper diameter (the annular ring's outside edge) --
        /// this is what collision/clearance checks use, matching every
        /// other circular `Item`.
        shape: Circle,
        /// Drill hole diameter. **Not** derivable from `shape.radius`
        /// by a fixed ratio: real boards vary it independently of the
        /// outer diameter (e.g. `interf_u.kicad_pcb`, a real KiCad demo
        /// board, uses 1.397mm pads with a 0.6mm drill -- a ~0.43
        /// ratio). Purely informational for
        /// collision/clearance purposes today (see `JlcpcbClearance`'s
        /// `VIA_TO_TRACK` doc comment for why clearance still uses a
        /// blanket conservative constant rather than a per-via value),
        /// but required for a re-exported via to be manufacturable
        /// rather than just round-trippable.
        drill: Unit,
        net: Option<NetId>,
    },
    /// A statically imported, already-filled copper zone/ground-pour
    /// (KiCad's `(zone ... (filled_polygon ...))`) -- a *static*
    /// obstacle Alladin never itself creates or reshapes. `outline` is
    /// exactly the polygon KiCad already filled; Alladin neither
/// re-fills nor live-updates it as new tracks are routed. Thermal
    /// spokes are produced at fill time for [`ZoneConnection::Thermal`]
    /// pads (see `alladin-pcb`'s `zone_fill`); zone-priority between
    /// overlapping pours of different nets remains unmodeled. `net: None`
    /// models a netless keepout area: since [`Item::net`] is then `None`,
    /// the same-net collision fast path in [`Node::query_colliding`] never
    /// fires, so it blocks every net unconditionally -- no separate
    /// keepout mechanism needed.
    Zone {
        outline: Polygon,
        layer: LayerId,
        net: Option<NetId>,
    },
    /// A genuine unplated mechanical hole (NPTH -- "non-plated through
    /// hole"), e.g. a board mounting hole for a screw: **no copper, no
    /// net, ever** -- unlike [`Item::Via`] (always plated, always
    /// electrical) or a through-hole [`Item::Pad`] (plated, carries a
    /// net), this is purely a drilled mechanical feature. Modelled as
    /// its own variant rather than a pad with `net: None` specifically
    /// so nothing downstream (BOM export, Excellon NPTH writing,
    /// a future soldermask/keepout check) can mistake it for copper
    /// that merely happens to be unconnected right now. `drill` is
    /// the hole's own diameter -- there is no separate "outer copper"
    /// diameter the way [`Item::Via`] has one, since there's no copper
    /// at all. Blocks both copper layers unconditionally (see
    /// [`Self::layers`]): a drilled-through hole leaves no board
    /// material for a track to run under on *either* side, regardless
    /// of which layer that track is on.
    Hole {
        position: Point,
        drill: Unit,
    },
}

impl Item {
    pub fn net(&self) -> Option<NetId> {
        match self {
            Item::Pad { net, .. } | Item::Track { net, .. } | Item::Via { net, .. } => *net,
            Item::Zone { net, .. } => *net,
            Item::Hole { .. } => None,
        }
    }

    pub fn layers(&self) -> (LayerId, Option<LayerId>) {
        match self {
            Item::Pad { layer, .. } | Item::Track { layer, .. } | Item::Zone { layer, .. } => {
                (*layer, None)
            }
            // Both round, both drilled all the way through the board,
            // both correctly block either copper layer regardless of
            // which one a candidate route is on -- see this variant's
            // own doc comment (`Item::Hole`) and `Item::Via`'s doc
            // comment for why a via already works this way.
            Item::Via { .. } | Item::Hole { .. } => (LayerId::FCu, Some(LayerId::BCu)),
        }
    }

    fn on_layer(&self, layer: LayerId) -> bool {
        match self.layers() {
            (a, None) => a == layer,
            (a, Some(b)) => a == layer || b == layer,
        }
    }

    fn aabb(&self) -> Aabb {
        match self {
            Item::Pad { shape, .. } => shape.aabb(),
            Item::Via { shape, .. } => Aabb::from_circle(shape),
            Item::Track { shape, .. } => Aabb::from_segment(shape),
            Item::Zone { outline, .. } => Aabb::from_polygon(outline),
            // The screw-head keep-out (see `hole_keepout_circle`) is
            // what the spatial prefilter has to cover, or exact checks
            // against the enlarged circle would never even be reached.
            Item::Hole { position, drill } => Aabb::from_circle(&hole_keepout_circle(*position, *drill)),
        }
    }
}

/// Pluggable clearance policy -- this is where JLCPCB DFM rules (today's
/// `jlc-ai-tools/jlc_dfm_rules.py`) get ported in as Rust logic, matching
/// the "Correct-by-Construction" principle: clearance is a hard axiom fed
/// into geometry, not a post-hoc DRC check.
///
/// `Sync`: clearance queries may share a `&dyn RuleResolver` across
/// worker threads -- a no-op requirement for both real implementors
/// below (`FixedClearance`, `JlcpcbClearance`), which are already plain,
/// stateless data with no interior mutability.
pub trait RuleResolver: Sync {
    fn clearance(&self, a: &Item, b: &Item) -> Unit;

    /// Upper bound on any clearance this resolver will ever return, used
    /// to size the broad-phase R-tree query window. Must never
    /// under-report, or real collisions get missed.
    fn max_clearance(&self) -> Unit;
}

/// A fixed clearance for every item pair -- the simplest possible
/// resolver, e.g. the JLCPCB `2layer_1oz` minimum (0.127 mm / 127000 nm).
pub struct FixedClearance(pub Unit);

impl RuleResolver for FixedClearance {
    fn clearance(&self, _a: &Item, _b: &Item) -> Unit {
        self.0
    }
    fn max_clearance(&self) -> Unit {
        self.0
    }
}

/// Real JLCPCB clearance rules for the `2layer_1oz` capability profile --
/// ported from the sibling project's `jlc-ai-tools/jlc_dfm_rules.py`
/// (`profile_2layer_1oz()`; values sourced from
/// <https://jlcpcb.com/capabilities/pcb-capabilities>, mirrored 2026-07-20).
///
/// This is Alladin's first *DFM-aware* resolver -- clearance now depends
/// on which two item *kinds* are interacting, exactly the distinction
/// KiCad's own `PNS::RULE_RESOLVER` makes, replacing the placeholder
/// [`FixedClearance`] used everywhere so far.
///
/// The `2layer_1oz` profile (this struct) and [`Jlcpcb2Layer2Oz`] are the
/// two ported so far. The Python source's `rule_variants()` also has
/// `2layer_2.5oz`/`3.5oz`/`4.5oz` and `multilayer_1oz` profiles with
/// further (usually wider still) minimums; those remain follow-up, and
/// Alladin still has no per-board notion of "which profile applies here"
/// -- callers pick a resolver explicitly. `2layer_2oz` is a separate
/// struct rather than a constructor parameter so each profile stays a
/// zero-sized type with compile-time constants.
pub struct JlcpcbClearance;

impl JlcpcbClearance {
    /// `pad_to_track` -- "Pad to track clearance. Min. 0.1 mm (0.09 mm
    /// locally for BGA pads)." Applied to any `Pad`-vs-`Track` pair.
    pub const PAD_TO_TRACK: Unit = 100_000;

    /// `via_hole_to_track` -- "Via hole to track clearance." Stricter
    /// than `pad_to_track` because it's measured to the *drilled hole*,
    /// not a copper pad edge. Alladin's [`Item::Via`] doesn't yet
    /// distinguish hole diameter from pad diameter, so this -- the
    /// larger, more conservative value -- is applied to every
    /// `Via`-vs-`Track` pair rather than risk under-clearing.
    pub const VIA_TO_TRACK: Unit = 200_000;

    /// `smd_pad_to_pad_different_nets` -- "SMD pad to pad clearance
    /// (different nets)." Also covers `Pad`-vs-`Via` and `Via`-vs-`Via`
    /// here: the official capability table has no separate via-to-pad
    /// row, and JLCDFM's own local geometry preflight
    /// (`JLCDFM_GOOD["smd_pad_to_pad"]` in the same Python source) uses
    /// this exact 0.15 mm bucket for via-to-pad too.
    pub const PAD_TO_PAD: Unit = 150_000;

    /// `min_track_spacing` -- "Min. track spacing, 1- and 2-layer, 1 oz
    /// (4 mil)." Track-to-track, different nets. (Same-net track pairs
    /// never reach this resolver at all -- [`Node::query_colliding`]
    /// short-circuits same-net pairs before calling `clearance()`,
    /// matching KiCad's own fast path; the Python source's separate,
    /// wider `same_net_track_spacing` = 0.25 mm rule is therefore not
    /// applicable here.)
    pub const TRACK_TO_TRACK: Unit = 100_000;
}

impl RuleResolver for JlcpcbClearance {
    fn clearance(&self, a: &Item, b: &Item) -> Unit {
        match (a, b) {
            (Item::Track { .. }, Item::Track { .. }) => Self::TRACK_TO_TRACK,
            // A mounting hole's own drill is mechanically drilled
            // exactly like a via's, so it gets the same, stricter
            // `via_hole_to_track` clearance rather than falling back to
            // the looser `pad_to_track` minimum -- see `Item::Hole`'s
            // own doc comment for why it's modelled separately from a
            // via at all despite sharing this clearance treatment.
            (Item::Via { .. } | Item::Hole { .. }, Item::Track { .. })
            | (Item::Track { .. }, Item::Via { .. } | Item::Hole { .. }) => Self::VIA_TO_TRACK,
            (Item::Pad { .. }, Item::Track { .. }) | (Item::Track { .. }, Item::Pad { .. }) => {
                Self::PAD_TO_TRACK
            }
            // A zone is a filled copper area, so it's treated exactly
            // like the item kind it would otherwise be a collision
            // partner for: zone-vs-track uses the track-to-track
            // minimum, zone-vs-pad/via/hole the pad/via-to-track
            // minimum (JLCPCB has no separate "zone" clearance row; treat
            // pours like the copper they collide with).
            (Item::Zone { .. }, Item::Track { .. }) | (Item::Track { .. }, Item::Zone { .. }) => {
                Self::TRACK_TO_TRACK
            }
            (Item::Zone { .. }, Item::Pad { .. } | Item::Via { .. } | Item::Hole { .. })
            | (Item::Pad { .. } | Item::Via { .. } | Item::Hole { .. }, Item::Zone { .. }) => Self::PAD_TO_TRACK,
            // Remaining combinations are Pad/Via/Hole vs. Pad/Via/Hole,
            // or Zone vs. Zone (never queried in practice -- Alladin
            // only ever routes `Track`/`Via` items, so a `Zone` never
            // appears as the moving `candidate` side of a collision
            // query -- but still needs a value for
            // `RuleResolver::clearance`'s total function).
            _ => Self::PAD_TO_PAD,
        }
    }

    fn max_clearance(&self) -> Unit {
        Self::VIA_TO_TRACK // the largest of the four constants above
    }
}

/// Real JLCPCB clearance rules for the `2layer_2oz` capability profile
/// (heavier 2 oz outer copper) -- ported from the same Python source's
/// `rule_variants()["2layer_2oz"]`.
///
/// That variant overrides exactly three of `profile_2layer_1oz()`'s rows:
/// `min_track_width`, `min_track_spacing` (0.10mm -> 0.16mm each -- thicker
/// copper needs a wider trace to hit the same current rating, and a wider
/// gap to avoid solder-mask bridging between traces), and
/// `soldermask_bridge` (0.10mm -> 0.20mm, not modelled here -- Alladin has
/// no soldermask-opening concept yet). `pad_to_track` and
/// `smd_pad_to_pad_different_nets` are untouched by the variant, so
/// [`Jlcpcb2Layer2Oz::PAD_TO_TRACK`] and [`Jlcpcb2Layer2Oz::PAD_TO_PAD`]
/// keep [`JlcpcbClearance`]'s values -- only [`Jlcpcb2Layer2Oz::TRACK_TO_TRACK`]
/// actually differs, since track *width* itself is a per-[`Item::Track`]
/// property the router already takes from its caller rather than
/// something a [`RuleResolver`] governs.
pub struct Jlcpcb2Layer2Oz;

impl Jlcpcb2Layer2Oz {
    /// Unchanged from `2layer_1oz` -- the `2layer_2oz` variant doesn't
    /// override `pad_to_track`.
    pub const PAD_TO_TRACK: Unit = JlcpcbClearance::PAD_TO_TRACK;

    /// Unchanged from `2layer_1oz` -- the `2layer_2oz` variant doesn't
    /// override `smd_pad_to_pad_different_nets`.
    pub const PAD_TO_PAD: Unit = JlcpcbClearance::PAD_TO_PAD;

    /// Unchanged from `2layer_1oz` -- `via_hole_to_track` is measured to
    /// the drilled hole, which the 2oz *outer copper* variant doesn't
    /// affect.
    pub const VIA_TO_TRACK: Unit = JlcpcbClearance::VIA_TO_TRACK;

    /// `min_track_spacing` under `2layer_2oz`: 0.16 mm, up from the
    /// `2layer_1oz` profile's 0.10 mm.
    pub const TRACK_TO_TRACK: Unit = 160_000;
}

impl RuleResolver for Jlcpcb2Layer2Oz {
    fn clearance(&self, a: &Item, b: &Item) -> Unit {
        match (a, b) {
            (Item::Track { .. }, Item::Track { .. }) => Self::TRACK_TO_TRACK,
            (Item::Via { .. } | Item::Hole { .. }, Item::Track { .. })
            | (Item::Track { .. }, Item::Via { .. } | Item::Hole { .. }) => Self::VIA_TO_TRACK,
            (Item::Pad { .. }, Item::Track { .. }) | (Item::Track { .. }, Item::Pad { .. }) => {
                Self::PAD_TO_TRACK
            }
            (Item::Zone { .. }, Item::Track { .. }) | (Item::Track { .. }, Item::Zone { .. }) => {
                Self::TRACK_TO_TRACK
            }
            (Item::Zone { .. }, Item::Pad { .. } | Item::Via { .. } | Item::Hole { .. })
            | (Item::Pad { .. } | Item::Via { .. } | Item::Hole { .. }, Item::Zone { .. }) => Self::PAD_TO_TRACK,
            _ => Self::PAD_TO_PAD,
        }
    }

    fn max_clearance(&self) -> Unit {
        Self::VIA_TO_TRACK
    }
}

/// The rest of JLCPCB's `2layer_1oz` DFM table -- everything
/// [`JlcpcbClearance`] doesn't already cover because it isn't a
/// pairwise item-to-item *clearance* (that struct's whole job), but a
/// scalar geometric minimum: how wide a trace/drill/silkscreen line is
/// allowed to be, or how close copper may come to the board's own edge.
/// Ported from the same source as [`JlcpcbClearance`] --
/// `jlc-ai-tools/jlc_dfm_rules.py`'s `profile_2layer_1oz()`, values
/// sourced from <https://jlcpcb.com/capabilities/pcb-capabilities>,
/// mirrored 2026-07-20 -- kept as a **separate** struct because these
/// values plug into different call sites than a [`RuleResolver`]
/// (`alladin-pcb`'s placement/routing/footprint-import code, not
/// `Node::query_colliding`). Some rows (silkscreen, soldermask, and
/// heavier copper variants of these minima) are documented here for
/// completeness even where no Rust call site enforces them yet.
pub struct JlcpcbDfm;

impl JlcpcbDfm {
    /// `min_track_width`, 1oz outer copper. **Not the same number as
    /// [`JlcpcbClearance::TRACK_TO_TRACK`]** (both happen to be 0.10mm
    /// at this specific weight, but one is a *spacing* between two
    /// tracks and this one is how thin a single trace's copper may be)
    /// -- they diverge as soon as copper weight changes (see
    /// [`Jlcpcb2Layer2Oz`], which only overrides the spacing side).
    pub const MIN_TRACK_WIDTH: Unit = 100_000;

    /// `min_via_hole` -- smallest drillable via hole diameter.
    pub const MIN_VIA_HOLE: Unit = 150_000;
    /// `min_via_diameter` -- smallest via *outer copper* diameter.
    pub const MIN_VIA_DIAMETER: Unit = 250_000;
    /// `min_via_annular_over_hole` -- minimum copper ring width around a
    /// via's own drill hole (i.e. `(MIN_VIA_DIAMETER - MIN_VIA_HOLE) /
    /// 2` at the absolute limits, but checked independently since a
    /// via can be larger than the minimum on one side only).
    pub const MIN_VIA_ANNULAR_RING: Unit = 100_000;

    /// `pth_annular_ring` -- minimum copper ring around a through-hole
    /// *component* pad's drill (stricter than a via's, since a
    /// mis-plated component pad risks a cold joint, not just a broken
    /// via).
    pub const MIN_PTH_ANNULAR_RING: Unit = 180_000;
    /// `min_drill_diameter` -- smallest hole this fab can drill at all,
    /// via or component pad.
    pub const MIN_DRILL_DIAMETER: Unit = 150_000;
    /// `min_npth_hole` -- smallest *non*-plated hole (mounting holes,
    /// not electrical) -- larger than `MIN_DRILL_DIAMETER` because NPTH
    /// holes are mechanically drilled post-plating, a coarser process.
    pub const MIN_NPTH_HOLE: Unit = 500_000;

    /// `min_smd_pad_size` -- smallest SMD pad JLCPCB will reliably
    /// solder-paste-print and place onto. Below this, solder-paste
    /// stencil apertures become unreliable regardless of what the
    /// pad's own copper geometry allows.
    pub const MIN_SMD_PAD_SIZE: Unit = 250_000;

    /// `copper_to_routed_edge` -- minimum clearance from any copper
    /// (pad *or* track) to a CNC-routed board edge. The most-violated
    /// DFM rule in hobbyist designs precisely because "the pad is
    /// technically inside the outline" (all this crate checked before
    /// this constant existed -- see [`alladin_geom::circle_within_outline`]'s
    /// doc comment) is necessary but not sufficient: real fabrication
    /// tooling needs clearance *from* the cut line, not just to stay on
    /// the right side of it.
    pub const COPPER_TO_ROUTED_EDGE: Unit = 200_000;
    /// `copper_to_vcut_edge` -- wider than [`Self::COPPER_TO_ROUTED_EDGE`]:
    /// V-scoring is far less precise than CNC routing. Not yet
    /// distinguished anywhere in this codebase (panelization/V-cut
    /// outlines aren't modelled), so [`Self::COPPER_TO_ROUTED_EDGE`] --
    /// the *smaller*, i.e. more permissive, of the two -- is what's
    /// actually applied everywhere today. Kept here so a future
    /// panelization feature has the real number on hand rather than
    /// needing to re-derive it.
    pub const COPPER_TO_VCUT_EDGE: Unit = 400_000;

    /// `silk_line_width`, **with an added safety margin**: JLCPCB's own
    /// published absolute floor is 0.15mm (confirmed live 2026-08-03
    /// across <https://jlcpcb.com/blog/pcb-silkscreen>,
    /// <https://jlcpcb.com/blog/character-design-specifications>, and
    /// <https://jlcpcb.com/help/article/instructions-for-ordering>: "the
    /// width of the texts ... need to be no less than 0.15mm"), but a
    /// design sitting exactly on that bare floor is also explicitly the
    /// one case JLCPCB itself warns it may silently "widen the strokes"
    /// on, or refuse complaints about unclear text for. This constant is
    /// therefore deliberately 0.17mm -- 0.15mm plus a ~13% buffer,
    /// inside the "10-15% on top of the real minimum" margin this
    /// codebase settled on for exactly this rule -- so a line this
    /// codebase calls "the minimum" is never the exact bare number a
    /// real print run might round down past. JLCPCB's own *recommended*
    /// production standard is wider still, 0.2mm (`alladin-pcb`'s own
    /// `DEFAULT_SILK_LINE_WIDTH` already uses exactly that) -- this
    /// constant is only ever the hard floor nothing may go *below*, not
    /// a suggested default.
    pub const MIN_SILK_LINE_WIDTH: Unit = 170_000;
    /// `pad_to_silkscreen` -- minimum gap between a copper pad and any
    /// silkscreen, so printed ink never bridges onto solderable copper
    /// (a "silk over copper" DRC violation).
    pub const SILK_TO_PAD: Unit = 150_000;

    /// `min_board_edge` -- JLCPCB's smallest manufacturable board
    /// dimension in either axis (below this, panelization is required
    /// regardless of design).
    pub const MIN_BOARD_DIMENSION: Unit = 3_000_000;

    /// JLCPCB's own LPI soldermask expansion on a 2-layer board: each
    /// mask opening is the pad's copper grown by this much per side
    /// (their capabilities table's "Soldermask Expansion" row, "2 layer:
    /// Expansion 0.038 mm each side", confirmed live 2026-08-07).
    /// Alladin doesn't draw mask layers itself -- this exists so
    /// [`Self::SOLDERMASK_DAM_MIN_PAD_GAP`]'s derivation is on record.
    pub const SOLDERMASK_EXPANSION: Unit = 38_000;
    /// Minimum *copper* gap between two pads for JLCPCB to fabricate a
    /// soldermask dam (web) between them on a 2-layer board -- their
    /// ordering instructions state it verbatim: "the spacing between
    /// the pads/pins needs to be at least 0.2mm on 1 or 2 layer board"
    /// (<https://jlcpcb.com/help/article/instructions-for-ordering>,
    /// "Solder-Resistance Bridges", confirmed live 2026-08-07; black/
    /// white mask needs 0.23mm -- not modelled, Alladin assumes the
    /// standard colours). Pads closer than this still *fabricate*, but
    /// JLCPCB merges ("gangs") their mask openings into one, leaving no
    /// dam -- a real solder-bridging risk at assembly, which is why
    /// this is a **report-level warning** (`alladin-pcb`'s DFM check),
    /// never a hard placement gate: plenty of real, fab-proven
    /// fine-pitch parts sit below it by design.
    pub const SOLDERMASK_DAM_MIN_PAD_GAP: Unit = 200_000;

    /// `hole_to_hole_clearance` (different nets) -- minimum wall-to-wall
    /// distance between two drilled holes on different nets (vias, PTH
    /// pads, NPTH mechanical holes alike): JLCPCB's capabilities table
    /// row "Hole to hole clearance (Different nets): 0.5mm". Drills
    /// wander; two walls closer than this risk breaking into each other.
    pub const HOLE_TO_HOLE_DIFFERENT_NET: Unit = 500_000;
    /// `via_to_via_clearance` (same net) -- the same wall-to-wall rule
    /// between two holes that share a net (e.g. a stitching-via
    /// cluster): JLCPCB's "Via to Via clearance (Same nets): 0.254mm".
    /// Smaller than [`Self::HOLE_TO_HOLE_DIFFERENT_NET`] because a
    /// breakout between same-net holes is an electrical non-event --
    /// only the mechanical drill-bit spacing still matters.
    pub const HOLE_TO_HOLE_SAME_NET: Unit = 254_000;

    /// Minimum clearance from a component's own **body** (its
    /// courtyard, not its copper) to the board edge -- a real,
    /// official JLCPCB *assembly* rule, distinct from every other
    /// constant in this struct (which are all *fabrication* rules
    /// sourced from `jlc_dfm_rules.py`/the PCB capabilities page):
    /// JLCPCB's own "Terms and Conditions of JLCPCB Assembly Service"
    /// page, DFM note 3, states verbatim "The distance between the
    /// body of the components and the edge of the board must be equal
    /// or greater than 2.5mm" (confirmed live 2026-08-02, see
    /// <https://jlcpcb.com/help/article/terms-and-conditions-of-jlcpcb-assembly-service>) --
    /// the pick-and-place nozzle and reflow-oven rails physically need
    /// this much clearance around a part's real body, well beyond
    /// [`Self::COPPER_TO_ROUTED_EDGE`]'s much smaller 0.20mm (that one
    /// is a *fabrication* rule about the bare board's own copper, not
    /// an *assembly* rule about a populated part's real, physical
    /// height/footprint sitting on top of it).
    pub const COMPONENT_BODY_TO_EDGE: Unit = 2_500_000;

    /// Minimum wall-to-wall clearance between two components'
    /// courtyards/bodies -- a conservative uniform floor for JLCPCB
    /// SMT assembly spacing (their package matrix goes as low as
    /// 0.15 mm for 0402–0402 and much higher for QFN/BGA; Alladin
    /// does not yet classify packages, so this 0.3 mm floor is the
    /// hard placement gate). Used by `alladin-pcb`'s
    /// `BoardDoc::check_placement` body-vs-body check (formerly 0 /
    /// touching-allowed).
    pub const COMPONENT_BODY_CLEARANCE: Unit = 300_000;

    /// Minimum clearance from an SMD pad's copper ("lead") to another
    /// footprint's plated or mechanical drill -- JLCPCB assembly DFM
    /// "Lead to hole distance". Same 0.3 mm conservative floor as
    /// [`Self::COMPONENT_BODY_CLEARANCE`] until a finer rule table
    /// exists. Distinct from copper pad-to-pad clearance
    /// ([`JlcpcbClearance`]) and from hole-to-hole drill spacing
    /// ([`Self::HOLE_TO_HOLE_DIFF_NET`]).
    pub const COMPONENT_LEAD_TO_HOLE: Unit = 300_000;

    /// Hard gate for a single track's copper width -- returns
    /// [`DfmViolation::TrackWidthBelowMin`] when `width` is thinner
    /// than [`Self::MIN_TRACK_WIDTH`]. Pairwise *spacing* between two
    /// tracks is a separate concern owned by [`JlcpcbClearance`]; this
    /// is only "how thin may one track be".
    pub fn check_track_width(width: Unit) -> Result<(), DfmViolation> {
        if width < Self::MIN_TRACK_WIDTH {
            return Err(DfmViolation::TrackWidthBelowMin);
        }
        Ok(())
    }

    /// Hard gate for a via's outer copper diameter, drill hole, and
    /// annular ring -- the three scalar JLCPCB rows that
    /// [`JlcpcbClearance`] (a pairwise item-to-item resolver) can
    /// never catch on its own. Applied at every write path that can
    /// create a via (`BoardDoc::try_add_via` and the GUI via tools), so a
    /// sub-minimum via can never land on a
    /// board even when an external tool invents one.
    pub fn check_via(diameter: Unit, drill: Unit) -> Result<(), DfmViolation> {
        if diameter < Self::MIN_VIA_DIAMETER {
            return Err(DfmViolation::ViaDiameterBelowMin);
        }
        if drill < Self::MIN_VIA_HOLE {
            return Err(DfmViolation::ViaHoleBelowMin);
        }
        if drill >= diameter {
            return Err(DfmViolation::ViaDrillExceedsDiameter);
        }
        // Integer nm: `(diameter - drill) / 2` is exact for the even
        // values JLCPCB's table publishes; no rounding needed.
        let annular = (diameter - drill) / 2;
        if annular < Self::MIN_VIA_ANNULAR_RING {
            return Err(DfmViolation::ViaAnnularRingBelowMin);
        }
        Ok(())
    }

    /// Hard gate for an SMD pad's copper geometry: its smallest
    /// dimension (a circle's diameter, a rect/oval's narrower side)
    /// must reach [`Self::MIN_SMD_PAD_SIZE`], or JLCPCB's solder-paste
    /// stencil apertures become unreliable. Applied where footprint
    /// geometry enters the system (`alladin-pcb`'s part registration,
    /// LCSC download, and footprint placement), since pads are never
    /// free-standing items.
    pub fn check_smd_pad(min_dimension: Unit) -> Result<(), DfmViolation> {
        if min_dimension < Self::MIN_SMD_PAD_SIZE {
            return Err(DfmViolation::SmdPadBelowMin);
        }
        Ok(())
    }

    /// Hard gate for a plated through-hole pad: the drill must be
    /// physically drillable ([`Self::MIN_DRILL_DIAMETER`]) and the
    /// copper ring around it -- `(smallest pad dimension - drill) / 2`,
    /// conservative for non-circular pads -- must reach
    /// [`Self::MIN_PTH_ANNULAR_RING`] (stricter than a via's ring: a
    /// mis-plated *component* pad risks a cold joint, not just a
    /// broken via barrel). Same entry points as [`Self::check_smd_pad`].
    pub fn check_pth_pad(min_dimension: Unit, drill: Unit) -> Result<(), DfmViolation> {
        if drill < Self::MIN_DRILL_DIAMETER {
            return Err(DfmViolation::PthDrillBelowMin);
        }
        if drill >= min_dimension {
            return Err(DfmViolation::PthDrillExceedsPad);
        }
        if (min_dimension - drill) / 2 < Self::MIN_PTH_ANNULAR_RING {
            return Err(DfmViolation::PthAnnularRingBelowMin);
        }
        Ok(())
    }

    /// Hard gate for a non-plated mechanical hole: JLCPCB drills NPTH
    /// holes post-plating with a coarser process, so anything below
    /// [`Self::MIN_NPTH_HOLE`] is refused. Same entry points as
    /// [`Self::check_smd_pad`].
    pub fn check_npth_hole(drill: Unit) -> Result<(), DfmViolation> {
        if drill < Self::MIN_NPTH_HOLE {
            return Err(DfmViolation::NpthHoleBelowMin);
        }
        Ok(())
    }

    /// The wall-to-wall spacing two drilled holes must keep, given
    /// whether they share a net -- [`Self::HOLE_TO_HOLE_SAME_NET`] only
    /// when both sides *have* a net and it's the same one; everything
    /// else (different nets, either side net-less, NPTH) gets the
    /// stricter [`Self::HOLE_TO_HOLE_DIFFERENT_NET`].
    pub fn required_hole_to_hole(net_a: Option<u32>, net_b: Option<u32>) -> Unit {
        match (net_a, net_b) {
            (Some(a), Some(b)) if a == b => Self::HOLE_TO_HOLE_SAME_NET,
            _ => Self::HOLE_TO_HOLE_DIFFERENT_NET,
        }
    }
}

/// A scalar JLCPCB DFM rule that a single item violates on its own
/// (independent of any other item on the board) -- the counterpart to
/// a pairwise clearance failure from [`RuleResolver`]. Kept as a
/// small `Copy` enum so [`crate`]-external callers (notably
/// `alladin-pcb`'s `PlacementError`) can carry it without allocating,
/// and so a refused write can name the exact row from
/// `profile_2layer_1oz()` that was hit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DfmViolation {
    /// Track copper thinner than [`JlcpcbDfm::MIN_TRACK_WIDTH`].
    TrackWidthBelowMin,
    /// Via outer copper smaller than [`JlcpcbDfm::MIN_VIA_DIAMETER`].
    ViaDiameterBelowMin,
    /// Via drill smaller than [`JlcpcbDfm::MIN_VIA_HOLE`].
    ViaHoleBelowMin,
    /// `(diameter - drill) / 2` thinner than
    /// [`JlcpcbDfm::MIN_VIA_ANNULAR_RING`].
    ViaAnnularRingBelowMin,
    /// Drill hole wider than (or equal to) the via's own outer copper
    /// -- not a published JLCPCB table row of its own, but physically
    /// impossible and a common transcription error when an external
    /// tool swaps the two numbers.
    ViaDrillExceedsDiameter,
    /// An SMD pad's smallest dimension below
    /// [`JlcpcbDfm::MIN_SMD_PAD_SIZE`].
    SmdPadBelowMin,
    /// A through-hole pad's drill below
    /// [`JlcpcbDfm::MIN_DRILL_DIAMETER`].
    PthDrillBelowMin,
    /// A through-hole pad's drill wider than (or equal to) its own
    /// copper -- the PTH twin of [`Self::ViaDrillExceedsDiameter`].
    PthDrillExceedsPad,
    /// A through-hole pad's copper ring thinner than
    /// [`JlcpcbDfm::MIN_PTH_ANNULAR_RING`].
    PthAnnularRingBelowMin,
    /// A non-plated mechanical hole below [`JlcpcbDfm::MIN_NPTH_HOLE`].
    NpthHoleBelowMin,
    /// Two drilled holes' walls closer than
    /// [`JlcpcbDfm::required_hole_to_hole`] allows for their net pair.
    HoleToHoleBelowMin,
}

impl std::fmt::Display for DfmViolation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DfmViolation::TrackWidthBelowMin => {
                write!(f, "track width below JLCPCB's 0.10 mm minimum")
            }
            DfmViolation::ViaDiameterBelowMin => {
                write!(f, "via diameter below JLCPCB's 0.25 mm minimum")
            }
            DfmViolation::ViaHoleBelowMin => {
                write!(f, "via drill below JLCPCB's 0.15 mm minimum")
            }
            DfmViolation::ViaAnnularRingBelowMin => {
                write!(f, "via annular ring below JLCPCB's 0.10 mm minimum")
            }
            DfmViolation::ViaDrillExceedsDiameter => {
                write!(f, "via drill is larger than (or equal to) the via's outer diameter")
            }
            DfmViolation::SmdPadBelowMin => {
                write!(f, "SMD pad's smallest dimension below JLCPCB's 0.25 mm minimum")
            }
            DfmViolation::PthDrillBelowMin => {
                write!(f, "through-hole pad drill below JLCPCB's 0.15 mm minimum")
            }
            DfmViolation::PthDrillExceedsPad => {
                write!(f, "through-hole pad drill is larger than (or equal to) the pad's own copper")
            }
            DfmViolation::PthAnnularRingBelowMin => {
                write!(f, "through-hole pad annular ring below JLCPCB's 0.18 mm minimum")
            }
            DfmViolation::NpthHoleBelowMin => {
                write!(f, "non-plated hole below JLCPCB's 0.50 mm minimum")
            }
            DfmViolation::HoleToHoleBelowMin => {
                write!(f, "two drill holes closer than JLCPCB's hole-to-hole minimum (0.50 mm different nets / 0.254 mm same net)")
            }
        }
    }
}

#[derive(Clone, PartialEq)]
struct IndexedItem {
    id: ItemId,
    envelope: AABB<[f64; 2]>,
}

impl RTreeObject for IndexedItem {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

fn aabb_to_envelope(b: Aabb) -> AABB<[f64; 2]> {
    AABB::from_corners(
        [b.min.x as f64, b.min.y as f64],
        [b.max.x as f64, b.max.y as f64],
    )
}

/// The routing "world": every placed pad/track/via, spatially indexed.
/// Equivalent role to `PNS::NODE`.
///
/// Note on Clone-on-Write: `Node` currently derives a plain deep `Clone`
/// (correctness first). The architecture note's "nanosecond CoW clone for
/// backtracking" is a real, separate optimization step (persistent
/// data structures, e.g. the `im` crate, or an explicit parent/diff chain
/// like KiCad's own `NODE::Branch()`) -- tracked as follow-up, not yet
/// implemented.
///
/// Note on removal: `items` never shrinks or reorders -- a removed slot
/// is tombstoned (`removed[i] = true`), never reused, so every
/// [`ItemId`] a caller is still holding onto stays meaningful (either
/// "still live" or "was removed", never silently repointed at some
/// unrelated later item the way a swap-remove would). This trades a
/// small amount of permanently-unusable `Vec` space for that guarantee
/// -- acceptable for a router's lifetime-of-one-board `Node`, unlike a
/// long-running server-style structure that would need real compaction.
pub struct Node {
    items: Vec<Item>,
    removed: Vec<bool>,
    live_count: usize,
    tree: RTree<IndexedItem>,
    /// Lazily-built, per-[`ItemId`] [`PolygonEdgeIndex`] cache for
    /// [`Item::Zone`] items -- see [`Self::zone_edge_index`]'s doc
    /// comment for why this exists and why caching (not just using the
    /// indexed check once) is what actually matters here. `Arc`, not
    /// `Rc`: `Node` must stay `Send` (it's handed wholesale to
/// `alladin-viewer`'s background router thread), which an `Rc`
/// inside it would silently break. `RwLock`, not `RefCell`: `Node`
/// must also stay `Sync` so `&Node` can be shared across parallel
/// collision/clearance queries -- a `RefCell` here would make that
/// impossible to even compile. The lock is only ever held briefly
/// (a `HashMap` lookup or insert), never across a collision check.
    ///
    /// `RwLock` has no blanket `Clone` impl (unlike `RefCell`), so
    /// [`Node`] can no longer `#[derive(Clone)]` -- see the manual
    /// `impl Clone for Node` below, which just locks and clones the
    /// cache's current contents like any other field.
    zone_edge_index_cache: RwLock<HashMap<ItemId, Arc<PolygonEdgeIndex>>>,
    /// Bumped by [`Self::add`]/[`Self::remove`]/[`Self::replace`] every
    /// time a `Pad`/`Via`/`Track`/`Hole` (never a bare `Item::Zone`
    /// fill island -- see each of those methods' own check) enters or
    /// leaves the world -- i.e. every time some existing `Item::Zone`
    /// fill could have gone stale against whatever's actually here now.
    /// Exists purely so a caller (`alladin_pcb::board_doc::ZoneRecord`'s
    /// own `filled_at_revision`) can cheaply answer "is this zone's fill
    /// still current?" as a plain integer comparison, instead of either
    /// eagerly re-filling on every single edit (real cost, e.g. for a
    /// SHOVE-heavy interactive drag) or never surfacing staleness at all
    /// until a manual refill happens to make it visually obvious.
    obstacle_revision: u64,
}

impl Clone for Node {
    fn clone(&self) -> Self {
        Self {
            items: self.items.clone(),
            removed: self.removed.clone(),
            live_count: self.live_count,
            tree: self.tree.clone(),
            zone_edge_index_cache: RwLock::new(self.zone_edge_index_cache.read().unwrap().clone()),
            obstacle_revision: self.obstacle_revision,
        }
    }
}

impl Default for Node {
    fn default() -> Self {
        Self {
            items: Vec::new(),
            removed: Vec::new(),
            live_count: 0,
            tree: RTree::new(),
            zone_edge_index_cache: RwLock::new(HashMap::new()),
            obstacle_revision: 0,
        }
    }
}

impl Node {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add(&mut self, item: Item) -> ItemId {
        if !matches!(item, Item::Zone { .. }) {
            self.obstacle_revision += 1;
        }
        let id = ItemId(self.items.len());
        let envelope = aabb_to_envelope(item.aabb());
        self.items.push(item);
        self.removed.push(false);
        self.live_count += 1;
        self.tree.insert(IndexedItem { id, envelope });
        id
    }

    /// See [`Self::obstacle_revision`]'s own doc comment.
    pub fn obstacle_revision(&self) -> u64 {
        self.obstacle_revision
    }

    /// `None` if `id` was never valid or has since been [`Self::remove`]d
    /// (including implicitly, via [`Self::replace`]) -- deliberately
    /// fallible, unlike the pre-removal version of this method, since a
    /// stale `ItemId` silently handing back stale geometry would be
    /// exactly the kind of bug SHOVE-style mutation makes possible for
    /// the first time in this codebase.
    pub fn get(&self, id: ItemId) -> Option<&Item> {
        if self.removed[id.0] {
            None
        } else {
            Some(&self.items[id.0])
        }
    }

/// Every *live* item currently in the world, in no particular order
/// (removed items are silently skipped). Unfiltered fallback when
/// [`Node::query_region`] doesn't cover every obstacle a route might
/// need -- unlike [`Node::query_colliding`], this deliberately does
/// *not* pre-filter by net/layer, since a path that happens to pass
/// near an irrelevant item is harmless (the real validity check is
/// still the exact [`Node::path_is_clear`] call, which does do that
/// filtering).
    pub fn iter(&self) -> impl Iterator<Item = &Item> + '_ {
        self.iter_with_ids().map(|(_, item)| item)
    }

    /// Same as [`Self::iter`], but paired with each item's own
    /// [`ItemId`] -- needed by anything that might later want to
/// [`Self::remove`]/[`Self::replace`] one of the items it's looking
/// at (SHOVE-style "is this candidate blocker's net otherwise
/// untouched by any neighbouring track?" checks are the first real
/// callers).
    pub fn iter_with_ids(&self) -> impl Iterator<Item = (ItemId, &Item)> + '_ {
        self.items
            .iter()
            .zip(self.removed.iter())
            .enumerate()
            .filter(|(_, (_, removed))| !**removed)
            .map(|(i, (item, _))| (ItemId(i), item))
    }

/// Every item whose bounding box intersects `region` -- the R-tree
/// spatial pre-filter for collision/clearance queries that only need
/// obstacles plausibly relevant to a given corridor, instead of every
/// item on the whole board. Same non-filtering-by-net/layer caveat as
/// [`Node::iter`]. Removed items are never returned -- they're gone
/// from the R-tree itself, not just filtered here.
    pub fn query_region(&self, region: Aabb) -> Vec<&Item> {
        let envelope = aabb_to_envelope(region);
        self.tree
            .locate_in_envelope_intersecting(&envelope)
            .map(|indexed| &self.items[indexed.id.0])
            .collect()
    }

    pub fn len(&self) -> usize {
        self.live_count
    }

    pub fn is_empty(&self) -> bool {
        self.live_count == 0
    }

    /// Removes `id` from the world -- gone from [`Self::iter`],
    /// [`Self::iter_with_ids`], [`Self::query_region`], and
    /// [`Self::query_colliding`] alike -- and hands back the removed
    /// item. The first real mutation `Node` has ever supported (every
    /// caller so far only ever [`Self::add`]ed); needed for SHOVE, which
    /// must be able to change an already-placed track's geometry rather
    /// than only ever routing new ones around it.
    ///
    /// # Panics
    /// If `id` is not currently live (never added, or already removed).
    /// Both are always a caller bug, not a condition worth handling
    /// gracefully -- unlike, say, a route genuinely failing to find a
    /// path, there's no legitimate reason to ask `Node` to remove
    /// something it doesn't have.
    pub fn remove(&mut self, id: ItemId) -> Item {
        assert!(
            !self.removed[id.0],
            "Node::remove: {id:?} was never added or already removed"
        );
        let item = self.items[id.0].clone();
        if !matches!(item, Item::Zone { .. }) {
            self.obstacle_revision += 1;
        }
        let envelope = aabb_to_envelope(item.aabb());
        let removed_from_tree = self.tree.remove(&IndexedItem { id, envelope });
        debug_assert!(
            removed_from_tree.is_some(),
            "Node::remove: {id:?} was live in `items` but missing from the R-tree -- an add/remove bookkeeping bug"
        );
        self.removed[id.0] = true;
        self.live_count -= 1;
        item
    }

    /// Atomically removes `id` and inserts `new_item` in its place,
    /// **keeping the same [`ItemId`]** -- the primitive SHOVE needs to
    /// change a track's geometry in place without invalidating any
    /// other `ItemId` a caller (e.g. a collision-query result gathered
    /// just beforehand) might still be holding. Equivalent to (but
    /// distinct from) `let old = node.remove(id); node.add(new_item)`,
    /// which would hand back a *different*, larger `ItemId` for the
    /// replacement rather than reusing `id`.
    ///
    /// Same panic condition as [`Self::remove`].
    pub fn replace(&mut self, id: ItemId, new_item: Item) -> Item {
        // `self.remove` below already bumps `obstacle_revision` for
        // `old`'s own side of this swap if it wasn't a `Zone`; this
        // covers `new_item`'s side too (never both the same call --
        // `replace` is Alladin's `Item::Pad`-in-place-move primitive
        // today, never used for a `Zone`, but there's no reason to bake
        // that assumption in here rather than just checking).
        if !matches!(new_item, Item::Zone { .. }) {
            self.obstacle_revision += 1;
        }
        let old = self.remove(id);
        let envelope = aabb_to_envelope(new_item.aabb());
        self.items[id.0] = new_item;
        self.removed[id.0] = false;
        self.live_count += 1;
        self.tree.insert(IndexedItem { id, envelope });
        old
    }

    /// Broad-phase + exact collision query: which existing items does
    /// `candidate` collide with, under `resolver`'s clearance rules?
    /// Same-net pairs never collide (KiCad's `sameNet` fast path). Use
    /// [`Self::is_colliding`] instead if only a yes/no answer is needed
    /// (e.g. [`Self::path_is_clear`]) -- it short-circuits on the first
    /// hit rather than always collecting every one.
    pub fn query_colliding(&self, candidate: &Item, resolver: &dyn RuleResolver) -> Vec<ItemId> {
        let search_box = aabb_to_envelope(candidate.aabb().inflate(resolver.max_clearance()));
        self.tree
            .locate_in_envelope_intersecting(&search_box)
            .filter(|indexed| self.item_collides(candidate, indexed.id, resolver))
            .map(|indexed| indexed.id)
            .collect()
    }

    /// [`Self::query_colliding`]'s yes/no equivalent -- same broad-phase
    /// R-tree query and exact per-item collision rule, but returns as
    /// soon as the first collision is found instead of allocating a
    /// `Vec` and scanning every remaining candidate in the search box.
/// This is the one that actually matters for hot-path performance:
/// [`Self::path_is_clear`] is called for every candidate clearance
/// check -- often millions of times for a single net on a real,
/// densely-populated board -- and the overwhelming majority of those
/// calls are either answered by the very first nearby item (a genuine
/// collision) or have to confirm *no* collision by checking every
/// nearby item anyway, so the only real waste
/// `query_colliding().is_empty()` was paying for here was the `Vec`
/// allocation/push overhead on every single call, not the search
/// itself -- removing it is a real, measured part of what fixed a
/// routing hang on a real board (see
/// `alladin_geom::PolygonEdgeIndex`'s doc comment for the other,
/// larger half of that fix).
    pub fn is_colliding(&self, candidate: &Item, resolver: &dyn RuleResolver) -> bool {
        let search_box = aabb_to_envelope(candidate.aabb().inflate(resolver.max_clearance()));
        self.tree
            .locate_in_envelope_intersecting(&search_box)
            .any(|indexed| self.item_collides(candidate, indexed.id, resolver))
    }

    /// Shared per-item collision rule used by both
    /// [`Self::query_colliding`] and [`Self::is_colliding`]: same-net
    /// skip, shared-layer check, `Item::Zone` skip, then the exact
    /// geometry test.
    ///
    /// An [`Item::Zone`] `other` is *never* a collision here, on
    /// purpose, even against a different net -- a zone fill is only a
    /// point-in-time snapshot of "what a pour looked like when it was
    /// last (re)filled" (see `alladin_pcb::board_doc::ZoneRecord`'s own
    /// doc comment), not a live obstacle a new `Pad`/`Track`/`Via` must
    /// route/place around. Treating it as a hard blocker here would
    /// make it impossible to ever draw a second net's trace on a layer
    /// that already has so much as one full-board plane on it: *every*
    /// candidate path would collide with that plane's existing fill, no
    /// matter where it was drawn, since the fill has no "corridor" left
    /// open for a not-yet-existing track -- exactly the deadlock a real
    /// user hit routing an LED daisy-chain signal net across a board
    /// with both copper layers already poured solid. The same board's
    /// next [`Self::query_colliding`]/[`Self::is_colliding`] caller that
    /// actually cares about zone geometry (`alladin_pcb::zone_fill`'s
    /// own *obstacle* computation, i.e. the other direction -- what a
    /// *new* pour must clear around) never goes through this method at
    /// all, so this skip only ever affects "is `candidate` blocked by an
    /// existing pour", never "does a pour correctly avoid `candidate`".
    fn item_collides(&self, candidate: &Item, other_id: ItemId, resolver: &dyn RuleResolver) -> bool {
        let other = &self.items[other_id.0];

        if matches!(other, Item::Zone { .. }) {
            return false;
        }

        if candidate.net().is_some() && candidate.net() == other.net() {
            return false; // same net: never a collision (matches KiCad's fast path)
        }

        let (layer_a, layer_a2) = candidate.layers();
        if !other.on_layer(layer_a) && layer_a2.map_or(true, |l| !other.on_layer(l)) {
            return false; // no shared copper layer
        }

        let clearance = resolver.clearance(candidate, other);
        items_collide(candidate, other, clearance)
    }

    /// `other`'s exact collision check when `other` is an [`Item::Zone`]
    /// -- `candidate` is never a `Zone` itself here (see
    /// `items_collide`'s doc comment on its own `Zone` arm for why), so
    /// this only ever needs to handle `candidate` being a `Pad`/`Via`/
    /// `Track`.
    fn zone_collides(&self, candidate: &Item, zone_id: ItemId, outline: &Polygon, clearance: Unit) -> bool {
        let index = self.zone_edge_index(zone_id, outline);
        match candidate {
            Item::Pad { shape: PadShape::Circle(c), .. } => circle_polygon_collides_indexed(c, &index, clearance),
            Item::Pad { shape: PadShape::Polygon { outline: pad_outline, .. }, .. } => {
                polygon_polygon_collides_indexed(pad_outline, &index, clearance)
            }
            Item::Via { shape, .. } => circle_polygon_collides_indexed(shape, &index, clearance),
            Item::Track { shape, .. } => segment_polygon_collides_indexed(shape, &index, clearance),
            Item::Hole { position, drill } => circle_polygon_collides_indexed(&Circle::new(*position, drill / 2), &index, clearance),
            Item::Zone { .. } => false,
        }
    }

    /// Lazily builds (once) and caches a [`PolygonEdgeIndex`] for the
/// zone at `zone_id`, keyed by [`ItemId`] rather than the polygon's
/// own content, so repeat queries against the *same* zone -- exactly
/// what happens hundreds or thousands of times over during dense
/// clearance checks -- only ever pay the index's own
/// `O(vertex count)` build cost once, not once per query (see
/// [`PolygonEdgeIndex`]'s doc comment for why that distinction is what
/// actually fixed a real routing hang, not just a constant-factor
/// speedup). Safe to cache for `Node`'s whole lifetime: zones are never
/// mutated -- [`Self::replace`]/[`Self::remove`] are only ever used
/// on `Track`/`Via` items by SHOVE-style relocation -- so a cached
/// index can never go stale.
    fn zone_edge_index(&self, zone_id: ItemId, outline: &Polygon) -> Arc<PolygonEdgeIndex> {
        if let Some(existing) = self.zone_edge_index_cache.read().unwrap().get(&zone_id) {
            return Arc::clone(existing);
        }
        let index = Arc::new(PolygonEdgeIndex::build(outline));
        self.zone_edge_index_cache.write().unwrap().insert(zone_id, Arc::clone(&index));
        index
    }

    /// True if `candidate` (typically a not-yet-committed standalone
    /// [`Item::Via`]) geometrically touches -- zero clearance, actual
    /// contact/overlap, not merely "far enough from" the usual DRC
    /// minimum -- at least one existing live item that shares both its
    /// net and at least one copper layer. Deliberately the *opposite*
    /// question from [`Self::is_colliding`]/[`Self::query_colliding`]:
    /// those treat every same-net pair as automatically non-colliding
    /// (see [`Self::item_collides`]'s fast path), so a standalone via
    /// floating several millimetres away from every other item on its
    /// own net "collides with nothing" exactly as cleanly as one
    /// sitting squarely on top of the pad/track/zone it was actually
    /// meant to stitch -- both look identical to [`Self::query_colliding`].
    /// This is the one place same-net items are checked against *each
    /// other* geometrically, specifically so a caller (`alladin-pcb`'s
    /// "Place vias" GUI tool, its `add-via` CLI command) can refuse an
    /// electrically pointless, dangling via before it's ever created,
    /// rather than silently leaving one on the board that looks placed
    /// but touches nothing -- the exact same failure mode as a
    /// `track_dangling` DRC violation, just for vias and caught before
    /// export instead of after.
    ///
    /// `exclude`, if given, is skipped even if it would otherwise match
    /// -- needed when `candidate` is itself already live in this same
    /// `Node` (e.g. checked right after [`Self::add`]): without it, the
    /// query would trivially find the candidate's own already-inserted
    /// copy as a "same net, perfectly overlapping" match every time,
    /// making the whole check a no-op. Pass `None` when `candidate`
    /// hasn't been inserted yet.
    pub fn touches_same_net(&self, candidate: &Item, exclude: Option<ItemId>) -> bool {
        let Some(net) = candidate.net() else { return false };
        let (layer_a, layer_a2) = candidate.layers();
        let envelope = aabb_to_envelope(candidate.aabb());
        self.tree.locate_in_envelope_intersecting(&envelope).any(|indexed| {
            if Some(indexed.id) == exclude {
                return false;
            }
            let other = &self.items[indexed.id.0];
            if other.net() != Some(net) {
                return false;
            }
            if !other.on_layer(layer_a) && layer_a2.map_or(true, |l| !other.on_layer(l)) {
                return false;
            }
            if let Item::Zone { outline, .. } = other {
                self.zone_collides(candidate, indexed.id, outline, 0)
            } else {
                items_collide(candidate, other, 0)
            }
        })
    }

    /// Groups every live item on `net` into its physically-connected
    /// copper components -- zero-clearance touch, the same semantics as
    /// [`Self::touches_same_net`], generalized from "one candidate
    /// against everything else" to "everything against everything
    /// else". A net whose items don't all end up in one single returned
    /// group is only *logically* one net (e.g. via `connect_pins`,
    /// which never itself draws any copper) without every one of its
    /// pads actually being copper-reachable from each other -- exactly
    /// the gap a plain "which net is this pad on" view can't show, and
    /// the gap a fragmented zone/pour fill (see [`Item::Zone`]'s own
    /// doc comment on "filled islands") is the most common real cause
    /// of. `alladin-pcb`'s `board_summary` MCP tool reports exactly this
    /// split back to the user/an AI client.
    ///
    /// Deliberately plain `O(n^2)` pairwise touch tests plus
    /// union-find, not an R-tree broad phase -- this runs once per
    /// diagnostic call against one net's items (at most a few hundred
    /// even on a busy board), not a hot path like
    /// [`Self::query_colliding`], so the simpler implementation is
    /// worth it.
    pub fn net_copper_components(&self, net: NetId) -> Vec<Vec<ItemId>> {
        let ids: Vec<ItemId> = self.iter_with_ids().filter(|(_, item)| item.net() == Some(net)).map(|(id, _)| id).collect();

        let mut parent: Vec<usize> = (0..ids.len()).collect();
        fn find(parent: &mut [usize], x: usize) -> usize {
            if parent[x] != x {
                parent[x] = find(parent, parent[x]);
            }
            parent[x]
        }
        fn union(parent: &mut [usize], a: usize, b: usize) {
            let (ra, rb) = (find(parent, a), find(parent, b));
            if ra != rb {
                parent[ra] = rb;
            }
        }

        for i in 0..ids.len() {
            for j in (i + 1)..ids.len() {
                if find(&mut parent, i) != find(&mut parent, j) && self.items_touch(ids[i], ids[j]) {
                    union(&mut parent, i, j);
                }
            }
        }

        let mut groups: HashMap<usize, Vec<ItemId>> = HashMap::new();
        for (i, &id) in ids.iter().enumerate() {
            groups.entry(find(&mut parent, i)).or_default().push(id);
        }
        groups.into_values().collect()
    }

    /// Zero-clearance touch test between two arbitrary live items (by
    /// id) -- the pairwise primitive [`Self::net_copper_components`]
    /// unions over. Both are assumed to already share a net (the only
    /// caller pre-filters for that), but this still has to re-derive
    /// the answer [`Self::item_collides`] gets almost for free from its
    /// own same-net fast path: that path is a *skip*, not a *touch*
    /// test, so it's useless here. Also has to handle every pairing
    /// [`items_collide`] itself refuses to answer at all
    /// (`Item::Zone`-vs-anything, since a `Zone` is never the *moving*
    /// side of an ordinary collision query) by dispatching to
    /// [`Self::zone_collides`] (pad/via/track-vs-zone, reusing its
    /// cached [`PolygonEdgeIndex`]) or a plain [`polygon_polygon_collides`]
    /// call (zone-vs-zone, e.g. two separate pours on the same net
    /// physically overlapping) instead -- mirroring
    /// [`Self::touches_same_net`]'s own `Item::Zone` special case.
    fn items_touch(&self, a: ItemId, b: ItemId) -> bool {
        let item_a = &self.items[a.0];
        let item_b = &self.items[b.0];

        let (layer_a, layer_a2) = item_a.layers();
        if !item_b.on_layer(layer_a) && layer_a2.map_or(true, |l| !item_b.on_layer(l)) {
            return false; // no shared copper layer -- e.g. a via still touches both
        }

        match (item_a, item_b) {
            (Item::Zone { outline: outline_a, .. }, Item::Zone { outline: outline_b, .. }) => {
                polygon_polygon_collides(outline_a, outline_b, 0)
            }
            (Item::Zone { outline, .. }, _) => self.zone_collides(item_b, b, outline, 0),
            (_, Item::Zone { outline, .. }) => self.zone_collides(item_a, a, outline, 0),
            _ => items_collide(item_a, item_b, 0),
        }
    }

    /// Convenience: true if a straight track from `from` to `to` (given
    /// width, net, layer, class) would hit anything already in the world.
    /// Used by manual routing to validate each candidate 45° leg before
    /// committing it.
    pub fn path_is_clear(
        &self,
        from: Point,
        to: Point,
        width: Unit,
        net: Option<NetId>,
        layer: LayerId,
        class: NetClass,
        resolver: &dyn RuleResolver,
    ) -> bool {
        let probe = Item::Track {
            shape: Segment::new(from, to, width),
            net,
            layer,
            class,
        };
        !self.is_colliding(&probe, resolver)
    }
}

fn hole_circle(position: Point, drill: Unit) -> Circle {
    Circle::new(position, drill / 2)
}

/// The copper keep-out circle around a mounting hole: radius = the full
/// drill diameter, i.e. a copper-free annulus of `drill / 2` beyond the
/// drilled wall. That is sized for the screw *head* that will sit on
/// the board, not just the drill bit: a metric cap/pan head is ~1.7-1.9x
/// its thread diameter, and the drill is the thread plus ~0.2mm, so
/// head radius always fits inside this circle (an M3 head, 5.5mm wide,
/// inside the 6.4mm keep-out of its 3.2mm hole). A washer-sized flange
/// can still poke past it -- that stays the designer's own call. Used
/// for every hole-vs-copper check (pads, vias, tracks, pours); the
/// mechanical drill-to-drill rules ([`hole_circle`],
/// `BoardDoc::violates_hole_to_hole`, edge distance) stay on the real
/// drill, where a screw head is irrelevant.
fn hole_keepout_circle(position: Point, drill: Unit) -> Circle {
    Circle::new(position, drill)
}

fn items_collide(a: &Item, b: &Item, clearance: Unit) -> bool {
    match (a, b) {
        (Item::Pad { shape: s1, .. }, Item::Pad { shape: s2, .. }) => s1.collides_with(s2, clearance),
        (Item::Pad { shape: s, .. }, Item::Via { shape: c, .. })
        | (Item::Via { shape: c, .. }, Item::Pad { shape: s, .. }) => s.collides_with_circle(c, clearance),
        (Item::Via { shape: c1, .. }, Item::Via { shape: c2, .. }) => circle_circle_collides(c1, c2, clearance),
        (Item::Pad { shape: s, .. }, Item::Track { shape: seg, .. })
        | (Item::Track { shape: seg, .. }, Item::Pad { shape: s, .. }) => s.collides_with_segment(seg, clearance),
        (Item::Via { shape: c, .. }, Item::Track { shape: s, .. })
        | (Item::Track { shape: s, .. }, Item::Via { shape: c, .. }) => {
            circle_segment_collides(c, s, clearance)
        }
        (Item::Track { shape: s1, .. }, Item::Track { shape: s2, .. }) => {
            segment_segment_collides(s1, s2, clearance)
        }
        // `Item::Hole` is a plain circle (no copper, but geometrically
        // the same round obstacle a via already is) -- reuses exactly
        // the same primitives as the `Via` arms above, just built from
        // `position`/`drill` instead of `Via`'s own `shape`/`net`.
        // Copper items keep clear of the screw-head-sized keep-out
        // (see `hole_keepout_circle`), not just the bare drill --
        // otherwise the mounting screw's head would sit right on top
        // of live copper.
        (Item::Pad { shape: s, .. }, Item::Hole { position, drill })
        | (Item::Hole { position, drill }, Item::Pad { shape: s, .. }) => s.collides_with_circle(&hole_keepout_circle(*position, *drill), clearance),
        (Item::Via { shape: c, .. }, Item::Hole { position, drill })
        | (Item::Hole { position, drill }, Item::Via { shape: c, .. }) => circle_circle_collides(c, &hole_keepout_circle(*position, *drill), clearance),
        (Item::Track { shape: s, .. }, Item::Hole { position, drill })
        | (Item::Hole { position, drill }, Item::Track { shape: s, .. }) => circle_segment_collides(&hole_keepout_circle(*position, *drill), s, clearance),
        // Hole vs hole is a drill-bit rule, not a screw-head rule --
        // measured wall to wall on the real drills.
        (Item::Hole { position: p1, drill: d1 }, Item::Hole { position: p2, drill: d2 }) => {
            circle_circle_collides(&hole_circle(*p1, *d1), &hole_circle(*p2, *d2), clearance)
        }
        // Every `Item::Zone` pair is handled by `Node::query_colliding`
        // itself (see `Node::zone_collides`), via a cached
        // `PolygonEdgeIndex` rather than this function's plain,
        // unbounded-vertex-count geometry checks -- and `candidate` is
        // never a `Zone` to begin with (Alladin only ever routes
        // `Track`/`Via` items, so a `Zone` is never the moving
        // `candidate` side of a `query_colliding` call). `query_colliding`
        // never actually calls this function with either side being a
        // `Zone`; this arm exists purely for match exhaustiveness.
        (Item::Zone { .. }, _) | (_, Item::Zone { .. }) => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_geom::MM;

    /// Reproduces the exact scenario from the C++ feasibility probe
    /// (probe-build/probe_main.cpp): two same-net pads 5mm apart, plus a
    /// blocking obstacle (different net) dead-center between them.
    #[test]
    fn straight_path_blocked_but_detour_clear() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000); // JLCPCB 0.127mm min clearance

        let net1 = Some(NetId(1));
        let net2 = Some(NetId(2));

        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), 400_000)),
            net: net1,
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });
        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(5 * MM, 0), 400_000)),
            net: net1,
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });
        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(2_500_000, 0), 800_000)),
            net: net2,
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });

        assert!(!world.path_is_clear(
            Point::new(0, 0),
            Point::new(5 * MM, 0),
            250_000,
            net1,
            LayerId::FCu,
            NetClass::C,
            &resolver,
        ));

        assert!(world.path_is_clear(
            Point::new(0, 0),
            Point::new(2_500_000, 1_500_000),
            250_000,
            net1,
            LayerId::FCu,
            NetClass::C,
            &resolver,
        ));
    }

    #[test]
    fn same_net_never_collides() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);
        let net1 = Some(NetId(1));

        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), 400_000)),
            net: net1,
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });

        // A same-net track passing right through the pad must not be
        // flagged -- it's the pad's own connection.
        assert!(world.path_is_clear(
            Point::new(-1_000_000, 0),
            Point::new(1_000_000, 0),
            250_000,
            net1,
            LayerId::FCu,
            NetClass::C,
            &resolver,
        ));
    }

    #[test]
    fn net_copper_components_reports_one_group_when_a_track_bridges_two_pads() {
        let mut world = Node::new();
        let net = NetId(1);
        let pad_a = world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 400_000)), net: Some(net), layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal });
        let pad_b = world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(2 * MM, 0), 400_000)), net: Some(net), layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal });
        let track = world.add(Item::Track {
            shape: Segment::new(Point::new(0, 0), Point::new(2 * MM, 0), 200_000),
            net: Some(net),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let components = world.net_copper_components(net);
        assert_eq!(components.len(), 1, "a track touching both pads must merge them into one component");
        let mut ids = components[0].clone();
        ids.sort_by_key(|id| id.0);
        let mut expected = vec![pad_a, pad_b, track];
        expected.sort_by_key(|id| id.0);
        assert_eq!(ids, expected);
    }

    #[test]
    fn net_copper_components_splits_two_untouched_pads_on_the_same_net() {
        let mut world = Node::new();
        let net = NetId(1);
        world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 400_000)), net: Some(net), layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal });
        world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(10 * MM, 0), 400_000)), net: Some(net), layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal });

        let components = world.net_copper_components(net);
        assert_eq!(components.len(), 2, "two same-net pads with nothing physically bridging them must be reported as disconnected");
    }

    #[test]
    fn net_copper_components_bridges_a_fcu_pad_to_a_bcu_zone_island_through_a_via() {
        let mut world = Node::new();
        let net = NetId(1);
        let pad = world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 400_000)), net: Some(net), layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal });
        let stub = world.add(Item::Track {
            shape: Segment::new(Point::new(0, 0), Point::new(500_000, 0), 200_000),
            net: Some(net),
            layer: LayerId::FCu,
            class: NetClass::C,
        });
        let via = world.add(Item::Via { shape: Circle::new(Point::new(500_000, 0), 300_000), drill: 200_000, net: Some(net) });
        let island = world.add(Item::Zone {
            outline: Polygon::new(vec![
                Point::new(0, -MM),
                Point::new(MM, -MM),
                Point::new(MM, MM),
                Point::new(0, MM),
            ]),
            layer: LayerId::BCu,
            net: Some(net),
        });

        let mut components = world.net_copper_components(net);
        assert_eq!(components.len(), 1, "a via spanning both layers must bridge the F.Cu pad chain to the B.Cu zone island");
        let mut ids = components.remove(0);
        ids.sort_by_key(|id| id.0);
        let mut expected = vec![pad, stub, via, island];
        expected.sort_by_key(|id| id.0);
        assert_eq!(ids, expected);
    }

    #[test]
    fn net_copper_components_keeps_two_untouching_zone_islands_apart() {
        let mut world = Node::new();
        let net = NetId(1);
        let far_apart = 50 * MM;
        world.add(Item::Zone {
            outline: Polygon::new(vec![Point::new(0, 0), Point::new(MM, 0), Point::new(MM, MM), Point::new(0, MM)]),
            layer: LayerId::BCu,
            net: Some(net),
        });
        world.add(Item::Zone {
            outline: Polygon::new(vec![
                Point::new(far_apart, 0),
                Point::new(far_apart + MM, 0),
                Point::new(far_apart + MM, MM),
                Point::new(far_apart, MM),
            ]),
            layer: LayerId::BCu,
            net: Some(net),
        });

        let components = world.net_copper_components(net);
        assert_eq!(components.len(), 2, "two zone islands far apart on the same net must not be reported as one component");
    }

    #[test]
    fn jlcpcb_clearance_distinguishes_item_kinds() {
        // Sanity-check the ported constants map to the right pairs --
        // catches a swapped match arm more directly than the end-to-end
        // tests below.
        let resolver = JlcpcbClearance;
        let pad = Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)), net: None, layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal };
        let via = Item::Via {
            shape: Circle::new(Point::new(0, 0), 500_000),
            drill: 250_000,
            net: None,
        };
        let track = Item::Track {
            shape: Segment::new(Point::new(0, 0), Point::new(MM, 0), 0),
            net: None,
            layer: LayerId::FCu,
            class: NetClass::C,
        };

        assert_eq!(resolver.clearance(&pad, &track), JlcpcbClearance::PAD_TO_TRACK);
        assert_eq!(resolver.clearance(&track, &pad), JlcpcbClearance::PAD_TO_TRACK);
        assert_eq!(resolver.clearance(&via, &track), JlcpcbClearance::VIA_TO_TRACK);
        assert_eq!(resolver.clearance(&track, &via), JlcpcbClearance::VIA_TO_TRACK);
        assert_eq!(resolver.clearance(&track, &track), JlcpcbClearance::TRACK_TO_TRACK);
        assert_eq!(resolver.clearance(&pad, &via), JlcpcbClearance::PAD_TO_PAD);
        assert_eq!(resolver.clearance(&pad, &pad), JlcpcbClearance::PAD_TO_PAD);
        assert_eq!(resolver.max_clearance(), JlcpcbClearance::VIA_TO_TRACK);
    }

    #[test]
    fn jlcpcb_2layer_2oz_only_widens_track_to_track() {
        // The `2layer_2oz` Python variant overrides exactly
        // `min_track_width`/`min_track_spacing`/`soldermask_bridge` and
        // leaves `pad_to_track` and `smd_pad_to_pad_different_nets`
        // alone -- so of the four pairs Alladin's `RuleResolver` models,
        // only track-to-track should actually move versus the
        // `2layer_1oz` baseline.
        let resolver = Jlcpcb2Layer2Oz;
        let pad = Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)), net: None, layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal };
        let via = Item::Via {
            shape: Circle::new(Point::new(0, 0), 500_000),
            drill: 250_000,
            net: None,
        };
        let track = Item::Track {
            shape: Segment::new(Point::new(0, 0), Point::new(MM, 0), 0),
            net: None,
            layer: LayerId::FCu,
            class: NetClass::C,
        };

        assert_eq!(resolver.clearance(&pad, &track), JlcpcbClearance::PAD_TO_TRACK);
        assert_eq!(resolver.clearance(&via, &track), JlcpcbClearance::VIA_TO_TRACK);
        assert_eq!(resolver.clearance(&pad, &pad), JlcpcbClearance::PAD_TO_PAD);

        assert_eq!(Jlcpcb2Layer2Oz::TRACK_TO_TRACK, 160_000);
        assert_ne!(Jlcpcb2Layer2Oz::TRACK_TO_TRACK, JlcpcbClearance::TRACK_TO_TRACK);
        assert_eq!(resolver.clearance(&track, &track), 160_000);
        assert_eq!(resolver.max_clearance(), JlcpcbClearance::VIA_TO_TRACK);
    }

    #[test]
    fn jlcpcb_dfm_values_match_the_ported_python_profile() {
        // Spot-check against `jlc-ai-tools/jlc_dfm_rules.py`'s
        // `profile_2layer_1oz()` -- catches a transcription error more
        // directly than any behavioural test could.
        assert_eq!(JlcpcbDfm::MIN_TRACK_WIDTH, 100_000);
        assert_eq!(JlcpcbDfm::MIN_VIA_HOLE, 150_000);
        assert_eq!(JlcpcbDfm::MIN_VIA_DIAMETER, 250_000);
        assert_eq!(JlcpcbDfm::MIN_PTH_ANNULAR_RING, 180_000);
        assert_eq!(JlcpcbDfm::MIN_DRILL_DIAMETER, 150_000);
        assert_eq!(JlcpcbDfm::MIN_SMD_PAD_SIZE, 250_000);
        assert_eq!(JlcpcbDfm::COPPER_TO_ROUTED_EDGE, 200_000);
        assert_eq!(JlcpcbDfm::MIN_BOARD_DIMENSION, 3_000_000);

        // `min_track_width` and `TRACK_TO_TRACK` (spacing) coincide at
        // this specific copper weight, but are conceptually different
        // rows in the source table -- makes sure nobody "simplifies"
        // one into an alias of the other later.
        assert_eq!(JlcpcbDfm::MIN_TRACK_WIDTH, JlcpcbClearance::TRACK_TO_TRACK);
    }

    #[test]
    fn jlcpcb_dfm_check_via_rejects_a_legal_sized_pair_with_too_thin_annular_ring() {
        // The exact size pair that slipped through today's ESP32 board
        // merge: outer at the absolute MIN_VIA_DIAMETER floor, drill at
        // the absolute MIN_VIA_HOLE floor -- each scalar alone is "legal",
        // but `(0.25 - 0.15) / 2 = 0.05 mm` is half of MIN_VIA_ANNULAR_RING.
        // Before `check_via` existed, clamping diameter and drill
        // independently let this land on a board and JLCPCB's DFM viewer
        // flagged it as Danger.
        assert_eq!(
            JlcpcbDfm::check_via(JlcpcbDfm::MIN_VIA_DIAMETER, JlcpcbDfm::MIN_VIA_HOLE),
            Err(DfmViolation::ViaAnnularRingBelowMin)
        );
        // Hobby default 0.6/0.3 -- ring 0.15 mm, well above the floor.
        assert_eq!(JlcpcbDfm::check_via(600_000, 300_000), Ok(()));
        // Bare minimum that still clears the annular ring: drill 0.15
        // needs outer >= 0.15 + 2*0.10 = 0.35 mm.
        assert_eq!(JlcpcbDfm::check_via(350_000, 150_000), Ok(()));
        assert_eq!(JlcpcbDfm::check_via(349_000, 150_000), Err(DfmViolation::ViaAnnularRingBelowMin));
    }

    #[test]
    fn jlcpcb_dfm_check_track_width_rejects_below_minimum() {
        assert_eq!(JlcpcbDfm::check_track_width(JlcpcbDfm::MIN_TRACK_WIDTH), Ok(()));
        assert_eq!(JlcpcbDfm::check_track_width(JlcpcbDfm::MIN_TRACK_WIDTH - 1), Err(DfmViolation::TrackWidthBelowMin));
    }

    #[test]
    fn jlcpcb_dfm_check_smd_pad_rejects_below_minimum_and_accepts_the_floor() {
        assert_eq!(JlcpcbDfm::check_smd_pad(JlcpcbDfm::MIN_SMD_PAD_SIZE), Ok(()));
        assert_eq!(JlcpcbDfm::check_smd_pad(JlcpcbDfm::MIN_SMD_PAD_SIZE - 1), Err(DfmViolation::SmdPadBelowMin));
    }

    #[test]
    fn jlcpcb_dfm_check_pth_pad_walks_all_three_failure_modes() {
        // A 1.0mm pad over a 0.6mm drill: ring 0.2mm, above the 0.18mm
        // floor -- the smallest commonly-real geometry that passes.
        assert_eq!(JlcpcbDfm::check_pth_pad(1_000_000, 600_000), Ok(()));
        assert_eq!(JlcpcbDfm::check_pth_pad(1_000_000, JlcpcbDfm::MIN_DRILL_DIAMETER - 1), Err(DfmViolation::PthDrillBelowMin));
        assert_eq!(JlcpcbDfm::check_pth_pad(600_000, 600_000), Err(DfmViolation::PthDrillExceedsPad));
        // 0.9mm pad over 0.6mm drill: ring 0.15mm < 0.18mm.
        assert_eq!(JlcpcbDfm::check_pth_pad(900_000, 600_000), Err(DfmViolation::PthAnnularRingBelowMin));
    }

    #[test]
    fn jlcpcb_dfm_check_npth_hole_rejects_below_minimum() {
        assert_eq!(JlcpcbDfm::check_npth_hole(JlcpcbDfm::MIN_NPTH_HOLE), Ok(()));
        assert_eq!(JlcpcbDfm::check_npth_hole(JlcpcbDfm::MIN_NPTH_HOLE - 1), Err(DfmViolation::NpthHoleBelowMin));
    }

    #[test]
    fn jlcpcb_dfm_required_hole_to_hole_only_relaxes_for_a_genuinely_shared_net() {
        assert_eq!(JlcpcbDfm::required_hole_to_hole(Some(3), Some(3)), JlcpcbDfm::HOLE_TO_HOLE_SAME_NET);
        assert_eq!(JlcpcbDfm::required_hole_to_hole(Some(3), Some(4)), JlcpcbDfm::HOLE_TO_HOLE_DIFFERENT_NET);
        assert_eq!(JlcpcbDfm::required_hole_to_hole(Some(3), None), JlcpcbDfm::HOLE_TO_HOLE_DIFFERENT_NET, "a net-less NPTH hole never shares a net");
        assert_eq!(JlcpcbDfm::required_hole_to_hole(None, None), JlcpcbDfm::HOLE_TO_HOLE_DIFFERENT_NET);
    }

    #[test]
    fn jlcpcb_component_body_to_edge_matches_the_official_assembly_terms_page() {
        // Sourced from JLCPCB's own "Terms and Conditions of JLCPCB
        // Assembly Service" page (DFM note 3), not `jlc_dfm_rules.py`
        // -- a separate spot check from
        // `jlcpcb_dfm_values_match_the_ported_python_profile` above
        // since it isn't from that same source file, and is an
        // *assembly* rule, an order of magnitude larger than the
        // *fabrication* `COPPER_TO_ROUTED_EDGE` rule it's easy to
        // confuse it with.
        assert_eq!(JlcpcbDfm::COMPONENT_BODY_TO_EDGE, 2_500_000);
        assert!(
            JlcpcbDfm::COMPONENT_BODY_TO_EDGE > JlcpcbDfm::COPPER_TO_ROUTED_EDGE * 10,
            "the assembly body-to-edge rule must not be confused with the much smaller fabrication copper-to-edge rule"
        );
    }

    #[test]
    fn jlcpcb_assembly_body_and_lead_to_hole_clearances_are_0_3mm() {
        assert_eq!(JlcpcbDfm::COMPONENT_BODY_CLEARANCE, 300_000);
        assert_eq!(JlcpcbDfm::COMPONENT_LEAD_TO_HOLE, 300_000);
        assert!(
            JlcpcbDfm::COMPONENT_BODY_CLEARANCE > JlcpcbDfm::COPPER_TO_ROUTED_EDGE,
            "assembly body spacing must not be confused with the smaller fab copper-to-edge rule"
        );
    }

    #[test]
    fn jlcpcb_2layer_2oz_rejects_a_gap_the_1oz_profile_would_accept() {
        // Behaviour-changing regression, not just a constants check: a
        // 0.13mm true gap between two different-net tracks clears the
        // `2layer_1oz` 0.10mm minimum but not the `2layer_2oz` 0.16mm
        // one.
        let gap = 130_000; // 0.13mm
        let mut world = Node::new();
        world.add(Item::Track {
            shape: Segment::new(Point::new(0, -5 * MM), Point::new(0, 5 * MM), 200_000), // 0.2mm wide
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });
        // Neighbour track's near edge sits `gap` away from the first
        // track's edge (half-width 100_000 each side).
        let x = 100_000 + gap + 100_000;

        assert!(world.path_is_clear(
            Point::new(x, -MM), Point::new(x, MM), 200_000,
            Some(NetId(2)), LayerId::FCu, NetClass::C, &JlcpcbClearance,
        ));
        assert!(!world.path_is_clear(
            Point::new(x, -MM), Point::new(x, MM), 200_000,
            Some(NetId(2)), LayerId::FCu, NetClass::C, &Jlcpcb2Layer2Oz,
        ));
    }

    #[test]
    fn jlcpcb_clearance_is_stricter_for_vias_than_pads_at_the_same_gap() {
        // Behaviour-changing regression, not just a constants check: a
        // 0.12mm true gap sits *between* pad_to_track (0.10mm) and
        // via_hole_to_track (0.20mm) -- so the same physical layout must
        // come out clear for a pad neighbour but still blocked for a via
        // neighbour, purely because JlcpcbClearance looks at item kind.
        let gap = 120_000; // 0.12mm
        let pad_radius = 500_000;
        let resolver = JlcpcbClearance;

        let mut world_pad = Node::new();
        world_pad.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), pad_radius)),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });
        assert!(
            world_pad.path_is_clear(
                Point::new(pad_radius + gap, -MM),
                Point::new(pad_radius + gap, MM),
                0,
                Some(NetId(2)),
                LayerId::FCu,
                NetClass::C,
                &resolver,
            ),
            "0.12mm gap must clear the 0.10mm pad_to_track minimum"
        );

        let mut world_via = Node::new();
        world_via.add(Item::Via {
            shape: Circle::new(Point::new(0, 0), pad_radius),
            drill: pad_radius, // arbitrary for this test; only `shape` matters here
            net: Some(NetId(1)),
        });
        assert!(
            !world_via.path_is_clear(
                Point::new(pad_radius + gap, -MM),
                Point::new(pad_radius + gap, MM),
                0,
                Some(NetId(2)),
                LayerId::FCu,
                NetClass::C,
                &resolver,
            ),
            "the same 0.12mm gap must still violate the stricter 0.20mm via_hole_to_track minimum"
        );

        // And the old placeholder resolver would have blocked *both*
        // cases at its uniform 0.127mm -- confirming this is a real
        // behavioural change, not just a different number in the same
        // place.
        let fixed = FixedClearance(127_000);
        assert!(!world_pad.path_is_clear(
            Point::new(pad_radius + gap, -MM), Point::new(pad_radius + gap, MM),
            0, Some(NetId(2)), LayerId::FCu, NetClass::C, &fixed,
        ));
    }

    #[test]
    fn removed_item_disappears_from_iteration_queries_and_len() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);

        let pad_id = world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });
        assert_eq!(world.len(), 1);
        assert!(!world.path_is_clear(
            Point::new(-1_000_000, 0), Point::new(1_000_000, 0), 0,
            Some(NetId(2)), LayerId::FCu, NetClass::C, &resolver,
        ));

        let removed = world.remove(pad_id);
        assert!(matches!(removed, Item::Pad { .. }));

        assert_eq!(world.len(), 0);
        assert!(world.is_empty());
        assert_eq!(world.iter().count(), 0);
        assert_eq!(world.iter_with_ids().count(), 0);
        assert!(world.get(pad_id).is_none());
        assert!(world.query_region(Aabb::from_circle(&Circle::new(Point::new(0, 0), 10 * MM))).is_empty());
        assert!(world.path_is_clear(
            Point::new(-1_000_000, 0), Point::new(1_000_000, 0), 0,
            Some(NetId(2)), LayerId::FCu, NetClass::C, &resolver,
        ));
    }

    #[test]
    #[should_panic(expected = "was never added or already removed")]
    fn removing_an_already_removed_id_panics() {
        let mut world = Node::new();
        let id = world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)),
            net: None,
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });
        world.remove(id);
        world.remove(id); // second removal of the same id: caller bug, must panic
    }

    #[test]
    fn replace_keeps_the_same_item_id_but_moves_the_geometry() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);

        let track_id = world.add(Item::Track {
            shape: Segment::new(Point::new(0, 0), Point::new(5 * MM, 0), 300_000),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let old = world.replace(
            track_id,
            Item::Track {
                shape: Segment::new(Point::new(0, 5 * MM), Point::new(5 * MM, 5 * MM), 300_000),
                net: Some(NetId(1)),
                layer: LayerId::FCu,
                class: NetClass::C,
            },
        );
        assert!(matches!(old, Item::Track { .. }));

        // Same id, new data.
        match world.get(track_id).expect("replace must keep the id live") {
            Item::Track { shape, .. } => assert_eq!(shape.a, Point::new(0, 5 * MM)),
            other => panic!("expected a Track, got {other:?}"),
        }
        assert_eq!(world.len(), 1, "replace must not change the live item count");

        // The old position is now clear; the new position collides.
        assert!(world.path_is_clear(
            Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            Some(NetId(2)), LayerId::FCu, NetClass::C, &resolver,
        ));
        assert!(!world.path_is_clear(
            Point::new(0, 5 * MM), Point::new(5 * MM, 5 * MM), 250_000,
            Some(NetId(2)), LayerId::FCu, NetClass::C, &resolver,
        ));
    }

    #[test]
    fn iter_with_ids_pairs_each_item_with_its_own_stable_id() {
        let mut world = Node::new();
        let a = world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 100)), net: None, layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal });
        let b = world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(1 * MM, 0), 100)), net: None, layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal });
        world.remove(a);
        let ids: Vec<ItemId> = world.iter_with_ids().map(|(id, _)| id).collect();
        assert_eq!(ids, vec![b], "the removed item's id must not appear, and the survivor keeps its original id");
    }

    #[test]
    fn different_layer_never_collides() {
        let mut world = Node::new();
        let resolver = FixedClearance(127_000);

        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), 2 * MM)), // huge pad
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });

        // Same footprint, but routed on the back copper layer: must be
        // unaffected by the front-layer pad.
        assert!(world.path_is_clear(
            Point::new(-1_000_000, 0),
            Point::new(1_000_000, 0),
            250_000,
            Some(NetId(2)),
            LayerId::BCu,
            NetClass::C,
            &resolver,
        ));
    }

    fn square_zone(min_mm: f64, max_mm: f64, layer: LayerId, net: Option<NetId>) -> Item {
        let p = |x: f64, y: f64| Point::new((x * MM as f64) as Unit, (y * MM as f64) as Unit);
        Item::Zone {
            outline: Polygon::new(vec![
                p(min_mm, min_mm),
                p(max_mm, min_mm),
                p(max_mm, max_mm),
                p(min_mm, max_mm),
            ]),
            layer,
            net,
        }
    }

    #[test]
    fn jlcpcb_clearance_treats_a_zone_like_its_collision_partners_own_kind() {
        // A filled zone has no dedicated row in JLCPCB's capability
        // table -- it must fall back to whichever constant applies to
        // the *other* item in the pair (see `RuleResolver::clearance`'s
        // Zone match arms' doc comment).
        let resolver = JlcpcbClearance;
        let zone = square_zone(0.0, 10.0, LayerId::FCu, None);
        let pad = Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)), net: None, layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal };
        let via = Item::Via { shape: Circle::new(Point::new(0, 0), 500_000), drill: 250_000, net: None };
        let track = Item::Track {
            shape: Segment::new(Point::new(0, 0), Point::new(MM, 0), 0),
            net: None,
            layer: LayerId::FCu,
            class: NetClass::C,
        };

        assert_eq!(resolver.clearance(&zone, &track), JlcpcbClearance::TRACK_TO_TRACK);
        assert_eq!(resolver.clearance(&track, &zone), JlcpcbClearance::TRACK_TO_TRACK);
        assert_eq!(resolver.clearance(&zone, &pad), JlcpcbClearance::PAD_TO_TRACK);
        assert_eq!(resolver.clearance(&pad, &zone), JlcpcbClearance::PAD_TO_TRACK);
        assert_eq!(resolver.clearance(&zone, &via), JlcpcbClearance::PAD_TO_TRACK);
        assert_eq!(resolver.clearance(&via, &zone), JlcpcbClearance::PAD_TO_TRACK);
    }

    #[test]
    fn a_different_net_track_crossing_a_filled_zone_is_never_blocked() {
        // A zone fill is a point-in-time snapshot of an existing pour,
        // not a live obstacle -- routing a *different* net's track
        // straight through it must succeed (the same way dragging a
        // footprint under an already-filled plane must, per
        // `alladin_pcb::board_doc::check_placement`'s own doc comment).
        // Otherwise, the very first full-board pour on either layer of
        // a two-layer board would make it permanently impossible to
        // ever route any other net anywhere at all: every candidate
        // path would cross *some* pour, on *some* layer, with no
        // corridor left open for a track that doesn't exist yet.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(square_zone(0.0, 10.0, LayerId::FCu, Some(NetId(1)))); // a 10x10mm GND pour

        // Different net (2), straight line right through the pour.
        assert!(world.path_is_clear(
            Point::new(-5 * MM, 5 * MM),
            Point::new(15 * MM, 5 * MM),
            250_000,
            Some(NetId(2)),
            LayerId::FCu,
            NetClass::C,
            &resolver,
        ));

        // Well clear of the pour entirely: unaffected either way.
        assert!(world.path_is_clear(
            Point::new(20 * MM, 0),
            Point::new(20 * MM, 10 * MM),
            250_000,
            Some(NetId(2)),
            LayerId::FCu,
            NetClass::C,
            &resolver,
        ));
    }

    #[test]
    fn a_same_net_track_may_run_straight_through_its_own_zone() {
        // Design point #4 from the plan: a track sharing the zone's net
        // (e.g. a GND pour's own stitching track) must never collide
        // with it -- the existing same-net fast path in
        // `query_colliding` handles this automatically, no special-case
        // needed in `items_collide` itself.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;
        let net1 = Some(NetId(1));

        world.add(square_zone(0.0, 10.0, LayerId::FCu, net1));

        assert!(world.path_is_clear(
            Point::new(-5 * MM, 5 * MM),
            Point::new(15 * MM, 5 * MM),
            250_000,
            net1,
            LayerId::FCu,
            NetClass::C,
            &resolver,
        ));
    }

    #[test]
    fn a_netless_zone_never_blocks_anything_either() {
        // Same reasoning as `a_different_net_track_crossing_a_filled_zone_is_never_blocked`,
        // just for a `net: None` zone (today only ever a leftover empty
        // `zone_fill` stub, never a real user-facing "keepout area" --
        // `alladin-pcb::board_doc::add_zone` only ever takes a concrete
        // `NetId` -- but every `Item::Zone` skips this collision check
        // regardless of its own net, so this must hold for `None` too).
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(square_zone(0.0, 10.0, LayerId::FCu, None));

        assert!(world.path_is_clear(
            Point::new(-5 * MM, 5 * MM),
            Point::new(15 * MM, 5 * MM),
            250_000,
            Some(NetId(1)),
            LayerId::FCu,
            NetClass::C,
            &resolver,
        ));
        assert!(world.path_is_clear(
            Point::new(-5 * MM, 5 * MM),
            Point::new(15 * MM, 5 * MM),
            250_000,
            None, // a netless track too -- must not be treated any differently
            LayerId::FCu,
            NetClass::C,
            &resolver,
        ));
    }

    #[test]
    fn a_zone_on_a_different_layer_never_collides() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(square_zone(0.0, 10.0, LayerId::BCu, None)); // back-copper pour

        assert!(world.path_is_clear(
            Point::new(-5 * MM, 5 * MM),
            Point::new(15 * MM, 5 * MM),
            250_000,
            Some(NetId(1)),
            LayerId::FCu, // routing on the front layer
            NetClass::C,
            &resolver,
        ));
    }

    #[test]
    fn zone_aabb_matches_its_polygon_s_vertex_extent() {
        let zone = square_zone(0.0, 10.0, LayerId::FCu, None);
        let bb = zone.aabb();
        assert_eq!(bb.min, Point::new(0, 0));
        assert_eq!(bb.max, Point::new(10 * MM, 10 * MM));
    }

    /// An axis-aligned rectangular pad's true collision shape --
    /// `half_width`/`half_height` along x/y respectively -- for the
    /// `PadShape::Polygon` regression tests below. Deliberately
    /// axis-aligned (not rotated): rotation itself is already
    /// exhaustively covered by `alladin_geom`'s own
    /// `polygon_polygon_collides_is_invariant_under_a_shared_rotation`
    /// and `polygon_within_outline_with_clearance_rejects_a_rotated_pad_*`
    /// tests, so these tests instead focus on this crate's own
    /// responsibility: wiring `PadShape::Polygon` correctly into
    /// `items_collide`/`zone_collides`.
    fn rect_pad(center: Point, half_width: Unit, half_height: Unit) -> PadShape {
        PadShape::Polygon {
            outline: Polygon::new(vec![
                Point::new(center.x - half_width, center.y - half_height),
                Point::new(center.x + half_width, center.y - half_height),
                Point::new(center.x + half_width, center.y + half_height),
                Point::new(center.x - half_width, center.y + half_height),
            ]),
            center,
        }
    }

    #[test]
    fn polygon_pad_correctly_blocks_a_track_that_the_old_inscribed_circle_model_would_have_missed() {
        // The actual reported bug this whole slice fixes: a 4mm x 1mm
        // pad's old collision shape was an *inscribed* circle of radius
        // `min(4mm, 1mm) / 2` = 0.5mm -- much smaller than the pad's
        // true 2mm half-length along its long axis. A track running
        // along that long axis, comfortably outside the old 0.5mm
        // circle but still directly over the pad's real copper, must
        // now correctly collide.
        let resolver = JlcpcbClearance;
        let mut world = Node::new();
        world.add(Item::Pad {
            shape: rect_pad(Point::new(0, 0), 2 * MM, 500_000),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });

        assert!(
            !world.path_is_clear(
                Point::new(1_500_000, -5 * MM), // x=1.5mm: outside the old 0.5mm circle, inside the true 2mm pad
                Point::new(1_500_000, 5 * MM),
                200_000,
                Some(NetId(2)),
                LayerId::FCu,
                NetClass::C,
                &resolver,
            ),
            "a track directly over the pad's true (long-axis) copper must be blocked, not just outside the old inscribed circle"
        );
    }

    #[test]
    fn polygon_pad_vs_track_collides_exactly_at_the_pad_to_track_clearance_boundary() {
        let resolver = JlcpcbClearance;
        let clearance = JlcpcbClearance::PAD_TO_TRACK;
        let mut world = Node::new();
        world.add(Item::Pad {
            shape: rect_pad(Point::new(0, 0), 2 * MM, 500_000), // true right edge at x=2mm
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });

        let half_width = 100_000; // 0.1mm track
        let x_exactly_clear = 2 * MM + clearance + half_width;
        assert!(
            world.path_is_clear(
                Point::new(x_exactly_clear, -MM),
                Point::new(x_exactly_clear, MM),
                2 * half_width,
                Some(NetId(2)),
                LayerId::FCu,
                NetClass::C,
                &resolver,
            ),
            "exactly at the clearance boundary must pass (>=, not >)"
        );
        assert!(
            !world.path_is_clear(
                Point::new(x_exactly_clear - 1, -MM),
                Point::new(x_exactly_clear - 1, MM),
                2 * half_width,
                Some(NetId(2)),
                LayerId::FCu,
                NetClass::C,
                &resolver,
            ),
            "one internal unit closer than the clearance boundary must collide"
        );
    }

    #[test]
    fn polygon_pad_vs_polygon_pad_collides_exactly_at_the_pad_to_pad_clearance_boundary() {
        let resolver = JlcpcbClearance;
        let clearance = JlcpcbClearance::PAD_TO_PAD;
        let mut world = Node::new();
        let half_width = 2 * MM;
        world.add(Item::Pad {
            shape: rect_pad(Point::new(0, 0), half_width, 500_000),
            net: Some(NetId(1)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        });

        let neighbor_at_x = |x: Unit| Item::Pad {
            shape: rect_pad(Point::new(x, 0), half_width, 500_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        };
        // First pad's right edge at x=2mm; second pad's own left edge
        // (its own half-width away from its center) placed exactly
        // `clearance` beyond that.
        let x_exactly_clear = half_width + clearance + half_width;
        assert!(
            !world.is_colliding(&neighbor_at_x(x_exactly_clear), &resolver),
            "exactly at PAD_TO_PAD clearance: must not collide"
        );
        assert!(
            world.is_colliding(&neighbor_at_x(x_exactly_clear - 1), &resolver),
            "one internal unit closer than PAD_TO_PAD: must collide"
        );
    }

    #[test]
    fn polygon_pad_never_collides_with_a_zone_even_well_inside_the_old_pad_to_track_clearance() {
        // `JlcpcbClearance::clearance` still reports a real
        // `PAD_TO_TRACK` distance for a pad-vs-zone pair (see
        // `jlcpcb_clearance_treats_a_zone_like_its_collision_partners_own_kind`)
        // -- that resolver-level number is exactly what
        // `alladin_pcb::zone_fill::fill_zone` uses to carve a pour's
        // *own* keep-out around this same pad. But `Node::item_collides`
        // itself now skips every `Item::Zone` other unconditionally
        // (see its own doc comment), so a pad placed deep inside a
        // different-net pour's old clearance margin -- even overlapping
        // the fill outline itself -- must never be reported as a
        // collision.
        let resolver = JlcpcbClearance;
        let mut world = Node::new();
        world.add(square_zone(0.0, 10.0, LayerId::FCu, Some(NetId(1)))); // right edge at x=10mm

        let half_width = 2 * MM;
        let pad_at_x = |x: Unit| Item::Pad {
            shape: rect_pad(Point::new(x, 5 * MM), half_width, 500_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
        };

        assert!(!world.is_colliding(&pad_at_x(10 * MM), &resolver), "straddling the zone's own edge: must not collide");
        assert!(!world.is_colliding(&pad_at_x(5 * MM), &resolver), "sitting squarely inside the zone: must not collide");
    }

    #[test]
    fn a_mounting_hole_has_no_net_and_blocks_both_copper_layers() {
        let hole = Item::Hole { position: Point::new(0, 0), drill: 3 * MM / 2 };
        assert_eq!(hole.net(), None);
        assert_eq!(hole.layers(), (LayerId::FCu, Some(LayerId::BCu)));
    }

    #[test]
    fn a_track_on_either_copper_layer_is_blocked_by_a_mounting_hole() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;
        world.add(Item::Hole { position: Point::new(0, 0), drill: 2 * MM });

        for layer in [LayerId::FCu, LayerId::BCu] {
            assert!(
                !world.path_is_clear(Point::new(-5 * MM, 0), Point::new(5 * MM, 0), 250_000, Some(NetId(1)), layer, NetClass::C, &resolver),
                "a mounting hole must block a track through it on either copper layer, got a false clear on {layer:?}"
            );
        }

        // Well clear of the hole entirely: unaffected.
        assert!(world.path_is_clear(
            Point::new(-5 * MM, 10 * MM), Point::new(5 * MM, 10 * MM), 250_000,
            Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver,
        ));
    }

    #[test]
    fn a_mounting_hole_gets_the_stricter_via_hole_clearance_from_a_track_not_the_looser_pad_clearance() {
        // Same reasoning `jlcpcb_clearance_is_stricter_for_vias_than_pads_at_the_same_gap`
        // already covers for a real via: a hole's own drill is
        // mechanically drilled exactly like a via's, so it must use
        // `VIA_TO_TRACK`, not the looser `PAD_TO_TRACK`.
        let resolver = JlcpcbClearance;
        let hole = Item::Hole { position: Point::new(0, 0), drill: MM };
        let track = Item::Track { shape: Segment::new(Point::new(0, 0), Point::new(MM, 0), 0), net: None, layer: LayerId::FCu, class: NetClass::C };
        assert_eq!(resolver.clearance(&hole, &track), JlcpcbClearance::VIA_TO_TRACK);
        assert_eq!(resolver.clearance(&track, &hole), JlcpcbClearance::VIA_TO_TRACK);
    }

    #[test]
    fn a_mounting_hole_collides_with_a_pad_via_and_another_hole_at_the_expected_clearance() {
        let resolver = JlcpcbClearance;
        let mut world = Node::new();
        world.add(Item::Hole { position: Point::new(0, 0), drill: 2 * MM }); // drill radius 1mm, screw-head keep-out radius 2mm

        // Copper (pads/vias) clears against the screw-head keep-out
        // circle (radius = full drill diameter, see
        // `hole_keepout_circle`), not the bare drill wall.
        let clearance = JlcpcbClearance::PAD_TO_PAD;
        let x_exactly_clear = 2 * MM + clearance + 500_000; // keep-out radius + clearance + pad radius
        let pad_at = |x: Unit| Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(x, 0), 500_000)), net: Some(NetId(1)), layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal };
        assert!(!world.is_colliding(&pad_at(x_exactly_clear), &resolver), "exactly at PAD_TO_PAD clearance past the keep-out: must not collide");
        assert!(world.is_colliding(&pad_at(x_exactly_clear - 1), &resolver), "one internal unit closer: must collide");

        let via_at = |x: Unit| Item::Via { shape: Circle::new(Point::new(x, 0), 500_000), drill: 250_000, net: Some(NetId(2)) };
        assert!(!world.is_colliding(&via_at(x_exactly_clear), &resolver));
        assert!(world.is_colliding(&via_at(x_exactly_clear - 1), &resolver));

        // Hole vs hole stays a drill-bit rule on the real drill walls
        // (radius 1mm + clearance + radius 0.5mm) -- a screw head can't
        // short two platingless holes.
        let hole_exactly_clear = MM + clearance + 500_000;
        let hole_at = |x: Unit| Item::Hole { position: Point::new(x, 0), drill: MM };
        assert!(!world.is_colliding(&hole_at(hole_exactly_clear), &resolver), "two holes exactly at PAD_TO_PAD clearance wall-to-wall: must not collide");
        assert!(world.is_colliding(&hole_at(hole_exactly_clear - 1), &resolver), "two holes one internal unit closer: must collide");
    }

    #[test]
    fn a_filled_zone_never_blocks_a_mounting_hole_even_well_inside_the_old_pad_to_track_clearance() {
        // Same "zone fills never block anything" contract as
        // `polygon_pad_never_collides_with_a_zone_even_well_inside_the_old_pad_to_track_clearance`,
        // for `Item::Hole` instead of `Item::Pad`.
        let resolver = JlcpcbClearance;
        let mut world = Node::new();
        world.add(square_zone(0.0, 10.0, LayerId::FCu, Some(NetId(1)))); // right edge at x=10mm

        let hole_at_x = |x: Unit| Item::Hole { position: Point::new(x, 5 * MM), drill: 2 * MM };
        assert!(!world.is_colliding(&hole_at_x(10 * MM), &resolver), "straddling the zone's own edge: must not collide");
        assert!(!world.is_colliding(&hole_at_x(5 * MM), &resolver), "sitting squarely inside the zone: must not collide");
    }

    #[test]
    fn a_mounting_holes_aabb_covers_its_screw_head_keepout() {
        // The AABB must span the keep-out circle (radius = drill
        // diameter, 2mm here), not just the 1mm drill radius --
        // otherwise the spatial prefilter would skip exact checks
        // against the enlarged circle entirely.
        let hole = Item::Hole { position: Point::new(5 * MM, -3 * MM), drill: 2 * MM };
        let bb = hole.aabb();
        assert_eq!(bb.min, Point::new(3 * MM, -5 * MM));
        assert_eq!(bb.max, Point::new(7 * MM, -1 * MM));
    }

    #[test]
    fn touches_same_net_is_false_for_a_same_net_via_floating_clear_of_everything() {
        // The exact "dangling standalone via" case this method exists to
        // catch: `is_colliding`/`query_colliding` would call this via
        // perfectly placeable (same net as the pad, several mm away --
        // never even considered a collision candidate), but it doesn't
        // actually touch anything on its own net either.
        let mut world = Node::new();
        world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)), net: Some(NetId(1)), layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal });
        let far_via = Item::Via { shape: Circle::new(Point::new(10 * MM, 0), 300_000), drill: 150_000, net: Some(NetId(1)) };
        assert!(!world.touches_same_net(&far_via, None));
    }

    #[test]
    fn touches_same_net_is_true_for_a_via_overlapping_a_same_net_pad() {
        let mut world = Node::new();
        world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)), net: Some(NetId(1)), layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal });
        let touching_via = Item::Via { shape: Circle::new(Point::new(700_000, 0), 300_000), drill: 150_000, net: Some(NetId(1)) };
        assert!(world.touches_same_net(&touching_via, None));
    }

    #[test]
    fn touches_same_net_ignores_an_overlapping_pad_on_a_different_net() {
        let mut world = Node::new();
        world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)), net: Some(NetId(2)), layer: LayerId::FCu, zone_connection: ZoneConnection::Thermal });
        let via = Item::Via { shape: Circle::new(Point::new(700_000, 0), 300_000), drill: 150_000, net: Some(NetId(1)) };
        assert!(!world.touches_same_net(&via, None), "different net, however close, must not count as touching");
    }

    #[test]
    fn touches_same_net_is_true_for_a_via_sitting_inside_a_same_net_zone() {
        // The actual, intended real-world use case: a GND stitching via
        // dropped inside a GND copper pour.
        let mut world = Node::new();
        world.add(square_zone(0.0, 10.0, LayerId::BCu, Some(NetId(1))));
        let via = Item::Via { shape: Circle::new(Point::new(5 * MM, 5 * MM), 300_000), drill: 150_000, net: Some(NetId(1)) };
        assert!(world.touches_same_net(&via, None));
    }

    #[test]
    fn touches_same_net_ignores_a_same_net_item_on_the_wrong_copper_layer() {
        let mut world = Node::new();
        world.add(Item::Pad { shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)), net: Some(NetId(1)), layer: LayerId::BCu, zone_connection: ZoneConnection::Thermal });
        // A track only on F.Cu can never touch a B.Cu-only pad even if
        // it geometrically overlaps -- no shared copper layer.
        let track = Item::Track { shape: Segment::new(Point::new(0, 0), Point::new(MM, 0), 200_000), net: Some(NetId(1)), layer: LayerId::FCu, class: NetClass::C };
        assert!(!world.touches_same_net(&track, None));
    }

    #[test]
    fn touches_same_net_with_exclude_ignores_the_excluded_id_even_if_it_would_otherwise_match() {
        // The exact bug `exclude` exists to prevent: checking a
        // candidate that's already live in this same `Node` (as
        // `BoardDoc::try_add_stitching_via` does, right after its own
        // `try_add_via` call) would otherwise trivially find the
        // candidate's own just-inserted copy as a "same net, perfectly
        // overlapping" match every time, making the whole check a
        // permanent no-op.
        let mut world = Node::new();
        let id = world.add(Item::Via { shape: Circle::new(Point::new(0, 0), 300_000), drill: 150_000, net: Some(NetId(1)) });
        let same_item = world.get(id).unwrap().clone();
        assert!(world.touches_same_net(&same_item, None), "sanity check: without excluding, it must find itself");
        assert!(!world.touches_same_net(&same_item, Some(id)), "excluding its own id must leave nothing else to match");
    }
}
