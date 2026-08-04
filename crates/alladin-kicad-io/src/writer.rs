//! From-scratch `.kicad_pcb` writing -- the counterpart to
//! [`crate::import_kicad_pcb`] that doesn't need an existing file to
//! start from (unlike [`crate::export_appending_items`], which only ever
//! *appends* newly-routed tracks/vias into an already-real board; see
//! its own doc comment). This is what a board *created inside Alladin
//! PCB itself* (see the development log's "Teil 29" entry) needs to
//! become a real, openable `.kicad_pcb` -- there is no "original file"
//! to append to.
//!
//! **Ground-truthed, not guessed**: every structural piece here (the
//! `(kicad_pcb (version ...) (generator ...) ...)` header, the fixed
//! `(layers ...)` table, the `(setup ...)` boilerplate, a `(footprint
//! ...)` block's exact required sub-forms, a `(pad ...)` form's
//! shape/layer/net syntax) was copied from real files KiCad 9.0.2's own
//! `pcbnew` Python API wrote on this machine (`CreateEmptyBoard` for the
//! header/setup skeleton; a small hand-built board with one footprint,
//! two pad shapes, a track and a board outline for everything else) --
//! not reverse-engineered from documentation or older, possibly-stale
//! real-world files. See the development log's corresponding entry
//! for the exact commands. This matters because
//! [`crate::export_appending_items`]'s own doc comment openly admits its
//! output was *never actually confirmed to open in real KiCad* -- this
//! writer's output is, from its very first version, both opened via
//! `kicad-cli` and DRC-checked against the genuine KiCad 9 engine before
//! being trusted.
//!
//! **Deliberate scope cuts, same "state it, don't hide it" style as the
//! rest of this crate:**
//! - Every pad keeps its *own* true shape here (circle/rect/oval, with
//!   its own local rotation) -- unlike `alladin-pcb`'s internal
//!   collision/routing model, which approximates every pad as a circle
//!   for `alladin-router` (a *routing aid*, never the source of truth
//!   for what gets written to disk). The exported file is exactly what
//!   the user actually placed, so real KiCad's own DRC -- not Alladin's
//!   conservative internal approximation -- is the authoritative check
//!   once a board leaves Alladin.
//! - No footprint graphics (silkscreen/courtyard/fab outlines, 3D
//!   models) are emitted -- only the electrically/mechanically load-
//!   bearing forms (`pad`, the `Reference`/`Value` properties). A real
//!   footprint library entry has a lot more decoration; none of it
//!   changes DRC or manufacturability, so it's left for a later, purely
//!   cosmetic pass.
//! - The board outline is written as a closed chain of straight
//!   `gr_line`s (from `alladin_geom::Polygon::points`, which is already
//!   how `alladin-pcb` itself models a rounded-rect outline internally
//!   -- flattened into short straight segments, not literal arcs) --
//!   never `gr_arc`. This exactly matches what Alladin's own internal
//!   placement/DRC already reasons about, so there is no
//!   "what Alladin thinks the board looks like" vs. "what got written"
//!   mismatch to worry about.
//! - Track/via net classes (`alladin_core::NetClass`) have no `.kicad_pcb`
//!   representation (same fact `crate`'s own module doc already
//!   documents for the *importer*) and are silently dropped here too.
//! - Zones (`Item::Zone`) are written from an explicit `zones: &[WriteZone]`
//!   parameter, **not** read out of `node`: a zone's raw user-drawn
//!   outline (needed for the mandatory `(polygon ...)` sub-form) only
//!   lives in `alladin-pcb`'s own `ZoneRecord`, which a bare `Node`/
//!   `Item::Zone` (only ever the *filled* result) has no room for -- see
//!   [`WriteZone`]'s own doc comment.

use std::collections::BTreeMap;

use alladin_core::{Item, JlcpcbDfm, LayerId, Node};
use alladin_geom::{Point, Polygon, Unit, MM};
use alladin_sexpr::SExpr;

use crate::{at_with_rotation, layer_name, mm, mm_str, pad_layers, point_form, track_to_sexpr, via_to_sexpr, ImportError};

fn sym(s: impl Into<String>) -> SExpr {
    SExpr::Sym(s.into())
}
fn str_(s: impl Into<String>) -> SExpr {
    SExpr::Str(s.into())
}
fn list(items: Vec<SExpr>) -> SExpr {
    SExpr::List(items)
}
fn tag(head: &str, args: Vec<SExpr>) -> SExpr {
    let mut v = vec![sym(head)];
    v.extend(args);
    list(v)
}
fn uuid_sexpr() -> SExpr {
    tag("uuid", vec![str_(uuid::Uuid::new_v4().to_string())])
}

/// A pad's true visual/manufacturing shape -- see this module's doc
/// comment for why this, not `alladin_core::Item::Pad`'s always-a-circle
/// `shape`, is what actually gets written to disk.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WritePadShape {
    Circle { diameter: Unit },
    Rect { width: Unit, height: Unit },
    /// KiCad's own `oval` pad: width == height is a legal (if unusual)
    /// degenerate case that just renders as a circle -- deliberately not
    /// special-cased away here, so the caller never has to know that.
    Oval { width: Unit, height: Unit },
}

/// A pad's real electrical/mechanical mounting -- real KiCad's own
/// three-way `(pad "" <mount> ...)` split. Deliberately its own
/// explicit field on [`WritePad`] rather than still inferred purely
/// from `drill.is_some()` (`Some` used to mean "thru_hole"
/// unconditionally): a genuine unplated mechanical hole
/// (`alladin_core::Item::Hole`, see that type's doc comment) *also*
/// has a drill, but is neither electrically connected nor plated, so
/// `drill.is_some()` alone can no longer tell `ThruHole` and
/// `NpThruHole` apart. [`Self::Smd`] still implies `drill: None`
/// (nothing about an SMD pad is ever a hole at all), but the writer
/// itself trusts whatever this field says rather than re-deriving it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PadMount {
    Smd,
    ThruHole,
    /// A mechanical mounting-hole "pad" -- no net, no copper ring, just
    /// the drilled hole itself (`size` on disk equals `drill`, see
    /// [`pad_to_sexpr`]'s own construction of it). This is how a
    /// [`crate::HoleTemplate`]-derived hole (see
    /// `alladin-pcb::footprint::HoleTemplate`) becomes a real,
    /// footprint-owned `(pad ...)` form rather than a free top-level
    /// item, matching how real KiCad's own `MountingHole` library
    /// footprints are built.
    NpThruHole,
}

/// One pad of a [`WriteFootprint`], carrying everything a real
/// `(pad ...)` form needs that `alladin_core::Item::Pad` alone can't
/// supply (true shape, number, own rotation) -- exactly the fields
/// `alladin-pcb`'s own `PadTemplate` already tracks for rendering, see
/// that type's doc comment for why they exist at all.
pub struct WritePad {
    /// Empty (`""`) for a [`PadMount::NpThruHole`] mechanical hole --
    /// real KiCad's own mounting-hole footprints leave a np_thru_hole
    /// pad's number blank too, since it's never referenced by any pin
    /// number/netlist.
    pub number: String,
    /// Position relative to the footprint's own origin, in the
    /// footprint's *unrotated* local frame -- written to disk exactly
    /// as-is (see [`pad_to_sexpr`]'s doc comment for why: KiCad itself
    /// applies the footprint's placement rotation on load, so this
    /// writer never should). Matches `PadTemplate::offset`'s own
    /// contract exactly, so callers can pass that field straight
    /// through.
    pub offset: Point,
    pub shape: WritePadShape,
    /// The pad's own local rotation in degrees, *before* the
    /// footprint's placement rotation is applied -- matches
    /// `PadTemplate::rotation_deg`'s contract.
    pub rotation_deg: f64,
    /// The pad's real KiCad mounting -- see [`PadMount`]'s own doc
    /// comment for why this is now a separate, explicit field rather
    /// than inferred from [`Self::drill`] alone.
    pub mount: PadMount,
    /// `Some(drill_diameter)` for a [`PadMount::ThruHole`]/
    /// [`PadMount::NpThruHole`] pad, `None` for [`PadMount::Smd`].
    pub drill: Option<Unit>,
    /// Which copper side this (SMD) pad lives on. Ignored for
    /// through-hole/mechanical-hole pads, which are always on both
    /// (`*.Cu`).
    pub layer: LayerId,
    pub net: Option<(u32, String)>,
}

/// One placed part, ready to become a `(footprint ...)` form. Carries
/// its own pads with full shape fidelity (see [`WritePad`]) -- deliberately
/// *not* `alladin-pcb`'s own `PlacedFootprint`, which only remembers a
/// template *name* (see that type's doc comment); the caller is
/// responsible for resolving the name back to real pad geometry (the
/// same lookup `crate::app`'s rendering code already does) before
/// building this.
pub struct WriteFootprint {
    pub reference: String,
    pub value: String,
    pub position: Point,
    pub rotation_deg: f64,
    pub pads: Vec<WritePad>,
}

/// One user-drawn zone/pour, ready to become a `(zone ...)` form.
/// Unlike a track/via, a real zone form carries *two* distinct
/// geometries: `outline` (the raw boundary the user actually drew,
/// written to the mandatory `(polygon (pts ...))` sub-form every zone
/// has, filled or not) and `islands` (the already-computed fill result
/// -- one `(filled_polygon ...)` per disjoint island, the exact
/// convention [`crate::import_zone`] already reads on the way back in,
/// see that function's own doc comment for the real-file shape this was
/// ground-truthed against). Building this needs the raw drawn outline,
/// which a bare `alladin_core::Node`/`Item::Zone` (only ever the
/// *filled* result) has no room for -- callers reach for this from
/// `alladin-pcb`'s own `ZoneRecord`, the one place that outline still
/// lives.
pub struct WriteZone {
    pub outline: Polygon,
    pub layer: LayerId,
    pub net: Option<(u32, String)>,
    pub islands: Vec<Polygon>,
}

fn pts_form(points: &[Point]) -> SExpr {
    let mut items = vec![sym("pts")];
    items.extend(points.iter().map(|&p| point_form("xy", p)));
    list(items)
}

/// Builds one `(zone ...)` form -- the handful of non-geometry fields
/// (`hatch`/`connect_pads`/`min_thickness`/`fill`) are fixed, sane
/// defaults (same "copied from a real KiCad-written file, not guessed"
/// convention as this module's own header doc comment describes for the
/// rest of the writer) rather than anything Alladin's own zone-fill
/// pipeline (`crate::zone_fill` in `alladin-pcb`, thermal-relief-free by
/// design -- see that module's doc comment) actually models; real
/// KiCad's own "Zone Properties" dialog lets a user change any of these
/// after import, exactly like [`setup_section`]'s plot params.
fn zone_to_sexpr(zone: &WriteZone) -> SExpr {
    let (net_id, net_name) = zone.net.clone().unwrap_or((0, String::new()));
    let mut form = vec![
        sym("zone"),
        tag("net", vec![sym(net_id.to_string())]),
        tag("net_name", vec![str_(net_name)]),
        tag("layer", vec![str_(layer_name(zone.layer).to_string())]),
        uuid_sexpr(),
        tag("hatch", vec![sym("edge"), sym("0.5")]),
        tag("connect_pads", vec![tag("clearance", vec![sym("0")])]),
        tag("min_thickness", vec![sym("0.1")]),
        tag("filled_areas_thickness", vec![sym("no")]),
        tag("fill", vec![sym("yes"), tag("thermal_gap", vec![sym("0.5")]), tag("thermal_bridge_width", vec![sym("0.5")])]),
        tag("polygon", vec![pts_form(&zone.outline.points)]),
    ];
    let layer_str = layer_name(zone.layer).to_string();
    for island in &zone.islands {
        form.push(tag("filled_polygon", vec![tag("layer", vec![str_(layer_str.clone())]), pts_form(&island.points)]));
    }
    list(form)
}

fn pad_layers_sexpr(pad: &WritePad) -> SExpr {
    let names: Vec<SExpr> = match pad.mount {
        PadMount::ThruHole | PadMount::NpThruHole => vec![str_("*.Cu"), str_("*.Mask")],
        PadMount::Smd => match pad.layer {
            LayerId::FCu => vec![str_("F.Cu"), str_("F.Mask"), str_("F.Paste")],
            LayerId::BCu => vec![str_("B.Cu"), str_("B.Mask"), str_("B.Paste")],
        },
    };
    tag("layers", names)
}

/// Builds one `(pad ...)` form. `footprint_rotation_deg` is needed only
/// for the `at` form's *angle* (third value) -- **ground-truth verified
/// against real `pcbnew`** (see this module's doc comment and
/// the development log's corresponding entry): unlike a pad's
/// position (`pad.offset`, always written raw/untouched -- KiCad itself
/// applies the footprint's placement rotation to it on load, exactly
/// like `crate::rotate` already does on import, so pre-rotating it here
/// would double-rotate every pad on any rotated footprint), a pad's
/// on-disk rotation *angle* is the **sum** of its own local rotation and
/// the footprint's placement rotation, not the local rotation alone.
///
/// **The written angle is negated relative to Alladin's own internal
/// convention** (see the development log's "Rotationsrichtung"
/// entry for the full ground-truth measurement this is based on):
/// `alladin_pcb::footprint::pad_world_position`/`app::rotate_and_place`
/// rotate a positive `rotation_deg` the standard mathematical
/// counter-clockwise way (`x' = x cosθ - y sinθ`, `y' = x sinθ + y
/// cosθ`), self-consistently across Alladin's own rendering, collision
/// detection and routing -- but real `pcbnew`, loading the *exact same*
/// `(at x y θ)` form, rotates the opposite way (`x' = x cosθ + y sinθ`,
/// `y' = -x sinθ + y cosθ`, i.e. mathematically `R(-θ)`). Confirmed by
/// a standalone probe (one footprint, one pad at local offset (2mm, 0),
/// rotation 90°, plus two marker pads at the two candidate world
/// positions) run through real `kicad-cli pcb drc`: the pad lands at
/// (0, -2mm), not Alladin's own (0, +2mm). Left uncorrected, *every*
/// pad of *every* non-zero-rotation footprint silently lands at its
/// mirror-image position once loaded by real KiCad -- for a simple
/// 2-pad part this can mean the two pins' world positions swap
/// outright, so a track Alladin itself routed correctly (in its own,
/// internally-consistent frame) ends up landing squarely on the
/// *other*, wrong-net pad once manufactured -- a real, silent short
/// that was the root cause of a user-reported bug ("überlappende
/// Leiterbahnen"/shorted nets on a hand-routed board, confirmed via
/// real `kicad-cli pcb drc`: 8 `shorting_items` on that exact file, all
/// on rotated footprints, zero on unrotated ones). Negating *only* the
/// angle actually written to disk (not `alladin_pcb`'s own internal
/// rotation math, which stays self-consistent and correct for its own
/// rendering/collision/routing) is sufficient to fix this: unlike a
/// true mirror/reflection, negating a rotation angle is still a pure
/// rotation, just the other way round, and [`import_footprints`]'s
/// read side undoes the exact same negation on the way back in.
fn pad_to_sexpr(pad: &WritePad, footprint_rotation_deg: f64) -> SExpr {
    let mount = match pad.mount {
        PadMount::Smd => "smd",
        PadMount::ThruHole => "thru_hole",
        PadMount::NpThruHole => "np_thru_hole",
    };
    let (shape_name, size) = match pad.shape {
        WritePadShape::Circle { diameter } => ("circle", (diameter, diameter)),
        WritePadShape::Rect { width, height } => ("rect", (width, height)),
        WritePadShape::Oval { width, height } => ("oval", (width, height)),
    };

    let mut form = vec![sym("pad"), str_(pad.number.clone()), sym(mount), sym(shape_name)];

    let total_angle = -(pad.rotation_deg + footprint_rotation_deg);
    let at = if total_angle == 0.0 {
        vec![sym(mm_str(pad.offset.x)), sym(mm_str(pad.offset.y))]
    } else {
        vec![sym(mm_str(pad.offset.x)), sym(mm_str(pad.offset.y)), sym(format_deg(total_angle))]
    };
    form.push(tag("at", at));
    form.push(tag("size", vec![sym(mm_str(size.0)), sym(mm_str(size.1))]));
    if let Some(drill) = pad.drill {
        form.push(tag("drill", vec![sym(mm_str(drill))]));
    }
    form.push(pad_layers_sexpr(pad));
    if let Some((id, name)) = &pad.net {
        form.push(tag("net", vec![sym(id.to_string()), str_(name.clone())]));
    }
    form.push(uuid_sexpr());
    list(form)
}

fn format_deg(deg: f64) -> String {
    let s = format!("{deg:.6}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s.is_empty() || s == "-" || s == "-0" {
        // `-0.0` (e.g. negating an exact `0.0` rotation, see
        // `pad_to_sexpr`'s doc comment) formats as the literal string
        // "-0" once trimmed -- harmless to real KiCad's own parser,
        // but pointless noise on disk, so it's folded back to "0"
        // here rather than at every call site.
        "0".to_string()
    } else {
        s.to_string()
    }
}

/// How far a pad's silkscreen-relevant footprint reaches from its own
/// center, in any direction -- used only to keep the `Reference`/`Value`
/// text clear of the actual copper (see [`footprint_vertical_extent`]),
/// not for any DRC/collision purpose (that's `alladin-pcb`'s own,
/// separate circular approximation, see `crate::writer`'s module doc
/// comment). Deliberately the *circumscribing* reach here (unlike the
/// narrower, routing-motivated radius `alladin-pcb`'s `lcsc.rs` picks
/// for collision): silkscreen sitting a little further from the pad
/// than strictly necessary is harmless, whereas text clipping into a
/// pad is a real, DRC-flagged defect (`silk_over_copper`).
fn pad_reach(pad: &WritePad) -> Unit {
    match pad.shape {
        WritePadShape::Circle { diameter } => diameter / 2,
        WritePadShape::Rect { width, height } | WritePadShape::Oval { width, height } => {
            (((width as f64).powi(2) + (height as f64).powi(2)).sqrt() / 2.0).round() as Unit
        }
    }
}

/// The lowest and highest Y any pad plausibly reaches, in the
/// footprint's own local (unrotated) frame -- what [`footprint_to_sexpr`]
/// uses to place `Reference` above and `Value` below the part, clear of
/// every pad, *for any footprint's actual real size* rather than a
/// fixed guess (see this module's doc comment: no per-part special
/// casing). Falls back to a small default span for a footprint with no
/// pads at all (never actually happens in practice, but keeps this
/// total rather than partial).
fn footprint_vertical_extent(pads: &[WritePad]) -> (Unit, Unit) {
    let mm1 = MM;
    pads.iter()
        .map(|p| {
            let reach = pad_reach(p);
            (p.offset.y - reach, p.offset.y + reach)
        })
        .fold(None, |acc: Option<(Unit, Unit)>, (lo, hi)| match acc {
            Some((accl, acch)) => Some((accl.min(lo), acch.max(hi))),
            None => Some((lo, hi)),
        })
        .unwrap_or((-mm1, mm1))
}

/// `y` and `footprint_rotation_deg` follow the exact same on-disk
/// convention as [`pad_to_sexpr`]'s `at` form (a footprint property is
/// just another footprint-local child item, positioned/rotated by
/// KiCad's loader the same way a pad is): `y` is the raw, local,
/// pre-rotation offset, and the written angle is the property's own
/// local rotation (always `0` here -- Alladin never rotates silkscreen
/// text independently of its footprint) plus `footprint_rotation_deg`.
///
/// Stroke thickness is [`JlcpcbDfm::MIN_SILK_LINE_WIDTH`], **not** a
/// second hardcoded literal -- this used to write a bare `"0.15"` here,
/// which is exactly JLCPCB's own published *absolute* floor with zero
/// margin (see that constant's own doc comment for the "why not sit
/// right on the real minimum" reasoning); every visible Reference/Value
/// label on every footprint this crate writes was silently thinner than
/// the rest of this codebase's own DFM floor. `DEFAULT_SILK_TEXT_HEIGHT`'s
/// 1.27mm KiCad-default character size is left alone -- there is no
/// official minimum text *height* to buffer above, only the stroke
/// width this constant already covers.
fn footprint_property(name: &str, value: &str, y: Unit, footprint_rotation_deg: f64, hide: bool) -> SExpr {
    let mut form = vec![
        sym("property"),
        str_(name.to_string()),
        str_(value.to_string()),
        tag("at", vec![sym("0"), sym(mm_str(y)), sym(format_deg(footprint_rotation_deg))]),
    ];
    if hide {
        form.push(tag("hide", vec![sym("yes")]));
    }
    form.push(uuid_sexpr());
    form.push(tag(
        "effects",
        vec![tag("font", vec![tag("size", vec![sym("1.27"), sym("1.27")]), tag("thickness", vec![sym(mm_str(JlcpcbDfm::MIN_SILK_LINE_WIDTH))])])],
    ));
    list(form)
}

fn footprint_to_sexpr(fp: &WriteFootprint) -> SExpr {
    let margin = MM;
    let (min_y, max_y) = footprint_vertical_extent(&fp.pads);

    let mut form = vec![
        sym("footprint"),
        str_(String::new()),
        tag("layer", vec![str_("F.Cu")]),
        uuid_sexpr(),
        // Negated relative to `fp.rotation_deg` -- see `pad_to_sexpr`'s
        // doc comment for the ground-truth measurement behind this
        // (real `pcbnew` rotates a positive on-disk angle the opposite
        // way Alladin's own internal rendering/collision/routing does).
        tag("at", if fp.rotation_deg == 0.0 {
            vec![sym(mm_str(fp.position.x)), sym(mm_str(fp.position.y))]
        } else {
            vec![sym(mm_str(fp.position.x)), sym(mm_str(fp.position.y)), sym(format_deg(-fp.rotation_deg))]
        }),
        // Both hidden since the "Beschriftungen nur im Editor" slice:
        // reference designators are *editor-side* orientation aids --
        // Alladin's own GUI draws them -- and must never reach the
        // fabricated silkscreen (on a dense board they land on pads,
        // which JLCPCB's DFM check flags as silk-over-pad defects;
        // assembly needs only the BOM/CPL CSVs, never printed refs).
        // Only deliberately placed annotations ([`WriteSilkLine`]/
        // [`WriteSilkDot`]) print. The properties are still *written*
        // (KiCad requires them, and re-imports keep the names) --
        // they're just never plotted.
        footprint_property("Reference", &fp.reference, min_y - margin, -fp.rotation_deg, true),
        footprint_property("Value", &fp.value, max_y + margin, -fp.rotation_deg, true),
        footprint_property("Datasheet", "", 0, -fp.rotation_deg, true),
        footprint_property("Description", "", 0, -fp.rotation_deg, true),
    ];

    for pad in &fp.pads {
        form.push(pad_to_sexpr(pad, fp.rotation_deg));
    }

    form.push(tag("embedded_fonts", vec![sym("no")]));
    list(form)
}

/// One straight silkscreen stroke segment -- what Alladin writes for
/// user-placed silk text after baking the embedded Hershey stroke font
/// into geometry. Becomes a top-level `(gr_line ...)` on `F.SilkS`/
/// `B.SilkS`. Using strokes (not `(gr_text ...)`) means KiCad displays
/// and plots the *same* glyph shapes Alladin previewed, with no
/// dependency on KiCad's own Newstroke font.
#[derive(Debug, Clone, PartialEq)]
pub struct WriteSilkLine {
    pub start: Point,
    pub end: Point,
    pub width: Unit,
    pub layer: LayerId,
}

/// `F.SilkS`/`B.SilkS` for silk primitives' [`LayerId`] -- distinct from
/// [`layer_name`] (which only ever names a *copper* layer, `F.Cu`/
/// `B.Cu`): the silk write types reuse [`LayerId`] purely as
/// "which side of the board", which side's *silkscreen* layer name
/// this then maps to.
fn silk_layer_name(layer: LayerId) -> &'static str {
    match layer {
        LayerId::FCu => "F.SilkS",
        LayerId::BCu => "B.SilkS",
    }
}

fn silk_line_to_sexpr(line: &WriteSilkLine) -> SExpr {
    tag(
        "gr_line",
        vec![
            point_form("start", line.start),
            point_form("end", line.end),
            tag("stroke", vec![tag("width", vec![sym(mm_str(line.width))]), tag("type", vec![sym("solid")])]),
            tag("layer", vec![str_(silk_layer_name(line.layer))]),
            uuid_sexpr(),
        ],
    )
}

/// One deliberately placed, filled silkscreen dot (a free dot or a
/// footprint's pin-1 marker -- by the time it reaches this crate the
/// distinction is gone, both are just ink) -- the round counterpart of
/// [`WriteSilkLine`], for the same no-dependency-cycle reason its own
/// small struct. Becomes a top-level filled `(gr_circle ...)`: a
/// primitive KiCad/Gerber reproduce *exactly* as drawn, no font
/// involved, so preview and fabrication output are trivially the same
/// shape.
#[derive(Debug, Clone, PartialEq)]
pub struct WriteSilkDot {
    pub center: Point,
    pub diameter: Unit,
    pub layer: LayerId,
}

fn silk_dot_to_sexpr(d: &WriteSilkDot) -> SExpr {
    tag(
        "gr_circle",
        vec![
            tag("center", vec![sym(mm_str(d.center.x)), sym(mm_str(d.center.y))]),
            // KiCad defines a circle by center + one point on it; +x is
            // as good as any.
            tag("end", vec![sym(mm_str(d.center.x + d.diameter / 2)), sym(mm_str(d.center.y))]),
            // Zero-width solid stroke + fill: the printed ink is exactly
            // the `diameter` circle, not a hair fatter.
            tag("stroke", vec![tag("width", vec![sym("0")]), tag("type", vec![sym("solid")])]),
            tag("fill", vec![sym("yes")]),
            tag("layer", vec![str_(silk_layer_name(d.layer))]),
            uuid_sexpr(),
        ],
    )
}

fn general_section() -> SExpr {
    tag("general", vec![tag("thickness", vec![sym("1.6")]), tag("legacy_teardrops", vec![sym("no")])])
}

/// The fixed 24-layer stackup every 2-layer (or fewer) KiCad board
/// declares regardless of how many are actually *used* -- copied
/// verbatim from `pcbnew.CreateEmptyBoard()`'s own output (see this
/// module's doc comment). `alladin-pcb` only ever places copper on
/// `F.Cu`/`B.Cu` today (see `alladin_core::LayerId`), but every one of
/// these entries still has to exist for the file to be a *structurally*
/// normal KiCad board -- e.g. `Edge.Cuts` (id 25) is what the outline
/// below is drawn on.
fn layers_section() -> SExpr {
    let rows: &[(i32, &str, &str, Option<&str>)] = &[
        (0, "F.Cu", "signal", None),
        (2, "B.Cu", "signal", None),
        (9, "F.Adhes", "user", Some("F.Adhesive")),
        (11, "B.Adhes", "user", Some("B.Adhesive")),
        (13, "F.Paste", "user", None),
        (15, "B.Paste", "user", None),
        (5, "F.SilkS", "user", Some("F.Silkscreen")),
        (7, "B.SilkS", "user", Some("B.Silkscreen")),
        (1, "F.Mask", "user", None),
        (3, "B.Mask", "user", None),
        (17, "Dwgs.User", "user", Some("User.Drawings")),
        (19, "Cmts.User", "user", Some("User.Comments")),
        (21, "Eco1.User", "user", Some("User.Eco1")),
        (23, "Eco2.User", "user", Some("User.Eco2")),
        (25, "Edge.Cuts", "user", None),
        (27, "Margin", "user", None),
        (31, "F.CrtYd", "user", Some("F.Courtyard")),
        (29, "B.CrtYd", "user", Some("B.Courtyard")),
        (35, "F.Fab", "user", None),
        (33, "B.Fab", "user", None),
        (39, "User.1", "user", None),
        (41, "User.2", "user", None),
        (43, "User.3", "user", None),
        (45, "User.4", "user", None),
    ];
    let mut items = vec![sym("layers")];
    for (id, name, kind, alt) in rows {
        let mut row = vec![sym(id.to_string()), str_(name.to_string()), sym(kind.to_string())];
        if let Some(a) = alt {
            row.push(str_(a.to_string()));
        }
        items.push(list(row));
    }
    list(items)
}

/// `pcbplotparams`' ~30 fields are all fixed, sane KiCad defaults --
/// copied verbatim from `pcbnew.CreateEmptyBoard()`'s own output (see
/// this module's doc comment). None of them affect DRC; they only
/// matter once a user actually plots/exports from *within* KiCad, at
/// which point KiCad's own "Plot" dialog lets them change any of these
/// interactively anyway.
fn setup_section() -> SExpr {
    let plot_params = tag(
        "pcbplotparams",
        vec![
            tag("layerselection", vec![sym("0x00000000_00000000_55555555_5755f5ff")]),
            tag("plot_on_all_layers_selection", vec![sym("0x00000000_00000000_00000000_00000000")]),
            tag("disableapertmacros", vec![sym("no")]),
            tag("usegerberextensions", vec![sym("no")]),
            tag("usegerberattributes", vec![sym("yes")]),
            tag("usegerberadvancedattributes", vec![sym("yes")]),
            tag("creategerberjobfile", vec![sym("yes")]),
            tag("dashed_line_dash_ratio", vec![sym("12.000000")]),
            tag("dashed_line_gap_ratio", vec![sym("3.000000")]),
            tag("svgprecision", vec![sym("4")]),
            tag("plotframeref", vec![sym("no")]),
            tag("mode", vec![sym("1")]),
            tag("useauxorigin", vec![sym("no")]),
            tag("hpglpennumber", vec![sym("1")]),
            tag("hpglpenspeed", vec![sym("20")]),
            tag("hpglpendiameter", vec![sym("15.000000")]),
            tag("pdf_front_fp_property_popups", vec![sym("yes")]),
            tag("pdf_back_fp_property_popups", vec![sym("yes")]),
            tag("pdf_metadata", vec![sym("yes")]),
            tag("pdf_single_document", vec![sym("no")]),
            tag("dxfpolygonmode", vec![sym("yes")]),
            tag("dxfimperialunits", vec![sym("yes")]),
            tag("dxfusepcbnewfont", vec![sym("yes")]),
            tag("psnegative", vec![sym("no")]),
            tag("psa4output", vec![sym("no")]),
            tag("plot_black_and_white", vec![sym("yes")]),
            tag("sketchpadsonfab", vec![sym("no")]),
            tag("plotpadnumbers", vec![sym("no")]),
            tag("hidednponfab", vec![sym("no")]),
            tag("sketchdnponfab", vec![sym("yes")]),
            tag("crossoutdnponfab", vec![sym("yes")]),
            tag("subtractmaskfromsilk", vec![sym("no")]),
            tag("outputformat", vec![sym("1")]),
            tag("mirror", vec![sym("no")]),
            tag("drillshape", vec![sym("1")]),
            tag("scaleselection", vec![sym("1")]),
            tag("outputdirectory", vec![str_("")]),
        ],
    );
    tag(
        "setup",
        vec![
            tag("pad_to_mask_clearance", vec![sym("0")]),
            tag("allow_soldermask_bridges_in_footprints", vec![sym("no")]),
            tag("tenting", vec![sym("front"), sym("back")]),
            plot_params,
        ],
    )
}

fn outline_to_gr_lines(outline: &[Polygon]) -> Vec<SExpr> {
    let mut out = Vec::new();
    for poly in outline {
        let n = poly.points.len();
        if n < 2 {
            continue;
        }
        for i in 0..n {
            let a = poly.points[i];
            let b = poly.points[(i + 1) % n];
            out.push(tag(
                "gr_line",
                vec![
                    point_form("start", a),
                    point_form("end", b),
                    tag("stroke", vec![tag("width", vec![sym("0.1")]), tag("type", vec![sym("default")])]),
                    tag("layer", vec![str_("Edge.Cuts")]),
                    uuid_sexpr(),
                ],
            ));
        }
    }
    out
}

/// Writes a complete, standalone `.kicad_pcb` file from scratch -- no
/// pre-existing file needed (see this module's doc comment for why that
/// makes this distinct from [`crate::export_appending_items`]).
///
/// `nets` must list every net id actually referenced by `footprints` or
/// `node` (net 0, "no net", is always implicit and must **not** be
/// included) -- any pad/track/via whose net id isn't in this list would
/// produce a file KiCad rejects as referencing an undefined net, exactly
/// the self-consistency hazard `export_appending_items`'s own doc
/// comment already calls out for the append case.
///
/// `node` supplies tracks and vias (`Item::Pad`/`Item::Zone` entries in
/// it are ignored: pads come from `footprints` instead, with real shape
/// fidelity `Item::Pad` alone can't carry -- see [`WritePad`]'s doc
/// comment -- and zones come from the separate `zones` parameter
/// instead, see [`WriteZone`]'s own doc comment for why).
pub fn write_kicad_pcb(
    outline: &[Polygon],
    footprints: &[WriteFootprint],
    node: &Node,
    nets: &[(u32, String)],
    zones: &[WriteZone],
    silk_lines: &[WriteSilkLine],
    silk_dots: &[WriteSilkDot],
) -> String {
    let mut top = vec![
        sym("kicad_pcb"),
        tag("version", vec![sym("20241229")]),
        tag("generator", vec![str_("alladin-pcb")]),
        tag("generator_version", vec![str_(env!("CARGO_PKG_VERSION"))]),
        general_section(),
        tag("paper", vec![str_("A4")]),
        layers_section(),
        setup_section(),
    ];

    top.push(tag("net", vec![sym("0"), str_("")]));
    let mut seen_nets: BTreeMap<u32, ()> = BTreeMap::new();
    for (id, name) in nets {
        seen_nets.insert(*id, ());
        top.push(tag("net", vec![sym(id.to_string()), str_(name.clone())]));
    }

    for fp in footprints {
        top.push(footprint_to_sexpr(fp));
    }

    top.extend(outline_to_gr_lines(outline));

    for line in silk_lines {
        top.push(silk_line_to_sexpr(line));
    }

    for d in silk_dots {
        top.push(silk_dot_to_sexpr(d));
    }

    for zone in zones {
        if let Some((id, _)) = &zone.net {
            debug_assert!(seen_nets.contains_key(id), "zone references an undeclared net");
        }
        top.push(zone_to_sexpr(zone));
    }

    for item in node.iter() {
        match item {
            Item::Track { shape, net, layer, .. } => {
                debug_assert!(net.is_none_or(|n| seen_nets.contains_key(&n.0)), "track references an undeclared net");
                top.push(track_to_sexpr(shape, *net, *layer));
            }
            Item::Via { shape, drill, net } => {
                debug_assert!(net.is_none_or(|n| seen_nets.contains_key(&n.0)), "via references an undeclared net");
                top.push(via_to_sexpr(shape, *drill, *net));
            }
            // Both footprint-owned, not free top-level items: pads
            // come from `footprints` (`WritePad`) and mounting holes
            // are written the exact same way, as `np_thru_hole` pads
            // inside their own footprint's block (see `WritePad::is_npth`).
            Item::Pad { .. } | Item::Hole { .. } => {}
            Item::Zone { .. } => {}
        }
    }

    top.push(tag("embedded_fonts", vec![sym("no")]));
    list(top).to_string()
}

/// One placed part read back out of a real `.kicad_pcb` file, with full
/// pad-shape fidelity -- the *read*-side counterpart of [`WriteFootprint`].
///
/// Deliberately separate from [`crate::import_kicad_pcb`]'s `ImportedBoard`,
/// which flattens every footprint's pads into bare circle-approximated
/// `Item::Pad`s in a single `Node` (fine for `alladin-router`'s routing
/// obstacles, useless for reconstructing *editable* parts: footprint
/// boundaries, reference designators, real pad shapes/numbers/rotations
/// and THT drill sizes are all erased in that path). This function exists
/// specifically for `alladin-pcb`'s round-trip import: turning a
/// (possibly hand-edited-in-real-KiCad) `.kicad_pcb` file back into
/// `PlacedFootprint`/`FootprintTemplate` data the editor can keep working
/// with, not just route around.
pub struct ImportedFootprint {
    pub reference: String,
    pub value: String,
    pub position: Point,
    pub rotation_deg: f64,
    pub pads: Vec<ImportedPad>,
    /// Every `np_thru_hole` pad form of this footprint, read back as a
    /// mechanical [`ImportedHole`] rather than an [`ImportedPad`] --
    /// see [`import_footprints`]'s own doc comment for where that
    /// split happens. Empty for every footprint with no mechanical
    /// holes of its own, which remains every electrical part.
    pub holes: Vec<ImportedHole>,
}

/// One mechanical hole of an [`ImportedFootprint`] -- the read-side
/// mirror of a [`PadMount::NpThruHole`] [`WritePad`], with only the
/// two facts that mount actually carries: its local offset (same
/// pre-footprint-rotation frame as [`ImportedPad::offset`]) and its
/// drill diameter. No number/shape/net/layer at all, unlike
/// [`ImportedPad`] -- a real `np_thru_hole` pad's own `size` always
/// equals its `drill` (a bare round hole, no annular ring), and it
/// never has a net, so there is nothing else to recover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImportedHole {
    pub offset: Point,
    pub drill: Unit,
}

/// One pad of an [`ImportedFootprint`]. `offset` and `rotation_deg` are
/// already in the same *local, pre-footprint-rotation* frame
/// [`WritePad`] expects -- the stored on-disk angle (footprint rotation
/// plus pad's own local rotation, see [`pad_to_sexpr`]'s doc comment) has
/// already had the footprint's own rotation subtracted back out here, so
/// callers can hand this struct's fields straight to a new `WritePad`/
/// `PadTemplate` unchanged, without redoing that arithmetic themselves.
pub struct ImportedPad {
    pub number: String,
    pub offset: Point,
    pub shape: WritePadShape,
    pub rotation_deg: f64,
    pub drill: Option<Unit>,
    pub layer: LayerId,
    pub net: Option<(u32, String)>,
}

fn property_text<'a>(footprint: &'a SExpr, name: &str) -> Option<&'a str> {
    footprint.children("property").find_map(|p| {
        let args = p.tagged("property")?;
        if args.first()?.text()? == name {
            args.get(1)?.text()
        } else {
            None
        }
    })
}

/// Reads a `(pad ...)` form's true shape/size -- the read-side mirror of
/// [`pad_to_sexpr`]'s `shape_name`/`size` writing. `roundrect`, `trapezoid`
/// and `custom` (any real pad shape this crate has no dedicated model for,
/// same limitation `crate::lcsc`'s own footprint parser already documents
/// for the same reason) fall back to a plain rectangle of the same
/// bounding size -- stated here, not hidden.
fn import_pad_shape(pad: &SExpr) -> WritePadShape {
    let shape_name = pad.tagged("pad").and_then(|a| a.get(2)).and_then(SExpr::text).unwrap_or("circle");
    let (w, h) = pad
        .child("size")
        .and_then(|s| s.tagged("size"))
        .map(|args| {
            let w = args.first().and_then(SExpr::as_f64).map(mm).unwrap_or(0);
            let h = args.get(1).and_then(SExpr::as_f64).map(mm).unwrap_or(w);
            (w, h)
        })
        .unwrap_or((0, 0));
    match shape_name {
        "circle" => WritePadShape::Circle { diameter: w },
        "oval" => WritePadShape::Oval { width: w, height: h },
        _ => WritePadShape::Rect { width: w, height: h },
    }
}

/// A `(pad ...)` form's real mounting string (`smd`/`thru_hole`/
/// `np_thru_hole`) -- the read-side mirror of [`pad_to_sexpr`]'s own
/// `mount` string, and what [`import_footprints`] switches on to
/// decide whether a given pad form becomes an [`ImportedPad`] or an
/// [`ImportedHole`].
fn pad_mount_text(pad: &SExpr) -> &str {
    pad.tagged("pad").and_then(|a| a.get(1)).and_then(SExpr::text).unwrap_or("smd")
}

fn pad_drill(pad: &SExpr) -> Option<Unit> {
    pad.child("drill").and_then(|d| d.tagged("drill")).and_then(|args| args.first().and_then(SExpr::as_f64)).map(mm)
}

fn import_pad(pad: &SExpr, footprint_rotation_deg: f64) -> ImportedPad {
    let number = pad.tagged("pad").and_then(|a| a.first()).and_then(SExpr::text).unwrap_or("").to_string();
    let is_thru_hole = pad_mount_text(pad) == "thru_hole";
    let (local_offset, stored_angle) = at_with_rotation(pad, "at");
    let drill = is_thru_hole.then(|| pad_drill(pad)).flatten();
    let (front, back) = pad_layers(pad);
    let layer = if back && !front { LayerId::BCu } else { LayerId::FCu };
    let net = pad.child("net").and_then(|n| n.tagged("net")).and_then(|args| {
        let id = args.first()?.as_i64()? as u32;
        (id != 0).then(|| (id, args.get(1).and_then(SExpr::text).unwrap_or("").to_string()))
    });

    ImportedPad {
        number,
        offset: local_offset,
        shape: import_pad_shape(pad),
        // Undo both the sum *and* the negation `pad_to_sexpr` writes
        // (see that function's doc comment for the ground-truth
        // measurement behind the negation): `footprint_rotation_deg`
        // here is already `import_footprints`' own negated (back to
        // Alladin's internal convention) value, so recovering the
        // pad's own local rotation is `-stored_angle -
        // footprint_rotation_deg`, not the pre-negation-fix
        // `stored_angle - footprint_rotation_deg`.
        rotation_deg: -stored_angle - footprint_rotation_deg,
        drill,
        layer,
        net,
    }
}

/// Reads a `np_thru_hole` `(pad ...)` form as an [`ImportedHole`] --
/// the read-side counterpart of [`pad_to_sexpr`]'s `PadMount::NpThruHole`
/// branch. `drill` falls back to the pad's own `size` if a real file
/// somehow omits `(drill ...)` (shouldn't happen for a genuine
/// `np_thru_hole` pad, but a missing hole size is a worse failure mode
/// than a slightly-off one).
fn import_hole(pad: &SExpr) -> ImportedHole {
    let (local_offset, _) = at_with_rotation(pad, "at");
    let size_diameter = match import_pad_shape(pad) {
        WritePadShape::Circle { diameter } => diameter,
        WritePadShape::Rect { width, height } | WritePadShape::Oval { width, height } => width.min(height),
    };
    ImportedHole { offset: local_offset, drill: pad_drill(pad).unwrap_or(size_diameter) }
}

/// Reads every `(footprint ...)` block of a `.kicad_pcb` file (already
/// loaded into a string) with full pad-shape fidelity. See
/// [`ImportedFootprint`]'s doc comment for why this exists alongside
/// [`crate::import_kicad_pcb`] rather than replacing it -- the two serve
/// different consumers (editable parts vs. routing obstacles) and callers
/// needing both (as `alladin-pcb`'s round-trip import does, for board
/// outline/tracks/vias/nets) should call both against the same source.
pub fn import_footprints(source: &str) -> Result<Vec<ImportedFootprint>, ImportError> {
    let root = alladin_sexpr::parse(source)?;
    if root.tagged("kicad_pcb").is_none() {
        return Err(ImportError::NotAKicadPcb);
    }

    Ok(root
        .children("footprint")
        .map(|footprint| {
            let (position, stored_rotation_deg) = at_with_rotation(footprint, "at");
            // Negated back to Alladin's own internal convention -- see
            // `pad_to_sexpr`'s doc comment for the ground-truth
            // measurement this undoes.
            let rotation_deg = -stored_rotation_deg;
            let mut pads = Vec::new();
            let mut holes = Vec::new();
            for pad in footprint.children("pad") {
                if pad_mount_text(pad) == "np_thru_hole" {
                    holes.push(import_hole(pad));
                } else {
                    pads.push(import_pad(pad, rotation_deg));
                }
            }
            ImportedFootprint {
                reference: property_text(footprint, "Reference").unwrap_or("").to_string(),
                value: property_text(footprint, "Value").unwrap_or("").to_string(),
                position,
                rotation_deg,
                pads,
                holes,
            }
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_geom::MM;

    fn mm(v: f64) -> Unit {
        (v * MM as f64).round() as Unit
    }

    #[test]
    fn output_is_a_syntactically_valid_single_kicad_pcb_form() {
        let text = write_kicad_pcb(&[], &[], &Node::new(), &[], &[], &[], &[]);
        let parsed = alladin_sexpr::parse(&text).expect("writer output must be valid S-expression syntax");
        assert!(parsed.tagged("kicad_pcb").is_some(), "root form must be `(kicad_pcb ...)`");
    }

    #[test]
    fn declares_every_net_it_was_given_plus_the_implicit_net_0() {
        let text = write_kicad_pcb(&[], &[], &Node::new(), &[(1, "GND".to_string()), (2, "VCC".to_string())], &[], &[], &[]);
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let net_ids: Vec<i64> =
            parsed.children("net").filter_map(|n| n.tagged("net").and_then(|a| a.first()).and_then(SExpr::as_i64)).collect();
        assert_eq!(net_ids, vec![0, 1, 2]);
    }

    fn one_pad_footprint(offset: Point, pad_rotation_deg: f64, footprint_rotation_deg: f64) -> WriteFootprint {
        WriteFootprint {
            reference: "U1".to_string(),
            value: "Test".to_string(),
            position: Point::new(mm(10.0), mm(5.0)),
            rotation_deg: footprint_rotation_deg,
            pads: vec![WritePad {
                number: "1".to_string(),
                offset,
                shape: WritePadShape::Rect { width: mm(2.0), height: mm(1.0) },
                rotation_deg: pad_rotation_deg,
                mount: PadMount::Smd,
                drill: None,
                layer: LayerId::FCu,
                net: None,
            }],
        }
    }

    /// Regression test for a real, ground-truth-caught bug (see
    /// `pad_to_sexpr`'s doc comment and the development log's
    /// corresponding entry): on a rotated footprint, a pad's `at`
    /// position must be written completely untouched (KiCad applies the
    /// footprint's own rotation on load) -- pre-rotating it here, as an
    /// earlier version of this writer did, double-rotates every pad on
    /// any rotated footprint the moment real KiCad opens the file.
    #[test]
    fn a_pad_s_written_position_is_untouched_by_the_footprint_s_own_rotation() {
        let offset = Point::new(mm(2.0), mm(0.0));
        let fp = one_pad_footprint(offset, 0.0, 45.0);
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let pad = parsed.children("footprint").next().unwrap().children("pad").next().unwrap();
        let at = pad.child("at").unwrap().tagged("at").unwrap();
        assert_eq!(at[0].as_f64(), Some(2.0), "pad x must stay the raw, unrotated local offset");
        assert_eq!(at[1].as_f64(), Some(0.0), "pad y must stay the raw, unrotated local offset");
    }

    /// Regression test for the rotation-*direction* bug this session
    /// found (distinct from, and in addition to, the angle-*sum* bug
    /// the two tests above already cover -- see `pad_to_sexpr`'s doc
    /// comment for the full story): a plain 90-degree-rotated
    /// footprint's pad at local offset (2mm, 0) must be written with
    /// angle `-90`, not `+90`. Ground-truth verified with a standalone
    /// probe (this exact footprint/offset/rotation, plus two marker
    /// pads at the two candidate world positions) run through real
    /// `kicad-cli pcb drc`: the pad lands at world (0, -2mm), which
    /// only `-90` (not `+90`) written to disk reproduces once real
    /// KiCad applies *its own* rotation convention on load.
    #[test]
    fn a_rotated_footprint_s_pad_angle_is_written_negated_to_match_real_kicad() {
        let fp = one_pad_footprint(Point::new(mm(2.0), 0), 0.0, 90.0);
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let footprint = parsed.children("footprint").next().unwrap();
        let fp_at = footprint.child("at").unwrap().tagged("at").unwrap();
        assert_eq!(fp_at[2].as_f64(), Some(-90.0), "the footprint's own `at` angle must be negated too");
        let pad = footprint.children("pad").next().unwrap();
        let pad_at = pad.child("at").unwrap().tagged("at").unwrap();
        assert_eq!(pad_at[2].as_f64(), Some(-90.0));
    }

    /// Regression test, other half of the same bug: the written angle
    /// must be the *sum* of the pad's own local rotation and the
    /// footprint's placement rotation (15 degrees local + 30 degrees
    /// footprint = 45 degrees), then **negated** -- see `pad_to_sexpr`'s
    /// doc comment for the separate, later-caught ground-truth
    /// measurement behind that negation (real `pcbnew` rotates a
    /// positive on-disk angle the opposite way Alladin's own internal
    /// convention does).
    #[test]
    fn a_pad_s_written_angle_is_the_sum_of_its_own_and_its_footprint_s_rotation() {
        let fp = one_pad_footprint(Point::new(mm(2.0), 0), 15.0, 30.0);
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let pad = parsed.children("footprint").next().unwrap().children("pad").next().unwrap();
        let at = pad.child("at").unwrap().tagged("at").unwrap();
        assert_eq!(at[2].as_f64(), Some(-45.0));
    }

    #[test]
    fn an_unrotated_pad_omits_the_angle_entirely_rather_than_writing_a_spurious_zero() {
        let fp = one_pad_footprint(Point::new(0, 0), 0.0, 0.0);
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let pad = parsed.children("footprint").next().unwrap().children("pad").next().unwrap();
        let at = pad.child("at").unwrap().tagged("at").unwrap();
        assert_eq!(at.len(), 2, "a truly unrotated pad's `at` must be just x and y, matching real KiCad's own output");
    }

    #[test]
    fn through_hole_pads_get_a_drill_and_multilayer_star_cu_while_smd_pads_do_not() {
        let mut tht = one_pad_footprint(Point::new(0, 0), 0.0, 0.0);
        tht.pads[0].mount = PadMount::ThruHole;
        tht.pads[0].drill = Some(mm(0.8));
        let text = write_kicad_pcb(&[], &[tht], &Node::new(), &[], &[], &[], &[]);
        assert!(text.contains("thru_hole"));
        assert!(text.contains("(drill 0.8)"));
        assert!(text.contains("*.Cu"));

        let smd = one_pad_footprint(Point::new(0, 0), 0.0, 0.0);
        let text = write_kicad_pcb(&[], &[smd], &Node::new(), &[], &[], &[], &[]);
        assert!(text.contains("smd"));
        // Not a plain `!text.contains("drill")`: the fixed `setup`
        // boilerplate always has a `drillshape` plotting option, which
        // is a real substring match but nothing to do with pads.
        assert!(!text.contains("(drill "), "an SMD pad must have no `(drill ...)` form at all");
    }

    fn np_thru_hole_footprint(offset: Point, drill: Unit) -> WriteFootprint {
        WriteFootprint {
            reference: "H1".to_string(),
            value: "Mounting hole (M3, NPTH)".to_string(),
            position: Point::new(mm(10.0), mm(5.0)),
            rotation_deg: 0.0,
            pads: vec![WritePad {
                number: String::new(),
                offset,
                shape: WritePadShape::Circle { diameter: drill },
                rotation_deg: 0.0,
                mount: PadMount::NpThruHole,
                drill: Some(drill),
                layer: LayerId::FCu,
                net: None,
            }],
        }
    }

    #[test]
    fn a_mounting_hole_writes_as_an_np_thru_hole_pad_with_no_net_and_size_equal_to_drill() {
        let fp = np_thru_hole_footprint(Point::new(0, 0), mm(3.2));
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        assert!(text.contains("np_thru_hole"), "must be written as a real np_thru_hole pad, not thru_hole or smd");
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let pad = parsed.children("footprint").next().unwrap().children("pad").next().unwrap();
        assert!(pad.child("net").is_none(), "a mechanical hole's own pad form must never carry a net form");
        let size = pad.child("size").unwrap().tagged("size").unwrap();
        assert_eq!(size[0].as_f64(), Some(3.2), "an np_thru_hole pad's size must equal its own drill, no annular ring");
        assert_eq!(pad.tagged("pad").unwrap()[0], SExpr::Str(String::new()), "a mounting hole's pad number must be blank");
    }

    #[test]
    fn a_mounting_hole_round_trips_through_import_footprints_as_a_hole_not_a_pad() {
        let fp = np_thru_hole_footprint(Point::new(mm(1.0), mm(2.0)), mm(2.2));
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        let imported = import_footprints(&text).unwrap();
        assert_eq!(imported[0].pads.len(), 0, "an np_thru_hole pad must not also show up as an ImportedPad");
        assert_eq!(imported[0].holes.len(), 1);
        assert_eq!(imported[0].holes[0].offset, Point::new(mm(1.0), mm(2.0)));
        assert_eq!(imported[0].holes[0].drill, mm(2.2));
    }

    #[test]
    fn reference_and_value_clear_every_pad_regardless_of_footprint_size() {
        // A deliberately huge pad (a 10mm x 1mm "module edge") -- the
        // Reference/Value silkscreen must still land outside its reach,
        // not at a fixed small offset that only works for small parts
        // (see `footprint_vertical_extent`'s doc comment: no per-part
        // special casing).
        let fp = one_pad_footprint(Point::new(0, 0), 0.0, 0.0);
        let mut fp = fp;
        fp.pads[0].shape = WritePadShape::Rect { width: mm(10.0), height: mm(1.0) };
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let footprint = parsed.children("footprint").next().unwrap();
        let props: Vec<&SExpr> = footprint.children("property").collect();
        let reference = props.iter().find(|p| p.tagged("property").unwrap()[0] == SExpr::Str("Reference".to_string())).unwrap();
        let ref_y = reference.child("at").unwrap().tagged("at").unwrap()[1].as_f64().unwrap();
        let pad_reach_mm = (10.0_f64.powi(2) + 1.0_f64.powi(2)).sqrt() / 2.0;
        assert!(ref_y < -pad_reach_mm, "Reference at y={ref_y} must clear the pad's own reach ({pad_reach_mm}mm)");
    }

    /// Regression test for a real bug: `footprint_property` used to
    /// write a bare hardcoded `"0.15"` stroke thickness for every
    /// Reference/Value label -- exactly JLCPCB's own published absolute
    /// floor with zero safety margin, and thinner than this codebase's
    /// own [`JlcpcbDfm::MIN_SILK_LINE_WIDTH`] DFM floor besides. Every
    /// footprint's visible silkscreen label was silently below this
    /// crate's own stated minimum until this constant was actually
    /// wired in here.
    #[test]
    fn reference_and_value_labels_use_the_buffered_jlcpcb_silk_minimum_not_the_bare_dfm_floor() {
        let fp = one_pad_footprint(Point::new(0, 0), 0.0, 0.0);
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let footprint = parsed.children("footprint").next().unwrap();
        let props: Vec<&SExpr> = footprint.children("property").collect();
        let reference = props.iter().find(|p| p.tagged("property").unwrap()[0] == SExpr::Str("Reference".to_string())).unwrap();
        let thickness = reference.child("effects").unwrap().child("font").unwrap().child("thickness").unwrap().tagged("thickness").unwrap()[0].as_f64().unwrap();
        assert_eq!(thickness, JlcpcbDfm::MIN_SILK_LINE_WIDTH as f64 / MM as f64);
        assert!(thickness > 0.15, "must sit strictly above JLCPCB's own bare 0.15mm absolute floor, got {thickness}mm");
    }

    /// Reference/Value labels are editor-side orientation aids only --
    /// they must never actually print (see `footprint_to_sexpr`'s own
    /// comment at their construction): both properties are still
    /// *written* (KiCad requires them) but marked `(hide yes)`, so the
    /// fabricated silkscreen carries only deliberately placed
    /// annotations.
    #[test]
    fn reference_and_value_properties_are_written_but_hidden_from_the_fabricated_silk() {
        let fp = one_pad_footprint(Point::new(0, 0), 0.0, 0.0);
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let footprint = parsed.children("footprint").next().unwrap();
        for name in ["Reference", "Value"] {
            let prop = footprint
                .children("property")
                .find(|p| p.tagged("property").unwrap()[0] == SExpr::Str(name.to_string()))
                .unwrap_or_else(|| panic!("{name} property must still be written"));
            let hide = prop.child("hide").unwrap_or_else(|| panic!("{name} must carry a hide form")).tagged("hide").unwrap();
            assert_eq!(hide[0], SExpr::Sym("yes".to_string()), "{name} must be hidden");
        }
    }

    #[test]
    fn board_outline_becomes_a_closed_chain_of_gr_lines_on_edge_cuts() {
        let outline = vec![Polygon::rounded_rect(mm(20.0), mm(10.0), 0, 4)];
        let text = write_kicad_pcb(&outline, &[], &Node::new(), &[], &[], &[], &[]);
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let lines: Vec<&SExpr> = parsed.children("gr_line").collect();
        assert_eq!(lines.len(), outline[0].points.len(), "one gr_line per outline edge, closing back to the first point");
        for line in &lines {
            let layer = line.child("layer").unwrap().tagged("layer").unwrap();
            assert_eq!(layer[0], SExpr::Str("Edge.Cuts".to_string()));
        }
    }

    #[test]
    fn tracks_and_vias_from_the_node_are_written_but_pads_and_zones_in_it_are_not() {
        use alladin_core::NetClass;
        use alladin_geom::{Circle, Segment};

        let mut node = Node::new();
        node.add(Item::Track { shape: Segment::new(Point::new(0, 0), Point::new(mm(5.0), 0), mm(0.25)), net: None, layer: LayerId::FCu, class: NetClass::C });
        node.add(Item::Via { shape: Circle::new(Point::new(mm(1.0), 0), mm(0.5)), drill: mm(0.3), net: None });
        node.add(Item::Pad { shape: alladin_core::PadShape::Circle(Circle::new(Point::new(0, 0), mm(0.5))), net: None, layer: LayerId::FCu });
        // A bare `Item::Zone` sitting in `node` itself (not passed via
        // the separate `zones` parameter, see `WriteZone`'s doc comment
        // for why the two are split) must still be ignored here.
        node.add(Item::Zone { outline: Polygon::new(vec![Point::new(0, 0), Point::new(mm(1.0), 0), Point::new(0, mm(1.0))]), layer: LayerId::FCu, net: None });

        let text = write_kicad_pcb(&[], &[], &node, &[], &[], &[], &[]);
        assert!(text.contains("(segment"));
        assert!(text.contains("(via"));
        assert!(!text.contains("(pad "), "bare top-level pads (outside a footprint) are invalid KiCad syntax");
        assert!(!text.contains("(zone"), "a zone sitting in `node` itself, not passed via `zones`, must not be written");
    }

    #[test]
    fn a_zone_from_the_zones_parameter_writes_its_outline_and_every_filled_island() {
        let outline = Polygon::new(vec![Point::new(0, 0), Point::new(mm(10.0), 0), Point::new(mm(10.0), mm(10.0)), Point::new(0, mm(10.0))]);
        let island_a = Polygon::new(vec![Point::new(0, 0), Point::new(mm(4.0), 0), Point::new(mm(4.0), mm(4.0)), Point::new(0, mm(4.0))]);
        let island_b = Polygon::new(vec![Point::new(mm(6.0), 0), Point::new(mm(10.0), 0), Point::new(mm(10.0), mm(4.0)), Point::new(mm(6.0), mm(4.0))]);
        let zone = WriteZone { outline: outline.clone(), layer: LayerId::FCu, net: Some((1, "GND".to_string())), islands: vec![island_a.clone(), island_b.clone()] };

        let text = write_kicad_pcb(&[], &[], &Node::new(), &[(1, "GND".to_string())], std::slice::from_ref(&zone), &[], &[]);
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let zones: Vec<&SExpr> = parsed.children("zone").collect();
        assert_eq!(zones.len(), 1);
        let z = zones[0];
        assert_eq!(z.child("net").unwrap().tagged("net").unwrap()[0].as_i64(), Some(1));
        assert_eq!(z.child("net_name").unwrap().tagged("net_name").unwrap()[0], SExpr::Str("GND".to_string()));
        assert_eq!(z.child("layer").unwrap().tagged("layer").unwrap()[0], SExpr::Str("F.Cu".to_string()));
        assert!(z.child("polygon").is_some(), "the raw drawn outline must always be written, even though it's never re-read on import");
        assert_eq!(z.children("filled_polygon").count(), 2, "one filled_polygon per island");
    }

    #[test]
    fn a_net_less_zone_writes_net_zero_with_no_name() {
        let outline = Polygon::new(vec![Point::new(0, 0), Point::new(mm(1.0), 0), Point::new(0, mm(1.0))]);
        let zone = WriteZone { outline, layer: LayerId::BCu, net: None, islands: vec![] };
        let text = write_kicad_pcb(&[], &[], &Node::new(), &[], std::slice::from_ref(&zone), &[], &[]);
        let parsed = alladin_sexpr::parse(&text).unwrap();
        let z = parsed.children("zone").next().unwrap();
        assert_eq!(z.child("net").unwrap().tagged("net").unwrap()[0].as_i64(), Some(0));
        assert_eq!(z.children("filled_polygon").count(), 0, "an unfilled zone still writes, just with no filled_polygon blocks");
    }

    #[test]
    fn import_footprints_round_trips_reference_value_position_and_rotation() {
        let fp = one_pad_footprint(Point::new(mm(2.0), 0), 0.0, 30.0);
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        let imported = import_footprints(&text).unwrap();
        assert_eq!(imported.len(), 1);
        assert_eq!(imported[0].reference, "U1");
        assert_eq!(imported[0].value, "Test");
        assert_eq!(imported[0].position, Point::new(mm(10.0), mm(5.0)));
        assert_eq!(imported[0].rotation_deg, 30.0);
    }

    /// Round-trip regression test for the exact rotation-arithmetic bug
    /// `a_pad_s_written_angle_is_the_sum_of_its_own_and_its_footprint_s_rotation`
    /// guards on the write side: a pad's own *local* rotation (here 15
    /// degrees, on a footprint placed at 30 degrees, so 45 degrees is
    /// what's actually stored on disk) must come back out unchanged,
    /// not as the raw stored sum.
    #[test]
    fn import_footprints_recovers_the_pad_s_own_local_rotation_not_the_stored_sum() {
        let fp = one_pad_footprint(Point::new(mm(2.0), 0), 15.0, 30.0);
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        let imported = import_footprints(&text).unwrap();
        assert_eq!(imported[0].pads[0].rotation_deg, 15.0, "must recover the pad's own local rotation, not the stored 45-degree sum");
        assert_eq!(imported[0].pads[0].offset, Point::new(mm(2.0), 0), "pad offset is written raw and must import back raw, untouched by rotation");
    }

    #[test]
    fn import_footprints_recovers_pad_number_shape_and_net() {
        let mut fp = one_pad_footprint(Point::new(0, 0), 0.0, 0.0);
        fp.pads[0].number = "42".to_string();
        fp.pads[0].shape = WritePadShape::Oval { width: mm(2.0), height: mm(1.0) };
        fp.pads[0].net = Some((3, "GND".to_string()));
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[(3, "GND".to_string())], &[], &[], &[]);
        let imported = import_footprints(&text).unwrap();
        let pad = &imported[0].pads[0];
        assert_eq!(pad.number, "42");
        assert_eq!(pad.shape, WritePadShape::Oval { width: mm(2.0), height: mm(1.0) });
        assert_eq!(pad.net, Some((3, "GND".to_string())));
    }

    #[test]
    fn import_footprints_recovers_through_hole_drill_diameter() {
        let mut fp = one_pad_footprint(Point::new(0, 0), 0.0, 0.0);
        fp.pads[0].mount = PadMount::ThruHole;
        fp.pads[0].drill = Some(mm(0.8));
        let text = write_kicad_pcb(&[], &[fp], &Node::new(), &[], &[], &[], &[]);
        let imported = import_footprints(&text).unwrap();
        assert_eq!(imported[0].pads[0].drill, Some(mm(0.8)));

        let smd = one_pad_footprint(Point::new(0, 0), 0.0, 0.0);
        let text = write_kicad_pcb(&[], &[smd], &Node::new(), &[], &[], &[], &[]);
        let imported = import_footprints(&text).unwrap();
        assert_eq!(imported[0].pads[0].drill, None, "an SMD pad must import with no drill, not a guessed one");
    }

    #[test]
    fn import_footprints_rejects_a_source_that_is_not_a_kicad_pcb_form() {
        assert!(matches!(import_footprints("(not_a_pcb)"), Err(ImportError::NotAKicadPcb)));
    }
}
