//! Built-in footprint templates for manual part placement, plus the
//! shared `PadTemplate`/`FootprintTemplate` shapes used by
//! `crate::parts_db` (the user's own saved parts) and `crate::lcsc`
//! (real LCSC/EasyEDA downloads).
//!
//! **Collision/routing geometry uses the pad's *true* shape** (see
//! `alladin_core::PadShape`), not a circular approximation --
//! [`world_items`] builds a real `PadShape::Polygon` (rotated,
//! world-positioned) for `PadShapeKind::Rect`/`Oval`, and only a plain
//! `PadShape::Circle` for `PadShapeKind::Circle`. [`PadTemplate::radius`]
//! is a **derived, secondary** value (still `min(width, height) / 2`,
//! used for the parts-database column, hole-size estimation, and
//! manufacturing export -- see `crate::lcsc`'s module doc comment), not
//! the collision shape itself.
//!
//! Every `PadTemplate` carries its true `shape` (circle/rect/oval) and
//! `rotation_deg`, used both for on-screen rendering (`crate::app`'s
//! pad drawing) and for [`world_items`]'s collision polygon.

use alladin_core::{thermal, DfmViolation, Item, JlcpcbDfm, LayerId, PadShape, ZoneConnection};
use alladin_geom::{Circle, Point, Polygon, Unit, MM};

/// A genuine unplated mechanical hole (see `alladin_core::Item::Hole`'s
/// own doc comment for why this is a real, separate primitive rather
/// than a copper pad with no net) belonging to a [`FootprintTemplate`]
/// -- a board mounting hole for a screw, an alignment pin, etc.
/// Deliberately as small as [`PadTemplate::circle`]'s own minimal shape:
/// `offset` (footprint-local, before rotation -- same contract as
/// [`PadTemplate::offset`]) and `drill` (the hole's own diameter) are
/// the only two facts a purely mechanical feature needs; there is no
/// `layer`/`net`/`number` to carry (see [`Item::Hole`]'s doc comment for
/// why).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HoleTemplate {
    pub offset: Point,
    pub drill: Unit,
}

/// A pad's true shape -- used for both rendering (`crate::app`'s pad
/// drawing) and, since the "Echte Pad-Geometrie" slice, for
/// [`world_items`]'s actual `alladin_core::PadShape::Polygon` collision
/// geometry (`Rect`/`Oval`) rather than just a circular approximation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PadShapeKind {
    Circle,
    Rect { width: Unit, height: Unit },
    Oval { width: Unit, height: Unit },
}

#[derive(Debug, Clone)]
pub struct PadTemplate {
    /// Position relative to the footprint's own origin, before rotation.
    pub offset: Point,
    /// A derived, secondary value -- `min(width, height) / 2` for a
    /// non-circular pad -- kept for the parts-database column,
    /// hole-size estimation, and manufacturing export (see
    /// `crate::lcsc`'s module doc comment), **not** the collision shape
    /// itself anymore
    /// (see this module's doc comment): [`world_items`] builds
    /// `alladin_core::PadShape` from [`Self::shape`]/[`Self::rotation_deg`]
    /// instead.
    pub radius: Unit,
    pub layer: LayerId,
    /// The pad's number/name as the real part designates it (`"1"`,
    /// `"2"`, `"A3"`, ... for a downloaded part; just `"1"`, `"2"`, ...
    /// for built-ins/hand-added parts) -- shown on the board so a
    /// downloaded part's pinout is actually legible, not just "some
    /// pads".
    pub number: String,
    /// True visual shape, for rendering (see this module's doc comment).
    pub shape: PadShapeKind,
    /// The pad's own rotation in degrees, *relative to the footprint*
    /// (applied before the footprint's own placement rotation) --
    /// distinct from `crate::board_doc::PlacedFootprint::rotation_deg`,
    /// which rotates the whole part. Only meaningful for `Rect`/`Oval`
    /// (a rotated circle looks the same as an unrotated one).
    pub rotation_deg: f64,
    /// `Some(drill_diameter)` for a through-hole pad, `None` for SMD --
    /// **manufacturing** information [`Self::radius`] alone cannot carry
    /// (that field is a derived geometric value, always present for
    /// every pad kind regardless of its true collision shape; whether a
    /// pad also has a physical drilled hole is an orthogonal fact).
    /// Only ever populated by real downloads (`crate::lcsc`, from
    /// EasyEDA's own hole diameter) or by deliberate built-ins that are
    /// truly PTH (today: [`wire_pad_template`]). [`Self::circle`] still
    /// defaults to `None` (SMD). Manufacturing export uses this to write
    /// a plated drill plus copper on both layers instead of silently
    /// downgrading every through-hole part to SMD.
    pub hole_diameter: Option<Unit>,
    /// The pin's *function*, as the schematic symbol names it (`"GND"`,
    /// `"VDD"`, `"DIN"`, `"DOUT"`, ...) -- distinct from [`Self::number`]
    /// (the pad's position/designator, e.g. `"3"`) and from the *board*
    /// net it ends up wired to after `Connect`. Only ever populated by
    /// real LCSC/EasyEDA downloads (`crate::lcsc`, from the schematic
    /// symbol SVG, which is the only place this fact lives -- the
    /// footprint/PCB data alone never carries it, see that module's doc
    /// comment); `None` for built-in/hand-added templates without a
    /// schematic symbol source for pin names.
    pub pin_name: Option<String>,
    /// How this pad joins a same-net copper pour — see
    /// [`ZoneConnection`]. Set by the parts DB / LCSC heuristic
    /// ([`apply_zone_connection_heuristic`]); built-ins default to
    /// Thermal.
    pub zone_connection: ZoneConnection,
}

impl PadTemplate {
    /// A plain circular SMD pad with a sequential number -- what every
    /// built-in/hand-added template pad actually is.
    fn circle(offset: Point, radius: Unit, layer: LayerId, number: impl Into<String>) -> Self {
        Self {
            offset,
            radius,
            layer,
            number: number.into(),
            shape: PadShapeKind::Circle,
            rotation_deg: 0.0,
            hole_diameter: None,
            pin_name: None,
            zone_connection: ZoneConnection::Thermal,
        }
    }
}

/// Longest axis of a pad's copper (circle diameter or rect/oval max side).
fn pad_longest_side(pad: &PadTemplate) -> Unit {
    match pad.shape {
        PadShapeKind::Circle => pad.radius * 2,
        PadShapeKind::Rect { width, height } | PadShapeKind::Oval { width, height } => {
            width.max(height)
        }
    }
}

fn pad_area_nm2(pad: &PadTemplate) -> i64 {
    match pad.shape {
        PadShapeKind::Circle => {
            let d = pad.radius * 2;
            d * d
        }
        PadShapeKind::Rect { width, height } | PadShapeKind::Oval { width, height } => {
            width * height
        }
    }
}

fn looks_like_exposed_pad_name(pad: &PadTemplate) -> bool {
    let name = pad.pin_name.as_deref().unwrap_or(pad.number.as_str());
    let n = name.trim().to_ascii_uppercase();
    if n.contains("EXPOSED") {
        return true;
    }
    matches!(
        n.as_str(),
        "EP" | "E.P." | "THERMAL" | "THERMAL PAD" | "THERMAL_PAD" | "PAD" | "GNDPAD" | "GND_PAD"
    ) || (n.starts_with("EP") && n.chars().nth(2).is_none_or(|c| !c.is_ascii_alphanumeric()))
}

/// Assign [`PadTemplate::zone_connection`] from pad geometry / EP grids.
/// Call on LCSC import and whenever a part is loaded from the parts DB
/// (so legacy rows downloaded before this rule still get a correct EP).
///
/// Rules (first match wins per pad):
/// 1. Several pads share the same `number` (exposed-pad paste grid) → Solid
/// 2. Pad / pin name looks like an exposed/thermal pad → Solid
/// 3. Longest side ≥ [`thermal::SOLID_MIN_SIDE`] → Solid
/// 4. Single dominant center pad (area ≥ 4× the median of the others,
///    near the pad centroid) → Solid — catches QFN EPs numbered as a
///    plain pin (e.g. RP2040 pad `57`) even when slightly under the
///    size floor after EasyEDA rounding
/// 5. Else Thermal
pub fn apply_zone_connection_heuristic(pads: &mut [PadTemplate]) {
    let mut number_counts: std::collections::HashMap<String, usize> =
        std::collections::HashMap::new();
    for pad in pads.iter() {
        *number_counts.entry(pad.number.clone()).or_default() += 1;
    }

    let areas: Vec<i64> = pads.iter().map(pad_area_nm2).collect();
    let mut dominant_center = vec![false; pads.len()];
    if pads.len() >= 3 {
        let (sum_x, sum_y) = pads.iter().fold((0_i64, 0_i64), |(sx, sy), p| {
            (sx + p.offset.x, sy + p.offset.y)
        });
        let n = pads.len() as i64;
        let cx = sum_x / n;
        let cy = sum_y / n;
        let span = pads.iter().fold(0_i64, |acc, p| {
            let dx = (p.offset.x - cx).abs();
            let dy = (p.offset.y - cy).abs();
            acc.max(dx).max(dy)
        });
        let near = (span / 5).max(MM / 10); // ≤20% of half-span, min 0.1mm
        for i in 0..pads.len() {
            let mut others: Vec<i64> = areas
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, a)| *a)
                .collect();
            others.sort_unstable();
            let median = others[others.len() / 2];
            if median <= 0 {
                continue;
            }
            let near_center =
                (pads[i].offset.x - cx).abs() <= near && (pads[i].offset.y - cy).abs() <= near;
            if near_center && areas[i] >= median.saturating_mul(4) {
                dominant_center[i] = true;
            }
        }
    }

    for (i, pad) in pads.iter_mut().enumerate() {
        let ep_grid = number_counts.get(&pad.number).copied().unwrap_or(0) > 1;
        pad.zone_connection = if ep_grid
            || looks_like_exposed_pad_name(pad)
            || pad_longest_side(pad) >= thermal::SOLID_MIN_SIDE
            || dominant_center[i]
        {
            ZoneConnection::Solid
        } else {
            ZoneConnection::Thermal
        };
    }
}

/// A footprint's own mechanical "keep-out" rectangle -- the physical
/// extent of the part's real body/plastic case, not its copper.
/// Local, unrotated, footprint-relative coordinates (same convention
/// as [`PadTemplate::offset`]) -- rotated/translated into world space
/// by [`world_courtyard`] exactly like a pad. Deliberately always a
/// plain axis-aligned rectangle in local space, not a true silhouette
/// -- the same "simpler, still correct enough" trade-off `crate::lcsc`'s
/// own `POLYGON` pad handling already accepts (see that module's doc
/// comment): every real source this ever comes from (a downloaded
/// part's own silkscreen outline, or the pad/hole bounding-box
/// fallback below) is already close to rectangular in practice, and a
/// rectangle is trivial to store as four plain numbers in
/// `crate::parts_db`'s SQLite schema.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Courtyard {
    pub center: Point,
    pub width: Unit,
    pub height: Unit,
}

#[derive(Debug, Clone)]
pub struct FootprintTemplate {
    /// Owned, not `&'static str`: templates no longer only come from the
    /// fixed [`builtin_templates`] list -- `crate::parts_db` loads more of
    /// them from the user's own parts database at runtime, where names
    /// obviously can't be `'static` string literals.
    pub name: String,
    /// Prefix for auto-generated reference designators (e.g. `"P"` ->
    /// `P1`, `P2`, ...) -- deliberately generic rather than a real
    /// component-class letter (`R`/`C`/`U`/...) since there's no part-type
    /// knowledge behind these templates yet.
    pub reference_prefix: String,
    pub pads: Vec<PadTemplate>,
    /// Genuine unplated mechanical holes this template also has, if
    /// any (see [`HoleTemplate`]'s own doc comment) -- empty for every
    /// ordinary electrical part (LCSC downloads, hand-added parts).
    /// Populated by mechanical builtins ([`mounting_hole_template`])
    /// and any imported NPTH geometry.
    pub holes: Vec<HoleTemplate>,
    /// Whether this template's own placed instances should be silently
    /// skipped by `crate::bom::build_bom_rows` -- purely mechanical
    /// parts (mounting holes, solder wire pads) are never a purchasable
    /// line item, matching how the *original* KiCad board this pipeline
    /// is meant to reproduce already marks them "Exclude from BOM" (see
    /// `crate::bom`'s module doc comment). `false` for every existing
    /// template (electrical parts are still included, unchanged
    /// behaviour) -- only [`wire_pad_template`]/[`mounting_hole_template`]
    /// and hand-registered mechanical parts set this.
    pub exclude_from_bom: bool,
    /// The part's own real silkscreen/assembly-layer body outline, if
    /// [`crate::lcsc`] found one in the downloaded footprint's raw
    /// data -- `None` for every built-in/hand-added
    /// template (none of those carry real body dimensions at all),
    /// and also `None` for plenty of real SMD parts whose footprint
    /// draws no dedicated body outline at all (a 0603 resistor, say).
    /// Use [`Self::courtyard`], never this field directly, to always
    /// get a real rectangle either way -- see that method's own
    /// fallback.
    pub explicit_courtyard: Option<Courtyard>,
}

impl FootprintTemplate {
    /// This template's own mechanical keep-out rectangle for
    /// assembly/body gates: the axis-aligned union of
    /// [`Self::explicit_courtyard`] (EasyEDA top-silk bbox, when
    /// present) and [`fallback_courtyard`] (every pad + NPTH hole).
    /// Silk alone must never win when it is *smaller* than the copper
    /// (a common EasyEDA USB/connector case) -- that under-size let
    /// SMD parts sit "legally" on shell pads that JLCPCB assembly DFM
    /// then rejects. When silk is larger than the pads (a plastic body
    /// overhanging gull-wing leads), the union keeps the larger silk.
    /// Two placed footprints' bodies must never overlap within
    /// [`alladin_core::JlcpcbDfm::COMPONENT_BODY_CLEARANCE`] (see
    /// `crate::board_doc::check_placement`).
    pub fn courtyard(&self) -> Courtyard {
        let pads_holes = fallback_courtyard(&self.pads, &self.holes);
        match self.explicit_courtyard {
            Some(silk) => union_courtyards(silk, pads_holes),
            None => pads_holes,
        }
    }
}

/// Axis-aligned bounding-box union of two local courtyards -- used by
/// [`FootprintTemplate::courtyard`] so silk and pad/hole extents never
/// under-cover each other.
fn union_courtyards(a: Courtyard, b: Courtyard) -> Courtyard {
    if a.width <= 0 && a.height <= 0 {
        return b;
    }
    if b.width <= 0 && b.height <= 0 {
        return a;
    }
    let a_min = Point::new(a.center.x - a.width / 2, a.center.y - a.height / 2);
    let a_max = Point::new(a.center.x + a.width / 2, a.center.y + a.height / 2);
    let b_min = Point::new(b.center.x - b.width / 2, b.center.y - b.height / 2);
    let b_max = Point::new(b.center.x + b.width / 2, b.center.y + b.height / 2);
    let min = Point::new(a_min.x.min(b_min.x), a_min.y.min(b_min.y));
    let max = Point::new(a_max.x.max(b_max.x), a_max.y.max(b_max.y));
    Courtyard {
        center: Point::new((min.x + max.x) / 2, (min.y + max.y) / 2),
        width: max.x - min.x,
        height: max.y - min.y,
    }
}

/// Every plated or mechanical drill this template would place at
/// `position`/`rotation_deg` -- PTH pads (`hole_diameter`) plus NPTH
/// [`HoleTemplate`]s. Used by assembly lead-to-hole gates; not emitted
/// into `Node` as separate `Item::Hole`s for PTH (those stay pads).
pub fn world_assembly_drills(
    template: &FootprintTemplate,
    position: Point,
    rotation_deg: f64,
) -> Vec<Circle> {
    let mut drills = Vec::new();
    for pad in &template.pads {
        let Some(drill) = pad.hole_diameter else {
            continue;
        };
        if drill <= 0 {
            continue;
        }
        drills.push(Circle::new(
            pad_world_position(pad.offset, position, rotation_deg),
            drill / 2,
        ));
    }
    for hole in &template.holes {
        drills.push(Circle::new(
            pad_world_position(hole.offset, position, rotation_deg),
            hole.drill / 2,
        ));
    }
    drills
}

fn mm(v: f64) -> Unit {
    (v * MM as f64).round() as Unit
}

/// The four local, unrotated corners of one pad's own true extent
/// (its own `rotation_deg` applied, the footprint's placement
/// rotation deliberately not) -- the building block
/// [`fallback_courtyard`] reduces to a bounding box. A `Circle` pad
/// has no rotation to speak of, so its own bounding square's corners
/// are returned directly rather than routed through the same
/// rotate-4-corners math as `Rect`/`Oval`.
fn pad_local_corners(pad: &PadTemplate) -> [Point; 4] {
    let (half_w, half_h) = match pad.shape {
        PadShapeKind::Circle => (pad.radius, pad.radius),
        PadShapeKind::Rect { width, height } | PadShapeKind::Oval { width, height } => {
            (width / 2, height / 2)
        }
    };
    [
        Point::new(-half_w, -half_h),
        Point::new(half_w, -half_h),
        Point::new(half_w, half_h),
        Point::new(-half_w, half_h),
    ]
    .map(|corner| corner.rotated(pad.rotation_deg).add(pad.offset))
}

/// The bounding box, in the template's own local/unrotated
/// coordinates, of every pad's *and* hole's true (possibly rotated)
/// extent -- [`crate::lcsc`]'s fallback whenever no real silkscreen
/// courtyard/body outline was found in a downloaded part's own data
/// (see that module's doc comment), and every hand-added/built-in
/// template's *only* source, since none of those carry
/// real body dimensions at all. A template with neither a pad nor a
/// hole at all (shouldn't happen in practice) degenerates to a
/// zero-size rectangle at the origin, which can never collide with
/// anything.
pub fn fallback_courtyard(pads: &[PadTemplate], holes: &[HoleTemplate]) -> Courtyard {
    let mut min = Point::new(Unit::MAX, Unit::MAX);
    let mut max = Point::new(Unit::MIN, Unit::MIN);
    let mut any = false;
    for pad in pads {
        for corner in pad_local_corners(pad) {
            any = true;
            min.x = min.x.min(corner.x);
            min.y = min.y.min(corner.y);
            max.x = max.x.max(corner.x);
            max.y = max.y.max(corner.y);
        }
    }
    for hole in holes {
        for corner in [
            Point::new(
                hole.offset.x - hole.drill / 2,
                hole.offset.y - hole.drill / 2,
            ),
            Point::new(
                hole.offset.x + hole.drill / 2,
                hole.offset.y + hole.drill / 2,
            ),
        ] {
            any = true;
            min.x = min.x.min(corner.x);
            min.y = min.y.min(corner.y);
            max.x = max.x.max(corner.x);
            max.y = max.y.max(corner.y);
        }
    }
    if !any {
        return Courtyard {
            center: Point::new(0, 0),
            width: 0,
            height: 0,
        };
    }
    Courtyard {
        center: Point::new((min.x + max.x) / 2, (min.y + max.y) / 2),
        width: max.x - min.x,
        height: max.y - min.y,
    }
}

/// The world-space courtyard rectangle of `template` placed at
/// `position`/`rotation_deg` -- same rotate-then-translate convention
/// as every pad (see [`pad_world_position`]/[`world_items`]).
/// Deliberately kept out of [`world_items`]/`Item`/`Node` entirely: a
/// courtyard has no copper and must never itself become a
/// collidable/DRC item (a routed track legitimately runs underneath
/// one) -- `crate::board_doc`'s own body-vs-body overlap check is the
/// only consumer.
pub fn world_courtyard(
    template: &FootprintTemplate,
    position: Point,
    rotation_deg: f64,
) -> Polygon {
    let courtyard = template.courtyard();
    let center = pad_world_position(courtyard.center, position, rotation_deg);
    pad_outline_polygon(courtyard.width, courtyard.height, 0, rotation_deg, center)
}

/// The placeable built-in library: SMD/PTH solder pads plus metric
/// mounting holes. Real ICs and headers come from the parts database /
/// LCSC download. Old demo names (THT header / SOIC placeholders with
/// no drill) live in [`legacy_builtin_templates`] so existing boards
/// still open; they are hidden from Place part / `list_parts`.
pub fn builtin_templates() -> Vec<FootprintTemplate> {
    vec![
        smd_solder_pad_template(LayerId::FCu),
        smd_solder_pad_template(LayerId::BCu),
        wire_pad_template(
            "Wire pad (PTH, 1.0mm hole)",
            mm(1.0),
            mm(1.0),
            ZoneConnection::Thermal,
        ),
        wire_pad_template(
            "Wire pad (solder, 2mm)",
            mm(1.25),
            mm(1.5),
            ZoneConnection::Thermal,
        ),
        wire_pad_template(
            "Wire pad (PTH, 2.0mm hole)",
            mm(1.6),
            mm(2.0),
            ZoneConnection::Solid,
        ),
        mounting_hole_template("M2", mm(2.2)),
        mounting_hole_template("M2.5", mm(2.7)),
        mounting_hole_template("M3", mm(3.2)),
    ]
}

/// Names of the former demo builtins that must not appear in Place part
/// or `list_parts`, but still resolve when an old board is opened.
pub fn is_legacy_demo_template(name: &str) -> bool {
    matches!(
        name,
        "2-pin THT (2.54mm pitch)"
            | "4-pin THT header (2.54mm pitch)"
            | "SOIC-8 (1.27mm pitch)"
    )
}

/// Geometry for [`is_legacy_demo_template`] names — load/move only.
/// These were SMD circles labelled THT/SOIC (no plated hole).
pub fn legacy_builtin_templates() -> Vec<FootprintTemplate> {
    vec![
        FootprintTemplate {
            name: "2-pin THT (2.54mm pitch)".to_string(),
            reference_prefix: "P".to_string(),
            pads: vec![
                PadTemplate::circle(Point::new(-mm(1.27), 0), mm(0.45), LayerId::FCu, "1"),
                PadTemplate::circle(Point::new(mm(1.27), 0), mm(0.45), LayerId::FCu, "2"),
            ],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: None,
        },
        FootprintTemplate {
            name: "4-pin THT header (2.54mm pitch)".to_string(),
            reference_prefix: "J".to_string(),
            pads: (0..4)
                .map(|i| {
                    PadTemplate::circle(
                        Point::new(mm(2.54) * (i as Unit) - mm(2.54 * 1.5), 0),
                        mm(0.45),
                        LayerId::FCu,
                        (i + 1).to_string(),
                    )
                })
                .collect(),
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: None,
        },
        FootprintTemplate {
            name: "SOIC-8 (1.27mm pitch)".to_string(),
            reference_prefix: "U".to_string(),
            pads: (0..8)
                .map(|i| {
                    let row_side = if i < 4 { -1.0 } else { 1.0 };
                    let index_in_row = if i < 4 { i } else { 7 - i };
                    PadTemplate::circle(
                        Point::new(
                            row_side as Unit * mm(2.65),
                            mm(1.27) * (index_in_row as Unit) - mm(1.27 * 1.5),
                        ),
                        mm(0.3),
                        LayerId::FCu,
                        (i + 1).to_string(),
                    )
                })
                .collect(),
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: None,
        },
    ]
}

/// Placeable builtins plus load-only demo ghosts — what a session needs
/// so an old board can still move/rotate those footprints.
pub fn session_builtin_templates() -> Vec<FootprintTemplate> {
    let mut templates = builtin_templates();
    templates.extend(legacy_builtin_templates());
    templates
}

/// SMD solder / test pad (no drill) on one copper face — surface-solder
/// a thin wire or use as a probe point. 1.5 mm copper, Thermal so a
/// same-net pour stays solderable. `exclude_from_bom: true`.
fn smd_solder_pad_template(layer: LayerId) -> FootprintTemplate {
    let side = match layer {
        LayerId::FCu => "F.Cu",
        LayerId::BCu => "B.Cu",
    };
    let pad = PadTemplate::circle(Point::new(0, 0), mm(0.75), layer, "1");
    FootprintTemplate {
        name: format!("Solder pad (SMD, 1.5mm, {side})"),
        reference_prefix: "P".to_string(),
        pads: vec![pad],
        holes: Vec::new(),
        exclude_from_bom: true,
        explicit_courtyard: None,
    }
}

/// A single PTH solder pad for a wire (power, ground strap, Litze).
/// `copper_radius` / `drill` are already in internal units. The
/// historical `"Wire pad (solder, 2mm)"` name (2.5 mm copper / 1.5 mm
/// drill, Thermal) is kept for board compatibility. Large current pads
/// (`Wire pad (PTH, 2.0mm hole)`) default to Solid so the pour can
/// carry amps; S/M stay Thermal for hand soldering. `exclude_from_bom`.
fn wire_pad_template(
    name: &str,
    copper_radius: Unit,
    drill: Unit,
    zone_connection: ZoneConnection,
) -> FootprintTemplate {
    let mut pad = PadTemplate::circle(Point::new(0, 0), copper_radius, LayerId::FCu, "1");
    pad.hole_diameter = Some(drill);
    pad.zone_connection = zone_connection;
    FootprintTemplate {
        name: name.to_string(),
        reference_prefix: "W".to_string(),
        pads: vec![pad],
        holes: Vec::new(),
        exclude_from_bom: true,
        explicit_courtyard: None,
    }
}

/// A pure mechanical mounting hole -- no pads at all, just one
/// [`HoleTemplate`] at the template's own origin, named after a common
/// screw size (`M2`/`M2.5`/`M3`, drill sizes from the same convention
/// real KiCad's own `MountingHole` footprint library uses: metric screw
/// diameter plus roughly 0.2mm clearance). `exclude_from_bom: true` for
/// the same reason [`wire_pad_template`] is -- a mounting hole is never
/// a purchasable line item.
fn mounting_hole_template(screw_size: &str, drill: Unit) -> FootprintTemplate {
    FootprintTemplate {
        name: format!("Mounting hole ({screw_size}, NPTH)"),
        reference_prefix: "H".to_string(),
        pads: Vec::new(),
        holes: vec![HoleTemplate {
            offset: Point::new(0, 0),
            drill,
        }],
        exclude_from_bom: true,
        explicit_courtyard: None,
    }
}

/// A straight row of THT pads, `pin_count` pads spaced `pitch` apart,
/// centered on the footprint's own origin -- the one parametric shape
/// [`crate::parts_db`]'s "Add part..." form can generate today. Enough
/// to let a user register their own simple through-hole parts
/// (resistors, headers, ...) by hand; full LCSC/EasyEDA downloads live
/// in `crate::lcsc`.
#[cfg_attr(not(test), allow(dead_code))]
pub fn straight_row_template(
    name: String,
    reference_prefix: String,
    pin_count: u32,
    pitch_mm: f64,
    pad_radius_mm: f64,
) -> FootprintTemplate {
    straight_row_template_with_hole(
        name,
        reference_prefix,
        pin_count,
        pitch_mm,
        pad_radius_mm,
        None,
    )
}

/// [`straight_row_template`] plus an optional plated drill on every pad
/// (`None` = SMD). Used by the "Add part..." form when the user types a
/// hole diameter.
pub fn straight_row_template_with_hole(
    name: String,
    reference_prefix: String,
    pin_count: u32,
    pitch_mm: f64,
    pad_radius_mm: f64,
    hole_diameter_mm: Option<f64>,
) -> FootprintTemplate {
    let pitch = mm(pitch_mm);
    let radius = mm(pad_radius_mm);
    let hole = hole_diameter_mm.filter(|d| *d > 0.0).map(mm);
    let span = pitch * (pin_count.max(1) as Unit - 1);
    let pads = (0..pin_count.max(1))
        .map(|i| {
            let mut pad = PadTemplate::circle(
                Point::new(pitch * i as Unit - span / 2, 0),
                radius,
                LayerId::FCu,
                (i + 1).to_string(),
            );
            pad.hole_diameter = hole;
            pad
        })
        .collect();
    FootprintTemplate {
        name,
        reference_prefix,
        pads,
        holes: Vec::new(),
        exclude_from_bom: false,
        explicit_courtyard: None,
    }
}

/// Rotates `offset` by `rotation_deg` (counter-clockwise, board-space
/// convention) around the origin and translates it to `position` -- the
/// world-space placement of one pad. Rounds to the nearest internal unit,
/// same convention as [`alladin_geom::Point::lerp`] and
/// `Polygon::rounded_rect`'s own trig.
pub fn pad_world_position(offset: Point, position: Point, rotation_deg: f64) -> Point {
    let rad = rotation_deg.to_radians();
    let (sin, cos) = rad.sin_cos();
    let x = offset.x as f64 * cos - offset.y as f64 * sin;
    let y = offset.x as f64 * sin + offset.y as f64 * cos;
    Point::new(
        position.x + x.round() as Unit,
        position.y + y.round() as Unit,
    )
}

/// Number of straight segments used to polygonize each rounded corner
/// of a `PadShapeKind::Oval` pad's stadium shape -- matches
/// `crate::board_doc`'s own `Polygon::rounded_rect` call for the board
/// outline, both deliberately generous (12 rather than tighter
/// hot-path arc/circle segment budgets) since this only ever runs
/// once per placement/drag frame per footprint, not once per dense
/// clearance candidate-point query.
const PAD_POLYGON_SEGMENTS_PER_CORNER: usize = 12;

/// Arc-chord safety factor for oval pad polygonization: a polygon's
/// chord between two points sampled off a true circular arc always lies
/// strictly *inside* that arc, so approximating an oval pad's rounded
/// end-caps with straight polygon edges would, uncorrected, under-cover
/// the pad's real copper right at the corners -- exactly the kind of gap
/// a route could then be (wrongly) allowed to cross. Scaling the oval's
/// own `width`/`height` up by this factor before polygonizing (see
/// [`world_items`]) grows the *entire* shape uniformly from its own
/// center, so the resulting polygon always fully encloses the true
/// oval -- a few micrometres of extra, harmless keep-out margin, never
/// a real DRC gap.
const PAD_POLYGON_SAFETY_FACTOR: f64 = 1.02;

/// Builds one pad's true collision outline (in local, unrotated,
/// footprint-relative coordinates centered on the origin) via
/// [`Polygon::rounded_rect`], then rotates it by `rotation_deg` and
/// translates it to `center` -- shared by both the `Rect`
/// (`corner_radius = 0`, a plain sharp-cornered rectangle) and `Oval`
/// (`corner_radius = min(width, height) / 2`, a full stadium shape)
/// cases in [`world_items`].
fn pad_outline_polygon(
    width: Unit,
    height: Unit,
    corner_radius: Unit,
    rotation_deg: f64,
    center: Point,
) -> Polygon {
    let local = Polygon::rounded_rect(
        width,
        height,
        corner_radius,
        PAD_POLYGON_SEGMENTS_PER_CORNER,
    );
    Polygon::new(
        local
            .points
            .into_iter()
            .map(|p| p.rotated(rotation_deg).add(center))
            .collect(),
    )
}

/// Every pad of `template`, placed in world space at `position` rotated
/// by `rotation_deg`, as ready-to-check-or-commit `Item::Pad`s (`net:
/// None` -- nets are assigned afterwards via pin connect, not at
/// placement). This is the candidate geometry both placement and
/// dragging validate before ever touching the `Node`, and exactly what
/// gets `Node::add`ed on a successful commit.
///
/// `PadShapeKind::Rect`/`Oval` become a real `alladin_core::PadShape::Polygon`
/// -- the pad's own true, rotated outline, not a circular approximation
/// (see this module's doc comment) -- rotated by the pad's own local
/// `rotation_deg` *plus* this footprint's own placement `rotation_deg`,
/// the same total-rotation composition `crate::app`'s rendering already
/// uses. `PadShapeKind::Circle` stays a plain `PadShape::Circle`
/// (rotating a circle changes nothing).
pub fn world_items(template: &FootprintTemplate, position: Point, rotation_deg: f64) -> Vec<Item> {
    let pad_items = template.pads.iter().map(|pad| {
        let center = pad_world_position(pad.offset, position, rotation_deg);
        let total_rotation = pad.rotation_deg + rotation_deg;
        let shape = match pad.shape {
            PadShapeKind::Circle => PadShape::Circle(Circle::new(center, pad.radius)),
            PadShapeKind::Rect { width, height } => PadShape::Polygon {
                outline: pad_outline_polygon(width, height, 0, total_rotation, center),
                center,
            },
            PadShapeKind::Oval { width, height } => {
                let width = (width as f64 * PAD_POLYGON_SAFETY_FACTOR).round() as Unit;
                let height = (height as f64 * PAD_POLYGON_SAFETY_FACTOR).round() as Unit;
                let corner_radius = width.min(height) / 2;
                PadShape::Polygon {
                    outline: pad_outline_polygon(
                        width,
                        height,
                        corner_radius,
                        total_rotation,
                        center,
                    ),
                    center,
                }
            }
        };
        Item::Pad {
            shape,
            net: None,
            layer: pad.layer,
            zone_connection: pad.zone_connection,
            hole_diameter: pad.hole_diameter,
        }
    });
    let hole_items = template.holes.iter().map(|hole| Item::Hole {
        position: pad_world_position(hole.offset, position, rotation_deg),
        drill: hole.drill,
    });
    pad_items.chain(hole_items).collect()
}

/// Every scalar JLCPCB DFM rule this pad/hole geometry violates,
/// labelled by the offending pad's number (or `hole N` for a mechanical
/// hole) -- the footprint-level counterpart to `JlcpcbDfm::check_via`.
/// Takes raw slices rather than a whole [`FootprintTemplate`] so the
/// LCSC download path can validate a part *before* it ever becomes a
/// saved template. A pad's "smallest dimension" is a circle's diameter
/// or a rect/oval's narrower side; a through-hole pad's ring is
/// measured against that same conservative number (a hole is assumed
/// centred -- Alladin's pad model has no per-pad hole offset).
pub fn template_dfm_violations(
    pads: &[PadTemplate],
    holes: &[HoleTemplate],
) -> Vec<(String, DfmViolation)> {
    let mut violations = Vec::new();
    for pad in pads {
        let min_dimension = match pad.shape {
            PadShapeKind::Circle => pad.radius * 2,
            PadShapeKind::Rect { width, height } | PadShapeKind::Oval { width, height } => {
                width.min(height)
            }
        };
        let checked = match pad.hole_diameter {
            None => JlcpcbDfm::check_smd_pad(min_dimension),
            Some(drill) => JlcpcbDfm::check_pth_pad(min_dimension, drill),
        };
        if let Err(violation) = checked {
            violations.push((format!("pad {}", pad.number), violation));
        }
    }
    for (index, hole) in holes.iter().enumerate() {
        if let Err(violation) = JlcpcbDfm::check_npth_hole(hole.drill) {
            violations.push((format!("hole {}", index + 1), violation));
        }
    }
    violations
}

/// [`template_dfm_violations`] minus the report-only findings -- the
/// subset strict enough to *hard-refuse* a template at registration,
/// LCSC download, and placement.
///
/// Stays report-only (returned as download `dfm_warnings`, not a hard block):
/// - `PthAnnularRingBelowMin` -- Alladin models every through-hole as
///   one round drill; real connectors' oval *slots* (USB shells, ...)
///   legitimately show a sub-0.18mm ring under that crude model even
///   though JLCPCB's actual slot rules accept them.
/// - `SmdPadBelowMin` -- fine-pitch QFN/QFP parts from LCSC (RP2040,
///   …) routinely have pads under JLCPCB's 0.25mm "good" floor and
///   still assemble; hard-gating would block every real MCU footprint
///   while the same warning is still surfaced for review.
///
/// The rules kept here (drill floor, drill >= pad, NPTH floor) are
/// model-independent facts a fab cannot ignore.
pub fn template_dfm_hard_violations(
    pads: &[PadTemplate],
    holes: &[HoleTemplate],
) -> Vec<(String, DfmViolation)> {
    template_dfm_violations(pads, holes)
        .into_iter()
        .filter(|(_, v)| {
            !matches!(
                v,
                DfmViolation::PthAnnularRingBelowMin | DfmViolation::SmdPadBelowMin
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn placeable_builtins_are_only_the_pad_set_and_mounting_holes() {
        let names: Vec<_> = builtin_templates().into_iter().map(|t| t.name).collect();
        assert_eq!(
            names,
            vec![
                "Solder pad (SMD, 1.5mm, F.Cu)",
                "Solder pad (SMD, 1.5mm, B.Cu)",
                "Wire pad (PTH, 1.0mm hole)",
                "Wire pad (solder, 2mm)",
                "Wire pad (PTH, 2.0mm hole)",
                "Mounting hole (M2, NPTH)",
                "Mounting hole (M2.5, NPTH)",
                "Mounting hole (M3, NPTH)",
            ]
        );
        for name in [
            "2-pin THT (2.54mm pitch)",
            "4-pin THT header (2.54mm pitch)",
            "SOIC-8 (1.27mm pitch)",
        ] {
            assert!(is_legacy_demo_template(name));
            assert!(legacy_builtin_templates().iter().any(|t| t.name == name));
        }
    }

    #[test]
    fn builtin_templates_are_non_empty_and_every_pad_has_a_positive_radius() {
        let templates = builtin_templates();
        assert!(!templates.is_empty());
        for t in &templates {
            assert!(
                !t.pads.is_empty() || !t.holes.is_empty(),
                "{} has neither pads nor holes",
                t.name
            );
            for pad in &t.pads {
                assert!(pad.radius > 0, "{} has a non-positive pad radius", t.name);
            }
            for hole in &t.holes {
                assert!(hole.drill > 0, "{} has a non-positive hole drill", t.name);
            }
        }
    }

    #[test]
    fn every_builtin_template_passes_the_hard_scalar_dfm_gate() {
        // The gate runs on every placement -- a builtin that trips it
        // would be unplaceable, so this is a regression tripwire for
        // both the builtins and the gate's own thresholds.
        for t in builtin_templates() {
            assert!(
                template_dfm_hard_violations(&t.pads, &t.holes).is_empty(),
                "{} violates JLCPCB scalar DFM",
                t.name
            );
        }
    }

    #[test]
    fn template_dfm_violations_flags_a_sub_minimum_smd_pad_by_its_number() {
        // 0.1mm radius -> 0.2mm diameter, under the 0.25mm SMD floor.
        let pads = vec![PadTemplate::circle(
            Point::new(0, 0),
            100_000,
            LayerId::FCu,
            "7",
        )];
        let all = template_dfm_violations(&pads, &[]);
        assert_eq!(
            all,
            vec![("pad 7".to_string(), DfmViolation::SmdPadBelowMin)]
        );
        assert!(
            template_dfm_hard_violations(&pads, &[]).is_empty(),
            "SMD pad floor must stay report-only (fine-pitch QFN)"
        );
    }

    #[test]
    fn template_dfm_violations_measures_a_rect_pad_by_its_narrower_side() {
        let mut pad = PadTemplate::circle(Point::new(0, 0), 100_000, LayerId::FCu, "1");
        pad.shape = PadShapeKind::Rect {
            width: 1_000_000,
            height: 200_000,
        };
        assert_eq!(
            template_dfm_violations(&[pad.clone()], &[]).len(),
            1,
            "a 1.0 x 0.2mm pad is under the floor on its narrow side"
        );
        pad.shape = PadShapeKind::Rect {
            width: 1_000_000,
            height: 250_000,
        };
        assert!(
            template_dfm_violations(&[pad], &[]).is_empty(),
            "1.0 x 0.25mm sits exactly on the floor"
        );
    }

    #[test]
    fn a_thin_pth_annular_ring_is_reported_but_never_a_hard_violation() {
        // A 1.0mm pad over a 0.7mm drill: 0.15mm ring, under the 0.18mm
        // floor -- exactly the crude-model false positive a real oval
        // slot produces, so it must warn without blocking.
        let mut pad = PadTemplate::circle(Point::new(0, 0), 500_000, LayerId::FCu, "1");
        pad.hole_diameter = Some(700_000);
        let pads = vec![pad];
        assert_eq!(
            template_dfm_violations(&pads, &[]),
            vec![("pad 1".to_string(), DfmViolation::PthAnnularRingBelowMin)]
        );
        assert!(
            template_dfm_hard_violations(&pads, &[]).is_empty(),
            "the PTH ring must stay report-only"
        );
    }

    #[test]
    fn template_dfm_violations_flags_a_pth_drill_below_the_drillable_floor_as_hard() {
        let mut pad = PadTemplate::circle(Point::new(0, 0), 500_000, LayerId::FCu, "1");
        pad.hole_diameter = Some(100_000); // under the 0.15mm drill floor
        let pads = vec![pad];
        assert_eq!(
            template_dfm_hard_violations(&pads, &[]),
            vec![("pad 1".to_string(), DfmViolation::PthDrillBelowMin)]
        );
    }

    #[test]
    fn template_dfm_violations_flags_an_npth_hole_below_its_floor() {
        let holes = vec![HoleTemplate {
            offset: Point::new(0, 0),
            drill: 400_000,
        }]; // under 0.5mm
        assert_eq!(
            template_dfm_violations(&[], &holes),
            vec![("hole 1".to_string(), DfmViolation::NpthHoleBelowMin)]
        );
        assert_eq!(
            template_dfm_hard_violations(&[], &holes).len(),
            1,
            "a too-small NPTH hole is a hard violation"
        );
    }

    #[test]
    fn the_mechanical_builtin_templates_are_excluded_from_bom_and_the_electrical_ones_are_not() {
        let templates = builtin_templates();
        for t in &templates {
            let is_mechanical = t.name.starts_with("Wire pad")
                || t.name.starts_with("Solder pad")
                || t.name.starts_with("Mounting hole");
            assert_eq!(
                t.exclude_from_bom, is_mechanical,
                "unexpected exclude_from_bom for {}",
                t.name
            );
        }
    }

    #[test]
    fn mounting_hole_templates_have_a_hole_and_no_pads() {
        let templates = builtin_templates();
        let holes: Vec<_> = templates
            .iter()
            .filter(|t| t.name.starts_with("Mounting hole"))
            .collect();
        assert_eq!(holes.len(), 3, "expected M2/M2.5/M3 mounting holes");
        for t in holes {
            assert!(t.pads.is_empty());
            assert_eq!(t.holes.len(), 1);
        }
    }

    #[test]
    fn wire_pad_is_pth_with_a_fab_legal_drill_for_litze() {
        let t = builtin_templates()
            .into_iter()
            .find(|t| t.name == "Wire pad (solder, 2mm)")
            .expect("legacy wire pad builtin");
        assert_eq!(t.pads.len(), 1);
        assert_eq!(t.pads[0].hole_diameter, Some(mm(1.5)));
        assert_eq!(t.pads[0].radius, mm(1.25));
        assert_eq!(t.pads[0].zone_connection, ZoneConnection::Thermal);
        assert_eq!(
            JlcpcbDfm::check_pth_pad(t.pads[0].radius * 2, t.pads[0].hole_diameter.unwrap()),
            Ok(())
        );
        assert!(template_dfm_violations(&t.pads, &t.holes).is_empty());
    }

    #[test]
    fn solder_pad_set_covers_smd_both_sides_and_three_pth_gauges() {
        let templates = builtin_templates();
        let find = |name: &str| {
            templates
                .iter()
                .find(|t| t.name == name)
                .unwrap_or_else(|| panic!("missing {name}"))
        };

        let smd_f = find("Solder pad (SMD, 1.5mm, F.Cu)");
        assert_eq!(smd_f.pads[0].layer, LayerId::FCu);
        assert_eq!(smd_f.pads[0].radius, mm(0.75));
        assert_eq!(smd_f.pads[0].hole_diameter, None);
        assert_eq!(smd_f.pads[0].zone_connection, ZoneConnection::Thermal);

        let smd_b = find("Solder pad (SMD, 1.5mm, B.Cu)");
        assert_eq!(smd_b.pads[0].layer, LayerId::BCu);
        assert_eq!(smd_b.pads[0].hole_diameter, None);

        let small = find("Wire pad (PTH, 1.0mm hole)");
        assert_eq!(small.pads[0].hole_diameter, Some(mm(1.0)));
        assert_eq!(small.pads[0].radius, mm(1.0));
        assert_eq!(small.pads[0].zone_connection, ZoneConnection::Thermal);

        let large = find("Wire pad (PTH, 2.0mm hole)");
        assert_eq!(large.pads[0].hole_diameter, Some(mm(2.0)));
        assert_eq!(large.pads[0].radius, mm(1.6));
        assert_eq!(large.pads[0].zone_connection, ZoneConnection::Solid);

        for t in [small, find("Wire pad (solder, 2mm)"), large] {
            let pad = &t.pads[0];
            assert_eq!(
                JlcpcbDfm::check_pth_pad(pad.radius * 2, pad.hole_diameter.unwrap()),
                Ok(())
            );
            let items = world_items(t, Point::new(0, 0), 0.0);
            assert_eq!(items[0].layers(), (LayerId::FCu, Some(LayerId::BCu)));
        }
    }

    #[test]
    fn world_items_emits_one_item_hole_per_template_hole() {
        let template = FootprintTemplate {
            name: "test".to_string(),
            reference_prefix: "T".to_string(),
            pads: vec![PadTemplate::circle(
                Point::new(0, 0),
                mm(0.5),
                LayerId::FCu,
                "1",
            )],
            holes: vec![HoleTemplate {
                offset: Point::new(mm(1.0), 0),
                drill: mm(2.2),
            }],
            exclude_from_bom: false,
            explicit_courtyard: None,
        };
        let items = world_items(&template, Point::new(mm(10.0), mm(10.0)), 0.0);
        assert_eq!(items.len(), 2);
        let holes: Vec<_> = items
            .iter()
            .filter(|i| matches!(i, Item::Hole { .. }))
            .collect();
        assert_eq!(holes.len(), 1);
        match holes[0] {
            Item::Hole { position, drill } => {
                assert_eq!(*position, Point::new(mm(11.0), mm(10.0)));
                assert_eq!(*drill, mm(2.2));
            }
            _ => unreachable!(),
        }
    }

    #[test]
    fn world_items_rotates_a_hole_offset_the_same_way_as_a_pad_offset() {
        let template = FootprintTemplate {
            name: "test".to_string(),
            reference_prefix: "T".to_string(),
            pads: Vec::new(),
            holes: vec![HoleTemplate {
                offset: Point::new(mm(1.0), 0),
                drill: mm(2.2),
            }],
            exclude_from_bom: false,
            explicit_courtyard: None,
        };
        let items = world_items(&template, Point::new(0, 0), 90.0);
        match items[0] {
            Item::Hole { position, .. } => {
                assert!(
                    position.x.abs() < 100,
                    "expected x to vanish, got {position:?}"
                );
                assert!(
                    (position.y - mm(1.0)).abs() < 100,
                    "expected y to become +1mm, got {position:?}"
                );
            }
            _ => unreachable!(),
        }
    }

    #[test]
    fn pad_world_position_at_zero_rotation_is_a_plain_translation() {
        let offset = Point::new(mm(1.0), mm(2.0));
        let position = Point::new(mm(10.0), mm(10.0));
        let world = pad_world_position(offset, position, 0.0);
        assert_eq!(world, Point::new(mm(11.0), mm(12.0)));
    }

    #[test]
    fn pad_world_position_at_90_degrees_swaps_and_flips_the_offset() {
        let offset = Point::new(mm(1.0), mm(0.0));
        let world = pad_world_position(offset, Point::new(0, 0), 90.0);
        assert!((world.x).abs() < 100, "expected x to vanish, got {world:?}");
        assert!(
            (world.y - mm(1.0)).abs() < 100,
            "expected y to become +1mm, got {world:?}"
        );
    }

    #[test]
    fn world_items_produces_one_pad_per_template_pad() {
        let template = &builtin_templates()[0];
        let items = world_items(template, Point::new(0, 0), 0.0);
        assert_eq!(items.len(), template.pads.len());
        assert!(items
            .iter()
            .all(|item| matches!(item, Item::Pad { net: None, .. })));
    }

    #[test]
    fn courtyard_floors_undersized_silk_to_the_pad_bbox() {
        // Silk 2x2 centred, pads span 6mm wide -- union must cover pads.
        let template = FootprintTemplate {
            name: "t".into(),
            reference_prefix: "U".into(),
            pads: vec![
                PadTemplate::circle(Point::new(-mm(3.0), 0), mm(0.5), LayerId::FCu, "1"),
                PadTemplate::circle(Point::new(mm(3.0), 0), mm(0.5), LayerId::FCu, "2"),
            ],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: Some(Courtyard {
                center: Point::new(0, 0),
                width: mm(2.0),
                height: mm(2.0),
            }),
        };
        let c = template.courtyard();
        let pads = fallback_courtyard(&template.pads, &template.holes);
        assert!(
            c.width >= pads.width,
            "silk must not shrink below pad bbox, got {} vs {}",
            c.width,
            pads.width
        );
        assert!(c.height >= pads.height);
    }

    #[test]
    fn courtyard_keeps_silk_when_it_is_larger_than_pads() {
        let template = FootprintTemplate {
            name: "t".into(),
            reference_prefix: "U".into(),
            pads: vec![PadTemplate::circle(
                Point::new(0, 0),
                mm(0.5),
                LayerId::FCu,
                "1",
            )],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: Some(Courtyard {
                center: Point::new(0, 0),
                width: mm(8.0),
                height: mm(4.0),
            }),
        };
        let c = template.courtyard();
        assert_eq!(c.width, mm(8.0));
        assert_eq!(c.height, mm(4.0));
    }

    #[test]
    fn straight_row_template_centers_evenly_spaced_pads_on_the_origin() {
        let t = straight_row_template("R0805".to_string(), "R".to_string(), 2, 2.0, 0.5);
        assert_eq!(t.pads.len(), 2);
        assert_eq!(t.pads[0].offset, Point::new(-mm(1.0), 0));
        assert_eq!(t.pads[1].offset, Point::new(mm(1.0), 0));
        assert_eq!(t.pads[0].radius, mm(0.5));
    }

    #[test]
    fn straight_row_template_clamps_to_at_least_one_pin() {
        let t = straight_row_template("weird".to_string(), "X".to_string(), 0, 2.0, 0.5);
        assert_eq!(
            t.pads.len(),
            1,
            "zero pins doesn't make sense, must fall back to one"
        );
    }

    #[test]
    fn built_in_and_hand_added_pads_are_numbered_sequentially_from_one() {
        for t in builtin_templates() {
            let numbers: Vec<String> = t.pads.iter().map(|p| p.number.clone()).collect();
            let expected: Vec<String> = (1..=t.pads.len()).map(|n| n.to_string()).collect();
            assert_eq!(numbers, expected, "{} isn't numbered 1, 2, 3, ...", t.name);
            assert!(
                t.pads.iter().all(|p| p.shape == PadShapeKind::Circle),
                "{} pads must render as plain circles",
                t.name
            );
        }
    }

    fn rect_pad_template(width: Unit, height: Unit, rotation_deg: f64) -> PadTemplate {
        PadTemplate {
            offset: Point::new(0, 0),
            radius: width.min(height) / 2,
            layer: LayerId::FCu,
            number: "1".to_string(),
            shape: PadShapeKind::Rect { width, height },
            rotation_deg,
            hole_diameter: None,
            pin_name: None,
            zone_connection: ZoneConnection::Thermal,
        }
    }

    fn oval_pad_template(width: Unit, height: Unit, rotation_deg: f64) -> PadTemplate {
        PadTemplate {
            offset: Point::new(0, 0),
            radius: width.min(height) / 2,
            layer: LayerId::FCu,
            number: "1".to_string(),
            shape: PadShapeKind::Oval { width, height },
            rotation_deg,
            hole_diameter: None,
            pin_name: None,
            zone_connection: ZoneConnection::Thermal,
        }
    }

    #[test]
    fn world_items_keeps_a_circle_pad_as_padshape_circle() {
        let template = FootprintTemplate {
            name: "t".to_string(),
            reference_prefix: "P".to_string(),
            pads: vec![PadTemplate {
                offset: Point::new(mm(1.0), 0),
                radius: mm(0.5),
                layer: LayerId::FCu,
                number: "1".to_string(),
                shape: PadShapeKind::Circle,
                rotation_deg: 0.0,
                hole_diameter: None,
                pin_name: None,
                zone_connection: ZoneConnection::Thermal,
            }],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: None,
        };
        let items = world_items(&template, Point::new(mm(10.0), mm(10.0)), 0.0);
        let Item::Pad { shape, .. } = &items[0] else {
            panic!("expected a pad")
        };
        assert_eq!(
            *shape,
            PadShape::Circle(Circle::new(Point::new(mm(11.0), mm(10.0)), mm(0.5)))
        );
    }

    #[test]
    fn world_items_builds_a_true_rectangular_polygon_for_a_rect_pad() {
        let template = FootprintTemplate {
            name: "t".to_string(),
            reference_prefix: "P".to_string(),
            pads: vec![rect_pad_template(mm(2.0), mm(1.0), 0.0)],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: None,
        };
        let items = world_items(&template, Point::new(0, 0), 0.0);
        let Item::Pad { shape, .. } = &items[0] else {
            panic!("expected a pad")
        };
        let PadShape::Polygon { outline, center } = shape else {
            panic!("a rect pad must collide as a polygon, not a circle")
        };
        assert_eq!(*center, Point::new(0, 0));
        // A 2mm x 1mm rect's corner must reach its own half-width/half-height
        // exactly (sharp corners, no rounding, no safety-factor inflation --
        // that's only for `Oval`) -- not the old inscribed-circle radius
        // (0.5mm) that used to under-cover this exact corner.
        let far_corner = Point::new(mm(1.0), mm(0.5));
        assert!(
            outline.points.iter().any(|p| p.distance(far_corner) < 10.0),
            "expected a sharp corner at the rect's true half-width/half-height, got {:?}",
            outline.points
        );
        assert!(
            outline.contains_point(Point::new(mm(0.9), mm(0.4))),
            "must cover copper well inside the true rectangle"
        );
        assert!(
            !outline.contains_point(Point::new(mm(1.1), 0)),
            "must not extend past the rectangle's true edge"
        );
    }

    #[test]
    fn world_items_rotates_a_rect_pad_by_its_own_rotation_plus_the_footprints_placement_rotation() {
        // A 2mm x 1mm pad, pre-rotated 90 degrees inside its own footprint
        // (so its long axis now points along Y), on a footprint placed with
        // an *additional* 90 degree rotation -- the two must compose to a
        // net 180 degrees, i.e. the long axis ends up back on X.
        let template = FootprintTemplate {
            name: "t".to_string(),
            reference_prefix: "P".to_string(),
            pads: vec![rect_pad_template(mm(2.0), mm(1.0), 90.0)],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: None,
        };
        let items = world_items(&template, Point::new(0, 0), 90.0);
        let Item::Pad { shape, .. } = &items[0] else {
            panic!("expected a pad")
        };
        let PadShape::Polygon { outline, .. } = shape else {
            panic!("expected a polygon")
        };
        assert!(
            outline.contains_point(Point::new(mm(0.9), 0)),
            "long axis should have rotated back onto X"
        );
        assert!(
            !outline.contains_point(Point::new(0, mm(0.9))),
            "short axis should now be along Y"
        );
    }

    #[test]
    fn world_items_grows_an_oval_pad_slightly_to_never_under_cover_its_true_rounded_ends() {
        let template = FootprintTemplate {
            name: "t".to_string(),
            reference_prefix: "P".to_string(),
            pads: vec![oval_pad_template(mm(2.0), mm(1.0), 0.0)],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: None,
        };
        let items = world_items(&template, Point::new(0, 0), 0.0);
        let Item::Pad { shape, .. } = &items[0] else {
            panic!("expected a pad")
        };
        let PadShape::Polygon { outline, center } = shape else {
            panic!("an oval pad must collide as a polygon, not a circle")
        };
        assert_eq!(*center, Point::new(0, 0));
        // The true oval's rightmost point is exactly at x = 1mm; the
        // polygonized, safety-factor-inflated outline must fully enclose
        // it (reach at least as far), never stop a hair short.
        assert!(
            outline.contains_point(Point::new(mm(1.0) - 1000, 0)),
            "must not under-cover the oval's true long-axis tip"
        );
        assert!(outline.contains_point(Point::new(0, 0)));
    }

    #[test]
    fn zone_connection_heuristic_marks_ep_grid_and_large_pads_solid() {
        let mut pads = vec![
            PadTemplate {
                offset: Point::new(0, 0),
                radius: mm(0.3),
                layer: LayerId::FCu,
                number: "1".into(),
                shape: PadShapeKind::Circle,
                rotation_deg: 0.0,
                hole_diameter: None,
                pin_name: None,
                zone_connection: ZoneConnection::Thermal,
            },
            PadTemplate {
                offset: Point::new(mm(1.0), 0),
                radius: mm(0.25),
                layer: LayerId::FCu,
                number: "9".into(),
                shape: PadShapeKind::Circle,
                rotation_deg: 0.0,
                hole_diameter: None,
                pin_name: None,
                zone_connection: ZoneConnection::Thermal,
            },
            PadTemplate {
                offset: Point::new(mm(1.5), 0),
                radius: mm(0.25),
                layer: LayerId::FCu,
                number: "9".into(),
                shape: PadShapeKind::Circle,
                rotation_deg: 0.0,
                hole_diameter: None,
                pin_name: None,
                zone_connection: ZoneConnection::Thermal,
            },
            {
                let mut large = rect_pad_template(mm(3.0), mm(3.0), 0.0);
                large.number = "EP".into();
                large
            },
        ];
        apply_zone_connection_heuristic(&mut pads);
        assert_eq!(pads[0].zone_connection, ZoneConnection::Thermal);
        assert_eq!(
            pads[1].zone_connection,
            ZoneConnection::Solid,
            "EP grid pad 9"
        );
        assert_eq!(
            pads[2].zone_connection,
            ZoneConnection::Solid,
            "EP grid pad 9"
        );
        assert_eq!(
            pads[3].zone_connection,
            ZoneConnection::Solid,
            "large pad ≥ 2mm"
        );
    }

    #[test]
    fn zone_connection_heuristic_marks_named_ep_and_dominant_center_solid() {
        let mut named = vec![PadTemplate {
            offset: Point::new(0, 0),
            radius: mm(0.3),
            layer: LayerId::FCu,
            number: "EP".into(),
            shape: PadShapeKind::Circle,
            rotation_deg: 0.0,
            hole_diameter: None,
            pin_name: None,
            zone_connection: ZoneConnection::Thermal,
        }];
        apply_zone_connection_heuristic(&mut named);
        assert_eq!(
            named[0].zone_connection,
            ZoneConnection::Solid,
            "pad number EP"
        );

        // RP2040-style: many tiny edge pads + one large center numbered "57".
        let mut qfn = Vec::new();
        qfn.push({
            let mut ep = rect_pad_template(mm(3.1), mm(3.1), 0.0);
            ep.number = "57".into();
            ep
        });
        for i in 0..8 {
            let mut p = rect_pad_template(mm(0.85), mm(0.2), 0.0);
            p.number = format!("{}", i + 1);
            p.offset = Point::new(mm(-2.6 + 0.4 * i as f64), mm(-3.4));
            qfn.push(p);
        }
        apply_zone_connection_heuristic(&mut qfn);
        assert_eq!(
            qfn[0].zone_connection,
            ZoneConnection::Solid,
            "dominant center / ≥2mm EP"
        );
        assert!(qfn[1..]
            .iter()
            .all(|p| p.zone_connection == ZoneConnection::Thermal));
    }
}
