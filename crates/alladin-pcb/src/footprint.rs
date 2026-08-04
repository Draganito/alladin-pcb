//! Built-in footprint templates for manual part placement, plus the
//! shared `PadTemplate`/`FootprintTemplate` shapes used by
//! `crate::parts_db` (the user's own saved parts) and `crate::lcsc`
//! (real LCSC/EasyEDA downloads).
//!
//! **Collision/routing geometry uses the pad's *true* shape** (see
//! `alladin_core::PadShape`), not a circular approximation --
//! [`world_items`] builds a real `PadShape::Polygon` (rotated,
//! world-positioned) for `PadShapeKind::Rect`/`Oval`, and only a plain
//! `PadShape::Circle` for `PadShapeKind::Circle`. This replaced an
//! earlier "inscribed circle" compromise (`radius = min(width, height)
//! / 2`) that was necessary while `alladin-core`/`alladin-router` had no
//! notion of a non-circular pad at all -- see the development log's
//! "Echte Pad-Geometrie" slice entry for the full story of why that
//! compromise was retired, and its "Siebter MVP-Slice" entry for the
//! compromise's own original history. [`PadTemplate::radius`] is now a
//! **derived, secondary** value (still `min(width, height) / 2`, still
//! used for the parts-database column, hole-size estimation, and KiCad
//! export -- see `crate::lcsc`'s module doc comment), not the collision
//! shape itself.
//!
//! What real, downloaded parts get to look *right* **and** route
//! correctly: every `PadTemplate` carries its true `shape`
//! (circle/rect/oval) and `rotation_deg`, used both for on-screen
//! rendering (`crate::app`'s pad drawing) and, since this slice, for
//! [`world_items`]'s actual collision polygon.

use alladin_core::{Item, LayerId, PadShape};
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
    /// hole-size estimation, and KiCad export (see `crate::lcsc`'s
    /// module doc comment), **not** the collision shape itself anymore
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
    /// Only ever populated by real
    /// downloads (`crate::lcsc`, from EasyEDA's own hole diameter) --
    /// every built-in/hand-added template pad is `None` (SMD) today,
    /// see [`Self::circle`]. Exists specifically so a real KiCad export
    /// (`crate::kicad_export`) can write a correct `thru_hole` pad with
    /// its actual drill size instead of silently downgrading every
    /// through-hole part (headers, THT connectors, ...) to SMD, which
    /// would produce an unmanufacturable board.
    pub hole_diameter: Option<Unit>,
    /// The pin's *function*, as the schematic symbol names it (`"GND"`,
    /// `"VDD"`, `"DIN"`, `"DOUT"`, ...) -- distinct from [`Self::number`]
    /// (the pad's position/designator, e.g. `"3"`) and from the *board*
    /// net it ends up wired to after `Connect`. Only ever populated by
    /// real LCSC/EasyEDA downloads (`crate::lcsc`, from the schematic
    /// symbol SVG, which is the only place this fact lives -- the
    /// footprint/PCB data alone never carries it, see that module's doc
    /// comment); `None` for every built-in/hand-added template and for
    /// KiCad imports (`crate::kicad_import`), which have no schematic
    /// data to draw it from either.
    pub pin_name: Option<String>,
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
        }
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
    /// ordinary electrical part (the 3 fixed [`builtin_templates`],
    /// every LCSC download, every `crate::parts_db::PartsDb::insert_part`
    /// hand-added part so far). Populated only by the two dedicated
    /// mechanical builtins ([`mounting_hole_template`]) and by a real
    /// KiCad import's own `np_thru_hole` pads (`crate::kicad_import`).
    pub holes: Vec<HoleTemplate>,
    /// Whether this template's own placed instances should be silently
    /// skipped by `crate::bom::build_bom_rows` -- purely mechanical
    /// parts (mounting holes, solder wire pads) are never a purchasable
    /// line item, matching how the *original* KiCad board this pipeline
    /// is meant to reproduce already marks them "Exclude from BOM" (see
    /// `crate::bom`'s module doc comment). `false` for every existing
    /// template (electrical parts are still included, unchanged
    /// behaviour) -- only [`wire_pad_template`]/[`mounting_hole_template`]
    /// and a user's own `register-part --exclude-from-bom` set this.
    pub exclude_from_bom: bool,
    /// The part's own real silkscreen/assembly-layer body outline, if
    /// [`crate::lcsc`] found one in the downloaded footprint's raw
    /// data -- `None` for every built-in/hand-added/KiCad-imported
    /// template (none of those carry real body dimensions at all),
    /// and also `None` for plenty of real SMD parts whose footprint
    /// draws no dedicated body outline at all (a 0603 resistor, say).
    /// Use [`Self::courtyard`], never this field directly, to always
    /// get a real rectangle either way -- see that method's own
    /// fallback.
    pub explicit_courtyard: Option<Courtyard>,
}

impl FootprintTemplate {
    /// This template's own mechanical keep-out rectangle: the real
    /// silkscreen/body outline if [`Self::explicit_courtyard`] has
    /// one, else the bounding box of every one of its own pads and
    /// holes (see [`fallback_courtyard`]). Two placed footprints'
    /// *bodies* must never overlap (see `crate::board_doc::check_placement`'s
    /// own gate) even where their copper pads happen not to -- exactly
    /// the "small part hidden underneath a big module" mistake a
    /// pads-only collision check can never catch.
    pub fn courtyard(&self) -> Courtyard {
        self.explicit_courtyard.unwrap_or_else(|| fallback_courtyard(&self.pads, &self.holes))
    }
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
        PadShapeKind::Rect { width, height } | PadShapeKind::Oval { width, height } => (width / 2, height / 2),
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
/// (see that module's doc comment), and every hand-added/built-in/
/// KiCad-imported template's *only* source, since none of those carry
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
            Point::new(hole.offset.x - hole.drill / 2, hole.offset.y - hole.drill / 2),
            Point::new(hole.offset.x + hole.drill / 2, hole.offset.y + hole.drill / 2),
        ] {
            any = true;
            min.x = min.x.min(corner.x);
            min.y = min.y.min(corner.y);
            max.x = max.x.max(corner.x);
            max.y = max.y.max(corner.y);
        }
    }
    if !any {
        return Courtyard { center: Point::new(0, 0), width: 0, height: 0 };
    }
    Courtyard { center: Point::new((min.x + max.x) / 2, (min.y + max.y) / 2), width: max.x - min.x, height: max.y - min.y }
}

/// The world-space courtyard rectangle of `template` placed at
/// `position`/`rotation_deg` -- same rotate-then-translate convention
/// as every pad (see [`pad_world_position`]/[`world_items`]).
/// Deliberately kept out of [`world_items`]/`Item`/`Node` entirely: a
/// courtyard has no copper and must never itself become a
/// collidable/DRC item (a routed track legitimately runs underneath
/// one) -- `crate::board_doc`'s own body-vs-body overlap check is the
/// only consumer.
pub fn world_courtyard(template: &FootprintTemplate, position: Point, rotation_deg: f64) -> Polygon {
    let courtyard = template.courtyard();
    let center = pad_world_position(courtyard.center, position, rotation_deg);
    pad_outline_polygon(courtyard.width, courtyard.height, 0, rotation_deg, center)
}

/// The fixed template library this MVP slice ships with. Deliberately
/// small and deliberately generic (pitch/pad-size only, no real part
/// semantics) -- enough to exercise placement, collision-locking, and
/// (in a follow-up step) interactive routing between real pads, without
/// waiting on the parts-database work.
pub fn builtin_templates() -> Vec<FootprintTemplate> {
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
                        Point::new(row_side as Unit * mm(2.65), mm(1.27) * (index_in_row as Unit) - mm(1.27 * 1.5)),
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
        wire_pad_template(),
        mounting_hole_template("M2", mm(2.2)),
        mounting_hole_template("M2.5", mm(2.7)),
        mounting_hole_template("M3", mm(3.2)),
    ]
}

/// A single, generic solder pad for a wire connection (power input,
/// ground strap, a hand-soldered jumper, ...) -- exactly the "Lötpads
/// für Strom usw." gap the "drawing-to-manufacturing" pipeline plan
/// identified: a board frequently needs *somewhere* to solder a bare
/// wire that isn't a real component with an LCSC part number.
/// `exclude_from_bom: true` (see that field's own doc comment): a bare
/// solder pad is never a purchasable BOM line item, matching how the
/// original board this pipeline targets already marks its own wire
/// pads "Exclude from BOM".
fn wire_pad_template() -> FootprintTemplate {
    FootprintTemplate {
        name: "Wire pad (solder, 2mm)".to_string(),
        reference_prefix: "W".to_string(),
        pads: vec![PadTemplate::circle(Point::new(0, 0), mm(1.0), LayerId::FCu, "1")],
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
        holes: vec![HoleTemplate { offset: Point::new(0, 0), drill }],
        exclude_from_bom: true,
        explicit_courtyard: None,
    }
}

/// A straight row of THT pads, `pin_count` pads spaced `pitch` apart,
/// centered on the footprint's own origin -- the one parametric shape
/// [`crate::parts_db`]'s "Add part..." form can generate today. Not a
/// real parts-database import (that's LCSC/EasyEDA API integration, a
/// separate later step -- see the development log's "Teil 29" entry),
/// just enough to let a user register their *own* simple through-hole
/// parts (resistors, headers, ...) without waiting for it.
pub fn straight_row_template(name: String, reference_prefix: String, pin_count: u32, pitch_mm: f64, pad_radius_mm: f64) -> FootprintTemplate {
    let pitch = mm(pitch_mm);
    let radius = mm(pad_radius_mm);
    let span = pitch * (pin_count.max(1) as Unit - 1);
    let pads = (0..pin_count.max(1))
        .map(|i| PadTemplate::circle(Point::new(pitch * i as Unit - span / 2, 0), radius, LayerId::FCu, (i + 1).to_string()))
        .collect();
    FootprintTemplate { name, reference_prefix, pads, holes: Vec::new(), exclude_from_bom: false, explicit_courtyard: None }
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
    Point::new(position.x + x.round() as Unit, position.y + y.round() as Unit)
}

/// Number of straight segments used to polygonize each rounded corner
/// of a `PadShapeKind::Oval` pad's stadium shape -- matches
/// `crate::board_doc`'s own `Polygon::rounded_rect` call for the board
/// outline, both deliberately generous (12 rather than
/// `alladin-router`'s tighter, hot-path `ARC_SEGMENTS`/`CIRCLE_SEGMENTS`
/// budgets) since this only ever runs once per placement/drag frame per
/// footprint, not once per A* candidate-point query.
const PAD_POLYGON_SEGMENTS_PER_CORNER: usize = 12;

/// Same role as `alladin-router`'s own `ARC_SAFETY_FACTOR` (see
/// `walkaround.rs`'s doc comment there): a polygon's chord between two
/// points sampled off a true circular arc always lies strictly *inside*
/// that arc, so approximating an oval pad's rounded end-caps with
/// straight polygon edges would, uncorrected, under-cover the pad's
/// real copper right at the corners -- exactly the kind of gap a route
/// could then be (wrongly) allowed to cross. Scaling the oval's own
/// `width`/`height` up by this factor before polygonizing (see
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
fn pad_outline_polygon(width: Unit, height: Unit, corner_radius: Unit, rotation_deg: f64, center: Point) -> Polygon {
    let local = Polygon::rounded_rect(width, height, corner_radius, PAD_POLYGON_SEGMENTS_PER_CORNER);
    Polygon::new(local.points.into_iter().map(|p| p.rotated(rotation_deg).add(center)).collect())
}

/// Every pad of `template`, placed in world space at `position` rotated
/// by `rotation_deg`, as ready-to-check-or-commit `Item::Pad`s (`net:
/// None` -- footprints placed by this MVP slice aren't wired to any net
/// yet, see the development log's "Teil 29" MVP-order decision). This
/// is the candidate geometry both placement and dragging validate before
/// ever touching the `Node`, and exactly what gets `Node::add`ed on a
/// successful commit.
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
            PadShapeKind::Rect { width, height } => {
                PadShape::Polygon { outline: pad_outline_polygon(width, height, 0, total_rotation, center), center }
            }
            PadShapeKind::Oval { width, height } => {
                let width = (width as f64 * PAD_POLYGON_SAFETY_FACTOR).round() as Unit;
                let height = (height as f64 * PAD_POLYGON_SAFETY_FACTOR).round() as Unit;
                let corner_radius = width.min(height) / 2;
                PadShape::Polygon { outline: pad_outline_polygon(width, height, corner_radius, total_rotation, center), center }
            }
        };
        Item::Pad { shape, net: None, layer: pad.layer }
    });
    let hole_items = template
        .holes
        .iter()
        .map(|hole| Item::Hole { position: pad_world_position(hole.offset, position, rotation_deg), drill: hole.drill });
    pad_items.chain(hole_items).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builtin_templates_are_non_empty_and_every_pad_has_a_positive_radius() {
        let templates = builtin_templates();
        assert!(!templates.is_empty());
        for t in &templates {
            assert!(!t.pads.is_empty() || !t.holes.is_empty(), "{} has neither pads nor holes", t.name);
            for pad in &t.pads {
                assert!(pad.radius > 0, "{} has a non-positive pad radius", t.name);
            }
            for hole in &t.holes {
                assert!(hole.drill > 0, "{} has a non-positive hole drill", t.name);
            }
        }
    }

    #[test]
    fn the_mechanical_builtin_templates_are_excluded_from_bom_and_the_electrical_ones_are_not() {
        let templates = builtin_templates();
        for t in &templates {
            let is_mechanical = t.name.starts_with("Wire pad") || t.name.starts_with("Mounting hole");
            assert_eq!(t.exclude_from_bom, is_mechanical, "unexpected exclude_from_bom for {}", t.name);
        }
    }

    #[test]
    fn mounting_hole_templates_have_a_hole_and_no_pads() {
        let templates = builtin_templates();
        let holes: Vec<_> = templates.iter().filter(|t| t.name.starts_with("Mounting hole")).collect();
        assert_eq!(holes.len(), 3, "expected M2/M2.5/M3 mounting holes");
        for t in holes {
            assert!(t.pads.is_empty());
            assert_eq!(t.holes.len(), 1);
        }
    }

    #[test]
    fn world_items_emits_one_item_hole_per_template_hole() {
        let template = FootprintTemplate {
            name: "test".to_string(),
            reference_prefix: "T".to_string(),
            pads: vec![PadTemplate::circle(Point::new(0, 0), mm(0.5), LayerId::FCu, "1")],
            holes: vec![HoleTemplate { offset: Point::new(mm(1.0), 0), drill: mm(2.2) }],
            exclude_from_bom: false,
            explicit_courtyard: None,
        };
        let items = world_items(&template, Point::new(mm(10.0), mm(10.0)), 0.0);
        assert_eq!(items.len(), 2);
        let holes: Vec<_> = items.iter().filter(|i| matches!(i, Item::Hole { .. })).collect();
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
            holes: vec![HoleTemplate { offset: Point::new(mm(1.0), 0), drill: mm(2.2) }],
            exclude_from_bom: false,
            explicit_courtyard: None,
        };
        let items = world_items(&template, Point::new(0, 0), 90.0);
        match items[0] {
            Item::Hole { position, .. } => {
                assert!(position.x.abs() < 100, "expected x to vanish, got {position:?}");
                assert!((position.y - mm(1.0)).abs() < 100, "expected y to become +1mm, got {position:?}");
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
        assert!((world.y - mm(1.0)).abs() < 100, "expected y to become +1mm, got {world:?}");
    }

    #[test]
    fn world_items_produces_one_pad_per_template_pad() {
        let template = &builtin_templates()[0];
        let items = world_items(template, Point::new(0, 0), 0.0);
        assert_eq!(items.len(), template.pads.len());
        assert!(items.iter().all(|item| matches!(item, Item::Pad { net: None, .. })));
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
        assert_eq!(t.pads.len(), 1, "zero pins doesn't make sense, must fall back to one");
    }

    #[test]
    fn built_in_and_hand_added_pads_are_numbered_sequentially_from_one() {
        for t in builtin_templates() {
            let numbers: Vec<String> = t.pads.iter().map(|p| p.number.clone()).collect();
            let expected: Vec<String> = (1..=t.pads.len()).map(|n| n.to_string()).collect();
            assert_eq!(numbers, expected, "{} isn't numbered 1, 2, 3, ...", t.name);
            assert!(t.pads.iter().all(|p| p.shape == PadShapeKind::Circle), "{} pads must render as plain circles", t.name);
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
            }],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: None,
        };
        let items = world_items(&template, Point::new(mm(10.0), mm(10.0)), 0.0);
        let Item::Pad { shape, .. } = &items[0] else { panic!("expected a pad") };
        assert_eq!(*shape, PadShape::Circle(Circle::new(Point::new(mm(11.0), mm(10.0)), mm(0.5))));
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
        let Item::Pad { shape, .. } = &items[0] else { panic!("expected a pad") };
        let PadShape::Polygon { outline, center } = shape else { panic!("a rect pad must collide as a polygon, not a circle") };
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
        assert!(outline.contains_point(Point::new(mm(0.9), mm(0.4))), "must cover copper well inside the true rectangle");
        assert!(!outline.contains_point(Point::new(mm(1.1), 0)), "must not extend past the rectangle's true edge");
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
        let Item::Pad { shape, .. } = &items[0] else { panic!("expected a pad") };
        let PadShape::Polygon { outline, .. } = shape else { panic!("expected a polygon") };
        assert!(outline.contains_point(Point::new(mm(0.9), 0)), "long axis should have rotated back onto X");
        assert!(!outline.contains_point(Point::new(0, mm(0.9))), "short axis should now be along Y");
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
        let Item::Pad { shape, .. } = &items[0] else { panic!("expected a pad") };
        let PadShape::Polygon { outline, center } = shape else { panic!("an oval pad must collide as a polygon, not a circle") };
        assert_eq!(*center, Point::new(0, 0));
        // The true oval's rightmost point is exactly at x = 1mm; the
        // polygonized, safety-factor-inflated outline must fully enclose
        // it (reach at least as far), never stop a hair short.
        assert!(outline.contains_point(Point::new(mm(1.0) - 1000, 0)), "must not under-cover the oval's true long-axis tip");
        assert!(outline.contains_point(Point::new(0, 0)));
    }
}
