//! Internal `.kicad_pcb` reader -- **not** a user-facing product feature.
//! Counterpart of `crate::kicad_export`: the external autorouter
//! (`crate::external_router`) writes a board out, runs KiCadRoutingTools,
//! and merges the routed result back through this module. Alladin's
//! editable format remains its own `.json`.
//!
//! Deliberately built on [`alladin_kicad_io::import_footprints`] (full
//! pad-shape/rotation/drill fidelity per footprint), **not**
//! [`alladin_kicad_io::import_kicad_pcb`]'s flat `Node` of bare
//! bounding-circle pads -- that path is exactly right for
//! `alladin-router`'s routing obstacles, but throws away every bit of
//! per-footprint structure (which pads belong to which part, its
//! reference designator, real pad shape/number/rotation) this editor
//! needs to keep treating an imported part as a real, movable,
//! re-exportable footprint rather than a frozen pile of circles. Still
//! calls `import_kicad_pcb` too, for the one thing it already does well
//! and `import_footprints` doesn't touch at all: the board outline and
//! already-routed tracks/vias/zones.
//!
//! **Every imported footprint bypasses [`BoardDoc::check_placement`]**
//! (via [`BoardDoc::insert_footprint_unchecked`]) -- see that method's
//! doc comment for why: a file real KiCad already accepted is by
//! definition already geometrically legal, and Alladin's own
//! circle-only collision approximation could otherwise reject valid
//! real geometry it wasn't conservative enough to model exactly.
//!
//! **Cross-board name collisions**: a generated template's name is the
//! *only* key `persistence.rs` uses to resolve a placed footprint back
//! to its template. Two different imported boards very plausibly share
//! a reference designator (`D1`/`U1`/`R1` are generic KiCad
//! auto-references, not globally unique across unrelated projects) --
//! without a real fix, both imports would persist a `parts_db` row
//! under the *identical* name, and whichever has the lower row id would
//! silently win for every board that references that name, including
//! ones that meant the other board's geometry. [`import_kicad_pcb_with_label`]
//! avoids this by embedding the source board's own label into every
//! generated name (see [`template_name_for`]) *and* into its `parts_db`
//! category (see [`persist_templates`]) -- callers that persist imported
//! templates into the parts DB should use it, not the plain unlabeled
//! [`import_kicad_pcb`]. Since the public KiCad import feature was
//! removed from the product, the only production caller left is the
//! external-autorouter bridge (`crate::external_router`), which reads
//! back tracks/vias only; the template-persistence path survives here
//! because the round-trip tests below still prove it correct.

use alladin_core::{Item, LayerId, Node};
use alladin_kicad_io::{ImportError, ImportedFootprint, WritePadShape};

use crate::board_doc::{BoardDoc, LayerCount};
use crate::footprint::{FootprintTemplate, PadShapeKind, PadTemplate};
use crate::parts_db::PartsDb;

/// The read-side mirror of `crate::kicad_export`'s pad-radius rule (see
/// `crate::footprint`'s and `crate::lcsc`'s module doc comments for the
/// full history): `min(width, height) / 2`, **not** the larger side and
/// **not** the bounding half-diagonal. Using the shorter side is what
/// stops neighbouring pads on a tightly-pitched real part (the exact
/// castellated-module case that motivated this rule originally) from
/// registering as colliding with each other the moment they're
/// reimported and something tries to route near them.
fn collision_radius(w: alladin_geom::Unit, h: alladin_geom::Unit) -> alladin_geom::Unit {
    (w.min(h) / 2).max(1)
}

fn pad_shape_kind(shape: WritePadShape) -> PadShapeKind {
    match shape {
        WritePadShape::Circle { .. } => PadShapeKind::Circle,
        WritePadShape::Rect { width, height } => PadShapeKind::Rect { width, height },
        WritePadShape::Oval { width, height } => PadShapeKind::Oval { width, height },
    }
}

fn shape_dims(shape: WritePadShape) -> (alladin_geom::Unit, alladin_geom::Unit) {
    match shape {
        WritePadShape::Circle { diameter } => (diameter, diameter),
        WritePadShape::Rect { width, height } => (width, height),
        WritePadShape::Oval { width, height } => (width, height),
    }
}

/// Derives an auto-reference *prefix* (`"J1"` -> `"J"`, `"U12"` ->
/// `"U"`) from a real file's own reference designator, purely so any
/// *new* part placed after import still gets a plausible-looking
/// auto-generated reference -- it plays no role in reconstructing the
/// imported part itself, whose own `reference` is kept verbatim (see
/// this module's doc comment).
fn reference_prefix_of(reference: &str) -> String {
    let prefix: String = reference.chars().take_while(|c| c.is_ascii_alphabetic()).collect();
    if prefix.is_empty() {
        "P".to_string()
    } else {
        prefix
    }
}

/// `board_label` is empty for every unlabeled call (see
/// [`import_kicad_pcb`]'s own doc comment) -- producing exactly
/// today's plain `"Imported: {reference}"`/`"Imported footprint #N"`,
/// unchanged. A real, non-empty `board_label` (see
/// [`import_kicad_pcb_with_label`]) embeds the *source board's own*
/// label right into the name, so the exact same reference imported
/// from two different boards (extremely likely -- `D1`/`U1`/`R1` are
/// generic KiCad auto-references, not globally unique) never produces
/// two identically-named `parts_db` rows that could resolve to the
/// wrong board's geometry -- see this module's own doc comment for why
/// that used to be a real, live risk.
fn template_name_for(reference: &str, index: usize, board_label: &str) -> String {
    match (reference.is_empty(), board_label.is_empty()) {
        (true, true) => format!("Imported footprint #{}", index + 1),
        (true, false) => format!("Imported [{board_label}] footprint #{}", index + 1),
        (false, true) => format!("Imported: {reference}"),
        (false, false) => format!("Imported [{board_label}]: {reference}"),
    }
}

fn build_template(fp: &ImportedFootprint, index: usize, board_label: &str) -> FootprintTemplate {
    let pads: Vec<PadTemplate> = fp
        .pads
        .iter()
        .map(|pad| {
            let (w, h) = shape_dims(pad.shape);
            PadTemplate {
                offset: pad.offset,
                radius: collision_radius(w, h),
                layer: pad.layer,
                number: pad.number.clone(),
                shape: pad_shape_kind(pad.shape),
                rotation_deg: pad.rotation_deg,
                hole_diameter: pad.drill,
                pin_name: None,
            }
        })
        .collect();

    let holes = fp.holes.iter().map(|h| crate::footprint::HoleTemplate { offset: h.offset, drill: h.drill }).collect::<Vec<_>>();
    // A footprint whose *only* content is mechanical holes (a real
    // `MountingHole:*` library part, the exact case this session's
    // LED-panel board actually has) is never a purchasable BOM line --
    // matches `crate::footprint::mounting_hole_template`'s own
    // `exclude_from_bom: true` for the same shape of template. A
    // footprint that has *both* real pads and holes (unusual, but not
    // impossible for a hand-built KiCad footprint) keeps `false`: it
    // still has real, presumably purchasable pads.
    let exclude_from_bom = !holes.is_empty() && pads.is_empty();

    FootprintTemplate {
        name: template_name_for(&fp.reference, index, board_label),
        reference_prefix: reference_prefix_of(&fp.reference),
        pads,
        holes,
        exclude_from_bom,
        // A real .kicad_pcb import has no reason to reach for the
        // pads/holes-derived fallback here -- `alladin_geom`'s own
        // KiCad footprint import already has the *actual* `F.CrtYd`
        // outline `import_kicad_pcb` could grab and this constructor
        // could pass on, but that's a real follow-up in its own
        // right, not this slice's scope; `None` still gets a correct
        // (if plain bounding-box) courtyard from `FootprintTemplate::courtyard`.
        explicit_courtyard: None,
    }
}

/// Whether any item on `node`, or any pad of `footprints`, lives on
/// `B.Cu` -- the only signal a bare `.kicad_pcb` file gives for guessing
/// [`LayerCount`] (which otherwise has no on-disk representation of its
/// own, see that type's doc comment: it isn't wired to anything beyond
/// the outline/label yet). A single-sided file imports as
/// [`LayerCount::One`]; anything using the back copper layer imports as
/// [`LayerCount::Two`] -- `>2`-layer boards aren't representable by this
/// editor at all yet (same documented MVP scope cut as
/// [`crate::board_doc::NewBoardParams::layer_count`]), so there is no
/// third case to guess here.
fn guess_layer_count(node: &Node, footprints: &[ImportedFootprint]) -> LayerCount {
    let node_uses_back = node.iter().any(|item| matches!(item.layers(), (_, Some(_)) | (LayerId::BCu, _)));
    let pads_use_back = footprints.iter().flat_map(|fp| &fp.pads).any(|pad| pad.layer == LayerId::BCu);
    if node_uses_back || pads_use_back {
        LayerCount::Two
    } else {
        LayerCount::One
    }
}

/// Everything [`import_kicad_pcb`] hands back: the reconstructed board,
/// plus the fresh, file-specific templates its footprints reference by
/// name (`crate::app` is responsible for appending these to whatever
/// template list -- built-ins plus the parts database -- it already has
/// live, exactly like it already does for `crate::lcsc` downloads).
pub struct ImportedDoc {
    pub doc: BoardDoc,
    /// Read only by this module's and `kicad_export.rs`'s round-trip
    /// tests since the public KiCad import feature was removed; the
    /// production bridge caller (`crate::external_router`) merges
    /// tracks/vias from `doc` only.
    #[cfg_attr(not(test), allow(dead_code))]
    pub templates: Vec<FootprintTemplate>,
}

/// Writes every one of [`ImportedDoc::templates`] into `parts_db`, so a
/// board saved right after import stays loadable in a *later, fresh*
/// process too -- without this, [`persistence::from_json`]'s `templates`
/// lookup (built-ins plus whatever `crate::app::load_templates` reads
/// back out of `parts_db`) would never contain these file-specific
/// `"Imported: <reference>"` names again once the importing process
/// exits, and reopening the saved file would fail with
/// [`crate::persistence::LoadError::UnknownTemplate`] even though the
/// file itself is perfectly intact. Call after
/// [`import_kicad_pcb_with_label`] when the imported templates must
/// survive into a later process via the parts database.
///
/// `category` is normally `"Imported files/{board_label}"` (matching
/// the same `board_label` given to [`import_kicad_pcb_with_label`]) --
/// give `""` only for the same unlabeled/no-category case
/// [`import_kicad_pcb`] itself uses. When `category` is non-empty, this
/// first deletes every part already under that exact category (see
/// [`crate::parts_db::PartsDb::delete_category_tree`]) before inserting
/// the fresh ones, so **re-importing the same board file replaces its
/// own previously-persisted parts** instead of piling up an ever-growing
/// set of duplicate rows every time a design gets re-imported after a
/// change -- exactly the "aufräumen" this whole category feature is
/// for, applied automatically rather than left for manual cleanup.
///
/// Every row is inserted with no LCSC code and an empty description
/// (this is a real board's own footprint, not a catalog part), so the
/// only way this can fail is a genuine database I/O problem -- treated
/// as fatal (aborts the whole import) rather than silently degrading
/// back to the ephemeral, unreloadable behaviour this function exists to
/// prevent. Returns each template paired with its new `parts` row id,
/// exactly the `(FootprintTemplate, i64)` shape `crate::app::EditorState`'s
/// `templates`/`template_origin` lists already keep side-by-side for
/// every database-backed template (downloaded, hand-registered, or now
/// imported).
///
/// No production caller since the public KiCad import feature was
/// removed -- kept because the save/reload round-trip tests below still
/// prove the bridge parser's template output correct end-to-end.
#[cfg_attr(not(test), allow(dead_code))]
pub fn persist_templates(parts_db: &PartsDb, category: &str, templates: Vec<FootprintTemplate>) -> Result<Vec<(FootprintTemplate, i64)>, String> {
    if !category.is_empty() {
        parts_db.delete_category_tree(category).map_err(|e| format!("couldn't clear out this board's previous import under \"{category}\": {e}"))?;
    }
    let category = (!category.is_empty()).then_some(category);
    templates
        .into_iter()
        .map(|t| {
            parts_db
                .insert_part_categorized(&t.name, &t.reference_prefix, "", None, &t.pads, &t.holes, t.exclude_from_bom, t.explicit_courtyard, category)
                .map(|record| (record.template, record.id))
                .map_err(|e| format!("couldn't save imported part \"{}\" to the parts database: {e}", t.name))
        })
        .collect()
}

/// Parses `source` (an already-read `.kicad_pcb` file) and rebuilds a
/// fully editable [`BoardDoc`], with every generated template plainly
/// named (`"Imported: {reference}"`, no board label) -- what every
/// internal, single-file-at-a-time caller (this module's own unit
/// tests, `kicad_export.rs`'s round-trip tests, `external_router.rs`'s
/// post-autoroute re-import, none of which risk a cross-board name
/// collision) has always used and keeps using unchanged. A real,
/// callers that persist templates across boards should prefer
/// [`import_kicad_pcb_with_label`] instead -- see that function's own
/// doc comment for why.
pub fn import_kicad_pcb(source: &str) -> Result<ImportedDoc, ImportError> {
    import_kicad_pcb_with_label(source, "")
}

/// Same as [`import_kicad_pcb`], except every generated template's name
/// (and optionally its `parts_db` category via [`persist_templates`])
/// embeds `board_label` -- see [`template_name_for`]'s own doc comment
/// for exactly how, and this module's doc comment for the real
/// cross-board name-collision this exists to prevent. Give `""` for
/// `board_label` to fall back to exactly [`import_kicad_pcb`]'s own
/// unlabeled behaviour.
pub fn import_kicad_pcb_with_label(source: &str, board_label: &str) -> Result<ImportedDoc, ImportError> {
    let flat = alladin_kicad_io::import_kicad_pcb(source)?;
    let footprints = alladin_kicad_io::import_footprints(source)?;

    let mut node = Node::new();
    for item in flat.node.iter() {
        // Both are footprint-owned and get rebuilt below from
        // `footprints`' own real per-footprint structure (`build_template`
        // + `insert_footprint_unchecked`) -- keeping either of the flat
        // import's own copies here would double them up on the final
        // board.
        if !matches!(item, Item::Pad { .. } | Item::Hole { .. }) {
            node.add(item.clone());
        }
    }

    let mut doc = BoardDoc {
        outline: flat.outline,
        layer_count: guess_layer_count(&node, &footprints),
        // A `.kicad_pcb` file has no copper-weight field to read at all
        // (it's a fab-order-time choice, not a design-file property) --
        // `OneOz` (JLCPCB's own no-extra-cost default) is the same
        // reasonable fallback `NewBoardParams::default`'s own "New
        // board" dialog already uses.
        copper_weight: crate::board_doc::CopperWeight::OneOz,
        node,
        footprints: Vec::new(),
        next_footprint_serial: 0,
        // Net id 0 is KiCad's fixed "no net" sentinel (always declared,
        // even by this crate's own writer -- see `write_kicad_pcb`'s
        // unconditional `(net 0 "")`), never a real user net -- same
        // exclusion `alladin_kicad_io::net_of` already applies when
        // reading a pad's own `(net ...)` form.
        nets: flat
            .nets
            .iter()
            .filter(|&(&id, _)| id != 0)
            .map(|(&id, name)| crate::board_doc::NetRecord { id: alladin_core::NetId(id), name: name.clone() })
            .collect(),
        next_net_serial: flat.nets.keys().copied().max().unwrap_or(0),
        // A `.kicad_pcb` file's zones import as already-filled, static
        // `Item::Zone`s (see `flat.node`'s own import, unaffected by
        // this loop's `Item::Pad` filter) with no `ZoneRecord` behind
        // them -- there's no user-drawn source outline to refill from,
        // matching `alladin_kicad_io::import_zone`'s own "static,
        // Alladin never reshapes it" contract.
        zones: Vec::new(),
        next_zone_serial: 0,
        // Restored directly, with no `BoardDoc::try_place_silk_text`
        // collision re-check -- same "a file real KiCad already
        // accepted is by definition already geometrically legal"
        // reasoning this module's own doc comment gives for every
        // imported footprint bypassing `check_placement` (Alladin's
        // own silk-to-pad collision model could otherwise reject real,
        // valid geometry it wasn't conservative enough to model
        // exactly).
        silk_texts: flat
            .silk_texts
            .into_iter()
            .enumerate()
            .map(|(index, t)| crate::board_doc::SilkText {
                id: crate::board_doc::SilkTextId(index),
                text: t.text,
                position: t.position,
                rotation_deg: t.rotation_deg,
                layer: t.layer,
                height: t.height,
                line_width: t.line_width,
            })
            .collect(),
        next_silk_text_serial: 0,
        // A `.kicad_pcb` file has no Alladin-style dot annotations to
        // restore (a `gr_circle` in someone else's file could be
        // anything); imports start dot-free.
        silk_dots: Vec::new(),
        next_silk_dot_serial: 0,
    };
    doc.next_silk_text_serial = doc.silk_texts.len();
    doc.nets.sort_by_key(|n| n.id.0);

    let mut templates = Vec::with_capacity(footprints.len());
    for (index, fp) in footprints.iter().enumerate() {
        let template = build_template(fp, index, board_label);
        let pad_nets: Vec<Option<alladin_core::NetId>> = fp.pads.iter().map(|pad| pad.net.as_ref().map(|(id, _)| alladin_core::NetId(*id))).collect();
        doc.insert_footprint_unchecked(&template, fp.reference.clone(), fp.position, fp.rotation_deg, &pad_nets);
        templates.push(template);
    }

    Ok(ImportedDoc { doc, templates })
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::NetId;
    use alladin_geom::{Point, Polygon, MM};
    use alladin_kicad_io::{PadMount, WriteFootprint, WritePad};

    fn mm(v: f64) -> alladin_geom::Unit {
        (v * MM as f64).round() as alladin_geom::Unit
    }

    fn sample_source() -> String {
        let outline = vec![Polygon::rounded_rect(mm(20.0), mm(15.0), 0, 4)];
        let header = WriteFootprint {
            reference: "J1".to_string(),
            value: "Header".to_string(),
            position: Point::new(mm(-5.0), 0),
            rotation_deg: 0.0,
            pads: vec![
                WritePad {
                    number: "1".to_string(),
                    offset: Point::new(-mm(1.27), 0),
                    shape: WritePadShape::Circle { diameter: mm(1.5) },
                    rotation_deg: 0.0,
                    mount: PadMount::ThruHole,
                    drill: Some(mm(0.8)),
                    layer: LayerId::FCu,
                    net: Some((1, "GND".to_string())),
                },
                WritePad {
                    number: "2".to_string(),
                    offset: Point::new(mm(1.27), 0),
                    shape: WritePadShape::Circle { diameter: mm(1.5) },
                    rotation_deg: 0.0,
                    mount: PadMount::ThruHole,
                    drill: Some(mm(0.8)),
                    layer: LayerId::FCu,
                    net: None,
                },
            ],
        };
        let smd = WriteFootprint {
            reference: "U1".to_string(),
            value: "ESP32".to_string(),
            position: Point::new(mm(5.0), 0),
            rotation_deg: 90.0,
            pads: vec![WritePad {
                number: "1".to_string(),
                offset: Point::new(mm(2.0), 0),
                shape: WritePadShape::Rect { width: mm(1.0), height: mm(0.3) },
                rotation_deg: 0.0,
                mount: PadMount::Smd,
                drill: None,
                layer: LayerId::BCu,
                net: Some((1, "GND".to_string())),
            }],
        };
        alladin_kicad_io::write_kicad_pcb(&outline, &[header, smd], &Node::new(), &[(1, "GND".to_string())], &[], &[], &[])
    }

    #[test]
    fn imports_every_footprint_with_its_own_reference_backed_template() {
        let imported = import_kicad_pcb(&sample_source()).unwrap();
        assert_eq!(imported.doc.footprints.len(), 2);
        assert_eq!(imported.templates.len(), 2);
        assert_eq!(imported.doc.footprints[0].reference, "J1");
        assert_eq!(imported.doc.footprints[1].reference, "U1");
        assert_eq!(imported.doc.footprints[0].template_name, imported.templates[0].name);
    }

    #[test]
    fn placement_is_unchecked_so_geometry_from_a_trusted_file_is_never_rejected() {
        // The board outline here is only 20x15mm and the two footprints
        // sit 10mm apart -- plausible, but this test's real point is
        // that *no* `PlacementError` path exists at all for import, so
        // there is nothing to assert other than "it imported both".
        let imported = import_kicad_pcb(&sample_source()).unwrap();
        assert_eq!(imported.doc.node.iter().filter(|i| matches!(i, Item::Pad { .. })).count(), 3);
    }

    #[test]
    fn recovers_real_pad_shape_number_and_drill() {
        let imported = import_kicad_pcb(&sample_source()).unwrap();
        let header = &imported.templates[0];
        assert_eq!(header.pads[0].number, "1");
        assert_eq!(header.pads[0].hole_diameter, Some(mm(0.8)));
        let smd = &imported.templates[1];
        assert_eq!(smd.pads[0].shape, PadShapeKind::Rect { width: mm(1.0), height: mm(0.3) });
        assert_eq!(smd.pads[0].hole_diameter, None);
    }

    #[test]
    fn persist_templates_writes_every_template_to_the_parts_db_with_matching_geometry() {
        let imported = import_kicad_pcb(&sample_source()).unwrap();
        let parts_db = crate::parts_db::PartsDb::open_in_memory().unwrap();
        let persisted = persist_templates(&parts_db, "", imported.templates.clone()).unwrap();
        assert_eq!(persisted.len(), 2);
        for (template, id) in &persisted {
            assert!(*id > 0);
            assert_eq!(template.pads.len(), imported.templates.iter().find(|t| t.name == template.name).unwrap().pads.len());
        }
        assert_eq!(parts_db.list_parts().unwrap().len(), 2, "both templates must actually have landed in the database, not just been echoed back");
    }

    #[test]
    fn a_board_saved_right_after_import_reloads_in_a_fresh_process_once_its_templates_are_persisted() {
        // Reproduces the exact "unknown footprint template" reload
        // failure a real KiCad import used to leave behind (see this
        // module's own `persist_templates` doc comment): import, persist,
        // save -- then reload using *only* what a brand new process
        // would have (the parts database, never the in-memory
        // `imported.templates` list this same process happened to build).
        let imported = import_kicad_pcb(&sample_source()).unwrap();
        let saved_json = crate::persistence::to_json(&imported.doc);

        let parts_db = crate::parts_db::PartsDb::open_in_memory().unwrap();
        persist_templates(&parts_db, "", imported.templates).unwrap();

        let fresh_process_templates: Vec<FootprintTemplate> = parts_db.list_parts().unwrap().into_iter().map(|record| record.template).collect();
        let reloaded = crate::persistence::from_json(&saved_json, &fresh_process_templates).expect("a board saved right after import must still load once its templates are persisted");
        assert_eq!(reloaded.footprints.len(), 2);
    }

    /// A second, independent board whose only footprint happens to
    /// share its reference designator (`"J1"`) with `sample_source`'s
    /// own header -- but with a visibly different pad layout (three
    /// pads at a different pitch, not two), so a test can tell whether
    /// the two ever got confused with each other after both are
    /// imported into the same `parts_db`.
    fn other_board_source_with_a_colliding_reference() -> String {
        let outline = vec![Polygon::rounded_rect(mm(20.0), mm(15.0), 0, 4)];
        let header = WriteFootprint {
            reference: "J1".to_string(),
            value: "Header3".to_string(),
            position: Point::new(0, 0),
            rotation_deg: 0.0,
            pads: (0..3)
                .map(|i| WritePad {
                    number: (i + 1).to_string(),
                    offset: Point::new(mm(2.0) * i as i64, 0),
                    shape: WritePadShape::Circle { diameter: mm(1.0) },
                    rotation_deg: 0.0,
                    mount: PadMount::ThruHole,
                    drill: Some(mm(0.6)),
                    layer: LayerId::FCu,
                    net: None,
                })
                .collect(),
        };
        alladin_kicad_io::write_kicad_pcb(&outline, &[header], &Node::new(), &[], &[], &[], &[])
    }

    #[test]
    fn importing_two_boards_with_the_same_reference_never_collide_when_labeled() {
        // The real bug this module's own doc comment describes: two
        // different boards' `J1` must land as two distinct `parts_db`
        // rows (different names, different categories), each keeping
        // its own real pad count -- not one silently shadowing the
        // other.
        let parts_db = crate::parts_db::PartsDb::open_in_memory().unwrap();

        let board_a = import_kicad_pcb_with_label(&sample_source(), "board_a").unwrap();
        persist_templates(&parts_db, "Imported files/board_a", board_a.templates).unwrap();

        let board_b = import_kicad_pcb_with_label(&other_board_source_with_a_colliding_reference(), "board_b").unwrap();
        persist_templates(&parts_db, "Imported files/board_b", board_b.templates).unwrap();

        let parts = parts_db.list_parts().unwrap();
        let a_j1 = parts.iter().find(|p| p.template.name == "Imported [board_a]: J1").expect("board_a's own labeled J1 must exist");
        let b_j1 = parts.iter().find(|p| p.template.name == "Imported [board_b]: J1").expect("board_b's own labeled J1 must exist");
        assert_ne!(a_j1.id, b_j1.id, "the two boards' J1 must be two separate rows");
        assert_eq!(a_j1.template.pads.len(), 2, "board_a's real 2-pad header must keep its own real geometry");
        assert_eq!(b_j1.template.pads.len(), 3, "board_b's real 3-pad header must keep its own real geometry, not board_a's");
        assert_eq!(a_j1.category, Some("Imported files/board_a".to_string()));
        assert_eq!(b_j1.category, Some("Imported files/board_b".to_string()));
    }

    #[test]
    fn reimporting_the_same_board_replaces_its_previous_parts_instead_of_duplicating_them() {
        let parts_db = crate::parts_db::PartsDb::open_in_memory().unwrap();

        let first = import_kicad_pcb_with_label(&sample_source(), "my_board").unwrap();
        persist_templates(&parts_db, "Imported files/my_board", first.templates).unwrap();
        assert_eq!(parts_db.list_parts().unwrap().len(), 2, "the first import's own two footprints");

        let second = import_kicad_pcb_with_label(&sample_source(), "my_board").unwrap();
        persist_templates(&parts_db, "Imported files/my_board", second.templates).unwrap();

        assert_eq!(parts_db.list_parts().unwrap().len(), 2, "re-importing the identical board must replace, not accumulate duplicate rows");
    }

    #[test]
    fn unlabeled_import_still_produces_the_original_plain_names() {
        // `import_kicad_pcb` itself (board_label == "") must be
        // completely unaffected by this feature -- every internal
        // caller (this module's own other tests, `kicad_export.rs`,
        // `external_router.rs`) relies on exactly this plain name.
        let imported = import_kicad_pcb(&sample_source()).unwrap();
        assert_eq!(imported.templates[0].name, "Imported: J1");
        assert_eq!(imported.templates[1].name, "Imported: U1");
    }

    #[test]
    fn recovers_pad_nets_from_the_declared_net_table() {
        let imported = import_kicad_pcb(&sample_source()).unwrap();
        let ground_pad = imported.doc.footprints[0].pad_item_ids[0];
        assert_eq!(imported.doc.node.get(ground_pad).unwrap().net(), Some(NetId(1)));
        let unconnected_pad = imported.doc.footprints[0].pad_item_ids[1];
        assert_eq!(imported.doc.node.get(unconnected_pad).unwrap().net(), None);
        assert_eq!(imported.doc.nets.len(), 1);
        assert_eq!(imported.doc.nets[0].name, "GND");
    }

    #[test]
    fn guesses_two_layers_when_any_pad_uses_the_back_copper() {
        let imported = import_kicad_pcb(&sample_source()).unwrap();
        assert_eq!(imported.doc.layer_count, LayerCount::Two);
    }

    #[test]
    fn a_single_sided_board_guesses_one_layer() {
        let outline = vec![Polygon::rounded_rect(mm(10.0), mm(10.0), 0, 4)];
        let fp = WriteFootprint {
            reference: "R1".to_string(),
            value: "10k".to_string(),
            position: Point::new(0, 0),
            rotation_deg: 0.0,
            pads: vec![WritePad {
                number: "1".to_string(),
                offset: Point::new(0, 0),
                shape: WritePadShape::Circle { diameter: mm(1.0) },
                rotation_deg: 0.0,
                mount: PadMount::Smd,
                drill: None,
                layer: LayerId::FCu,
                net: None,
            }],
        };
        let text = alladin_kicad_io::write_kicad_pcb(&outline, &[fp], &Node::new(), &[], &[], &[], &[]);
        let imported = import_kicad_pcb(&text).unwrap();
        assert_eq!(imported.doc.layer_count, LayerCount::One);
    }

    #[test]
    fn preserves_the_board_outline() {
        let imported = import_kicad_pcb(&sample_source()).unwrap();
        assert_eq!(imported.doc.outline.len(), 1);
    }

    #[test]
    fn rejects_a_source_that_is_not_a_kicad_pcb_form() {
        assert!(matches!(import_kicad_pcb("(not_a_pcb)"), Err(ImportError::NotAKicadPcb)));
    }

    /// Internal bridge round trip: `.kicad_pcb` -> [`import_kicad_pcb`]
    /// -> `crate::kicad_export::export_kicad_pcb` -> still a valid file
    /// with the same footprints/pads/nets. Proves the autorouter's
    /// interchange path composes; every other test here only exercises
    /// `import_kicad_pcb` in isolation.
    #[test]
    fn reimported_board_re_exports_to_an_equally_valid_kicad_pcb_file() {
        let original = sample_source();
        let imported = import_kicad_pcb(&original).unwrap();

        let re_exported = crate::kicad_export::export_kicad_pcb(&imported.doc, &imported.templates);
        let parsed = alladin_sexpr::parse(&re_exported).expect("re-export must still be valid S-expression syntax");
        assert!(parsed.tagged("kicad_pcb").is_some());
        assert_eq!(parsed.children("footprint").count(), 2, "both imported footprints must survive a re-export");

        let re_imported = import_kicad_pcb(&re_exported).unwrap();
        assert_eq!(re_imported.doc.footprints.len(), 2);
        assert_eq!(re_imported.doc.footprints[0].reference, "J1");
        assert_eq!(re_imported.doc.footprints[1].reference, "U1");
        assert_eq!(re_imported.doc.footprints[1].rotation_deg, 90.0, "footprint rotation must survive import -> export -> import again");
        assert_eq!(re_imported.templates[1].pads[0].shape, PadShapeKind::Rect { width: mm(1.0), height: mm(0.3) });
        assert_eq!(re_imported.doc.nets.len(), 1);
        assert_eq!(re_imported.doc.nets[0].name, "GND");
    }

    /// Same round trip as
    /// Silk text is baked to `(gr_line ...)` strokes on KiCad export
    /// (same Hershey geometry as the preview), so a re-import does
    /// *not* recover editable [`crate::board_doc::SilkText`] -- the
    /// ink survives as geometry in the `.kicad_pcb`. Editable text
    /// lives in Alladin's own `.json`; KiCad export is for inspection
    /// / autorouter / matching view.
    #[test]
    fn a_placed_silk_text_exports_as_gr_line_strokes_not_gr_text() {
        let mut doc = crate::board_doc::NewBoardParams::default().create();
        doc.try_place_silk_text("REV A", Point::new(mm(5.0), mm(-3.0)), 90.0, LayerId::BCu, crate::board_doc::DEFAULT_SILK_TEXT_HEIGHT)
            .expect("center of an empty 50x30mm board must be a legal silk placement");

        let exported = crate::kicad_export::export_kicad_pcb(&doc, &[]);
        assert!(!exported.contains("gr_text"), "must bake text to strokes, not emit gr_text");
        assert!(exported.contains(r#"(layer "B.SilkS")"#), "strokes must land on back silk");
        assert!(exported.matches("gr_line").count() > 5, "a short word must produce several stroke segments");

        let imported = import_kicad_pcb(&exported).unwrap();
        assert!(imported.doc.silk_texts.is_empty(), "baked strokes are not re-imported as editable SilkText");
    }

}
