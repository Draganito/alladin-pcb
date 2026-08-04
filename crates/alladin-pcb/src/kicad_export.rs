//! Internal `.kicad_pcb` writer -- **not** a user-facing product feature.
//! Alladin's editable format is its own `.json`; manufacturing uses the
//! native Gerber path. This module exists so
//! [`crate::external_router`] can hand KiCadRoutingTools a board file
//! (and so silk can be baked to stroke geometry for that interchange).
//! Do not re-expose Import/Export-to-KiCad in the GUI/MCP/CLI.
//!
//! This module's only job is the *adaptation*: [`BoardDoc`] (Alladin's
//! own editing-time model, where a placed part only remembers its
//! template's *name* -- see [`crate::board_doc::PlacedFootprint`]'s doc
//! comment) plus the live `templates` list (needed to resolve that name
//! back to real pad geometry, the exact same lookup `crate::app`'s own
//! rendering code already does) into `alladin_kicad_io::WriteFootprint`,
//! which carries full pad shape/number/rotation/drill fidelity. All the
//! actual `.kicad_pcb` syntax knowledge lives in `alladin-kicad-io`
//! itself -- see that crate's `writer` module doc comment for how its
//! output was ground-truth verified against real KiCad 9.

use alladin_core::{Item, LayerId};
use alladin_kicad_io::{write_kicad_pcb, PadMount, WriteFootprint, WritePad, WritePadShape, WriteSilkDot, WriteSilkLine, WriteZone};

use crate::board_doc::BoardDoc;
use crate::footprint::{FootprintTemplate, PadShapeKind};

fn write_pad_shape(kind: PadShapeKind, radius: alladin_geom::Unit) -> WritePadShape {
    match kind {
        PadShapeKind::Circle => WritePadShape::Circle { diameter: radius * 2 },
        PadShapeKind::Rect { width, height } => WritePadShape::Rect { width, height },
        PadShapeKind::Oval { width, height } => WritePadShape::Oval { width, height },
    }
}

/// Resolves every placed footprint's pads against `templates`, carrying
/// each pad's *actual* on-board net (read from `doc.node`, which is the
/// only place a specific placed pad's net assignment lives -- templates
/// themselves are net-less blueprints) into a real name via `doc.nets`.
///
/// A placed footprint whose `template_name` no longer matches anything
/// in `templates` is silently skipped rather than panicking: this can
/// only happen if a template was deleted from the parts database after
/// being placed on a board still open in the same session, an existing,
/// already-accepted edge case (see `crate::persistence`'s own "resolve
/// against a template list" handling) -- an export missing one
/// unresolvable part is far better than a crashed export.
pub fn build_write_footprints(doc: &BoardDoc, templates: &[FootprintTemplate]) -> Vec<WriteFootprint> {
    doc.footprints
        .iter()
        .filter_map(|placed| {
            let template = templates.iter().find(|t| t.name == placed.template_name)?;
            let mut pads: Vec<WritePad> = placed
                .pad_item_ids
                .iter()
                .zip(&template.pads)
                .map(|(&item_id, pad_template)| {
                    let net = match doc.node.get(item_id) {
                        Some(Item::Pad { net: Some(net_id), .. }) => {
                            let name = doc
                                .nets
                                .iter()
                                .find(|n| n.id == *net_id)
                                .map(|n| n.name.clone())
                                .unwrap_or_else(|| format!("Net{}", net_id.0));
                            Some((net_id.0, name))
                        }
                        _ => None,
                    };
                    WritePad {
                        number: pad_template.number.clone(),
                        offset: pad_template.offset,
                        shape: write_pad_shape(pad_template.shape, pad_template.radius),
                        rotation_deg: pad_template.rotation_deg,
                        mount: if pad_template.hole_diameter.is_some() { PadMount::ThruHole } else { PadMount::Smd },
                        drill: pad_template.hole_diameter,
                        layer: pad_template.layer,
                        net,
                    }
                })
                .collect();
            // A footprint's own mechanical holes (`crate::footprint::HoleTemplate`,
            // see `PadMount::NpThruHole`'s own doc comment for why this
            // reuses `WritePad`/`WriteFootprint::pads` rather than a
            // separate hole list) -- written after the real pads, same
            // order `crate::footprint::world_items` already produces
            // them in.
            pads.extend(template.holes.iter().map(|hole_template| WritePad {
                number: String::new(),
                offset: hole_template.offset,
                shape: WritePadShape::Circle { diameter: hole_template.drill },
                rotation_deg: 0.0,
                mount: PadMount::NpThruHole,
                drill: Some(hole_template.drill),
                layer: LayerId::FCu,
                net: None,
            }));
            Some(WriteFootprint {
                reference: placed.reference.clone(),
                value: template.name.clone(),
                position: placed.position,
                rotation_deg: placed.rotation_deg,
                pads,
            })
        })
        .collect()
}

/// Turns every recorded `crate::board_doc::ZoneRecord` into a
/// `WriteZone`, resolving its net id to a human name via `doc.nets`
/// (falling back to `NetN`, same convention `build_write_footprints`
/// already uses for a pad net with no recorded name) and its
/// `item_ids` back to each fill island's actual polygon via `doc.node`
/// -- `ZoneRecord` itself only remembers *which* node items are its
/// islands, not their geometry (see that field's own doc comment).
pub fn build_write_zones(doc: &BoardDoc) -> Vec<WriteZone> {
    doc.zones
        .iter()
        .map(|zone| {
            let name = doc.nets.iter().find(|n| n.id == zone.net).map(|n| n.name.clone()).unwrap_or_else(|| format!("Net{}", zone.net.0));
            let islands = zone
                .item_ids
                .iter()
                .filter_map(|&id| match doc.node.get(id) {
                    Some(Item::Zone { outline, .. }) => Some(outline.clone()),
                    _ => None,
                })
                .collect();
            WriteZone { outline: zone.outline.clone(), layer: zone.layer, net: Some((zone.net.0, name)), islands }
        })
        .collect()
}

/// Every free-standing [`crate::board_doc::SilkText`] baked into
/// stroke segments ([`WriteSilkLine`]) via the same Hershey layout the
/// GUI and native Gerber already use. Exporting geometry -- not
/// `(gr_text ...)` -- keeps KiCad's view identical to Alladin and
/// avoids embedding KiCad's GPL Newstroke font data.
pub fn build_write_silk_lines(doc: &BoardDoc) -> Vec<WriteSilkLine> {
    doc.silk_texts
        .iter()
        .flat_map(|t| {
            t.stroke_segments().into_iter().map(|seg| WriteSilkLine {
                start: seg.a,
                end: seg.b,
                width: seg.width,
                layer: t.layer,
            })
        })
        .collect()
}

/// Every printed silk dot: the free-standing
/// [`crate::board_doc::SilkDot`]s plus each footprint's enabled pin-1
/// marker (already resolved to its world-space circle by
/// [`crate::board_doc::PlacedFootprint::pin1_marker_circle`]) -- by
/// export time the distinction is irrelevant, both are just deliberate
/// round ink, so they share one `WriteSilkDot` list. These, together
/// with [`build_write_silk_lines`]'s output, are the *only* silk that
/// prints at all -- reference designators stay editor-only (see
/// `alladin_kicad_io::writer`'s hidden Reference/Value properties).
pub fn build_write_silk_dots(doc: &BoardDoc) -> Vec<WriteSilkDot> {
    let mut dots: Vec<WriteSilkDot> =
        doc.silk_dots.iter().map(|d| WriteSilkDot { center: d.position, diameter: d.diameter, layer: d.layer }).collect();
    dots.extend(doc.footprints.iter().filter_map(|fp| {
        fp.pin1_marker_circle().map(|c| WriteSilkDot { center: c.center, diameter: c.radius * 2, layer: alladin_core::LayerId::FCu })
    }));
    dots
}

/// Renders `doc` to a complete `.kicad_pcb` file's text -- see this
/// module's doc comment.
pub fn export_kicad_pcb(doc: &BoardDoc, templates: &[FootprintTemplate]) -> String {
    let footprints = build_write_footprints(doc, templates);
    let nets: Vec<(u32, String)> = doc.nets.iter().map(|n| (n.id.0, n.name.clone())).collect();
    let zones = build_write_zones(doc);
    let silk_lines = build_write_silk_lines(doc);
    let silk_dots = build_write_silk_dots(doc);
    write_kicad_pcb(&doc.outline, &footprints, &doc.node, &nets, &zones, &silk_lines, &silk_dots)
}

/// Renders the `.kicad_pro` companion `export_kicad_pcb`'s output
/// always wants sitting next to it -- see
/// `alladin_kicad_io::write_kicad_pro`'s own doc comment for the full
/// "48 false DRC violations without this file" ground-truth story.
/// `clearance` is the exporting board's own [`BoardDoc::net_class_clearance`]
/// -- copper-weight-sensitive (0.10mm for 1oz, 0.16mm for 2oz) -- so a
/// board's exported project always honestly declares the DFM rules that
/// board itself actually enforces, never a hardcoded stand-in for one
/// specific profile. Everything else here is board-independent (trace/
/// via defaults), so this still doesn't need the rest of `doc`.
pub fn export_kicad_pro(project_filename: &str, clearance: alladin_geom::Unit) -> String {
    alladin_kicad_io::write_kicad_pro(
        project_filename,
        clearance,
        crate::routing::DEFAULT_TRACE_WIDTH,
        crate::board_doc::DEFAULT_VIA_DIAMETER,
        crate::board_doc::DEFAULT_VIA_DRILL,
    )
}

/// Writes `doc` as both a `.kicad_pcb` at `pcb_path` and its
/// `.kicad_pro` sibling (same stem, swapped extension) right next to
/// it. Internal entry point for [`crate::external_router`] (and tests);
/// not exposed as a product feature.
pub fn export_kicad_files(doc: &BoardDoc, templates: &[FootprintTemplate], pcb_path: &std::path::Path) -> std::io::Result<()> {
    std::fs::write(pcb_path, export_kicad_pcb(doc, templates))?;
    let pro_path = pcb_path.with_extension("kicad_pro");
    let project_filename = pro_path.file_name().and_then(|n| n.to_str()).unwrap_or("board.kicad_pro").to_string();
    std::fs::write(&pro_path, export_kicad_pro(&project_filename, doc.net_class_clearance()))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board_doc::{CopperWeight, LayerCount, NewBoardParams};
    use alladin_geom::{Point, Polygon};

    fn test_board() -> BoardDoc {
        NewBoardParams { width_mm: 40.0, height_mm: 40.0, layer_count: LayerCount::Two, copper_weight: CopperWeight::OneOz, corner_radius_mm: 0.0 }.create()
    }

    #[test]
    fn exports_a_placed_footprint_with_its_real_pad_geometry_and_a_connected_net() {
        let mut board = test_board();
        let templates = crate::footprint::builtin_templates();
        let template = &templates[0];
        let a = board.try_place_footprint(template, Point::new(-10_000_000, 0), 0.0).unwrap();
        let b = board.try_place_footprint(template, Point::new(10_000_000, 0), 0.0).unwrap();
        let pad_a = board.footprints.iter().find(|f| f.id == a).unwrap().pad_item_ids[0];
        let pad_b = board.footprints.iter().find(|f| f.id == b).unwrap().pad_item_ids[0];
        board.connect_pads(pad_a, pad_b).unwrap();

        let text = export_kicad_pcb(&board, &templates);
        let parsed = alladin_sexpr::parse(&text).expect("must be valid S-expression syntax");
        assert!(parsed.tagged("kicad_pcb").is_some());
        assert_eq!(parsed.children("footprint").count(), 2);
        assert!(text.contains("Net1"), "the connected net must actually be declared and referenced");
    }

    #[test]
    fn exports_a_manually_placed_via_on_its_net() {
        // `write_kicad_pcb` already writes every `Item::Via` in `doc.node`
        // generically (it never needed editor-specific changes for this,
        // see the development log's via/layer-switch slice) -- this
        // just confirms that still holds now that `BoardDoc::try_add_via`
        // is a real way to get one onto the board, not just an importer.
        let mut board = test_board();
        let templates = crate::footprint::builtin_templates();
        let template = &templates[0];
        let a = board.try_place_footprint(template, Point::new(-10_000_000, 0), 0.0).unwrap();
        let b = board.try_place_footprint(template, Point::new(10_000_000, 0), 0.0).unwrap();
        let pad_a = board.footprints.iter().find(|f| f.id == a).unwrap().pad_item_ids[0];
        let pad_b = board.footprints.iter().find(|f| f.id == b).unwrap().pad_item_ids[0];
        let net = board.connect_pads(pad_a, pad_b).unwrap();
        board.try_add_via(Point::new(0, 5_000_000), net, 600_000, 300_000).expect("open space must accept a via");

        let text = export_kicad_pcb(&board, &templates);
        let parsed = alladin_sexpr::parse(&text).expect("must be valid S-expression syntax");
        assert_eq!(parsed.children("via").count(), 1, "the manually placed via must be exported");
        assert!(text.contains("Net1"), "the via's net must be declared and referenced, not just the pads'");
    }

    #[test]
    fn a_drawn_zone_round_trips_through_kicad_export_and_reimport() {
        use alladin_core::LayerId;

        let mut board = test_board(); // 40mm x 40mm outline
        let net = board.create_net();
        let outline = Polygon::new(vec![
            Point::new(-15_000_000, -15_000_000),
            Point::new(15_000_000, -15_000_000),
            Point::new(15_000_000, 15_000_000),
            Point::new(-15_000_000, 15_000_000),
        ]);
        board.add_zone(outline, LayerId::FCu, net);
        let expected_island_count = board.zones[0].item_ids.len();
        assert!(expected_island_count > 0, "a zone drawn over open board space must fill to at least one island");

        let text = export_kicad_pcb(&board, &[]);
        let parsed = alladin_sexpr::parse(&text).expect("must be valid S-expression syntax");
        assert_eq!(parsed.children("zone").count(), 1, "one ZoneRecord must become one (zone ...) form");
        let zone_form = parsed.children("zone").next().unwrap();
        assert_eq!(zone_form.children("filled_polygon").count(), expected_island_count, "one filled_polygon per fill island");

        let imported = crate::kicad_import::import_kicad_pcb(&text).expect("the exported file must re-import cleanly");
        let reimported_island_count =
            imported.doc.node.iter().filter(|item| matches!(item, Item::Zone { layer: LayerId::FCu, net: Some(n), .. } if *n == net)).count();
        assert_eq!(reimported_island_count, expected_island_count, "every filled_polygon island must survive export+reimport as its own Item::Zone");
    }

    #[test]
    fn exports_a_mounting_hole_as_an_np_thru_hole_pad_with_no_net_and_it_reimports_cleanly() {
        let mut board = test_board();
        let templates = crate::footprint::builtin_templates();
        let template = templates.iter().find(|t| t.name.starts_with("Mounting hole (M3")).unwrap();
        board.try_place_footprint(template, Point::new(0, 0), 0.0).unwrap();

        let text = export_kicad_pcb(&board, &templates);
        assert!(text.contains("np_thru_hole"), "a mounting hole must export as a real np_thru_hole pad");
        let parsed = alladin_sexpr::parse(&text).expect("must be valid S-expression syntax");
        assert_eq!(parsed.children("footprint").count(), 1);
        let pad = parsed.children("footprint").next().unwrap().children("pad").next().unwrap();
        assert!(pad.child("net").is_none(), "a mounting hole must never carry a net form");

        let imported = crate::kicad_import::import_kicad_pcb(&text).expect("the exported file must re-import cleanly");
        assert_eq!(imported.doc.footprints.len(), 1);
        assert_eq!(imported.templates[0].holes.len(), 1, "the np_thru_hole pad must round-trip as a real HoleTemplate");
        assert!(imported.templates[0].pads.is_empty(), "it must not also come back as a copper pad");
        assert!(imported.templates[0].exclude_from_bom, "a pure mounting-hole template must stay excluded from the BOM after re-import");
        let hole_count = imported.doc.node.iter().filter(|item| matches!(item, Item::Hole { .. })).count();
        assert_eq!(hole_count, 1, "the reimported board must have exactly one Item::Hole, not zero or a duplicate");
    }

    #[test]
    fn export_kicad_pro_declares_a_default_net_class_at_alladins_own_clearance() {
        // 100_000 nm (`JlcpcbClearance::TRACK_TO_TRACK`) == 0.1 mm -- see
        // `alladin_kicad_io::write_kicad_pro`'s doc comment for why a
        // real `kicad-cli pcb drc` run needs exactly this declared, not
        // KiCad's own built-in 0.2mm fallback.
        let text = export_kicad_pro("board.kicad_pro", alladin_core::JlcpcbClearance::TRACK_TO_TRACK);
        let value: serde_json::Value = serde_json::from_str(&text).expect("must be valid JSON");
        assert_eq!(value["net_settings"]["classes"][0]["clearance"], 0.1);
        assert_eq!(value["net_settings"]["classes"][0]["name"], "Default");
    }

    #[test]
    fn export_kicad_pro_declares_a_2oz_boards_wider_clearance() {
        let text = export_kicad_pro("board.kicad_pro", alladin_core::Jlcpcb2Layer2Oz::TRACK_TO_TRACK);
        let value: serde_json::Value = serde_json::from_str(&text).expect("must be valid JSON");
        assert_eq!(value["net_settings"]["classes"][0]["clearance"], 0.16, "a 2oz board must honestly declare its own wider 0.16mm minimum, not silently understate it as 1oz's 0.1mm");
    }

    #[test]
    fn export_kicad_files_declares_a_2oz_boards_clearance_when_the_board_itself_is_2oz() {
        let board = NewBoardParams { copper_weight: CopperWeight::TwoOz, ..NewBoardParams::default() }.create();
        let dir = std::env::temp_dir().join(format!("alladin_pcb_export_kicad_files_2oz_test_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let pcb_path = dir.join("my_2oz_board.kicad_pcb");

        export_kicad_files(&board, &[], &pcb_path).expect("both files must write successfully");

        let pro_path = dir.join("my_2oz_board.kicad_pro");
        let value: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&pro_path).unwrap()).expect("must be valid JSON");
        assert_eq!(value["net_settings"]["classes"][0]["clearance"], 0.16);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn export_kicad_files_writes_both_the_pcb_and_its_kicad_pro_sibling() {
        let board = test_board();
        let dir = std::env::temp_dir().join(format!("alladin_pcb_export_kicad_files_test_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let pcb_path = dir.join("my_board.kicad_pcb");

        export_kicad_files(&board, &[], &pcb_path).expect("both files must write successfully");

        assert!(pcb_path.exists(), "the .kicad_pcb itself must exist");
        let pro_path = dir.join("my_board.kicad_pro");
        assert!(pro_path.exists(), "the .kicad_pro sibling must exist right next to it, same stem");
        let pro_text = std::fs::read_to_string(&pro_path).unwrap();
        assert!(pro_text.contains("\"Default\""), "the sibling must actually declare the Default net class");

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_footprint_whose_template_was_deleted_is_skipped_not_a_panic() {
        let mut board = test_board();
        let templates = crate::footprint::builtin_templates();
        board.try_place_footprint(&templates[0], Point::new(0, 0), 0.0).unwrap();

        let text = export_kicad_pcb(&board, &[]); // template list no longer has it
        let parsed = alladin_sexpr::parse(&text).unwrap();
        assert_eq!(parsed.children("footprint").count(), 0);
    }
}
