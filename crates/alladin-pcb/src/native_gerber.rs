//! Native manufacturing export: [`BoardDoc`] → Gerber + Excellon + CPL,
//! without `kicad-cli`.
//!
//! Built on `alladin-gerber` (API oriented on Karel Tavernier's
//! `gerber_writer`). Emits JLCPCB-shaped deliverables:
//! `<stem>_gerbers.zip` + `<stem>_cpl.csv`.

use std::io::{Seek, Write};
use std::path::{Path, PathBuf};

use alladin_core::{Item, LayerId, ZoneConnection};
use alladin_geom::{Point, Unit, MM};
use alladin_gerber::{
    set_generation_software, Circle as GerberCircle, DrillKind, ExcellonFile, GerberLayer, Oblong,
    PadMaster, Path as GerberPath, Rectangle,
};

use crate::board_doc::BoardDoc;
use crate::footprint::{pad_world_position, FootprintTemplate, PadShapeKind};

/// The complete JLCPCB SMT-assembly file set: Gerber+drill zip, CPL, and
/// BOM. Named fields (not a bare tuple) so GUI/MCP/CLI can report each
/// path without positional ambiguity.
pub struct ManufacturingFiles {
    pub gerber_zip: PathBuf,
    pub position_csv: PathBuf,
    pub bom_csv: PathBuf,
}

/// Soldermask expansion beyond copper pad size. KiCad-compatible /
/// JLCPCB plots use **0** (mask opening == pad size) when no
/// per-pad/board `solder_mask_margin` is set -- verified by golden
/// diff on the LED-panel board. Keep in lockstep with that.
const MASK_EXPANSION: Unit = 0;

/// Edge-cuts stroke width (cosmetic; fabs use the centreline).
const EDGE_STROKE: Unit = MM / 10; // 0.10 mm

/// KiCad-compatible Gerber/position plots negate board-Y (`Y' = -Y`).
/// Matching that convention keeps native Gerbers + CPL drop-in
/// compatible with JLCPCB and other fabs that expect KiCad-style coords.
fn fab_point(p: Point) -> Point {
    Point::new(p.x, -p.y)
}

#[derive(Debug)]
pub enum NativeGerberError {
    Io(std::io::Error),
    Zip(zip::result::ZipError),
}

impl std::fmt::Display for NativeGerberError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NativeGerberError::Io(e) => write!(f, "filesystem error: {e}"),
            NativeGerberError::Zip(e) => write!(f, "zip error: {e}"),
        }
    }
}

impl From<std::io::Error> for NativeGerberError {
    fn from(e: std::io::Error) -> Self {
        NativeGerberError::Io(e)
    }
}

impl From<zip::result::ZipError> for NativeGerberError {
    fn from(e: zip::result::ZipError) -> Self {
        NativeGerberError::Zip(e)
    }
}

/// One named file that goes into the Gerber zip.
struct NamedFile {
    name: String,
    contents: String,
}

/// Export manufacturing files natively from `doc` into `out_dir`.
///
/// Writes `<stem>_gerbers.zip`, `<stem>_cpl.csv`, and `<stem>_bom.csv`
/// (`bom_csv_contents` is the already-rendered JLCPCB BOM from
/// `crate::bom`). Does **not** need KiCad installed.
pub fn export_manufacturing_files_native(
    doc: &BoardDoc,
    templates: &[FootprintTemplate],
    stem: &str,
    out_dir: &Path,
    bom_csv_contents: &str,
) -> Result<ManufacturingFiles, NativeGerberError> {
    set_generation_software("Dragan Bojovic", "Alladin PCB", env!("CARGO_PKG_VERSION"));
    std::fs::create_dir_all(out_dir)?;

    let files = build_gerber_files(doc, templates, stem);
    let gerber_zip = out_dir.join(format!("{stem}_gerbers.zip"));
    zip_named_files(&files, &gerber_zip)?;

    let position_csv = out_dir.join(format!("{stem}_cpl.csv"));
    std::fs::write(&position_csv, build_jlcpcb_cpl(doc, templates))?;

    let bom_csv = out_dir.join(format!("{stem}_bom.csv"));
    std::fs::write(&bom_csv, bom_csv_contents)?;

    Ok(ManufacturingFiles {
        gerber_zip,
        position_csv,
        bom_csv,
    })
}

fn build_gerber_files(
    doc: &BoardDoc,
    templates: &[FootprintTemplate],
    stem: &str,
) -> Vec<NamedFile> {
    let mut f_cu = GerberLayer::new("Copper,L1,Top,Signal", false);
    let mut b_cu = GerberLayer::new("Copper,L2,Bot,Signal", false);
    let mut f_mask = GerberLayer::new("Soldermask,Top", true);
    let mut b_mask = GerberLayer::new("Soldermask,Bot", true);
    let mut f_paste = GerberLayer::new("Paste,Top", false);
    let mut b_paste = GerberLayer::new("Paste,Bot", false);
    let mut f_silk = GerberLayer::new("Legend,Top", false);
    let mut b_silk = GerberLayer::new("Legend,Bot", false);
    let mut edge = GerberLayer::new("Profile,NP", false);
    let mut pth = ExcellonFile::new(DrillKind::Plated);
    let mut npth = ExcellonFile::new(DrillKind::NonPlated);

    // --- Tracks / vias / zones / free holes from the node ------------
    // Pads come from templates below (true R/O/C apertures). Emitting
    // them from `Item::Pad`'s collision polygons would explode into a
    // unique UserPolygon macro per flash.
    for item in doc.node.iter() {
        match item {
            Item::Track { shape, layer, .. } => {
                let layer_g = if *layer == LayerId::FCu {
                    &mut f_cu
                } else {
                    &mut b_cu
                };
                layer_g.add_trace_line(
                    fab_point(shape.a),
                    fab_point(shape.b),
                    shape.width,
                    "Conductor",
                );
            }
            Item::Via { shape, drill, .. } => {
                let master = PadMaster::Circle(GerberCircle::new(shape.radius * 2, "ViaPad"));
                let c = fab_point(shape.center);
                f_cu.add_pad(master.clone(), c, 0.0);
                b_cu.add_pad(master, c, 0.0);
                pth.add_hole(c, *drill);
            }
            Item::Pad { .. } => {}
            Item::Zone { outline, layer, .. } => {
                let pts: Vec<Point> = outline.points.iter().copied().map(fab_point).collect();
                let path = GerberPath::from_closed_ring(&pts);
                let layer_g = if *layer == LayerId::FCu {
                    &mut f_cu
                } else {
                    &mut b_cu
                };
                layer_g.add_region(path, "Conductor", false);
            }
            // Mounting holes are emitted from footprint templates below
            // (same source BOM/CPL use). Skipping `Item::Hole`
            // here avoids doubling every NPTH when the hole also lives
            // on its template.
            Item::Hole { .. } => {}
        }
    }

    // --- Pads from templates: copper / mask / paste / THT drills -----
    for placed in &doc.footprints {
        let Some(template) = templates.iter().find(|t| t.name == placed.template_name) else {
            continue;
        };
        for pad in &template.pads {
            let center = fab_point(pad_world_position(
                pad.offset,
                placed.position,
                placed.rotation_deg,
            ));
            // Y-flip mirrors rotation sense too: Gerber/pos use the
            // negated board angle (KiCad-compatible `at` convention).
            let total_rot = -(pad.rotation_deg + placed.rotation_deg);
            let is_tht = pad.hole_diameter.is_some();
            let (cu_fn, mask_fn) = if is_tht {
                ("ComponentPad", "ComponentPad")
            } else {
                ("SMDPad,CuDef", "SMDPad,CuDef")
            };

            let cu = pad_template_master(pad, 0, cu_fn);
            if is_tht {
                f_cu.add_pad(cu.clone(), center, total_rot);
                b_cu.add_pad(cu, center, total_rot);
            } else if pad.layer == LayerId::FCu {
                f_cu.add_pad(cu, center, total_rot);
            } else {
                b_cu.add_pad(cu, center, total_rot);
            }

            // Mask opening (negative polarity layer -- flash = opening).
            let mask_master = pad_template_master(pad, MASK_EXPANSION, mask_fn);
            if pad.layer == LayerId::FCu || is_tht {
                f_mask.add_pad(mask_master.clone(), center, total_rot);
            }
            if pad.layer == LayerId::BCu || is_tht {
                b_mask.add_pad(mask_master, center, total_rot);
            }

            // Paste: SMD only, same size as copper (no expansion).
            if !is_tht {
                let paste_master = pad_template_master(pad, 0, cu_fn);
                if pad.layer == LayerId::FCu {
                    f_paste.add_pad(paste_master, center, total_rot);
                } else {
                    b_paste.add_pad(paste_master, center, total_rot);
                }
            }

            if let Some(drill) = pad.hole_diameter {
                pth.add_hole(center, drill);
            }
        }
        for hole in &template.holes {
            let center = fab_point(pad_world_position(
                hole.offset,
                placed.position,
                placed.rotation_deg,
            ));
            npth.add_hole(center, hole.drill);
            // Mechanical holes also need a soldermask opening on both sides
            // so solder mask doesn't tent the hole shut.
            let master = PadMaster::Circle(GerberCircle::new(
                hole.drill + MASK_EXPANSION * 2,
                "ComponentPad",
            ));
            f_mask.add_pad(master.clone(), center, 0.0);
            b_mask.add_pad(master, center, 0.0);
        }
    }

    // Vias are tented (no soldermask opening) -- same default as KiCad's
    // plotter. Mask flashes above only cover pads and mechanical holes.

    // --- Silkscreen -------------------------------------------------
    for text in &doc.silk_texts {
        let silk = if text.layer == LayerId::FCu {
            &mut f_silk
        } else {
            &mut b_silk
        };
        for seg in text.stroke_segments() {
            silk.add_trace_line(
                fab_point(seg.a),
                fab_point(seg.b),
                seg.width,
                "NonConductor",
            );
        }
    }
    for dot in &doc.silk_dots {
        let silk = if dot.layer == LayerId::FCu {
            &mut f_silk
        } else {
            &mut b_silk
        };
        silk.add_pad(
            PadMaster::Circle(GerberCircle::new(dot.diameter, "NonConductor")),
            fab_point(dot.position),
            0.0,
        );
    }
    for fp in &doc.footprints {
        if let Some(c) = fp.pin1_marker_circle() {
            f_silk.add_pad(
                PadMaster::Circle(GerberCircle::new(c.radius * 2, "NonConductor")),
                fab_point(c.center),
                0.0,
            );
        }
    }

    // --- Board outline ----------------------------------------------
    for poly in &doc.outline {
        let pts: Vec<Point> = poly.points.iter().copied().map(fab_point).collect();
        let path = GerberPath::from_closed_ring(&pts);
        edge.add_traces_path(path, EDGE_STROKE, "Profile", false);
    }

    let mut out = Vec::new();
    out.push(NamedFile {
        name: format!("{stem}-F_Cu.gtl"),
        contents: f_cu.dump(),
    });
    out.push(NamedFile {
        name: format!("{stem}-B_Cu.gbl"),
        contents: b_cu.dump(),
    });
    out.push(NamedFile {
        name: format!("{stem}-F_Mask.gts"),
        contents: f_mask.dump(),
    });
    out.push(NamedFile {
        name: format!("{stem}-B_Mask.gbs"),
        contents: b_mask.dump(),
    });
    out.push(NamedFile {
        name: format!("{stem}-F_Paste.gtp"),
        contents: f_paste.dump(),
    });
    out.push(NamedFile {
        name: format!("{stem}-B_Paste.gbp"),
        contents: b_paste.dump(),
    });
    out.push(NamedFile {
        name: format!("{stem}-F_Silkscreen.gto"),
        contents: f_silk.dump(),
    });
    out.push(NamedFile {
        name: format!("{stem}-B_Silkscreen.gbo"),
        contents: b_silk.dump(),
    });
    out.push(NamedFile {
        name: format!("{stem}-Edge_Cuts.gm1"),
        contents: edge.dump(),
    });
    if !pth.is_empty() {
        out.push(NamedFile {
            name: format!("{stem}-PTH.drl"),
            contents: pth.dump(),
        });
    }
    if !npth.is_empty() {
        out.push(NamedFile {
            name: format!("{stem}-NPTH.drl"),
            contents: npth.dump(),
        });
    }
    out
}

fn pad_template_master(
    pad: &crate::footprint::PadTemplate,
    expansion: Unit,
    function: &str,
) -> PadMaster {
    match pad.shape {
        PadShapeKind::Circle => {
            PadMaster::Circle(GerberCircle::new(pad.radius * 2 + expansion * 2, function))
        }
        PadShapeKind::Rect { width, height } => PadMaster::Rectangle(Rectangle::new(
            width + expansion * 2,
            height + expansion * 2,
            function,
        )),
        PadShapeKind::Oval { width, height } => PadMaster::Oblong(Oblong::new(
            width + expansion * 2,
            height + expansion * 2,
            function,
        )),
    }
}

/// JLCPCB CPL CSV directly from placed footprints (no kicad-cli).
fn build_jlcpcb_cpl(doc: &BoardDoc, templates: &[FootprintTemplate]) -> String {
    let mut out = String::from("Designator,Mid X,Mid Y,Layer,Rotation\n");
    let mut rows: Vec<_> = doc.footprints.iter().collect();
    rows.sort_by(|a, b| a.reference.cmp(&b.reference));
    for fp in rows {
        // Skip pure mechanical hole parts (exclude_from_bom) -- same as BOM.
        if templates
            .iter()
            .find(|t| t.name == fp.template_name)
            .map(|t| t.exclude_from_bom)
            .unwrap_or(false)
        {
            continue;
        }
        // Layer: a part is "Top" if it has any front-side pad, else Bottom.
        let layer = templates
            .iter()
            .find(|t| t.name == fp.template_name)
            .map(|t| {
                if t.pads.iter().any(|p| p.layer == LayerId::FCu) || t.pads.is_empty() {
                    "Top"
                } else {
                    "Bottom"
                }
            })
            .unwrap_or("Top");
        // Match KiCad-compatible / JLCPCB position CSV: Y negated vs
        // board coords, rotation negated vs Alladin's internal angle.
        // Bottom parts get an extra mirror convention in some fab pos
        // formats — we keep the same single negation for both sides,
        // which matched KiCad-style top-side output on the LED-panel
        // board.
        let x = alladin_gerber::to_mm(fp.position.x);
        let y = alladin_gerber::to_mm(-fp.position.y);
        let rot = -fp.rotation_deg;
        out.push_str(&format!(
            "{},{:.6},{:.6},{},{:.6}\n",
            crate::bom::csv_field(&fp.reference.to_uppercase()),
            x,
            y,
            layer,
            rot
        ));
    }
    out
}

fn zip_named_files(files: &[NamedFile], zip_path: &Path) -> Result<(), NativeGerberError> {
    let file = std::fs::File::create(zip_path)?;
    let mut writer = zip::ZipWriter::new(file);
    write_named_files_to_zip(&mut writer, files)?;
    writer.finish()?;
    Ok(())
}

fn write_named_files_to_zip<W: Write + Seek>(
    writer: &mut zip::ZipWriter<W>,
    files: &[NamedFile],
) -> Result<(), NativeGerberError> {
    let options = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Deflated);
    let mut sorted: Vec<&NamedFile> = files.iter().collect();
    sorted.sort_by(|a, b| a.name.cmp(&b.name));
    for f in sorted {
        writer.start_file(&f.name, options)?;
        writer.write_all(f.contents.as_bytes())?;
    }
    Ok(())
}

/// In-memory manufacturing bundle (gerbers + CPL + BOM) for WASM download.
#[cfg(target_arch = "wasm32")]
pub fn export_manufacturing_zip_bytes(
    doc: &BoardDoc,
    templates: &[FootprintTemplate],
    stem: &str,
    bom_csv_contents: &str,
) -> Result<Vec<u8>, NativeGerberError> {
    set_generation_software("Dragan Bojovic", "Alladin PCB", env!("CARGO_PKG_VERSION"));
    let mut files = build_gerber_files(doc, templates, stem);
    files.push(NamedFile {
        name: format!("{stem}_cpl.csv"),
        contents: build_jlcpcb_cpl(doc, templates),
    });
    files.push(NamedFile {
        name: format!("{stem}_bom.csv"),
        contents: bom_csv_contents.to_string(),
    });
    let mut cursor = std::io::Cursor::new(Vec::new());
    {
        let mut writer = zip::ZipWriter::new(&mut cursor);
        write_named_files_to_zip(&mut writer, &files)?;
        writer.finish()?;
    }
    Ok(cursor.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board_doc::{CopperWeight, LayerCount, NewBoardParams};
    use crate::footprint::PadTemplate;
    use alladin_core::LayerId;

    fn empty_board() -> BoardDoc {
        NewBoardParams {
            width_mm: 40.0,
            height_mm: 40.0,
            layer_count: LayerCount::Two,
            copper_weight: CopperWeight::OneOz,
            corner_radius_mm: 0.0,
        }
        .create()
    }

    #[test]
    fn native_export_writes_zip_and_cpl_for_an_empty_board() {
        let board = empty_board();
        let dir = std::env::temp_dir().join(format!(
            "alladin_native_gerber_empty_{}",
            std::process::id()
        ));
        let files = export_manufacturing_files_native(
            &board,
            &[],
            "board",
            &dir,
            "Comment,Designator,Footprint,LCSC Part #\n",
        )
        .unwrap();
        assert!(files.gerber_zip.exists());
        assert!(files.position_csv.exists());
        assert!(files.bom_csv.exists());
        assert!(std::fs::read_to_string(&files.bom_csv)
            .unwrap()
            .starts_with("Comment,Designator,Footprint,LCSC Part #"));

        let zip_file = std::fs::File::open(&files.gerber_zip).unwrap();
        let mut archive = zip::ZipArchive::new(zip_file).unwrap();
        let mut names: Vec<String> = (0..archive.len())
            .map(|i| archive.by_index(i).unwrap().name().to_string())
            .collect();
        names.sort();
        assert!(names.iter().any(|n| n.ends_with("-F_Cu.gtl")));
        assert!(names.iter().any(|n| n.ends_with("-B_Cu.gbl")));
        assert!(names.iter().any(|n| n.ends_with("-Edge_Cuts.gm1")));
        assert!(names.iter().any(|n| n.ends_with("-F_Mask.gts")));
        assert!(names.iter().any(|n| n.ends_with("-F_Silkscreen.gto")));

        let edge = archive.by_name("board-Edge_Cuts.gm1").unwrap();
        // Drop to read -- ZipFile needs to be consumed; re-open via by_name after.
        drop(edge);
        let mut edge = archive.by_name("board-Edge_Cuts.gm1").unwrap();
        let mut edge_text = String::new();
        std::io::Read::read_to_string(&mut edge, &mut edge_text).unwrap();
        assert!(edge_text.contains("Profile,NP"));
        assert!(edge_text.contains("D01*"), "outline must be stroked");

        let cpl = std::fs::read_to_string(&files.position_csv).unwrap();
        assert!(cpl.starts_with("Designator,Mid X,Mid Y,Layer,Rotation\n"));

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn native_export_puts_a_via_on_both_copper_layers_and_in_pth_drill() {
        let mut board = empty_board();
        board.node.add(Item::Via {
            shape: alladin_geom::Circle::new(Point::new(0, 0), MM / 2),
            drill: MM / 3,
            net: None,
        });
        let dir =
            std::env::temp_dir().join(format!("alladin_native_gerber_via_{}", std::process::id()));
        let files = export_manufacturing_files_native(
            &board,
            &[],
            "via",
            &dir,
            "Comment,Designator,Footprint,LCSC Part #\n",
        )
        .unwrap();
        let zip_file = std::fs::File::open(&files.gerber_zip).unwrap();
        let mut archive = zip::ZipArchive::new(zip_file).unwrap();

        let mut f_cu = String::new();
        std::io::Read::read_to_string(&mut archive.by_name("via-F_Cu.gtl").unwrap(), &mut f_cu)
            .unwrap();
        let mut b_cu = String::new();
        std::io::Read::read_to_string(&mut archive.by_name("via-B_Cu.gbl").unwrap(), &mut b_cu)
            .unwrap();
        assert!(f_cu.contains("ViaPad"));
        assert!(b_cu.contains("ViaPad"));
        assert!(f_cu.contains("D03*"));

        let mut pth = String::new();
        std::io::Read::read_to_string(&mut archive.by_name("via-PTH.drl").unwrap(), &mut pth)
            .unwrap();
        assert!(pth.contains("Plated,1,2,PTH"));
        assert!(pth.contains("X0Y0") || pth.contains("X0.0Y0"));

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn native_export_bakes_silk_text_into_front_silkscreen() {
        let mut board = empty_board();
        board
            .try_place_silk_text(
                "ALLADIN",
                Point::new(0, 0),
                0.0,
                LayerId::FCu,
                crate::board_doc::DEFAULT_SILK_TEXT_HEIGHT,
            )
            .unwrap();
        let dir =
            std::env::temp_dir().join(format!("alladin_native_gerber_silk_{}", std::process::id()));
        let files = export_manufacturing_files_native(
            &board,
            &[],
            "silk",
            &dir,
            "Comment,Designator,Footprint,LCSC Part #\n",
        )
        .unwrap();
        let zip_file = std::fs::File::open(&files.gerber_zip).unwrap();
        let mut archive = zip::ZipArchive::new(zip_file).unwrap();
        let mut silk = String::new();
        std::io::Read::read_to_string(
            &mut archive.by_name("silk-F_Silkscreen.gto").unwrap(),
            &mut silk,
        )
        .unwrap();
        assert!(silk.contains("D01*"), "silk strokes must draw");
        assert!(
            silk.matches("D01*").count() > 5,
            "a word of stroke font must produce many segments"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn cpl_lists_a_placed_smd_part() {
        let mut board = empty_board();
        let template = FootprintTemplate {
            name: "C_0402".to_string(),
            reference_prefix: "C".to_string(),
            pads: vec![
                PadTemplate {
                    offset: Point::new(-MM / 2, 0),
                    radius: MM / 4,
                    layer: LayerId::FCu,
                    number: "1".into(),
                    shape: PadShapeKind::Rect {
                        width: MM / 2,
                        height: MM / 3,
                    },
                    rotation_deg: 0.0,
                    hole_diameter: None,
                    pin_name: None,
                    zone_connection: ZoneConnection::Thermal,
                },
                PadTemplate {
                    offset: Point::new(MM / 2, 0),
                    radius: MM / 4,
                    layer: LayerId::FCu,
                    number: "2".into(),
                    shape: PadShapeKind::Rect {
                        width: MM / 2,
                        height: MM / 3,
                    },
                    rotation_deg: 0.0,
                    hole_diameter: None,
                    pin_name: None,
                    zone_connection: ZoneConnection::Thermal,
                },
            ],
            holes: Vec::new(),
            exclude_from_bom: false,
            explicit_courtyard: None,
        };
        board
            .try_place_footprint(&template, Point::new(MM, 2 * MM), 90.0)
            .unwrap();
        let cpl = build_jlcpcb_cpl(&board, &[template]);
        // Fab convention: Y and rotation negated vs board coords.
        assert!(
            cpl.contains("C1,1.000000,-2.000000,Top,-90.000000"),
            "got:\n{cpl}"
        );
    }
}
