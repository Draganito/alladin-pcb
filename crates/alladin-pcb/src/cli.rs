//! Headless CLI: board create, parts download, pin connect, and
//! read-only listing. No KiCad bridge, no external autorouter, no
//! placement batch tools.

use crate::board_doc::{BoardDoc, CopperWeight, LayerCount, NewBoardParams};
use crate::footprint::FootprintTemplate;
use crate::parts_db::PartsDb;
use alladin_core::ItemId;
use alladin_geom::MM;
use std::path::PathBuf;

#[derive(clap::Parser)]
#[command(
    name = "alladin-pcb",
    about = "Correct-by-construction PCB editor. Run with no arguments to launch the GUI; any subcommand below runs headless instead."
)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Command,
}

#[derive(clap::Subcommand)]
pub enum Command {
    /// Creates a new, empty board and saves it as an Alladin PCB `.json` file.
    NewBoard {
        path: PathBuf,
        #[arg(long, default_value_t = 50.0)]
        width_mm: f32,
        #[arg(long, default_value_t = 30.0)]
        height_mm: f32,
        /// Must be 2. Alladin is 2-layer only; older `--layers 1` is rejected.
        #[arg(long, default_value_t = LayerCount::Two)]
        layers: LayerCount,
        #[arg(long = "copper-oz", default_value_t = CopperWeight::OneOz)]
        copper_weight: CopperWeight,
        #[arg(long, default_value_t = 1.0)]
        corner_radius_mm: f32,
    },
    /// Downloads a part from LCSC/EasyEDA into the local parts database.
    DownloadPart {
        /// LCSC C-number, e.g. `C2040`.
        lcsc: String,
    },
    /// Joins two pins onto the same net.
    Connect {
        board: PathBuf,
        ref1: String,
        pin1: String,
        ref2: String,
        pin2: String,
    },
    /// Lists nets and pad counts.
    ListNets { board: PathBuf },
    /// Lists footprints.
    ListFootprints { board: PathBuf },
    /// Compact board summary.
    BoardSummary { board: PathBuf },
}

fn open_parts_db() -> PartsDb {
    crate::app::open_parts_db()
}

fn load_templates(parts_db: &PartsDb) -> Vec<FootprintTemplate> {
    crate::app::load_templates(parts_db).0
}

fn load_board(path: &PathBuf, parts_db: &PartsDb) -> Result<BoardDoc, String> {
    let templates = load_templates(parts_db);
    let (doc, _) = crate::app::load_from_path(path, &templates, parts_db)?;
    Ok(doc)
}

fn save_board(doc: &BoardDoc, path: &PathBuf, parts_db: &PartsDb) -> Result<(), String> {
    let (templates, template_origin, _, _) = crate::app::load_templates(parts_db);
    crate::app::save_to_path(doc, path, &templates, &template_origin, parts_db)
}

fn find_pin(doc: &BoardDoc, templates: &[FootprintTemplate], reference: &str, pin: &str) -> Result<ItemId, String> {
    let fp = doc
        .footprints
        .iter()
        .find(|f| f.reference == reference)
        .ok_or_else(|| format!("no footprint with reference {reference}"))?;
    let template = templates
        .iter()
        .find(|t| t.name == fp.template_name)
        .ok_or_else(|| format!("template missing for {reference}"))?;
    let idx = template
        .pads
        .iter()
        .position(|p| p.number == pin)
        .ok_or_else(|| format!("no pin {pin} on {reference}"))?;
    Ok(fp.pad_item_ids[idx])
}

pub fn run(cli: Cli) -> Result<(), String> {
    match cli.command {
        Command::NewBoard { path, width_mm, height_mm, layers, copper_weight, corner_radius_mm } => {
            let params = NewBoardParams {
                width_mm,
                height_mm,
                layer_count: layers,
                copper_weight,
                corner_radius_mm,
            };
            if !params.is_valid() {
                return Err("board dimensions/corner radius are not valid".into());
            }
            let doc = params.create();
            let parts_db = open_parts_db();
            save_board(&doc, &path, &parts_db)?;
            println!("created {}", path.display());
            Ok(())
        }
        Command::DownloadPart { lcsc } => {
            let parts_db = open_parts_db();
            let fetched = crate::lcsc::fetch_by_lcsc_code(&lcsc).map_err(|e| e.to_string())?;
            let mut screen = crate::app::Screen::NewBoard(NewBoardParams::default());
            let json = crate::app::download_lcsc_part_write(&mut screen, &parts_db, Ok(fetched));
            println!("{json}");
            if json.get("ok") == Some(&serde_json::Value::Bool(true)) {
                Ok(())
            } else {
                Err(json.get("error").and_then(|e| e.as_str()).unwrap_or("download failed").to_string())
            }
        }
        Command::Connect { board, ref1, pin1, ref2, pin2 } => {
            let parts_db = open_parts_db();
            let mut doc = load_board(&board, &parts_db)?;
            let templates = load_templates(&parts_db);
            let a = find_pin(&doc, &templates, &ref1, &pin1)?;
            let b = find_pin(&doc, &templates, &ref2, &pin2)?;
            let net = doc.connect_pads(a, b).map_err(|e| e.to_string())?;
            let name = doc.nets.iter().find(|n| n.id == net).map(|n| n.name.as_str()).unwrap_or("?");
            save_board(&doc, &board, &parts_db)?;
            println!("connected {ref1}.{pin1} -- {ref2}.{pin2} on {name}");
            Ok(())
        }
        Command::ListNets { board } => {
            let parts_db = open_parts_db();
            let doc = load_board(&board, &parts_db)?;
            for net in &doc.nets {
                let pads = doc.pads_on_net(net.id).len();
                println!("{} (id {}) pads={}", net.name, net.id.0, pads);
            }
            Ok(())
        }
        Command::ListFootprints { board } => {
            let parts_db = open_parts_db();
            let doc = load_board(&board, &parts_db)?;
            for fp in &doc.footprints {
                println!(
                    "{} {} @({:.2},{:.2}) r={}",
                    fp.reference,
                    fp.template_name,
                    fp.position.x as f64 / MM as f64,
                    fp.position.y as f64 / MM as f64,
                    fp.rotation_deg
                );
            }
            Ok(())
        }
        Command::BoardSummary { board } => {
            let parts_db = open_parts_db();
            let doc = load_board(&board, &parts_db)?;
            println!(
                "footprints={} nets={} tracks={} vias={} zones={}",
                doc.footprints.len(),
                doc.nets.len(),
                doc.node.iter().filter(|i| matches!(i, alladin_core::Item::Track { .. })).count(),
                doc.node.iter().filter(|i| matches!(i, alladin_core::Item::Via { .. })).count(),
                doc.zones.len()
            );
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_board_writes_a_loadable_json() {
        let dir = std::env::temp_dir().join(format!("alladin-cli-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("t.json");
        run(Cli {
            command: Command::NewBoard {
                path: path.clone(),
                width_mm: 20.0,
                height_mm: 10.0,
                layers: LayerCount::Two,
                copper_weight: CopperWeight::OneOz,
                corner_radius_mm: 0.5,
            },
        })
        .unwrap();
        let parts_db = PartsDb::open_in_memory().unwrap();
        let doc = load_board(&path, &parts_db).unwrap();
        assert_eq!(doc.layer_count, LayerCount::Two);
    }
}
