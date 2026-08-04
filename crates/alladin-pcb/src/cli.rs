//! The scripting/AI entry point the user's original vision calls out
//! explicitly: "die Schnittstelle nicht vergessen für KI, damit diese
//! Parts runterladen kann, Board erstellen, Parts platzieren und
//! Netzliste machen kann" (see the development log's "Teil 29"
//! entry). Every subcommand here is a thin wrapper around the exact
//! same headless, already-tested logic the GUI (`crate::app`) drives --
//! `BoardDoc`, `crate::parts_db`, `crate::lcsc`, `crate::bom`,
//! `crate::native_gerber` -- so a script and a human editing the same
//! board file can never observe different behaviour.
//!
//! A script/AI drives a board across several separate process
//! invocations (`place-part`, then `connect`, then `export-manufacturing`, ...),
//! so every mutating subcommand here follows "load the `.json` board,
//! apply one change, save it back" -- there's no long-lived process
//! state between commands, deliberately: it's the same on-disk file
//! `crate::app`'s own Open/Save round-trips through, so the GUI can pick
//! up right where a script left off (and vice versa).
//!
//! KiCad board import/export is intentionally *not* a CLI product
//! feature -- Alladin's own `.json` is the editable format; `.kicad_pcb`
//! remains an internal bridge for the external autorouter only.

use crate::board_doc::{BoardDoc, CopperWeight, LayerCount, NewBoardParams, ZoneId};
use crate::footprint::{FootprintTemplate, HoleTemplate, PadTemplate};
use crate::parts_db::PartsDb;
use alladin_core::LayerId;
use alladin_geom::{Point, Polygon, Unit, MM};
use std::path::PathBuf;

/// Which of the two copper layers [`Command::AddZone`]'s `--layer`
/// targets, spelled `front`/`back` on the command line (matching how a
/// script/AI thinks about a board -- "the top pour") rather than the
/// exact KiCad layer name, but mapping 1:1 to
/// [`alladin_core::LayerId::FCu`]/`BCu` -- the same two-value
/// vocabulary the GUI's own zone-layer picker offers (`app.rs`:
/// "F.Cu"/"B.Cu").
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum ZoneLayerArg {
    Front,
    Back,
}

impl ZoneLayerArg {
    fn to_layer_id(self) -> LayerId {
        match self {
            ZoneLayerArg::Front => LayerId::FCu,
            ZoneLayerArg::Back => LayerId::BCu,
        }
    }

    /// The KiCad-style name to print in CLI output (e.g. `add-zone`'s
    /// summary line), distinct from the `front`/`back` spelling
    /// [`std::fmt::Display`] below uses for `--help`/`default_value_t`.
    fn as_kicad_str(self) -> &'static str {
        match self {
            ZoneLayerArg::Front => "F.Cu",
            ZoneLayerArg::Back => "B.Cu",
        }
    }
}

impl std::fmt::Display for ZoneLayerArg {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", if *self == ZoneLayerArg::Front { "front" } else { "back" })
    }
}

#[derive(clap::Parser)]
#[command(
    name = "alladin-pcb",
    about = "Correct-by-construction PCB editor. Run with no arguments to launch the GUI; any subcommand below runs headless instead, for scripting or driving from an AI agent."
)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Command,
}

#[derive(clap::Subcommand)]
pub enum Command {
    /// Creates a new, empty board and saves it as an Alladin PCB `.json` file.
    NewBoard {
        /// Where to save the new board (an Alladin PCB `.json` file, not KiCad's).
        path: PathBuf,
        #[arg(long, default_value_t = 50.0)]
        width_mm: f32,
        #[arg(long, default_value_t = 30.0)]
        height_mm: f32,
        /// 1 or 2.
        #[arg(long, default_value_t = LayerCount::Two)]
        layers: LayerCount,
        /// 1 or 2 (oz/ft²) -- picks which real JLCPCB DFM/clearance rules
        /// this board enforces for its whole lifetime. 2oz needs wider
        /// track spacing (0.16mm vs 1oz's 0.10mm) but carries more current.
        #[arg(long = "copper-oz", default_value_t = CopperWeight::OneOz)]
        copper_weight: CopperWeight,
        #[arg(long, default_value_t = 1.0)]
        corner_radius_mm: f32,
    },
    /// Lists every footprint template available for `place-part --template`
    /// (built-ins plus everything in your local parts database).
    ListTemplates,
    /// Downloads a part from LCSC/EasyEDA by its C-number and saves it,
    /// with full pad geometry, to your local parts database.
    DownloadPart {
        /// e.g. `C2040`.
        lcsc_code: String,
    },
    /// Re-fetches an *already-downloaded* part and overwrites its
    /// existing parts-database row in place -- for backfilling a
    /// genuinely new field (pin names, courtyard, ...) onto every
    /// part downloaded before that field existed, without losing the
    /// part's own database row id (`download-part` alone would
    /// refuse it as a duplicate instead). Refuses if `lcsc_code`
    /// isn't already in your library -- use `download-part` for that.
    UpdatePart {
        /// e.g. `C2040`.
        lcsc_code: String,
    },
    /// Places a template instance on a board, saving the result back to the same file.
    ///
    /// `--x-mm`/`--y-mm`/`--rotation-deg` accept negative values directly
    /// (e.g. `--x-mm -10`), since a board's origin is its center -- no
    /// `-- -10` escaping needed.
    #[command(allow_negative_numbers = true)]
    PlacePart {
        /// The board file to modify in place.
        board: PathBuf,
        /// A template name from `list-templates` (an exact match).
        #[arg(long)]
        template: String,
        #[arg(long)]
        x_mm: f64,
        #[arg(long)]
        y_mm: f64,
        #[arg(long, default_value_t = 0.0)]
        rotation_deg: f64,
    },
    /// Joins two pins onto the same electrical net, saving the result back to the same file.
    Connect {
        /// The board file to modify in place.
        board: PathBuf,
        /// Reference designator of the first pin's footprint, e.g. `U1`.
        #[arg(long)]
        ref1: String,
        /// Pad number on that footprint, e.g. `1`.
        #[arg(long)]
        pin1: String,
        #[arg(long)]
        ref2: String,
        #[arg(long)]
        pin2: String,
    },
    /// Exports the complete JLCPCB SMT set natively (no KiCad):
    /// `<stem>_gerbers.zip`, `<stem>_cpl.csv`, and `<stem>_bom.csv`.
    ExportManufacturing {
        board: PathBuf,
        /// Output directory for Gerber zip, CPL, and BOM.
        out_dir: PathBuf,
    },
    /// Replaces a board's outline with an arbitrary polygon shape --
    /// chamfered corners, notches, cutouts, whatever the real board
    /// needs, not just `new-board`'s rounded rect. Exactly one of
    /// `--from-kicad`/`--points-file` must be given. Refuses (leaving
    /// the board file untouched) if any already-placed footprint,
    /// track, or via would fall outside the new shape.
    SetOutline {
        /// The board file to modify in place.
        board: PathBuf,
        /// Lift *just* the `Edge.Cuts` outline out of a reference
        /// `.kicad_pcb` file (outline geometry only -- not a full
        /// board import).
        #[arg(long)]
        from_kicad: Option<PathBuf>,
        /// A JSON file with one or more polygons, each a list of
        /// `{"x_mm": .., "y_mm": ..}` points in board-millimetres, e.g.
        /// `[[{"x_mm":-20,"y_mm":-10}, {"x_mm":20,"y_mm":-10}, ...]]` --
        /// for a fully parametric outline with no reference file at
        /// hand. More than one polygon (even-odd combined, same
        /// convention as [`crate::board_doc::BoardDoc::outline`]) is
        /// how to add a separate internal cutout.
        #[arg(long)]
        points_file: Option<PathBuf>,
    },
    /// Registers a new part into your local parts database (for
    /// `place-part --template`), without the GUI's "Add part..." form
    /// -- mirrors it (`app.rs`'s `AddPartForm`) but reachable from a
    /// script/AI. Give exactly one of the two shape flags:
    /// `--pin-count` for a straight row of solder pads (a generic wire
    /// pad, resistor, header, ...), or `--hole-diameter-mm` for a pure
    /// mechanical mounting hole (NPTH, no copper pads at all).
    RegisterPart {
        /// Shown in `list-templates` and used as `place-part --template`.
        name: String,
        /// Prefix for auto-generated reference designators, e.g. `W` for a wire pad, `H` for a mounting hole.
        #[arg(long, default_value = "U")]
        reference_prefix: String,
        #[arg(long, default_value = "")]
        description: String,
        /// A straight row of this many through-hole solder pads, evenly spaced by `--pitch-mm`.
        #[arg(long)]
        pin_count: Option<u32>,
        #[arg(long, default_value_t = 2.54)]
        pitch_mm: f64,
        #[arg(long, default_value_t = 0.45)]
        pad_radius_mm: f64,
        /// A pure mechanical, unplated (NPTH) mounting hole of this drill diameter -- no copper, no net.
        #[arg(long)]
        hole_diameter_mm: Option<f64>,
        /// Marks the part so the manufacturing BOM skips it -- e.g. a wire pad or mounting hole is never a purchasable line item.
        #[arg(long)]
        exclude_from_bom: bool,
        /// Where this part shows up in the GUI's "Place part" category tree, e.g. "Custom". Left empty, it files under "Uncategorized".
        #[arg(long, default_value = "")]
        category: String,
    },
    /// Draws and fills a new copper zone/pour, saving the result back to
    /// the same file -- the headless equivalent of `Tool::DrawZone` in
    /// the GUI. Extends the "AI builds everything, human only routes"
    /// pipeline to also cover pours.
    AddZone {
        /// The board file to modify in place.
        board: PathBuf,
        /// An already-existing net's name (as printed by a prior
        /// `connect` call, e.g. `Net1`) -- a zone can only target a net
        /// that already exists.
        #[arg(long)]
        net: String,
        /// `front` or `back` (F.Cu / B.Cu).
        #[arg(long, default_value_t = ZoneLayerArg::Front)]
        layer: ZoneLayerArg,
        /// A JSON file with a *single* polygon's board-millimetre
        /// points, e.g. `[{"x_mm":-10,"y_mm":-10}, ...]` -- one nesting
        /// level shallower than `set-outline --points-file` (a zone
        /// outline is one polygon, not several).
        #[arg(long)]
        points_file: PathBuf,
    },
    /// Re-runs every zone's fill against the board's current state --
    /// needed because a fill is a point-in-time snapshot that goes
    /// stale as more parts/tracks get added after the pour was drawn.
    RefillZones {
        /// The board file to modify in place.
        board: PathBuf,
    },
    /// Read-only: lists every zone currently on the board (id, net,
    /// layer, outline point count, current filled-island count), so a
    /// script/AI can check zone state without needing the GUI.
    ListZones {
        board: PathBuf,
    },
    /// Places a via (e.g. a GND stitching via), saving the result back
    /// to the same file. The headless equivalent of the GUI's
    /// `Tool::PlaceVia`.
    #[command(allow_negative_numbers = true)]
    AddVia {
        /// The board file to modify in place.
        board: PathBuf,
        /// An already-existing net's name (from a prior `connect` call).
        #[arg(long)]
        net: String,
        #[arg(long)]
        x_mm: f64,
        #[arg(long)]
        y_mm: f64,
        /// Outer copper diameter. Defaults to the same 0.6mm the GUI's
        /// manual routing uses.
        #[arg(long, default_value_t = 0.6)]
        diameter_mm: f64,
        /// Drill diameter. Defaults to the same 0.3mm the GUI's manual
        /// routing uses.
        #[arg(long, default_value_t = 0.3)]
        drill_mm: f64,
    },
    /// Auto-routes a straight-to-DRC-clear copper trace between two
    /// already-connected pins on their shared layer, saving the result
    /// back to the same file -- lets a script/AI finish simple,
    /// single-layer connections (e.g. "wire up the +5V pads") without
    /// a human steering the interactive router. Both pins must already
    /// share a net (run `connect` first) and sit on the same copper
    /// layer; there's no automatic via/layer-hop insertion.
    Route {
        /// The board file to modify in place.
        board: PathBuf,
        #[arg(long)]
        ref1: String,
        #[arg(long)]
        pin1: String,
        #[arg(long)]
        ref2: String,
        #[arg(long)]
        pin2: String,
    },
    /// Runs the optional external KiCadRoutingTools autorouter
    /// (github.com/drandyhaas/KiCadRoutingTools -- a separate,
    /// user-installed tool, entirely outside alladin-pcb itself) against
    /// the board: exports it, runs `route.py` as a subprocess (its
    /// stdout/stderr streamed straight to this process's own, live),
    /// then merges the resulting tracks/vias into the board and saves
    /// it back -- the same pipeline the GUI's "Autoroute (extern)"
    /// dialog runs, just headless and blocking end-to-end (a real run
    /// can take real minutes). A safety backup of the board from right
    /// before the merge is written alongside it as
    /// `<board>.before-autoroute.json` -- there's no undo stack, so
    /// this is the way back from an unwanted result. Requires the tool
    /// to already be configured (via the GUI's Autoroute (extern)
    /// settings window, or `--tool-dir` here for a one-off).
    AutorouteExternal {
        /// The board file to modify in place.
        board: PathBuf,
        /// Overrides the persisted Autoroute (extern) settings' tool
        /// folder (the one directly containing `route.py`) for this
        /// run only -- omit to use whatever's already configured via
        /// the GUI.
        #[arg(long)]
        tool_dir: Option<PathBuf>,
        /// An exact net name (as printed by a prior `connect` call) to
        /// route -- repeat for more than one. Omit entirely to route
        /// every net on the board with more than one pad.
        #[arg(long)]
        nets: Vec<String>,
        /// Extra arguments appended verbatim to `route.py`'s own argv
        /// (e.g. `"--bus"`), overriding -- for this run only, never
        /// saved -- the persisted settings' own extra-arguments field.
        #[arg(long)]
        extra_args: Option<String>,
    },
}

/// [`Command::AutorouteExternal`]'s `Option<bool>` DRC/connectivity
/// check fields, spelled out for the CLI's own plain-text summary
/// line -- same three-way meaning as
/// `crate::external_router::AutorouteReport::drc_ok`'s own doc comment
/// (missing script vs. actually failed are different things worth
/// telling apart).
fn fmt_autoroute_check(ok: Option<bool>) -> &'static str {
    match ok {
        Some(true) => "passed",
        Some(false) => "FAILED",
        None => "not available (script not found in the tool folder)",
    }
}

fn mm(v: f64) -> Unit {
    (v * MM as f64).round() as Unit
}

/// The `list-templates` subcommand's actual formatting logic, split out
/// from [`run`] so it's testable without a live parts database.
fn format_templates(templates: &[FootprintTemplate], origin: &[Option<i64>], category: &[Option<String>]) -> String {
    let mut lines = Vec::with_capacity(templates.len());
    for ((template, origin), category) in templates.iter().zip(origin).zip(category) {
        let source = if origin.is_some() { "db" } else { "builtin" };
        let category = category.as_deref().unwrap_or(crate::parts_db::UNCATEGORIZED_LABEL);
        lines.push(format!("{}\t[{source}]\t[{category}]\t{} pads", template.name, template.pads.len()));
    }
    lines.join("\n")
}

/// [`Command::PlacePart`]'s logic, taking an already-loaded board and
/// template list so it's testable without any file I/O. Returns the
/// newly placed part's auto-generated reference designator.
fn place_part(
    doc: &mut BoardDoc,
    templates: &[FootprintTemplate],
    template_name: &str,
    x_mm: f64,
    y_mm: f64,
    rotation_deg: f64,
) -> Result<String, String> {
    let template = templates
        .iter()
        .find(|t| t.name == template_name)
        .ok_or_else(|| format!("unknown template \"{template_name}\" -- run `list-templates` to see what's available"))?;
    let position = Point::new(mm(x_mm), mm(y_mm));
    let id = doc.try_place_footprint(template, position, rotation_deg).map_err(|e| format!("couldn't place {template_name}: {e}"))?;
    Ok(doc.footprints.iter().find(|f| f.id == id).expect("just-placed footprint must exist").reference.clone())
}

/// [`Command::Connect`]'s logic, taking an already-loaded board and
/// template list so it's testable without any file I/O. Returns the
/// joined net's human-readable name.
fn connect_pins(
    doc: &mut BoardDoc,
    templates: &[FootprintTemplate],
    ref1: &str,
    pin1: &str,
    ref2: &str,
    pin2: &str,
) -> Result<String, String> {
    let a = doc.find_pad(templates, ref1, pin1).ok_or_else(|| format!("no such pin: {ref1} pin {pin1}"))?;
    let b = doc.find_pad(templates, ref2, pin2).ok_or_else(|| format!("no such pin: {ref2} pin {pin2}"))?;
    let net = doc.connect_pads(a, b).map_err(|e| format!("couldn't connect {ref1}.{pin1} to {ref2}.{pin2}: {e}"))?;
    Ok(doc.nets.iter().find(|n| n.id == net).map(|n| n.name.clone()).unwrap_or_else(|| "?".to_string()))
}

/// [`Command::AddVia`]'s logic, taking an already-loaded board so it's
/// testable without any file I/O -- same net-name-resolution shape as
/// [`crate::board_doc::BoardDoc::find_net_by_name`]'s other caller,
/// `add_zone`. Returns the newly placed via's id.
fn add_via(doc: &mut BoardDoc, net_name: &str, x_mm: f64, y_mm: f64, diameter_mm: f64, drill_mm: f64) -> Result<alladin_core::ItemId, String> {
    let net = doc
        .find_net_by_name(net_name)
        .ok_or_else(|| format!("unknown net \"{net_name}\" -- only a net already created by a prior `connect` call can be targeted"))?;
    let position = Point::new(mm(x_mm), mm(y_mm));
    doc.try_add_stitching_via(position, net, mm(diameter_mm), mm(drill_mm))
        .map_err(|e| format!("couldn't place a via at ({x_mm}, {y_mm})mm: {e}"))
}

/// [`Command::Route`]'s logic, taking an already-loaded board and
/// template list so it's testable without any file I/O -- same
/// pin-resolution shape as [`connect_pins`]. Returns the number of
/// `Item::Track` legs committed.
fn route_pins(doc: &mut BoardDoc, templates: &[FootprintTemplate], ref1: &str, pin1: &str, ref2: &str, pin2: &str) -> Result<usize, String> {
    let a = doc.find_pad(templates, ref1, pin1).ok_or_else(|| format!("no such pin: {ref1} pin {pin1}"))?;
    let b = doc.find_pad(templates, ref2, pin2).ok_or_else(|| format!("no such pin: {ref2} pin {pin2}"))?;
    doc.try_route_pads(a, b).map_err(|e| format!("couldn't route {ref1}.{pin1} -- {ref2}.{pin2}: {e}"))
}

/// One point of a [`Command::SetOutline`] `--points-file` polygon,
/// board-millimetres (matching every other `--x-mm`/`--y-mm`-style flag
/// in this module) rather than raw internal nanometre [`Unit`]s, so a
/// script/AI writing this file doesn't have to know Alladin's internal
/// scale at all.
#[derive(serde::Deserialize)]
struct OutlinePointMm {
    x_mm: f64,
    y_mm: f64,
}

/// [`Command::SetOutline`]'s logic for turning its two mutually
/// exclusive input flags into the actual `Vec<Polygon>` [`BoardDoc::set_outline`]
/// needs, split out from [`run`] so the "exactly one of the two, valid
/// JSON, non-empty polygons" validation is testable without any file
/// I/O of its own beyond what the caller already read.
fn resolve_outline(from_kicad_source: Option<&str>, points_json: Option<&str>) -> Result<Vec<Polygon>, String> {
    match (from_kicad_source, points_json) {
        (Some(_), Some(_)) => Err("--from-kicad and --points-file are mutually exclusive -- give exactly one".to_string()),
        (None, None) => Err("give exactly one of --from-kicad or --points-file".to_string()),
        (Some(source), None) => alladin_kicad_io::import_outline_only(source).map_err(|e| e.to_string()),
        (None, Some(json)) => {
            let polygons: Vec<Vec<OutlinePointMm>> = serde_json::from_str(json).map_err(|e| format!("invalid --points-file JSON: {e}"))?;
            if polygons.is_empty() {
                return Err("--points-file must contain at least one polygon".to_string());
            }
            Ok(polygons
                .into_iter()
                .map(|points| Polygon::new(points.into_iter().map(|p| Point::new(mm(p.x_mm), mm(p.y_mm))).collect()))
                .collect())
        }
    }
}

/// [`Command::RegisterPart`]'s logic for turning its two mutually
/// exclusive shape flags into the actual pads/holes
/// [`crate::parts_db::PartsDb::insert_part`] needs, split out from
/// [`run`] so the "exactly one of the two" validation is testable
/// without a real database.
fn build_registered_part(
    name: &str,
    reference_prefix: &str,
    pin_count: Option<u32>,
    pitch_mm: f64,
    pad_radius_mm: f64,
    hole_diameter_mm: Option<f64>,
) -> Result<(Vec<PadTemplate>, Vec<HoleTemplate>), String> {
    match (pin_count, hole_diameter_mm) {
        (Some(_), Some(_)) => {
            Err("--pin-count and --hole-diameter-mm are mutually exclusive -- a wire pad and a mounting hole are two different parts".to_string())
        }
        (None, None) => Err("give exactly one of --pin-count (a row of solder pads) or --hole-diameter-mm (a mounting hole)".to_string()),
        (Some(pin_count), None) => {
            let template = crate::footprint::straight_row_template(name.to_string(), reference_prefix.to_string(), pin_count, pitch_mm, pad_radius_mm);
            Ok((template.pads, Vec::new()))
        }
        (None, Some(drill_mm)) => Ok((Vec::new(), vec![HoleTemplate { offset: Point::new(0, 0), drill: mm(drill_mm) }])),
    }
}

/// [`Command::AddZone`]'s `--points-file` parsing: a *single* polygon's
/// board-millimetre points, reusing [`OutlinePointMm`]'s point shape --
/// one nesting level shallower than [`resolve_outline`]'s `Vec<Vec<..>>`
/// (a zone outline is one [`Polygon`], not several), split out from
/// [`run`] so the "valid JSON, non-empty" validation is testable
/// without any file I/O.
fn resolve_zone_outline(points_json: &str) -> Result<Polygon, String> {
    let points: Vec<OutlinePointMm> = serde_json::from_str(points_json).map_err(|e| format!("invalid --points-file JSON: {e}"))?;
    if points.is_empty() {
        return Err("--points-file must contain at least one point".to_string());
    }
    Ok(Polygon::new(points.into_iter().map(|p| Point::new(mm(p.x_mm), mm(p.y_mm))).collect()))
}

/// [`Command::AddZone`]'s logic, taking an already-loaded board so it's
/// testable without any file I/O. Returns the newly created zone's id
/// -- the caller looks the rest (island count, etc.) up from
/// `doc.zones` afterwards, same "return just what the caller couldn't
/// otherwise get back" shape as [`place_part`]'s reference designator.
fn add_zone(doc: &mut BoardDoc, net_name: &str, layer: LayerId, outline: Polygon) -> Result<ZoneId, String> {
    let net = doc
        .find_net_by_name(net_name)
        .ok_or_else(|| format!("unknown net \"{net_name}\" -- only a net already created by a prior `connect` call can be targeted"))?;
    Ok(doc.add_zone(outline, layer, net))
}

/// [`Command::ListZones`]'s formatting logic, split out from [`run`] so
/// it's testable without any file I/O -- same shape as
/// [`format_templates`].
fn format_zones(doc: &BoardDoc) -> String {
    let mut lines = Vec::with_capacity(doc.zones.len());
    for zone in &doc.zones {
        let net_name = doc.nets.iter().find(|n| n.id == zone.net).map(|n| n.name.as_str()).unwrap_or("?");
        let layer = match zone.layer {
            LayerId::FCu => "F.Cu",
            LayerId::BCu => "B.Cu",
        };
        lines.push(format!(
            "Zone {}\tnet=\"{net_name}\"\tlayer={layer}\toutline_points={}\tfilled_islands={}",
            zone.id.0,
            zone.outline.points.len(),
            zone.item_ids.len(),
        ));
    }
    lines.join("\n")
}

fn load_board_with_templates(board: &std::path::Path, parts_db: &PartsDb) -> Result<(BoardDoc, Vec<FootprintTemplate>), String> {
    let (templates, _, _, _) = crate::app::load_templates(parts_db);
    let doc = crate::app::load_from_path(board, &templates)?;
    Ok((doc, templates))
}

pub fn run(cli: Cli) -> Result<(), String> {
    match cli.command {
        Command::NewBoard { path, width_mm, height_mm, layers, copper_weight, corner_radius_mm } => {
            let params = NewBoardParams { width_mm, height_mm, layer_count: layers, copper_weight, corner_radius_mm };
            if !params.is_valid() {
                return Err(format!(
                    "invalid board: {width_mm}x{height_mm}mm with a {corner_radius_mm}mm corner radius isn't physically sane"
                ));
            }
            let doc = params.create();
            crate::app::save_to_path(&doc, &path)?;
            println!("Created a {width_mm}x{height_mm}mm, {layers}-layer, {copper_weight} board at {}", path.display());
            Ok(())
        }
        Command::ListTemplates => {
            let parts_db = crate::app::open_parts_db();
            let (templates, origin, _, category) = crate::app::load_templates(&parts_db);
            println!("{}", format_templates(&templates, &origin, &category));
            Ok(())
        }
        Command::DownloadPart { lcsc_code } => {
            let parts_db = crate::app::open_parts_db();
            let part = crate::lcsc::fetch_by_lcsc_code(&lcsc_code).map_err(|e| e.to_string())?;
            let record = parts_db
                .insert_part_categorized(
                    &part.name,
                    &part.reference_prefix,
                    &part.description,
                    Some(&part.lcsc_code),
                    &part.pads,
                    &[],
                    false,
                    part.explicit_courtyard,
                    part.category.as_deref(),
                )
                .map_err(|e| e.to_string())?;
            println!("Downloaded and saved: \"{}\" ({}), {} pads", record.template.name, lcsc_code, record.template.pads.len());
            Ok(())
        }
        Command::UpdatePart { lcsc_code } => {
            let parts_db = crate::app::open_parts_db();
            let part = crate::lcsc::fetch_by_lcsc_code(&lcsc_code).map_err(|e| e.to_string())?;
            let record = parts_db
                .update_part_by_lcsc_code(
                    &lcsc_code,
                    &part.name,
                    &part.reference_prefix,
                    &part.description,
                    &part.pads,
                    &[],
                    part.explicit_courtyard,
                    part.category.as_deref(),
                )
                .map_err(|e| e.to_string())?;
            let courtyard_note = match record.template.explicit_courtyard {
                Some(c) => format!("{:.2}mm x {:.2}mm silkscreen courtyard", c.width as f64 / MM as f64, c.height as f64 / MM as f64),
                None => "no silkscreen courtyard in the source data (falls back to the pad bounding box)".to_string(),
            };
            println!("Updated: \"{}\" ({}), {} pads, {courtyard_note}", record.template.name, lcsc_code, record.template.pads.len());
            Ok(())
        }
        Command::PlacePart { board, template, x_mm, y_mm, rotation_deg } => {
            let parts_db = crate::app::open_parts_db();
            let (mut doc, templates) = load_board_with_templates(&board, &parts_db)?;
            let reference = place_part(&mut doc, &templates, &template, x_mm, y_mm, rotation_deg)?;
            crate::app::save_to_path(&doc, &board)?;
            println!("Placed {reference} ({template}) at ({x_mm}, {y_mm})mm, {rotation_deg}deg");
            Ok(())
        }
        Command::Connect { board, ref1, pin1, ref2, pin2 } => {
            let parts_db = crate::app::open_parts_db();
            let (mut doc, templates) = load_board_with_templates(&board, &parts_db)?;
            let net_name = connect_pins(&mut doc, &templates, &ref1, &pin1, &ref2, &pin2)?;
            crate::app::save_to_path(&doc, &board)?;
            println!("Connected {ref1}.{pin1} -- {ref2}.{pin2} on {net_name}");
            Ok(())
        }
        Command::ExportManufacturing { board, out_dir } => {
            let parts_db = crate::app::open_parts_db();
            let (doc, templates) = load_board_with_templates(&board, &parts_db)?;
            let (_, origin, _, _) = crate::app::load_templates(&parts_db);
            let bom_csv = crate::bom::to_csv(&crate::bom::build_bom_rows(&doc, &templates, &origin, &parts_db));
            let stem = board.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
            let files = crate::native_gerber::export_manufacturing_files_native(&doc, &templates, stem, &out_dir, &bom_csv).map_err(|e| e.to_string())?;
            println!("Native manufacturing export:");
            println!("  gerbers: {}", files.gerber_zip.display());
            println!("  cpl:     {}", files.position_csv.display());
            println!("  bom:     {}", files.bom_csv.display());
            Ok(())
        }
        Command::SetOutline { board, from_kicad, points_file } => {
            let from_kicad_source = from_kicad.as_deref().map(std::fs::read_to_string).transpose().map_err(|e| e.to_string())?;
            let points_json = points_file.as_deref().map(std::fs::read_to_string).transpose().map_err(|e| e.to_string())?;
            let new_outline = resolve_outline(from_kicad_source.as_deref(), points_json.as_deref())?;

            let parts_db = crate::app::open_parts_db();
            let (mut doc, templates) = load_board_with_templates(&board, &parts_db)?;
            let polygon_count = new_outline.len();
            doc.set_outline(new_outline, &templates).map_err(|e| format!("couldn't set the new outline: {e}"))?;
            crate::app::save_to_path(&doc, &board)?;
            println!("Set a new {polygon_count}-polygon outline on {}", board.display());
            Ok(())
        }
        Command::RegisterPart { name, reference_prefix, description, pin_count, pitch_mm, pad_radius_mm, hole_diameter_mm, exclude_from_bom, category } => {
            let (pads, holes) = build_registered_part(&name, &reference_prefix, pin_count, pitch_mm, pad_radius_mm, hole_diameter_mm)?;
            let parts_db = crate::app::open_parts_db();
            let category = (!category.trim().is_empty()).then_some(category.trim());
            let record = parts_db
                .insert_part_categorized(&name, &reference_prefix, &description, None, &pads, &holes, exclude_from_bom, None, category)
                .map_err(|e| e.to_string())?;
            let bom_note = if exclude_from_bom { " (excluded from BOM)" } else { "" };
            println!(
                "Registered \"{}\": {} pad(s), {} hole(s){bom_note}",
                record.template.name,
                record.template.pads.len(),
                record.template.holes.len()
            );
            Ok(())
        }
        Command::AddZone { board, net, layer, points_file } => {
            let points_json = std::fs::read_to_string(&points_file).map_err(|e| e.to_string())?;
            let outline = resolve_zone_outline(&points_json)?;
            let point_count = outline.points.len();

            let parts_db = crate::app::open_parts_db();
            let (mut doc, _templates) = load_board_with_templates(&board, &parts_db)?;
            let zone_id = add_zone(&mut doc, &net, layer.to_layer_id(), outline)?;
            let island_count = doc.zones.iter().find(|z| z.id == zone_id).expect("just-added zone must exist").item_ids.len();
            crate::app::save_to_path(&doc, &board)?;
            println!(
                "Added zone {} on net \"{net}\" ({}): {island_count} filled island(s) from a {point_count}-point outline",
                zone_id.0,
                layer.as_kicad_str()
            );
            Ok(())
        }
        Command::RefillZones { board } => {
            let parts_db = crate::app::open_parts_db();
            let (mut doc, _templates) = load_board_with_templates(&board, &parts_db)?;
            let zone_count = doc.zones.len();
            doc.refill_all_zones();
            crate::app::save_to_path(&doc, &board)?;
            println!("Refilled {zone_count} zone(s) on {}", board.display());
            Ok(())
        }
        Command::ListZones { board } => {
            let parts_db = crate::app::open_parts_db();
            let (doc, _templates) = load_board_with_templates(&board, &parts_db)?;
            println!("{}", format_zones(&doc));
            Ok(())
        }
        Command::AddVia { board, net, x_mm, y_mm, diameter_mm, drill_mm } => {
            let parts_db = crate::app::open_parts_db();
            let (mut doc, _templates) = load_board_with_templates(&board, &parts_db)?;
            add_via(&mut doc, &net, x_mm, y_mm, diameter_mm, drill_mm)?;
            crate::app::save_to_path(&doc, &board)?;
            println!("Added a via on net \"{net}\" at ({x_mm}, {y_mm})mm ({diameter_mm}mm/{drill_mm}mm drill)");
            Ok(())
        }
        Command::Route { board, ref1, pin1, ref2, pin2 } => {
            let parts_db = crate::app::open_parts_db();
            let (mut doc, templates) = load_board_with_templates(&board, &parts_db)?;
            let segments = route_pins(&mut doc, &templates, &ref1, &pin1, &ref2, &pin2)?;
            crate::app::save_to_path(&doc, &board)?;
            println!("Routed {ref1}.{pin1} -- {ref2}.{pin2}: {segments} track segment(s)");
            Ok(())
        }
        Command::AutorouteExternal { board, tool_dir, nets, extra_args } => {
            let parts_db = crate::app::open_parts_db();
            let (mut doc, templates) = load_board_with_templates(&board, &parts_db)?;

            let mut settings = crate::external_router::ExternalRouterSettings::load();
            if let Some(dir) = tool_dir {
                settings.tool_dir = dir.to_string_lossy().into_owned();
            }
            if let Some(extra) = extra_args {
                settings.extra_args = extra;
            }
            let net_names = if nets.is_empty() { doc.multi_item_net_names() } else { nets };
            if net_names.is_empty() {
                return Err("no nets to route: the board has no net with more than one pad, and --nets gave none explicitly".to_string());
            }

            let handle = crate::external_router::run_autoroute(&doc, &templates, net_names, settings).map_err(|e| e.to_string())?;
            let report = loop {
                match handle.events.recv() {
                    Ok(crate::external_router::AutorouteEvent::Log(line)) => println!("{line}"),
                    Ok(crate::external_router::AutorouteEvent::Done(result)) => break result.map_err(|e| e.to_string())?,
                    Err(_) => return Err("the external autoroute background thread ended unexpectedly".to_string()),
                }
            };

            let item_count = report.items.len();
            let backup_path = board.with_extension("before-autoroute.json");
            crate::app::merge_autoroute_items(&mut doc, &Some(board.clone()), report.items)
                .map_err(|e| format!("route.py finished successfully but the result couldn't be merged: {e}"))?;
            crate::app::save_to_path(&doc, &board)?;

            println!(
                "Autorouted {}/{} requested net(s), merged {item_count} track/via item(s) into {} (backup: {}). DRC check: {}. Connectivity check: {}.",
                report.routed_nets.len(),
                report.requested_nets.len(),
                board.display(),
                backup_path.display(),
                fmt_autoroute_check(report.drc_ok),
                fmt_autoroute_check(report.connected_ok),
            );
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board_doc::{CopperWeight, LayerCount};

    fn two_pin_template() -> FootprintTemplate {
        crate::footprint::builtin_templates().remove(0)
    }

    fn test_board() -> BoardDoc {
        NewBoardParams { width_mm: 40.0, height_mm: 40.0, layer_count: LayerCount::Two, copper_weight: CopperWeight::OneOz, corner_radius_mm: 0.0 }.create()
    }

    #[test]
    fn format_templates_marks_db_and_builtin_origins_distinctly() {
        let templates = crate::footprint::builtin_templates();
        let origin = vec![None; templates.len()];
        let category = vec![None; templates.len()];
        let text = format_templates(&templates, &origin, &category);
        assert!(text.contains("[builtin]"));
        assert!(!text.contains("[db]"));

        let origin_from_db = vec![Some(1_i64); templates.len()];
        let text = format_templates(&templates, &origin_from_db, &category);
        assert!(text.contains("[db]"));
    }

    #[test]
    fn format_templates_shows_a_real_category_or_uncategorized_as_a_fallback() {
        let templates = vec![crate::footprint::builtin_templates().remove(0), crate::footprint::builtin_templates().remove(0)];
        let origin = vec![Some(1_i64), Some(2_i64)];
        let category = vec![Some("Resistors".to_string()), None];
        let text = format_templates(&templates, &origin, &category);
        assert!(text.contains("[Resistors]"));
        assert!(text.contains("[Uncategorized]"));
    }

    #[test]
    fn place_part_succeeds_and_returns_the_auto_generated_reference() {
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        let reference = place_part(&mut doc, &templates, &templates[0].name.clone(), 0.0, 0.0, 0.0).unwrap();
        assert_eq!(reference, "P1");
        assert_eq!(doc.footprints.len(), 1);
    }

    #[test]
    fn place_part_rejects_an_unknown_template_name() {
        let mut doc = test_board();
        let err = place_part(&mut doc, &[], "no-such-template", 0.0, 0.0, 0.0).unwrap_err();
        assert!(err.contains("unknown template"), "unexpected error: {err}");
    }

    #[test]
    fn place_part_surfaces_a_rejected_off_board_placement() {
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        let err = place_part(&mut doc, &templates, &templates[0].name.clone(), 1000.0, 1000.0, 0.0).unwrap_err();
        assert!(err.contains("couldn't place"), "unexpected error: {err}");
    }

    #[test]
    fn connect_pins_joins_two_pads_by_reference_and_pad_number() {
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        place_part(&mut doc, &templates, &templates[0].name.clone(), -10.0, 0.0, 0.0).unwrap();
        place_part(&mut doc, &templates, &templates[0].name.clone(), 10.0, 0.0, 0.0).unwrap();

        let net_name = connect_pins(&mut doc, &templates, "P1", "1", "P2", "1").unwrap();
        assert_eq!(net_name, "Net1");
        assert_eq!(doc.nets.len(), 1);
    }

    #[test]
    fn resolve_outline_rejects_neither_flag_given() {
        let err = resolve_outline(None, None).unwrap_err();
        assert!(err.contains("exactly one"), "unexpected error: {err}");
    }

    #[test]
    fn resolve_outline_rejects_both_flags_given() {
        let err = resolve_outline(Some("(kicad_pcb)"), Some("[]")).unwrap_err();
        assert!(err.contains("mutually exclusive"), "unexpected error: {err}");
    }

    #[test]
    fn resolve_outline_rejects_an_empty_points_file() {
        let err = resolve_outline(None, Some("[]")).unwrap_err();
        assert!(err.contains("at least one polygon"), "unexpected error: {err}");
    }

    #[test]
    fn resolve_outline_parses_a_points_file_into_millimetre_scaled_polygons() {
        let json = r#"[[{"x_mm": -10.0, "y_mm": -5.0}, {"x_mm": 10.0, "y_mm": -5.0}, {"x_mm": 10.0, "y_mm": 5.0}, {"x_mm": -10.0, "y_mm": 5.0}]]"#;
        let outline = resolve_outline(None, Some(json)).unwrap();
        assert_eq!(outline.len(), 1);
        assert_eq!(outline[0].points, vec![
            Point::new(-10 * MM, -5 * MM),
            Point::new(10 * MM, -5 * MM),
            Point::new(10 * MM, 5 * MM),
            Point::new(-10 * MM, 5 * MM),
        ]);
    }

    #[test]
    fn resolve_outline_supports_more_than_one_polygon_for_a_cutout() {
        let json = r#"[
            [{"x_mm": -10.0, "y_mm": -10.0}, {"x_mm": 10.0, "y_mm": -10.0}, {"x_mm": 10.0, "y_mm": 10.0}, {"x_mm": -10.0, "y_mm": 10.0}],
            [{"x_mm": -1.0, "y_mm": -1.0}, {"x_mm": 1.0, "y_mm": -1.0}, {"x_mm": 1.0, "y_mm": 1.0}, {"x_mm": -1.0, "y_mm": 1.0}]
        ]"#;
        let outline = resolve_outline(None, Some(json)).unwrap();
        assert_eq!(outline.len(), 2, "a second polygon must come through as a separate even-odd cutout piece");
    }

    #[test]
    fn resolve_outline_lifts_just_the_edge_cuts_outline_from_a_reference_kicad_file() {
        let outline_polys = vec![Polygon::rounded_rect(20 * MM, 15 * MM, 0, 4)];
        let source = alladin_kicad_io::write_kicad_pcb(&outline_polys, &[], &alladin_core::Node::new(), &[], &[], &[], &[]);
        let outline = resolve_outline(Some(&source), None).unwrap();
        assert_eq!(outline.len(), 1, "the rectangle's own 4-segment outline must chain into exactly one closed polygon");
    }

    #[test]
    fn build_registered_part_rejects_neither_shape_flag_given() {
        let err = build_registered_part("Nothing", "U", None, 2.54, 0.45, None).unwrap_err();
        assert!(err.contains("exactly one"), "unexpected error: {err}");
    }

    #[test]
    fn build_registered_part_rejects_both_shape_flags_given() {
        let err = build_registered_part("Both", "U", Some(2), 2.54, 0.45, Some(2.2)).unwrap_err();
        assert!(err.contains("mutually exclusive"), "unexpected error: {err}");
    }

    #[test]
    fn build_registered_part_builds_a_straight_row_of_pads_from_pin_count() {
        let (pads, holes) = build_registered_part("Header", "J", Some(3), 2.54, 0.45, None).unwrap();
        assert_eq!(pads.len(), 3);
        assert!(holes.is_empty(), "a wire-pad-style part has no mechanical holes");
        assert_eq!(pads[0].number, "1");
        assert_eq!(pads[2].number, "3");
    }

    #[test]
    fn build_registered_part_builds_a_single_mounting_hole_from_hole_diameter() {
        let (pads, holes) = build_registered_part("M2 Hole", "H", None, 2.54, 0.45, Some(2.2)).unwrap();
        assert!(pads.is_empty(), "a mounting-hole-style part has no copper pads");
        assert_eq!(holes.len(), 1);
        assert_eq!(holes[0].drill, mm(2.2));
        assert_eq!(holes[0].offset, Point::new(0, 0));
    }

    #[test]
    fn register_part_flow_saves_a_mounting_hole_marked_excluded_from_bom() {
        let parts_db = crate::parts_db::PartsDb::open_in_memory().unwrap();

        let (pads, holes) = build_registered_part("M2 Mounting Hole", "H", None, 2.54, 0.45, Some(2.2)).unwrap();
        let record = parts_db.insert_part("M2 Mounting Hole", "H", "", None, &pads, &holes, true).unwrap();

        assert!(record.template.pads.is_empty());
        assert_eq!(record.template.holes.len(), 1);
        assert!(record.template.exclude_from_bom);

        let reloaded = parts_db.list_parts().unwrap();
        assert_eq!(reloaded.len(), 1);
        assert!(reloaded[0].template.exclude_from_bom, "must round-trip through PartsDb, not just hold in memory as a local value");
    }

    #[test]
    fn register_part_flow_with_a_blank_category_string_saves_it_as_uncategorized() {
        // Mirrors exactly the `Command::RegisterPart` handler's own
        // "" -> `None` trimming, without going through `run()` itself
        // (which reaches for the *real* on-disk parts database via
        // `crate::app::open_parts_db()` -- never something a unit test
        // should touch).
        let parts_db = crate::parts_db::PartsDb::open_in_memory().unwrap();
        let (pads, holes) = build_registered_part("Blank Category Part", "U", None, 2.54, 0.45, Some(2.2)).unwrap();
        let category = "";
        let category = (!category.trim().is_empty()).then_some(category.trim());
        let record = parts_db.insert_part_categorized("Blank Category Part", "U", "", None, &pads, &holes, false, None, category).unwrap();
        assert_eq!(record.category, None);
    }

    #[test]
    fn register_part_flow_with_a_real_category_saves_it_trimmed() {
        let parts_db = crate::parts_db::PartsDb::open_in_memory().unwrap();
        let (pads, holes) = build_registered_part("Custom Part", "U", None, 2.54, 0.45, Some(2.2)).unwrap();
        let category = "  Custom  ";
        let category = (!category.trim().is_empty()).then_some(category.trim());
        let record = parts_db.insert_part_categorized("Custom Part", "U", "", None, &pads, &holes, false, None, category).unwrap();
        assert_eq!(record.category, Some("Custom".to_string()));
    }

    #[test]
    fn connect_pins_rejects_an_unknown_pin() {
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        place_part(&mut doc, &templates, &templates[0].name.clone(), 0.0, 0.0, 0.0).unwrap();

        let err = connect_pins(&mut doc, &templates, "P1", "99", "P1", "1").unwrap_err();
        assert!(err.contains("no such pin"), "unexpected error: {err}");
    }

    fn square_outline_mm(half_width_mm: f64) -> Polygon {
        let half = mm(half_width_mm);
        Polygon::new(vec![Point::new(-half, -half), Point::new(half, -half), Point::new(half, half), Point::new(-half, half)])
    }

    #[test]
    fn resolve_zone_outline_rejects_an_empty_points_file() {
        let err = resolve_zone_outline("[]").unwrap_err();
        assert!(err.contains("at least one point"), "unexpected error: {err}");
    }

    #[test]
    fn resolve_zone_outline_rejects_invalid_json() {
        let err = resolve_zone_outline("not json").unwrap_err();
        assert!(err.contains("invalid --points-file JSON"), "unexpected error: {err}");
    }

    #[test]
    fn resolve_zone_outline_parses_a_single_polygons_millimetre_scaled_points() {
        // One nesting level shallower than `resolve_outline`'s
        // `Vec<Vec<..>>` -- a zone outline is a single flat point list.
        let json = r#"[{"x_mm": -10.0, "y_mm": -10.0}, {"x_mm": 10.0, "y_mm": -10.0}, {"x_mm": 10.0, "y_mm": 10.0}, {"x_mm": -10.0, "y_mm": 10.0}]"#;
        let outline = resolve_zone_outline(json).unwrap();
        assert_eq!(outline.points, vec![
            Point::new(-10 * MM, -10 * MM),
            Point::new(10 * MM, -10 * MM),
            Point::new(10 * MM, 10 * MM),
            Point::new(-10 * MM, 10 * MM),
        ]);
    }

    #[test]
    fn add_zone_rejects_an_unknown_net_name() {
        let mut doc = test_board();
        let err = add_zone(&mut doc, "no-such-net", LayerId::FCu, square_outline_mm(10.0)).unwrap_err();
        assert!(err.contains("unknown net"), "unexpected error: {err}");
        assert!(doc.zones.is_empty(), "a rejected add-zone must not record anything");
    }

    #[test]
    fn add_zone_on_a_valid_net_fills_at_least_one_island() {
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        place_part(&mut doc, &templates, &templates[0].name.clone(), -15.0, -15.0, 0.0).unwrap();
        place_part(&mut doc, &templates, &templates[0].name.clone(), -15.0, 15.0, 0.0).unwrap();
        let net_name = connect_pins(&mut doc, &templates, "P1", "1", "P2", "1").unwrap();

        let zone_id = add_zone(&mut doc, &net_name, LayerId::FCu, square_outline_mm(10.0)).unwrap();

        assert_eq!(doc.zones.len(), 1);
        let record = doc.zones.iter().find(|z| z.id == zone_id).unwrap();
        assert!(!record.item_ids.is_empty(), "an obstacle-free pour must fill to at least one island");
        assert_eq!(record.layer, LayerId::FCu);
    }

    #[test]
    fn format_zones_prints_net_layer_outline_point_count_and_island_count() {
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        place_part(&mut doc, &templates, &templates[0].name.clone(), -15.0, -15.0, 0.0).unwrap();
        place_part(&mut doc, &templates, &templates[0].name.clone(), -15.0, 15.0, 0.0).unwrap();
        let net_name = connect_pins(&mut doc, &templates, "P1", "1", "P2", "1").unwrap();
        add_zone(&mut doc, &net_name, LayerId::FCu, square_outline_mm(10.0)).unwrap();

        let text = format_zones(&doc);
        assert!(text.contains(&format!("net=\"{net_name}\"")), "unexpected output: {text}");
        assert!(text.contains("layer=F.Cu"), "unexpected output: {text}");
        assert!(text.contains("outline_points=4"), "unexpected output: {text}");
        assert!(text.contains("filled_islands=1"), "unexpected output: {text}");
    }

    #[test]
    fn format_zones_is_empty_for_a_board_with_no_zones() {
        let doc = test_board();
        assert_eq!(format_zones(&doc), "");
    }

    #[test]
    fn refill_all_zones_splits_a_pour_after_a_different_net_track_is_added_through_it() {
        // Simulates "the board changed after the pour was drawn" (a
        // later track routed straight through it) the same way
        // `zone_fill`'s own tests build obstacles: by inserting
        // directly into `doc.node`, since a *live* `try`-gated route
        // through already-filled zone copper would itself be refused
        // as a collision -- refilling is precisely what reconciles the
        // pour with such a change afterwards.
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        place_part(&mut doc, &templates, &templates[0].name.clone(), -15.0, -15.0, 0.0).unwrap();
        place_part(&mut doc, &templates, &templates[0].name.clone(), -15.0, 15.0, 0.0).unwrap();
        let net_name = connect_pins(&mut doc, &templates, "P1", "1", "P2", "1").unwrap();
        let zone_id = add_zone(&mut doc, &net_name, LayerId::FCu, square_outline_mm(10.0)).unwrap();
        let islands_before = doc.zones.iter().find(|z| z.id == zone_id).unwrap().item_ids.len();
        assert_eq!(islands_before, 1, "an obstacle-free pour must fill to a single island");

        let other_net = doc.create_net();
        doc.node.add(alladin_core::Item::Track {
            shape: alladin_geom::Segment::new(Point::new(-10 * MM, 0), Point::new(10 * MM, 0), mm(0.25)),
            net: Some(other_net),
            layer: LayerId::FCu,
            class: alladin_core::NetClass::C,
        });

        doc.refill_all_zones();
        let islands_after = doc.zones.iter().find(|z| z.id == zone_id).unwrap().item_ids.len();
        assert_eq!(islands_after, 2, "the track must have split the single pour into two islands, one above and one below it");
    }

    #[test]
    fn add_via_rejects_an_unknown_net_name() {
        let mut doc = test_board();
        let err = add_via(&mut doc, "no-such-net", 0.0, 5.0, 0.6, 0.3).unwrap_err();
        assert!(err.contains("unknown net"), "unexpected error: {err}");
    }

    #[test]
    fn add_via_touching_its_own_net_succeeds_and_adds_it_to_the_node() {
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        place_part(&mut doc, &templates, &templates[0].name.clone(), -10.0, 0.0, 0.0).unwrap();
        place_part(&mut doc, &templates, &templates[0].name.clone(), 10.0, 0.0, 0.0).unwrap();
        let net_name = connect_pins(&mut doc, &templates, "P1", "1", "P2", "1").unwrap();

        // Right on top of P1's own pin "1" pad (at -10.0 - 1.27 = -11.27mm,
        // 0.0mm) -- geometrically legal (same net, so never a collision)
        // *and* actually touching the net it's meant to stitch, unlike
        // the dangling case covered below.
        let id = add_via(&mut doc, &net_name, -11.27, 0.0, 0.6, 0.3).expect("a via touching its own net's pad must be accepted");
        match doc.node.get(id) {
            Some(alladin_core::Item::Via { .. }) => {}
            other => panic!("expected a via, got {other:?}"),
        }
    }

    #[test]
    fn add_via_surfaces_a_rejected_off_board_placement() {
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        place_part(&mut doc, &templates, &templates[0].name.clone(), -10.0, 0.0, 0.0).unwrap();
        place_part(&mut doc, &templates, &templates[0].name.clone(), 10.0, 0.0, 0.0).unwrap();
        let net_name = connect_pins(&mut doc, &templates, "P1", "1", "P2", "1").unwrap();

        let err = add_via(&mut doc, &net_name, 1000.0, 1000.0, 0.6, 0.3).unwrap_err();
        assert!(err.contains("couldn't place a via"), "unexpected error: {err}");
    }

    #[test]
    fn add_via_rejects_a_dangling_via_that_touches_nothing_on_its_own_net() {
        // Same board/net as the two tests above, but the via itself sits
        // in open space, comfortably on-board and colliding with
        // nothing -- exactly the case `try_add_via` alone can't catch,
        // see `BoardDoc::try_add_stitching_via`'s doc comment. This is
        // what the old version of this test (before this fix) used to
        // call "open space must accept a via" and treat as success.
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        place_part(&mut doc, &templates, &templates[0].name.clone(), -10.0, 0.0, 0.0).unwrap();
        place_part(&mut doc, &templates, &templates[0].name.clone(), 10.0, 0.0, 0.0).unwrap();
        let net_name = connect_pins(&mut doc, &templates, "P1", "1", "P2", "1").unwrap();

        let err = add_via(&mut doc, &net_name, 0.0, 10.0, 0.6, 0.3).unwrap_err();
        assert!(err.contains("wouldn't touch"), "unexpected error: {err}");
    }

    #[test]
    fn route_pins_rejects_pins_that_are_not_yet_connected() {
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        place_part(&mut doc, &templates, &templates[0].name.clone(), -10.0, 0.0, 0.0).unwrap();
        place_part(&mut doc, &templates, &templates[0].name.clone(), 10.0, 0.0, 0.0).unwrap();

        let err = route_pins(&mut doc, &templates, "P1", "1", "P2", "1").unwrap_err();
        assert!(err.contains("couldn't route"), "unexpected error: {err}");
    }

    #[test]
    fn route_pins_rejects_an_unknown_pin() {
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        place_part(&mut doc, &templates, &templates[0].name.clone(), 0.0, 0.0, 0.0).unwrap();

        let err = route_pins(&mut doc, &templates, "P1", "99", "P1", "1").unwrap_err();
        assert!(err.contains("no such pin"), "unexpected error: {err}");
    }

    #[test]
    fn route_pins_succeeds_and_returns_a_positive_segment_count() {
        let mut doc = test_board();
        let templates = vec![two_pin_template()];
        // Rotated 90 degrees so each footprint's own *second*,
        // unconnected pad lands off the straight horizontal line
        // between the two connected first pads, rather than sitting
        // exactly on it -- a colinear obstacle directly on the path
        // that (separately confirmed) makes the router detour via the
        // board's own corners instead of a small local swing, a real
        // quirk of the frozen `alladin-router` search for that specific
        // degenerate arrangement, not something `try_route_pads` itself
        // needs to work around.
        place_part(&mut doc, &templates, &templates[0].name.clone(), -10.0, 0.0, 90.0).unwrap();
        place_part(&mut doc, &templates, &templates[0].name.clone(), 10.0, 0.0, 90.0).unwrap();
        connect_pins(&mut doc, &templates, "P1", "1", "P2", "1").unwrap();
        let tracks_before = doc.node.iter().filter(|item| matches!(item, alladin_core::Item::Track { .. })).count();

        let segments = route_pins(&mut doc, &templates, "P1", "1", "P2", "1").expect("an open board must have a DRC-clear path");

        assert!(segments >= 1, "a successful route must commit at least one track leg");
        let tracks_after = doc.node.iter().filter(|item| matches!(item, alladin_core::Item::Track { .. })).count();
        assert_eq!(tracks_after - tracks_before, segments);
    }
}
