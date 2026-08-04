//! A read-only MCP (Model Context Protocol) server, embedded directly in
//! the running GUI process and always listening on a fixed localhost
//! port whenever `alladin-pcb`'s GUI is open. Exists purely to close a
//! blind spot the file-based `crate::cli` can never see: things like
//! the active [`crate::app`]`::Tool`, an in-progress route/zone/via
//! session, or the current selection are *transient, in-memory*
//! `EditorState` fields -- never written to the board's `.json` file --
//! so no amount of CLI/file introspection can ever answer "what is the
//! GUI actually doing right now?". An AI assistant driving this editor
//! needs exactly that, instead of reverse-engineering it from
//! screenshots and source code.
//!
//! This module only owns the *transport*: a dedicated OS thread with its
//! own tiny `tokio` runtime, hosting an `rmcp` "Streamable HTTP" server
//! bound to `127.0.0.1` (loopback-only, no auth -- a local dev
//! introspection aid, never meant to be reachable off-box). It knows
//! nothing about `BoardDoc`/`EditorState` themselves; every `#[tool]`
//! method below just forwards a [`McpQuery`] across an
//! [`std::sync::mpsc::Sender`] to the UI thread and awaits a JSON
//! [`String`] answer back through an embedded `tokio::sync::oneshot`
//! reply channel. `crate::app::handle_mcp_query` (drained once per frame
//! from `PcbApp::ui`) is where the actual "look at the live state and
//! build JSON" logic lives, since that's the only place with access to
//! `EditorState`'s private fields in the first place.
//!
//! Only JSON *strings* ever cross the thread boundary -- `BoardDoc`/
//! `EditorState` never need to be `Send`/`Sync`, and the UI thread stays
//! their sole owner and reader, exactly like today.
//!
//! **Write tools** (place a part, download from LCSC, connect pins,
//! route, add a via/zone, save, export) mirror `crate::cli`'s own
//! mutating subcommands 1:1, but act on the *live, currently open*
//! `EditorState`/`BoardDoc` directly rather than "load a file, mutate,
//! save it back" -- so a change lands immediately in the running GUI,
//! no reload round-trip needed. They're gated behind the
//! `--allow-ai-write` startup flag (see `main.rs`, threaded through
//! [`spawn_server`]'s `allow_ai_write` parameter): every write tool
//! checks it first and, if disabled, refuses with a plain-text
//! explanation instead of ever touching the board -- the read-only
//! tools above are completely unaffected by this flag.

use std::sync::mpsc;
use std::time::Duration;

use alladin_geom::MM;

use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{CallToolResult, ContentBlock, ServerCapabilities, ServerInfo};
use rmcp::schemars;
use rmcp::transport::streamable_http_server::session::local::LocalSessionManager;
use rmcp::transport::streamable_http_server::{StreamableHttpServerConfig, StreamableHttpService};
use rmcp::{tool, tool_handler, tool_router, ErrorData as McpError, ServerHandler};
use tokio::sync::oneshot;

/// The fixed localhost port the embedded MCP server always binds to --
/// "always on" per the design decision, so a client just points at a
/// known address instead of discovering a port some other way.
pub const PORT: u16 = 8642;

fn default_reference_prefix() -> String {
    "U".to_string()
}
fn default_corner_radius_mm() -> f64 {
    1.0
}
fn default_pitch_mm() -> f64 {
    2.54
}
fn default_pad_radius_mm() -> f64 {
    0.45
}
fn default_via_diameter_mm() -> f64 {
    0.6
}
fn default_via_drill_mm() -> f64 {
    0.3
}
fn default_stub_width_mm() -> f64 {
    0.25
}
fn default_trace_width_mm() -> f64 {
    0.25
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct CreateBoardArgs {
    pub width_mm: f64,
    pub height_mm: f64,
    /// 1 or 2.
    pub layers: u8,
    /// 1 or 2 (oz/ft²) -- picks which real JLCPCB DFM/clearance rules this
    /// board enforces for its whole lifetime. 2oz needs wider track
    /// spacing (0.16mm vs 1oz's 0.10mm) but carries more current.
    #[serde(default = "default_copper_weight_oz")]
    pub copper_weight_oz: u8,
    #[serde(default = "default_corner_radius_mm")]
    pub corner_radius_mm: f64,
}

fn default_copper_weight_oz() -> u8 {
    1
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct PlaceFootprintArgs {
    /// An exact template name from `get_footprints`/the GUI's parts
    /// panel (built-in or from your parts database).
    pub template: String,
    pub x_mm: f64,
    pub y_mm: f64,
    #[serde(default)]
    pub rotation_deg: f64,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct DownloadLcscPartArgs {
    /// e.g. `"C2040"`.
    pub lcsc_code: String,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct RegisterPartArgs {
    pub name: String,
    #[serde(default = "default_reference_prefix")]
    pub reference_prefix: String,
    #[serde(default)]
    pub description: String,
    /// A straight row of this many through-hole pads, evenly spaced by
    /// `pitch_mm` -- give exactly one of this or `hole_diameter_mm`.
    pub pin_count: Option<u32>,
    #[serde(default = "default_pitch_mm")]
    pub pitch_mm: f64,
    #[serde(default = "default_pad_radius_mm")]
    pub pad_radius_mm: f64,
    /// A pure mechanical, unplated mounting hole of this drill diameter
    /// -- no copper, no net. Give exactly one of this or `pin_count`.
    pub hole_diameter_mm: Option<f64>,
    #[serde(default)]
    pub exclude_from_bom: bool,
    /// Where this part shows up in the GUI's "Place part" category
    /// tree, e.g. `"Custom"` -- blank (the default) files it under
    /// "Uncategorized", same as every hand-registered part before this
    /// field existed.
    #[serde(default)]
    pub category: String,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct CheckNetContinuityArgs {
    /// Restrict the check to one net (exact name, e.g. `"GND"`). Omit to
    /// check every net with more than one pad at once.
    #[serde(default)]
    pub net_name: Option<String>,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct ConnectPinsArgs {
    /// Reference designator of the first pin's footprint, e.g. `"P1"`.
    pub ref1: String,
    /// Pad number on that footprint, e.g. `"1"`.
    pub pin1: String,
    pub ref2: String,
    pub pin2: String,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct RoutePinsArgs {
    pub ref1: String,
    pub pin1: String,
    pub ref2: String,
    pub pin2: String,
    /// Copper width for the resulting track(s), in millimetres.
    /// Defaults to `alladin-pcb`'s own [`crate::routing::DEFAULT_TRACE_WIDTH`]
    /// (0.25mm) when omitted.
    #[serde(default = "default_trace_width_mm")]
    pub width_mm: f64,
}

/// Starts a manual, human-style drag from one pin -- see
/// [`AlladinMcp::start_route`]'s tool description for how this differs
/// from [`RoutePinsArgs`]'s single-shot pathfinding auto-route.
#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct StartRouteArgs {
    /// The starting pin's footprint reference, e.g. `"P1"`.
    pub reference: String,
    /// The pad number on that footprint, e.g. `"1"`.
    pub pin: String,
    /// Copper width for the track this drag lays down, in millimetres.
    /// Defaults to 0.25mm when omitted.
    #[serde(default = "default_trace_width_mm")]
    pub width_mm: f64,
    /// Diameter/drill for any via a later `drop_via_and_switch_layer`
    /// call during this same drag places. Defaults to 0.6mm/0.3mm when
    /// omitted, matching [`AddViaArgs`]'s own defaults.
    #[serde(default = "default_via_diameter_mm")]
    pub via_diameter_mm: f64,
    #[serde(default = "default_via_drill_mm")]
    pub via_drill_mm: f64,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct RouteToArgs {
    pub x_mm: f64,
    pub y_mm: f64,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct AddViaArgs {
    /// An already-existing net's name (from a prior `connect_pins` call).
    pub net: String,
    pub x_mm: f64,
    pub y_mm: f64,
    #[serde(default = "default_via_diameter_mm")]
    pub diameter_mm: f64,
    #[serde(default = "default_via_drill_mm")]
    pub drill_mm: f64,
}

/// Places a stitching via right next to an already-connected pin,
/// pointing away from its own footprint's body, plus the short stub
/// track that connects it -- the AI-driven equivalent of the GUI's
/// right-click "Add via near pin" menu (see
/// [`crate::board_doc::BoardDoc::try_add_pin_stitching_via`]'s own doc
/// comment for the exact placement rule). Unlike that menu, a refused
/// natural spot is reported back as a plain error here rather than
/// starting an interactive drag -- an MCP call has no cursor to steer
/// with; on refusal, move the part with `place_footprint`'s
/// already-placed-footprint move path (there isn't one -- see the
/// tool description note) or simply try a different pin, then retry.
#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct AddPinStitchingViaArgs {
    /// The pin's footprint reference, e.g. `"U1"`.
    pub reference: String,
    /// The pad number on that footprint, e.g. `"3"`.
    pub pin: String,
    #[serde(default = "default_via_diameter_mm")]
    pub diameter_mm: f64,
    #[serde(default = "default_via_drill_mm")]
    pub drill_mm: f64,
    #[serde(default = "default_stub_width_mm")]
    pub stub_width_mm: f64,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct PointMmArg {
    pub x_mm: f64,
    pub y_mm: f64,
}

fn default_silk_rotation_deg() -> f64 {
    0.0
}

fn default_silk_height_mm() -> f64 {
    crate::board_doc::DEFAULT_SILK_TEXT_HEIGHT as f64 / MM as f64
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct AddSilkTextArgs {
    /// The annotation's own text, e.g. `"REV A"` or `"+"`. Refused if
    /// empty/whitespace-only.
    pub text: String,
    pub x_mm: f64,
    pub y_mm: f64,
    /// Degrees, counter-clockwise, `0` = horizontal/upright. Any value
    /// is accepted (not just multiples of 90 -- see this tool's own
    /// description).
    #[serde(default = "default_silk_rotation_deg")]
    pub rotation_deg: f64,
    /// `"front"` (F.SilkS, over F.Cu pads) or `"back"` (B.SilkS, over
    /// B.Cu pads).
    pub layer: String,
    /// Character height in mm, default 1.0mm -- the same
    /// `SILK_TEXT_HEIGHT_STEPS_MM` sizes the GUI's own size stepper
    /// offers (1.0/1.5/2.0/2.5/3.0mm) work well here too, but any
    /// positive value is accepted. A bigger height means a bigger
    /// collision rectangle, so a placement that fits at the default
    /// size can still be refused at a much larger one.
    #[serde(default = "default_silk_height_mm")]
    pub height_mm: f64,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct AddZoneArgs {
    /// An already-existing net's name.
    pub net: String,
    /// `"front"` (F.Cu) or `"back"` (B.Cu).
    pub layer: String,
    /// The zone outline's corners, in board millimetres.
    pub points: Vec<PointMmArg>,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct SaveBoardArgs {
    /// Omit to save to the board's current file path (it must already
    /// have one -- i.e. it was opened from, or already saved to, a file).
    pub path: Option<String>,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct RenameNetArgs {
    /// The net's current name (as shown by `get_nets`, e.g. an
    /// auto-generated `"Net3"` -- or an already-renamed net's current
    /// human name, to rename it again).
    pub net: String,
    /// The new name, e.g. `"GND"`/`"5V"`. Must be non-empty (after
    /// trimming whitespace) and not already used by a *different* net.
    pub new_name: String,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct ExportManufacturingFilesArgs {
    /// Directory to write every manufacturing file into (created if it
    /// doesn't exist yet). Gets the complete JLCPCB SMT set:
    /// `<stem>_gerbers.zip` (Gerbers + Excellon), `<stem>_cpl.csv`, and
    /// `<stem>_bom.csv` -- `<stem>` is the board's own save filename if
    /// it has one, else `"board"`. Native export; no KiCad required.
    pub out_dir: String,
}

/// [`AlladinMcp::start_external_autoroute`]'s arguments.
#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct StartExternalAutorouteArgs {
    /// Exact net names (as shown by `get_nets`) to route. Omit or leave
    /// empty to route every net with more than one pad -- the same
    /// default the GUI's Autoroute (extern) dialog pre-selects.
    #[serde(default)]
    pub nets: Vec<String>,
    /// Overrides the persisted Autoroute (extern) settings' own
    /// free-text "extra arguments" field for this one run only (never
    /// written back to `external_router.json`) -- e.g. `"--bus"` for a
    /// single one-off call. Omit to use whatever's currently configured
    /// in the settings window.
    pub extra_args: Option<String>,
}

/// One operation inside a [`run_batch`][AlladinMcp::run_batch] call --
/// the exact same typed arguments its own single-call tool takes (e.g.
/// [`PlaceFootprintArgs`]), just wrapped in a `{"tool": "...", "args": {...}}`
/// envelope so a whole heterogeneous sequence of them can live in one
/// JSON array. `download_lcsc_part` is deliberately not included: its
/// network fetch runs on the MCP thread itself, before anything ever
/// reaches [`McpQuery`]/the UI thread (see
/// [`AlladinMcp::download_lcsc_part`]), so it can't be folded into
/// `crate::app::run_batch_write`'s single synchronous dispatch loop the
/// way every operation below can. Call it standalone first, then batch
/// the `place_footprint` calls against the template name it registers.
#[derive(serde::Deserialize, schemars::JsonSchema)]
#[serde(tag = "tool", content = "args", rename_all = "snake_case")]
pub enum BatchOp {
    CreateBoard(CreateBoardArgs),
    PlaceFootprint(PlaceFootprintArgs),
    RegisterPart(RegisterPartArgs),
    ConnectPins(ConnectPinsArgs),
    RoutePins(RoutePinsArgs),
    StartRoute(StartRouteArgs),
    RouteTo(RouteToArgs),
    FixCorner,
    UndoLastCorner,
    FinishRoute,
    CancelRoute,
    DropViaAndSwitchLayer,
    AddVia(AddViaArgs),
    AddPinStitchingVia(AddPinStitchingViaArgs),
    AddZone(AddZoneArgs),
    AddSilkText(AddSilkTextArgs),
    RefillZones,
    RenameNet(RenameNetArgs),
    SaveBoard(SaveBoardArgs),
    ExportManufacturingFiles(ExportManufacturingFilesArgs),
}

fn default_stop_on_error() -> bool {
    true
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct RunBatchArgs {
    /// Run in order, all in a single round-trip.
    pub operations: Vec<BatchOp>,
    /// Stop at the first operation whose own result would contain an
    /// `"error"` key (the default) -- every later operation is then
    /// reported back as `{"skipped": true}` instead of being attempted.
    /// Set to `false` to run every operation regardless of earlier
    /// failures and collect all of their individual results.
    #[serde(default = "default_stop_on_error")]
    pub stop_on_error: bool,
}

/// One pending request from an MCP tool call, waiting for
/// [`crate::app::PcbApp::ui`] to drain it (once per frame, see that
/// function's top) and answer via the embedded `reply` channel.
pub enum McpQuery {
    /// The live editor/tool state -- see `crate::app::editor_state_json`.
    EditorState { reply: oneshot::Sender<String> },
    /// File path, board size/layers, and item counts.
    BoardOverview { reply: oneshot::Sender<String> },
    /// Every net, with which `footprint.pin` pads sit on it.
    Nets { reply: oneshot::Sender<String> },
    /// Every copper zone/pour, mirroring the CLI's `list-zones`.
    Zones { reply: oneshot::Sender<String> },
    /// Every placed footprint, with each pad's net.
    Footprints { reply: oneshot::Sender<String> },
    /// Physical copper-continuity check -- see
    /// `crate::app::net_continuity_json`.
    CheckNetContinuity { args: CheckNetContinuityArgs, reply: oneshot::Sender<String> },
    /// Creates a brand-new board and switches the GUI over to it --
    /// only while [`crate::app::Screen::NewBoard`] is showing (never
    /// silently discards an already-open board).
    CreateBoard { args: CreateBoardArgs, reply: oneshot::Sender<String> },
    /// Places a template instance on the live board.
    PlaceFootprint { args: PlaceFootprintArgs, reply: oneshot::Sender<String> },
    /// The network fetch already happened on the MCP thread (see
    /// [`AlladinMcp::download_lcsc_part`]) -- this just inserts the
    /// result into the parts database and refreshes the live template
    /// list, exactly like the GUI's own background-download handler.
    DownloadLcscPart { fetched: Result<crate::lcsc::FetchedPart, crate::lcsc::FetchError>, reply: oneshot::Sender<String> },
    /// Registers a new hand-described part into the parts database.
    RegisterPart { args: RegisterPartArgs, reply: oneshot::Sender<String> },
    /// Joins two pins onto the same electrical net.
    ConnectPins { args: ConnectPinsArgs, reply: oneshot::Sender<String> },
    /// Auto-routes a straight-to-DRC-clear trace between two
    /// already-connected, same-layer pins.
    RoutePins { args: RoutePinsArgs, reply: oneshot::Sender<String> },
    /// Starts a manual, human-style routing drag from one pin -- see
    /// [`AlladinMcp::start_route`]'s tool description.
    StartRoute { args: StartRouteArgs, reply: oneshot::Sender<String> },
    /// Steers the live end of the in-progress drag towards a point --
    /// the MCP equivalent of moving the mouse.
    RouteTo { args: RouteToArgs, reply: oneshot::Sender<String> },
    /// Fixes the current live leg as a permanent corner and starts a
    /// fresh one from there -- the MCP equivalent of pressing Space.
    FixCorner { reply: oneshot::Sender<String> },
    /// Un-fixes the most recently fixed corner -- the MCP equivalent of
    /// pressing Backspace.
    UndoLastCorner { reply: oneshot::Sender<String> },
    /// Commits the whole drag as real track(s), provided the live end
    /// is currently docked onto a same-net target pin (call `route_to`
    /// with that pin's position first) -- the MCP equivalent of
    /// clicking the target pin.
    FinishRoute { reply: oneshot::Sender<String> },
    /// Abandons the in-progress drag without touching the board -- the
    /// MCP equivalent of pressing Escape.
    CancelRoute { reply: oneshot::Sender<String> },
    /// Drops a via at the drag's current live end, commits every leg up
    /// to it, and continues the drag on the other copper layer -- the
    /// MCP equivalent of pressing V mid-drag.
    DropViaAndSwitchLayer { reply: oneshot::Sender<String> },
    /// Places a stitching via.
    AddVia { args: AddViaArgs, reply: oneshot::Sender<String> },
    /// Places a stitching via right next to an already-connected pin,
    /// plus the stub track connecting it -- see [`AddPinStitchingViaArgs`]'s
    /// own doc comment.
    AddPinStitchingVia { args: AddPinStitchingViaArgs, reply: oneshot::Sender<String> },
    /// Draws and fills a new copper zone/pour.
    AddZone { args: AddZoneArgs, reply: oneshot::Sender<String> },
    /// Places a free-standing silkscreen text annotation -- see
    /// [`AlladinMcp::add_silk_text`]'s tool description.
    AddSilkText { args: AddSilkTextArgs, reply: oneshot::Sender<String> },
    /// Re-runs every zone's fill against the board's current state.
    RefillZones { reply: oneshot::Sender<String> },
    /// Gives an existing net a human name (e.g. `"Net3"` -> `"GND"`).
    RenameNet { args: RenameNetArgs, reply: oneshot::Sender<String> },
    /// Saves the live board to disk.
    SaveBoard { args: SaveBoardArgs, reply: oneshot::Sender<String> },
    /// Exports Gerber zip + CPL + BOM (native writer) -- see
    /// [`AlladinMcp::export_manufacturing_files`]'s tool description.
    ExportManufacturingFiles { args: ExportManufacturingFilesArgs, reply: oneshot::Sender<String> },
    /// Runs a whole sequence of write operations against the live
    /// board in one UI-thread pass -- see
    /// [`AlladinMcp::run_batch`]'s tool description.
    RunBatch { args: RunBatchArgs, reply: oneshot::Sender<String> },
    /// Starts the optional external KiCadRoutingTools autorouter (see
    /// `crate::external_router`) as a background subprocess -- see
    /// [`AlladinMcp::start_external_autoroute`]'s tool description.
    /// Deliberately not part of [`RunBatch`]/[`BatchOp`]: like
    /// `download_lcsc_part`, it doesn't finish within one synchronous
    /// UI-thread pass, so it can't be folded into that dispatch loop.
    StartExternalAutoroute { args: StartExternalAutorouteArgs, reply: oneshot::Sender<String> },
    /// Reports the currently running/last-finished external-autoroute
    /// job started by [`Self::StartExternalAutoroute`] -- see
    /// [`AlladinMcp::get_external_autoroute_status`]'s tool description.
    GetExternalAutorouteStatus { reply: oneshot::Sender<String> },
}

impl McpQuery {
    /// Whether this query can mutate the live board/screen in any way
    /// (i.e. every write tool) -- see `crate::app::PcbApp::pending_job`'s
    /// own doc comment for why *every* write query (not just the
    /// handful that actually run their own work on a background
    /// thread) is refused with a busy message while a background job
    /// is in flight: it's the only way to guarantee nothing else ever
    /// mutates `Screen` between when a background job's snapshot was
    /// taken and when its result gets merged back into the live one.
    /// The read-only introspection queries below are completely
    /// unaffected by any of this and always run immediately.
    pub fn is_write(&self) -> bool {
        !matches!(
            self,
            McpQuery::EditorState { .. }
                | McpQuery::BoardOverview { .. }
                | McpQuery::Nets { .. }
                | McpQuery::Zones { .. }
                | McpQuery::Footprints { .. }
                | McpQuery::GetExternalAutorouteStatus { .. }
        )
    }

    /// Answers this query's own reply channel directly with `text`,
    /// bypassing `crate::app::handle_mcp_query`/`Screen`/`PartsDb`
    /// entirely -- used by `crate::app::PcbApp::ui` to report "busy"
    /// immediately for a write query that arrives while
    /// `PcbApp::pending_job` is still running.
    pub fn reply_now(self, text: String) {
        use McpQuery::*;
        let reply = match self {
            EditorState { reply }
            | BoardOverview { reply }
            | Nets { reply }
            | Zones { reply }
            | Footprints { reply }
            | CheckNetContinuity { reply, .. }
            | CreateBoard { reply, .. }
            | PlaceFootprint { reply, .. }
            | DownloadLcscPart { reply, .. }
            | RegisterPart { reply, .. }
            | ConnectPins { reply, .. }
            | RoutePins { reply, .. }
            | StartRoute { reply, .. }
            | RouteTo { reply, .. }
            | FixCorner { reply }
            | UndoLastCorner { reply }
            | FinishRoute { reply }
            | CancelRoute { reply }
            | DropViaAndSwitchLayer { reply }
            | AddVia { reply, .. }
            | AddPinStitchingVia { reply, .. }
            | AddZone { reply, .. }
            | AddSilkText { reply, .. }
            | RefillZones { reply }
            | RenameNet { reply, .. }
            | SaveBoard { reply, .. }
            | ExportManufacturingFiles { reply, .. }
            | RunBatch { reply, .. }
            | StartExternalAutoroute { reply, .. }
            | GetExternalAutorouteStatus { reply } => reply,
        };
        let _ = reply.send(text);
    }
}

/// How long a `#[tool]` method waits for the UI thread to answer before
/// giving up and reporting a timeout instead of hanging the MCP client
/// forever -- covers the (rare) case where the UI thread is blocked on
/// something like a native "Open file" modal dialog.
const REPLY_TIMEOUT: Duration = Duration::from_secs(3);

/// [`AlladinMcp::ask_with_timeout`]'s budget for tools whose underlying
/// `BoardDoc` operation is a real geometric search rather than a simple
/// state check -- `route_pins` (pathfinding across whatever's already
/// on the board), `add_zone`/`refill_zones` (a full zone-fill pass).
/// [`REPLY_TIMEOUT`]'s 3s turned out to be routinely too short for
/// `route_pins` on a board with more than a couple of obstacles in the
/// way: the search itself was fine (it always finished, and the track
/// really did land -- confirmed by polling `get_board_overview`
/// afterwards), it just legitimately took a few seconds longer than
/// that, so the *only* thing that was actually broken was this
/// timeout reporting a false "didn't respond" error over a slow-but-
/// genuinely-still-working search.
const SLOW_REPLY_TIMEOUT: Duration = Duration::from_secs(20);

/// [`AlladinMcp::run_batch`]'s own timeout budget. A batch runs as one
/// single, synchronous pass on the UI thread before it replies at all
/// (see `crate::app::run_batch_write`) -- a 60-operation batch mixing
/// in several `route_pins`/`add_zone` calls can legitimately take far
/// longer than [`REPLY_TIMEOUT`] or even [`SLOW_REPLY_TIMEOUT`] alone,
/// so the budget has to scale with how many operations were actually
/// requested rather than being one fixed constant. Scales at 2s/op,
/// floored at [`SLOW_REPLY_TIMEOUT`] (so even a tiny batch gets at
/// least as much slack as a single slow operation would on its own)
/// and capped at 5 minutes as a last-resort safety net against a truly
/// pathological operation hanging the UI thread forever.
fn batch_timeout(op_count: usize) -> Duration {
    let scaled = Duration::from_secs(2).saturating_mul(op_count.max(1) as u32);
    scaled.clamp(SLOW_REPLY_TIMEOUT, Duration::from_secs(300))
}

/// The `rmcp` tool handler. Deliberately tiny and stateless beyond the
/// one `mpsc::Sender` -- `StreamableHttpService` constructs a fresh one
/// per request in the stateless mode this server runs in (see
/// [`spawn_server`]), so nothing here can accumulate state across calls
/// anyway.
#[derive(Clone)]
struct AlladinMcp {
    tx: mpsc::Sender<McpQuery>,
    /// Whether this process was launched with `--allow-ai-write` (see
    /// `main.rs`) -- fixed for the process's whole lifetime, checked by
    /// every write tool below before it does anything else. Cheap to
    /// carry into every per-request [`AlladinMcp`] clone since it's
    /// just a `bool`.
    allow_ai_write: bool,
}

impl AlladinMcp {
    fn new(tx: mpsc::Sender<McpQuery>, allow_ai_write: bool) -> Self {
        Self { tx, allow_ai_write }
    }

    /// Sends whatever [`McpQuery`] `make` builds (once handed a fresh
    /// reply channel) to the UI thread and waits for its JSON answer --
    /// degrading to a plain-text explanation, never a hard MCP error,
    /// if the UI thread is gone, drops the request, or is simply slow
    /// right now. Same "never fail the caller over a transient hiccup"
    /// convention `reload_from_disk` already uses elsewhere in this
    /// codebase.
    async fn ask(&self, make: impl FnOnce(oneshot::Sender<String>) -> McpQuery) -> Result<CallToolResult, McpError> {
        self.ask_with_timeout(REPLY_TIMEOUT, make).await
    }

    /// [`Self::ask`], but with an explicit timeout -- see
    /// [`SLOW_REPLY_TIMEOUT`]'s doc comment for why `route_pins`/
    /// `add_zone`/`refill_zones` need a longer one than everything else.
    async fn ask_with_timeout(&self, timeout: Duration, make: impl FnOnce(oneshot::Sender<String>) -> McpQuery) -> Result<CallToolResult, McpError> {
        let (reply, rx) = oneshot::channel();
        if self.tx.send(make(reply)).is_err() {
            return Ok(CallToolResult::success(vec![ContentBlock::text("error: the alladin-pcb GUI's editor thread is gone".to_string())]));
        }
        let text = match tokio::time::timeout(timeout, rx).await {
            Ok(Ok(json)) => json,
            Ok(Err(_)) => "error: the alladin-pcb GUI dropped the request without answering".to_string(),
            Err(_) => format!("error: the alladin-pcb GUI didn't respond within {}s (maybe a modal dialog is open right now)", timeout.as_secs()),
        };
        Ok(CallToolResult::success(vec![ContentBlock::text(text)]))
    }

    /// `Some(refusal)` if this process wasn't launched with
    /// `--allow-ai-write` -- every write tool below calls this first and
    /// returns immediately if it's `Some`, *before* sending anything to
    /// the UI thread, so a disabled write tool never touches the board
    /// at all, not even to read it.
    fn require_write_access(&self) -> Option<Result<CallToolResult, McpError>> {
        if self.allow_ai_write {
            None
        } else {
            Some(Ok(CallToolResult::success(vec![ContentBlock::text(
                "error: write access is disabled for this alladin-pcb process -- relaunch it with --allow-ai-write to enable place/connect/route/save tools (read-only tools are unaffected)".to_string(),
            )])))
        }
    }
}

#[tool_router]
impl AlladinMcp {
    #[tool(
        description = "The live alladin-pcb editor state right now: which tool is active, any in-progress route/zone/via session, the current selection, hover position, and pending status messages. This is the one thing the board's .json file and the CLI can never show -- it's all transient, in-memory UI state."
    )]
    async fn get_editor_state(&self) -> Result<CallToolResult, McpError> {
        self.ask(|reply| McpQuery::EditorState { reply }).await
    }

    #[tool(description = "Board overview: file path, board width/height/layer count, and item counts (nets, footprints, tracks, vias, zones).")]
    async fn get_board_overview(&self) -> Result<CallToolResult, McpError> {
        self.ask(|reply| McpQuery::BoardOverview { reply }).await
    }

    #[tool(description = "Every net on the board: id, name, pin count, and which footprint.pin pads sit on it.")]
    async fn get_nets(&self) -> Result<CallToolResult, McpError> {
        self.ask(|reply| McpQuery::Nets { reply }).await
    }

    #[tool(description = "Every copper zone/pour on the board: id, net, layer, outline point count, and current filled island count.")]
    async fn get_zones(&self) -> Result<CallToolResult, McpError> {
        self.ask(|reply| McpQuery::Zones { reply }).await
    }

    #[tool(description = "Every placed footprint: reference, template name, position/rotation, and each pad's net (or null if that pin isn't wired to anything).")]
    async fn get_footprints(&self) -> Result<CallToolResult, McpError> {
        self.ask(|reply| McpQuery::Footprints { reply }).await
    }

    #[tool(
        description = "Checks whether each net's copper (pads + tracks + vias + zone/pour islands) is actually physically continuous, not just logically declared. get_nets only reports which pads a net's name was assigned to (e.g. via connect_pins/rename_net) -- it says nothing about whether copper actually reaches every one of them. A fragmented zone/pour fill (get_zones' filled_islands > 1) is the most common real cause of a gap: some pads end up sitting on an island the rest of the net's copper never actually touches, even though every pad still shows up under the same net name. Without net_name, reports a summary across every net with more than one pad plus full island/pad detail for any net that isn't fully connected; with net_name, always reports that one net's full breakdown, connected or not. Run this after any batch of connect_pins/add_zone/add_via/route_pins work on a net before trusting it's actually wired end to end."
    )]
    async fn check_net_continuity(&self, Parameters(args): Parameters<CheckNetContinuityArgs>) -> Result<CallToolResult, McpError> {
        self.ask(|reply| McpQuery::CheckNetContinuity { args, reply }).await
    }

    // -- Write tools below: all gated by `--allow-ai-write` (see
    // `require_write_access`), all acting on the live, currently open
    // board rather than a file path. --

    #[tool(
        description = "Creates a brand-new board (width/height in mm, 1/2 layers, 1/2 oz copper weight, rounded-rect corner radius in mm) and opens it in the GUI. Copper weight picks which real JLCPCB DFM/clearance rules the board enforces for its whole lifetime (2oz needs wider track spacing but carries more current). Only works while no board is open yet (the 'New board' screen) -- won't silently discard an already-open board."
    )]
    async fn create_board(&self, Parameters(args): Parameters<CreateBoardArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::CreateBoard { args, reply }).await
    }

    #[tool(description = "Places an instance of an existing footprint template (see get_footprints/list-templates) on the live board, at (x_mm, y_mm), optionally rotated. Returns the new part's auto-generated reference designator.")]
    async fn place_footprint(&self, Parameters(args): Parameters<PlaceFootprintArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::PlaceFootprint { args, reply }).await
    }

    #[tool(description = "Downloads a part from LCSC/EasyEDA by its C-number (with full pad geometry) and saves it to your local parts database, ready for place_footprint. Network fetch happens here, off the GUI thread, so it never blocks the UI.")]
    async fn download_lcsc_part(&self, Parameters(args): Parameters<DownloadLcscPartArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        let rx = crate::lcsc::fetch_in_background(args.lcsc_code);
        let fetched = match tokio::task::spawn_blocking(move || rx.recv()).await {
            Ok(Ok(result)) => result,
            Ok(Err(_)) | Err(_) => {
                return Ok(CallToolResult::success(vec![ContentBlock::text("error: the LCSC download thread ended unexpectedly".to_string())]));
            }
        };
        self.ask(|reply| McpQuery::DownloadLcscPart { fetched, reply }).await
    }

    #[tool(
        description = "Registers a hand-described part into your local parts database, without needing an LCSC part -- give exactly one of pin_count (a straight row of through-hole pads, e.g. a header or resistor) or hole_diameter_mm (a pure mechanical mounting hole)."
    )]
    async fn register_part(&self, Parameters(args): Parameters<RegisterPartArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::RegisterPart { args, reply }).await
    }

    #[tool(description = "Joins two pins (by footprint reference + pad number, e.g. ref1=\"P1\" pin1=\"1\") onto the same electrical net on the live board. Returns the net's name.")]
    async fn connect_pins(&self, Parameters(args): Parameters<ConnectPinsArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::ConnectPins { args, reply }).await
    }

    #[tool(
        description = "Auto-routes a straight-to-DRC-clear copper trace between two pins that already share a net (run connect_pins first) and sit on the same copper layer -- no automatic via/layer-hop insertion. Optional width_mm sets the track's copper width (default 0.25mm)."
    )]
    async fn route_pins(&self, Parameters(args): Parameters<RoutePinsArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask_with_timeout(SLOW_REPLY_TIMEOUT, |reply| McpQuery::RoutePins { args, reply }).await
    }

    #[tool(
        description = "Starts a manual, human-style routing drag from one pin -- the interactive drag-a-trace mechanic a human steers with the mouse (walkaround/shove against whatever's already on the board, snapped to clean 45-degree angles), NOT route_pins' single-shot pathfinding search. Use this instead of route_pins on a busy/complex board: it can never trigger a pathological, board-wide search, because it never runs one -- each step is one cheap, local check, exactly as fast as a human dragging the mouse one frame at a time. After this, call route_to repeatedly (steering towards where you want the trace to go, calling fix_corner whenever you want to lock in a bend) and finish once route_to lands on/near the same-net target pin (finish_route then commits it). cancel_route abandons the whole drag without touching the board. Refuses if this pin has no net yet (connect_pins first) or if a route is already in progress (finish_route or cancel_route it first). Optional width_mm sets this drag's track width (default 0.25mm); via_diameter_mm/via_drill_mm (defaults 0.6mm/0.3mm) set the via a later drop_via_and_switch_layer on this same drag would place."
    )]
    async fn start_route(&self, Parameters(args): Parameters<StartRouteArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::StartRoute { args, reply }).await
    }

    #[tool(
        description = "Steers the in-progress routing drag's live end towards (x_mm, y_mm) -- the MCP equivalent of moving the mouse during a manual drag (see start_route). Always succeeds as a call; the response's live_end_clear/blocked_reason tell you whether the resulting leg is actually usable right now (fix_corner requires it clear; landing on/near the same-net target pin sets hover_target/preview, which finish_route then needs)."
    )]
    async fn route_to(&self, Parameters(args): Parameters<RouteToArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::RouteTo { args, reply }).await
    }

    #[tool(
        description = "Fixes the routing drag's current live leg as a permanent corner and starts a fresh live leg from there -- the MCP equivalent of pressing Space mid-drag. Refuses if the current leg is blocked, hasn't moved from the last fixed point yet, or the live end is currently docked onto a pad (finish_route instead)."
    )]
    async fn fix_corner(&self) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::FixCorner { reply }).await
    }

    #[tool(description = "Un-fixes the routing drag's most recently fixed corner -- the MCP equivalent of pressing Backspace mid-drag. Refuses if no corner has been fixed yet.")]
    async fn undo_last_corner(&self) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::UndoLastCorner { reply }).await
    }

    #[tool(
        description = "Commits the in-progress routing drag as real track(s) on the board -- the MCP equivalent of clicking the target pin to finish a manual drag. Requires the most recent route_to call to have landed on/near a same-net pin other than the start one; refuses (leaving the drag alive so you can keep steering) otherwise."
    )]
    async fn finish_route(&self) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::FinishRoute { reply }).await
    }

    #[tool(description = "Abandons the in-progress routing drag without touching the board -- the MCP equivalent of pressing Escape mid-drag.")]
    async fn cancel_route(&self) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::CancelRoute { reply }).await
    }

    #[tool(
        description = "Drops a via at the routing drag's current live end (free-steered leg, or a docked pad if route_to landed on one), commits every fixed corner plus every leg up to that via as real track(s), then continues the SAME drag on the other copper layer (F.Cu<->B.Cu) from there, with every fixed corner cleared -- the MCP equivalent of pressing V mid-drag, for through-hole layer changes mid-trace. Does not require being docked onto a pad first (unlike finish_route) -- dropping a via to continue across open space is the whole point. Refuses if there's no usable live leg right now (route_to somewhere first) or if the via itself would be refused there (e.g. it would collide with something already at that exact point)."
    )]
    async fn drop_via_and_switch_layer(&self) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::DropViaAndSwitchLayer { reply }).await
    }

    #[tool(description = "Places a via (e.g. a GND stitching via) at (x_mm, y_mm) on an already-existing net. Refused if it wouldn't actually touch any existing copper on that net (a dangling via).")]
    async fn add_via(&self, Parameters(args): Parameters<AddViaArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::AddVia { args, reply }).await
    }

    #[tool(
        description = "Places a stitching via right next to an already-connected pin (e.g. a GND via next to a decoupling cap's GND pad), automatically positioned along the radial direction away from that pin's own footprint body, plus the short stub track that connects the two -- the same natural spot a human routing engineer would reach for by hand, and the AI-driven equivalent of the GUI's right-click \"Add via near pin\" menu. The pin must already be on a net (connect_pins first). Refused (touching nothing) if that natural spot doesn't actually work right now -- too close to the board edge, or something else already occupies it/the short stub's path; an MCP call has no cursor to nudge it with like the GUI does, so on refusal either place the part further from other copper first, or just try a different pin."
    )]
    async fn add_pin_stitching_via(&self, Parameters(args): Parameters<AddPinStitchingViaArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::AddPinStitchingVia { args, reply }).await
    }

    #[tool(
        description = "Draws and fills a new copper zone/pour on an already-existing net -- give its outline as a list of {x_mm, y_mm} points and which layer (\"front\"=F.Cu or \"back\"=B.Cu)."
    )]
    async fn add_zone(&self, Parameters(args): Parameters<AddZoneArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask_with_timeout(SLOW_REPLY_TIMEOUT, |reply| McpQuery::AddZone { args, reply }).await
    }

    #[tool(
        description = "Places a free-standing silkscreen text annotation (a project title, polarity mark, warning label, revision string, ...) -- distinct from a footprint's own auto-placed reference/value labels. `height_mm` defaults to 1.0mm; any positive value works, but a bigger one grows the collision rectangle. Refused (touching nothing) if it would print over a same-side copper pad (JLCPCB's real 0.15mm silk-to-pad clearance), overlap another already-placed silk text, or leave the board/hug its edge -- move it further from other copper/text/the edge and retry, shrink it, or place it on the other side (\"front\"/\"back\") if that side is less crowded."
    )]
    async fn add_silk_text(&self, Parameters(args): Parameters<AddSilkTextArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::AddSilkText { args, reply }).await
    }

    #[tool(
        description = "Re-runs every zone's fill against the board's current state -- needed after new tracks/vias/parts were added since a zone was drawn, since a fill is a point-in-time snapshot that can go stale."
    )]
    async fn refill_zones(&self) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask_with_timeout(SLOW_REPLY_TIMEOUT, |reply| McpQuery::RefillZones { reply }).await
    }

    #[tool(
        description = "Renames an existing net (found by its current name, e.g. an auto-generated \"Net3\") to a human name like \"GND\" or \"5V\" -- shows up everywhere that net's name is used: get_nets, the GUI's net list/ratsnest labels, and the exported .kicad_pcb's net table. Fails if new_name is empty or already used by a different net."
    )]
    async fn rename_net(&self, Parameters(args): Parameters<RenameNetArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::RenameNet { args, reply }).await
    }

    #[tool(description = "Saves the live board to disk. Omit path to save to its current file (it must already have one); give path to Save As.")]
    async fn save_board(&self, Parameters(args): Parameters<SaveBoardArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::SaveBoard { args, reply }).await
    }

    #[tool(
        description = "Exports the complete JLCPCB SMT-assembly file set into out_dir using Alladin's native writer (no KiCad): <stem>_gerbers.zip (F/B Cu, Mask, Paste, Silk, Edge Cuts, PTH/NPTH), <stem>_cpl.csv (Designator,Mid X,Mid Y,Layer,Rotation), and <stem>_bom.csv (Comment,Designator,Footprint,LCSC Part #)."
    )]
    async fn export_manufacturing_files(&self, Parameters(args): Parameters<ExportManufacturingFilesArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask_with_timeout(SLOW_REPLY_TIMEOUT, |reply| McpQuery::ExportManufacturingFiles { args, reply }).await
    }

    #[tool(
        description = "Starts KiCadRoutingTools (github.com/drandyhaas/KiCadRoutingTools) -- a separate, user-installed external autorouter this is an optional integration for, NOT part of alladin-pcb's own routing -- as a background subprocess against the live board, and replies immediately (a real run can take minutes on a busy board; poll get_external_autoroute_status for progress, don't wait on this call). Optional nets restricts it to those exact net names (see get_nets); omit/empty routes every net with more than one pad. Optional extra_args passes a one-off extra argument string straight into route.py's own argv (e.g. \"--bus\"), overriding -- for this single run only, never saved -- whatever's configured in the GUI's Autoroute (extern) settings window. Requires that settings window's tool folder to already point at a working local KiCadRoutingTools checkout (its own \"Diagnose\" button confirms this); refuses immediately, before spawning anything, if it isn't configured yet or route.py can't be found there. Once the job reports done, its routed tracks/vias are NOT merged onto the board automatically -- reviewing the DRC/connectivity result and merging (or discarding) is a deliberate manual step in the GUI's Autoroute (extern) dialog, by design; this tool only starts and reports on the run. Refuses if a job from an earlier call is still running."
    )]
    async fn start_external_autoroute(&self, Parameters(args): Parameters<StartExternalAutorouteArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::StartExternalAutoroute { args, reply }).await
    }

    #[tool(
        description = "Reports the external-autoroute job started by start_external_autoroute: status is one of \"idle\" (none started yet, or already discarded in the GUI), \"running\" (with the subprocess's log lines seen so far), \"done\" (requested/routed net names, DRC/connectivity check results if KiCadRoutingTools' own check_drc.py/check_connected.py are present, and how many track/via items are waiting to be merged in the GUI dialog), or \"failed\" (with the error message). Read-only -- safe to poll repeatedly regardless of --allow-ai-write."
    )]
    async fn get_external_autoroute_status(&self) -> Result<CallToolResult, McpError> {
        self.ask(|reply| McpQuery::GetExternalAutorouteStatus { reply }).await
    }

    #[tool(
        description = "Runs a whole sequence of write operations (create_board, place_footprint, register_part, connect_pins, route_pins, start_route/route_to/fix_corner/undo_last_corner/finish_route/cancel_route/drop_via_and_switch_layer, add_via, add_zone, refill_zones, rename_net, save_board, export_manufacturing_files) as ONE MCP call instead of one round-trip per operation -- this is the way to build up a whole board's placement/netlist/pour/routing without an AI client's token/context cost growing with the number of operations (every earlier call otherwise stays in the conversation and gets re-sent on every later one). A whole manual-style route (start_route, one or more route_to/fix_corner pairs, a final route_to onto the target, finish_route) batches just as well as any other operation sequence -- the in-progress drag persists across operations within the same batch exactly like it would across separate calls. NOT batchable: download_lcsc_part (its network fetch runs on the MCP server itself, off the UI thread) -- call it standalone first, then batch place_footprint calls against the template name it returns. Give operations as a list of {\"tool\": \"<name>\", \"args\": {...}} objects, e.g. {\"tool\": \"place_footprint\", \"args\": {\"template\": \"...\", \"x_mm\": 1.0, \"y_mm\": 2.0}} (fix_corner/undo_last_corner/finish_route/cancel_route/refill_zones take no args at all, e.g. {\"tool\": \"fix_corner\"}). The response is one JSON object with ok/ok_count/error_count/stopped_early plus a \"results\" array holding each operation's own result, in the exact shape its single-call tool would return (an \"error\" key on failure), tagged with its index and tool name. By default (stop_on_error=true) the batch stops at the first failing operation and reports every later one as {\"skipped\": true}; set stop_on_error=false to run all of them regardless and collect every individual result."
    )]
    async fn run_batch(&self, Parameters(args): Parameters<RunBatchArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        let timeout = batch_timeout(args.operations.len());
        self.ask_with_timeout(timeout, |reply| McpQuery::RunBatch { args, reply }).await
    }
}

#[tool_handler]
impl ServerHandler for AlladinMcp {
    fn get_info(&self) -> ServerInfo {
        let write_note = if self.allow_ai_write {
            "Write tools are ENABLED for this process (launched with --allow-ai-write): create_board, place_footprint, \
             download_lcsc_part, register_part, connect_pins, route_pins, start_route/route_to/fix_corner/undo_last_corner/ \
             finish_route/cancel_route/drop_via_and_switch_layer, add_via, add_zone, refill_zones, rename_net, save_board, \
             export_manufacturing_files, start_external_autoroute all act directly on the live board and take effect immediately in the GUI \
             (start_external_autoroute only starts a background job -- poll get_external_autoroute_status for its progress; merging its result \
             onto the board is a manual step in the GUI's Autoroute (extern) dialog, not something these MCP tools do on their own). On a busy/complex board, prefer the \
             start_route/route_to/... family over route_pins: it's the same manual, human-steered drag-a-trace mechanic \
             (walkaround/shove, cheap local checks) the GUI's mouse dragging uses, instead of route_pins' single-shot \
             pathfinding search, which can be slow (or, on a sufficiently obstructed board, pathological) the busier the \
             board gets. Building up a whole board (many placements/connections/routes/etc.)? Prefer run_batch over \
             individually calling each one -- one round-trip instead of N keeps your own token/context cost from growing \
             with the board's size."
        } else {
            "Write tools exist (create_board, place_footprint, connect_pins, route_pins, start_route/route_to/..., \
             add_via, add_zone, rename_net, save_board, export_manufacturing_files, run_batch, ...) but are DISABLED for this process \
             -- every one of them will refuse with an explanation. Relaunch alladin-pcb with --allow-ai-write to enable \
             them."
        };
        ServerInfo::new(ServerCapabilities::builder().enable_tools().build()).with_instructions(format!(
            "Introspection into a running alladin-pcb GUI: the live editor/tool state (active tool, an in-progress \
             route/zone/via session, selection, hover position -- none of which is ever written to the board file) \
             and the board's own contents (nets, zones, footprints). Use it to see what's actually on-screen right \
             now instead of guessing from the board file or a screenshot. {write_note}"
        ))
    }
}

/// Starts the embedded MCP server on a dedicated OS thread with its own
/// `tokio` runtime, bound to `127.0.0.1:`[`PORT`]. Never fails the
/// caller: a bind failure (most likely a *second* `alladin-pcb` GUI
/// already holding the port) is logged to stderr and the thread simply
/// exits, leaving the rest of the app completely unaffected -- same
/// "degrade gracefully, never hard-fail" convention
/// `crate::app::load_templates` already uses for a broken parts
/// database. `tx` is the sending half of the channel whose receiving
/// half [`crate::app::PcbApp`] drains once per frame; cloned once per
/// incoming HTTP request (see the `service_factory` closure below) since
/// this server runs in `rmcp`'s stateless mode -- a fresh [`AlladinMcp`]
/// per request, never relying on in-memory state surviving between
/// requests.
pub fn spawn_server(tx: mpsc::Sender<McpQuery>, port: u16, allow_ai_write: bool) {
    std::thread::spawn(move || {
        let runtime = match tokio::runtime::Builder::new_multi_thread().enable_all().build() {
            Ok(runtime) => runtime,
            Err(e) => {
                eprintln!("alladin-pcb: MCP server disabled, couldn't start its tokio runtime: {e}");
                return;
            }
        };
        runtime.block_on(async move {
            let addr = format!("127.0.0.1:{port}");
            let listener = match tokio::net::TcpListener::bind(&addr).await {
                Ok(listener) => listener,
                Err(e) => {
                    eprintln!("alladin-pcb: MCP server disabled, couldn't bind {addr} (maybe another alladin-pcb GUI is already running): {e}");
                    return;
                }
            };
            let service =
                StreamableHttpService::new(move || Ok(AlladinMcp::new(tx.clone(), allow_ai_write)), LocalSessionManager::default().into(), StreamableHttpServerConfig::default());
            let router = axum::Router::new().nest_service("/mcp", service);
            eprintln!(
                "alladin-pcb: MCP server listening on http://{addr}/mcp (write tools {})",
                if allow_ai_write { "ENABLED" } else { "disabled -- pass --allow-ai-write to enable" }
            );
            let _ = axum::serve(listener, router).await;
        });
    });
}
