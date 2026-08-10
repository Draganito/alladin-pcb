//! Mini MCP (Model Context Protocol) surface for the running GUI.
//!
//! Transport only: a dedicated OS thread with its own tiny `tokio`
//! runtime, hosting an `rmcp` "Streamable HTTP" server on
//! `127.0.0.1` (loopback-only, no auth). Every `#[tool]` forwards a
//! [`McpQuery`] across an [`std::sync::mpsc::Sender`] to the UI thread
//! and awaits a JSON [`String`] on an embedded `tokio::sync::oneshot`
//! reply channel. `crate::app::handle_mcp_query` builds the answers.
//!
//! Tool surface -- read-only: `get_footprints`, `get_nets`,
//! `board_summary`, `list_parts`, `check_board`, `get_routing_scene`,
//! `probe_route`; write (require `--allow-ai-write`): `new_board`,
//! `download_lcsc_part`, `place_footprint`, `move_footprint`,
//! `remove_footprint`, `connect_pins`, `disconnect_pin`,
//! `add_pin_stitching_via`, `rename_net`, `save_board`, `commit_route`,
//! `ripup_wire`. Placement, netlist, and
//! copper-route writes run through the same DFM gates and undo history
//! as the GUI's own gestures. Zone fill stays in the GUI.

use std::sync::mpsc;
use std::time::Duration;

use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{CallToolResult, ContentBlock, ServerCapabilities, ServerInfo};
use rmcp::schemars;
use rmcp::transport::streamable_http_server::session::local::LocalSessionManager;
use rmcp::transport::streamable_http_server::{StreamableHttpServerConfig, StreamableHttpService};
use rmcp::{tool, tool_handler, tool_router, ErrorData as McpError, ServerHandler};
use tokio::sync::oneshot;

/// Fixed localhost port the embedded MCP server always binds to.
pub const PORT: u16 = 8642;

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct DownloadLcscPartArgs {
    /// e.g. `"C2040"`.
    pub lcsc_code: String,
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
pub struct SaveBoardArgs {
    /// Omit to save to the board's current file path (it must already
    /// have one -- i.e. it was opened from, or already saved to, a file).
    pub path: Option<String>,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct PlaceFootprintArgs {
    /// Template name exactly as `list_parts` reports it,
    /// e.g. `"CC0603KRX7R9BB104"` or `"SolderPadTH"`.
    pub template: String,
    /// Horizontal position in mm; the origin (0,0) is the board's
    /// center, +x right, +y down (same coordinates `get_footprints`
    /// reports).
    pub x_mm: f64,
    /// Vertical position in mm (see `x_mm`).
    pub y_mm: f64,
    /// Rotation in degrees; omit for 0.
    pub rotation_deg: Option<f64>,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct MoveFootprintArgs {
    /// Reference designator of the footprint to move, e.g. `"U3"`.
    pub reference: String,
    /// New position in mm, board-center origin (see `place_footprint`).
    pub x_mm: f64,
    pub y_mm: f64,
    /// New rotation in degrees; omit to keep the current rotation.
    pub rotation_deg: Option<f64>,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct RemoveFootprintArgs {
    /// Reference designator of the footprint to remove, e.g. `"C7"`.
    pub reference: String,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct DisconnectPinArgs {
    /// Reference designator of the pin's footprint, e.g. `"U1"`.
    pub reference: String,
    /// Pad number on that footprint, e.g. `"3"`.
    pub pin: String,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct AddPinStitchingViaArgs {
    /// Footprint reference for a single pin, e.g. `"U1"` -- give
    /// together with `pin`. Leave out to use `net` batch mode instead.
    pub reference: Option<String>,
    /// Pad number on that footprint, e.g. `"1"`.
    pub pin: Option<String>,
    /// Batch mode: stitch EVERY pad on this net (e.g. `"GND"`) that
    /// doesn't already have a same-net via right next to it. Mutually
    /// exclusive with reference+pin.
    pub net: Option<String>,
    /// Via outer diameter in mm; omit for the GUI's current default.
    pub via_diameter_mm: Option<f64>,
    /// Via drill in mm; omit for the GUI's current default.
    pub via_drill_mm: Option<f64>,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct RenameNetArgs {
    /// Current name of the net, exactly as `get_nets` reports it.
    pub net: String,
    /// New name, e.g. `"GND"` or `"5V"`.
    pub new_name: String,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct NewBoardArgs {
    pub width_mm: f64,
    pub height_mm: f64,
    /// Copper layers; only 2 is supported end-to-end today. Omit for 2.
    pub layer_count: Option<u8>,
    /// JLCPCB copper weight profile, 1 or 2 (oz). Omit for 1.
    pub copper_weight_oz: Option<u8>,
    /// Corner radius of the board outline in mm. Omit for 1.0.
    pub corner_radius_mm: Option<f64>,
    /// A board is already open: refuse unless this is `true`, so an AI
    /// can't silently discard the human's current work.
    pub replace_current: Option<bool>,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct ProbeRouteArgs {
    /// One or more route candidates to clearance-check in a single call.
    /// Each candidate: `{ "net": "GND", "width_mm"?: 0.25, "segments":
    /// [{ "layer": "FCu"|"BCu", "points_mm": [[x,y], ...] }],
    /// "vias_mm"?: [[x,y], ...] }` — vias required between multi-layer
    /// segments, each via at the shared junction point.
    pub candidates: Vec<serde_json::Value>,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct CommitRouteArgs {
    /// A single route candidate (same shape as one `probe_route`
    /// candidate). Re-validated with the same gates before commit.
    pub route: serde_json::Value,
}

#[derive(serde::Deserialize, schemars::JsonSchema)]
pub struct RipupWireArgs {
    /// Rip up the whole electrically-continuous wire nearest this point
    /// (mm, board-center origin). Ignored when `net` is set.
    pub x_mm: Option<f64>,
    pub y_mm: Option<f64>,
    /// When set, remove every track and via on this net (pads stay).
    pub net: Option<String>,
}

/// One pending request from an MCP tool call, waiting for
/// [`crate::app::PcbApp::ui`] to drain it and answer via `reply`.
pub enum McpQuery {
    /// Every net, with which `footprint.pin` pads sit on it.
    Nets { reply: oneshot::Sender<String> },
    /// Every placed footprint, with each pad's net.
    Footprints { reply: oneshot::Sender<String> },
    /// One-call working picture -- see `crate::app::board_summary_json`.
    BoardSummary { reply: oneshot::Sender<String> },
    /// Network fetch already happened on the MCP thread; UI inserts into
    /// the parts DB and refreshes templates.
    DownloadLcscPart {
        fetched: Result<crate::lcsc::FetchedPart, crate::lcsc::FetchError>,
        reply: oneshot::Sender<String>,
    },
    /// Joins two pins onto the same electrical net.
    ConnectPins { args: ConnectPinsArgs, reply: oneshot::Sender<String> },
    /// Saves the live board to disk.
    SaveBoard { args: SaveBoardArgs, reply: oneshot::Sender<String> },
    /// Every placeable template in the parts library.
    ListParts { reply: oneshot::Sender<String> },
    /// Routing-completeness + DFM verification report.
    CheckBoard { reply: oneshot::Sender<String> },
    /// Places a library template on the board (same DFM gates as the GUI).
    PlaceFootprint { args: PlaceFootprintArgs, reply: oneshot::Sender<String> },
    /// Moves/rotates an already-placed footprint.
    MoveFootprint { args: MoveFootprintArgs, reply: oneshot::Sender<String> },
    /// Removes a placed footprint (and its wires/pads).
    RemoveFootprint { args: RemoveFootprintArgs, reply: oneshot::Sender<String> },
    /// Takes one pin off its net.
    DisconnectPin { args: DisconnectPinArgs, reply: oneshot::Sender<String> },
    /// Via + stub right next to a pin (or every pad on a net), placed
    /// automatically like the GUI's "Add via near pin".
    AddPinStitchingVia { args: AddPinStitchingViaArgs, reply: oneshot::Sender<String> },
    /// Renames a net.
    RenameNet { args: RenameNetArgs, reply: oneshot::Sender<String> },
    /// Creates a fresh board and switches the GUI to it.
    NewBoard { args: NewBoardArgs, reply: oneshot::Sender<String> },
    /// Geometry + open copper bridges for AI routing.
    GetRoutingScene { reply: oneshot::Sender<String> },
    /// Batched clearance probe (same gates as the GUI preview).
    ProbeRoute { args: ProbeRouteArgs, reply: oneshot::Sender<String> },
    /// Commit a cleared polyline (+ optional vias) onto the live board.
    CommitRoute { args: CommitRouteArgs, reply: oneshot::Sender<String> },
    /// Remove a wire near a point, or all copper on a named net.
    RipupWire { args: RipupWireArgs, reply: oneshot::Sender<String> },
}

/// How long a `#[tool]` waits for the UI thread before reporting timeout.
const REPLY_TIMEOUT: Duration = Duration::from_secs(3);

/// Longer budget for `board_summary` (whole-board continuity sweep).
const SLOW_REPLY_TIMEOUT: Duration = Duration::from_secs(20);

#[derive(Clone)]
struct AlladinMcp {
    tx: mpsc::Sender<McpQuery>,
    allow_ai_write: bool,
}

impl AlladinMcp {
    fn new(tx: mpsc::Sender<McpQuery>, allow_ai_write: bool) -> Self {
        Self { tx, allow_ai_write }
    }

    async fn ask(&self, make: impl FnOnce(oneshot::Sender<String>) -> McpQuery) -> Result<CallToolResult, McpError> {
        self.ask_with_timeout(REPLY_TIMEOUT, make).await
    }

    async fn ask_with_timeout(&self, timeout: Duration, make: impl FnOnce(oneshot::Sender<String>) -> McpQuery) -> Result<CallToolResult, McpError> {
        let (reply, rx) = oneshot::channel();
        if self.tx.send(make(reply)).is_err() {
            return Ok(CallToolResult::success(vec![ContentBlock::text("error: the alladin-pcb GUI's editor thread is gone".to_string())]));
        }
        let text = match tokio::time::timeout(timeout, rx).await {
            Ok(Ok(json)) => json,
            Ok(Err(_)) => "error: the alladin-pcb GUI dropped the request without answering".to_string(),
            Err(_) => format!(
                "error: no reply within {}s (a modal dialog may be blocking the GUI thread)",
                timeout.as_secs()
            ),
        };
        Ok(CallToolResult::success(vec![ContentBlock::text(text)]))
    }

    fn require_write_access(&self) -> Option<Result<CallToolResult, McpError>> {
        if self.allow_ai_write {
            None
        } else {
            Some(Ok(CallToolResult::success(vec![ContentBlock::text(
                "error: write access is disabled for this alladin-pcb process -- relaunch it with --allow-ai-write to enable write tools (read-only tools are unaffected)".to_string(),
            )])))
        }
    }
}

#[tool_router]
impl AlladinMcp {
    #[tool(description = "Every net on the board: id, name, pin count, and which footprint.pin pads sit on it.")]
    async fn get_nets(&self) -> Result<CallToolResult, McpError> {
        self.ask(|reply| McpQuery::Nets { reply }).await
    }

    #[tool(description = "Every placed footprint: reference, template name, position/rotation, and each pad's net (or null if that pin isn't wired to anything).")]
    async fn get_footprints(&self) -> Result<CallToolResult, McpError> {
        self.ask(|reply| McpQuery::Footprints { reply }).await
    }

    #[tool(
        description = "One-call working picture of the live board -- call this FIRST on any task, and again after each batch of changes. Returns board overview (size/layers/copper weight/item counts), key DFM numbers, and what is still unfinished: pins not assigned to any net, and nets whose copper is not yet physically one piece."
    )]
    async fn board_summary(&self) -> Result<CallToolResult, McpError> {
        self.ask_with_timeout(SLOW_REPLY_TIMEOUT, |reply| McpQuery::BoardSummary { reply }).await
    }

    #[tool(description = "Downloads a part from LCSC/EasyEDA by its C-number (with full pad geometry) and saves it to your local parts database. Network fetch happens here, off the GUI thread, so it never blocks the UI.")]
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

    #[tool(description = "Joins two pins (by footprint reference + pad number, e.g. ref1=\"P1\" pin1=\"1\") onto the same electrical net on the live board. Returns the net's name.")]
    async fn connect_pins(&self, Parameters(args): Parameters<ConnectPinsArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::ConnectPins { args, reply }).await
    }

    #[tool(description = "Saves the live board to disk. Omit path to save to its current file (it must already have one); give path to Save As.")]
    async fn save_board(&self, Parameters(args): Parameters<SaveBoardArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::SaveBoard { args, reply }).await
    }

    #[tool(description = "Every placeable template in the parts library: name (the exact string place_footprint wants), reference prefix, pad count, body size in mm, category, and description. Call this before place_footprint if you aren't sure of a name.")]
    async fn list_parts(&self) -> Result<CallToolResult, McpError> {
        self.ask(|reply| McpQuery::ListParts { reply }).await
    }

    #[tool(
        description = "Verification report for the live board -- call it after a batch of changes to check your own work. Returns ok=true only when every pin sits on a net, every net's copper is physically one piece, and no zone fill is stale. Also lists report-only DFM findings per placed template. Placement/clearance/edge rules need no scan here: every write path already enforces them."
    )]
    async fn check_board(&self) -> Result<CallToolResult, McpError> {
        self.ask_with_timeout(SLOW_REPLY_TIMEOUT, |reply| McpQuery::CheckBoard { reply }).await
    }

    #[tool(
        description = "Places a parts-library template on the live board at x/y mm (board-center origin, +x right, +y down). The same JLCPCB DFM gates as the GUI apply: board edge distance, pad/body clearance, hole spacing -- a refusal names the violated rule; pick a different spot and retry. Returns the auto-assigned reference (e.g. \"U17\"). The human can undo this with Ctrl+Z."
    )]
    async fn place_footprint(&self, Parameters(args): Parameters<PlaceFootprintArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::PlaceFootprint { args, reply }).await
    }

    #[tool(description = "Moves (and optionally rotates) an already-placed footprint by reference. Same DFM gates as place_footprint; on refusal the part stays where it was.")]
    async fn move_footprint(&self, Parameters(args): Parameters<MoveFootprintArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::MoveFootprint { args, reply }).await
    }

    #[tool(description = "Removes a placed footprint by reference, along with its pads, holes, and any wires ending on them. Nets left without pads are pruned. The human can undo this with Ctrl+Z.")]
    async fn remove_footprint(&self, Parameters(args): Parameters<RemoveFootprintArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::RemoveFootprint { args, reply }).await
    }

    #[tool(description = "Takes one pin (footprint reference + pad number) off whatever net it is on -- connect_pins' undo, for fixing a mis-wire. A net left with no pads disappears.")]
    async fn disconnect_pin(&self, Parameters(args): Parameters<DisconnectPinArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::DisconnectPin { args, reply }).await
    }

    #[tool(
        description = "Places a stitching via with a short connecting stub right next to a pin, automatically at the same spot the GUI's right-click \"Add via near pin\" picks (radially away from the part, sweeping to nearby angles if the natural spot is blocked) -- no coordinates needed. Single pin: reference+pin. Batch: net=\"GND\" stitches every pad on that net that doesn't already have a same-net via next to it, and reports placed/skipped/failed per pad. Same DFM gates and Ctrl+Z undo as the GUI."
    )]
    async fn add_pin_stitching_via(&self, Parameters(args): Parameters<AddPinStitchingViaArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask_with_timeout(SLOW_REPLY_TIMEOUT, |reply| McpQuery::AddPinStitchingVia { args, reply }).await
    }

    #[tool(description = "Renames a net (give its current name exactly as get_nets reports it). Use real names like \"5V\", \"GND\", \"DATA\" so exports and the GUI read well.")]
    async fn rename_net(&self, Parameters(args): Parameters<RenameNetArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::RenameNet { args, reply }).await
    }

    #[tool(
        description = "Creates a fresh empty board of width x height mm and switches the GUI to it. If a board is already open this refuses unless replace_current=true -- ask the human first; their unsaved work would be gone (undo history does not survive the switch)."
    )]
    async fn new_board(&self, Parameters(args): Parameters<NewBoardArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::NewBoard { args, reply }).await
    }

    #[tool(
        description = "Routing scene for laying copper like the GUI's manual 45° router: every pad (ref/pin/net/x/y/layer), existing tracks and vias, open_bridges (shortest pad-to-pad links between copper islands that still need joining, sorted by distance), and default width/via/clearance rules. Call this before probe_route / commit_route. Not an autorouter — you propose polylines."
    )]
    async fn get_routing_scene(&self) -> Result<CallToolResult, McpError> {
        self.ask_with_timeout(SLOW_REPLY_TIMEOUT, |reply| McpQuery::GetRoutingScene { reply }).await
    }

    #[tool(
        description = "Batched clearance probe for proposed copper routes — same gates as the GUI's green/red live preview (path clearance + board-edge margin + via DFM). Pass candidates: [{net, width_mm?, segments:[{layer:FCu|BCu, points_mm:[[x,y],...]}], vias_mm?:[[x,y],...]}]. Multi-layer routes need one via per junction (via point = last point of segment i and first of segment i+1). Returns results[] per candidate: ok, or blocked with the exact leg (segment_index, leg_index, leg_mm) and colliding[] — kind/net/footprint/layer/position of up to 3 items in the way, so you can route around them. Does not mutate the board."
    )]
    async fn probe_route(&self, Parameters(args): Parameters<ProbeRouteArgs>) -> Result<CallToolResult, McpError> {
        self.ask_with_timeout(SLOW_REPLY_TIMEOUT, |reply| McpQuery::ProbeRoute { args, reply }).await
    }

    #[tool(
        description = "Commits one copper route (same candidate shape as probe_route) onto the live board after re-running the same clearance/via gates, then verifies connectivity: the route must actually join the net's copper islands (bridge_closed=true, copper_pieces_before/after in the reply). A clean-looking route that lands in free space or on the wrong layer is rolled back and refused — no false positives. On refusal nothing is written and the error names the gate. Use after a successful probe_route. Ctrl+Z undoes. Zone fill stays in the GUI."
    )]
    async fn commit_route(&self, Parameters(args): Parameters<CommitRouteArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::CommitRoute { args, reply }).await
    }

    #[tool(
        description = "Rip up routed copper: either pass net=\"GND\" to remove every track/via on that net (pads stay), or pass x_mm+y_mm to delete the whole electrically-continuous wire nearest that point. Ctrl+Z undoes."
    )]
    async fn ripup_wire(&self, Parameters(args): Parameters<RipupWireArgs>) -> Result<CallToolResult, McpError> {
        if let Some(refusal) = self.require_write_access() {
            return refusal;
        }
        self.ask(|reply| McpQuery::RipupWire { args, reply }).await
    }
}

#[tool_handler]
impl ServerHandler for AlladinMcp {
    fn get_info(&self) -> ServerInfo {
        let write_note = if self.allow_ai_write {
            "Write tools are ENABLED for this process (launched with --allow-ai-write): \
             new_board, download_lcsc_part, place/move/remove_footprint, connect_pins, \
             disconnect_pin, add_pin_stitching_via, rename_net, save_board, commit_route, and \
             ripup_wire act directly on the live board/parts DB."
        } else {
            "Write tools (new_board, download_lcsc_part, place/move/remove_footprint, connect_pins, \
             disconnect_pin, add_pin_stitching_via, rename_net, save_board, commit_route, \
             ripup_wire) are DISABLED for \
             this process -- every one of them will refuse with an explanation. Relaunch \
             alladin-pcb with --allow-ai-write to enable them."
        };
        ServerInfo::new(ServerCapabilities::builder().enable_tools().build()).with_instructions(format!(
            "MCP surface for a running alladin-pcb GUI: board setup, parts download + placement, \
             netlist wiring, manual-style copper routing (get_routing_scene / probe_route / \
             commit_route — same 45°-clearance gates as the GUI, not an autorouter), and \
             self-verification (check_board), with read-only supports (get_footprints, get_nets, \
             board_summary, list_parts). Writes run through the same JLCPCB DFM gates and Ctrl+Z \
             undo history as the human's own GUI gestures. Zone fill stays in the GUI. {write_note}"
        ))
    }
}

/// Starts the embedded MCP server on a dedicated OS thread with its own
/// `tokio` runtime, bound to `127.0.0.1:`[`PORT`]. Never fails the
/// caller: a bind failure is logged to stderr and the thread exits.
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
