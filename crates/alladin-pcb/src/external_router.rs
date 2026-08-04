//! Optional, purely additive integration with
//! [KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools)
//! (drandyhaas) -- a third-party Python/Rust autorouter this crate
//! shells out to as a subprocess -- wrap a real external tool rather
//! than embed/FFI its internals.
//!
//! **Why a subprocess, not a library/FFI dependency**: the explicit
//! requirement behind this module is "never have to touch Alladin's
//! own code again just because the external tool got updated". A
//! vendored/FFI dependency would tie this crate to that tool's own
//! internal Rust/Python API surface, which *does* change across
//! releases; its command-line interface is the one thing its own
//! README documents as stable across "months" of development
//! (`--nets`/`--track-width`/`--via-size`/`--via-drill`/
//! `--overwrite`, positional `<input> <output>`). This module depends
//! on exactly that small, documented contract, plus one deliberate
//! escape hatch for everything else: [`ExternalRouterSettings::extra_args`]
//! is a free-text passthrough appended verbatim to `route.py`'s own
//! argv, so a future flag (`--bus`, `--ordering mps`, ...) becomes
//! usable the moment a user types it in, with zero code changes here.
//!
//! **What this module does NOT do**: it never modifies
//! `crate::board_doc::BoardDoc`, `crate::routing`, or any of Alladin's
//! own collision/DRC code. Every existing routing feature (interactive
//! walkaround/shove dragging, `route_pins`' pathfinding, zone fills)
//! stays completely untouched -- this is a wholly separate, optional
//! path a user opts into per the plan's explicit "additiv, kein
//! Eingriff" requirement. It reuses exactly two already-existing,
//! unmodified building blocks: [`crate::kicad_export::export_kicad_files`]
//! to hand the external tool a board, and [`crate::kicad_import::import_kicad_pcb`]
//! to read its answer back.
//!
//! Three pieces, in the order a caller actually uses them:
//! - [`ExternalRouterSettings`]: the tool's install location plus
//!   routing parameters, persisted as its own small JSON file (see
//!   [`ExternalRouterSettings::load`]/[`Self::save`]) -- deliberately
//!   separate from `crate::persistence`'s board-file format, since
//!   this is process/machine-local configuration, not board data.
//! - [`diagnose`]: read-only environment checks (Python found? script
//!   found? `numpy`/`scipy`/`shapely` importable?) a settings dialog
//!   can show as a checklist before ever attempting a real run. Never
//!   installs or changes anything on its own -- see its own doc
//!   comment for why that's a deliberate, non-negotiable property.
//! - [`run_autoroute`]: spawns `route.py` on a background thread (this
//!   can take real minutes on a busy board) and streams
//!   [`AutorouteEvent`]s back through a channel a UI polls once per
//!   frame -- the exact same "background thread + `Receiver` the GUI
//!   drains with `try_recv()`" shape `crate::lcsc::fetch_in_background`
//!   already established -- an autoroute pass over a real, busy net
//!   list is too slow for a blocking UI-thread call.

use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use alladin_core::{Item, NetId};
use alladin_geom::MM;

use crate::board_doc::BoardDoc;
use crate::footprint::FootprintTemplate;

/// Everything [`run_autoroute`] needs beyond the board/nets themselves
/// -- the tool's own install location plus the routing parameters
/// every `route.py` invocation passes through its small, stable flag
/// set (see this module's doc comment). Persisted verbatim as JSON
/// (see [`Self::load`]/[`Self::save`]) so a user only has to fill this
/// in once per machine, not once per board.
///
/// **Deliberately no `clearance_mm` field.** One used to exist here,
/// editable in the GUI from 0.05mm up, and was passed through as
/// `route.py --clearance`. Two real problems with that, not just a
/// UX wart: (1) its default was hardcoded to `2layer_1oz`'s
/// `PAD_TO_TRACK` (0.1mm) regardless of the actually-open board's own
/// `copper_weight`, so it silently under-stated the true 0.16mm
/// minimum on a 2oz board; (2) `route.py`'s `--clearance` is
/// documented as a *ceiling* -- the value actually used is
/// `min(the sibling .kicad_pro's own Default net-class clearance,
/// --clearance)` -- so that too-low default didn't just fail to widen
/// anything, it could silently *narrow* a 2oz board's real routing
/// clearance below its own manufacturable minimum. Since
/// `crate::kicad_export::export_kicad_files` already writes a fresh,
/// correctly copper-weight-aware `.kicad_pro` right before every
/// `route.py` invocation (see `crate::board_doc::BoardDoc::net_class_clearance`),
/// the actually-correct fix is to never pass `--clearance` at all --
/// `route.py` then falls back to reading that same `.kicad_pro`'s own
/// Default class itself, which is Alladin's single, only-ever-correct
/// source of truth for JLCPCB clearance. There is now no locally
/// editable clearance number anywhere in this module that could ever
/// drift from it.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ExternalRouterSettings {
    /// The cloned `KiCadRoutingTools` repo's root -- expected to
    /// contain `route.py` directly (and, optionally, `check_drc.py`/
    /// `check_connected.py`, see [`run_autoroute`]'s doc comment).
    /// Empty means "not configured yet" ([`diagnose`]/[`run_autoroute`]
    /// both treat that as [`ExternalRouterError::ToolNotConfigured`]
    /// rather than a confusing "file not found" a moment later).
    #[serde(default)]
    pub tool_dir: String,
    /// The Python interpreter to run `route.py` (and the verification
    /// scripts) with -- a plain command name (resolved against `PATH`)
    /// or an absolute path to a specific interpreter/venv.
    #[serde(default = "default_python_bin")]
    pub python_bin: String,
    #[serde(default = "default_track_width_mm")]
    pub track_width_mm: f64,
    #[serde(default = "default_via_diameter_mm")]
    pub via_diameter_mm: f64,
    #[serde(default = "default_via_drill_mm")]
    pub via_drill_mm: f64,
    /// Free-text, appended verbatim (after quote-aware whitespace
    /// splitting, see [`split_args`]) to `route.py`'s own argv -- the
    /// deliberate escape hatch this module's doc comment describes for
    /// every flag not already in this struct's own fixed fields.
    #[serde(default)]
    pub extra_args: String,
}

fn default_python_bin() -> String {
    "python3".to_string()
}
/// `.max(JlcpcbDfm::MIN_TRACK_WIDTH)` below isn't a no-op guard against
/// impossible input here (`DEFAULT_TRACE_WIDTH` is already comfortably
/// above the JLCPCB minimum) -- it's there so a value loaded back from
/// an *old* `external_router.json` (written before this floor existed,
/// or hand-edited) can never silently resurrect a sub-minimum default;
/// see this module's doc comment on why "JLCPCB's own minimum, never
/// user-overridable downward" is this whole module's guiding rule now,
/// not just for the clearance field that used to exist here.
fn default_track_width_mm() -> f64 {
    (crate::routing::DEFAULT_TRACE_WIDTH as f64 / MM as f64).max(alladin_core::JlcpcbDfm::MIN_TRACK_WIDTH as f64 / MM as f64)
}
fn default_via_diameter_mm() -> f64 {
    (crate::board_doc::DEFAULT_VIA_DIAMETER as f64 / MM as f64).max(alladin_core::JlcpcbDfm::MIN_VIA_DIAMETER as f64 / MM as f64)
}
fn default_via_drill_mm() -> f64 {
    (crate::board_doc::DEFAULT_VIA_DRILL as f64 / MM as f64).max(alladin_core::JlcpcbDfm::MIN_VIA_HOLE as f64 / MM as f64)
}

impl Default for ExternalRouterSettings {
    fn default() -> Self {
        Self {
            tool_dir: String::new(),
            python_bin: default_python_bin(),
            track_width_mm: default_track_width_mm(),
            via_diameter_mm: default_via_diameter_mm(),
            via_drill_mm: default_via_drill_mm(),
            extra_args: String::new(),
        }
    }
}

/// `dirs::config_dir()/alladin-pcb/external_router.json` -- a small,
/// standalone file deliberately outside `crate::persistence`'s own
/// board-save format (this is per-machine tool configuration, not
/// board data a `.json` board file should carry around). `None` only
/// on a platform `dirs::config_dir()` itself can't resolve at all
/// (undocumented/exotic target), in which case [`ExternalRouterSettings::load`]/
/// [`Self::save`] degrade to "in-memory only for this run" rather than
/// failing the caller.
fn settings_path() -> Option<PathBuf> {
    Some(dirs::config_dir()?.join("alladin-pcb").join("external_router.json"))
}

impl ExternalRouterSettings {
    /// Loads the persisted settings, or [`Self::default`] if none were
    /// ever saved, the file is unreadable, or it doesn't parse --
    /// same "degrade to a sane default rather than fail the caller"
    /// convention `crate::app::load_templates` already uses for a
    /// broken parts database. [`Self::clamp_to_jlcpcb_minimums`] runs
    /// on the result either way, so a value an *old* build's wider
    /// GUI slider range (or hand-editing) let below the real JLCPCB
    /// minimum before this module started enforcing it can never keep
    /// silently coming back on every future launch just because it's
    /// already sitting in the file -- `#[serde(default = "...")]` only
    /// ever helps a *missing* field, never an explicit too-low one.
    pub fn load() -> Self {
        let mut settings: Self = settings_path().and_then(|p| std::fs::read_to_string(p).ok()).and_then(|s| serde_json::from_str(&s).ok()).unwrap_or_default();
        settings.clamp_to_jlcpcb_minimums();
        settings
    }

    /// Raises `track_width_mm`/`via_diameter_mm`/`via_drill_mm` up to
    /// the real JLCPCB DFM minimum if they're currently below it --
    /// never lowers a legitimately wider, deliberately-chosen value
    /// (e.g. a wide power-net track for current capacity). There is no
    /// equivalent for clearance: this module no longer has a
    /// `clearance_mm` field to clamp at all -- see this struct's own
    /// doc comment for why the board's own JLCPCB net-class clearance
    /// (via `.kicad_pro`, `route.py`'s own fallback) is the only source
    /// now, not a locally-editable number that could ever drift from
    /// it.
    fn clamp_to_jlcpcb_minimums(&mut self) {
        self.track_width_mm = self.track_width_mm.max(alladin_core::JlcpcbDfm::MIN_TRACK_WIDTH as f64 / MM as f64);
        self.via_diameter_mm = self.via_diameter_mm.max(alladin_core::JlcpcbDfm::MIN_VIA_DIAMETER as f64 / MM as f64);
        self.via_drill_mm = self.via_drill_mm.max(alladin_core::JlcpcbDfm::MIN_VIA_HOLE as f64 / MM as f64);
    }

    pub fn save(&self) -> std::io::Result<()> {
        let path = settings_path().ok_or_else(|| std::io::Error::other("couldn't resolve a config directory on this platform"))?;
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, serde_json::to_string_pretty(self).expect("ExternalRouterSettings has no types that can fail to serialize"))
    }
}

/// Splits a free-text field into argv-style tokens, honouring
/// single/double-quoted substrings (`--foo "a b"` -> two tokens, not
/// three) -- enough for [`ExternalRouterSettings::extra_args`]'s own
/// job of feeding extra flags straight into `route.py`'s `argparse`,
/// deliberately not a full shell parser (no `$VAR` expansion,
/// backslash escapes, or globbing): this field is passed directly as
/// a subprocess's `argv`, never through an actual shell, so none of
/// that ever applies anyway.
pub fn split_args(input: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    let mut current = String::new();
    let mut in_token = false;
    let mut quote: Option<char> = None;
    for c in input.chars() {
        match quote {
            Some(q) => {
                if c == q {
                    quote = None;
                } else {
                    current.push(c);
                }
            }
            None => match c {
                '"' | '\'' => {
                    quote = Some(c);
                    in_token = true;
                }
                c if c.is_whitespace() => {
                    if in_token {
                        tokens.push(std::mem::take(&mut current));
                        in_token = false;
                    }
                }
                c => {
                    current.push(c);
                    in_token = true;
                }
            },
        }
    }
    if in_token {
        tokens.push(current);
    }
    tokens
}

/// Everything that can go wrong running the external autorouter --
/// same plain-data, `Display`-only shape every other error enum in
/// this crate uses (see e.g. `crate::native_gerber::NativeGerberError`),
/// not `alladin-kicad-io`'s `thiserror` convention.
#[derive(Debug)]
pub enum ExternalRouterError {
    /// [`ExternalRouterSettings::tool_dir`] is still empty -- the one
    /// failure mode a user must actually *act* on first (open the
    /// settings dialog and point it at a cloned `KiCadRoutingTools`
    /// checkout), so it gets its own variant instead of folding into
    /// [`Self::ScriptNotFound`] below.
    ToolNotConfigured,
    /// [`ExternalRouterSettings::python_bin`] isn't runnable at all.
    PythonNotFound,
    /// `tool_dir` is set, but `route.py` isn't sitting directly inside
    /// it.
    ScriptNotFound(PathBuf),
    /// `route.py` ran but exited non-zero.
    ProcessFailed { stderr: String },
    /// [`AutorouteHandle::cancel`] was called before the process
    /// finished on its own.
    Cancelled,
    Io(std::io::Error),
    /// `route.py` finished and wrote an output file, but
    /// `crate::kicad_import::import_kicad_pcb` couldn't parse it back.
    ImportFailed(String),
}

impl std::fmt::Display for ExternalRouterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExternalRouterError::ToolNotConfigured => {
                write!(f, "KiCadRoutingTools isn't configured yet -- open the Autoroute (extern) settings and set its folder (the one containing route.py)")
            }
            ExternalRouterError::PythonNotFound => {
                write!(f, "the configured Python binary wasn't found on PATH -- check the \"Python\" field in the Autoroute (extern) settings")
            }
            ExternalRouterError::ScriptNotFound(path) => {
                write!(f, "route.py not found at {} -- check the tool folder in the Autoroute (extern) settings", path.display())
            }
            ExternalRouterError::ProcessFailed { stderr } => write!(f, "route.py failed: {}", stderr.trim()),
            ExternalRouterError::Cancelled => write!(f, "cancelled"),
            ExternalRouterError::Io(e) => write!(f, "filesystem/process error: {e}"),
            ExternalRouterError::ImportFailed(e) => write!(f, "route.py finished, but its output couldn't be read back: {e}"),
        }
    }
}

impl From<std::io::Error> for ExternalRouterError {
    fn from(e: std::io::Error) -> Self {
        ExternalRouterError::Io(e)
    }
}

/// Read-only environment report [`diagnose`] builds -- every field is
/// a plain fact a settings dialog can render as its own checklist row
/// (green/red), never a side effect: nothing in this module ever
/// installs, downloads, or modifies anything on the user's system on
/// its own (matching the plan's explicit "kein automatischer git
/// clone/pip install durch Alladin" requirement) -- a missing
/// prerequisite is reported here so the *user* can act on it, not
/// silently worked around.
#[derive(Debug, Clone)]
pub struct DiagnoseReport {
    pub python_found: bool,
    pub python_version: Option<String>,
    pub script_found: bool,
    pub numpy_ok: bool,
    pub scipy_ok: bool,
    pub shapely_ok: bool,
    /// `route.py --help` actually running to completion -- the closest
    /// thing to "would a real invocation at least start" this can
    /// check without touching a real board.
    pub help_ok: bool,
}

impl DiagnoseReport {
    /// Whether every prerequisite [`run_autoroute`] actually needs is
    /// met -- deliberately excludes [`Self::help_ok`] (a real,
    /// confirmed-working install can still have a `route.py` whose
    /// `--help` output this check happens not to like, e.g. a future
    /// version printing to stderr instead of stdout; that's a soft
    /// signal for the checklist, not a hard gate).
    pub fn is_ready(&self) -> bool {
        self.python_found && self.script_found && self.numpy_ok && self.scipy_ok && self.shapely_ok
    }
}

fn python_importable(python_bin: &str, module: &str) -> bool {
    Command::new(python_bin).args(["-c", &format!("import {module}")]).output().map(|o| o.status.success()).unwrap_or(false)
}

/// Read-only checks against `settings` -- see [`DiagnoseReport`]'s own
/// doc comment for why this never changes anything on the system it
/// runs on, purely a checklist a settings dialog shows before the user
/// ever tries a real autoroute run.
pub fn diagnose(settings: &ExternalRouterSettings) -> DiagnoseReport {
    let version_output = Command::new(&settings.python_bin).arg("--version").output().ok().filter(|o| o.status.success());
    let python_version = version_output.map(|o| {
        // Python 2 (still occasionally the system default) prints
        // `--version`'s output to stderr rather than stdout; trying
        // both, stdout first, covers either without needing to know
        // which major version is actually installed ahead of time.
        let stdout = String::from_utf8_lossy(&o.stdout).trim().to_string();
        if stdout.is_empty() {
            String::from_utf8_lossy(&o.stderr).trim().to_string()
        } else {
            stdout
        }
    });
    let python_found = python_version.is_some();

    let script_path = Path::new(&settings.tool_dir).join("route.py");
    let script_found = script_path.is_file();

    let numpy_ok = python_found && python_importable(&settings.python_bin, "numpy");
    let scipy_ok = python_found && python_importable(&settings.python_bin, "scipy");
    let shapely_ok = python_found && python_importable(&settings.python_bin, "shapely");

    let help_ok = python_found
        && script_found
        && Command::new(&settings.python_bin).arg(&script_path).arg("--help").output().map(|o| o.status.success()).unwrap_or(false);

    DiagnoseReport { python_found, python_version, script_found, numpy_ok, scipy_ok, shapely_ok, help_ok }
}

/// One line of progress from a running [`run_autoroute`] job (raw
/// stdout/stderr from `route.py` or a verification script, interleaved
/// as it arrives -- exact ordering between the two streams isn't
/// preserved, matching how a terminal showing both at once would look
/// anyway), or the job's own final outcome.
pub enum AutorouteEvent {
    Log(String),
    /// Sent exactly once, always last.
    Done(Result<AutorouteReport, ExternalRouterError>),
}

/// [`run_autoroute`]'s successful outcome: the new track/via [`Item`]s
/// it found (already filtered to the requested nets and remapped onto
/// the *live* board's own [`NetId`]s, see [`filter_and_remap_routed_items`]
/// -- ready to hand straight to `alladin_core::Node::add`, one call per
/// item), plus enough of a report for a UI to show a real "X/Y nets
/// routed, DRC/connectivity check passed?" summary before the caller
/// decides whether to actually merge [`Self::items`] into the live
/// board.
pub struct AutorouteReport {
    pub items: Vec<Item>,
    pub requested_nets: Vec<String>,
    /// The subset of `requested_nets` that actually got at least one
    /// track/via back from this run -- `route.py` can legitimately
    /// leave a net unrouted (too congested, conflicting constraints,
    /// ...) without that being a process *failure* (exit code 0
    /// either way), so this is the real "did it work" signal, not the
    /// exit status.
    pub routed_nets: Vec<String>,
    /// `Some(true)`/`Some(false)` if `check_drc.py` exists in the tool
    /// folder and ran to completion, `None` if it's simply not present
    /// (an older/trimmed checkout) -- see [`run_autoroute`]'s own doc
    /// comment for why a missing verification script is never treated
    /// as a hard failure.
    pub drc_ok: Option<bool>,
    /// Same shape as `drc_ok`, for `check_connected.py`.
    pub connected_ok: Option<bool>,
}

/// The shared handle [`run_autoroute`] hands back: the event stream
/// plus a way to actually stop the child process mid-run --
/// deliberately two different `Arc<Mutex<...>>` cells (the live
/// [`Child`] handle, and a plain cancelled flag) rather than one,
/// since the flag has to be checked/set independently of whether the
/// background thread currently happens to hold the child lock (see
/// [`run_blocking`]'s polling loop for why a single lock covering both
/// would risk `cancel()` blocking until the very process it's trying
/// to kill exits on its own).
pub struct AutorouteHandle {
    pub events: Receiver<AutorouteEvent>,
    child: Arc<Mutex<Option<Child>>>,
    cancelled: Arc<AtomicBool>,
}

impl AutorouteHandle {
    /// Kills the running `route.py` (or verification script) process,
    /// if any is currently running, and marks this job as
    /// user-cancelled so its final [`AutorouteEvent::Done`] reports
    /// [`ExternalRouterError::Cancelled`] rather than a confusing
    /// "process failed" for what is, from the killed process's own
    /// point of view, indistinguishable from any other non-zero exit.
    /// A no-op once the job has already finished on its own.
    pub fn cancel(&self) {
        self.cancelled.store(true, Ordering::SeqCst);
        if let Ok(mut guard) = self.child.lock() {
            if let Some(child) = guard.as_mut() {
                let _ = child.kill();
            }
        }
    }
}

fn stream_lines(reader: impl std::io::Read, tx: &Sender<AutorouteEvent>) {
    for line in BufReader::new(reader).lines() {
        match line {
            Ok(line) => {
                if tx.send(AutorouteEvent::Log(line)).is_err() {
                    break; // receiver dropped -- nothing left to stream to
                }
            }
            Err(_) => break,
        }
    }
}

/// Runs `check_drc.py`/`check_connected.py` (whichever `script_name`
/// names) against `pcb_path`, if the tool checkout actually has it --
/// see [`AutorouteReport::drc_ok`]'s own doc comment for why a missing
/// script is `None`, not a failure. Its combined stdout+stderr is
/// streamed as one [`AutorouteEvent::Log`] (only if non-empty) so a
/// live dialog can show exactly what either check reported, not just
/// the pass/fail bit this returns.
fn run_verification_script(python_bin: &str, tool_dir: &Path, script_name: &str, pcb_path: &Path, tx: &Sender<AutorouteEvent>) -> Option<bool> {
    let script = tool_dir.join(script_name);
    if !script.is_file() {
        return None;
    }
    let output = Command::new(python_bin).arg(&script).arg(pcb_path).output().ok()?;
    let text = format!("{}{}", String::from_utf8_lossy(&output.stdout), String::from_utf8_lossy(&output.stderr));
    let trimmed = text.trim();
    if !trimmed.is_empty() {
        let _ = tx.send(AutorouteEvent::Log(format!("--- {script_name} ---\n{trimmed}")));
    }
    Some(output.status.success())
}

/// Filters `imported_node`'s `Item::Track`/`Item::Via` items down to
/// just those on a net named in `requested_net_names`, remapping each
/// kept item's [`NetId`] from the *imported file's own* numbering
/// (whatever `crate::kicad_export::export_kicad_pcb` happened to
/// assign when it wrote the temp file `route.py` worked from -- an
/// entirely separate numbering from the live, currently-open board's
/// own [`NetId`]s) onto the live board's matching net, via
/// `live_net_of` (in practice, `crate::board_doc::BoardDoc::find_net_by_name`
/// against a live-net-name snapshot taken when the job started -- see
/// [`run_autoroute`]). An item whose net name isn't found on the live
/// board at all (a net renamed or deleted mid-run) is silently
/// dropped rather than merged with no net -- a track that's supposed
/// to be on a specific net but silently landing on no net at all would
/// be a real, confusing correctness bug, far worse than just not
/// merging that one item.
///
/// Pure, `Node`-only logic with no subprocess/filesystem involvement
/// at all -- deliberately factored out of [`run_blocking`] so this
/// exact net-name filter/remap behavior is unit-testable without a
/// real `route.py` install (see this module's own tests).
pub fn filter_and_remap_routed_items(
    imported_node: &alladin_core::Node,
    imported_nets: &[(u32, String)],
    requested_net_names: &[String],
    live_net_of: impl Fn(&str) -> Option<NetId>,
) -> Vec<Item> {
    let mut out = Vec::new();
    for item in imported_node.iter() {
        if !matches!(item, Item::Track { .. } | Item::Via { .. }) {
            continue;
        }
        let Some(imported_net_id) = item.net() else { continue };
        let Some((_, name)) = imported_nets.iter().find(|(id, _)| *id == imported_net_id.0) else { continue };
        if !requested_net_names.iter().any(|n| n == name) {
            continue;
        }
        let Some(live_id) = live_net_of(name) else { continue };
        out.push(remap_net(item, live_id));
    }
    out
}

fn remap_net(item: &Item, new_net: NetId) -> Item {
    match item.clone() {
        Item::Track { shape, layer, class, .. } => Item::Track { shape, net: Some(new_net), layer, class },
        Item::Via { shape, drill, .. } => Item::Via { shape, drill, net: Some(new_net) },
        other => other,
    }
}

/// Every `route.py` flag [`run_blocking`] passes beyond the fixed
/// `<input> <output>` positionals, as plain strings -- split out into
/// its own pure function (rather than built inline against a
/// [`Command`]) so this exact argument list is unit-testable without
/// ever spawning a real process, matching this module's own
/// [`filter_and_remap_routed_items`]-style "testable pure core, thin
/// I/O shell around it" convention. **Deliberately never emits
/// `--clearance`** -- see [`ExternalRouterSettings`]'s own doc comment
/// for why that flag is a ceiling capped against the sibling
/// `.kicad_pro`'s own Default net-class clearance, and letting
/// `route.py` read that value itself (by never overriding it here) is
/// the only way to guarantee this always matches Alladin's own JLCPCB
/// clearance for the board's actual copper weight.
fn route_py_args(net_names: &[String], settings: &ExternalRouterSettings) -> Vec<String> {
    let mut args = Vec::new();
    if !net_names.is_empty() {
        args.push("--nets".to_string());
        args.extend(net_names.iter().cloned());
    }
    args.push("--track-width".to_string());
    args.push(settings.track_width_mm.to_string());
    args.push("--via-size".to_string());
    args.push(settings.via_diameter_mm.to_string());
    args.push("--via-drill".to_string());
    args.push(settings.via_drill_mm.to_string());
    args.extend(split_args(&settings.extra_args));
    args
}

/// The actual background-thread body: spawns `route.py`, streams its
/// output, waits for it to finish (polling rather than a single
/// blocking `wait()`, see [`AutorouteHandle`]'s own doc comment for
/// why), then -- only on a clean exit with a real output file --
/// optionally runs the tool's own verification scripts and finally
/// imports/filters/remaps the result. Every early return uses `?`
/// through [`ExternalRouterError`]'s `From<std::io::Error>` for
/// filesystem trouble; process-specific failures get their own
/// explicit variants so the final report is specific about what
/// actually went wrong.
#[allow(clippy::too_many_arguments)]
fn run_blocking(
    in_path: &Path,
    out_path: &Path,
    live_nets: &[(u32, String)],
    net_names: &[String],
    settings: &ExternalRouterSettings,
    tx: &Sender<AutorouteEvent>,
    child_slot: &Arc<Mutex<Option<Child>>>,
    cancelled: &Arc<AtomicBool>,
) -> Result<AutorouteReport, ExternalRouterError> {
    let script_path = Path::new(&settings.tool_dir).join("route.py");
    if !script_path.is_file() {
        return Err(ExternalRouterError::ScriptNotFound(script_path));
    }

    let mut cmd = Command::new(&settings.python_bin);
    cmd.arg(&script_path).arg(in_path).arg(out_path).args(route_py_args(net_names, settings));
    cmd.stdout(Stdio::piped()).stderr(Stdio::piped());

    let mut child = cmd.spawn().map_err(|e| if e.kind() == std::io::ErrorKind::NotFound { ExternalRouterError::PythonNotFound } else { ExternalRouterError::Io(e) })?;
    let stdout = child.stdout.take();
    let stderr = child.stderr.take();
    let stdout_thread = stdout.map(|out| {
        let tx = tx.clone();
        thread::spawn(move || stream_lines(out, &tx))
    });
    let stderr_thread = stderr.map(|err| {
        let tx = tx.clone();
        thread::spawn(move || stream_lines(err, &tx))
    });

    *child_slot.lock().unwrap() = Some(child);

    // Polls rather than a single blocking `wait()` so `child_slot`'s
    // lock is only ever held for one `try_wait()` call at a time --
    // see `AutorouteHandle`'s own doc comment for why a long-held lock
    // here would make `cancel()` block until the very process it's
    // trying to kill exits on its own, i.e. never actually cancel
    // anything.
    let wait_result = loop {
        {
            let mut guard = child_slot.lock().unwrap();
            match guard.as_mut() {
                None => break Err(ExternalRouterError::Cancelled),
                Some(child) => match child.try_wait() {
                    Ok(Some(status)) => break Ok(status),
                    Ok(None) => {}
                    Err(e) => break Err(ExternalRouterError::Io(e)),
                },
            }
        }
        thread::sleep(Duration::from_millis(80));
    };
    *child_slot.lock().unwrap() = None;
    if let Some(t) = stdout_thread {
        let _ = t.join();
    }
    if let Some(t) = stderr_thread {
        let _ = t.join();
    }

    if cancelled.load(Ordering::SeqCst) {
        return Err(ExternalRouterError::Cancelled);
    }
    let status = wait_result?;
    if !status.success() {
        return Err(ExternalRouterError::ProcessFailed { stderr: format!("route.py exited with {status}") });
    }
    if !out_path.is_file() {
        return Err(ExternalRouterError::ProcessFailed { stderr: "route.py finished but didn't write an output file".to_string() });
    }

    let tool_dir = Path::new(&settings.tool_dir);
    let drc_ok = run_verification_script(&settings.python_bin, tool_dir, "check_drc.py", out_path, tx);
    let connected_ok = run_verification_script(&settings.python_bin, tool_dir, "check_connected.py", out_path, tx);

    let routed_text = std::fs::read_to_string(out_path)?;
    let imported = crate::kicad_import::import_kicad_pcb(&routed_text).map_err(|e| ExternalRouterError::ImportFailed(e.to_string()))?;
    let imported_nets: Vec<(u32, String)> = imported.doc.nets.iter().map(|n| (n.id.0, n.name.clone())).collect();

    let live_net_of = |name: &str| live_nets.iter().find(|(_, n)| n == name).map(|(id, _)| NetId(*id));
    let items = filter_and_remap_routed_items(&imported.doc.node, &imported_nets, net_names, live_net_of);

    let mut routed_net_ids = std::collections::HashSet::new();
    for item in &items {
        if let Some(id) = item.net() {
            routed_net_ids.insert(id);
        }
    }
    let routed_nets: Vec<String> = net_names.iter().filter(|name| live_net_of(name).is_some_and(|id| routed_net_ids.contains(&id))).cloned().collect();

    Ok(AutorouteReport { items, requested_nets: net_names.to_vec(), routed_nets, drc_ok, connected_ok })
}

/// Starts an autoroute run against `doc`'s current state, restricted
/// to `net_names`, on a background thread -- returns as soon as the
/// board has been exported and the subprocess spawned (never blocks
/// for the run itself to finish; see [`AutorouteHandle`] for how a
/// caller follows along). Errors returned directly (rather than only
/// ever through the event channel) are the ones detectable *before*
/// ever spawning a subprocess: not configured, `route.py` missing, or
/// the board's own KiCad export failing -- everything that can only
/// go wrong *during* the run (the process itself, reading its output
/// back) arrives later as an [`AutorouteEvent::Done`].
///
/// `net_names` empty means "let `route.py` decide" (it treats a
/// missing `--nets` as "route everything unrouted" per its own
/// README) -- Alladin still reports back only the nets whose name
/// resolves against the live board (see [`filter_and_remap_routed_items`]),
/// so an empty list is a real, useful "just route what's left" mode,
/// not a no-op.
pub fn run_autoroute(doc: &BoardDoc, templates: &[FootprintTemplate], net_names: Vec<String>, settings: ExternalRouterSettings) -> Result<AutorouteHandle, ExternalRouterError> {
    if settings.tool_dir.trim().is_empty() {
        return Err(ExternalRouterError::ToolNotConfigured);
    }
    let script_path = Path::new(&settings.tool_dir).join("route.py");
    if !script_path.is_file() {
        return Err(ExternalRouterError::ScriptNotFound(script_path));
    }

    let unique = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos()).unwrap_or(0);
    let dir = std::env::temp_dir().join(format!("alladin_pcb_autoroute_{}_{unique}", std::process::id()));
    std::fs::create_dir_all(&dir)?;
    let in_path = dir.join("board.kicad_pcb");
    crate::kicad_export::export_kicad_files(doc, templates, &in_path)?;
    let out_path = dir.join("board_routed.kicad_pcb");
    let live_nets: Vec<(u32, String)> = doc.nets.iter().map(|n| (n.id.0, n.name.clone())).collect();

    let (tx, rx) = mpsc::channel();
    let child_slot: Arc<Mutex<Option<Child>>> = Arc::new(Mutex::new(None));
    let cancelled = Arc::new(AtomicBool::new(false));
    let child_slot_thread = Arc::clone(&child_slot);
    let cancelled_thread = Arc::clone(&cancelled);
    let tx_thread = tx;
    let dir_for_cleanup = dir.clone();

    thread::spawn(move || {
        let result = run_blocking(&in_path, &out_path, &live_nets, &net_names, &settings, &tx_thread, &child_slot_thread, &cancelled_thread);
        let _ = tx_thread.send(AutorouteEvent::Done(result));
        std::fs::remove_dir_all(&dir_for_cleanup).ok();
    });

    Ok(AutorouteHandle { events: rx, child: child_slot, cancelled })
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::{LayerId, NetClass, Node};
    use alladin_geom::{Circle, Point, Segment};

    #[test]
    fn split_args_handles_plain_whitespace_and_quoted_substrings() {
        assert_eq!(split_args(""), Vec::<String>::new());
        assert_eq!(split_args("  "), Vec::<String>::new());
        assert_eq!(split_args("--bus --ordering mps"), vec!["--bus", "--ordering", "mps"]);
        assert_eq!(split_args(r#"--foo "a b" --bar"#), vec!["--foo", "a b", "--bar"]);
        assert_eq!(split_args("--foo 'x y' plain"), vec!["--foo", "x y", "plain"]);
    }

    #[test]
    fn split_args_tolerates_an_unterminated_quote_by_taking_the_rest_of_the_input() {
        assert_eq!(split_args(r#"--foo "unterminated"#), vec!["--foo", "unterminated"]);
    }

    fn net_items() -> Node {
        let mut node = Node::new();
        node.add(Item::Track { shape: Segment::new(Point::new(0, 0), Point::new(MM, 0), 200_000), net: Some(NetId(1)), layer: LayerId::FCu, class: NetClass::C });
        node.add(Item::Via { shape: Circle::new(Point::new(MM, 0), 300_000), drill: 150_000, net: Some(NetId(2)) });
        // No net at all -- must never survive the filter regardless of
        // requested_net_names.
        node.add(Item::Track { shape: Segment::new(Point::new(0, MM), Point::new(MM, MM), 200_000), net: None, layer: LayerId::FCu, class: NetClass::C });
        node
    }

    #[test]
    fn filter_and_remap_keeps_only_requested_nets_and_remaps_onto_the_live_net_id() {
        let node = net_items();
        let imported_nets = vec![(1u32, "GND".to_string()), (2u32, "5V".to_string())];
        let requested = vec!["GND".to_string()];
        let live_net_of = |name: &str| if name == "GND" { Some(NetId(42)) } else { None };

        let items = filter_and_remap_routed_items(&node, &imported_nets, &requested, live_net_of);

        assert_eq!(items.len(), 1, "only the GND track must survive -- not the 5V via, not the netless track");
        match &items[0] {
            Item::Track { net, .. } => assert_eq!(*net, Some(NetId(42)), "must be remapped onto the live board's own NetId, not the imported file's NetId(1)"),
            other => panic!("expected the remapped Track, got {other:?}"),
        }
    }

    #[test]
    fn filter_and_remap_drops_an_item_whose_net_name_has_no_live_counterpart() {
        // Simulates a net renamed/deleted on the live board while the
        // background job was running -- must be dropped, never merged
        // with a bogus/missing net (see this fn's own doc comment).
        let node = net_items();
        let imported_nets = vec![(1u32, "GND".to_string()), (2u32, "5V".to_string())];
        let requested = vec!["GND".to_string(), "5V".to_string()];
        let live_net_of = |name: &str| if name == "GND" { Some(NetId(42)) } else { None };

        let items = filter_and_remap_routed_items(&node, &imported_nets, &requested, live_net_of);
        assert_eq!(items.len(), 1, "the 5V item must be dropped -- no live net named 5V exists");
    }

    #[test]
    fn filter_and_remap_returns_nothing_for_an_empty_requested_list() {
        let node = net_items();
        let imported_nets = vec![(1u32, "GND".to_string()), (2u32, "5V".to_string())];
        let items = filter_and_remap_routed_items(&node, &imported_nets, &[], |_| Some(NetId(1)));
        assert!(items.is_empty());
    }

    #[test]
    fn settings_round_trip_through_json_preserves_every_field() {
        let settings = ExternalRouterSettings {
            tool_dir: "/home/user/KiCadRoutingTools".to_string(),
            python_bin: "python3.12".to_string(),
            track_width_mm: 0.3,
            via_diameter_mm: 0.7,
            via_drill_mm: 0.35,
            extra_args: "--bus --ordering mps".to_string(),
        };
        let json = serde_json::to_string(&settings).unwrap();
        let loaded: ExternalRouterSettings = serde_json::from_str(&json).unwrap();
        assert_eq!(loaded, settings);
    }

    #[test]
    fn an_old_settings_file_with_a_leftover_clearance_mm_key_still_loads() {
        // Guards the plan's explicit backward-compat promise: an
        // `external_router.json` written by a pre-refactor Alladin
        // build still has a `"clearance_mm"` key on disk. serde must
        // ignore it silently (no `deny_unknown_fields`) rather than
        // fail `ExternalRouterSettings::load` and fall back to
        // defaults, discarding the user's other settings.
        let json = r#"{
            "tool_dir": "/home/user/KiCadRoutingTools",
            "python_bin": "python3.12",
            "track_width_mm": 0.3,
            "via_diameter_mm": 0.7,
            "via_drill_mm": 0.35,
            "clearance_mm": 0.12,
            "extra_args": ""
        }"#;
        let loaded: ExternalRouterSettings = serde_json::from_str(json).expect("unknown clearance_mm field must not break deserialization");
        assert_eq!(loaded.tool_dir, "/home/user/KiCadRoutingTools");
        assert!((loaded.track_width_mm - 0.3).abs() < 1e-9);
    }

    #[test]
    fn default_settings_match_alladins_own_jlcpcb_and_routing_defaults() {
        let settings = ExternalRouterSettings::default();
        assert!((settings.track_width_mm - 0.25).abs() < 1e-9, "must match crate::routing::DEFAULT_TRACE_WIDTH, got {}", settings.track_width_mm);
    }

    #[test]
    fn defaults_never_fall_below_the_real_jlcpcb_dfm_minimums() {
        let settings = ExternalRouterSettings::default();
        assert!(settings.track_width_mm >= alladin_core::JlcpcbDfm::MIN_TRACK_WIDTH as f64 / MM as f64);
        assert!(settings.via_diameter_mm >= alladin_core::JlcpcbDfm::MIN_VIA_DIAMETER as f64 / MM as f64);
        assert!(settings.via_drill_mm >= alladin_core::JlcpcbDfm::MIN_VIA_HOLE as f64 / MM as f64);
    }

    #[test]
    fn clamp_to_jlcpcb_minimums_raises_a_too_low_stored_value_but_keeps_a_wider_one() {
        let mut settings = ExternalRouterSettings { track_width_mm: 0.01, via_diameter_mm: 0.01, via_drill_mm: 0.01, ..ExternalRouterSettings::default() };
        settings.clamp_to_jlcpcb_minimums();
        assert!((settings.track_width_mm - alladin_core::JlcpcbDfm::MIN_TRACK_WIDTH as f64 / MM as f64).abs() < 1e-9);
        assert!((settings.via_diameter_mm - alladin_core::JlcpcbDfm::MIN_VIA_DIAMETER as f64 / MM as f64).abs() < 1e-9);
        assert!((settings.via_drill_mm - alladin_core::JlcpcbDfm::MIN_VIA_HOLE as f64 / MM as f64).abs() < 1e-9);

        let mut wide = ExternalRouterSettings { track_width_mm: 1.0, ..ExternalRouterSettings::default() };
        wide.clamp_to_jlcpcb_minimums();
        assert!((wide.track_width_mm - 1.0).abs() < 1e-9, "a deliberately wider-than-minimum value must never be lowered");
    }

    #[test]
    fn a_2oz_boards_exported_kicad_pro_clearance_and_route_py_args_never_diverge() {
        // Ties together the two halves of this plan's fix: (1)
        // `crate::kicad_export::export_kicad_files` already declares a
        // 2oz board's real 0.16mm minimum in the `.kicad_pro` sibling
        // `route.py` reads (see `kicad_export`'s own
        // `export_kicad_files_declares_a_2oz_boards_clearance_when_the_board_itself_is_2oz`
        // test), and (2) `route_py_args` never emits a `--clearance`
        // ceiling that could override that value downward. Together
        // these guarantee an autoroute run against a 2oz board always
        // routes at 0.16mm, never silently falls back to 1oz's 0.10mm.
        let board = crate::board_doc::NewBoardParams {
            width_mm: 40.0,
            height_mm: 40.0,
            layer_count: crate::board_doc::LayerCount::Two,
            copper_weight: crate::board_doc::CopperWeight::TwoOz,
            corner_radius_mm: 0.0,
        }
        .create();
        assert_eq!(board.net_class_clearance(), alladin_core::Jlcpcb2Layer2Oz::TRACK_TO_TRACK, "sanity: the board itself must resolve the 2oz clearance");

        let dir = std::env::temp_dir().join(format!("alladin_pcb_external_router_2oz_consistency_test_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let pcb_path = dir.join("board.kicad_pcb");
        crate::kicad_export::export_kicad_files(&board, &[], &pcb_path).expect("export must succeed");
        let pro_text = std::fs::read_to_string(pcb_path.with_extension("kicad_pro")).unwrap();
        let pro_json: serde_json::Value = serde_json::from_str(&pro_text).expect("must be valid JSON");
        assert_eq!(pro_json["net_settings"]["classes"][0]["clearance"], 0.16, "the .kicad_pro route.py reads must honestly declare the 2oz minimum");
        std::fs::remove_dir_all(&dir).ok();

        let args = route_py_args(&[], &ExternalRouterSettings::default());
        assert!(!args.iter().any(|a| a == "--clearance"), "no --clearance ceiling may ever be passed, or it could undercut the 0.16mm value just verified above");
    }

    #[test]
    fn route_py_args_never_contains_a_clearance_flag() {
        // The regression this whole plan exists to prevent: `--clearance`
        // must never again reach `route.py`'s argv, since it's a
        // ceiling that can silently undercut the board's own JLCPCB
        // net-class clearance -- see `ExternalRouterSettings`'s own doc
        // comment.
        let settings = ExternalRouterSettings::default();
        let args = route_py_args(&["GND".to_string(), "5V".to_string()], &settings);
        assert!(!args.iter().any(|a| a == "--clearance"), "must never pass --clearance, got {args:?}");
        assert!(args.iter().any(|a| a == "--track-width"), "must still pass the flags this module does own");
    }

    #[test]
    fn a_missing_tool_dir_is_reported_as_not_configured_before_ever_spawning_anything() {
        let board = crate::board_doc::NewBoardParams {
            width_mm: 40.0,
            height_mm: 40.0,
            layer_count: crate::board_doc::LayerCount::Two,
            copper_weight: crate::board_doc::CopperWeight::OneOz,
            corner_radius_mm: 0.0,
        }
        .create();
        let settings = ExternalRouterSettings::default(); // tool_dir left empty
        match run_autoroute(&board, &[], Vec::new(), settings) {
            Err(ExternalRouterError::ToolNotConfigured) => {}
            _ => panic!("expected ToolNotConfigured"),
        }
    }

    #[test]
    fn a_tool_dir_without_route_py_is_reported_as_script_not_found() {
        let board = crate::board_doc::NewBoardParams {
            width_mm: 40.0,
            height_mm: 40.0,
            layer_count: crate::board_doc::LayerCount::Two,
            copper_weight: crate::board_doc::CopperWeight::OneOz,
            corner_radius_mm: 0.0,
        }
        .create();
        let dir = std::env::temp_dir().join(format!("alladin_pcb_external_router_test_empty_dir_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut settings = ExternalRouterSettings::default();
        settings.tool_dir = dir.to_string_lossy().into_owned();

        match run_autoroute(&board, &[], Vec::new(), settings) {
            Err(ExternalRouterError::ScriptNotFound(_)) => {}
            _ => panic!("expected ScriptNotFound"),
        }
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn diagnose_reports_every_check_as_false_for_a_deliberately_bogus_python_binary() {
        let mut settings = ExternalRouterSettings::default();
        settings.python_bin = "definitely-not-a-real-python-binary-alladin-test".to_string();
        let report = diagnose(&settings);
        assert!(!report.python_found);
        assert!(!report.numpy_ok);
        assert!(!report.scipy_ok);
        assert!(!report.shapely_ok);
        assert!(!report.help_ok);
        assert!(!report.is_ready());
    }

    /// The one test that needs a real, locally configured
    /// `KiCadRoutingTools` checkout -- skipped, not failed, everywhere
    /// else (CI, a fresh clone). Configure via the
    /// `ALLADIN_KICAD_ROUTING_TOOLS_DIR` environment variable pointing
    /// at a real checkout (with `numpy`/`scipy`/`shapely` installed and
    /// `build_router.py` already run) to actually exercise this locally.
    #[test]
    fn run_autoroute_end_to_end_against_a_real_locally_configured_tool() {
        let Ok(tool_dir) = std::env::var("ALLADIN_KICAD_ROUTING_TOOLS_DIR") else {
            eprintln!("skipping: ALLADIN_KICAD_ROUTING_TOOLS_DIR not set");
            return;
        };
        let mut settings = ExternalRouterSettings::default();
        settings.tool_dir = tool_dir;
        if !diagnose(&settings).is_ready() {
            eprintln!("skipping: KiCadRoutingTools at {} isn't fully set up (see diagnose())", settings.tool_dir);
            return;
        }

        use crate::board_doc::{CopperWeight, LayerCount, NewBoardParams};
        let mut board =
            NewBoardParams { width_mm: 40.0, height_mm: 40.0, layer_count: LayerCount::Two, copper_weight: CopperWeight::OneOz, corner_radius_mm: 0.0 }.create();
        let templates = crate::footprint::builtin_templates();
        let template = &templates[0];
        let a = board.try_place_footprint(template, Point::new(-10 * MM, 0), 0.0).unwrap();
        let b = board.try_place_footprint(template, Point::new(10 * MM, 0), 0.0).unwrap();
        let pad_a = board.footprints.iter().find(|f| f.id == a).unwrap().pad_item_ids[0];
        let pad_b = board.footprints.iter().find(|f| f.id == b).unwrap().pad_item_ids[0];
        board.connect_pads(pad_a, pad_b).unwrap();

        let handle = run_autoroute(&board, &templates, vec!["Net1".to_string()], settings).expect("a configured tool must start successfully");
        let report = loop {
            match handle.events.recv().expect("the background job must eventually answer") {
                AutorouteEvent::Log(line) => eprintln!("route.py: {line}"),
                AutorouteEvent::Done(result) => break result.expect("a real, simple two-pad board must route successfully"),
            }
        };
        assert_eq!(report.routed_nets, vec!["Net1".to_string()]);
        assert!(!report.items.is_empty(), "a real successful route must produce at least one track");
    }
}
