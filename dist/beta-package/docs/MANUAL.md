# Alladin PCB — The Manual

The complete A-to-Z user manual, as of v0.2.0-beta.1.
Deutsche Fassung: [HANDBUCH.md](HANDBUCH.md). For a guided, hands-on
introduction with a worked example see
[ANLEITUNG_FUER_ANFAENGER.md](https://github.com/Draganito/alladin-pcb/blob/main/ANLEITUNG_FUER_ANFAENGER.md) (German) —
this manual is the reference that covers *everything*.

---

## Table of contents

1. [What Alladin PCB is — the concept](#1-what-alladin-pcb-is--the-concept)
2. [Installation and startup](#2-installation-and-startup)
3. [Creating a new board](#3-creating-a-new-board)
4. [The editor at a glance](#4-the-editor-at-a-glance)
5. [View and navigation](#5-view-and-navigation)
6. [Parts: database, LCSC download, custom templates](#6-parts-database-lcsc-download-custom-templates)
7. [Placing and editing parts](#7-placing-and-editing-parts)
8. [Nets](#8-nets)
9. [Routing traces](#9-routing-traces)
10. [Vias](#10-vias)
11. [Zones and power planes](#11-zones-and-power-planes)
12. [Silkscreen](#12-silkscreen)
13. [Trace, via and grid settings](#13-trace-via-and-grid-settings)
14. [Saving, opening, files](#14-saving-opening-files)
15. [Exporting manufacturing files and ordering at JLCPCB](#15-exporting-manufacturing-files-and-ordering-at-jlcpcb)
16. [External autorouter (KiCadRoutingTools)](#16-external-autorouter-kicadroutingtools)
17. [AI control via MCP](#17-ai-control-via-mcp)
18. [Command line (CLI)](#18-command-line-cli)
19. [All keyboard shortcuts](#19-all-keyboard-shortcuts)
20. [Understanding messages and solving problems](#20-understanding-messages-and-solving-problems)
21. [Deliberate limitations](#21-deliberate-limitations)
22. [Glossary](#22-glossary)

---

## 1. What Alladin PCB is — the concept

Alladin PCB is an interactive editor for 1- and 2-layer boards built
around one principle that sets it apart from classic PCB tools:
**correct-by-construction**. Classic programs let you draw anything and
report rule violations afterwards in a Design Rule Check (DRC). Alladin
inverts that: actions that would violate the manufacturing rules are
**not allowed to happen in the first place**. A part that would land too
close to another turns red and cannot be dropped there; a trace that
would create a short is never committed. Whatever is on the board is
rule-compliant by construction.

The rules behind this are the real **JLCPCB manufacturing rules**
(clearances depending on copper weight: 0.10 mm at 1 oz, 0.16 mm at
2 oz; minimum trace widths; via geometry). That is also why there is no
built-in after-the-fact DRC button — it would always be green.

Other cornerstones:

- **Own board format**: boards are `.json` files in Alladin's own
  format. There is no KiCad import/export in the UI; KiCad is not
  needed for anything.
- **Native manufacturing output**: Gerber, drill, placement (CPL) and
  bill-of-materials (BOM) files are written by Alladin itself, directly
  in JLCPCB's format.
- **Interactive routing with intelligence**: while you drag a trace,
  the live preview automatically avoids obstacles (walkaround), finds
  paths with an A* search, and can push other traces aside (shove) —
  with a live preview of what would happen.
- **AI-drivable**: a built-in MCP server lets an AI assistant (e.g. in
  Cursor) build the board through validated tools — under exactly the
  same rules as a human user (chapter 17).
- **Optional external autorouter**: for routing many nets
  automatically, Alladin integrates the independent MIT-licensed
  project
  [KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools)
  as a subprocess (chapter 16).

## 2. Installation and startup

### 2.1 Debian package (recommended)

The release file is a single Debian package, available on the
[Releases page](https://github.com/Draganito/alladin-pcb/releases):

```bash
sudo apt install ./alladin-pcb_<version>_amd64.deb
alladin-pcb
```

The package installs:

| Path | Contents |
|---|---|
| `/usr/bin/alladin-pcb` | The program |
| `/usr/share/alladin-pcb/KiCadRoutingTools/` | The external autorouter, bundled ready to run |
| `/usr/share/alladin-pcb/cursor-setup/` | Ready-made Cursor/MCP setup for AI control |
| `/usr/share/doc/alladin-pcb/` | Documentation and license notes |

The autorouter's Python dependencies (`numpy`, `scipy`, `shapely`) are
installed automatically by apt — no venv, no pip, nothing to build.
Requirements: Debian/Ubuntu on x86-64, X11 or Wayland, glibc 2.39+
(check with `ldd --version`).

### 2.2 From source

```bash
git clone https://github.com/Draganito/alladin-pcb.git
cd alladin-pcb
cargo build --release -p alladin-pcb
./target/release/alladin-pcb
```

Requires a recent stable Rust. Tests: `cargo test --workspace`.

### 2.3 Launch modes

- `alladin-pcb` — starts the graphical editor.
- `alladin-pcb --allow-ai-write` — GUI with AI write access enabled
  over MCP (chapter 17). Without this flag an AI can only read the
  board, never modify it.
- `alladin-pcb <subcommand> …` — any other argument switches to the
  headless command-line mode without a GUI (chapter 18);
  `alladin-pcb --help` lists all subcommands.

On GUI startup the last opened board is loaded automatically in the
background (status "⏳ Board wird geladen…"). The window title is always
"Alladin PCB"; the current filename is shown in the toolbar.

## 3. Creating a new board

The start screen "Alladin PCB — New board" (also reachable later via
"New board...") asks for the basics:

| Field | Meaning | Default |
|---|---|---|
| "Width (mm)" / "Height (mm)" | Outer board dimensions (1–500 mm) | 50 × 30 |
| "Layers" | 1 or 2 copper layers | 2 |
| "Copper weight" | "1oz" or "2oz". Determines the binding minimum clearance: 0.10 mm (1 oz) or 0.16 mm (2 oz) — per JLCPCB rules, not switchable anywhere in the program. | 1oz |
| "Corner radius (mm)" | Corner radius of the board outline, 0 = rectangular | 1.0 |

"Create board" creates the board and enters the editor. Invalid values
show "Invalid dimensions: …" and disable the button. **Careful:**
"New board..." from within the editor does not ask about unsaved
changes — save first.

## 4. The editor at a glance

The editor has three areas:

**Top toolbar** (wraps on narrow windows):

- Status line: "Alladin PCB — 2-layer, 1oz board", next to it the AI
  status: "🔒 AI-Schreibzugriff aus (nur lesen via MCP)" (AI write
  access off, read-only) or "🔓 AI-Schreibzugriff aktiv (MCP)" (AI
  write access active).
- File handling: "Fit to board", "New board...", "Open...", "Save",
  "Save As...", "Export manufacturing files...", "Autoroute (extern)…"
  with a gear "⚙" for its settings, and the filename (or "(unsaved)").
- Tools (clickable toggles): "Connect pins", "Route traces",
  "Place vias", "Draw zone", "Place silk text", "Place silk dot" —
  plus "Refill zones" as an action. There is no "Select" button:
  **selection is the default state**, reachable any time with `Escape`.
- Settings: trace width, via dimensions, "Reset", "Snap to grid",
  "Grid (mm)".
- Visibility ("Show:"): checkboxes for Outline, Pads, Tracks, Vias,
  Zones, B.Cu, Mounting holes, Ratsnest.

**Canvas** (center): the board on a dark green substrate, with grid
dots when snapping is on.

**Right side panel**: "Place part" (part list), "Download part (LCSC)",
"Add part to database...", "Parts (n)" (placed parts), selection
details, "Nets (n)" (net list), "Power/ground planes" and — depending
on the active tool — its help and input fields.

Red text lines under the toolbar or in the side panel are error
messages from the most recent action (chapter 20).

## 5. View and navigation

| Action | Effect |
|---|---|
| Mouse wheel over the canvas | Zoom |
| Dragging on empty canvas | Pan the view. Don't start over a part — that moves the part instead. Panning is disabled during an active route. |
| "Fit to board" | Fit the whole board into the view |

The "Show:" checkboxes toggle layers:

- **"Outline"**, **"Pads"**, **"Tracks"**, **"Vias"**, **"Zones"**,
  **"Mounting holes"** — the respective object kinds.
- **"Show back copper (B.Cu)"** — the back side is normally drawn
  dimmed (same net hue, darker); this checkbox hides it entirely.
- **"Ratsnest"** — the thin airlines showing which connections of a net
  still lack a trace. Every net has its own color; ratsnest, traces and
  pads of one net share it.

In the net list, "○/◉" **highlights** a single net: all other nets are
strongly dimmed — very useful for following one connection on a crowded
board.

Hovering a pad shows a tooltip `REF.pin-number`, with known pin
functions e.g. `U10.3 (VDD)`.

## 6. Parts: database, LCSC download, custom templates

### 6.1 Built-in parts

Always available under "Place part": "2-pin THT (2.54mm pitch)",
"4-pin THT header (2.54mm pitch)", "SOIC-8 (1.27mm pitch)", "Wire pad
(solder, 2mm)", "Mounting hole (M2, NPTH)", "Mounting hole (M2.5,
NPTH)", "Mounting hole (M3, NPTH)".

### 6.2 LCSC download

The main way to real parts: under "Download part (LCSC)" enter an LCSC
part number (e.g. `C2040`) and click "Download". Alladin fetches the
real footprint (pads with shape, size, position), the pin function
names (GND/VDD/DIN/…, where available) and the category from the
LCSC/EasyEDA database and stores everything permanently in your
personal parts library (`~/.local/share/alladin-pcb/parts.sqlite3`).
Downloaded parts appear in collapsible categories ("Category (count)")
in the "Place part" list.

Deleting: "✖" next to a part removes it from the database, "🗑" next to
a category removes the whole category — both with a confirmation dialog
("This cannot be undone.").

### 6.3 Simple custom templates

"Add part to database..." opens a form for simple custom footprints:
"Name", "Ref. prefix" (e.g. `R` for resistors), "Pins" (1–64),
"Pitch (mm)", "Pad radius (mm)", "Description", "Category".
"Save to parts database" stores it permanently. For complex footprints
the LCSC download is almost always the better route.

## 7. Placing and editing parts

### 7.1 Placing

1. Click a part in the "Place part" list → placement mode, a "ghost"
   preview follows the cursor. **Green** = position allowed,
   **red** = collision (it will not place there).
2. `R` rotates the preview in 90° steps ("Rotation: n°" in the panel).
3. Click to place. The mode stays active — keep clicking to place more
   instances.
4. `Escape` or "Cancel placement (Esc)" leaves the mode.

**Matrix placement**: the panel offers "Rows", "Cols", "Pitch X (mm)",
"Pitch Y (mm)" (default 1×1) — one click then places the whole grid at
once, e.g. 5×4 LEDs at 12.7 mm pitch. Dragging near the board's center
axes snaps to yellow guide lines.

### 7.2 Selecting, moving, rotating, deleting

In the default state (select): click a part → yellow selection ring;
the panel shows position/rotation and the hints "Drag it on the board
to move. R to rotate, Del to remove."

- **Move**: click and drag. The ghost shows green/red whether the
  target position is legal; releasing on red snaps the part back. With
  "Snap to grid" active, positions snap to the grid.
- **Rotate**: `R` (refused if the rotated pose would collide).
- **Delete**: `Delete`/`Backspace` or "✖" in the "Parts" list.
- **"Pin-1-Punkt (Silk)"**: checkbox on the selected part — places a
  silkscreen dot at pin 1 that moves with the part (important for
  polarized parts such as LEDs).

Already-routed traces do **not** move along with a part; the net
membership is preserved (the ratsnest shows the connection as open
again). After bigger rearrangements, delete affected traces and
re-route them.

## 8. Nets

A net is the logical statement "these pads belong together
electrically". Nets are created with the **"Connect pins"** tool:

1. Activate "Connect pins".
2. Click the first pin (cyan ring, message "First pin selected — click
   the pin to connect it to.").
3. Click the second pin → both are on the same net. If one already has
   a net, the other joins it; if neither has one, a new net "NetN" is
   created.
4. **Shift-click** on a pin removes it from its net.
5. Clicking empty space or `Escape` cancels the pending selection.

The **"Nets (n)"** list in the side panel shows every net with its pin
count. The name is a directly editable text field — name nets sensibly
right away (`GND`, `5V`, `3V3`, `DATA` …); commit with Enter or by
leaving the field. "✖" deletes the whole net: all pins are disconnected
and **all of the net's copper (traces, vias) is removed**. "○/◉"
toggles the highlight (chapter 5).

## 9. Routing traces

The heart of the program. Tool **"Route traces"**:

1. Click the start pin — it must already belong to a net (otherwise:
   "This pin has no net yet — connect it to one first.").
2. Move the mouse: the live preview shows the path Alladin would lay —
   with 45°/90° corners, automatically around obstacles (walkaround),
   using an A* path search across multiple obstacles when needed.
3. Click a target pin **of the same net** → the trace is committed.

### 9.1 Keys during an active route

| Key | Effect |
|---|---|
| `Space` | Fix the current corner (a firm bend), then continue |
| `V` | Drop a via at the cursor position and switch to the other copper layer |
| `Backspace` | Undo the last fixed corner |
| `Escape` | Abort the route entirely |

The panel counts along ("n corner(s) fixed.").

### 9.2 The preview colors

| Appearance | Meaning |
|---|---|
| Solid line in the net color | This path is clear and would be committed as shown |
| Solid **red** line | No legal path here right now (collision or too close to the board edge) |
| **Dashed** line in the net color + **orange dashed** foreign traces | Shove preview: the path is possible if Alladin pushes the orange-marked foreign traces aside. Clicking the target pin does exactly that. |

Shove only moves foreign traces in ways that keep every rule satisfied
— if that is impossible, the preview stays red.

### 9.3 Changing finished traces

In the select default state:

- **Clicking** selects the trace/via ("Selected: trace/via …").
- **Dragging a segment** reshapes the trace — with the same live logic
  (green/red, walkaround) as routing. Vias cannot be moved by dragging.
- `Delete`/`Backspace` deletes **the whole connected trace** (the net
  remains; the ratsnest shows the connection as open again, and it can
  be re-routed any time).

## 10. Vias

Three ways to get a via:

1. **Mid-route**: key `V` (chapter 9.1) — via at the cursor position,
   the rest of the trace continues on the other layer.
2. **Stitching vias** with the **"Place vias"** tool: first click a pad
   that already belongs to a net — that selects the net (yellow
   message: "Stitching net "GND" — click to place a via."). Every
   further click on the board drops a via of that net there. Clicking
   another pad switches nets; `Escape` resets. Typical use: stitching a
   copper pour on the back to the same net on the front.
3. **Right-click on a pad** → context menu **"Add via near pin"**:
   places a via right next to the pad with a short connecting stub. If
   the natural spot is blocked, a preview hangs on the cursor
   (green/red) and the next click places via+stub at a legal position;
   `Escape` cancels.

Via diameter and drill are set beforehand in the toolbar (chapter 13).

## 11. Zones and power planes

### 11.1 Drawing a free zone ("Draw zone")

1. Activate the "Draw zone" tool.
2. In the side panel **first** choose "Net:" (e.g. `GND`) and "Layer:"
   ("F.Cu"/"B.Cu").
3. Click the corner points of the outline one after another (orange
   preview).
4. Close it: click the **first** point again (from 3 points on, the
   first point shows a ring), press `Enter`, or click "Finish
   outline". "Cancel" discards.

The area fills immediately: all foreign pads, traces and vias are
cut out with correct clearance automatically; members of the zone's own
net are connected.

### 11.2 Full-board planes with one click

Under **"Power/ground planes"** in the side panel: tick "Solid F.Cu
plane" or "Solid B.Cu plane" and pick the net in the dropdown —
Alladin fills the entire board area of that layer with the net.
Changing the net refills; unticking removes the plane.

### 11.3 Refreshing ("Refill zones")

Zone fills are snapshots. After board changes the toolbar shows the
warning "⚠ Zones may be stale … — click Refill zones". Clicking
**"Refill zones"** recomputes all zones in the background (status
"⏳ zone refill…"). Always refill once before the manufacturing export.

## 12. Silkscreen

- **"Place silk text"**: enter "Text:" in the panel, choose "Side:"
  ("Front (F.SilkS)" / "Back (B.SilkS)"), optionally "Rotate 90°" and
  size via "−/+" — then place it on the board (green/red ghost like for
  parts; empty text is not placed). Alladin uses the Hershey Futural
  stroke font: **the preview is exactly what ends up in the Gerber**.
- **"Place silk dot"**: places round marker dots (side and diameter in
  the panel) — e.g. for pin-1 markers in places the automatic
  "Pin-1-Punkt" checkbox (chapter 7.2) doesn't cover.
- **Editing**: click in the select default state (yellow frame) — then
  move by dragging, `R` rotates texts, "−/+" changes the size,
  `Delete` removes.

Part references (R1, U10, …) are shown in the editor but are **not**
exported to the Gerber — the silkscreen stays clean; assembly runs off
the CPL file.

## 13. Trace, via and grid settings

In the toolbar, applying to all **future** copper (existing copper is
never changed):

| Field | Default | Meaning |
|---|---|---|
| "Trace width (mm):" | 0.25 | Width of new traces (minimum: JLCPCB rule) |
| "Via diameter (mm):" | 0.60 | Outer diameter of new vias |
| "Via drill (mm):" | 0.30 | Drill of new vias |
| "Reset" | — | Resets the three values to 0.25/0.6/0.3 |
| "Snap to grid" | on | Placing and moving snaps to the grid |
| "Grid (mm):" | 1.0 | Grid pitch (0.05–50 mm), only active while snapping |

## 14. Saving, opening, files

- **"Save"** writes to the current path (first time acts like "Save
  As..."), **"Save As..."** writes to a new name, **"Open..."** loads
  an Alladin `.json` file (file dialog filter "Aladin PCB board",
  `*.json`). Errors appear in red ("Couldn't open/save board: …").
- **Automatic reload**: Alladin watches the open file (~every 300 ms).
  If it changes externally — e.g. through a CLI command or a script —
  Alladin reloads it ("Board reloaded from disk."). A broken file is
  rejected and the last good state is kept.
- **Backups**: before merging an autorouter result, Alladin
  automatically writes `<board>.before-autoroute.json`.
- There are **no** Ctrl+S/Ctrl+O shortcuts — saving happens through the
  buttons.

## 15. Exporting manufacturing files and ordering at JLCPCB

Click **"Export manufacturing files..."**, choose a target folder —
Alladin natively (without KiCad) writes three files:

| File | Contents |
|---|---|
| `<name>_gerbers.zip` | Copper layers, solder mask, silkscreen, outline (edge cuts) and Excellon drill files |
| `<name>_cpl.csv` | Placement data (pick & place): designator, Mid X/Y, layer, rotation |
| `<name>_bom.csv` | Bill of materials with LCSC part numbers |

Ordering at JLCPCB:

1. Upload the Gerber zip at [jlcpcb.com](https://jlcpcb.com). Check the
   preview (outline, layers, drills).
2. Choose the board options; **copper weight** must match the board's
   setting (1 oz/2 oz).
3. Enable "SMT Assembly", upload the BOM and CPL files.
4. In part matching, verify every BOM line maps to a JLCPCB stock part
   (the LCSC numbers from the Alladin download match directly).
5. In the placement preview (2D/3D), check the orientation of polarized
   parts (pin-1 markers), then order.

Before all this: hit "Refill zones" once and save.

## 16. External autorouter (KiCadRoutingTools)

Alladin itself routes only interactively. For automatically routing
many nets at once it integrates the independent project
[KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools)
(MIT, Andy Haas) as a subprocess — unmodified, replaceable, optional.

### 16.1 Setup (once per machine)

With the Debian package the tool is already installed. Click the gear
"⚙" in the toolbar → window **"Autoroute (extern) settings"**:

| Field | Value (deb install) |
|---|---|
| "Tool folder" | `/usr/share/alladin-pcb/KiCadRoutingTools` |
| "Python" | `python3` |
| "Track width (mm)" / "Via diameter (mm)" / "Via drill (mm)" | defaults are fine (0.25/0.6/0.3) |
| "Clearance (mm)" | fixed to the JLCPCB rule, not editable |
| "Extra arguments" | empty (for special cases, e.g. `--bus`) |

Click **"Diagnose"** — six checks must turn green (Python, `route.py`,
numpy, scipy, shapely, `route.py --help`). Then **"Save"**. On source
installs, "Copy setup instructions" shows the clone/install steps.

### 16.2 Running it

1. Click **"Autoroute (extern)…"**.
2. Tick the nets to route in the dialog (preselected: all nets with
   more than one pad), **"Route n net(s)"**.
3. Watch the live log (seconds to minutes depending on the board);
   "Cancel run" aborts.
4. After "Finished." the report shows: "x/y requested net(s) routed",
   the results of "DRC check" and "Connectivity check", and how many
   new tracks/vias are waiting.
5. **Important**: the results are *not* on the board yet. **"Merge
   into board"** applies them (a backup
   `<board>.before-autoroute.json` is written first); **"Discard"**
   throws them away.

## 17. AI control via MCP

Alladin has a built-in MCP server (Model Context Protocol) through
which an AI assistant — for instance in Cursor — can drive the **live
open board**. The AI uses exactly the same validated operations as a
human: anything that would violate the manufacturing rules is refused.
Alladin is the guard; the AI cannot build anything illegal.

### 17.1 Setup

1. Start Alladin with write access enabled:

   ```bash
   alladin-pcb --allow-ai-write
   ```

   Without this flag only the read tools work; every write operation is
   refused with a clear message. The state is visible in the toolbar
   ("🔓 AI-Schreibzugriff aktiv (MCP)").

2. Copy the **contents** of the bundled setup into the project folder
   you open in Cursor — both `.cursor/` and `.cursorignore` (deb
   install: from `/usr/share/alladin-pcb/cursor-setup/`, source tree:
   `contrib/cursor-setup/`):

   | File | Purpose |
   |---|---|
   | `.cursor/mcp.json` | Connects Cursor to `http://127.0.0.1:8642/mcp` |
   | `.cursor/rules/alladin-mcp.mdc` | Working rules for the AI: MCP tools only, terse reporting, start with `board_summary`, poll `get_job_status` on timeouts |
   | `.cursorignore` | Hides board `.json` files from the AI — the MCP tools are its only path to the board |

The server listens on localhost only (port 8642), without
authentication — it is meant for the local machine. It runs whenever
the GUI is open.

### 17.2 Background jobs, timeouts and the single job slot

Fast tools answer within 3 seconds. Compute-heavy operations (zone
fills, routing searches, continuity checks, exports, batches) run as a
**background job** — and there is exactly **one slot**. Three rules of
conduct for AI clients follow:

- If a slow tool answers "no reply within Ns", the job is **still
  running**. Poll `get_job_status` every few seconds until `running` is
  `null` — `last_finished.result` then contains exactly the reply the
  original call would have returned.
- **Never re-issue** the operation just because the reply timed out —
  it would run a second time.
- While a job is running, further write calls are refused immediately
  as "busy" (pure read tools keep answering).

The external autorouter has its own status channel:
`start_external_autoroute` returns immediately,
`get_external_autoroute_status` reports `idle`/`running`/`done`/
`failed`. Applying the result ("Merge into board") deliberately remains
a manual click in the GUI.

### 17.3 Tool reference (32 tools)

**Reading — always allowed, even without `--allow-ai-write`:**

| Tool | Returns |
|---|---|
| `get_editor_state` | Live UI state: active tool, in-progress route/zone, selection, messages |
| `get_board_overview` | File path, dimensions, layers, counts (nets, parts, tracks, vias, zones) |
| `get_nets` | Every net with pin membership |
| `get_zones` | All zones with net, layer, outline and island counts |
| `get_footprints` | All placed parts with position, rotation, pad nets |
| `get_job_status` | Status of the background-job slot + full result of the last job |
| `get_external_autoroute_status` | Status of the external autorouter run |

**Slow analyses — no write flag needed, but they occupy the job slot:**

| Tool | Returns |
|---|---|
| `board_summary` | The whole picture in one call: dimensions, rules, what is still unfinished (pins without a net, discontinuous nets). Recommended first call. |
| `check_net_continuity` | Checks physical copper continuity (pads+tracks+vias+zones), optionally for a single net (`net_name`) |

**Writing — only with `--allow-ai-write`:**

| Group | Tools |
|---|---|
| Board | `create_board` (only on the New-board screen), `save_board` (without `path` = Save, with = Save As) |
| Parts | `place_footprint`, `download_lcsc_part` (not batchable), `register_part` |
| Nets | `connect_pins`, `rename_net` |
| Automatic routing | `route_pins` (point-to-point path search, same layer, no vias) |
| Manual routing (drag family) | `start_route` → `route_to` → `fix_corner` / `undo_last_corner` / `drop_via_and_switch_layer` → `finish_route` or `cancel_route` — the MCP equivalent of the mouse plus `Space`/`Backspace`/`V`/`Escape` |
| Vias | `add_via` (free stitching via, must touch the net's copper), `add_pin_stitching_via` (via + stub right at a pad, with automatic fallback search) |
| Zones | `add_zone` (polygon on `front`/`back`), `refill_zones` |
| Silkscreen | `add_silk_text` |
| Manufacturing | `export_manufacturing_files` (Gerber zip + CPL + BOM into a folder) |
| Autorouter | `start_external_autoroute` (optional `nets`, `extra_args`) |
| Batch | `run_batch` — runs a list of operations in one pass (`operations: [{"tool": …, "args": {…}}]`, `stop_on_error` on by default). All write tools are batchable except `download_lcsc_part` and `start_external_autoroute`. |

**Proven pattern for AI board building:** fetch parts with
`download_lcsc_part` → placements/nets/routes/zones/save as one
`run_batch` → verify with `board_summary` and `check_net_continuity`.

## 18. Command line (CLI)

Any argument other than `--allow-ai-write` starts Alladin without a GUI
as a command-line tool: load board file → run the operation → save,
process ends. This makes boards scriptable. Overview with
`alladin-pcb --help`, details per command with
`alladin-pcb <command> --help`.

| Command | Purpose | Key arguments |
|---|---|---|
| `new-board <file>` | Create an empty board | `--width-mm` (50), `--height-mm` (30), `--layers` (2), `--copper-oz` (1), `--corner-radius-mm` (1) |
| `list-templates` | List all footprint templates (built-in + database) | — |
| `download-part <C-no>` | Fetch an LCSC part into the database | e.g. `C2040`; refuses duplicates |
| `update-part <C-no>` | Re-fetch an already-downloaded part and overwrite it | — |
| `place-part <board>` | Place a part | `--template` (name from `list-templates`), `--x-mm`, `--y-mm`, `--rotation-deg` |
| `connect <board>` | Put two pins on one net | `--ref1`/`--pin1`, `--ref2`/`--pin2` |
| `route <board>` | Find and lay a trace between two connected pins (one layer, no vias) | `--ref1`/`--pin1`, `--ref2`/`--pin2` |
| `add-via <board>` | Place a stitching via (must touch the net's copper) | `--net`, `--x-mm`, `--y-mm`, `--diameter-mm` (0.6), `--drill-mm` (0.3) |
| `add-zone <board>` | Create and fill a copper pour | `--net`, `--layer front\|back`, `--points-file` (JSON polygon `[{"x_mm":…,"y_mm":…},…]`) |
| `refill-zones <board>` | Refill all zones | — |
| `list-zones <board>` | List zones | — |
| `set-outline <board>` | Replace the board outline | exactly one of `--from-kicad <file>` (Edge.Cuts only) or `--points-file` (multiple polygons = cutouts); refuses if existing items would fall outside |
| `register-part <name>` | Register a simple custom part | `--reference-prefix`, exactly one of `--pin-count` (pad row, `--pitch-mm`, `--pad-radius-mm`) or `--hole-diameter-mm` (NPTH hole), `--exclude-from-bom`, `--category` |
| `export-manufacturing <board> <folder>` | Write Gerber zip + CPL + BOM | — |
| `autoroute-external <board>` | Run the external autorouter blocking and merge the result (backup `*.before-autoroute.json` is written automatically) | `--nets` (repeatable; omit = all multi-pad nets), `--tool-dir`, `--extra-args` |

Coordinates are millimeters with the origin at the board center;
negative values work (`--x-mm -10`). A typical scripting pipeline:

```bash
alladin-pcb new-board board.json --width-mm 50 --height-mm 30
alladin-pcb download-part C2040
alladin-pcb place-part board.json --template "…" --x-mm 0 --y-mm 0
alladin-pcb connect board.json --ref1 U1 --pin1 1 --ref2 R1 --pin2 1
alladin-pcb route board.json --ref1 U1 --pin1 1 --ref2 R1 --pin2 1
alladin-pcb add-zone board.json --net Net1 --layer front --points-file pour.json
alladin-pcb refill-zones board.json
alladin-pcb export-manufacturing board.json ./fab
```

If the GUI has the same file open, it reloads external changes
automatically (chapter 14) — so you can watch the script work live.

## 19. All keyboard shortcuts

| Key | Context | Effect |
|---|---|---|
| `Escape` | everywhere | Back to the select default state; aborts running actions (placement, connection, route, trace drag, zone outline, stitching net, pin via) |
| `R` | placement mode | Rotate the preview +90° |
| `R` | select, part chosen | Rotate the part +90° (only if collision-free) |
| `R` | select, silk text chosen / silk text mode | Rotate the text +90° |
| `Space` | active route | Fix a corner |
| `V` | active route | Drop a via + switch layer |
| `Backspace` | active route with fixed corners | Undo the last corner |
| `Delete` / `Backspace` | select, something chosen | Delete part / whole trace / silk element |
| `Enter` | zone mode | Close and fill the outline |
| `Enter` | text fields (net name, LCSC) | Confirm the input |
| `Shift`+click | connect mode, on a pad | Remove the pad from its net |
| Mouse wheel | canvas | Zoom |
| Drag (empty area) | canvas | Pan the view |
| Right-click | on a pad | Context menu "Add via near pin" |

Deliberately **absent**: Ctrl+Z/Ctrl+Y (see chapter 21), Ctrl+S/Ctrl+O,
layer hotkeys other than `V` mid-route.

## 20. Understanding messages and solving problems

Alladin refuses illegal actions and tells you why. The most common
messages:

| Message | Meaning / remedy |
|---|---|
| "This pin has no net yet — connect it to one first." | Routing only starts at pins with a net. Use "Connect pins" first. |
| "this leg collides with something or comes too close to the board edge" | The current leg is blocked. Pull a different path, fix a corner earlier, or switch layers with `V`. |
| "route found, but comes within X.XXmm of the board edge" | The found path violates the edge clearance — adjust the path. |
| "can't fix a corner here — move the mouse first, or this leg is blocked" | `Space` at an invalid spot. |
| "no clear route here yet to drop a via onto" / "can't place a via here: …" | `V` at a blocked spot — a via needs room on both layers. |
| "Stitching net "…" — click to place a via." | Not an error: the via tool is waiting for the target position. |
| "⚠ Zones may be stale …" | The board changed since the last fill → "Refill zones". |
| "Couldn't open/save board: …" | Filesystem problem (path, permissions); details in the message. |
| "Board reloaded from disk." | Not an error: the file changed externally and was reloaded. |
| Search aborts ("no path", "search too complex") | No legal path currently exists between the start and the cursor (or it is too convoluted). Set intermediate corners with `Space` and route in stages. |

Ground rule: **a refusal means "not legal here right now", not
"broken".** Alladin never allows anything that would violate the
manufacturing rules — the way forward is a different path, another
layer or more room, never "pull harder".

## 21. Deliberate limitations

- **No undo/redo.** There is no global undo history. This is cushioned
  by the core principle: illegal things never happen in the first
  place, database deletions require a confirmation, traces can be
  re-routed any time, and autorouter merges automatically create a
  backup file. Still: **save regularly**, ideally under new names as
  manual versioning.
- **No measuring tool.** Check distances via the grid or the position
  display of selected objects.
- **No KiCad import/export in the UI.** Alladin's `.json` is the only
  board format; manufacturing is native. (Internally, only the external
  autorouter uses a KiCad interchange.)
- **No built-in DRC button** — unnecessary by construction (chapter 1).
- **1–2 copper layers, JLCPCB rule set.** More layers or other
  manufacturers' rule sets are currently out of scope.

## 22. Glossary

| Term | Meaning |
|---|---|
| **Footprint** | A part's board geometry: pads, holes, outline |
| **Pad** | A single solder terminal of a footprint |
| **Net** | Logical group of pads at the same potential (e.g. GND) |
| **Track / trace** | Copper connection on one layer |
| **Via** | Plated hole connecting the copper layers |
| **Zone / pour / plane** | Large filled copper area of one net |
| **F.Cu / B.Cu** | Front (top) / back (bottom) copper |
| **Ratsnest** | Airlines of connections not yet routed |
| **Walkaround** | Automatic obstacle avoidance while dragging |
| **Shove** | Pushing foreign traces aside to make room |
| **Stitching via** | Via joining copper of the same net across layers |
| **Clearance** | Mandatory minimum distance between copper of different nets |
| **DRC** | Design Rule Check — always satisfied by construction in Alladin |
| **Gerber / Excellon** | Industry formats for fabrication / drill data |
| **BOM** | Bill of materials |
| **CPL** | Component placement list |
| **LCSC** | Parts distributor; its C-numbers drive the part download and the BOM |
| **MCP** | Model Context Protocol — the interface an AI uses to drive Alladin |
