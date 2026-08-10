# Alladin PCB — The Manual

The complete A-to-Z user manual, as of v0.2.0-beta.1.
Deutsche Fassung: [HANDBUCH.md](HANDBUCH.md). For a guided, hands-on
introduction with a worked example see
[ANLEITUNG_FUER_ANFAENGER.md](../ANLEITUNG_FUER_ANFAENGER.md) (German) —
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
16. [Parts transfer Desktop ↔ Web](#16-parts-transfer-desktop--web)
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
- **Manual routing**: guided 45°/orthogonal traces and segment drag;
  no external autorouter.
- **AI-drivable (desktop)**: mini MCP for parts, placement, netlist,
  manual-style copper routing, and save (chapter 17). Zone fill stays in the GUI.
- **Portable boards**: saving embeds the non-builtin parts used on the
  board (`embedded_parts`), so one `.json` opens on desktop or web.
  Optional library export/import covers spare parts (chapter 16).

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
| `/usr/share/alladin-pcb/cursor-setup/` | Ready-made Cursor/MCP setup for AI control |
| `/usr/share/doc/alladin-pcb/` | Documentation and license notes |

Requirements: Debian/Ubuntu on x86-64, X11 or Wayland, glibc 2.39+ (`ldd --version`).
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
| "Import outline DXF…" | Optional board outline from LibreCAD/FreeCAD (closed `LWPOLYLINE`, or one closed ring of `LINE`/`ARC` segments; bulges/arcs tessellated). Dimensions then come from the DXF; corner radius is unused. | — |
| "Width (mm)" / "Height (mm)" | Outer board dimensions (1–500 mm), ignored when a DXF outline is loaded | 50 × 30 |
| "Layers" | 1 or 2 copper layers | 2 |
| "Copper weight" | "1oz" or "2oz". Determines the binding minimum clearance: 0.10 mm (1 oz) or 0.16 mm (2 oz) — per JLCPCB rules, not switchable anywhere in the program. | 1oz |
| "Corner radius (mm)" | Corner radius of the board outline, 0 = rectangular (unused with DXF) | 1.0 |

"Create board" creates the board and enters the editor. Invalid values
show "Invalid dimensions: …" and disable the button. **Careful:**
"New board..." from within the editor does not ask about unsaved
changes — save first.

**DXF tips:** Export one closed outer contour only (no dimensions, no
blocks). LibreCAD: closed polyline (arcs as bulges are fine). FreeCAD:
a sketch exported as geometry usually becomes `LINE`/`ARC` segments —
Alladin joins them if they form a single closed ring. Circles alone,
empty files, and leftover construction lines are rejected.

## 4. The editor at a glance

The editor has three areas:

**Top toolbar** (wraps on narrow windows):

- Status line: "Alladin PCB — 2-layer, 1oz board", next to it the AI
  status: "🔒 AI-Schreibzugriff aus (nur lesen via MCP)" (AI write
  access off, read-only) or "🔓 AI-Schreibzugriff aktiv (MCP)" (AI
  write access active).
- File handling: "Fit to board", "New board...", "Open...", "Save",
  "Save As…", "Export manufacturing files…"
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
2. Move the mouse: the live preview shows guided 45°/orthogonal legs.
   Obstacles are not auto-avoided; on collision the preview stays invalid (red).
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

### 9.3 Changing finished traces

In the select default state:

- **Clicking** selects the trace/via ("Selected: trace/via …").
- **Dragging a segment** reshapes the trace (segment drag; vertex count
  stays fixed) with the same green/red validity as routing. Vias cannot
  be moved by dragging.
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
  Save also embeds the non-builtin parts used on the board (see
  chapter 16).
- **Automatic reload**: Alladin watches the open file (~every 300 ms).
  If it changes externally — e.g. through a CLI command or a script —
  Alladin reloads it ("Board reloaded from disk."). A broken file is
  rejected and the last good state is kept.
- **Backups**: there is no automatic backup mechanism; save often under new names.
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


## 16. Parts transfer Desktop ↔ Web

The experimental web build (WASM) has no LCSC download and no MCP.
Boards carry their own parts:

1. Desktop: download any LCSC parts you need, place them, then **Save**.
   Alladin embeds the non-builtin templates used on that board in the
   same `.json` (`embedded_parts`).
2. Web: **Open…** that board file — footprints resolve from the embed;
   no separate parts import is required for parts already on the board.
3. Route manually and download the manufacturing zip.

**Optional — whole library:** **Export parts…** / **Import parts…**
writes or loads a portable `alladin-parts.json` when you want spare
templates that are not yet placed on a board.

There is no LCSC network proxy in the browser. If you host the WASM
build publicly, AGPL §13 requires offering Corresponding Source (this
repository). A public demo is served from GitHub Pages:
[https://draganito.github.io/alladin-pcb/](https://draganito.github.io/alladin-pcb/).



## 17. AI control via MCP

On **desktop**, Alladin embeds an MCP server. An AI can set up a board,
fetch and place parts, wire the netlist, lay copper with the same
clearance gates as the GUI's manual 45° router, and verify its own work.
Zone fill stays in the GUI. There is no classical autorouter.

### 17.1 Setup

1. Launch the GUI with `alladin-pcb --allow-ai-write`.
2. Copy the contents of `contrib/cursor-setup/` (or, from the deb,
   `/usr/share/alladin-pcb/cursor-setup/`) into your Cursor project
   (`.cursor/` and `.cursorignore`).
3. MCP URL: `http://127.0.0.1:8642/mcp`.

### 17.2 Tool reference (18 tools)

Read-only (always available):

| Tool | Purpose |
|---|---|
| `board_summary` | Overview / todo |
| `get_footprints` | Placed footprints |
| `get_nets` | Nets and pins |
| `list_parts` | Every placeable parts-library template |
| `check_board` | Verification report (netlist complete? copper connected? zones fresh? DFM findings) |
| `get_routing_scene` | Pads, tracks/vias, open copper bridges, routing rules |
| `probe_route` | Batched clearance check for proposed polylines (+ vias) |

Write (need `--allow-ai-write`):

| Tool | Purpose |
|---|---|
| `new_board` | Create a fresh board (refuses to discard an open one unless told to) |
| `download_lcsc_part` | LCSC → parts DB |
| `place_footprint` | Place a library template (same DFM gates as the GUI) |
| `move_footprint` | Move/rotate a placed part |
| `remove_footprint` | Remove a placed part |
| `connect_pins` | Netlist (join two pins) |
| `disconnect_pin` | Take one pin off its net |
| `rename_net` | Give a net a real name (`5V`, `GND`, …) |
| `save_board` | Save the board |
| `commit_route` | Lay a cleared copper route (same gates as the GUI preview) |
| `ripup_wire` | Remove a wire near a point, or all tracks/vias on a net |

### 17.3 Copper routing workflow

1. `get_routing_scene` — see `open_bridges` (shortest pad pairs between copper islands).
2. Propose one or more polylines (`segments` with `layer` + `points_mm`; multi-layer needs `vias_mm` at junctions).
3. `probe_route` — batch-test candidates (green/red = same gates as live preview). A blocked result names the exact leg and the items in the way (kind, net, footprint, layer, position), so the AI can route around them.
4. `commit_route` — write the first clear candidate (Ctrl+Z undoes). The commit also verifies connectivity: a route that doesn't actually join the net's copper islands (wrong layer, ends in free space) is rolled back and refused — the reply reports `bridge_closed` and the island count before/after.
5. `check_board` until `open_nets` is empty. On blockage: corners, other layer + via, or `ripup_wire`.

Every MCP write runs through the same JLCPCB DFM gates and the same
Ctrl+Z undo history as your own GUI gestures — you can always take back
what the AI did. Zone fill stays in the GUI.



## 18. Command line (CLI)

With no arguments the GUI starts. With a subcommand Alladin runs headless:

| Command | Purpose |
|---|---|
| `new-board <path>` | Create an empty board (`--width-mm`, `--height-mm`, `--layers`, `--copper-oz`, `--corner-radius-mm`) |
| `download-part <C-Nr>` | Download an LCSC part into the parts DB |
| `connect <board> <ref1> <pin1> <ref2> <pin2>` | Join two pins onto the same net |
| `list-nets <board>` | List nets |
| `list-footprints <board>` | List footprints |
| `board-summary <board>` | Compact overview |

Fab export goes through the GUI. MCP covers board setup, parts,
placement, netlist, manual-style copper routing, verification, and save.


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
| `Ctrl+Z` / `Cmd+Z` | editor (not in a text field) | Undo last board change (up to 40 steps) |
| `Ctrl+Y` / `Ctrl+Shift+Z` | editor (not in a text field) | Redo |
| `Enter` | zone mode | Close and fill the outline |
| `Enter` | text fields (net name, LCSC) | Confirm the input |
| `Shift`+click | connect mode, on a pad | Remove the pad from its net |
| Mouse wheel | canvas | Zoom |
| Drag (empty area) | canvas | Pan the view |
| Right-click | on a pad | Context menu "Add via near pin" |

Deliberately **absent**: Ctrl+S/Ctrl+O, layer hotkeys other than `V` mid-route.

## 20. Understanding messages and solving problems

Alladin refuses illegal actions and tells you why. The most common
messages:

| Message | Meaning / remedy |
|---|---|
| "This pin has no net yet — connect it to one first." | Routing only starts at pins with a net. Use "Connect pins" first. |
| "this leg collides with something or comes too close to the board edge" | The current leg is blocked. Pull a different path, fix a corner earlier, or switch layers with `V`. |
| "final leg comes within X.XXmm of the board edge" | The live path violates the edge clearance — adjust the path. |
| "can't fix a corner here — move the mouse first, or this leg is blocked" | `Space` at an invalid spot. |
| "no clear route here yet to drop a via onto" / "can't place a via here: …" | `V` at a blocked spot — a via needs room on both layers. |
| "Stitching net "…" — click to place a via." | Not an error: the via tool is waiting for the target position. |
| "⚠ Zones may be stale …" | The board changed since the last fill → "Refill zones". |
| "Couldn't open/save board: …" | Filesystem problem (path, permissions); details in the message. |
| "Board reloaded from disk." | Not an error: the file changed externally and was reloaded. |
| Red / blocked live preview | No legal guided path from the last corner to the cursor. Steer differently or set corners with `Space`. |

Ground rule: **a refusal means "not legal here right now", not
"broken".** Alladin never allows anything that would violate the
manufacturing rules — the way forward is a different path, another
layer or more room, never "pull harder".

## 21. Deliberate limitations

- **Light undo only.** Ctrl+Z / Ctrl+Y restore recent **board** changes
  (placement, nets, copper, zones, silk) with a capped history — not
  camera, tool mode, or parts-database edits. Zone fill / refill /
  solid-plane toggle each count as one step. Save often for longer-term
  versioning; there is no autosave.
- **No measuring tool.** Check distances via the grid or the position
  display of selected objects.
- **No KiCad import/export in the UI.** Alladin `.json` is the only board format; manufacturing is native.
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
| **Segment drag** | Drag an existing track segment; knicks follow, no new corners |
| **Stitching via** | Via joining copper of the same net across layers |
| **Clearance** | Mandatory minimum distance between copper of different nets |
| **DRC** | Design Rule Check — always satisfied by construction in Alladin |
| **Gerber / Excellon** | Industry formats for fabrication / drill data |
| **BOM** | Bill of materials |
| **CPL** | Component placement list |
| **LCSC** | Parts distributor; its C-numbers drive the part download and the BOM |
| **embedded_parts** | Non-builtin footprints stored inside the board `.json` on save |
| **alladin-parts** | Optional portable library JSON (spare templates, not on a board yet) |
| **MCP** | Model Context Protocol — the interface an AI uses to drive Alladin |
