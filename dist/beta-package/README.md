# Alladin PCB

**For hobbyists and makers — get from idea to a JLCPCB board quickly.**

Correct-by-construction PCB editor: real **JLCPCB** design rules enforced
live while you place and route. Desktop AI via an embedded **MCP** surface.
Written in Rust. Builds as a native desktop app and (experimental) WASM web shell.

Aimed at ESP32 / smart-home / small robotics-style 1–2 layer boards, not
as a full professional EDA suite.

**License: [AGPL-3.0-only](LICENSE)** — Copyright © 2026 Dragan Bojovic.
See [NOTICE](NOTICE) for third-party credits (Hershey font).

Deutsche Einstiegsanleitung: [ANLEITUNG_FUER_ANFAENGER.md](ANLEITUNG_FUER_ANFAENGER.md).
Complete A-to-Z manual: [docs/MANUAL.md](docs/MANUAL.md) (English) /
[docs/HANDBUCH.md](docs/HANDBUCH.md) (German).
Architecture: [docs/architecture.md](docs/architecture.md).

![Alladin PCB](docs/screenshot.png)

## Download (precompiled, nothing to build)

Grab the ready-to-run beta from the
**[Releases page](https://github.com/Draganito/alladin-pcb/releases)**:
`alladin-pcb_<version>_amd64.deb` is the one release file — the Alladin
binary, Cursor MCP setup, and docs:

```bash
sudo apt install ./alladin-pcb_<version>_amd64.deb
alladin-pcb
```

Debian/Ubuntu x86-64, glibc 2.39+. Everything below is only needed if
you want to build from source.

## What you get

- **Own board format** — Alladin `.json` is the source of truth.
- **DXF board outline** — New Board can import a closed contour from
  LibreCAD or FreeCAD (`LWPOLYLINE`, or one closed ring of `LINE`/`ARC`
  segments). Dimensions come from the DXF; concave shapes fill correctly
  on the canvas.
- **Native manufacturing export** — one action writes:
  - `<name>_gerbers.zip` (Gerber + drill) — or a web manufacturing zip
  - `<name>_cpl.csv` (pick & place)
  - `<name>_bom.csv` (LCSC part numbers)
- **No KiCad required** to design or order boards.
- **Manual routing** — guided 45° traces and segment drag; no external
  autorouter.
- **Parts library** — LCSC download on desktop; boards embed the parts
  they use so one `.json` opens on desktop or web. Optional
  `alladin-parts` JSON export/import for spare library templates.
- **Silkscreen** — embedded Hershey Futural stroke font; editor preview
  matches Gerber output.

## Build

Requirements: recent Rust (stable), Linux desktop (X11/Wayland), glibc 2.39+.

```bash
cargo build --release -p alladin-pcb
./target/release/alladin-pcb
```

Tests:

```bash
cargo test -p alladin-pcb
```

## Try the web demo (experimental)

**Live:** [draganito.github.io/alladin-pcb](https://draganito.github.io/alladin-pcb/)

Same editor shell as desktop (open/save board JSON, manual routing, fab
zip download). No LCSC download and no MCP in the browser — boards carry
used parts via `embedded_parts`. Hosted from this repository under AGPL
§13 (Corresponding Source = this tree). Deployed by
[`.github/workflows/pages.yml`](.github/workflows/pages.yml) after push
to `main` (enable **Settings → Pages → Source: GitHub Actions** once).

Local web shell (needs `wasm32-unknown-unknown` + [Trunk](https://trunkrs.dev/)):

```bash
rustup target add wasm32-unknown-unknown
trunk serve --config web/Trunk.toml
```

Save a board on desktop and open the same `.json` in the web shell —
used parts travel inside the file. Optional library export/import covers
templates not yet on a board.

## AI control (MCP, desktop)

```bash
./target/release/alladin-pcb --allow-ai-write
```

An AI can set up a board, fetch and place parts, wire the netlist, lay
copper with the same clearance gates as the GUI's manual router, and
verify its own work — read-only: `board_summary`, `get_footprints`,
`get_nets`, `list_parts`, `check_board`, `get_routing_scene`,
`probe_route`, `probe_placement`, `suggest_route` (write only with
`commit=true`); write: `new_board`, `download_lcsc_part`,
`place_footprint`, `move_footprint`, `place_parts`, `move_parts`,
`remove_footprint`, `connect_pins`, `disconnect_pin`,
`add_pin_stitching_via`, `rename_net`, `set_zone_connection`,
`save_board`, `commit_route`, `ripup_wire`. Every write runs through the same DFM gates and Ctrl+Z undo
history as GUI gestures. Zone fill stays in the GUI (no autorouter).

Copy the *contents* of [`contrib/cursor-setup/`](contrib/cursor-setup)
(`.cursor/` **and** `.cursorignore`) into the folder you open in Cursor:
`.cursor/mcp.json` points at `http://127.0.0.1:8642/mcp`.

## Example board

[`examples/darkroom_led_panel_4x5_slim.json`](examples/darkroom_led_panel_4x5_slim.json)
— open it from the GUI (**Open…**).

## Repository layout

```text
crates/           Rust workspace members
web/              Trunk WASM shell (experimental)
tools/hershey/    Font sources + generator
docs/             Manuals + architecture notes
examples/         Example Alladin boards (.json)
contrib/          Optional Cursor MCP config
dist/             Scripts/templates for the Debian package
ANLEITUNG_….md    German beginner guide
```

See [docs/architecture.md](docs/architecture.md).

## Credits

- Silkscreen glyphs: Hershey Futural (Dr. A. V. Hershey / James Hurt) —
  see `tools/hershey/USE_RESTRICTION.txt`.
