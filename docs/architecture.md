# Architecture (short)

Alladin is a correct-by-construction 2-layer PCB editor focused on
JLCPCB manufacturability. The editable board format is Alladin's own
JSON. Manufacturing outputs (Gerber + Excellon zip, CPL, BOM) are written
natively — no KiCad install and no `kicad-cli` required.

One product, two shells: native desktop and WASM web. Routing is manual
(guided 45° tracks + segment drag). Boards embed the parts they use so a
single JSON opens on desktop or web. Web transfers boards/parts as files
(no LCSC in the browser).

## Crates

| Crate | Role |
|---|---|
| `alladin-geom` | Geometry in integer nanometres (`1_000_000` = 1 mm); ear-clip triangulation for concave fills |
| `alladin-core` | Board world (`Node`/`Item`), JLCPCB clearance rules |
| `alladin-gerber` | Native Gerber / Excellon writer |
| `alladin-render` | egui camera + draw helpers (outline/zone stroke; no heavy fills) |
| `alladin-pcb` | GUI; desktop: SQLite parts, LCSC, MCP, CLI, DXF outline import; web: file I/O |

Board outlines may be a rounded rectangle or a polygon from DXF
(`dxf_outline`: closed `LWPOLYLINE`, or one closed `LINE`/`ARC` ring).
The green soldermask substrate is painted via triangulated mesh so
concave notches do not leak fill.

## Desktop vs web

| | Desktop | Web (WASM) |
|---|---|---|
| Parts | SQLite + LCSC download; merge from opened boards | Session DB + import/`embedded_parts` |
| Board | Open/save paths | Upload/download JSON |
| MCP | Full surface on `:8642` (board / place / netlist / route / verify) | none |
| Fab export | Folder write | Manufacturing zip download |

MCP tools — read-only: `board_summary`, `get_footprints`, `get_nets`,
`list_parts`, `check_board`, `get_routing_scene`, `probe_route`; write
(need `--allow-ai-write`): `new_board`, `download_lcsc_part`,
`place_footprint`, `move_footprint`, `remove_footprint`, `connect_pins`,
`disconnect_pin`, `rename_net`, `save_board`, `commit_route`, `ripup_wire`.
Copper routing reuses the GUI's clearance gates (not an autorouter).
Zone fill stays in the GUI.

## What Alladin is not

- Not a KiCad fork and not linked against KiCad libraries.
- Does not require KiCad to place, route, or export for fabrication.
- No bundled external autorouter.

## Silkscreen font

Preview and Gerber share the same embedded Hershey Futural stroke font
(`tools/hershey/`). See `NOTICE` and `tools/hershey/USE_RESTRICTION.txt`.
