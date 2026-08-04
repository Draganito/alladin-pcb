# Architecture (short)

Alladin is a correct-by-construction 2-layer PCB editor focused on
JLCPCB manufacturability. The editable board format is Alladin's own
JSON. Manufacturing outputs (Gerber + Excellon zip, CPL, BOM) are written
natively — no KiCad install and no `kicad-cli` required.

## Crates

| Crate | Role |
|---|---|
| `alladin-geom` | Geometry in integer nanometres (`1_000_000` = 1 mm) |
| `alladin-core` | Board world (`Node`/`Item`), JLCPCB clearance rules |
| `alladin-router` | Interactive path search (A*, walkaround, shove) |
| `alladin-sexpr` | Generic S-expression parser |
| `alladin-kicad-io` | Optional `.kicad_pcb` read/write used only as a bridge for the external autorouter — not a product import/export |
| `alladin-gerber` | Native Gerber / Excellon writer |
| `alladin-render` | egui camera + draw helpers |
| `alladin-pcb` | GUI, MCP server, CLI |

## What Alladin is not

- Not a KiCad fork and not linked against KiCad libraries.
- Does not require KiCad to place, route, or export for fabrication.
- Does not vendor [KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools); that tool stays an unmodified optional external program.

## Silkscreen font

Preview and Gerber share the same embedded Hershey Futural stroke font
(`tools/hershey/`). See `NOTICE` and `tools/hershey/USE_RESTRICTION.txt`.
