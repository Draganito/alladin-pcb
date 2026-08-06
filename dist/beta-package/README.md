# Alladin PCB — Binary package

Precompiled Linux build of **Alladin PCB** (AGPL-3.0). Source:
[github.com/Draganito/alladin-pcb](https://github.com/Draganito/alladin-pcb).

Deutsche Version: [LIESMICH.txt](LIESMICH.txt). License notes: [LICENSE.txt](LICENSE.txt).

![Alladin PCB](docs/screenshot.png)

## Contents

| Path | What it is |
|---|---|
| `alladin-pcb` | The PCB editor (Linux x86-64, precompiled) |
| `KiCadRoutingTools/` | Optional external autorouter, bundled **unmodified** for easier setup — [KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools) (MIT). Not part of the Alladin source repo. |
| `cursor-setup/` | Ready-made Cursor setup: MCP config, AI working rules, `.cursorignore` |
| `docs/` | Screenshot + Hershey font use-restriction note |

## Requirements

- Linux x86-64, X11 or Wayland, **glibc 2.39+** (`ldd --version`)
- Autorouter only: `sudo apt install python3-numpy python3-scipy python3-shapely`
- KiCad optional (own DRC only). Manufacturing is native; boards are Alladin `.json`.

## Run

```bash
chmod +x alladin-pcb
./alladin-pcb
```

AI write access: `./alladin-pcb --allow-ai-write` — then copy the
*contents* of `cursor-setup/` (`.cursor/` **and** `.cursorignore`) into
your Cursor project folder. The included rules make the AI work
tool-first and terse; `.cursorignore` hides board `.json` files so the
MCP tools are its only way to touch the board.

## Product facts

- Own `.json` board format; no KiCad import/export in the UI.
- **Export manufacturing files…** → Gerber zip + CPL + BOM (native).
- Hershey Futural silkscreen font (preview = Gerber).
- Autorouter: configure **Autoroute (extern)** to this folder's
  `KiCadRoutingTools` (contains `route.py`). You can update that folder
  independently without changing Alladin.
- The autorouter's compiled Rust core (`rust_router/grid_router.so`,
  Linux x86-64) is **already included** — nothing needs to be built.
  To update or rebuild it later: `python3 build_router.py` inside
  `KiCadRoutingTools/` (downloads the matching prebuilt, or builds from
  source with `--from-source` if you have Rust). Full instructions are
  in `KiCadRoutingTools/README.md`.
