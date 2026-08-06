# Alladin PCB — Debian package

Precompiled Linux build of **Alladin PCB** (AGPL-3.0), packaged as a
`.deb`. Source:
[github.com/Draganito/alladin-pcb](https://github.com/Draganito/alladin-pcb).

Deutsche Version: [LIESMICH.txt](LIESMICH.txt). License notes: [LICENSE.txt](LICENSE.txt).

![Alladin PCB](docs/screenshot.png)

## Install

```bash
sudo apt install ./alladin-pcb_<version>_amd64.deb
```

That's the whole setup: the autorouter's Python dependencies
(`python3-numpy`, `python3-scipy`, `python3-shapely`) are declared as
package dependencies and installed automatically — no venv, no
`pip install`, nothing to build.

## What lands where

| Path | What it is |
|---|---|
| `/usr/bin/alladin-pcb` | The PCB editor (Linux x86-64) |
| `/usr/share/alladin-pcb/KiCadRoutingTools/` | Optional external autorouter, bundled **unmodified** — [KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools) (MIT, Andy Haas). Not part of the Alladin source repo. |
| `/usr/share/alladin-pcb/cursor-setup/` | Ready-made Cursor setup: MCP config, AI working rules, `.cursorignore` |
| `/usr/share/doc/alladin-pcb/` | This file, LIESMICH.txt, LICENSE.txt, docs |

## Requirements

- Debian/Ubuntu on x86-64, X11 or Wayland, **glibc 2.39+** (`ldd --version`)
- KiCad optional (own DRC only). Manufacturing is native; boards are Alladin `.json`.

## Run

```bash
alladin-pcb
```

AI write access: `alladin-pcb --allow-ai-write` — then copy the
*contents* of `/usr/share/alladin-pcb/cursor-setup/` (`.cursor/`
**and** `.cursorignore`) into your Cursor project folder. The included
rules make the AI work tool-first and terse; `.cursorignore` hides
board `.json` files so the MCP tools are its only way to touch the
board.

## Product facts

- Own `.json` board format; no KiCad import/export in the UI.
- **Export manufacturing files…** → Gerber zip + CPL + BOM (native).
- Hershey Futural silkscreen font (preview = Gerber).
- Autorouter: in **Autoroute (extern) settings**, set the tool folder to
  `/usr/share/alladin-pcb/KiCadRoutingTools` and Python to `python3`,
  press *Diagnose* (all checks should be green), then *Save*.
- The autorouter's compiled Rust core (`rust_router/grid_router.so`,
  Linux x86-64) is **already included** — nothing needs to be built.
