# Alladin PCB — web shell (experimental)

**Live demo (GitHub Pages):**
[https://draganito.github.io/alladin-pcb/](https://draganito.github.io/alladin-pcb/)

Trunk-based `eframe` WASM build. From the repo root:

```bash
rustup target add wasm32-unknown-unknown
cargo install trunk --locked   # once
# From the repo root (unset NO_COLOR if trunk rejects --no-color=1):
env -u NO_COLOR -u FORCE_COLOR trunk serve --config web/Trunk.toml --address 127.0.0.1 --port 8080
# → http://127.0.0.1:8080/
```

CI publishes the release build to GitHub Pages via
`.github/workflows/pages.yml` (`--public-url /alladin-pcb/`). Enable
**Settings → Pages → Source: GitHub Actions** once on the repo.

## Data transfer (no proxy)

The browser build has **no** LCSC download and **no** MCP.

1. On desktop: download/place parts, **Save** the board — used
   non-builtin templates are embedded in the same `.json`.
2. In the web UI: **Open…** that board JSON (no separate parts import
   needed for parts already on the board).

Optional: **Export parts…** / **Import parts…** for a portable
`alladin-parts.json` of spare library templates.

## AGPL §13

If you host this WASM build so others can run it over a network, you must
offer Corresponding Source of Alladin under AGPL-3.0 (this repository /
the same license terms). Shipping only the `.wasm` without source access
is not enough. The GitHub Pages demo satisfies that by pointing at this
same public source tree.
