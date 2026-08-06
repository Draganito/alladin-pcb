#!/usr/bin/env bash
# Builds the binary release package for Alladin PCB.
#
# Result (always under dist/):
#   dist/alladin-pcb_<version>_amd64.deb   THE release file: binary +
#                                          bundled KiCadRoutingTools +
#                                          docs + cursor-setup (see
#                                          Cargo.toml's [package.metadata.deb])
#   dist/alladin-test/                     internal stage folder (cleaned
#                                          KiCadRoutingTools copy the deb
#                                          build references; not shipped)
#
# Usage:  dist/make_beta_package.sh [--skip-build]
#
# One-time prerequisite:  cargo install cargo-deb --locked

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEMPLATE_DIR="$PROJECT_DIR/dist/beta-package"
STAGE="$PROJECT_DIR/dist/alladin-test"

# Respect CARGO_TARGET_DIR (Cursor/sandbox often redirects the target
# tree). Falling back to ./target only when unset.
TARGET_DIR="${CARGO_TARGET_DIR:-$PROJECT_DIR/target}"
BIN="$TARGET_DIR/release/alladin-pcb"

# 1. Build the latest release binary (skip with --skip-build)
if [[ "${1:-}" != "--skip-build" ]]; then
    echo "==> cargo build --release -p alladin-pcb"
    echo "    CARGO_TARGET_DIR=$TARGET_DIR"
    (cd "$PROJECT_DIR" && cargo build --release -p alladin-pcb)
fi
[[ -x "$BIN" ]] || { echo "ERROR: $BIN missing"; exit 1; }
echo "==> binary: $BIN ($(date -r "$BIN" '+%Y-%m-%d %H:%M'), $(du -h "$BIN" | cut -f1))"

# 2. Binary sanity checks (before packaging anything)
HELP=$("$BIN" --help)
echo "$HELP" | grep -q 'export-manufacturing' \
    || { echo "ERROR: binary missing export-manufacturing"; exit 1; }
if echo "$HELP" | grep -qE 'export-kicad|import-kicad|export-bom'; then
    echo "ERROR: binary still exposes removed public KiCad/BOM CLI commands — rebuild is stale"
    exit 1
fi
echo "$HELP" | grep -q '_bom.csv' \
    || { echo "ERROR: export-manufacturing help must mention BOM CSV"; exit 1; }

# 3. Stage a cleaned KiCadRoutingTools copy (what the deb bundles)
echo "==> staging cleaned KiCadRoutingTools in $STAGE"
[[ -d "$PROJECT_DIR/KiCadRoutingTools" ]] \
    || { echo "ERROR: $PROJECT_DIR/KiCadRoutingTools missing (clone beside the source tree; needed unmodified for the binary package only)"; exit 1; }
rm -rf "$STAGE"
mkdir -p "$STAGE"
cp -a "$PROJECT_DIR/KiCadRoutingTools" "$STAGE/KiCadRoutingTools"
rm -rf "$STAGE/KiCadRoutingTools/.git" "$STAGE/KiCadRoutingTools/.venv"
find "$STAGE/KiCadRoutingTools" -type d -name __pycache__ -prune -exec rm -rf {} +

# 4. Debian package (uses the binary from step 1 and the stage from step 3)
echo "==> cargo deb"
command -v cargo-deb >/dev/null \
    || { echo "ERROR: cargo-deb not installed (fix: cargo install cargo-deb --locked)"; exit 1; }
rm -f "$PROJECT_DIR"/dist/alladin-pcb_*.deb
DEB=$(cd "$PROJECT_DIR" && cargo deb -p alladin-pcb --no-build -o "$PROJECT_DIR/dist/" | tail -1)
[[ -f "$DEB" ]] || { echo "ERROR: cargo deb produced no package"; exit 1; }

# 5. Deb sanity checks
DEB_LISTING=$(dpkg-deb -c "$DEB")
for f in ./usr/bin/alladin-pcb \
         ./usr/share/alladin-pcb/KiCadRoutingTools/route.py \
         ./usr/share/alladin-pcb/KiCadRoutingTools/LICENSE \
         ./usr/share/alladin-pcb/KiCadRoutingTools/rust_router/grid_router.so \
         ./usr/share/alladin-pcb/cursor-setup/.cursor/mcp.json \
         ./usr/share/alladin-pcb/cursor-setup/.cursor/rules/alladin-mcp.mdc \
         ./usr/share/alladin-pcb/cursor-setup/.cursorignore \
         ./usr/share/doc/alladin-pcb/README.md \
         ./usr/share/doc/alladin-pcb/LIESMICH.txt \
         ./usr/share/doc/alladin-pcb/LICENSE.txt \
         ./usr/share/doc/alladin-pcb/docs/screenshot.png \
         ./usr/share/doc/alladin-pcb/docs/HANDBUCH.md \
         ./usr/share/doc/alladin-pcb/docs/MANUAL.md \
         ./usr/share/doc/alladin-pcb/docs/hershey-USE_RESTRICTION.txt; do
    grep -q " $f\$" <<<"$DEB_LISTING" \
        || { echo "ERROR: $f missing in deb"; exit 1; }
done
if grep -qE "\.git/|\.venv/|__pycache__/" <<<"$DEB_LISTING"; then
    echo "ERROR: .git/.venv/__pycache__ leaked into deb"; exit 1
fi
# Never ship Alladin source in the binary package. (KiCadRoutingTools'
# own rust_router/ sources are intended and allowed -- bundled unmodified.)
if grep -E "crates/alladin|alladin-(core|geom|gerber|render|router|sexpr|kicad-io)" <<<"$DEB_LISTING" | grep -vq "KiCadRoutingTools"; then
    echo "ERROR: Alladin source tree leaked into deb"; exit 1
fi
dpkg-deb --info "$DEB" | grep -q "python3-shapely" \
    || { echo "ERROR: deb is missing the python3 module Depends"; exit 1; }
DEB_SIZE=$(stat -c%s "$DEB")
[[ "$DEB_SIZE" -lt 99000000 ]] \
    || { echo "ERROR: deb exceeds GitHub's 100 MB release-asset comfort zone"; exit 1; }

echo
echo "OK: $DEB  ($(du -h "$DEB" | cut -f1))"
