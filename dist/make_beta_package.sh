#!/usr/bin/env bash
# Builds the optional binary convenience package for Alladin PCB.
#
# Result (always under dist/):
#   dist/alladin-pcb-test.zip   packaged folder "alladin-test/"
#   dist/alladin-test/          stage folder (same contents, for git push)
#
# Package contents:
#   alladin-pcb          freshly built, stripped release binary
#   KiCadRoutingTools/   external autorouter, bundled unmodified
#                        (without .git/.venv/__pycache__) — NOT in the
#                        AGPL source tree; only here for easier first run
#   README.md, LIESMICH.txt, LICENSE.txt, cursor-setup/, docs/
#                        from dist/beta-package/
#
# Usage:  dist/make_beta_package.sh [--skip-build]

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEMPLATE_DIR="$PROJECT_DIR/dist/beta-package"
STAGE="$PROJECT_DIR/dist/alladin-test"
ZIP="$PROJECT_DIR/dist/alladin-pcb-test.zip"

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

# 2. Assemble the stage folder
echo "==> assembling $STAGE"
rm -rf "$STAGE"
mkdir -p "$STAGE"

cp "$BIN" "$STAGE/alladin-pcb"
strip "$STAGE/alladin-pcb"
chmod +x "$STAGE/alladin-pcb"

[[ -d "$PROJECT_DIR/KiCadRoutingTools" ]] \
    || { echo "ERROR: $PROJECT_DIR/KiCadRoutingTools missing (clone beside the source tree; needed unmodified for the binary package only)"; exit 1; }
cp -a "$PROJECT_DIR/KiCadRoutingTools" "$STAGE/KiCadRoutingTools"
rm -rf "$STAGE/KiCadRoutingTools/.git" "$STAGE/KiCadRoutingTools/.venv"
find "$STAGE/KiCadRoutingTools" -type d -name __pycache__ -prune -exec rm -rf {} +

cp "$TEMPLATE_DIR/README.md" "$TEMPLATE_DIR/LIESMICH.txt" "$TEMPLATE_DIR/LICENSE.txt" "$STAGE/"
cp -a "$TEMPLATE_DIR/docs" "$STAGE/docs"
mkdir -p "$STAGE/cursor-setup/.cursor/rules"
cp "$TEMPLATE_DIR/cursor-setup/.cursor/mcp.json" "$STAGE/cursor-setup/.cursor/"
cp "$TEMPLATE_DIR/cursor-setup/.cursor/rules/alladin-mcp.mdc" "$STAGE/cursor-setup/.cursor/rules/"
cp "$TEMPLATE_DIR/cursor-setup/.cursorignore" "$STAGE/cursor-setup/"

# 3. Zip (contents rooted at alladin-test/)
echo "==> zipping $ZIP"
rm -f "$ZIP"
(cd "$PROJECT_DIR/dist" && zip -qr "$ZIP" alladin-test)

# 4. Sanity checks
echo "==> checks"
LISTING=$(unzip -l "$ZIP")
for f in README.md LIESMICH.txt LICENSE.txt alladin-pcb KiCadRoutingTools/LICENSE cursor-setup/.cursor/mcp.json cursor-setup/.cursor/rules/alladin-mcp.mdc cursor-setup/.cursorignore docs/screenshot.png docs/hershey-USE_RESTRICTION.txt; do
    grep -q "alladin-test/$f" <<<"$LISTING" \
        || { echo "ERROR: $f missing in zip"; exit 1; }
done
if grep -qE "\.git/|\.venv/|__pycache__/" <<<"$LISTING"; then
    echo "ERROR: .git/.venv/__pycache__ leaked into zip"; exit 1
fi
# Never ship Alladin source in the binary package.
if grep -qE "alladin-test/crates/|alladin-test/Cargo\.toml|alladin-test/\.git/" <<<"$LISTING"; then
    echo "ERROR: Alladin source tree leaked into zip"; exit 1
fi
BIG=$(find "$STAGE" -type f -size +95M | wc -l)
[[ "$BIG" -eq 0 ]] || { echo "ERROR: file(s) over GitHub's 100 MB limit"; exit 1; }

HELP=$("$STAGE/alladin-pcb" --help)
echo "$HELP" | grep -q 'export-manufacturing' \
    || { echo "ERROR: binary missing export-manufacturing"; exit 1; }
if echo "$HELP" | grep -qE 'export-kicad|import-kicad|export-bom'; then
    echo "ERROR: binary still exposes removed public KiCad/BOM CLI commands — rebuild is stale"
    exit 1
fi
echo "$HELP" | grep -q '_bom.csv' \
    || { echo "ERROR: export-manufacturing help must mention BOM CSV"; exit 1; }

echo
echo "OK: $ZIP  ($(du -h "$ZIP" | cut -f1)),  unpacked: $(du -sh "$STAGE" | cut -f1)"
echo "Stage folder for direct git push: $STAGE"
