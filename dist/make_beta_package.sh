#!/usr/bin/env bash
# Builds the binary release package for Alladin PCB.
#
# Result (always under dist/):
#   dist/alladin-pcb_<version>_amd64.deb   THE release file: binary +
#                                          docs + cursor-setup (see
#                                          crates/alladin-pcb/Cargo.toml
#                                          [package.metadata.deb])
#
# Usage:  dist/make_beta_package.sh [--skip-build]
#
# One-time prerequisite:  cargo install cargo-deb --locked

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
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

# 2. Binary sanity checks
HELP=$("$BIN" --help)
echo "$HELP" | grep -qiE 'download-part|connect' \
    || { echo "ERROR: binary missing download-part / connect CLI surface"; exit 1; }
if echo "$HELP" | grep -qE '[[:space:]](export-kicad|import-kicad|export-bom|autoroute)[[:space:]]'; then
    echo "ERROR: binary still exposes removed KiCad/autoroute CLI commands — rebuild is stale"
    exit 1
fi

# 3. Debian package
echo "==> cargo deb"
command -v cargo-deb >/dev/null \
    || { echo "ERROR: cargo-deb not installed (fix: cargo install cargo-deb --locked)"; exit 1; }
rm -f "$PROJECT_DIR"/dist/alladin-pcb_*.deb
DEB=$(cd "$PROJECT_DIR" && cargo deb -p alladin-pcb --no-build -o "$PROJECT_DIR/dist/" | tail -1)
[[ -f "$DEB" ]] || { echo "ERROR: cargo deb produced no package"; exit 1; }

# 4. Deb sanity checks
DEB_LISTING=$(dpkg-deb -c "$DEB")
for f in \
    ./usr/bin/alladin-pcb \
    ./usr/share/alladin-pcb/cursor-setup/.cursor/mcp.json \
    ./usr/share/alladin-pcb/cursor-setup/.cursor/rules/alladin-mcp.mdc \
    ./usr/share/alladin-pcb/cursor-setup/.cursorignore \
    ./usr/share/doc/alladin-pcb/README.md \
    ./usr/share/doc/alladin-pcb/LIESMICH.txt \
    ./usr/share/doc/alladin-pcb/LICENSE.txt \
    ./usr/share/doc/alladin-pcb/docs/screenshot.png \
    ./usr/share/doc/alladin-pcb/docs/jlcpcb-smt-dfm-darkroom.png \
    ./usr/share/doc/alladin-pcb/docs/HANDBUCH.md \
    ./usr/share/doc/alladin-pcb/docs/MANUAL.md \
    ./usr/share/doc/alladin-pcb/docs/hershey-USE_RESTRICTION.txt
do
    grep -q " $f\$" <<<"$DEB_LISTING" \
        || { echo "ERROR: $f missing in deb"; exit 1; }
done
if grep -qE 'KiCadRoutingTools|\.git/|\.venv/|__pycache__/' <<<"$DEB_LISTING"; then
    echo "ERROR: KiCadRoutingTools or cache dirs leaked into deb"; exit 1
fi
if grep -E 'crates/alladin|alladin-(core|geom|gerber|render|router|sexpr|kicad-io)' <<<"$DEB_LISTING"; then
    echo "ERROR: Alladin source tree leaked into deb"; exit 1
fi
DEB_SIZE=$(stat -c%s "$DEB")
[[ "$DEB_SIZE" -lt 99000000 ]] \
    || { echo "ERROR: deb exceeds GitHub's 100 MB release-asset comfort zone"; exit 1; }

echo
echo "OK: $DEB  ($(du -h "$DEB" | cut -f1))"
