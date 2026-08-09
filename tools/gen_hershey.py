#!/usr/bin/env python3
"""Generate Alladin's embedded stroke-font table from Hershey Futural.

Reads the public-domain Hershey "futural" (Simplex Roman) glyph file
vendored at tools/hershey/futural.jhf and writes
crates/alladin-pcb/src/stroke_font_data.rs with one Hershey-
encoded string per ASCII codepoint U+0020..U+007E (space through
tilde). Decoding at runtime is unchanged -- see crate::stroke_font.

Source license (must be redistributed with the glyph data): see
tools/hershey/USE_RESTRICTION.txt -- originally created by
Dr. A. V. Hershey (U.S. NBS); this distribution format by James Hurt.

Run from the repo root:  python3 tools/gen_hershey.py
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "tools" / "hershey" / "futural.jhf"
DST = ROOT / "crates" / "alladin-pcb" / "src" / "stroke_font_data.rs"

FIRST_CODEPOINT = 0x20
LAST_CODEPOINT = 0x7E  # '~' -- skip the trailing DEL-like block glyph
COUNT = LAST_CODEPOINT - FIRST_CODEPOINT + 1


def rust_escape(s: str) -> str:
    return s.replace("\\", "\\\\").replace('"', '\\"')


def load_jhf(path: Path) -> list[str]:
    """Return glyph bodies (left/right + vertices) in file order."""
    glyphs: list[str] = []
    for ln in path.read_text(encoding="ascii").splitlines():
        if not ln.strip():
            continue
        # cols 0:4 glyph id, 5:7 vertex-pair count, 8: body
        body = ln[8:]
        n = int(ln[5:8])
        if len(body) != n * 2:
            raise SystemExit(f"bad jhf line (claimed {n} pairs, body len {len(body)}): {ln!r}")
        glyphs.append(body)
    return glyphs


def main() -> None:
    glyphs = load_jhf(SRC)
    if len(glyphs) < COUNT:
        raise SystemExit(f"only found {len(glyphs)} glyphs in {SRC}, need {COUNT}")
    glyphs = glyphs[:COUNT]

    lines = [
        "// GENERATED FILE -- do not edit by hand. Regenerate with:",
        "//   python3 tools/gen_hershey.py",
        "//",
        "// Glyph data from the Hershey Futural (Simplex Roman) font,",
        "// public-domain stroke font by Dr. A. V. Hershey (U.S. NBS).",
        "// Distribution format by James Hurt (Cognition, Inc.).",
        "// Redistribution requires the acknowledgements in",
        "// tools/hershey/USE_RESTRICTION.txt.",
        "//",
        "// One Hershey-encoded string per codepoint U+0020..U+007E,",
        "// decoded at runtime by `crate::stroke_font`. Silkscreen text",
        "// is baked to stroke geometry on KiCad export, so Alladin's",
        "// preview, native Gerber, and the exported .kicad_pcb all",
        "// share this same glyph geometry -- no KiCad Newstroke/GPL",
        "// font data is embedded.",
        "",
        "/// The codepoint `STROKE_GLYPHS[0]` encodes (a space); glyph index",
        "/// = codepoint - FIRST_CODEPOINT.",
        f"pub(crate) const FIRST_CODEPOINT: u32 = 0x{FIRST_CODEPOINT:X};",
        "",
        f"pub(crate) const STROKE_GLYPHS: [&str; {COUNT}] = [",
    ]
    for i, g in enumerate(glyphs):
        cp = FIRST_CODEPOINT + i
        ch = chr(cp)
        label = ch if ch.isprintable() and ch != " " else f"U+{cp:04X}"
        lines.append(f'    "{rust_escape(g)}", // {label}')
    lines.append("];")
    lines.append("")

    DST.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {COUNT} glyphs to {DST}")


if __name__ == "__main__":
    main()
