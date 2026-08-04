//! Runtime decoder and layout engine for the embedded Hershey Futural
//! (Simplex Roman) stroke font (see `crate::stroke_font_data` for the
//! glyph table's provenance). Embedding it here is what makes "what
//! Alladin previews", "what the native Gerber export plots", and "what
//! the KiCad `.kicad_pcb` shows" the *same picture*: the GUI draws
//! these exact polylines, the DFM collision in `crate::board_doc`
//! checks these exact polylines (inflated by the text's real line
//! width), native Gerber strokes them, and KiCad export bakes the same
//! strokes as `(gr_line ...)` geometry (not `(gr_text ...)`, so KiCad
//! never re-renders with its own font).
//!
//! Decoding follows the classic Hershey encoding (same rules KiCad's
//! `STROKE_FONT::loadNewStrokeFont` uses for its *format*, but the
//! glyph *data* here is public-domain Hershey Futural, not KiCad
//! Newstroke): each glyph string's first two chars give the glyph's
//! own left/right extent (`char - 'R'`, scaled by 1/21), " R" raises
//! the pen, every other char pair is one polyline point
//! (`x = c0 - 'R'` shifted by the left extent, `y = c1 - 'R' - 8`,
//! both scaled by 1/21). Coordinates are "reduced": multiply by the
//! text's cap height to get real units, y grows downward, y = 0 is the
//! baseline, capitals span roughly y in [-1, 0], descenders reach
//! below 0.
//!
//! Layout matches the historical single-line, center/center-
//! justified stroke-text case Alladin produces -- including the
//! familiar 1.17 line-height multiplier and `stroke_width * 0.052`
//! vertical nudge used by KiCad's stroke font, so spacing stays
//! predictable for engineering silk.

use std::sync::OnceLock;

use crate::stroke_font_data::{FIRST_CODEPOINT, STROKE_GLYPHS};

/// One decoded Hershey glyph, still in reduced (unscaled) Hershey
/// coordinates -- see this module's doc comment for the coordinate
/// conventions.
pub struct Glyph {
    /// Pen-down polylines; every consecutive point pair is one drawn
    /// stroke segment.
    pub strokes: Vec<Vec<(f64, f64)>>,
    /// Horizontal advance to the next character, in reduced units
    /// (multiply by cap height).
    pub advance: f64,
}

/// Hershey glyphs are defined on a 21-unit grid; this maps them to the
/// "reduced" -1..+1-ish range (same scale factor KiCad's stroke font
/// uses for this encoding).
const STROKE_FONT_SCALE: f64 = 1.0 / 21.0;

/// Shifts the raw Hershey y values so y = 0 is the baseline.
const FONT_OFFSET: i32 = -8;

fn decode_glyph(encoded: &str) -> Glyph {
    let bytes = encoded.as_bytes();
    let mut strokes: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut current: Vec<(f64, f64)> = Vec::new();
    let mut start_x = 0.0;
    let mut advance = 0.0;
    let mut i = 0;
    while i + 1 < bytes.len() {
        let c0 = bytes[i];
        let c1 = bytes[i + 1];
        if i == 0 {
            let left = (c0 as i32 - b'R' as i32) as f64 * STROKE_FONT_SCALE;
            let right = (c1 as i32 - b'R' as i32) as f64 * STROKE_FONT_SCALE;
            start_x = left;
            advance = right - left;
        } else if c0 == b' ' && c1 == b'R' {
            if current.len() > 1 {
                strokes.push(std::mem::take(&mut current));
            } else {
                current.clear();
            }
        } else {
            let x = (c0 as i32 - b'R' as i32) as f64 * STROKE_FONT_SCALE - start_x;
            let y = (c1 as i32 - b'R' as i32 + FONT_OFFSET) as f64 * STROKE_FONT_SCALE;
            current.push((x, y));
        }
        i += 2;
    }
    if current.len() > 1 {
        strokes.push(current);
    }
    Glyph { strokes, advance }
}

fn glyphs() -> &'static [Glyph] {
    static GLYPHS: OnceLock<Vec<Glyph>> = OnceLock::new();
    GLYPHS.get_or_init(|| STROKE_GLYPHS.iter().map(|s| decode_glyph(s)).collect())
}

/// The decoded glyph for `c`, with fallback: anything outside the
/// embedded ASCII range renders as '?'.
pub fn glyph_for(c: char) -> &'static Glyph {
    let glyphs = glyphs();
    let index = (c as u32).wrapping_sub(FIRST_CODEPOINT) as usize;
    glyphs.get(index).unwrap_or_else(|| &glyphs[('?' as u32 - FIRST_CODEPOINT) as usize])
}

/// Single-line height fudge for center-justified vertical positioning.
const LINE_HEIGHT_FACTOR: f64 = 1.17;

/// Stroke-font vertical nudge used with the line-height fudge above.
const STROKE_WIDTH_Y_NUDGE: f64 = 0.052;

/// Total horizontal advance of `text` at cap height `height` (any
/// units -- the result is in the same units): each glyph moves the
/// cursor by its own advance, a space by the space glyph's.
pub fn text_advance(text: &str, height: f64) -> f64 {
    text.chars().map(|c| glyph_for(c).advance * height).sum()
}

/// Lays `text` out as a single-line, center/center-justified stroke
/// text of cap height `height` and stroke width `line_width` (both in
/// the same unit; output points are in that unit too): returns every
/// pen-down polyline, positioned relative to the text's anchor point,
/// unrotated -- rotation is the caller's business.
pub fn layout_polylines(text: &str, height: f64, line_width: f64) -> Vec<Vec<(f64, f64)>> {
    let width = text_advance(text, height);
    let baseline_y = height - line_width * STROKE_WIDTH_Y_NUDGE - (LINE_HEIGHT_FACTOR * height) / 2.0;
    let mut cursor_x = -width / 2.0;
    let mut out = Vec::new();
    for c in text.chars() {
        let glyph = glyph_for(c);
        for stroke in &glyph.strokes {
            out.push(stroke.iter().map(|&(x, y)| (x * height + cursor_x, y * height + baseline_y)).collect());
        }
        cursor_x += glyph.advance * height;
    }
    out
}

/// The tight, unrotated bounding box of `text`'s real ink (stroke
/// centerlines inflated by half the stroke width on every side),
/// relative to the anchor, as `(min_x, min_y, max_x, max_y)` -- what
/// `SilkText::bounding_rect` builds its click-target/selection-ring/
/// board-edge rectangle from. Whitespace-only text has no ink at all;
/// that degenerates to the advance-wide, cap-height-tall nominal box
/// so no caller ever sees an empty rectangle.
pub fn layout_bounds(text: &str, height: f64, line_width: f64) -> (f64, f64, f64, f64) {
    let polylines = layout_polylines(text, height, line_width);
    let mut min = (f64::MAX, f64::MAX);
    let mut max = (f64::MIN, f64::MIN);
    for line in &polylines {
        for &(x, y) in line {
            min.0 = min.0.min(x);
            min.1 = min.1.min(y);
            max.0 = max.0.max(x);
            max.1 = max.1.max(y);
        }
    }
    if min.0 > max.0 {
        let half_w = (text_advance(text, height) / 2.0).max(height / 4.0);
        let baseline_y = height - line_width * STROKE_WIDTH_Y_NUDGE - (LINE_HEIGHT_FACTOR * height) / 2.0;
        return (-half_w, baseline_y - height, half_w, baseline_y);
    }
    let pad = line_width / 2.0;
    (min.0 - pad, min.1 - pad, max.0 + pad, max.1 + pad)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_the_space_glyph_as_pure_advance_with_no_strokes() {
        let space = glyph_for(' ');
        assert!(space.strokes.is_empty(), "a space prints nothing");
        assert!(space.advance > 0.0, "but still advances the cursor");
    }

    #[test]
    fn capital_a_spans_from_the_baseline_up_and_a_g_reaches_below_it() {
        let a = glyph_for('A');
        let ys: Vec<f64> = a.strokes.iter().flatten().map(|&(_, y)| y).collect();
        assert!(ys.iter().cloned().fold(f64::MAX, f64::min) < -0.9, "'A' must reach roughly one cap height above the baseline");
        // Hershey baseline sits 1/21 below y = 0 (FONT_OFFSET residual)
        // -- 'A' bottoms out there, not at exactly 0.
        assert!(ys.iter().cloned().fold(f64::MIN, f64::max) < 0.06, "'A' must not reach meaningfully below the baseline");

        let g = glyph_for('g');
        let g_max_y = g.strokes.iter().flatten().map(|&(_, y)| y).fold(f64::MIN, f64::max);
        assert!(g_max_y > 0.1, "'g''s descender must reach below the baseline (y > 0)");
    }

    #[test]
    fn unknown_codepoints_fall_back_to_the_question_mark_glyph() {
        let fallback = glyph_for('\u{4E2D}');
        let question = glyph_for('?');
        assert_eq!(fallback.advance, question.advance);
        assert_eq!(fallback.strokes.len(), question.strokes.len());
    }

    #[test]
    fn layout_is_horizontally_centered_on_the_anchor() {
        let polylines = layout_polylines("HH", 1.0, 0.1);
        let min_x = polylines.iter().flatten().map(|&(x, _)| x).fold(f64::MAX, f64::min);
        let max_x = polylines.iter().flatten().map(|&(x, _)| x).fold(f64::MIN, f64::max);
        assert!((min_x + max_x).abs() < 0.05, "identical glyphs left and right of the anchor must balance out, got {min_x}..{max_x}");
    }

    #[test]
    fn layout_scales_linearly_with_height() {
        let small = text_advance("REV A", 1.0);
        let big = text_advance("REV A", 3.0);
        assert!((big - small * 3.0).abs() < 1e-9);
    }

    #[test]
    fn whitespace_only_text_still_yields_a_non_empty_bounding_box() {
        let (min_x, min_y, max_x, max_y) = layout_bounds("   ", 1.0, 0.15);
        assert!(max_x > min_x);
        assert!(max_y > min_y);
    }
}
