//! Board-outline import from a simple DXF (LibreCAD / FreeCAD).
//!
//! Accepts one outer contour from either:
//! - a closed [`LWPOLYLINE`] (closed flag, or first vertex == last), or
//! - a single closed ring of [`LINE`] / [`ARC`] segments (FreeCAD sketch
//!   export). Every segment must join into that one ring — leftovers or
//!   dangling geometry are rejected.
//!
//! Arc segments (polyline bulge / standalone [`ARC`]) are tessellated into
//! short chords. Coordinates are treated as millimetres; Y is flipped so
//! the CAD screen's "up" matches Alladin's on-screen up (+y is down in
//! board space). The contour is then translated so its bounding-box
//! center sits on the board origin.

use alladin_geom::{Point, Polygon, Unit, MM};

/// How finely bulge/ARC segments are broken into chords.
const ARC_SEGMENTS_PER_FULL_TURN: usize = 64;
#[derive(Debug, Clone)]
pub struct DxfOutline {
    /// Closed polygon, board-centered, Y already flipped into Alladin space.
    pub polygon: Polygon,
    /// Axis-aligned size after import (mm).
    pub width_mm: f64,
    pub height_mm: f64,
    pub vertex_count: usize,
    pub source_kind: &'static str,
}

#[derive(Debug, Clone, PartialEq)]
pub enum DxfOutlineError {
    Empty,
    NoContour,
    Degenerate,
}

impl std::fmt::Display for DxfOutlineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Empty => write!(f, "DXF file is empty"),
            Self::NoContour => write!(
                f,
                "no usable outline found -- need one closed LWPOLYLINE or a single closed ring of LINE/ARC segments"
            ),
            Self::Degenerate => write!(f, "outline is degenerate (too few points or zero area)"),
        }
    }
}

impl std::error::Error for DxfOutlineError {}

/// Parse a DXF byte buffer into a board outline polygon.
pub fn parse_dxf_outline(bytes: &[u8]) -> Result<DxfOutline, DxfOutlineError> {
    // Latin-1 / ANSI (LibreCAD) and UTF-8 both work: every byte → char.
    let text: String = bytes.iter().map(|&b| b as char).collect();
    if text.trim().is_empty() {
        return Err(DxfOutlineError::Empty);
    }
    let pairs = parse_groups(&text);
    let (polylines, lines, arcs) = collect_entities(&pairs);

    let mut candidates: Vec<(Vec<(f64, f64)>, &'static str)> = Vec::new();

    for pl in &polylines {
        if let Some(pts) = polyline_to_points(pl) {
            candidates.push((pts, "LWPOLYLINE"));
        }
    }

    if let Some(pts) = line_arc_to_closed_ring(&lines, &arcs) {
        let kind = if arcs.is_empty() {
            "LINE ring"
        } else {
            "LINE/ARC ring"
        };
        candidates.push((pts, kind));
    }

    let (mut pts_mm, kind) = pick_largest_contour(candidates).ok_or(DxfOutlineError::NoContour)?;
    if pts_mm.len() < 3 {
        return Err(DxfOutlineError::Degenerate);
    }
    // Drop duplicate closing vertex if present.
    if pts_mm.len() >= 2 {
        let a = pts_mm[0];
        let b = *pts_mm.last().unwrap();
        if (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9 {
            pts_mm.pop();
        }
    }
    if pts_mm.len() < 3 {
        return Err(DxfOutlineError::Degenerate);
    }

    // Flip Y (DXF Y-up → Alladin +y down) then center on origin.
    for p in &mut pts_mm {
        p.1 = -p.1;
    }
    let (min_x, max_x, min_y, max_y) = bbox_f64(&pts_mm);
    let cx = (min_x + max_x) * 0.5;
    let cy = (min_y + max_y) * 0.5;
    let width_mm = max_x - min_x;
    let height_mm = max_y - min_y;
    if width_mm <= 1e-6 || height_mm <= 1e-6 {
        return Err(DxfOutlineError::Degenerate);
    }

    let points: Vec<Point> = pts_mm
        .iter()
        .map(|(x, y)| Point::new(mm_to_nm(x - cx), mm_to_nm(y - cy)))
        .collect();

    // Remove consecutive duplicates after quantization.
    let mut cleaned = Vec::with_capacity(points.len());
    for p in points {
        if cleaned.last().map(|q: &Point| *q != p).unwrap_or(true) {
            cleaned.push(p);
        }
    }
    if cleaned.len() >= 2 && cleaned.first() == cleaned.last() {
        cleaned.pop();
    }
    if cleaned.len() < 3 {
        return Err(DxfOutlineError::Degenerate);
    }

    let polygon = Polygon::new(cleaned);
    let area = polygon_area_abs(&polygon);
    if area < (MM as f64) * (MM as f64) {
        // < 1 mm²
        return Err(DxfOutlineError::Degenerate);
    }

    Ok(DxfOutline {
        vertex_count: polygon.points.len(),
        polygon,
        width_mm,
        height_mm,
        source_kind: kind,
    })
}

fn mm_to_nm(mm: f64) -> Unit {
    (mm * MM as f64).round() as Unit
}

fn parse_groups(text: &str) -> Vec<(i32, String)> {
    let mut out = Vec::new();
    let mut lines = text.lines();
    while let Some(code_line) = lines.next() {
        let code = match code_line.trim().parse::<i32>() {
            Ok(c) => c,
            Err(_) => continue,
        };
        let value = lines.next().unwrap_or("").to_string();
        out.push((code, value));
    }
    out
}

#[derive(Debug, Default)]
struct LwPolyline {
    verts: Vec<(f64, f64)>,
    bulges: Vec<f64>,
    closed: bool,
}

#[derive(Debug, Clone, Copy)]
struct DxfLine {
    a: (f64, f64),
    b: (f64, f64),
}

#[derive(Debug, Clone, Copy)]
struct DxfArc {
    cx: f64,
    cy: f64,
    radius: f64,
    start_deg: f64,
    end_deg: f64,
}

impl DxfArc {
    fn start_point(self) -> (f64, f64) {
        self.point_at_deg(self.start_deg)
    }

    fn end_point(self) -> (f64, f64) {
        self.point_at_deg(self.end_deg)
    }

    fn point_at_deg(self, deg: f64) -> (f64, f64) {
        let rad = deg.to_radians();
        (
            self.cx + self.radius * rad.cos(),
            self.cy + self.radius * rad.sin(),
        )
    }

    /// CCW sweep degrees from `start_deg` to `end_deg` in DXF convention.
    fn sweep_deg(self) -> f64 {
        let mut sweep = self.end_deg - self.start_deg;
        while sweep <= 0.0 {
            sweep += 360.0;
        }
        while sweep > 360.0 {
            sweep -= 360.0;
        }
        sweep
    }
}

#[derive(Debug, Clone, Copy)]
enum RingSeg {
    Line(DxfLine),
    Arc(DxfArc),
}

impl RingSeg {
    fn endpoints(self) -> ((f64, f64), (f64, f64)) {
        match self {
            Self::Line(l) => (l.a, l.b),
            Self::Arc(a) => (a.start_point(), a.end_point()),
        }
    }
}

fn collect_entities(pairs: &[(i32, String)]) -> (Vec<LwPolyline>, Vec<DxfLine>, Vec<DxfArc>) {
    let mut polylines = Vec::new();
    let mut lines = Vec::new();
    let mut arcs = Vec::new();

    let mut i = 0;
    while i < pairs.len() {
        let (code, ref val) = pairs[i];
        if code != 0 {
            i += 1;
            continue;
        }
        match val.trim() {
            "LWPOLYLINE" => {
                let (pl, next) = read_lwpolyline(pairs, i + 1);
                if !pl.verts.is_empty() {
                    polylines.push(pl);
                }
                i = next;
            }
            "LINE" => {
                let (line, next) = read_line(pairs, i + 1);
                if let Some(line) = line {
                    lines.push(line);
                }
                i = next;
            }
            "ARC" => {
                let (arc, next) = read_arc(pairs, i + 1);
                if let Some(arc) = arc {
                    arcs.push(arc);
                }
                i = next;
            }
            _ => i += 1,
        }
    }
    (polylines, lines, arcs)
}

fn read_lwpolyline(pairs: &[(i32, String)], mut i: usize) -> (LwPolyline, usize) {
    let mut pl = LwPolyline::default();
    let mut pending_x: Option<f64> = None;
    let mut n_hint: Option<usize> = None;
    while i < pairs.len() {
        let (code, ref val) = pairs[i];
        if code == 0 {
            break;
        }
        match code {
            90 => n_hint = val.trim().parse().ok(),
            70 => {
                if let Ok(flags) = val.trim().parse::<i32>() {
                    pl.closed = (flags & 1) != 0;
                }
            }
            10 => pending_x = val.trim().parse().ok(),
            20 => {
                if let (Some(x), Ok(y)) = (pending_x.take(), val.trim().parse::<f64>()) {
                    pl.verts.push((x, y));
                    // default bulge 0 until a 42 follows this vertex
                    pl.bulges.push(0.0);
                }
            }
            42 => {
                if let Ok(b) = val.trim().parse::<f64>() {
                    if let Some(last) = pl.bulges.last_mut() {
                        *last = b;
                    }
                }
            }
            _ => {}
        }
        i += 1;
    }
    if let Some(n) = n_hint {
        if pl.verts.len() > n {
            pl.verts.truncate(n);
            pl.bulges.truncate(n);
        }
    }
    // If first==last, treat as closed even without flag.
    if pl.verts.len() >= 2 {
        let a = pl.verts[0];
        let b = *pl.verts.last().unwrap();
        if (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9 {
            pl.closed = true;
        }
    }
    (pl, i)
}

fn read_line(pairs: &[(i32, String)], mut i: usize) -> (Option<DxfLine>, usize) {
    let mut x1 = None;
    let mut y1 = None;
    let mut x2 = None;
    let mut y2 = None;
    while i < pairs.len() {
        let (code, ref val) = pairs[i];
        if code == 0 {
            break;
        }
        match code {
            10 => x1 = val.trim().parse().ok(),
            20 => y1 = val.trim().parse().ok(),
            11 => x2 = val.trim().parse().ok(),
            21 => y2 = val.trim().parse().ok(),
            _ => {}
        }
        i += 1;
    }
    let line = match (x1, y1, x2, y2) {
        (Some(x1), Some(y1), Some(x2), Some(y2)) => Some(DxfLine { a: (x1, y1), b: (x2, y2) }),
        _ => None,
    };
    (line, i)
}

fn read_arc(pairs: &[(i32, String)], mut i: usize) -> (Option<DxfArc>, usize) {
    let mut cx = None;
    let mut cy = None;
    let mut radius = None;
    let mut start = None;
    let mut end = None;
    while i < pairs.len() {
        let (code, ref val) = pairs[i];
        if code == 0 {
            break;
        }
        match code {
            10 => cx = val.trim().parse().ok(),
            20 => cy = val.trim().parse().ok(),
            40 => radius = val.trim().parse().ok(),
            50 => start = val.trim().parse().ok(),
            51 => end = val.trim().parse().ok(),
            _ => {}
        }
        i += 1;
    }
    let arc = match (cx, cy, radius, start, end) {
        (Some(cx), Some(cy), Some(radius), Some(start_deg), Some(end_deg)) if radius > 0.0 => {
            Some(DxfArc { cx, cy, radius, start_deg, end_deg })
        }
        _ => None,
    };
    (arc, i)
}

fn polyline_to_points(pl: &LwPolyline) -> Option<Vec<(f64, f64)>> {
    if pl.verts.len() < 2 {
        return None;
    }
    let n = pl.verts.len();
    let closed = pl.closed
        || (n >= 2
            && (pl.verts[0].0 - pl.verts[n - 1].0).abs() < 1e-9
            && (pl.verts[0].1 - pl.verts[n - 1].1).abs() < 1e-9);
    if !closed {
        return None;
    }

    // Effective vertex list without duplicate closer for segment iteration.
    let mut verts = pl.verts.clone();
    let mut bulges = pl.bulges.clone();
    while bulges.len() < verts.len() {
        bulges.push(0.0);
    }
    if verts.len() >= 2 {
        let a = verts[0];
        let b = *verts.last().unwrap();
        if (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9 {
            verts.pop();
            bulges.pop();
        }
    }
    let n = verts.len();
    if n < 3 {
        return None;
    }

    let mut out = Vec::new();
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        let bulge = bulges[i];
        out.push(a);
        if bulge.abs() > 1e-12 {
            let mid = tessellate_bulge(a, b, bulge);
            // skip first (a) already pushed; skip last (b) — next loop adds it
            if mid.len() > 2 {
                out.extend_from_slice(&mid[1..mid.len() - 1]);
            }
        }
    }
    Some(out)
}

/// Tessellate a DXF bulge arc from `a` to `b` into chord points including endpoints.
fn tessellate_bulge(a: (f64, f64), b: (f64, f64), bulge: f64) -> Vec<(f64, f64)> {
    // Included angle = 4 * atan(bulge); sign encodes CCW (+) / CW (-).
    let angle = 4.0 * bulge.atan();
    if angle.abs() < 1e-12 {
        return vec![a, b];
    }
    let (ax, ay) = a;
    let (bx, by) = b;
    let dx = bx - ax;
    let dy = by - ay;
    let chord = (dx * dx + dy * dy).sqrt();
    if chord < 1e-12 {
        return vec![a];
    }
    let s = bulge.abs();
    let radius = chord * (s * s + 1.0) / (4.0 * s);
    let mx = (ax + bx) * 0.5;
    let my = (ay + by) * 0.5;
    // Unit left-normal of a→b.
    let nx = -dy / chord;
    let ny = dx / chord;
    let dist_center = (radius * radius - (chord * 0.5).powi(2)).max(0.0).sqrt();
    // Positive bulge bows to the left of a→b; for |angle|<=π the center
    // sits on that same side.
    let sign = if bulge >= 0.0 { 1.0 } else { -1.0 };
    let side = if angle.abs() <= std::f64::consts::PI + 1e-9 { sign } else { -sign };
    let cx = mx + side * nx * dist_center;
    let cy = my + side * ny * dist_center;

    let start = (ay - cy).atan2(ax - cx);
    let steps = ((angle.abs() / std::f64::consts::TAU) * ARC_SEGMENTS_PER_FULL_TURN as f64)
        .ceil()
        .max(2.0) as usize;
    let mut pts = Vec::with_capacity(steps + 1);
    for k in 0..=steps {
        let t = k as f64 / steps as f64;
        let th = start + angle * t;
        pts.push((cx + radius * th.cos(), cy + radius * th.sin()));
    }
    if let Some(first) = pts.first_mut() {
        *first = a;
    }
    if let Some(last) = pts.last_mut() {
        *last = b;
    }
    pts
}

/// Join every LINE and ARC into exactly one closed ring and tessellate arcs.
/// Rejects dangling segments, forks (vertex degree ≠ 2), and leftover geometry.
fn line_arc_to_closed_ring(lines: &[DxfLine], arcs: &[DxfArc]) -> Option<Vec<(f64, f64)>> {
    let mut segs: Vec<RingSeg> = Vec::with_capacity(lines.len() + arcs.len());
    segs.extend(lines.iter().copied().map(RingSeg::Line));
    segs.extend(arcs.iter().copied().map(RingSeg::Arc));
    if segs.len() < 3 {
        return None;
    }

    // 0.01 mm grid for endpoint joining
    fn key(p: (f64, f64)) -> (i64, i64) {
        ((p.0 * 100.0).round() as i64, (p.1 * 100.0).round() as i64)
    }

    use std::collections::HashMap;
    // (seg_index, forward): forward means travel first→second endpoint
    let mut adj: HashMap<(i64, i64), Vec<(usize, bool)>> = HashMap::new();
    for (i, seg) in segs.iter().enumerate() {
        let (a, b) = seg.endpoints();
        adj.entry(key(a)).or_default().push((i, true));
        adj.entry(key(b)).or_default().push((i, false));
    }
    if !adj.values().all(|v| v.len() == 2) {
        return None;
    }

    let start_k = *adj.keys().next()?;
    let mut used = vec![false; segs.len()];
    let (first_i, first_fwd) = adj.get(&start_k)?.first().copied()?;
    used[first_i] = true;

    let mut ordered: Vec<(usize, bool)> = vec![(first_i, first_fwd)];
    let (a0, b0) = segs[first_i].endpoints();
    let mut cur_k = key(if first_fwd { b0 } else { a0 });

    for _ in 0..segs.len() + 2 {
        if cur_k == start_k && ordered.len() > 1 {
            break;
        }
        let candidates = adj.get(&cur_k)?;
        let mut advanced = false;
        for &(si, forward) in candidates {
            if used[si] {
                continue;
            }
            let (a, b) = segs[si].endpoints();
            let (from, to) = if forward { (a, b) } else { (b, a) };
            if key(from) != cur_k {
                continue;
            }
            used[si] = true;
            ordered.push((si, forward));
            cur_k = key(to);
            advanced = true;
            break;
        }
        if !advanced {
            break;
        }
    }

    if cur_k != start_k || !used.iter().all(|&u| u) || ordered.len() != segs.len() {
        return None;
    }

    let mut ring_pts = Vec::new();
    for &(si, forward) in &ordered {
        match segs[si] {
            RingSeg::Line(line) => {
                let (from, to) = if forward { (line.a, line.b) } else { (line.b, line.a) };
                if ring_pts.is_empty() {
                    ring_pts.push(from);
                }
                ring_pts.push(to);
            }
            RingSeg::Arc(arc) => {
                let chord = tessellate_dxf_arc(arc, forward);
                if ring_pts.is_empty() {
                    ring_pts.extend(chord);
                } else if chord.len() > 1 {
                    ring_pts.extend_from_slice(&chord[1..]);
                }
            }
        }
    }

    if ring_pts.len() >= 2 {
        let a = ring_pts[0];
        let b = *ring_pts.last().unwrap();
        if key(a) == key(b) {
            ring_pts.pop();
        }
    }
    if ring_pts.len() < 3 {
        return None;
    }
    Some(ring_pts)
}

/// Tessellate a DXF ARC into chord points including endpoints.
/// `forward == true` walks start→end (DXF CCW); `false` walks end→start.
fn tessellate_dxf_arc(arc: DxfArc, forward: bool) -> Vec<(f64, f64)> {
    let sweep = arc.sweep_deg().to_radians();
    if sweep < 1e-12 {
        return vec![arc.start_point()];
    }
    let steps = ((sweep / std::f64::consts::TAU) * ARC_SEGMENTS_PER_FULL_TURN as f64)
        .ceil()
        .max(2.0) as usize;
    let start_rad = arc.start_deg.to_radians();
    let mut pts = Vec::with_capacity(steps + 1);
    for k in 0..=steps {
        let t = k as f64 / steps as f64;
        let th = start_rad + sweep * t;
        pts.push((
            arc.cx + arc.radius * th.cos(),
            arc.cy + arc.radius * th.sin(),
        ));
    }
    // Snap endpoints to the analytic ARC ends (avoids float drift at joints).
    if let Some(first) = pts.first_mut() {
        *first = arc.start_point();
    }
    if let Some(last) = pts.last_mut() {
        *last = arc.end_point();
    }
    if !forward {
        pts.reverse();
    }
    pts
}

fn pick_largest_contour(candidates: Vec<(Vec<(f64, f64)>, &'static str)>) -> Option<(Vec<(f64, f64)>, &'static str)> {
    candidates
        .into_iter()
        .filter(|(pts, _)| pts.len() >= 3)
        .max_by(|a, b| {
            let aa = shoelace_abs(&a.0);
            let bb = shoelace_abs(&b.0);
            aa.partial_cmp(&bb).unwrap_or(std::cmp::Ordering::Equal)
        })
}

fn shoelace_abs(pts: &[(f64, f64)]) -> f64 {
    if pts.len() < 3 {
        return 0.0;
    }
    let n = pts.len();
    let mut sum = 0.0;
    for i in 0..n {
        let (x1, y1) = pts[i];
        let (x2, y2) = pts[(i + 1) % n];
        sum += x1 * y2 - x2 * y1;
    }
    (sum * 0.5).abs()
}

fn bbox_f64(pts: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for &(x, y) in pts {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }
    (min_x, max_x, min_y, max_y)
}

fn polygon_area_abs(poly: &Polygon) -> f64 {
    let n = poly.points.len();
    let mut sum = 0.0;
    for i in 0..n {
        let a = poly.points[i];
        let b = poly.points[(i + 1) % n];
        sum += (a.x as f64) * (b.y as f64) - (b.x as f64) * (a.y as f64);
    }
    (sum * 0.5).abs()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn examples_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples")
    }

    #[test]
    fn librecad_closed_lwpolyline_testboard3() {
        let path = examples_dir().join("testboard3.dxf");
        if !path.exists() {
            return;
        }
        let bytes = std::fs::read(&path).unwrap();
        let outline = parse_dxf_outline(&bytes).expect("testboard3");
        assert_eq!(outline.source_kind, "LWPOLYLINE");
        assert!(outline.vertex_count >= 3);
        assert!(outline.width_mm > 100.0);
        assert!(outline.height_mm > 100.0);
        assert!(outline.polygon.contains_point(Point::new(0, 0)));
    }

    #[test]
    fn librecad_bulge_polyline_testboard4() {
        let path = examples_dir().join("testboard4.dxf");
        if !path.exists() {
            return;
        }
        let bytes = std::fs::read(&path).unwrap();
        let outline = parse_dxf_outline(&bytes).expect("testboard4");
        assert_eq!(outline.source_kind, "LWPOLYLINE");
        // bulges should add vertices beyond the 11 control points
        assert!(outline.vertex_count > 11, "arcs should tessellate, got {}", outline.vertex_count);
        assert!(outline.polygon.contains_point(Point::new(0, 0)));
    }

    #[test]
    fn freecad_line_ring_testboard5() {
        let path = examples_dir().join("testboard5.dxf");
        if !path.exists() {
            return;
        }
        let bytes = std::fs::read(&path).unwrap();
        let outline = parse_dxf_outline(&bytes).expect("testboard5");
        assert_eq!(outline.source_kind, "LINE ring");
        assert_eq!(outline.vertex_count, 6);
        assert!((outline.width_mm - 97.14).abs() < 0.5);
        assert!(outline.polygon.contains_point(Point::new(0, 0)));
    }

    #[test]
    fn minimal_closed_lwpolyline_string() {
        let dxf = r#"0
SECTION
2
ENTITIES
0
LWPOLYLINE
90
4
70
1
10
0
20
0
10
40
20
0
10
40
20
30
10
0
20
30
0
ENDSEC
0
EOF
"#;
        let outline = parse_dxf_outline(dxf.as_bytes()).unwrap();
        assert_eq!(outline.vertex_count, 4);
        assert!((outline.width_mm - 40.0).abs() < 0.01);
        assert!((outline.height_mm - 30.0).abs() < 0.01);
        assert!(outline.polygon.contains_point(Point::new(0, 0)));
    }

    #[test]
    fn open_polyline_is_rejected() {
        let dxf = r#"0
SECTION
2
ENTITIES
0
LWPOLYLINE
90
3
70
0
10
0
20
0
10
10
20
0
10
10
20
10
0
ENDSEC
0
EOF
"#;
        assert!(parse_dxf_outline(dxf.as_bytes()).is_err());
    }

    #[test]
    fn freecad_line_arc_sketch_import() {
        let path = examples_dir().join("Unbenannt1-KörperSketch.dxf");
        if !path.exists() {
            return;
        }
        let bytes = std::fs::read(&path).unwrap();
        let outline = parse_dxf_outline(&bytes).expect("line/arc sketch");
        assert_eq!(outline.source_kind, "LINE/ARC ring");
        assert!(outline.vertex_count > 7, "arcs should tessellate");
        assert!(outline.width_mm > 40.0);
        assert!(outline.height_mm > 40.0);
        // Irregular FreeCAD sketches need not contain the bbox center.
        assert!(polygon_area_abs(&outline.polygon) > 100.0 * (MM as f64) * (MM as f64));
    }

    #[test]
    fn freecad_polyline_named_line_arc_export() {
        let path = examples_dir().join("polyline.dxf");
        if !path.exists() {
            return;
        }
        let bytes = std::fs::read(&path).unwrap();
        let outline = parse_dxf_outline(&bytes).expect("polyline.dxf");
        assert_eq!(outline.source_kind, "LINE/ARC ring");
        assert!(outline.vertex_count > 24);
        assert!((outline.width_mm - 70.3).abs() < 2.0);
        assert!((outline.height_mm - 89.2).abs() < 2.0);
        assert!(outline.polygon.contains_point(Point::new(0, 0)));
    }

    #[test]
    fn minimal_line_arc_ring_string() {
        // Unit square with bottom edge replaced by a semicircle bulging outward (-Y).
        // Lines: (0,0)->(0,10), (0,10)->(10,10), (10,10)->(10,0)
        // Arc: center (5,0), r=5, DXF CCW from 180° (0,0) to 0° (10,0) → lower half.
        let dxf = r#"0
SECTION
2
ENTITIES
0
LINE
10
0
20
0
11
0
21
10
0
LINE
10
0
20
10
11
10
21
10
0
LINE
10
10
20
10
11
10
21
0
0
ARC
10
5
20
0
40
5
50
180
51
0
0
ENDSEC
0
EOF
"#;
        let outline = parse_dxf_outline(dxf.as_bytes()).unwrap();
        assert_eq!(outline.source_kind, "LINE/ARC ring");
        assert!(outline.vertex_count > 4);
        assert!((outline.width_mm - 10.0).abs() < 0.05);
        assert!(outline.height_mm > 14.0); // semicircle adds ~5 mm
        assert!(outline.polygon.contains_point(Point::new(0, 0)));
    }

    #[test]
    fn leftover_line_rejects_line_arc_ring() {
        // Closed triangle of lines plus a dangling line — must reject.
        let dxf = r#"0
SECTION
2
ENTITIES
0
LINE
10
0
20
0
11
10
21
0
0
LINE
10
10
20
0
11
5
21
8
0
LINE
10
5
20
8
11
0
21
0
0
LINE
10
20
20
20
11
25
21
20
0
ENDSEC
0
EOF
"#;
        assert!(parse_dxf_outline(dxf.as_bytes()).is_err());
    }
}
