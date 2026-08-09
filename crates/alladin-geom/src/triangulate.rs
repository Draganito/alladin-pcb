//! Ear-clipping triangulation for simple polygons (no holes).
//!
//! Used for correct filled rendering of concave board outlines. Not a
//! general mesh library — self-intersecting or multi-contour inputs may
//! fail and return an empty triangle list.

use crate::{Point, Polygon};

/// Triangulate a simple polygon into index triples into `poly.points`.
///
/// Returns an empty list when the polygon is degenerate, self-intersecting
/// in a way that blocks ear clipping, or has fewer than three distinct
/// vertices. Hole polygons are not supported here — callers that need
/// cutouts should triangulate the outer ring and paint holes separately.
pub fn triangulate_simple(poly: &Polygon) -> Vec<[usize; 3]> {
    let pts = &poly.points;
    let n0 = pts.len();
    if n0 < 3 {
        return Vec::new();
    }

    // Work on an index ring, dropping consecutive duplicates (and a
    // duplicate closer if present).
    let mut idx: Vec<usize> = Vec::with_capacity(n0);
    for i in 0..n0 {
        if idx.last().map(|&j| pts[j] != pts[i]).unwrap_or(true) {
            idx.push(i);
        }
    }
    if idx.len() >= 2 && pts[*idx.first().unwrap()] == pts[*idx.last().unwrap()] {
        idx.pop();
    }
    if idx.len() < 3 {
        return Vec::new();
    }

    let area2 = signed_area2(pts, &idx);
    if area2 == 0 {
        return Vec::new();
    }

    let mut tris = Vec::with_capacity(idx.len().saturating_sub(2));
    let mut guard = 0usize;
    let guard_max = idx.len() * idx.len() + 8;

    while idx.len() > 3 {
        guard += 1;
        if guard > guard_max {
            return Vec::new();
        }
        let mut clipped = false;
        let m = idx.len();
        for i in 0..m {
            if is_ear(pts, &idx, i, area2) {
                let prev = idx[(i + m - 1) % m];
                let cur = idx[i];
                let next = idx[(i + 1) % m];
                tris.push([prev, cur, next]);
                idx.remove(i);
                clipped = true;
                break;
            }
        }
        if !clipped {
            return Vec::new();
        }
    }
    tris.push([idx[0], idx[1], idx[2]]);
    tris
}

fn signed_area2(pts: &[Point], idx: &[usize]) -> i128 {
    let n = idx.len();
    let mut sum = 0i128;
    for i in 0..n {
        let a = pts[idx[i]];
        let b = pts[idx[(i + 1) % n]];
        sum += a.x as i128 * b.y as i128 - b.x as i128 * a.y as i128;
    }
    sum
}

fn cross(o: Point, a: Point, b: Point) -> i128 {
    let ox = o.x as i128;
    let oy = o.y as i128;
    (a.x as i128 - ox) * (b.y as i128 - oy) - (a.y as i128 - oy) * (b.x as i128 - ox)
}

fn is_ear(pts: &[Point], idx: &[usize], i: usize, poly_area2: i128) -> bool {
    let m = idx.len();
    let prev = pts[idx[(i + m - 1) % m]];
    let cur = pts[idx[i]];
    let next = pts[idx[(i + 1) % m]];
    let c = cross(prev, cur, next);
    // Convex if the corner turns the same way as the polygon winding.
    if c == 0 || (c > 0) != (poly_area2 > 0) {
        return false;
    }
    // No other vertex strictly inside the ear triangle.
    for (j, &vj) in idx.iter().enumerate() {
        if j == (i + m - 1) % m || j == i || j == (i + 1) % m {
            continue;
        }
        if point_in_triangle(pts[vj], prev, cur, next) {
            return false;
        }
    }
    true
}

/// Inclusive of boundary — safe for ear tests (blocks ears that would
/// trap a boundary vertex).
fn point_in_triangle(p: Point, a: Point, b: Point, c: Point) -> bool {
    let c1 = cross(a, b, p);
    let c2 = cross(b, c, p);
    let c3 = cross(c, a, p);
    let has_neg = c1 < 0 || c2 < 0 || c3 < 0;
    let has_pos = c1 > 0 || c2 > 0 || c3 > 0;
    !(has_neg && has_pos)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MM;

    fn area_of_tris(pts: &[Point], tris: &[[usize; 3]]) -> f64 {
        tris.iter()
            .map(|&[i, j, k]| {
                let a = pts[i];
                let b = pts[j];
                let c = pts[k];
                ((a.x as f64) * (b.y as f64 - c.y as f64)
                    + (b.x as f64) * (c.y as f64 - a.y as f64)
                    + (c.x as f64) * (a.y as f64 - b.y as f64))
                    .abs()
                    * 0.5
            })
            .sum()
    }

    fn poly_area(poly: &Polygon) -> f64 {
        let n = poly.points.len();
        let mut sum = 0.0;
        for i in 0..n {
            let a = poly.points[i];
            let b = poly.points[(i + 1) % n];
            sum += a.x as f64 * b.y as f64 - b.x as f64 * a.y as f64;
        }
        (sum * 0.5).abs()
    }

    #[test]
    fn triangle_is_one_triangle() {
        let poly = Polygon::new(vec![
            Point::new(0, 0),
            Point::new(10 * MM, 0),
            Point::new(0, 10 * MM),
        ]);
        let tris = triangulate_simple(&poly);
        assert_eq!(tris.len(), 1);
    }

    #[test]
    fn rectangle_is_two_triangles() {
        let poly = Polygon::new(vec![
            Point::new(0, 0),
            Point::new(40 * MM, 0),
            Point::new(40 * MM, 30 * MM),
            Point::new(0, 30 * MM),
        ]);
        let tris = triangulate_simple(&poly);
        assert_eq!(tris.len(), 2);
        assert!((area_of_tris(&poly.points, &tris) - poly_area(&poly)).abs() < 1.0);
    }

    #[test]
    fn concave_c_shape_covers_interior_not_notch() {
        // C-shape (opening to the right), Y-down board space, CW winding:
        //
        //  (0,0)----(30,0)----(30,10)
        //    |                  |
        //  (0,40)  (10,30)--(30,30)
        //    |       |
        //  (0,50)---(30,50)
        let poly = Polygon::new(vec![
            Point::new(0, 0),
            Point::new(30 * MM, 0),
            Point::new(30 * MM, 10 * MM),
            Point::new(10 * MM, 10 * MM),
            Point::new(10 * MM, 40 * MM),
            Point::new(30 * MM, 40 * MM),
            Point::new(30 * MM, 50 * MM),
            Point::new(0, 50 * MM),
        ]);
        let tris = triangulate_simple(&poly);
        assert!(tris.len() >= 6, "expected several triangles, got {}", tris.len());
        let covered = area_of_tris(&poly.points, &tris);
        let expected = poly_area(&poly);
        assert!(
            (covered - expected).abs() / expected < 1e-6,
            "triangle area {covered} vs poly {expected}"
        );

        // Point in the notch (outside the C) must not lie in any triangle.
        let notch = Point::new(20 * MM, 25 * MM);
        assert!(!poly.contains_point(notch));
        for &[i, j, k] in &tris {
            assert!(
                !point_in_triangle(notch, poly.points[i], poly.points[j], poly.points[k])
                    || cross(poly.points[i], poly.points[j], poly.points[k]) == 0,
                "notch point fell inside a triangle"
            );
        }

        // Point inside the left bar must be covered.
        let inside = Point::new(5 * MM, 25 * MM);
        assert!(poly.contains_point(inside));
        let hit = tris.iter().any(|&[i, j, k]| {
            point_in_triangle(inside, poly.points[i], poly.points[j], poly.points[k])
        });
        assert!(hit, "interior point not covered by triangulation");
    }
}
