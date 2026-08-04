//! Ratsnest geometry: given the positions of every pad on one net, which
//! straight "still needs a track here" lines should the editor draw?
//! Pure graph/geometry, no `Node`/`BoardDoc` knowledge at all -- kept
//! separate and directly testable, the same split this workspace already
//! uses for e.g. `alladin_geom::Polygon::rounded_rect`.
//!
//! Deliberately a minimum spanning tree, not a naive star (every pad
//! connected to the first one) or a complete graph (every pair connected):
//! a star can draw a very long, visually misleading line across the whole
//! board when the *first* pad in net-membership order happens to sit far
//! from the rest, and a complete graph is O(n^2) line clutter for even a
//! modest net. A minimum spanning tree is what every real EDA tool's
//! ratsnest actually approximates, and is the shortest possible connected
//! set of "still needs wiring" hints for a given pad layout.

use alladin_geom::Point;

/// Prim's algorithm: starting from pad 0, repeatedly attaches whichever
/// not-yet-connected pad is nearest to the tree built so far. `O(n^2)`,
/// which is entirely fine at ratsnest scale (a handful to a few dozen
/// pads per net) -- no need for a heap-based `O(n log n)` version here.
/// Returns each edge as a `(from_index, to_index)` pair into `points`;
/// `points.len() < 2` returns no edges (nothing to connect).
pub fn minimum_spanning_edges(points: &[Point]) -> Vec<(usize, usize)> {
    let n = points.len();
    if n < 2 {
        return Vec::new();
    }

    let mut in_tree = vec![false; n];
    let mut best_dist = vec![f64::INFINITY; n];
    let mut best_from = vec![0usize; n];
    in_tree[0] = true;
    for j in 1..n {
        best_dist[j] = points[0].distance(points[j]);
        best_from[j] = 0;
    }

    let mut edges = Vec::with_capacity(n - 1);
    for _ in 1..n {
        let next = (0..n)
            .filter(|&j| !in_tree[j])
            .min_by(|&a, &b| best_dist[a].partial_cmp(&best_dist[b]).unwrap())
            .expect("at least one node is still outside the tree");

        edges.push((best_from[next], next));
        in_tree[next] = true;

        for j in 0..n {
            if !in_tree[j] {
                let d = points[next].distance(points[j]);
                if d < best_dist[j] {
                    best_dist[j] = d;
                    best_from[j] = next;
                }
            }
        }
    }
    edges
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_geom::MM;

    #[test]
    fn zero_or_one_point_has_no_edges() {
        assert!(minimum_spanning_edges(&[]).is_empty());
        assert!(minimum_spanning_edges(&[Point::new(0, 0)]).is_empty());
    }

    #[test]
    fn two_points_produce_exactly_one_edge_between_them() {
        let points = [Point::new(0, 0), Point::new(5 * MM, 0)];
        let edges = minimum_spanning_edges(&points);
        assert_eq!(edges, vec![(0, 1)]);
    }

    #[test]
    fn a_spanning_tree_has_exactly_n_minus_one_edges_and_connects_everything() {
        let points = [
            Point::new(0, 0),
            Point::new(10 * MM, 0),
            Point::new(5 * MM, 10 * MM),
            Point::new(20 * MM, 20 * MM),
        ];
        let edges = minimum_spanning_edges(&points);
        assert_eq!(edges.len(), points.len() - 1);

        // Union-find-by-hand: every point must end up reachable from 0.
        let mut reachable = vec![false; points.len()];
        reachable[0] = true;
        let mut changed = true;
        while changed {
            changed = false;
            for &(a, b) in &edges {
                if reachable[a] && !reachable[b] {
                    reachable[b] = true;
                    changed = true;
                }
                if reachable[b] && !reachable[a] {
                    reachable[a] = true;
                    changed = true;
                }
            }
        }
        assert!(reachable.iter().all(|&r| r), "every pad must be connected into the tree");
    }

    #[test]
    fn picks_the_short_direct_edges_over_a_long_star_from_the_first_point() {
        // A far-away first point plus two points close to *each other* but
        // far from it: the MST must use the short edge between the close
        // pair, not force both through the distant first point (a star
        // would draw one, much longer, extra line here).
        let far = Point::new(0, 0);
        let close_a = Point::new(100 * MM, 0);
        let close_b = Point::new(101 * MM, 0);
        let points = [far, close_a, close_b];

        let edges = minimum_spanning_edges(&points);
        assert_eq!(edges.len(), 2);
        assert!(
            edges.contains(&(1, 2)) || edges.contains(&(2, 1)),
            "expected the short close_a-close_b edge to be part of the tree, got {edges:?}"
        );
    }
}
