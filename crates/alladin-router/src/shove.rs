//! SHOVE: push *movable* existing tracks out of the way, instead of
//! only ever routing new tracks around fixed obstacles.
//!
//! Every other module in this crate (`walkaround`, `capsule_walkaround`,
//! `astar`, `optimizer`) answers "how do I get around what's already
//! there without disturbing it?" This module answers the complementary
//! question KiCad's `PNS::SHOVE` exists for: what if the right answer
//! isn't a detour at all, but moving the thing in the way? Real PNS
//! shove is a deep, recursive negotiation (a shoved track can itself
//! need to shove *its* neighbours, cascading outward). This module is
//! deliberately **not** that -- see [`try_shove_blockers`]'s own doc
//! comment for the precise, honest scope, and the development log's
//! "Teil 19"/"Teil 20"/"Teil 21"/"Teil 23" entries for the full story of
//! why this scope and not more, yet -- "Teil 21" in particular is worth
//! reading before attempting to extend this to real recursive cascading:
//! it documents a non-obvious dead end (recursing on "does exactly one
//! *other* item collide with the blocker's own, already-valid, unchanged
//! straight line" can *never* fire, for a structural reason, not a
//! tuning one). "Teil 23" documents why *this* module's own multi-
//! blocker extension is a structurally different, unaffected mechanism.
//!
//! Only possible at all since `alladin_core::Node` gained
//! [`alladin_core::Node::remove`] -- before that, `Node` was strictly
//! add-only, matching every caller's needs until this one.
//!
//! **Design history, kept here because it matters for reviewing this
//! code:** the first version of this module (see the development log's "Teil
//! 19") moved the blocker by rigidly translating its whole [`Segment`]
//! sideways -- simple, but with a real correctness gap found while
//! writing this module's own tests: a track's endpoints are usually
//! *not* free-floating, they sit exactly on a pad/via (or a sibling
//! track leg's shared vertex), and translating *both* endpoints
//! sideways would have silently pulled the track's copper off its pad.
//! The fix ("Teil 20") isn't a smarter translation -- it's to not
//! translate at all: [`try_shove_blockers`] instead **re-routes each
//! blocker between its own exact, unchanged endpoints**, with the
//! desired new route added as a temporary obstacle for that search.
//! Since the endpoints never move, whatever they were anchored to
//! (a pad, a via, or a sibling leg's shared vertex) stays connected for
//! free -- a strict improvement in both correctness and scope, not
//! a trade-off. It also means this module has essentially no new
//! geometry of its own left to get wrong: the actual re-routing is
//! delegated straight to [`crate::route_single_net`], the same
//! walkaround/A* engine every other module here already trusts.
//!
//! **"Teil 23" update:** originally this only ever handled *exactly
//! one* blocker on the direct line, refusing outright the moment a
//! second one was found. [`try_shove_blockers`] now handles any number
//! up to [`MAX_SIMULTANEOUS_BLOCKERS`] -- see that function's doc
//! comment for why processing several *simultaneous* blockers of one
//! fixed target line is a fundamentally different (and unaffected)
//! mechanism from the recursive "chase the blocker's own blocker" idea
//! that "Teil 21" found to be a dead end.

use alladin_core::{Item, LayerId, NetClass, NetId, Node, RuleResolver};
use alladin_geom::{contains_segment_evenodd, Point, Polygon, Segment, Unit};

/// Upper bound on how many *simultaneous* blockers of the direct line
/// [`try_shove_blockers`] will attempt to clear at once. Each one needs
/// its own full [`crate::route_single_net`] search (an already
/// non-trivial visibility-graph A* run), and every earlier blocker's
/// freshly-committed reroute becomes extra geometry every later
/// blocker's own search has to reckon with -- so cost grows with this
/// count, not just with board size. Four is generous for a real 2-layer
/// board's direct line (crossing four *other* class-C tracks at once on
/// one straight run is already an unusually congested scene) while
/// keeping worst-case cost bounded; raise it if a real board's own
/// results show it refusing scenes that a human router would consider
/// obviously shovable.
const MAX_SIMULTANEOUS_BLOCKERS: usize = 4;

/// Attempt to clear a **direct, straight** `from`→`to` route by
/// re-routing every blocking track on it, one at a time, around it. On
/// success, each blocker's old geometry is replaced by its own new,
/// verified-clear path (possibly more than one [`Item::Track`] leg now,
/// if a reroute needed a bend) and `Some(vec![from, to])` is returned --
/// the direct path is now genuinely valid. On failure, `world` is
/// returned **completely untouched**: every trial reroute happens on a
/// throwaway [`Node::clone`], and `world` itself is only ever mutated
/// once *every* blocker's trial has already fully succeeded.
///
/// **Deliberately a narrow, well-understood slice of real SHOVE, not the
/// general case** -- concretely, this refuses (returns `None`) unless
/// *all* of the following hold:
///
/// - The direct line is blocked by somewhere between one and
///   [`MAX_SIMULTANEOUS_BLOCKERS`] existing items (inclusive). More than
///   that, or a blocker that isn't in the way of the *direct* line at
///   all (only of some detour), isn't attempted.
/// - *Every one* of those blockers is an [`Item::Track`] of
///   [`NetClass::C`] -- exactly the class [`NetClass::C`]'s own doc
///   comment already describes as "flexible, may be shoved by later
///   passes". Pads, vias, and `NetClass::A`/`B` tracks (frozen/wide-
///   corridor by design) are never moved -- if even one blocker isn't
///   shovable, the whole attempt is refused before touching anything.
/// - [`crate::route_single_net`] can find *some* path for each
///   blocker's own net between its own two existing endpoints, in a
///   world where every blocker has been removed, the desired new route
///   has been added as a stand-in obstacle, and every blocker processed
///   so far has already been re-added at its own new position. If any
///   one blocker is truly boxed in with nowhere left to go even for its
///   own (much more flexible, bend-anywhere) reroute, the whole attempt
///   gives up rather than force it -- there's no partial success.
///
/// **What "several simultaneous blockers" means here, and why it's not
/// the "Teil 21" dead end:** this processes every item that *currently*
/// collides with the fixed, unchanging `from`→`to` line -- one call to
/// [`alladin_core::Node::query_colliding`] against that one probe,
/// simply not requiring the result to have length exactly one anymore.
/// It never recurses on any blocker's *own* line the way the reverted
/// "Teil 21" attempt did (which asked "what *else* collides with the
/// blocker's own, already-valid, unchanged position" -- provably always
/// empty, since that position was valid before this call ever started).
/// A congested direct line can perfectly well have two, three, or four
/// *different* class-C tracks crossing it at different points along its
/// length, all at once, all present *before* this function is even
/// called -- exactly what this handles now. True recursive/cascading
/// shove (a blocker's *new* position colliding with some *third*,
/// previously-uninvolved item that itself then also needs shoving) is
/// still out of scope -- see the development log's "Teil 21"/"Teil
/// 23" entries.
///
/// Because each blocker's own endpoints never move, this works
/// regardless of what they're anchored to -- a pad, a via, or a sibling
/// [`Item::Track`] leg's shared vertex (a multi-leg net, committed as
/// one [`Item::Track`] per leg) -- without needing to know or care
/// which.
///
/// Only ever tries clearing the **direct** `from`→`to` line -- if
/// walkaround/A* already found *some* (possibly detoured) path for the
/// new route, this function isn't even relevant; it's meant as a
/// last-resort fallback *after* [`crate::route_single_net`] has
/// already failed outright for the new route (the interactive router
/// in `alladin-pcb` is the current caller).
pub fn try_shove_blockers(
    world: &mut Node,
    from: Point,
    to: Point,
    width: Unit,
    net: NetId,
    layer: LayerId,
    class: NetClass,
    resolver: &dyn RuleResolver,
    outline: &[Polygon],
) -> Option<Vec<Point>> {
    if !outline.is_empty() && !contains_segment_evenodd(outline, from, to) {
        return None; // the direct line itself would leave the board -- no shove fixes that
    }

    let route_probe = Item::Track {
        shape: Segment::new(from, to, width),
        net: Some(net),
        layer,
        class,
    };

    let blocker_ids = world.query_colliding(&route_probe, resolver);
    if blocker_ids.is_empty() || blocker_ids.len() > MAX_SIMULTANEOUS_BLOCKERS {
        return None; // nothing to shove, or too congested for this bounded attempt
    }

    // Snapshot every blocker's shovable geometry up front, against the
    // *original* world -- refuse the whole attempt the moment even one
    // of them isn't a shovable class-C track, before touching anything.
    let mut blockers = Vec::with_capacity(blocker_ids.len());
    for &id in &blocker_ids {
        match world.get(id) {
            Some(Item::Track { shape, net: Some(n), layer: l, class: NetClass::C }) => {
                if shape.a == shape.b {
                    return None; // degenerate zero-length track: no meaningful reroute to search for
                }
                blockers.push((id, *shape, *n, *l));
            }
            _ => return None, // not a Track, no net, or not the shovable NetClass
        }
    }

    // Trial run on a throwaway clone: remove every blocker, stand the
    // desired new route in as a temporary obstacle in their place, and
    // ask the existing walkaround/A* engine to re-route each blocker's
    // own net between its own unchanged endpoints, one at a time.
    let mut trial = world.clone();
    for &(id, ..) in &blockers {
        trial.remove(id);
    }
    trial.add(Item::Track {
        shape: Segment::new(from, to, width),
        net: Some(net),
        layer,
        class,
    });

    // Each successful reroute is committed into `trial` immediately, so
    // the *next* blocker's own search correctly treats it as real
    // geometry to avoid -- these are simultaneous obstacles of the same
    // fixed target line, not independent one-off trials.
    let mut new_paths: Vec<(NetId, LayerId, Unit, Vec<Point>)> = Vec::with_capacity(blockers.len());
    for &(_, shape, blocker_net, blocker_layer) in &blockers {
        let path = crate::route_single_net(
            &trial,
            shape.a,
            shape.b,
            shape.width,
            blocker_net,
            blocker_layer,
            NetClass::C,
            resolver,
            outline,
        )?;
        for leg in path.windows(2) {
            trial.add(Item::Track {
                shape: Segment::new(leg[0], leg[1], shape.width),
                net: Some(blocker_net),
                layer: blocker_layer,
                class: NetClass::C,
            });
        }
        new_paths.push((blocker_net, blocker_layer, shape.width, path));
    }

    // Every blocker rerouted successfully -- apply the exact same
    // removal + rebuild to the real world. `world` was never touched
    // before this point.
    for &(id, ..) in &blockers {
        world.remove(id);
    }
    for (blocker_net, blocker_layer, blocker_width, path) in new_paths {
        for leg in path.windows(2) {
            world.add(Item::Track {
                shape: Segment::new(leg[0], leg[1], blocker_width),
                net: Some(blocker_net),
                layer: blocker_layer,
                class: NetClass::C,
            });
        }
    }

    Some(vec![from, to])
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::PadShape;
    use alladin_core::JlcpcbClearance;
    use alladin_geom::{Circle, MM};

    /// The one case this slice exists for: a single class-`C` track
    /// sits exactly on the direct line between `from` and `to`, with
    /// open space to route around it into.
    #[test]
    fn shoves_a_lone_blocking_track_off_the_direct_line() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(Item::Track {
            shape: Segment::new(Point::new(2 * MM, -3 * MM), Point::new(2 * MM, 3 * MM), 250_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let from = Point::new(0, 0);
        let to = Point::new(5 * MM, 0);
        assert!(
            !world.path_is_clear(from, to, 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver),
            "test setup: the direct line must actually be blocked before the shove is attempted"
        );

        let path = try_shove_blockers(
            &mut world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("a lone class-C blocker with open space around it must be shovable");

        assert_eq!(path, vec![from, to]);

        // The direct line is now genuinely clear against the *post-shove* world.
        assert!(world.path_is_clear(from, to, 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));

        // The blocker still exists, still belongs to net 2, but no
        // longer sits on its old straight line -- this is a shove, not
        // a silent deletion.
        let still_there = world
            .iter()
            .any(|item| matches!(item, Item::Track { net: Some(NetId(2)), .. }));
        assert!(still_there, "the blocker must still exist somewhere, just rerouted");
        let old_position_still_occupied = world.iter().any(|item| matches!(
            item,
            Item::Track { shape, net: Some(NetId(2)), .. } if shape.a == Point::new(2 * MM, -3 * MM) && shape.b == Point::new(2 * MM, 3 * MM)
        ));
        assert!(!old_position_still_occupied, "the blocker must have actually moved, not stayed put");
    }

    #[test]
    fn preserves_the_blockers_own_endpoints_exactly() {
        // The whole point of the reroute-based design: the blocker's
        // true endpoints (wherever they're anchored) never move, only
        // its interior path does.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        let blocker_a = Point::new(2 * MM, -3 * MM);
        let blocker_b = Point::new(2 * MM, 3 * MM);
        world.add(Item::Track {
            shape: Segment::new(blocker_a, blocker_b, 250_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        try_shove_blockers(
            &mut world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("must succeed (same scenario as shoves_a_lone_blocking_track_off_the_direct_line)");

        let net2_tracks: Vec<Segment> = world
            .iter()
            .filter_map(|item| match item {
                Item::Track { shape, net: Some(NetId(2)), .. } => Some(*shape),
                _ => None,
            })
            .collect();
        assert!(!net2_tracks.is_empty());

        let touches_a = net2_tracks.iter().any(|s| s.a == blocker_a || s.b == blocker_a);
        let touches_b = net2_tracks.iter().any(|s| s.a == blocker_b || s.b == blocker_b);
        assert!(touches_a, "the rerouted blocker must still start exactly at its original endpoint {blocker_a:?}");
        assert!(touches_b, "the rerouted blocker must still end exactly at its original endpoint {blocker_b:?}");
    }

    #[test]
    fn preserves_connectivity_to_a_same_net_sibling_leg() {
        // Two Track legs of net 2 sharing a vertex at (2mm, 3mm) -- the
        // vertical leg is what blocks the direct route. Slice 1's
        // rigid-translation design used to *refuse* this case outright
        // (translating would have torn the shared vertex apart); the
        // reroute-based design handles it for free, since the shared
        // vertex (2mm, 3mm) is one of the vertical leg's own unchanged
        // endpoints.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        let shared_vertex = Point::new(2 * MM, 3 * MM);
        world.add(Item::Track {
            shape: Segment::new(Point::new(2 * MM, -3 * MM), shared_vertex, 250_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });
        world.add(Item::Track {
            shape: Segment::new(shared_vertex, Point::new(4 * MM, 3 * MM), 250_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        try_shove_blockers(
            &mut world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("must now succeed: the shared vertex is one of the blocker's own unchanged endpoints");

        let sibling_still_attached = world.iter().any(|item| matches!(
            item,
            Item::Track { shape, net: Some(NetId(2)), .. }
                if shape.a == shared_vertex || shape.b == shared_vertex
        ));
        assert!(sibling_still_attached, "the sibling leg's shared vertex must still be touched by *something* of net 2");
    }

    #[test]
    fn preserves_connectivity_to_real_pads() {
        // The exact bug this module's design was rewritten to fix: a
        // simple, realistic pad-to-pad net (no sibling legs at all)
        // must still be connected to *both* its pads after being
        // shoved, not just floating near them.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        let pad_a_pos = Point::new(2 * MM, -3 * MM);
        let pad_b_pos = Point::new(2 * MM, 3 * MM);
        world.add(Item::Pad { shape: PadShape::Circle(Circle::new(pad_a_pos, 100_000)), net: Some(NetId(2)), layer: LayerId::FCu });
        world.add(Item::Pad { shape: PadShape::Circle(Circle::new(pad_b_pos, 100_000)), net: Some(NetId(2)), layer: LayerId::FCu });
        world.add(Item::Track {
            shape: Segment::new(pad_a_pos, pad_b_pos, 250_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        try_shove_blockers(
            &mut world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("a pad-to-pad net must still be shovable");

        let net2_tracks: Vec<Segment> = world
            .iter()
            .filter_map(|item| match item {
                Item::Track { shape, net: Some(NetId(2)), .. } => Some(*shape),
                _ => None,
            })
            .collect();
        assert!(net2_tracks.iter().any(|s| s.a == pad_a_pos || s.b == pad_a_pos), "must still reach pad A at {pad_a_pos:?}");
        assert!(net2_tracks.iter().any(|s| s.a == pad_b_pos || s.b == pad_b_pos), "must still reach pad B at {pad_b_pos:?}");
    }

    #[test]
    fn refuses_to_shove_a_pad() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(2 * MM, 0), 800_000)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });

        assert!(try_shove_blockers(
            &mut world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .is_none());
    }

    #[test]
    fn refuses_to_shove_a_class_a_track() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(Item::Track {
            shape: Segment::new(Point::new(2 * MM, -3 * MM), Point::new(2 * MM, 3 * MM), 250_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::A, // frozen -- must never be shoved
        });

        assert!(try_shove_blockers(
            &mut world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .is_none());
    }

    #[test]
    fn refuses_if_any_one_of_several_simultaneous_blockers_is_not_shovable() {
        // Three items on the direct line at once: the "Teil 20" blocker
        // in the middle, plus two long walls flanking it closely enough
        // to also cross the same horizontal direct line themselves.
        // Note what this test *doesn't* say any more (see "Teil 23" in
        // the development log): with the walls at `NetClass::C` as in
        // this scenario's original single-blocker-only version, the
        // multi-blocker code below would now correctly recognise all
        // three as shovable and successfully reroute every one of them
        // (removing the walls too frees the space the middle blocker
        // needed) -- a genuine capability improvement, not a bug. To
        // still test "all-or-nothing" refusal, the walls here are
        // `NetClass::A` (frozen, matches `refuses_to_shove_a_class_a_track`'s
        // single-blocker case) -- one non-shovable item among several
        // simultaneous blockers must still refuse the *entire* attempt
        // before touching anything, not just skip that one item.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(Item::Track {
            shape: Segment::new(Point::new(2 * MM, -3 * MM), Point::new(2 * MM, 3 * MM), 250_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });
        world.add(Item::Track {
            shape: Segment::new(Point::new(2 * MM + 400_000, -20 * MM), Point::new(2 * MM + 400_000, 20 * MM), 250_000),
            net: Some(NetId(3)),
            layer: LayerId::FCu,
            class: NetClass::A, // frozen -- disqualifies the whole multi-blocker attempt
        });
        world.add(Item::Track {
            shape: Segment::new(Point::new(2 * MM - 400_000, -20 * MM), Point::new(2 * MM - 400_000, 20 * MM), 250_000),
            net: Some(NetId(4)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        assert!(try_shove_blockers(
            &mut world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .is_none());
    }

    #[test]
    fn shoves_two_separate_lone_blockers_off_the_direct_line_at_once() {
        // "Teil 23": the actual new capability -- two *independent*
        // class-C tracks, each crossing the direct line at a different
        // point, each individually shovable into open space. The old
        // exactly-one-blocker restriction would have refused this
        // outright; the multi-blocker code must now clear both.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(Item::Track {
            shape: Segment::new(Point::new(1_500_000, -3 * MM), Point::new(1_500_000, 3 * MM), 250_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });
        world.add(Item::Track {
            shape: Segment::new(Point::new(3_500_000, -3 * MM), Point::new(3_500_000, 3 * MM), 250_000),
            net: Some(NetId(3)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let from = Point::new(0, 0);
        let to = Point::new(5 * MM, 0);
        let path = try_shove_blockers(
            &mut world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .expect("two independent, individually-shovable blockers must both be cleared");
        assert_eq!(path, vec![from, to]);

        assert!(world.path_is_clear(from, to, 250_000, Some(NetId(1)), LayerId::FCu, NetClass::C, &resolver));

        // Both original blockers must still exist, still on their own
        // nets, but neither still sitting on its old straight line.
        for (old_a, old_b, net) in [
            (Point::new(1_500_000, -3 * MM), Point::new(1_500_000, 3 * MM), NetId(2)),
            (Point::new(3_500_000, -3 * MM), Point::new(3_500_000, 3 * MM), NetId(3)),
        ] {
            let still_there = world.iter().any(|item| matches!(item, Item::Track { net: Some(n), .. } if *n == net));
            assert!(still_there, "blocker on net {net:?} must still exist somewhere, just rerouted");
            let old_position_still_occupied = world.iter().any(|item| matches!(
                item,
                Item::Track { shape, net: Some(n), .. } if *n == net && shape.a == old_a && shape.b == old_b
            ));
            assert!(!old_position_still_occupied, "blocker on net {net:?} must have actually moved, not stayed put");
        }
    }

    #[test]
    fn refuses_outright_with_more_than_the_simultaneous_blocker_budget() {
        // MAX_SIMULTANEOUS_BLOCKERS + 1 lone class-C blockers all
        // crossing the direct line at once -- refused before even
        // attempting a single reroute, regardless of whether each one
        // would individually have been shovable.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        for i in 0..(MAX_SIMULTANEOUS_BLOCKERS as i64 + 1) {
            let x = (i + 1) * 500_000;
            world.add(Item::Track {
                shape: Segment::new(Point::new(x, -3 * MM), Point::new(x, 3 * MM), 250_000),
                net: Some(NetId(100 + i as u32)),
                layer: LayerId::FCu,
                class: NetClass::C,
            });
        }

        assert!(try_shove_blockers(
            &mut world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .is_none());
    }

    #[test]
    fn refuses_the_whole_attempt_if_one_of_several_blockers_cannot_be_rerouted() {
        // Two blockers on the direct line: one (net 2, at x=1.5mm) with
        // open space to reroute into, the other (net 3, at x=3.5mm) with
        // one endpoint permanently sealed off by four pads of an
        // unrelated net (net 99) -- the same "box a point in on all
        // four sides, wide enough with clearance to leave no gap"
        // trick used elsewhere in this crate's tests, so
        // [`crate::route_single_net`] provably can never reach it, for
        // any path. The seal is placed well
        // away from y=0 (at y=3mm, not on the direct line itself) so
        // none of its four pads become a *third* blocker of the direct
        // line -- this scenario is specifically about a
        // shovable-looking blocker (an ordinary class-C track) whose
        // own reroute search still fails, not about a disqualified
        // blocker (already covered by
        // `refuses_if_any_one_of_several_simultaneous_blockers_is_not_shovable`).
        // Success on the first blocker alone must not be enough: the
        // whole attempt must refuse, and `world` must stay exactly as
        // it started (not even the first blocker's trial reroute leaks
        // through).
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(Item::Track {
            shape: Segment::new(Point::new(1_500_000, -3 * MM), Point::new(1_500_000, 3 * MM), 250_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let boxed_a = Point::new(3_500_000, -MM);
        let boxed_b = Point::new(3_500_000, 3 * MM); // sealed endpoint
        world.add(Item::Track {
            shape: Segment::new(boxed_a, boxed_b, 250_000),
            net: Some(NetId(3)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });
        for (dx, dy) in [(-1_000_000, 0), (1_000_000, 0), (0, -1_000_000), (0, 1_000_000)] {
            world.add(Item::Pad {
                shape: PadShape::Circle(Circle::new(Point::new(boxed_b.x + dx, boxed_b.y + dy), 900_000)),
                net: Some(NetId(99)),
                layer: LayerId::FCu,
            });
        }

        let from = Point::new(0, 0);
        let to = Point::new(5 * MM, 0);

        // Sanity check on the fixture itself: only the two tracks (not
        // any of the sealing pads) should collide with the direct line
        // -- otherwise this would accidentally be testing the
        // "disqualified blocker" path instead of "a shovable-looking
        // blocker whose reroute genuinely fails".
        let probe = Item::Track { shape: Segment::new(from, to, 250_000), net: Some(NetId(1)), layer: LayerId::FCu, class: NetClass::C };
        let blocker_nets: Vec<NetId> = world
            .query_colliding(&probe, &resolver)
            .into_iter()
            .filter_map(|id| world.get(id).and_then(Item::net))
            .collect();
        assert_eq!(
            blocker_nets.len(), 2,
            "fixture sanity check: only the two class-C tracks should collide with the direct line, got {blocker_nets:?}"
        );

        let before_len = world.len();
        let result = try_shove_blockers(
            &mut world, from, to, 250_000, NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        );
        assert!(result.is_none(), "the sealed-in second blocker must doom the whole multi-blocker attempt: {result:?}");
        assert_eq!(world.len(), before_len, "a fully-failed multi-blocker attempt must leave world untouched");
        let still_at_old_position = world.iter().any(|item| matches!(
            item,
            Item::Track { shape, net: Some(NetId(3)), .. } if shape.a == boxed_a && shape.b == boxed_b
        ));
        assert!(still_at_old_position, "the sealed-in blocker must not have moved at all");
        let first_blocker_still_at_old_position = world.iter().any(|item| matches!(
            item,
            Item::Track { shape, net: Some(NetId(2)), .. }
                if shape.a == Point::new(1_500_000, -3 * MM) && shape.b == Point::new(1_500_000, 3 * MM)
        ));
        assert!(
            first_blocker_still_at_old_position,
            "the first blocker's own successful trial reroute must not leak into `world` once the second blocker dooms the whole attempt"
        );
    }

    #[test]
    fn refuses_when_the_direct_line_itself_leaves_the_board_outline() {
        // Otherwise the exact same shovable setup as
        // `shoves_a_lone_blocking_track_off_the_direct_line`, but the
        // board outline is narrower than the route's own straight line
        // -- no shove of the blocker can ever fix that.
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        world.add(Item::Track {
            shape: Segment::new(Point::new(2 * MM, -3 * MM), Point::new(2 * MM, 3 * MM), 250_000),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
            class: NetClass::C,
        });

        let outline = vec![Polygon::new(vec![
            Point::new(-MM, -MM),
            Point::new(3 * MM, -MM), // board ends well short of `to` at 5mm
            Point::new(3 * MM, MM),
            Point::new(-MM, MM),
        ])];

        assert!(try_shove_blockers(
            &mut world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &outline,
        )
        .is_none());
    }

    #[test]
    fn world_is_left_untouched_when_a_shove_attempt_fails() {
        let mut world = Node::new();
        let resolver = JlcpcbClearance;

        let pad_id = world.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(2 * MM, 0), 800_000)),
            net: Some(NetId(2)),
            layer: LayerId::FCu,
        });

        let before = world.get(pad_id).unwrap().clone();
        assert!(try_shove_blockers(
            &mut world, Point::new(0, 0), Point::new(5 * MM, 0), 250_000,
            NetId(1), LayerId::FCu, NetClass::C, &resolver, &[],
        )
        .is_none());

        assert_eq!(world.len(), 1, "a failed shove attempt must not add or remove anything");
        let after = world.get(pad_id).unwrap().clone();
        match (before, after) {
            (Item::Pad { shape: s1, .. }, Item::Pad { shape: s2, .. }) => assert_eq!(s1, s2),
            _ => panic!("item kind changed across a failed shove attempt"),
        }
    }
}
