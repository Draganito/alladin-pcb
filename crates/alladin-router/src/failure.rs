//! [`FailureReason`]: a structured, actionable answer to "why couldn't
//! this net be routed", surfacing everywhere a plain `None`/"FAILED"
//! used to be the entire story (see the development log's "router
//! feedback" entry for the reasoning). Built so a human -- or an
//! upstream placement/netlisting AI -- has enough information to
//! actually *fix* the problem (move a component, widen a gap, place a
//! via at a specific spot before handing the board back to this
//! router) instead of only learning *that* something failed.
//!
//! Deliberately **not** a router feature in itself: Alladin's router
//! stays single-layer with no automatic via insertion -- placement and
//! netlisting (by a human or an AI) are responsible for ensuring every
//! net *can* be routed on one layer, vias included, before this router
//! ever sees it. [`FailureReason`] only diagnoses a already-failed
//! attempt; it never changes the board or suggests a via be inserted
//! automatically.
//!
//! Each variant is meant to point at a different fix:
//! [`FailureReason::EndpointBlocked`]/[`FailureReason::EndpointOffBoard`]
//! mean this *specific* net's own `from`/`to` was placed badly;
//! [`FailureReason::NoPathExists`]/[`FailureReason::SearchTooComplex`]
//! mean the endpoints themselves are fine, but the *surrounding* area
//! is either genuinely impassable or too crowded to search through in
//! practice.

use alladin_core::ItemId;
use alladin_geom::{Aabb, Point};

/// Which end of a routing request (`from` or `to`) a [`FailureReason`]
/// is about.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Endpoint {
    From,
    To,
}

/// See this module's own doc comment for the overall intent and how
/// each variant maps to a different fix.
#[derive(Debug, Clone)]
pub enum FailureReason {
    /// `endpoint` itself already collides with existing copper before
    /// any search even starts -- widening the search can never fix
    /// this, since every candidate edge leaving that point inherits the
    /// same collision (see [`crate::astar::find_path_astar`]'s own
    /// `endpoint_is_clear` doc comment for why this is checked first,
    /// as a cheap short-circuit, rather than only discovered after a
    /// full search). `blocking_items` names exactly what it collides
    /// with -- look them up via [`alladin_core::Node::get`].
    EndpointBlocked {
        endpoint: Endpoint,
        at: Point,
        blocking_items: Vec<ItemId>,
    },
    /// `endpoint` sits outside every polygon in the board outline --
    /// this net's own pad/waypoint was placed off-board (or no board
    /// outline reaches that far).
    EndpointOffBoard { endpoint: Endpoint, at: Point },
    /// Every reachable point in `region_searched` (the widest area the
    /// router tried -- generally the whole board outline, or an
    /// approximate area around `from`/`to` if no outline was supplied
    /// at all) was fully explored, and there genuinely is no
    /// single-layer path connecting the two endpoints: a real dead
    /// end, not a search shortfall. Needs either a layer change (a via,
    /// placed upstream of the router -- see this module's own doc
    /// comment) or more room carved out somewhere in `region_searched`.
    NoPathExists {
        region_searched: Aabb,
        candidate_points: usize,
        nearby_items: usize,
    },
    /// The candidate graph covering `region_searched` grew past what's
    /// practical to search exhaustively (see
    /// `crate::astar::MAX_FULL_FALLBACK_POINTS`/
    /// `crate::astar::MAX_TOTAL_CANDIDATE_POINTS_PER_STAGE`) before the
    /// router could ever *prove* no path exists. Unlike
    /// [`FailureReason::NoPathExists`], this means the router gave up
    /// for performance reasons, not because it proved a dead end --
    /// `nearby_items`/`candidate_points` measure exactly how crowded
    /// `region_searched` is, i.e. how much thinning out would help.
    SearchTooComplex {
        region_searched: Aabb,
        candidate_points: usize,
        nearby_items: usize,
    },
}
