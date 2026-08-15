//! Interactive "drag a trace" routing. This is a **manual** router: the
//! human steers with 45°-snapped legs; clearance is checked every frame
//! (`Node::path_is_clear` + board-edge margin). Docking onto a same-net
//! pad uses the same snapped stub as free-steering -- no A* search.
//! Obstacles must be steered around with corners.
//!
//! [`TraceDrag`] lets an existing segment be moved: neighbouring elbows
//! adapt, vertex count stays constant, other tracks are left alone.

use alladin_core::{Item, ItemId, JlcpcbDfm, LayerId, NetClass, NetId, ZoneConnection};
use alladin_geom::{segment_within_outline_with_clearance, Point, Polygon, Unit};

use crate::board_doc::{BoardDoc, PlacementError, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL};

/// Hobby-friendly default trace width (0.25mm) -- comfortably above
/// JLCPCB's absolute minimum (0.127mm) so a first-time user's traces
/// aren't fighting DFM limits by default. A per-net width override is
/// not offered yet.
pub const DEFAULT_TRACE_WIDTH: Unit = 250_000;

/// Whether every leg of `path` keeps JLCPCB's real `copper_to_routed_edge`
/// margin from the board outline, at trace width `width`.
pub(crate) fn path_keeps_edge_clearance(path: &[Point], width: Unit, outline: &[Polygon]) -> bool {
    path_keeps_edge_margin(path, width, outline, JlcpcbDfm::COPPER_TO_ROUTED_EDGE)
}

/// [`path_keeps_edge_clearance`] with a caller-chosen margin instead of
/// the hard [`JlcpcbDfm::COPPER_TO_ROUTED_EDGE`] fab minimum -- what the
/// MCP route gates use to enforce a comfort distance from the cut line
/// (see `mcp_routing::EDGE_COMFORT_MARGIN`) that a candidate can relax
/// per-call, but never below the fab minimum.
pub(crate) fn path_keeps_edge_margin(
    path: &[Point],
    width: Unit,
    outline: &[Polygon],
    margin: Unit,
) -> bool {
    path.windows(2)
        .all(|leg| segment_within_outline_with_clearance(leg[0], leg[1], width, margin, outline))
}

/// Snaps the direction from `from` to `cursor` onto the nearest clean
/// 45-degree-grid angle, returning the vertex/vertices *after* `from`
/// needed to get there: one point if `cursor` already lies on a clean
/// horizontal/vertical/diagonal ray from `from` (or `from == cursor`, in
/// which case the result is empty), two if a diagonal "elbow" is needed
/// to actually reach `cursor` -- the same 45-then-straight shape every
/// PCB CAD tool's own 45-degree interactive router draws. This never
/// fails to reach `cursor` exactly (the last point is always `cursor`
/// itself); it only ever changes the *route* there, never the
/// destination, so the live leg always ends right under the mouse.
fn snapped_legs(from: Point, cursor: Point) -> Vec<Point> {
    let dx = cursor.x - from.x;
    let dy = cursor.y - from.y;
    if dx == 0 && dy == 0 {
        return Vec::new();
    }
    let m = dx.abs().min(dy.abs());
    let elbow = Point::new(from.x + dx.signum() * m, from.y + dy.signum() * m);
    let mut legs = Vec::with_capacity(2);
    if elbow != from {
        legs.push(elbow);
    }
    if cursor != elbow {
        legs.push(cursor);
    }
    legs
}

/// The mirror-image elbow of [`snapped_legs`]: straight along the
/// dominant axis *first*, then a 45-degree diagonal into `cursor` --
/// the other of the two canonical "45-then-straight" shapes every PCB
/// CAD tool offers (KiCad flips between them with a keypress). Same
/// contract as [`snapped_legs`]: the last point is always `cursor`
/// itself, only the route there differs. Used purely as an automatic
/// fallback when [`snapped_legs`]'s diagonal-first variant is blocked
/// (see [`RoutingDrag::update`] / [`TraceDrag::update`]) -- the human
/// still steers, this just stops the live leg going red when the
/// mirrored elbow of the *same* human-chosen direction would be fine.
/// For a pure horizontal/vertical/diagonal drag both variants collapse
/// to the identical single leg, so callers skip the second clearance
/// check whenever the outputs match.
fn snapped_legs_axis_first(from: Point, cursor: Point) -> Vec<Point> {
    let dx = cursor.x - from.x;
    let dy = cursor.y - from.y;
    if dx == 0 && dy == 0 {
        return Vec::new();
    }
    let m = dx.abs().min(dy.abs());
    let elbow = Point::new(cursor.x - dx.signum() * m, cursor.y - dy.signum() * m);
    let mut legs = Vec::with_capacity(2);
    if elbow != from {
        legs.push(elbow);
    }
    if cursor != elbow {
        legs.push(cursor);
    }
    legs
}

/// Why [`RoutingDrag::drop_via_and_switch_layer`] refused a mid-route
/// via, surfaced to the UI so a rejected `V` keypress can say something
/// more useful than a single generic "can't place a via here" for two
/// genuinely different situations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DropViaError {
    /// There's no usable live leg to drop the via at right now (e.g.
    /// the drag was just started and hasn't moved yet, or the current
    /// leg itself is blocked) -- nothing to place a via *onto*.
    NoLiveRoute,
    /// A usable leg exists, but the via itself would be refused right
    /// there -- see [`PlacementError`].
    Via(PlacementError),
}

impl std::fmt::Display for DropViaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DropViaError::NoLiveRoute => write!(f, "no clear route here yet to drop a via onto"),
            DropViaError::Via(e) => write!(f, "can't place a via here: {e}"),
        }
    }
}

/// An in-progress drag from one pad, steered live by the human. Nothing
/// here touches `BoardDoc::node` until [`RoutingDrag::commit`] or
/// [`RoutingDrag::drop_via_and_switch_layer`] succeeds -- the same
/// "preview vs. commit" split every other drag in this editor
/// (footprint placement, footprint move) already uses.
pub struct RoutingDrag {
    pub from_pad: ItemId,
    from_point: Point,
    net: NetId,
    layer: LayerId,
    width: Unit,
    /// Diameter/drill for the via [`Self::drop_via_and_switch_layer`]
    /// places -- same "configurable at `start`-time, defaults to the
    /// module constants" relationship `width` above has to
    /// [`DEFAULT_TRACE_WIDTH`].
    via_diameter: Unit,
    via_drill: Unit,
    /// Corners the human has already fixed (see [`Self::fix_corner`]),
    /// oldest first, each entry the one or two points [`snapped_legs`]
    /// produced for that corner -- kept as one entry per fix, not a
    /// flattened point list, so [`Self::undo_last_corner`] can pop a
    /// whole corner (both points of a diagonal elbow) in one step
    /// rather than leaving a dangling half-corner behind.
    fixed: Vec<Vec<Point>>,
    /// The current live, snapped-angle leg(s) from [`Self::last_fixed_point`]
    /// towards wherever the cursor is right now -- empty while
    /// [`Self::hover_target`] is docked onto a pad (the live end is
    /// [`Self::preview`] instead, see its own doc comment), otherwise
    /// always exactly [`snapped_legs`]'s output for the current cursor.
    live_legs: Vec<Point>,
    /// Whether [`Self::live_legs`] is currently free of collisions *and*
    /// keeps the board-edge margin -- gates both [`Self::fix_corner`]
    /// and [`Self::drop_via_and_switch_layer`] while not docked onto a
    /// pad, and drives the live leg's red/green colouring in the UI.
    live_legs_clear: bool,
    /// Same-net pad under the cursor, if any -- gates [`Self::commit`].
    pub hover_target: Option<ItemId>,
    /// Clearance-clean snapped stub onto [`Self::hover_target`] (first
    /// point is [`Self::last_fixed_point`]), or `None`.
    pub preview: Option<Vec<Point>>,
    /// Stub clears copper but violates board-edge copper margin.
    pub edge_clearance_violation: bool,
}

impl RoutingDrag {
    /// The net this drag is routing on -- used by the UI to colour the
    /// live preview consistently with the ratsnest/pad colouring
    /// (`alladin_render::net_color`).
    pub fn net(&self) -> NetId {
        self.net
    }

    /// Where this drag currently originates from -- [`Self::from_pad`]'s
    /// center, or the last via dropped by [`Self::drop_via_and_switch_layer`]
    /// if one has been -- for the UI to know where to start drawing the
    /// fixed-corner path from.
    pub fn origin(&self) -> Point {
        self.from_point
    }

    /// Starts a drag from `from_pad`, or `None` if that pad has no net
    /// yet -- there is nothing to route *to* without one (see
    /// `board_doc.rs`'s `connect_pads`: assigning a net is a separate,
    /// prerequisite step, matching the project's "no schematic, the
    /// layout is the netlist" decision).
    /// (Production code goes through [`Self::start_with_options`] by
    /// now; this default-width shorthand survives for the extensive
    /// routing tests below.)
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn start(doc: &BoardDoc, from_pad: ItemId) -> Option<Self> {
        Self::start_with_width(doc, from_pad, DEFAULT_TRACE_WIDTH)
    }

    /// Same as [`Self::start`], but with an explicit trace width instead
    /// of always [`DEFAULT_TRACE_WIDTH`] -- the GUI's "Trace width"
    /// toolbar field goes through this. Vias this drag might later drop
    /// (see [`Self::drop_via_and_switch_layer`]) still use
    /// [`DEFAULT_VIA_DIAMETER`]/[`DEFAULT_VIA_DRILL`]; use
    /// [`Self::start_with_options`] to configure those too.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn start_with_width(doc: &BoardDoc, from_pad: ItemId, width: Unit) -> Option<Self> {
        Self::start_with_options(
            doc,
            from_pad,
            width,
            DEFAULT_VIA_DIAMETER,
            DEFAULT_VIA_DRILL,
        )
    }

    /// Same as [`Self::start_with_width`], but also lets the caller
    /// override the diameter/drill any mid-drag via-and-layer-switch
    /// (see [`Self::drop_via_and_switch_layer`]) will use, instead of
    /// always [`DEFAULT_VIA_DIAMETER`]/[`DEFAULT_VIA_DRILL`] -- the
    /// GUI's "Via" toolbar fields (`crate::app::EditorState::via_diameter`/
    /// `via_drill`) go through this.
    pub fn start_with_options(
        doc: &BoardDoc,
        from_pad: ItemId,
        width: Unit,
        via_diameter: Unit,
        via_drill: Unit,
    ) -> Option<Self> {
        let (from_point, layer, net) = doc.pad_endpoint(from_pad)?;
        Some(Self {
            from_pad,
            from_point,
            net: net?,
            layer,
            width,
            via_diameter,
            via_drill,
            fixed: Vec::new(),
            live_legs: Vec::new(),
            live_legs_clear: false,
            hover_target: None,
            preview: None,
            edge_clearance_violation: false,
        })
    }

    /// The point every live leg -- manual or docked -- currently starts
    /// from: the last corner the human fixed with [`Self::fix_corner`],
    /// or [`Self::from_point`] itself if none has been fixed yet.
    /// `pub(crate)` for `crate::app`'s preview painter, which anchors
    /// the "dock search still running" dashed placeholder line here.
    pub(crate) fn last_fixed_point(&self) -> Point {
        self.fixed
            .last()
            .and_then(|corner| corner.last().copied())
            .unwrap_or(self.from_point)
    }

    /// [`Self::from_point`] followed by every already-fixed corner,
    /// flattened -- the path prefix every commit/via-drop builds on top
    /// of.
    fn fixed_path(&self) -> Vec<Point> {
        std::iter::once(self.from_point)
            .chain(self.fixed.iter().flatten().copied())
            .collect()
    }

    /// Every already-fixed corner, flattened, for the UI to draw as
    /// solid "this is settled" legs -- [`Self::from_point`] itself is
    /// drawn separately (the starting pin's own ring), so it's
    /// deliberately not included here.
    pub fn fixed_points(&self) -> Vec<Point> {
        self.fixed.iter().flatten().copied().collect()
    }

    /// The current live leg(s): docked snapped stub if [`Self::hover_target`]
    /// is `Some`, otherwise the free-steered [`Self::live_legs`] -- plus
    /// whether that leg is currently legal to fix/commit/via onto. This
    /// is the one thing the UI needs to draw the "business end" of the
    /// drag, regardless of which mode produced it.
    pub fn live_end(&self) -> (&[Point], bool) {
        if self.hover_target.is_some() {
            match &self.preview {
                Some(path) => (&path[1..], true),
                None => (&[], false),
            }
        } else {
            (&self.live_legs, self.live_legs_clear)
        }
    }

    /// Whether an unblocked, un-docked live leg exists right now (used
    /// by the UI to decide whether Space would actually do anything).
    pub fn can_fix_corner(&self) -> bool {
        self.hover_target.is_none() && !self.live_legs.is_empty() && self.live_legs_clear
    }

    /// Whether at least one corner has already been fixed. (The UI
    /// calls `undo_last_corner` directly and lets it report failure;
    /// only the tests below query this predicate.)
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn can_undo_corner(&self) -> bool {
        !self.fixed.is_empty()
    }

    /// How many corners have been fixed so far -- for the UI's "N
    /// corner(s) fixed" status line; note this is *not*
    /// `self.fixed_points().len()`, since a single diagonal-elbow
    /// corner is two points but still only one fix.
    pub fn fixed_corner_count(&self) -> usize {
        self.fixed.len()
    }

    /// Recomputes the live end towards `cursor` every frame. While the
    /// cursor sits over a different same-net pad, docks with a snapped
    /// 45° stub onto that pad's center (same geometry as free-steer) if
    /// clearance + edge margin allow; otherwise free-steers
    /// [`Self::live_legs`]. Purely geometric -- no path search.
    pub fn update(&mut self, doc: &BoardDoc, cursor: Point) {
        let last = self.last_fixed_point();
        self.hover_target = doc
            .pad_at(cursor)
            .filter(|&id| id != self.from_pad && doc.pad_net(id) == Ok(Some(self.net)));

        if let Some(target) = self.hover_target {
            self.live_legs.clear();
            self.live_legs_clear = false;
            let target_point = doc.pad_center(target).unwrap_or(cursor);
            let try_stub = |stub: Vec<Point>| -> (Option<Vec<Point>>, bool, bool) {
                let mut path = vec![last];
                path.extend_from_slice(&stub);
                // Already sitting on the pad center: trivial dock.
                if stub.is_empty() {
                    return (Some(path), true, true);
                }
                let copper_ok = self.legs_copper_clear(doc, last, &stub);
                let edge_ok = path_keeps_edge_clearance(&path, self.width, &doc.outline);
                if copper_ok && edge_ok {
                    (Some(path), true, true)
                } else {
                    (None, copper_ok, edge_ok)
                }
            };
            let stub = snapped_legs(last, target_point);
            let (preview, copper_ok, edge_ok) = try_stub(stub.clone());
            if let Some(path) = preview {
                self.preview = Some(path);
                self.edge_clearance_violation = false;
                return;
            }
            let alt = snapped_legs_axis_first(last, target_point);
            if alt != stub {
                let (preview, copper_ok_alt, edge_ok_alt) = try_stub(alt);
                if let Some(path) = preview {
                    self.preview = Some(path);
                    self.edge_clearance_violation = false;
                    return;
                }
                self.preview = None;
                // Prefer reporting edge when copper was fine on either attempt.
                self.edge_clearance_violation =
                    (copper_ok && !edge_ok) || (copper_ok_alt && !edge_ok_alt);
            } else {
                self.preview = None;
                self.edge_clearance_violation = copper_ok && !edge_ok;
            }
        } else {
            self.preview = None;
            self.edge_clearance_violation = false;
            self.live_legs = snapped_legs(last, cursor);
            self.live_legs_clear = self.legs_are_clear(doc, last, &self.live_legs);
            if !self.live_legs_clear {
                let alt = snapped_legs_axis_first(last, cursor);
                if alt != self.live_legs && self.legs_are_clear(doc, last, &alt) {
                    self.live_legs = alt;
                    self.live_legs_clear = true;
                }
            }
        }
    }

    /// Copper clearance only (no board-edge check).
    fn legs_copper_clear(&self, doc: &BoardDoc, last: Point, legs: &[Point]) -> bool {
        if legs.is_empty() {
            return false;
        }
        let resolver = doc.resolver();
        let mut prev = last;
        for &p in legs {
            if !doc.node.path_is_clear(
                prev,
                p,
                self.width,
                Some(self.net),
                self.layer,
                NetClass::C,
                resolver,
            ) {
                return false;
            }
            prev = p;
        }
        true
    }

    /// Whether the straight leg(s) `last -> legs[0] -> ...` are clear of
    /// collisions and keep the board-edge margin. `false` for empty `legs`.
    fn legs_are_clear(&self, doc: &BoardDoc, last: Point, legs: &[Point]) -> bool {
        if legs.is_empty() {
            return false;
        }
        let mut path = Vec::with_capacity(legs.len() + 1);
        path.push(last);
        path.extend_from_slice(legs);
        self.legs_copper_clear(doc, last, legs)
            && path_keeps_edge_clearance(&path, self.width, &doc.outline)
    }

    /// Fixes the current live, free-steered leg(s) as a permanent corner.
    pub fn fix_corner(&mut self) -> bool {
        if !self.can_fix_corner() {
            return false;
        }
        self.fixed.push(std::mem::take(&mut self.live_legs));
        self.live_legs_clear = false;
        true
    }

    /// Un-fixes the most recently fixed corner.
    pub fn undo_last_corner(&mut self) -> bool {
        if self.fixed.pop().is_none() {
            return false;
        }
        self.live_legs.clear();
        self.live_legs_clear = false;
        true
    }

    /// Commits fixed corners plus the docked snapped stub. Refuses unless
    /// hover-docked with a clearance-clean [`Self::preview`].
    pub fn commit(&self, doc: &mut BoardDoc) -> bool {
        let Some(_target) = self.hover_target else {
            return false;
        };
        let Some(dock_path) = &self.preview else {
            return false;
        };
        let mut path = self.fixed_path();
        path.extend_from_slice(&dock_path[1..]);
        doc.try_add_track_path(&path, self.net, self.layer, self.width, NetClass::C)
            .is_ok()
    }

    /// Drops a via at the current live end, commits tracks up to it, flips layer.
    pub fn drop_via_and_switch_layer(&mut self, doc: &mut BoardDoc) -> Result<(), DropViaError> {
        let (live, clear) = self.live_end();
        if !clear || live.is_empty() {
            return Err(DropViaError::NoLiveRoute);
        }
        let via_center = *live.last().unwrap();

        let via_id = doc
            .try_add_via(via_center, self.net, self.via_diameter, self.via_drill)
            .map_err(DropViaError::Via)?;

        let mut path = self.fixed_path();
        path.extend_from_slice(live);
        if let Err(e) = doc.try_add_track_path(&path, self.net, self.layer, self.width, NetClass::C)
        {
            doc.node.remove(via_id);
            return Err(DropViaError::Via(e));
        }

        self.layer = match self.layer {
            LayerId::FCu => LayerId::BCu,
            LayerId::BCu => LayerId::FCu,
        };
        self.from_point = via_center;
        self.fixed.clear();
        self.hover_target = None;
        self.preview = None;
        self.live_legs.clear();
        self.live_legs_clear = false;
        self.edge_clearance_violation = false;
        Ok(())
    }

    /// Human-readable reason the current cursor can't be fixed/committed.
    pub fn blocked_reason(&self, _doc: &BoardDoc, _cursor: Point) -> Option<String> {
        if self.hover_target.is_some() {
            if self.preview.is_some() {
                return None;
            }
            if self.edge_clearance_violation {
                let mm = JlcpcbDfm::COPPER_TO_ROUTED_EDGE as f64 / alladin_geom::MM as f64;
                return Some(format!(
                    "final leg comes within {mm:.2}mm of the board edge"
                ));
            }
            return Some(
                "final leg onto this pin is blocked -- add corners to steer around obstacles"
                    .to_string(),
            );
        }
        if self.live_legs.is_empty() || self.live_legs_clear {
            return None;
        }
        Some("this leg collides with something or comes too close to the board edge".to_string())
    }
}

/// Whether `id` (an `Item::Track`/`Item::Via` on the same wire, or the
/// leg [`TraceDrag`] was started from itself) touches `point` on
/// `layer` -- an `Item::Via`'s center touches *both* copper layers
/// (that's the whole point of a via), so its own `layer` is ignored;
/// an `Item::Track` only touches on its own actual layer.
fn item_touches(doc: &BoardDoc, id: ItemId, point: Point, layer: LayerId) -> bool {
    match doc.node.get(id) {
        Some(Item::Track {
            shape, layer: l, ..
        }) => *l == layer && (shape.a == point || shape.b == point),
        Some(Item::Via { shape, .. }) => shape.center == point,
        _ => false,
    }
}

/// One end of [`TraceDrag::start`]'s grabbed leg, resolved to (a) the
/// point that end must stay pinned to no matter how far the drag goes,
/// and (b) the one extra leg (if any) that gets consumed/re-routed
/// along with it. Walks **at most one** neighbor: if exactly one other
/// item in `wire` touches `point` on `layer` and it's an `Item::Track`,
/// that leg's own *other* endpoint becomes the pinned point and the leg
/// itself is marked for removal; anything else -- nothing touching
/// (already a pad/wire end), an `Item::Via` (bridges layers but never
/// itself moves), or more than one neighbor (a branch/junction) -- means
/// `point` is already the pinned anchor, full stop, nothing consumed.
/// This one-neighbor cap is deliberate, not a shortcut: it keeps a drag
/// local (only the grabbed leg and its immediate neighbors ever move),
/// exactly what "click a segment, the rest of a long route stays put"
/// requires.
fn resolve_anchor(
    doc: &BoardDoc,
    wire: &[ItemId],
    leg_id: ItemId,
    point: Point,
    layer: LayerId,
) -> (Point, Option<ItemId>) {
    let mut touching = wire
        .iter()
        .copied()
        .filter(|&id| id != leg_id)
        .filter(|&id| item_touches(doc, id, point, layer));
    let Some(only) = touching.next() else {
        return (point, None);
    };
    if touching.next().is_some() {
        return (point, None); // a branch/junction: stop right here, don't guess which side to follow
    }
    match doc.node.get(only) {
        Some(Item::Track { shape, .. }) => {
            let far = if shape.a == point { shape.b } else { shape.a };
            (far, Some(only))
        }
        _ => (point, None), // a Via: bridges layers, but its own position is never part of a drag
    }
}

/// An in-progress "grab a trace segment and drag it" gesture: the
/// grabbed leg plus, on each side, at most one immediate neighbor leg
/// (see [`resolve_anchor`]) all get deleted and replaced by a fresh,
/// 45-degree-grid-snapped path from the left anchor through the cursor
/// to the right anchor -- reusing the exact same [`snapped_legs`]
/// [`RoutingDrag`] steers a live route with, just applied from *both*
/// ends towards the cursor instead of from one fixed start. Everything
/// further away on the wire (past either anchor) is never touched.
/// Same "preview vs. commit" split as [`RoutingDrag`]: nothing lands in
/// `BoardDoc::node` until [`Self::commit`] succeeds.
pub struct TraceDrag {
    net: NetId,
    layer: LayerId,
    width: Unit,
    /// Every `Item::Track`/`Item::Via` [`Self::commit`] will delete:
    /// the originally grabbed leg, plus whichever 0-2 neighbor legs
    /// [`resolve_anchor`] consumed on either side.
    to_remove: Vec<ItemId>,
    left_anchor: Point,
    right_anchor: Point,
    /// The last path [`Self::update`] computed from [`Self::left_anchor`]
    /// through the cursor to [`Self::right_anchor`] -- empty only before
    /// the first `update()` call. Kept even while [`Self::clear`] is
    /// `false` so a blocked drag still has something to draw (in red --
    /// same convention [`RoutingDrag::live_legs`] uses), rather than the
    /// preview just vanishing.
    path: Vec<Point>,
    /// Whether [`Self::path`] is fully clear of collisions and keeps the
    /// board-edge margin. Also `false` before the first `update()`.
    clear: bool,
}

impl TraceDrag {
    /// Starts a drag on `leg_id`, which must be an `Item::Track` (not a
    /// via -- there is no "drag a via" gesture here, a via's own
    /// position never moves, see [`resolve_anchor`]'s doc comment) that
    /// already has a net. `None` for anything else.
    pub fn start(doc: &BoardDoc, leg_id: ItemId) -> Option<Self> {
        let Some(Item::Track {
            shape,
            net: Some(net),
            layer,
            ..
        }) = doc.node.get(leg_id)
        else {
            return None;
        };
        let (net, layer, shape) = (*net, *layer, *shape);
        let wire = doc.connected_wire(leg_id);

        let (left_anchor, left_remove) = resolve_anchor(doc, &wire, leg_id, shape.a, layer);
        let (right_anchor, right_remove) = resolve_anchor(doc, &wire, leg_id, shape.b, layer);

        let mut to_remove = vec![leg_id];
        to_remove.extend(left_remove);
        to_remove.extend(right_remove);

        Some(Self {
            net,
            layer,
            width: shape.width,
            to_remove,
            left_anchor,
            right_anchor,
            path: Vec::new(),
            clear: false,
        })
    }

    pub fn net(&self) -> NetId {
        self.net
    }

    /// Every id [`Self::commit`] will delete -- so a live UI can hide
    /// them from its normal item rendering while the drag preview draws
    /// their replacement instead, the same "don't draw the original
    /// twice" treatment footprint dragging already gives the footprint
    /// being moved.
    pub fn removed_ids(&self) -> &[ItemId] {
        &self.to_remove
    }

    /// The current path and whether it's clear -- for live rendering,
    /// same shape as [`RoutingDrag::live_end`]. Empty/`false` before the
    /// first [`Self::update`].
    pub fn live(&self) -> (&[Point], bool) {
        (&self.path, self.clear)
    }

    /// [`Self::path`] if it's committable right now, `None` otherwise
    /// (either blocked, or before the first [`Self::update`]).
    fn preview(&self) -> Option<&[Point]> {
        (self.clear && self.path.len() >= 2).then_some(self.path.as_slice())
    }

    /// Recomputes [`Self::path`]/[`Self::clear`] for `cursor`:
    /// [`snapped_legs`] from each anchor towards `cursor` independently,
    /// joined at `cursor` itself (so both halves always meet exactly
    /// there, whatever angle each anchor's own geometry forces), then
    /// checked leg-by-leg against the whole board (same-net items,
    /// including every leg about to be deleted, never count as
    /// collisions against themselves or each other -- see
    /// `Node::is_colliding`'s own same-net fast path) and against the
    /// board-edge margin.
    ///
    /// When that default (diagonal-first on both halves) is blocked,
    /// the three remaining combinations of diagonal-first /
    /// straight-first elbows per half (see [`snapped_legs_axis_first`])
    /// are tried in turn and the first clear one wins -- same
    /// "mirror the elbow before going red" fallback
    /// [`RoutingDrag::update`] applies to its single free-steered leg,
    /// just per anchor half here. The cursor position itself is never
    /// second-guessed; a blocked drag still shows the default path in
    /// red.
    pub fn update(&mut self, doc: &BoardDoc, cursor: Point) {
        let assemble = |left_anchor: Point,
                        right_anchor: Point,
                        mut from_left: Vec<Point>,
                        mut from_right: Vec<Point>|
         -> Vec<Point> {
            from_right.pop(); // drop the duplicate `cursor` both halves end on
            let mut path = vec![left_anchor];
            path.append(&mut from_left);
            path.extend(from_right.into_iter().rev());
            path.push(right_anchor);
            path.dedup();
            path
        };
        let resolver = doc.resolver();
        let is_clear = |path: &[Point]| -> bool {
            path.len() >= 2
                && path.windows(2).all(|leg| {
                    doc.node.path_is_clear(
                        leg[0],
                        leg[1],
                        self.width,
                        Some(self.net),
                        self.layer,
                        NetClass::C,
                        resolver,
                    )
                })
                && path_keeps_edge_clearance(path, self.width, &doc.outline)
        };

        let half = |anchor: Point, axis_first: bool| -> Vec<Point> {
            if axis_first {
                snapped_legs_axis_first(anchor, cursor)
            } else {
                snapped_legs(anchor, cursor)
            }
        };

        let default_path = assemble(
            self.left_anchor,
            self.right_anchor,
            half(self.left_anchor, false),
            half(self.right_anchor, false),
        );
        if is_clear(&default_path) {
            self.clear = true;
            self.path = default_path;
            return;
        }
        for (left_axis_first, right_axis_first) in [(false, true), (true, false), (true, true)] {
            let path = assemble(
                self.left_anchor,
                self.right_anchor,
                half(self.left_anchor, left_axis_first),
                half(self.right_anchor, right_axis_first),
            );
            if path != default_path && is_clear(&path) {
                self.clear = true;
                self.path = path;
                return;
            }
        }
        self.clear = false;
        self.path = default_path;
    }

    /// Deletes every leg [`Self::start`] marked for removal and adds
    /// the current [`Self::preview`] in their place. Refuses (returns
    /// `false`, touching nothing) while no legal preview exists --
    /// releasing the mouse over a blocked position must leave the
    /// original trace exactly as it was, not commit a half-drawn mess.
    pub fn commit(&self, doc: &mut BoardDoc) -> bool {
        let Some(path) = self.preview() else {
            return false;
        };
        doc.replace_wire_segment(
            &self.to_remove,
            path,
            self.net,
            self.layer,
            self.width,
            NetClass::C,
        )
        .is_ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board_doc::NewBoardParams;
    use crate::footprint::builtin_templates;
    use alladin_core::{Item, PadShape};
    use alladin_geom::{Circle, Segment, MM};

    /// Two 2-pin THT footprints, 20mm apart on open board, with pad 0 of
    /// each already joined onto the same net -- the baseline every
    /// routing test in this module drags a trace across.
    fn two_footprints_connected() -> (BoardDoc, ItemId, ItemId) {
        let mut doc = NewBoardParams::default().create();
        let template = &builtin_templates()[0];
        doc.try_place_footprint(template, Point::new(-10 * MM, 0), 0.0)
            .unwrap();
        doc.try_place_footprint(template, Point::new(10 * MM, 0), 0.0)
            .unwrap();
        let pad_a = doc.footprints[0].pad_item_ids[0];
        let pad_b = doc.footprints[1].pad_item_ids[0];
        doc.connect_pads(pad_a, pad_b).unwrap();
        (doc, pad_a, pad_b)
    }

    #[test]
    fn start_returns_none_for_a_pad_without_a_net() {
        let mut doc = NewBoardParams::default().create();
        let template = &builtin_templates()[0];
        doc.try_place_footprint(template, Point::new(0, 0), 0.0)
            .unwrap();
        let pad = doc.footprints[0].pad_item_ids[0];
        assert!(RoutingDrag::start(&doc, pad).is_none());
    }

    #[test]
    fn start_succeeds_for_a_pad_with_a_net_and_has_no_live_leg_yet() {
        let (doc, pad_a, _pad_b) = two_footprints_connected();
        let drag = RoutingDrag::start(&doc, pad_a).expect("a connected pin must be routable from");
        assert_eq!(drag.from_pad, pad_a);
        let (live, _) = drag.live_end();
        assert!(live.is_empty(), "no live leg before the first update()");
    }

    #[test]
    fn snapped_legs_keeps_a_pure_horizontal_drag_as_a_single_leg() {
        let legs = snapped_legs(Point::new(0, 0), Point::new(5 * MM, 0));
        assert_eq!(legs, vec![Point::new(5 * MM, 0)]);
    }

    #[test]
    fn snapped_legs_keeps_a_pure_diagonal_drag_as_a_single_leg() {
        let legs = snapped_legs(Point::new(0, 0), Point::new(3 * MM, 3 * MM));
        assert_eq!(legs, vec![Point::new(3 * MM, 3 * MM)]);
    }

    #[test]
    fn snapped_legs_builds_a_45_then_straight_elbow_for_an_arbitrary_drag() {
        // Dominant axis is horizontal (5mm > 2mm): a 2mm diagonal leg,
        // then a 3mm horizontal leg to close the remaining distance.
        let from = Point::new(0, 0);
        let cursor = Point::new(5 * MM, 2 * MM);
        let legs = snapped_legs(from, cursor);
        assert_eq!(
            legs,
            vec![Point::new(2 * MM, 2 * MM), Point::new(5 * MM, 2 * MM)]
        );
        // The elbow leg must be a clean 45 degrees, the second leg
        // perfectly horizontal.
        let elbow = legs[0];
        assert_eq!((elbow.x - from.x).abs(), (elbow.y - from.y).abs());
        assert_eq!(legs[1].y, elbow.y);
    }

    #[test]
    fn snapped_legs_is_empty_when_the_cursor_has_not_moved() {
        let p = Point::new(3 * MM, -1 * MM);
        assert!(snapped_legs(p, p).is_empty());
    }

    #[test]
    fn snapped_legs_axis_first_builds_a_straight_then_45_elbow() {
        // Mirror of the diagonal-first case above: a 3mm horizontal leg
        // first, then the 2mm diagonal into the same cursor position.
        let from = Point::new(0, 0);
        let cursor = Point::new(5 * MM, 2 * MM);
        let legs = snapped_legs_axis_first(from, cursor);
        assert_eq!(legs, vec![Point::new(3 * MM, 0), cursor]);
        assert_eq!(
            legs[0].y, from.y,
            "the first leg must run straight along the dominant axis"
        );
        assert_eq!(
            (cursor.x - legs[0].x).abs(),
            (cursor.y - legs[0].y).abs(),
            "the second leg must be a clean 45 degrees"
        );
    }

    #[test]
    fn snapped_legs_axis_first_collapses_to_the_same_legs_for_clean_angles() {
        // Pure horizontal, pure diagonal, and a not-moved cursor have no
        // second elbow variant -- both functions must agree exactly, so
        // the fallback in `update` can skip the redundant re-check.
        let from = Point::new(0, 0);
        for cursor in [Point::new(5 * MM, 0), Point::new(3 * MM, 3 * MM), from] {
            assert_eq!(
                snapped_legs_axis_first(from, cursor),
                snapped_legs(from, cursor)
            );
        }
    }

    #[test]
    fn update_produces_a_clear_snapped_live_leg_over_open_board() {
        let (doc, pad_a, _pad_b) = two_footprints_connected();
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        drag.update(&doc, Point::new(-5 * MM, 3 * MM)); // open space, not a pad
        assert!(drag.hover_target.is_none());
        let (live, clear) = drag.live_end();
        assert!(!live.is_empty());
        assert!(clear, "an unobstructed leg over open board must be clear");
        assert!(drag.can_fix_corner());
    }

    #[test]
    fn update_falls_back_to_the_straight_first_elbow_when_the_diagonal_first_one_is_blocked() {
        let (mut doc, pad_a, _pad_b) = two_footprints_connected();
        let a = doc.pad_center(pad_a).unwrap();
        // Cursor 2mm right, 8mm up from the start pin. The default
        // diagonal-first elbow bends at a+(2,2) and its vertical leg
        // (x = a.x+2) then runs straight through a foreign-net pad
        // planted at a+(2,5). The straight-first mirror (vertical to
        // a+(0,6), then diagonal) stays >2mm clear of that pad.
        doc.node.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(a.x + 2 * MM, a.y + 5 * MM), 500_000)),
            net: Some(NetId(7777)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
            hole_diameter: None,
        });

        let cursor = Point::new(a.x + 2 * MM, a.y + 8 * MM);
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        drag.update(&doc, cursor);

        let (live, clear) = drag.live_end();
        assert!(
            clear,
            "the mirrored elbow must rescue this leg instead of going red"
        );
        assert_eq!(
            live,
            vec![Point::new(a.x, a.y + 6 * MM), cursor],
            "expected the straight-first elbow onto the exact same cursor position"
        );
        assert!(
            drag.can_fix_corner(),
            "the rescued leg must be fixable like any other clear leg"
        );
    }

    #[test]
    fn fix_corner_locks_the_live_leg_in_and_starts_a_fresh_one() {
        let (doc, pad_a, _pad_b) = two_footprints_connected();
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        drag.update(&doc, Point::new(-5 * MM, 3 * MM));
        let expected_corner =
            snapped_legs(doc.pad_center(pad_a).unwrap(), Point::new(-5 * MM, 3 * MM));
        assert!(
            drag.fix_corner(),
            "a clear, un-docked live leg must be fixable"
        );

        assert_eq!(drag.fixed_points(), expected_corner);
        assert!(
            !drag.can_fix_corner(),
            "the live leg is cleared right after fixing, until the next update()"
        );
        assert!(drag.can_undo_corner());

        // The next update() must steer on from the newly fixed corner,
        // not from the original start point.
        drag.update(&doc, Point::new(-5 * MM, 8 * MM));
        let (live, clear) = drag.live_end();
        assert!(clear);
        assert_eq!(
            live,
            vec![Point::new(-5 * MM, 8 * MM)],
            "a pure vertical continuation from the fixed corner"
        );
    }

    #[test]
    fn fix_corner_refuses_when_the_cursor_has_not_moved_yet() {
        let (doc, pad_a, _pad_b) = two_footprints_connected();
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        assert!(
            !drag.fix_corner(),
            "there is no live leg before the first update()"
        );
        assert!(drag.fixed_points().is_empty());
    }

    #[test]
    fn fix_corner_refuses_while_docked_onto_a_pad() {
        let (doc, pad_a, pad_b) = two_footprints_connected();
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        let pad_b_center = doc.pad_center(pad_b).unwrap();
        drag.update(&doc, pad_b_center);
        assert_eq!(drag.hover_target, Some(pad_b));
        assert!(
            !drag.fix_corner(),
            "docking onto a pad is finished by clicking it, not by fixing a corner onto it"
        );
    }

    #[test]
    fn undo_last_corner_pops_a_whole_elbow_not_just_one_point() {
        let (doc, pad_a, _pad_b) = two_footprints_connected();
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        // A diagonal drag that snaps to a two-point elbow.
        drag.update(&doc, Point::new(-5 * MM, 2 * MM));
        assert!(drag.fix_corner());
        assert!(!drag.fixed_points().is_empty());

        assert!(drag.undo_last_corner());
        assert!(
            drag.fixed_points().is_empty(),
            "undo must remove the entire fixed corner, both elbow points"
        );
        assert!(!drag.can_undo_corner());
    }

    #[test]
    fn undo_last_corner_refuses_when_nothing_has_been_fixed() {
        let (doc, pad_a, _pad_b) = two_footprints_connected();
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        assert!(!drag.undo_last_corner());
    }

    #[test]
    fn update_docks_onto_a_hovered_same_net_pad_with_a_snapped_preview() {
        // Single-pad footprints: snapped dock is a straight stub.
        let (doc, pad_a, pad_b) = two_tiny_pads_connected_at_y(0.0);
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        let pad_b_center = doc.pad_center(pad_b).unwrap();
        // Tiny test pads: hover the exact center so pad_at hits.
        drag.update(&doc, pad_b_center);

        assert_eq!(drag.hover_target, Some(pad_b));
        let path = drag
            .preview
            .as_ref()
            .expect("a clear, unobstructed line must be found");
        assert_eq!(
            *path.last().unwrap(),
            pad_b_center,
            "the preview must dock exactly onto the pad center"
        );
    }

    #[test]
    fn commit_adds_a_track_when_docked_with_a_valid_preview() {
        let (mut doc, pad_a, pad_b) = two_tiny_pads_connected_at_y(0.0);
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        let pad_b_center = doc.pad_center(pad_b).unwrap();
        drag.update(&doc, pad_b_center);

        let before = doc.node.len();
        assert!(
            drag.commit(&mut doc),
            "a docked, unobstructed drag must commit"
        );
        assert!(
            doc.node.len() > before,
            "commit must add at least one track item"
        );
    }

    #[test]
    fn commit_still_succeeds_across_a_different_net_full_board_solid_plane() {
        // Solid F.Cu pour of another net must not block same-net routing
        // (see `alladin_core::Node::item_collides`). Snapped dock only --
        // use single-pad footprints so the stub is unobstructed.
        let (mut doc, pad_a, pad_b) = two_tiny_pads_connected_at_y(0.0);
        let signal_net = doc.pad_net(pad_a).unwrap().unwrap();

        let plane_net = doc.create_net();
        let board_outline = doc.outline.clone();
        doc.add_zone(board_outline[0].clone(), LayerId::FCu, plane_net)
            .unwrap();
        assert!(
            doc.node
                .iter()
                .any(|item| matches!(item, Item::Zone { .. })),
            "the plane must have actually filled"
        );

        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        let pad_b_center = doc.pad_center(pad_b).unwrap();
        drag.update(&doc, pad_b_center);
        assert_eq!(
            drag.hover_target,
            Some(pad_b),
            "docking onto the target pad must be unaffected by the plane underneath it"
        );

        let before = doc.node.len();
        assert!(
            drag.commit(&mut doc),
            "a docked drag must still commit even straight across a different-net solid plane"
        );
        assert!(
            doc.node.iter().any(|item| matches!(item, Item::Track { net: Some(n), layer: LayerId::FCu, .. } if *n == signal_net)),
            "the committed track must actually be on the signal net, not swallowed by the plane"
        );
        assert!(
            doc.node.len() > before,
            "commit must add at least one track item"
        );
    }

    #[test]
    fn commit_includes_every_previously_fixed_corner() {
        let (mut doc, pad_a, pad_b) = two_footprints_connected();
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        drag.update(&doc, Point::new(-5 * MM, 5 * MM));
        assert!(drag.fix_corner());

        let pad_b_center = doc.pad_center(pad_b).unwrap();
        drag.update(&doc, pad_b_center);
        assert!(
            drag.commit(&mut doc),
            "a docked drag with a fixed corner behind it must still commit"
        );

        let tracks: Vec<_> = doc
            .node
            .iter()
            .filter(|item| matches!(item, Item::Track { .. }))
            .collect();
        assert!(
            tracks.len() >= 2,
            "expected at least one track leg before and one after the fixed corner, got {tracks:?}"
        );
    }

    #[test]
    fn commit_refuses_when_not_hovering_any_pad() {
        let (mut doc, pad_a, _pad_b) = two_footprints_connected();
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        drag.update(&doc, Point::new(0, 20 * MM)); // empty space

        let before = doc.node.len();
        assert!(!drag.commit(&mut doc));
        assert_eq!(
            doc.node.len(),
            before,
            "a refused commit must not touch the node"
        );
    }

    #[test]
    fn commit_refuses_towards_a_pad_with_no_net_assigned() {
        let (mut doc, pad_a, _pad_b) = two_footprints_connected();
        // The second footprint's *other* pin: same part as the routable
        // target, but never connected to any net.
        let unconnected_sibling = doc.footprints[1].pad_item_ids[1];
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        let sibling_center = doc.pad_center(unconnected_sibling).unwrap();
        drag.update(&doc, sibling_center);

        assert_eq!(
            drag.hover_target, None,
            "a no-net pad must not be treated as a dock target"
        );
        assert!(!drag.commit(&mut doc));
    }

    #[test]
    fn drop_via_and_switch_layer_places_a_via_at_the_requested_diameter_and_drill_not_the_default()
    {
        let (doc, pad_a, _pad_b) = two_footprints_connected();
        let custom_diameter: Unit = 800_000; // 0.8mm, != DEFAULT_VIA_DIAMETER
        let custom_drill: Unit = 400_000; // 0.4mm, != DEFAULT_VIA_DRILL
        let mut drag = RoutingDrag::start_with_options(
            &doc,
            pad_a,
            DEFAULT_TRACE_WIDTH,
            custom_diameter,
            custom_drill,
        )
        .unwrap();
        let mut doc = doc;
        drag.update(&doc, Point::new(0, 5 * MM));

        drag.drop_via_and_switch_layer(&mut doc)
            .expect("dropping a via on open space must succeed");

        let via = doc.node.iter().find_map(|item| {
            if let Item::Via { shape, drill, .. } = item {
                Some((shape.radius, *drill))
            } else {
                None
            }
        });
        let (radius, drill) = via.expect("a via must have been added");
        assert_eq!(
            radius,
            custom_diameter / 2,
            "the via must use the diameter start_with_options was given, not DEFAULT_VIA_DIAMETER"
        );
        assert_eq!(
            drill, custom_drill,
            "the via must use the drill start_with_options was given, not DEFAULT_VIA_DRILL"
        );
    }

    #[test]
    fn drop_via_and_switch_layer_commits_a_track_then_a_via_and_flips_layer() {
        let (mut doc, pad_a, pad_b) = two_footprints_connected();
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        assert_eq!(drag.layer, LayerId::FCu);
        // Off the pin row entirely (both footprints' pads sit on
        // y=0mm), so the snapped leg getting there doesn't clip the
        // start footprint's own *other*, unconnected pad along the way.
        let via_point = Point::new(0, 5 * MM);
        drag.update(&doc, via_point);
        let (_, clear) = drag.live_end();
        assert!(clear, "a clear line to the via point must exist first");

        let before = doc.node.len();
        assert!(
            drag.drop_via_and_switch_layer(&mut doc).is_ok(),
            "dropping a via on open space must succeed"
        );
        assert!(
            doc.node.len() > before,
            "must have added at least a track leg and a via"
        );
        assert!(
            doc.node.iter().any(|item| matches!(item, Item::Via { .. })),
            "a via must have been added"
        );
        assert!(
            doc.node
                .iter()
                .any(|item| matches!(item, Item::Track { net: Some(n), .. } if *n == drag.net())),
            "the leg leading up to the via must have been committed as a real track"
        );
        assert_eq!(
            drag.layer,
            LayerId::BCu,
            "the drag must continue on the other copper layer"
        );
        assert!(drag.fixed_points().is_empty(), "any prior fixed corners must be cleared after the via, since they're already committed");
        let (live, _) = drag.live_end();
        assert!(
            live.is_empty(),
            "the live leg must be cleared, forcing a fresh route on the new layer"
        );

        // The drag must still be usable afterwards, not just mechanically
        // switched: finishing the route to the original target proves the
        // second, post-via leg genuinely gets committed too.
        let pad_b_center = doc.pad_center(pad_b).unwrap();
        drag.update(&doc, pad_b_center);
        assert!(
            drag.commit(&mut doc),
            "the drag must still be able to finish routing after the layer switch"
        );
    }

    #[test]
    fn drop_via_and_switch_layer_refuses_without_an_active_live_leg() {
        let (mut doc, pad_a, _pad_b) = two_footprints_connected();
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap(); // no update() yet -- nothing live

        let before = doc.node.len();
        assert_eq!(
            drag.drop_via_and_switch_layer(&mut doc),
            Err(DropViaError::NoLiveRoute)
        );
        assert_eq!(
            doc.node.len(),
            before,
            "a refused via drop must not touch the node"
        );
        assert_eq!(
            drag.layer,
            LayerId::FCu,
            "a refused via drop must leave the layer unchanged"
        );
    }

    #[test]
    fn drop_via_and_switch_layer_refuses_a_via_too_close_to_the_edge_and_leaves_the_drag_unchanged()
    {
        let (mut doc, pad_a, _pad_b) = two_footprints_connected();
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        // 0.4mm from the board's y=+15mm edge (`NewBoardParams::default()`
        // is 50x30mm): comfortably enough for a 0.25mm-wide *track* leg
        // (needs 0.125mm half-width + 0.2mm margin = 0.325mm) but not for
        // the default 0.6mm-diameter *via* this call would try to drop
        // there (needs 0.3mm radius + 0.2mm margin = 0.5mm) -- exactly the
        // gap this method's own via-specific rejection has to catch, that
        // the track-only edge check `update()` already ran can't.
        let near_edge = Point::new(0, 15 * MM - 400_000);
        drag.update(&doc, near_edge);
        let (_, clear) = drag.live_end();
        assert!(clear, "the thin track leg itself must be a legal route");

        let before = doc.node.len();
        assert_eq!(
            drag.drop_via_and_switch_layer(&mut doc),
            Err(DropViaError::Via(PlacementError::OffBoard)),
            "a via that would violate the board edge margin must be refused"
        );
        assert_eq!(
            doc.node.len(),
            before,
            "a refused via drop must not add anything, not even the track leg leading to it"
        );
        assert_eq!(
            drag.layer,
            LayerId::FCu,
            "a refused via drop must leave the layer unchanged"
        );
        let (live, clear_after) = drag.live_end();
        assert!(
            clear_after && !live.is_empty(),
            "a refused via drop must leave the existing live leg intact"
        );
    }

    /// Two 1-pad footprints with a deliberately tiny 0.05mm pad radius
    /// (below JLCPCB's own `min_smd_pad_size`, but the "Add part" form's
    /// 0.125mm floor is a GUI-only guard -- `straight_row_template` has
    /// no such restriction, exactly like `builtin_templates` doesn't
    /// either) placed at `y`, on a 40mm-square, sharp-cornered board
    /// (outline at x/y = ±20mm). A tiny pad radius keeps *pad placement*
    /// legal under JLCPCB's *copper* edge margin (`radius +
    /// COPPER_TO_ROUTED_EDGE`) at a `y` where a 0.25mm-wide *trace*
    /// between them (`width/2 + COPPER_TO_ROUTED_EDGE` margin, a
    /// stricter number since `width/2` > pad radius here) would not be
    /// -- exactly the gap `path_keeps_edge_clearance` exists to close,
    /// that placement's own `check_placement` alone cannot.
    ///
    /// Placed via [`BoardDoc::insert_footprint_unchecked`], not
    /// [`BoardDoc::try_place_footprint`]: every `y` this helper is
    /// called with sits well inside JLCPCB's real, much stricter
    /// *body*-to-edge assembly margin too (`JlcpcbDfm::COMPONENT_BODY_TO_EDGE`,
    /// 2.5mm) -- correct and deliberate for these two synthetic,
    /// 0.05mm-radius test pads to fail if placed through the ordinary,
    /// fully-gated path, but irrelevant noise for what these specific
    /// tests exist to check (track/via-to-edge routing behaviour, not
    /// footprint placement DFM).
    fn two_tiny_pads_connected_at_y(y_mm: f64) -> (BoardDoc, ItemId, ItemId) {
        let mut doc = NewBoardParams {
            width_mm: 40.0,
            height_mm: 40.0,
            layer_count: crate::board_doc::LayerCount::Two,
            copper_weight: crate::board_doc::CopperWeight::OneOz,
            corner_radius_mm: 0.0,
        }
        .create();
        let template =
            crate::footprint::straight_row_template("tiny".into(), "P".into(), 1, 1.0, 0.05);
        let y = (y_mm * MM as f64).round() as Unit;
        doc.insert_footprint_unchecked(&template, "P1".into(), Point::new(-10 * MM, y), 0.0, &[]);
        doc.insert_footprint_unchecked(&template, "P2".into(), Point::new(10 * MM, y), 0.0, &[]);
        let pad_a = doc.footprints[0].pad_item_ids[0];
        let pad_b = doc.footprints[1].pad_item_ids[0];
        doc.connect_pads(pad_a, pad_b).unwrap();
        (doc, pad_a, pad_b)
    }

    #[test]
    fn update_rejects_a_dock_route_that_would_hug_the_board_edge_too_closely() {
        // y = 19.7mm -- 0.3mm from the top edge at y=20mm: clears the
        // tiny pads' own 0.25mm minimum but not a 0.25mm-wide trace's
        // stricter 0.325mm minimum.
        let (doc, pad_a, pad_b) = two_tiny_pads_connected_at_y(19.7);
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        let pad_b_center = doc.pad_center(pad_b).unwrap();
        drag.update(&doc, pad_b_center);

        assert_eq!(
            drag.hover_target,
            Some(pad_b),
            "still docks onto the target pad"
        );
        assert!(
            drag.preview.is_none(),
            "a route that violates the board-edge margin must not be offered as a preview"
        );
        assert!(
            drag.edge_clearance_violation,
            "the rejection must be attributed to the edge margin, not a generic routing failure"
        );
    }

    #[test]
    fn update_accepts_a_dock_route_that_clears_the_board_edge_margin() {
        // Same setup, but y = 19.6mm -- 0.4mm from the edge, comfortably
        // past the 0.325mm trace minimum. Guards against the new check
        // being over-eager.
        let (doc, pad_a, pad_b) = two_tiny_pads_connected_at_y(19.6);
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        let pad_b_center = doc.pad_center(pad_b).unwrap();
        drag.update(&doc, pad_b_center);

        assert!(
            drag.preview.is_some(),
            "0.4mm of edge clearance is well past the 0.325mm minimum"
        );
        assert!(!drag.edge_clearance_violation);
    }

    #[test]
    fn blocked_reason_names_the_board_edge_specifically_not_a_generic_routing_failure() {
        let (doc, pad_a, pad_b) = two_tiny_pads_connected_at_y(19.7);
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        let pad_b_center = doc.pad_center(pad_b).unwrap();
        drag.update(&doc, pad_b_center);

        let reason = drag
            .blocked_reason(&doc, pad_b_center)
            .expect("a rejected route must explain itself");
        assert!(
            reason.contains("board edge"),
            "expected the edge-specific message, got: {reason}"
        );
    }

    #[test]
    fn blocked_reason_names_a_blocked_free_steered_leg() {
        let (doc, pad_a, _pad_b) = two_tiny_pads_connected_at_y(19.7);
        let mut drag = RoutingDrag::start(&doc, pad_a).unwrap();
        // Straight towards the board center, but staying at the same
        // too-close y=19.7mm the whole way -- not hovering any pad, so
        // this exercises the free-steered leg's own edge check, not the
        // dock-preview one.
        let y = (19.7 * MM as f64).round() as Unit;
        drag.update(&doc, Point::new(0, y));
        let (_, clear) = drag.live_end();
        assert!(!clear, "test setup: the leg must actually be blocked");

        let reason = drag
            .blocked_reason(&doc, Point::new(0, 0))
            .expect("a blocked free-steered leg must explain itself");
        assert!(!reason.is_empty());
    }

    /// A three-leg wire (four points, `a_center -> p1 -> p2 -> b_center`)
    /// between the same two footprints [`two_footprints_connected`]
    /// places -- the baseline every [`TraceDrag`] test bends or drags
    /// the *middle* leg of. Returns the ids of the three legs in path
    /// order (`leg_a_p1`, `leg_p1_p2`, `leg_p2_b`).
    fn three_leg_wire() -> (BoardDoc, ItemId, ItemId, ItemId, Point, Point) {
        let (mut doc, pad_a, pad_b) = two_footprints_connected();
        let a_center = doc.pad_center(pad_a).unwrap();
        let b_center = doc.pad_center(pad_b).unwrap();
        let net = doc.pad_net(pad_a).unwrap().unwrap();
        let p1 = Point::new(-5 * MM, 0);
        let p2 = Point::new(5 * MM, 0);
        doc.add_track_path(
            &[a_center, p1, p2, b_center],
            net,
            LayerId::FCu,
            250_000,
            NetClass::C,
        );
        let mut legs: Vec<ItemId> = doc
            .node
            .iter_with_ids()
            .filter(|(_, i)| matches!(i, Item::Track { .. }))
            .map(|(id, _)| id)
            .collect();
        legs.sort_by_key(|&id| {
            let Some(Item::Track { shape, .. }) = doc.node.get(id) else {
                unreachable!()
            };
            shape.a.x.min(shape.b.x)
        });
        let (leg_a_p1, leg_p1_p2, leg_p2_b) = (legs[0], legs[1], legs[2]);
        (doc, leg_a_p1, leg_p1_p2, leg_p2_b, a_center, b_center)
    }

    #[test]
    fn start_on_the_middle_leg_of_a_three_leg_wire_pins_both_pad_centers_and_marks_all_three_legs_for_removal(
    ) {
        let (doc, leg_a_p1, leg_p1_p2, leg_p2_b, a_center, b_center) = three_leg_wire();
        let drag =
            TraceDrag::start(&doc, leg_p1_p2).expect("a track leg with a net must be draggable");
        assert_eq!(drag.left_anchor, a_center);
        assert_eq!(drag.right_anchor, b_center);
        assert_eq!(drag.to_remove.len(), 3);
        for id in [leg_a_p1, leg_p1_p2, leg_p2_b] {
            assert!(drag.to_remove.contains(&id));
        }
    }

    #[test]
    fn start_on_the_leg_touching_a_pad_pins_that_side_to_the_pad_itself_with_nothing_to_remove_there(
    ) {
        let (doc, leg_a_p1, leg_p1_p2, _leg_p2_b, a_center, _b_center) = three_leg_wire();
        let drag = TraceDrag::start(&doc, leg_a_p1).unwrap();
        assert_eq!(
            drag.left_anchor, a_center,
            "the pad end is already the pinned point"
        );
        assert_eq!(
            drag.to_remove.len(),
            2,
            "the grabbed leg plus its one neighbor, but nothing beyond the pad"
        );
        assert!(drag.to_remove.contains(&leg_a_p1));
        assert!(drag.to_remove.contains(&leg_p1_p2));
    }

    #[test]
    fn start_stops_at_a_via_without_marking_it_for_removal_or_moving_it() {
        let (mut doc, pad_a, pad_b) = two_footprints_connected();
        let a_center = doc.pad_center(pad_a).unwrap();
        let b_center = doc.pad_center(pad_b).unwrap();
        let net = doc.pad_net(pad_a).unwrap().unwrap();
        let via_point = Point::new(0, 5 * MM);
        // Via first, then stubs -- `try_add_via` refuses landing on a
        // track that is already there (same-net included).
        doc.try_add_via(via_point, net, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL)
            .unwrap();
        doc.add_track_path(
            &[a_center, via_point],
            net,
            LayerId::FCu,
            250_000,
            NetClass::C,
        );
        doc.add_track_path(
            &[via_point, b_center],
            net,
            LayerId::BCu,
            250_000,
            NetClass::C,
        );
        let leg_on_fcu = doc
            .node
            .iter_with_ids()
            .find(|(_, i)| {
                matches!(
                    i,
                    Item::Track {
                        layer: LayerId::FCu,
                        ..
                    }
                )
            })
            .unwrap()
            .0;
        let via_id = doc
            .node
            .iter_with_ids()
            .find(|(_, i)| matches!(i, Item::Via { .. }))
            .unwrap()
            .0;

        let drag = TraceDrag::start(&doc, leg_on_fcu).unwrap();
        assert_eq!(drag.left_anchor, a_center);
        assert_eq!(
            drag.right_anchor, via_point,
            "the via's own center pins this side, exactly where it already sits"
        );
        assert_eq!(
            drag.to_remove,
            vec![leg_on_fcu],
            "the via itself must never be a removal candidate"
        );
        assert!(!drag.to_remove.contains(&via_id));
    }

    #[test]
    fn update_joins_both_anchors_through_the_cursor_with_clean_snapped_halves() {
        let (doc, _leg_a_p1, leg_p1_p2, _leg_p2_b, a_center, b_center) = three_leg_wire();
        let mut drag = TraceDrag::start(&doc, leg_p1_p2).unwrap();
        let cursor = Point::new(0, 5 * MM);
        drag.update(&doc, cursor);
        let (path, clear) = drag.live();
        assert!(
            clear,
            "dragging straight down over open board must stay clear"
        );
        let path = path.to_vec();
        assert_eq!(*path.first().unwrap(), a_center);
        assert_eq!(*path.last().unwrap(), b_center);
        assert!(
            path.contains(&cursor),
            "the new path must actually pass through the dragged point"
        );
        // Every leg must itself be a clean 45/90-degree segment.
        for leg in path.windows(2) {
            let dx = (leg[1].x - leg[0].x).abs();
            let dy = (leg[1].y - leg[0].y).abs();
            assert!(
                dx == 0 || dy == 0 || dx == dy,
                "leg {:?} -> {:?} is not on the 45-degree grid",
                leg[0],
                leg[1]
            );
        }
    }

    #[test]
    fn update_refuses_a_cursor_position_that_would_collide_with_another_net() {
        let (mut doc, _leg_a_p1, leg_p1_p2, _leg_p2_b, ..) = three_leg_wire();
        let blocker_net = NetId(9999);
        doc.node.add(Item::Track {
            shape: Segment::new(Point::new(0, -10 * MM), Point::new(0, 10 * MM), 250_000),
            net: Some(blocker_net),
            layer: LayerId::FCu,
            class: NetClass::C,
        });
        let mut drag = TraceDrag::start(&doc, leg_p1_p2).unwrap();
        drag.update(&doc, Point::new(0, 5 * MM));
        let (_, clear) = drag.live();
        assert!(
            !clear,
            "dragging straight through an unrelated net's wall must be refused"
        );
    }

    #[test]
    fn trace_drag_update_falls_back_to_a_mirrored_elbow_half_when_the_default_is_blocked() {
        let (mut doc, _leg_a_p1, leg_p1_p2, _leg_p2_b, a_center, b_center) = three_leg_wire();
        // Cursor 6mm below the wire. The default (diagonal-first on
        // both halves) bends the right half at b_center + (-6, -6) --
        // plant a foreign-net pad exactly on that corner. The first
        // fallback combination (right half straight-first, bending at
        // b_center + (-2.73mm, 0) towards the cursor) stays ~2mm clear.
        let cursor = Point::new(0, -6 * MM);
        let blocked_corner = Point::new(b_center.x - 6 * MM, b_center.y - 6 * MM);
        doc.node.add(Item::Pad {
            shape: PadShape::Circle(Circle::new(blocked_corner, 400_000)),
            net: Some(NetId(7777)),
            layer: LayerId::FCu,
            zone_connection: ZoneConnection::Thermal,
            hole_diameter: None,
        });

        let mut drag = TraceDrag::start(&doc, leg_p1_p2).unwrap();
        drag.update(&doc, cursor);
        let (path, clear) = drag.live();
        assert!(
            clear,
            "the mirrored right-half elbow must rescue this drag instead of going red"
        );

        // Left half unchanged (diagonal-first), right half mirrored
        // (straight along x from b, then diagonal down into the cursor).
        let left_m = (cursor.x - a_center.x)
            .abs()
            .min((cursor.y - a_center.y).abs());
        let right_m = (cursor.x - b_center.x)
            .abs()
            .min((cursor.y - b_center.y).abs());
        let expected = vec![
            a_center,
            Point::new(a_center.x + left_m, cursor.y),
            cursor,
            Point::new(cursor.x + right_m, cursor.y + right_m),
            b_center,
        ];
        assert_eq!(
            path, expected,
            "expected the diagonal-first left half joined to the straight-first right half"
        );
    }

    #[test]
    fn commit_deletes_every_marked_leg_and_adds_exactly_the_previewed_path() {
        let (mut doc, leg_a_p1, leg_p1_p2, leg_p2_b, ..) = three_leg_wire();
        let mut drag = TraceDrag::start(&doc, leg_p1_p2).unwrap();
        drag.update(&doc, Point::new(0, 5 * MM));
        let (expected_path, clear) = drag.live();
        assert!(clear, "test setup: this drag must be legal");
        let expected_path = expected_path.to_vec();

        assert!(drag.commit(&mut doc));
        for id in [leg_a_p1, leg_p1_p2, leg_p2_b] {
            assert!(
                doc.node.get(id).is_none(),
                "every leg the drag consumed must be gone"
            );
        }
        let remaining: Vec<(Point, Point)> = doc
            .node
            .iter()
            .filter_map(|item| match item {
                Item::Track {
                    shape,
                    net: Some(n),
                    ..
                } if *n == drag.net() => Some((shape.a, shape.b)),
                _ => None,
            })
            .collect();
        assert_eq!(
            remaining.len(),
            expected_path.len() - 1,
            "one Item::Track per leg of the previewed path"
        );
        for leg in expected_path.windows(2) {
            assert!(
                remaining.contains(&(leg[0], leg[1])),
                "leg {:?} -> {:?} missing after commit",
                leg[0],
                leg[1]
            );
        }
    }

    #[test]
    fn commit_refuses_and_touches_nothing_while_no_preview_exists() {
        let (mut doc, _leg_a_p1, leg_p1_p2, _leg_p2_b, ..) = three_leg_wire();
        let drag = TraceDrag::start(&doc, leg_p1_p2).unwrap(); // never update()d: no preview yet
        let before = doc.node.len();
        assert!(
            !drag.commit(&mut doc),
            "committing before any update() must be refused"
        );
        assert_eq!(doc.node.len(), before);
    }
}
