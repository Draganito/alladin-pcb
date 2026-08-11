//! Board-space (internal nanometre units, see `alladin_geom`'s module doc
//! comment) <-> screen-pixel transform, plus every actual `egui::Painter`
//! draw call for rendering a board's static state. Nothing in here touches
//! routing logic at all -- it only ever reads [`alladin_core::Item`]/
//! [`alladin_geom::Polygon`] data already produced elsewhere (an imported
//! board, a live/replayed routing mirror, or the interactive `alladin-pcb`
//! editor's own live document).
//!
//! Extracted out of `alladin-viewer` (formerly its own `render` module) so
//! a second GUI crate (`alladin-pcb`, the interactive editor) can reuse the
//! exact same camera math and draw calls instead of duplicating ~250 lines
//! of already-tested code. `alladin-viewer`'s own live-routing overlay
//! (`draw_live_overlay`) stays here too since it's still generic board
//! rendering, just of transient search state rather than committed items.
//!
//! **Deliberately outline-only for zones/board outline, never filled.**
//! Two independent reasons, not just one: (1) `egui::Shape`'s only convex-
//! aware fill path is wrong for a genuinely non-convex polygon (a real
//! board outline's own concave notch, or an irregularly-shaped copper
//! pour), which would render visibly incorrect shapes; (2) a real zone
//! polygon can have tens of thousands of vertices -- re-tessellating
//! that into a fill every single frame is real, avoidable per-frame cost. A
//! stroked outline is correct for any polygon shape and cheap regardless of
//! vertex count, and still clearly shows an island's extent.

use alladin_core::{Item, LayerId, NetId};
use alladin_geom::{Aabb, Point, Polygon, MM};
use eframe::egui::{self, Color32, Painter, Pos2, Rect, Shape, Stroke};

#[derive(Clone, Copy)]
pub struct Camera {
    /// Board position, in millimetres, currently at the centre of the
    /// viewport.
    pub center_mm: egui::Vec2,
    pub pixels_per_mm: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self { center_mm: egui::Vec2::ZERO, pixels_per_mm: 3.0 }
    }
}

impl Camera {
    pub fn board_to_screen(&self, rect: Rect, p: Point) -> Pos2 {
        let mm = egui::vec2(p.x as f32 / MM as f32, p.y as f32 / MM as f32);
        rect.center() + (mm - self.center_mm) * self.pixels_per_mm
    }

    /// Inverse of [`Self::board_to_screen`]: a screen pixel position (e.g.
    /// a mouse click) back into board-space nanometre coordinates.
    pub fn screen_to_board(&self, rect: Rect, p: Pos2) -> Point {
        let mm = (p - rect.center()) / self.pixels_per_mm + self.center_mm;
        Point::new((mm.x as f64 * MM as f64).round() as i64, (mm.y as f64 * MM as f64).round() as i64)
    }

    pub fn screen_delta_to_board_mm(&self, delta: egui::Vec2) -> egui::Vec2 {
        delta / self.pixels_per_mm
    }

    /// Centres and scales the camera so `bounds` fills `rect` with a small
    /// margin -- the "fit to board" action.
    pub fn fit(&mut self, rect: Rect, bounds: Aabb) {
        let w_mm = ((bounds.max.x - bounds.min.x) as f32 / MM as f32).max(1.0);
        let h_mm = ((bounds.max.y - bounds.min.y) as f32 / MM as f32).max(1.0);
        const MARGIN: f32 = 1.15;
        let ppm = (rect.width() / (w_mm * MARGIN)).min(rect.height() / (h_mm * MARGIN));
        self.pixels_per_mm = ppm.clamp(0.02, 500.0);
        self.center_mm = egui::vec2(
            (bounds.min.x + bounds.max.x) as f32 / 2.0 / MM as f32,
            (bounds.min.y + bounds.max.y) as f32 / 2.0 / MM as f32,
        );
    }

    pub fn zoom_by(&mut self, factor: f32) {
        self.pixels_per_mm = (self.pixels_per_mm * factor).clamp(0.02, 500.0);
    }
}

pub struct LayerToggles {
    pub outline: bool,
    pub zones: bool,
    pub pads: bool,
    pub vias: bool,
    pub tracks: bool,
    pub back_layer: bool,
    /// Mechanical `Item::Hole`s (mounting holes, see
    /// `alladin_core::Item::Hole`'s own doc comment) -- kept as its own
    /// toggle rather than folded into [`Self::pads`] or [`Self::vias`]
    /// (even though it's drawn right alongside vias in
    /// [`draw_board`]'s own layer ordering) since a hole is neither: no
    /// copper, no net, nothing pad-shaped or via-shaped about it at
    /// all, and a user hiding pads/vias to declutter a routing-focused
    /// view still very much wants to see where the mechanical
    /// mounting holes are.
    pub holes: bool,
}

impl Default for LayerToggles {
    fn default() -> Self {
        Self { outline: true, zones: true, pads: true, vias: true, tracks: true, back_layer: true, holes: true }
    }
}

/// A stable, arbitrary-but-deterministic colour per net id, so the same net
/// always reads as the same colour across pads/tracks/vias/zones -- not
/// meant to match any particular KiCad net-class colour scheme, just to
/// make "which net is this" visually distinguishable at a glance.
pub fn net_color(net: Option<NetId>) -> Color32 {
    match net {
        None => Color32::from_gray(130),
        Some(NetId(id)) => {
            let hue = ((id.wrapping_mul(2_654_435_761) >> 16) & 0xff) as f32 / 255.0;
            egui::ecolor::Hsva::new(hue, 0.65, 0.9, 1.0).into()
        }
    }
}

pub fn layer_tint(layer: LayerId, color: Color32) -> Color32 {
    match layer {
        LayerId::FCu => color,
        LayerId::BCu => color.gamma_multiply(0.55),
    }
}

/// Dims `color` toward the background when `highlight` names a net
/// other than `item_net` -- the entire rendering side of `alladin-pcb`'s
/// "click a net in the side panel to highlight it on the board"
/// feature (see that crate's `EditorState::highlighted_net`). Returns
/// `color` completely unchanged whenever `highlight` is `None` (no
/// highlight active) or already matches `item_net`, so a highlighted
/// net's own items keep exactly their normal, undimmed colour -- the
/// contrast against everything else newly dimmed around it is what
/// makes it "pop", not any extra brightening of its own.
pub fn net_highlight_dim(color: Color32, item_net: Option<NetId>, highlight: Option<NetId>) -> Color32 {
    match highlight {
        Some(net) if item_net != Some(net) => color.gamma_multiply(0.18),
        _ => color,
    }
}

pub fn draw_polygon_outline(painter: &Painter, rect: Rect, camera: &Camera, poly: &Polygon, stroke: Stroke) {
    if poly.points.len() < 2 {
        return;
    }
    let mut pts: Vec<Pos2> = poly.points.iter().map(|&p| camera.board_to_screen(rect, p)).collect();
    pts.push(pts[0]);
    painter.add(Shape::line(pts, stroke));
}

/// Same as [`draw_polygon_outline`], but for a zone's own `outline`
/// specifically -- which (unlike a plain board-outline `Polygon`) may be
/// "keyholed": `alladin_geom::fill::FilledRegion::sealed`'s doc comment
/// explains why every hole a fill carves out (e.g. clearance around a
/// different-net pad sitting under the pour) gets spliced into the single
/// outer ring via a thin zero-width slit, walked once *out* to the hole
/// and once straight back along the *exact* same two points, so the
/// hole-bearing result still fits `Item::Zone`'s single hole-less
/// `outline: Polygon` field. A `NonZero`-rule *fill* would cancel a
/// "there and back" pair like that to nothing on its own; a naive stroke
/// of the raw point loop instead draws it as a real, visible line --
/// cutting straight across the pour to whatever unrelated, differently-
/// netted pad the hole happens to be carved around (reported as a stray
/// "connection" line that stayed put even with the ratsnest layer turned
/// off, since it isn't ratsnest at all). Do the same cancellation the
/// fill rule would have done, explicitly: drop every edge whose exact
/// reverse also occurs somewhere else in the same ring before stroking
/// what's left, so each hole's loop still renders as its own closed ring
/// and the bridge itself renders as nothing.
pub fn draw_zone_outline(painter: &Painter, rect: Rect, camera: &Camera, poly: &Polygon, stroke: Stroke) {
    for run in bridge_free_runs(&poly.points) {
        let pts: Vec<Pos2> = run.iter().map(|&p| camera.board_to_screen(rect, p)).collect();
        painter.add(Shape::line(pts, stroke));
    }
}

/// The actual bridge-detection-and-split from [`draw_zone_outline`]'s doc
/// comment, pulled out as a pure board-space function so it's testable
/// without a live [`Painter`]: walks `ring` (implicitly closed, i.e.
/// `ring[len-1]` connects back to `ring[0]`) and returns every maximal
/// run of consecutive points whose connecting edges are *not* a bridge --
/// each returned run is drawn as its own stroked polyline by the caller.
fn bridge_free_runs(ring: &[Point]) -> Vec<Vec<Point>> {
    let n = ring.len();
    if n < 2 {
        return Vec::new();
    }
    let edges: std::collections::HashSet<(Point, Point)> = (0..n).map(|i| (ring[i], ring[(i + 1) % n])).collect();

    let mut runs = Vec::new();
    let mut run: Vec<Point> = Vec::new();
    for i in 0..n {
        let a = ring[i];
        let b = ring[(i + 1) % n];
        if edges.contains(&(b, a)) {
            if run.len() > 1 {
                runs.push(std::mem::take(&mut run));
            } else {
                run.clear();
            }
            continue;
        }
        if run.is_empty() {
            run.push(a);
        }
        run.push(b);
    }
    if run.len() > 1 {
        runs.push(run);
    }
    runs
}

/// Draws the static/committed board state: outline, zones, tracks,
/// pads, vias -- in that order (each layer painted over the previous, so
/// pads/vias stay visible on top of a zone pour, matching how a real board
/// actually looks).
pub fn draw_board(
    painter: &Painter,
    rect: Rect,
    camera: &Camera,
    outline: &[Polygon],
    items: &[Item],
    layers: &LayerToggles,
    highlight_net: Option<NetId>,
) {
    if layers.outline {
        for poly in outline {
            draw_polygon_outline(painter, rect, camera, poly, Stroke::new(2.0, Color32::from_rgb(90, 210, 130)));
        }
    }

    if layers.zones {
        for item in items {
            if let Item::Zone { outline, layer, net } = item {
                if *layer == LayerId::BCu && !layers.back_layer {
                    continue;
                }
                let color = layer_tint(*layer, net_color(*net)).gamma_multiply(0.7);
                let color = net_highlight_dim(color, *net, highlight_net);
                draw_zone_outline(painter, rect, camera, outline, Stroke::new(1.0, color));
            }
        }
    }

    if layers.tracks {
        for item in items {
            if let Item::Track { shape, net, layer, .. } = item {
                if *layer == LayerId::BCu && !layers.back_layer {
                    continue;
                }
                let a = camera.board_to_screen(rect, shape.a);
                let b = camera.board_to_screen(rect, shape.b);
                let width_px = (shape.width as f32 / MM as f32 * camera.pixels_per_mm).max(1.0);
                let color = net_highlight_dim(layer_tint(*layer, net_color(*net)), *net, highlight_net);
                painter.line_segment([a, b], Stroke::new(width_px, color));
                // `egui::Painter::line_segment` renders a butt-capped
                // quad, not a rounded capsule -- harmless for a single
                // isolated leg, but a multi-leg track (every bend a
                // separate `Item::Track`) then shows a visible notch on
                // the bend's outer corner where two butt-capped
                // rectangles fail to fully cover each other, even
                // though the *collision* model for a track has always
                // been a true capsule (round ends, see
                // `alladin_geom::dist_point_to_segment`). Filling in a
                // round cap at each endpoint turns the rendering into
                // the same capsule the collision math already uses --
                // at a shared bend point, two legs' caps coincide and
                // seamlessly close the joint; at a track's own open
                // end, it now also matches the true rounded copper a
                // real fab would produce.
                let radius_px = width_px / 2.0;
                painter.circle_filled(a, radius_px, color);
                painter.circle_filled(b, radius_px, color);
            }
        }
    }

    if layers.pads {
        for item in items {
            if let Item::Pad { shape, net, layer, .. } = item {
                if *layer == LayerId::BCu && !layers.back_layer {
                    continue;
                }
                // Always a plain filled circle here, even for a
                // `PadShape::Polygon` pad (using its bounding radius) --
                // this crate's real pad-shape rendering lives in
                // `alladin-pcb::app::draw_pad_shape` instead (see that
                // call site's own doc comment: `layers.pads` is always
                // forced off before this function is ever called with a
                // pad in the picture, so this branch is a generic
                // fallback for callers that don't need per-pad
                // shape/rotation fidelity, not the interactive editor's
                // own rendering).
                let center = camera.board_to_screen(rect, shape.center());
                let radius_px = (shape.bounding_radius() as f32 / MM as f32 * camera.pixels_per_mm).max(1.0);
                let color = net_highlight_dim(layer_tint(*layer, net_color(*net)), *net, highlight_net);
                painter.circle_filled(center, radius_px, color);
            }
        }
    }

    if layers.vias {
        for item in items {
            if let Item::Via { shape, net, .. } = item {
                let center = camera.board_to_screen(rect, shape.center);
                let radius_px = (shape.radius as f32 / MM as f32 * camera.pixels_per_mm).max(1.0);
                let color = net_highlight_dim(net_color(*net).gamma_multiply(0.85), *net, highlight_net);
                painter.circle_filled(center, radius_px, color);
                painter.circle_stroke(center, radius_px, Stroke::new(1.0, Color32::BLACK));
            }
        }
    }

    if layers.holes {
        for item in items {
            if let Item::Hole { position, drill } = item {
                // Deliberately *unfilled* -- a real NPTH mounting hole
                // has no copper at all (see `Item::Hole`'s own doc
                // comment: no net, no annular ring), so a solid fill
                // like a pad's or via's own copper-colored disc would
                // misrepresent it as plated. A single plain grey ring
                // at the true drill radius (no second, larger "pad"
                // ring the way a via draws its copper *and* its own
                // drill) is the whole story for a hole: just a bare
                // mechanical opening through the board.
                let center = camera.board_to_screen(rect, *position);
                let radius_px = (*drill as f32 / 2.0 / MM as f32 * camera.pixels_per_mm).max(1.0);
                painter.circle_stroke(center, radius_px, Stroke::new(1.5, Color32::from_gray(190)));
            }
        }
    }
}

/// Draws the live/replayed routing overlay on top of [`draw_board`]'s
/// output: the current net's search corridor (a translucent rectangle) and,
/// once one is available, its resulting path (a highlighted polyline).
pub fn draw_live_overlay(
    painter: &Painter,
    rect: Rect,
    camera: &Camera,
    corridor: Option<Aabb>,
    path: Option<&[Point]>,
) {
    if let Some(c) = corridor {
        let min = camera.board_to_screen(rect, c.min);
        let max = camera.board_to_screen(rect, c.max);
        let region = Rect::from_two_pos(min, max);
        painter.rect_filled(region, 0.0, Color32::from_rgba_unmultiplied(255, 220, 0, 28));
        painter.rect_stroke(region, 0.0, Stroke::new(1.5, Color32::from_rgb(255, 200, 0)), egui::StrokeKind::Middle);
    }
    if let Some(path) = path {
        if path.len() >= 2 {
            let pts: Vec<Pos2> = path.iter().map(|&p| camera.board_to_screen(rect, p)).collect();
            painter.add(Shape::line(pts, Stroke::new(3.0, Color32::from_rgb(255, 70, 70))));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn net_highlight_dim_leaves_color_untouched_with_no_highlight_active() {
        let color = Color32::from_rgb(200, 60, 60);
        assert_eq!(net_highlight_dim(color, Some(NetId(1)), None), color);
        assert_eq!(net_highlight_dim(color, None, None), color, "even a netless item must be unaffected when nothing is highlighted");
    }

    #[test]
    fn net_highlight_dim_leaves_the_highlighted_nets_own_items_untouched() {
        let color = Color32::from_rgb(200, 60, 60);
        assert_eq!(net_highlight_dim(color, Some(NetId(1)), Some(NetId(1))), color);
    }

    #[test]
    fn net_highlight_dim_dims_every_other_net_and_netless_items_alike() {
        let color = Color32::from_rgb(200, 60, 60);
        let dimmed_other_net = net_highlight_dim(color, Some(NetId(2)), Some(NetId(1)));
        let dimmed_netless = net_highlight_dim(color, None, Some(NetId(1)));
        assert_ne!(dimmed_other_net, color, "a different net must actually be dimmed");
        assert_eq!(dimmed_other_net, dimmed_netless, "no-net and wrong-net must be dimmed identically");
    }

    #[test]
    fn fit_centers_the_bounds_at_the_rects_center() {
        let rect = Rect::from_min_size(Pos2::new(0.0, 0.0), egui::vec2(800.0, 600.0));
        let bounds = Aabb { min: Point::new(0, 0), max: Point::new(50 * MM, 30 * MM) };
        let mut camera = Camera::default();
        camera.fit(rect, bounds);

        let center_board = Point::new(25 * MM, 15 * MM);
        let screen = camera.board_to_screen(rect, center_board);
        assert!((screen.x - rect.center().x).abs() < 0.5, "expected board center at screen center, got {screen:?}");
        assert!((screen.y - rect.center().y).abs() < 0.5, "expected board center at screen center, got {screen:?}");
    }

    #[test]
    fn fit_scales_so_the_bounds_dont_overflow_the_rect() {
        let rect = Rect::from_min_size(Pos2::new(0.0, 0.0), egui::vec2(800.0, 600.0));
        let bounds = Aabb { min: Point::new(0, 0), max: Point::new(50 * MM, 30 * MM) };
        let mut camera = Camera::default();
        camera.fit(rect, bounds);

        let top_left = camera.board_to_screen(rect, bounds.min);
        let bottom_right = camera.board_to_screen(rect, bounds.max);
        assert!(top_left.x >= rect.left() - 1.0 && top_left.y >= rect.top() - 1.0);
        assert!(bottom_right.x <= rect.right() + 1.0 && bottom_right.y <= rect.bottom() + 1.0);
    }

    #[test]
    fn zoom_by_is_clamped_to_a_sane_range() {
        let mut camera = Camera::default();
        camera.zoom_by(1e9);
        assert!(camera.pixels_per_mm <= 500.0);
        camera.zoom_by(1e-9);
        assert!(camera.pixels_per_mm >= 0.02);
    }

    #[test]
    fn board_to_screen_moves_right_and_down_for_increasing_board_coordinates() {
        let rect = Rect::from_min_size(Pos2::new(0.0, 0.0), egui::vec2(800.0, 600.0));
        let camera = Camera { center_mm: egui::Vec2::ZERO, pixels_per_mm: 2.0 };
        let origin = camera.board_to_screen(rect, Point::new(0, 0));
        let right = camera.board_to_screen(rect, Point::new(10 * MM, 0));
        let down = camera.board_to_screen(rect, Point::new(0, 10 * MM));
        assert!(right.x > origin.x);
        assert!(down.y > origin.y);
    }

    #[test]
    fn default_layer_toggles_shows_every_layer_including_mounting_holes() {
        let layers = LayerToggles::default();
        assert!(layers.outline && layers.zones && layers.pads && layers.vias && layers.tracks && layers.back_layer && layers.holes);
    }

    #[test]
    fn bridge_free_runs_on_a_plain_hole_less_ring_returns_it_unchanged() {
        let square = vec![Point::new(0, 0), Point::new(10 * MM, 0), Point::new(10 * MM, 10 * MM), Point::new(0, 10 * MM)];
        let runs = bridge_free_runs(&square);
        assert_eq!(runs.len(), 1);
        let mut expected = square.clone();
        expected.push(square[0]);
        assert_eq!(runs[0], expected, "a ring with no coincident reversed edges must come back as a single run, closed back to its own start point");
    }

    #[test]
    fn bridge_free_runs_drops_a_keyhole_bridge_and_still_closes_the_holes_own_loop() {
        // The exact shape `alladin_geom::fill::seal_holes` produces for a
        // centered square hole: outer boundary spliced at its first point
        // with the hole's own ring, both bridge endpoints duplicated.
        let outer = [Point::new(-10 * MM, -10 * MM), Point::new(10 * MM, -10 * MM), Point::new(10 * MM, 10 * MM), Point::new(-10 * MM, 10 * MM)];
        let hole = [Point::new(-3 * MM, -3 * MM), Point::new(3 * MM, -3 * MM), Point::new(3 * MM, 3 * MM), Point::new(-3 * MM, 3 * MM)];
        let sealed: Vec<Point> = vec![
            outer[0], outer[1], outer[2], outer[3],
            // Bridge splice sits right after outer[0]/wraps to it -- mirror
            // `splice_hole`'s own "boundary[i], hole[j..], boundary[i]" shape.
        ];
        // Build it exactly the way `splice_hole` does: insert the bridge
        // right after `outer[0]` in the outer ring.
        let mut sealed = sealed;
        sealed.clear();
        sealed.push(outer[0]);
        sealed.push(hole[0]);
        sealed.push(hole[1]);
        sealed.push(hole[2]);
        sealed.push(hole[3]);
        sealed.push(hole[0]);
        sealed.push(outer[0]);
        sealed.push(outer[1]);
        sealed.push(outer[2]);
        sealed.push(outer[3]);

        let runs = bridge_free_runs(&sealed);
        assert_eq!(runs.len(), 2, "must split into the outer ring and the hole's own closed loop, with the bridge itself drawn nowhere");

        let hole_run = runs.iter().find(|r| r.contains(&hole[1])).expect("one run must be the hole's own loop");
        assert_eq!(hole_run.len(), 5, "hole loop is its own 4-edge closed ring: 4 distinct points plus the closing repeat");
        assert!(!hole_run.contains(&outer[1]), "the hole's run must never include an outer-boundary point");

        let outer_run = runs.iter().find(|r| r.contains(&outer[1])).expect("one run must be the outer boundary");
        assert!(!outer_run.contains(&hole[1]), "the outer run must never include a hole point");
    }

    #[test]
    fn bridge_free_runs_on_a_degenerate_two_point_sliver_draws_nothing() {
        // A ring collapsed entirely to a there-and-back sliver (both of
        // its two edges are each other's exact reverse) must vanish
        // completely -- there is no "real" boundary left once the bridge
        // pair cancels out.
        let sliver = vec![Point::new(0, 0), Point::new(5 * MM, 0)];
        assert!(bridge_free_runs(&sliver).is_empty());
    }

    #[test]
    fn screen_to_board_is_the_inverse_of_board_to_screen() {
        let rect = Rect::from_min_size(Pos2::new(0.0, 0.0), egui::vec2(800.0, 600.0));
        let camera = Camera { center_mm: egui::vec2(5.0, -3.0), pixels_per_mm: 4.0 };
        let original = Point::new(12 * MM, 7 * MM);
        let screen = camera.board_to_screen(rect, original);
        let back = camera.screen_to_board(rect, screen);
        assert!((back.x - original.x).abs() < 1_000, "round-trip x drifted: {back:?} vs {original:?}");
        assert!((back.y - original.y).abs() < 1_000, "round-trip y drifted: {back:?} vs {original:?}");
    }
}
