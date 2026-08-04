//! Alladin KiCad I/O: minimal `.kicad_pcb` importer and exporter.
//!
//! Builds on `alladin-sexpr`'s generic parser to add exactly the
//! KiCad-specific semantic knowledge needed to turn a `.kicad_pcb`
//! file's existing geometry into an `alladin_core::Node` -- pads (from
//! every `footprint`), already-routed tracks (`segment`), and vias
//! (`via`), plus the net id -> name table. This is the first piece of
//! Alladin that can load a *real* board instead of a hand-built demo
//! scene (see the development log's running "Weiterhin offen" list).
//!
//! [`export_appending_items`] is the other direction: takes a board's
//! original source text plus whatever new `Item::Track`/`Item::Via`
//! items `alladin-router` produced, and returns the file text with them
//! appended -- closing the loop so a routed result can actually be
//! written back out, not just held in memory. See that function's docs
//! for its own scope cuts.
//!
//! **Deliberate scope cuts** (each one documented at its point of use
//! below, in the same "state the simplification, don't hide it" style
//! as `alladin_core::JlcpcbClearance`'s doc comments):
//!
//! - Every pad shape (`rect`, `circle`, `oval`, `roundrect`, `custom`)
//!   is imported as a bounding *circle* of radius `max(size.w, size.h) /
//!   2`, matching every other Alladin item today (`Item::Pad`/`Item::Via`
//!   are circle-only). This can only ever make Alladin *more*
//!   conservative than the true pad outline, never less -- consistent
//!   with "Correct-by-Construction" never under-clearing.
//! - A pad's own local rotation (`(at dx dy pad_rot)`'s third value) is
//!   ignored -- irrelevant for a shape (a circle) that's rotationally
//!   symmetric about its own center. The *footprint's* rotation is not
//!   ignored: it correctly transforms every pad's position.
//! - The board outline (`Edge.Cuts` graphics) *is* read, chained into
//!   closed [`alladin_geom::Polygon`]s from both straight `gr_line` and
//!   curved `gr_arc` forms (see [`import_board_outline`]'s doc comment
//!   for the chaining algorithm, its "fails open" behaviour on an
//!   unclosable chain, and `gr_arc`'s chord-approximation trade-off) --
//!   but `gr_circle` (a fully round board, or a mounting-hole cutout
//!   drawn as a standalone circle) is not, and neither are
//!   footprint-level graphic items, groups, or 3D models -- none of the
//!   rest are obstacles/nets `alladin-core::Node` has a representation
//!   for yet.
//! - Already-filled copper zones/ground-pours (`(zone ...
//!   (filled_polygon ...))`) *are* imported, as static
//!   `alladin_core::Item::Zone` obstacles -- see [`import_zone`]'s doc
//!   comment for the exact scope (only the already-computed fill, not
//!   the zone's unfilled outline) and its documented limitations (no
//!   live re-fill as Alladin adds tracks, no thermal-relief spoke
//!   geometry, no zone-priority modelling between overlapping zones).
//! - [`import_kicad_pcb`] gives every imported (already-routed)
//!   `segment` a placeholder `NetClass::C` -- it only ever sees the
//!   board file, and (see `project` module docs for the full story of
//!   how this was discovered) **`.kicad_pcb` itself doesn't define real
//!   net classes** in current KiCad versions, whatever this crate's
//!   docs used to claim. [`import_kicad_project`] is the real answer:
//!   it also takes the sibling `.kicad_pro` project file and resolves
//!   each track's actual KiCad netclass (ground-truth verified) plus a
//!   best-effort guess at Alladin's own A/B/C tier (explicitly *not*
//!   ground-truth verified -- see `project` module docs).
//! - Units are assumed to be KiCad's default (millimetres); no support
//!   for a file-level unit override.
//! - Footprint rotation's sign/direction convention: **ground-truth
//!   verified** against `pcbnew`'s own Python API across 176 real pads
//!   in two real KiCad demo boards (including genuinely non-right-angle
//!   rotations) -- see `rotate`'s doc comment and
//!   the development log's "Teil 9" update. (This bullet used to say the
//!   opposite -- "has not yet been cross-checked" -- and the convention
//!   it described back then was, in fact, wrong; left as a reminder of
//!   why this whole crate leans on real-file validation rather than
//!   internal consistency alone.)

use alladin_core::{Item, LayerId, NetClass, NetId, Node, PadShape};
use alladin_geom::{cross2d, Circle, Point, Segment, Unit, MM};
use alladin_sexpr::{parse, ParseError, SExpr};
use std::collections::HashMap;
use thiserror::Error;

mod project;
pub use project::{write_kicad_pro, KicadNetClasses, ProjectParseError};

mod writer;
pub use writer::{
    import_footprints, write_kicad_pcb, ImportedFootprint, ImportedHole, ImportedPad, PadMount, WriteFootprint, WritePad, WritePadShape,
    WriteSilkDot, WriteSilkLine, WriteZone,
};

#[derive(Debug, Error)]
pub enum ImportError {
    #[error("failed to parse the file as an S-expression: {0}")]
    Parse(#[from] ParseError),
    #[error("root form is not a `(kicad_pcb ...)` list")]
    NotAKicadPcb,
    #[error("failed to parse the sibling `.kicad_pro` project file: {0}")]
    Project(#[from] ProjectParseError),
}

/// Result of a successful [`import_kicad_pcb`] call.
pub struct ImportedBoard {
    /// Every pad/track/via from the file, ready to route more nets
    /// against with `alladin-router`.
    pub node: Node,
    /// The file's `(net <id> "<name>")` table, id -> name. Alladin's own
    /// [`NetId`] reuses KiCad's net ordinals directly (no remapping), so
    /// this is purely for human-readable diagnostics/labelling.
    pub nets: HashMap<u32, String>,
    /// The physical board boundary, chained from `Edge.Cuts` graphic
    /// lines into closed polygons -- see [`import_board_outline`] for
    /// the chaining algorithm and its documented failure mode. Empty if
    /// the file has no `Edge.Cuts` geometry, or if what's there
    /// couldn't be chained into a closed loop.
    ///
    /// Multiple entries are meant to be combined with **even-odd**
    /// semantics (see [`alladin_geom::contains_point_evenodd`], not a
    /// plain "on the board if *any* polygon contains the point" OR) --
    /// this correctly handles both a board with genuinely separate
    /// allowed regions (e.g. a main body plus a protruding tab, each
    /// counted once, both routable) *and* a real internal cutout/slot
    /// (also `Edge.Cuts` geometry, also chains into its own closed
    /// polygon -- nested inside the main boundary, so a point inside it
    /// counts twice and is correctly excluded) *without* this importer
    /// ever having to work out which polygon is whose hole. Found to be
    /// necessary via a real board (`RaspberryPi-HAT.kicad_pcb`, which
    /// has exactly such a slot) once `gr_arc` support let its outline
    /// chain into closed polygons at all -- see the development log's
    /// "Teil 16"/"Teil 17" entries. Callers combining these polygons
    /// themselves (rather than via `alladin-router`, which already does
    /// this correctly) must use the even-odd helpers, not a plain
    /// `.iter().any(...)`, or they'll silently reintroduce this exact
    /// bug.
    pub outline: Vec<alladin_geom::Polygon>,
    /// Every top-level `(gr_text ...)` found on `F.SilkS`/`B.SilkS` --
    /// see [`ImportedSilkText`]. Alladin's own export bakes silk text
    /// to `(gr_line ...)` strokes ([`crate::WriteSilkLine`]) instead,
    /// so boards written by Alladin re-import with an empty list here;
    /// this recovers editable text from *external* KiCad boards that
    /// still use `gr_text`. A footprint-owned `fp_text` is *not*
    /// included.
    pub silk_texts: Vec<ImportedSilkText>,
}

/// One imported top-level silkscreen `gr_text` annotation from an
/// external KiCad board. `layer` reuses [`LayerId`] purely as "which
/// side" (`FCu` = `F.SilkS`, `BCu` = `B.SilkS`), matching
/// [`crate::WriteSilkLine::layer`].
#[derive(Debug, Clone, PartialEq)]
pub struct ImportedSilkText {
    pub text: String,
    pub position: Point,
    pub rotation_deg: f64,
    pub layer: LayerId,
    pub height: Unit,
    pub line_width: Unit,
}

/// Parsed-input fallbacks for [`ImportedSilkText::height`]/`line_width`
/// if a real file's `gr_text` is somehow missing its own `(effects
/// (font ...))` sub-form entirely (`kicad-cli`/`pcbnew` always write
/// one, so this should only ever matter for a hand-edited or
/// third-party-tool-written file) -- plain, readable literal defaults,
/// deliberately not shared with `alladin_pcb::board_doc`'s own
/// `DEFAULT_SILK_TEXT_HEIGHT`/`DEFAULT_SILK_LINE_WIDTH` constants
/// (this crate has no dependency on `alladin-pcb` at all, see this
/// module's doc comment).
const FALLBACK_SILK_TEXT_HEIGHT_MM: f64 = 1.0;
const FALLBACK_SILK_LINE_WIDTH_MM: f64 = 0.2;

/// Reads one top-level `(gr_text "..." (at x y [angle]) (layer "...")
/// (effects (font (size h h) (thickness t))))` form -- `None` if
/// `layer` isn't `F.SilkS`/`B.SilkS` (a `gr_text` on some other layer,
/// e.g. `Cmts.User`/`Dwgs.User`, isn't modelled here) or the form is
/// otherwise malformed (no text, no layer).
fn import_silk_text(form: &SExpr) -> Option<ImportedSilkText> {
    let text = form.tagged("gr_text")?.first().and_then(SExpr::text)?.to_string();
    let layer_name = form.child("layer").and_then(|l| l.tagged("layer")).and_then(|args| args.first().and_then(SExpr::text))?;
    let layer = match layer_name {
        "F.SilkS" => LayerId::FCu,
        "B.SilkS" => LayerId::BCu,
        _ => return None,
    };
    let (position, stored_rotation_deg) = at_with_rotation(form, "at");
    // Same negation `import_footprints`'s own `at` read undoes -- see
    // that function's doc comment for the ground-truthed sign
    // convention this mirrors.
    let rotation_deg = -stored_rotation_deg;

    let font = form.child("effects").and_then(|e| e.child("font"));
    let height = font
        .and_then(|f| f.child("size"))
        .and_then(|s| s.tagged("size"))
        .and_then(|args| args.first().and_then(SExpr::as_f64))
        .map(mm)
        .unwrap_or(mm(FALLBACK_SILK_TEXT_HEIGHT_MM));
    let line_width = font
        .and_then(|f| f.child("thickness"))
        .and_then(|t| t.tagged("thickness"))
        .and_then(|args| args.first().and_then(SExpr::as_f64))
        .map(mm)
        .unwrap_or(mm(FALLBACK_SILK_LINE_WIDTH_MM));

    Some(ImportedSilkText { text, position, rotation_deg, layer, height, line_width })
}

/// Parse and import a `.kicad_pcb` file's contents (already read into a
/// string -- this crate does no file I/O itself, matching every other
/// Alladin crate's "pure logic, no I/O" convention until the app layer
/// needs it).
///
/// Every imported track gets a placeholder `NetClass::C` -- a bare
/// `.kicad_pcb` file has no netclass information at all in current
/// KiCad versions (see this crate's module docs). Use
/// [`import_kicad_project`] instead if you also have the sibling
/// `.kicad_pro` project file and want real netclass-derived priorities.
pub fn import_kicad_pcb(source: &str) -> Result<ImportedBoard, ImportError> {
    import_kicad_pcb_impl(source, &|_net_name| NetClass::C)
}

/// Like [`import_kicad_pcb`], but also takes the board's sibling
/// `.kicad_pro` **project** file (JSON, already read into a string) to
/// resolve each imported track's real net class instead of the
/// placeholder `NetClass::C`. See the `project` module's docs for what
/// this can and can't do -- in short: which *KiCad* netclass a net
/// belongs to is ground-truth verified, but the further guess at which
/// of Alladin's own coarse A/B/C tiers that maps to is a documented
/// best-effort heuristic, not a derivation.
pub fn import_kicad_project(
    pcb_source: &str,
    project_source: &str,
) -> Result<ImportedBoard, ImportError> {
    let classes = KicadNetClasses::parse(project_source)?;
    import_kicad_pcb_impl(pcb_source, &|net_name| classes.alladin_class_of(net_name))
}

/// Parses `source` and returns *only* its `Edge.Cuts` outline (see
/// [`import_board_outline`]) -- the thin wrapper `crate::cli`'s
/// `set-outline --from-kicad` (in `alladin-pcb`) actually calls, so that
/// crate doesn't need `alladin-sexpr` as a direct dependency just to
/// parse a root form itself. Given a board's own already-full
/// [`import_kicad_pcb`] result, prefer reading `.outline` off that
/// instead of re-parsing the same source a second time.
pub fn import_outline_only(source: &str) -> Result<Vec<alladin_geom::Polygon>, ImportError> {
    let root = parse(source)?;
    if root.tagged("kicad_pcb").is_none() {
        return Err(ImportError::NotAKicadPcb);
    }
    Ok(import_board_outline(&root))
}

fn import_kicad_pcb_impl(
    source: &str,
    classify: &dyn Fn(&str) -> NetClass,
) -> Result<ImportedBoard, ImportError> {
    let root = parse(source)?;
    if root.tagged("kicad_pcb").is_none() {
        return Err(ImportError::NotAKicadPcb);
    }

    let mut nets = HashMap::new();
    for net_form in root.children("net") {
        if let Some(args) = net_form.tagged("net") {
            let id = args.first().and_then(SExpr::as_i64);
            let name = args.get(1).and_then(SExpr::text);
            if let (Some(id), Some(name)) = (id, name) {
                nets.insert(id as u32, name.to_string());
            }
        }
    }

    let mut node = Node::new();

    for footprint in root.children("footprint") {
        import_footprint(footprint, &mut node);
    }
    for segment in root.children("segment") {
        import_segment(segment, &mut node, &nets, classify);
    }
    for via in root.children("via") {
        import_via(via, &mut node);
    }
    for zone in root.children("zone") {
        import_zone(zone, &mut node);
    }

    let outline = import_board_outline(&root);
    let silk_texts = root.children("gr_text").filter_map(import_silk_text).collect();

    Ok(ImportedBoard { node, nets, outline, silk_texts })
}

/// Chains a file's `Edge.Cuts`-layer `gr_line` forms into closed board-
/// outline polygons.
///
/// **Why chaining is needed at all:** KiCad does not write board-outline
/// segments in walk order -- confirmed on a real file
/// (`interf_u.kicad_pcb`'s 9-segment outline, a non-convex rectangle
/// with a protruding tab): its `gr_line` forms appear in an order with
/// no relationship to the polygon's actual connectivity, and each
/// individual line's own `start`/`end` direction is arbitrary too (some
/// need to be walked forwards, others backwards, to form a connected
/// loop). This does the obvious greedy thing: repeatedly extend a
/// growing chain by finding *any* remaining edge sharing an endpoint
/// with the chain's current free end, in either direction, until the
/// chain closes back on its starting point or no matching edge remains.
///
/// `gr_arc` (curved outline segments, e.g. rounded corners) forms are
/// also chained in, approximated as a short polyline (see
/// [`arc_polyline`]'s own doc comment for the geometry and its honestly
/// documented chord-approximation trade-off) -- **`gr_circle` (a fully
/// round board, or a mounting-hole cutout drawn as a standalone circle)
/// is still not handled**, the one remaining gap.
///
/// **Documented failure mode, not hidden:** if the file's edges don't
/// chain into a fully closed loop (a gap, or `Edge.Cuts` geometry this
/// function doesn't understand, e.g. `gr_circle`), the unclosed partial
/// chain is silently dropped rather than returned as a bogus "closed"
/// polygon. This fails *open*: no outline is reported for that loop, so
/// nothing downstream will wrongly reject a route for leaving a
/// boundary Alladin failed to reconstruct. That is the opposite of this
/// codebase's usual "never under-clear" bias -- stated plainly here
/// because it's a real, asymmetric risk: a router that silently ignores
/// a board edge it couldn't parse is *less* safe than one that's merely
/// conservative. Verified against `interf_u.kicad_pcb`'s real 9-segment
/// (all-straight) outline: chains into exactly one closed 9-point
/// polygon, matching `pcbnew`'s own `BOARD::GetBoardPolygonOutlines()`
/// output exactly (same edge set; the starting vertex and winding
/// direction differ, which doesn't change what area the polygon covers)
/// -- see the development log's corresponding update for the
/// point-containment cross-check too. Verified again after `gr_arc`
/// support was added against a real rounded-corner board
/// (`RaspberryPi-HAT.kicad_pcb`, mixed `gr_line`/`gr_arc` outline,
/// previously the canonical "fails open, 0 polygons" example): now
/// chains into a real closed polygon instead -- see
/// the development log's "Teil 16" entry.
/// Maximum distance (internal units, i.e. nanometres) between two
/// graphic elements' endpoints for [`import_board_outline`]'s chaining
/// to treat them as "the same point", instead of requiring bit-exact
/// [`Point`] equality.
///
/// Needed for a real, measured reason, not defensive guessing: on
/// `RaspberryPi-HAT.kicad_pcb`'s real rounded-corner outline, a
/// `gr_arc`'s endpoint and the neighbouring `gr_line`'s matching
/// endpoint differ by about 0.9 micrometres (`100.5mm` on the line vs.
/// `100.499127mm` on the arc) -- almost certainly KiCad's own
/// floating-point arc-geometry rounding at the moment the file was
/// generated, not a parsing bug here. Exact equality (which is all
/// `interf_u.kicad_pcb`'s all-straight outline ever needed, since its
/// `gr_line` endpoints do match bit-for-bit) silently rejected that as
/// "not connected" and dropped the *entire* chain -- turning a real,
/// common shape (a rounded rectangle) back into the "fails open, 0
/// polygons" case this function's doc comment used to describe as
/// `gr_arc`'s whole reason for not being supported at all. 5 micrometres
/// is generous relative to that measured ~0.9 micrometre gap while
/// staying many orders of magnitude below any real board feature size
/// (minimum trace/spacing is measured in tens to low hundreds of
/// micrometres even on aggressive fabrication processes; mechanical
/// board features are mm-to-cm scale) -- so this can never bridge a
/// genuine gap in the outline, only absorb sub-micrometre float noise.
const CHAIN_TOLERANCE: Unit = 5_000; // 5 micrometres

fn points_match(a: Point, b: Point) -> bool {
    a.distance(b) <= CHAIN_TOLERANCE as f64
}

/// `pub`, not just an internal helper of [`import_kicad_pcb_impl`]: this
/// is also exactly what `crate::cli`'s `set-outline --from-kicad` (in
/// `alladin-pcb`) needs to lift *just* a reference file's `Edge.Cuts`
/// outline into an already-existing board, without importing the rest
/// of that file's footprints/tracks the way a full [`import_kicad_pcb`]
/// would.
pub fn import_board_outline(root: &SExpr) -> Vec<alladin_geom::Polygon> {
    let mut edges: Vec<(Point, Point)> = Vec::new();
    let is_edge_cuts = |form: &SExpr| {
        form.child("layer")
            .and_then(|l| l.tagged("layer"))
            .and_then(|args| args.first().and_then(SExpr::text))
            == Some("Edge.Cuts")
    };

    for line in root.children("gr_line") {
        if !is_edge_cuts(line) {
            continue;
        }
        if let (Some(start), Some(end)) = (point_of(line, "start"), point_of(line, "end")) {
            edges.push((start, end));
        }
    }
    for arc in root.children("gr_arc") {
        if !is_edge_cuts(arc) {
            continue;
        }
        if let (Some(start), Some(mid), Some(end)) =
            (point_of(arc, "start"), point_of(arc, "mid"), point_of(arc, "end"))
        {
            let polyline = arc_polyline(start, mid, end);
            edges.extend(polyline.windows(2).map(|w| (w[0], w[1])));
        }
    }

    let mut polygons = Vec::new();
    while let Some((a, b)) = edges.pop() {
        let mut chain = vec![a, b];
        loop {
            let free_end = *chain.last().unwrap();
            if chain.len() > 2 && points_match(free_end, chain[0]) {
                break; // closed the loop
            }
            let Some(idx) = edges
                .iter()
                .position(|&(s, e)| points_match(s, free_end) || points_match(e, free_end))
            else {
                break; // dead end: see doc comment on the "fails open" tradeoff
            };
            let (s, e) = edges.remove(idx);
            chain.push(if points_match(s, free_end) { e } else { s });
        }
        if chain.len() > 3 && points_match(*chain.last().unwrap(), chain[0]) {
            chain.pop(); // drop the duplicated closing point
            polygons.push(alladin_geom::Polygon::new(chain));
        }
        // else: unclosed chain, silently dropped (see doc comment).
    }
    polygons
}

/// How many straight sub-edges a `gr_arc` is approximated by. Unlike
/// `alladin-router`'s collision-boundary arc sampling (which needs a
/// safety factor pushing samples *outside* the true clearance radius --
/// see that crate's `ARC_SAFETY_FACTOR` -- getting an outline boundary
/// wrong by a chord-approximation sliver is not a DRC-critical error,
/// just a slightly-less-exact board shape, so a plain segment count with
/// no inflation is enough here.
const ARC_SEGMENTS: usize = 16;

/// Approximates a KiCad `gr_arc`'s three-point arc definition (`start`,
/// a point `mid` known to lie *on* the arc between them, and `end`) as a
/// polyline, for chaining into [`import_board_outline`]'s edge list the
/// same way a `gr_line` already is.
///
/// KiCad's current file format doesn't give a centre/radius/sweep-angle
/// directly (unlike its own pre-6 format) -- the actual circle has to be
/// reconstructed first via [`circumcircle`], the standard "circle
/// through three points" construction. `mid` is what disambiguates
/// *which* of the two arcs between `start` and `end` (the short way
/// around the circle, or the long way) is the real one: the correct arc
/// is whichever one's angular sweep actually passes through `mid`'s own
/// angle.
///
/// The returned polyline's first and last points are exactly the input
/// `start`/`end` (not recomputed from the fitted circle) so that a
/// neighbouring edge sharing that exact corner coordinate still chains
/// correctly -- only the interior points come from sampling the fitted
/// arc.
///
/// **Degenerate case:** if `start`, `mid`, `end` are exactly collinear
/// (checked with the same exact, rounding-free integer cross product
/// [`alladin_geom::cross2d`] the rest of this codebase's geometry relies
/// on for exactness -- not a floating-point epsilon guess, which would
/// be unreliable at these coordinate magnitudes), there is no well-defined
/// circle through them; the only sane reading of "a straight line with a
/// redundant collinear midpoint" is a straight line, so this returns
/// `[start, end]` directly.
///
/// **Known, honestly documented approximation, not hidden:** a chord
/// polyline of a *convex* arc (bulging away from the polygon's own
/// interior -- by far the common real case, e.g. every rounded
/// rectangle corner) always sits slightly *inside* the true curve, so
/// the reconstructed outline is a hair smaller than the real board --
/// the same safe direction this codebase already prefers elsewhere
/// (never claiming to be inside a boundary you're not actually inside
/// of). For the rarer *concave* arc (curving into the interior), the
/// bias flips the other way by the same tiny amount. Both are bounded by
/// [`ARC_SEGMENTS`] and shrink quickly as that count grows; a real fix
/// (exact arc-vs-segment containment checks throughout
/// `alladin_geom::Polygon`) is real follow-up work, not attempted here.
fn arc_polyline(start: Point, mid: Point, end: Point) -> Vec<Point> {
    let Some((center, radius)) = circumcircle(start, mid, end) else {
        return vec![start, end];
    };

    let angle_of = |p: Point| ((p.y - center.y) as f64).atan2((p.x - center.x) as f64);
    let a_start = angle_of(start);
    let a_mid = angle_of(mid);
    let a_end = angle_of(end);

    let normalize = |mut d: f64| {
        use std::f64::consts::PI;
        while d > PI {
            d -= 2.0 * PI;
        }
        while d <= -PI {
            d += 2.0 * PI;
        }
        d
    };

    let short = normalize(a_end - a_start);
    let mid_offset = normalize(a_mid - a_start);
    // The real sweep is the "short" delta only if `mid`'s own angle
    // actually falls within it; otherwise the arc really goes the long
    // way around the circle instead.
    let delta = if mid_offset.abs() <= short.abs() && mid_offset.signum() == short.signum() {
        short
    } else if short > 0.0 {
        short - std::f64::consts::TAU
    } else {
        short + std::f64::consts::TAU
    };

    let mut points = vec![start];
    for k in 1..ARC_SEGMENTS {
        let angle = a_start + delta * (k as f64 / ARC_SEGMENTS as f64);
        points.push(Point::new(
            center.x + (radius * angle.cos()).round() as Unit,
            center.y + (radius * angle.sin()).round() as Unit,
        ));
    }
    points.push(end);
    points
}

/// The unique circle passing through three points (the circumscribed
/// circle of the triangle they form), via the standard determinant
/// construction. `None` if the three points are exactly collinear (or
/// two of them coincide) -- there is no such circle in that case. The
/// collinearity test itself uses exact integer arithmetic
/// ([`alladin_geom::cross2d`]); only the actual centre/radius solve
/// needs floating point (an exact rational solution isn't worth the
/// complexity for a value that's immediately going to be sampled with
/// `cos`/`sin` anyway).
fn circumcircle(a: Point, b: Point, c: Point) -> Option<(Point, f64)> {
    if cross2d(a, b, c) == 0 {
        return None;
    }

    let (ax, ay) = (a.x as f64, a.y as f64);
    let (bx, by) = (b.x as f64, b.y as f64);
    let (cx, cy) = (c.x as f64, c.y as f64);

    let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    let a_sq = ax * ax + ay * ay;
    let b_sq = bx * bx + by * by;
    let c_sq = cx * cx + cy * cy;

    let center_x = (a_sq * (by - cy) + b_sq * (cy - ay) + c_sq * (ay - by)) / d;
    let center_y = (a_sq * (cx - bx) + b_sq * (ax - cx) + c_sq * (bx - ax)) / d;
    let center = Point::new(center_x.round() as Unit, center_y.round() as Unit);
    let radius = center.distance(a);
    Some((center, radius))
}

fn mm(v: f64) -> Unit {
    (v * MM as f64).round() as Unit
}

/// Reads a `(net <id> ["<name>"])` sub-form's id, mapping KiCad's
/// reserved net 0 ("no net") to `None` and everything else straight into
/// an Alladin [`NetId`] -- no remapping, KiCad's own ordinals are reused
/// as-is.
fn net_of(form: &SExpr) -> Option<NetId> {
    form.child("net")?
        .tagged("net")?
        .first()?
        .as_i64()
        .map(|id| id as u32)
        .filter(|&id| id != 0)
        .map(NetId)
}

/// Reads an `(<head> x y)`-shaped sub-form (`start`, `end`, or a
/// two-value `at`) as a millimetre point, converted to internal
/// nanometre units.
fn point_of(form: &SExpr, head: &str) -> Option<Point> {
    let args = form.child(head)?.tagged(head)?;
    let x = args.first()?.as_f64()?;
    let y = args.get(1)?.as_f64()?;
    Some(Point::new(mm(x), mm(y)))
}

/// Reads a footprint or pad's `(at x y [rot])` in one step: position (mm
/// -> nm) plus rotation in degrees (`0.0` if the third value is absent,
/// matching KiCad's own default).
fn at_with_rotation(form: &SExpr, head: &str) -> (Point, f64) {
    let args = form.child(head).and_then(|f| f.tagged(head)).unwrap_or(&[]);
    let x = args.first().and_then(SExpr::as_f64).unwrap_or(0.0);
    let y = args.get(1).and_then(SExpr::as_f64).unwrap_or(0.0);
    let rot = args.get(2).and_then(SExpr::as_f64).unwrap_or(0.0);
    (Point::new(mm(x), mm(y)), rot)
}

/// Rotate an already-nanometre-scaled offset by `deg` degrees, matching
/// KiCad's own footprint-rotation convention.
///
/// **This sign convention is now ground-truth verified**, not just
/// internally consistent: cross-checked against `pcbnew`'s own Python
/// API (`footprint.Pads()[i].GetPosition()`) on a real KiCad demo file
/// (`complex_hierarchy.kicad_pcb`, footprint `C103`, `(at 160.02 78.359
/// 90)`, pad "2" locally at `(at 15 0 90)`) -- see
/// the development log's corresponding update for the full story. The
/// first implementation used the standard-textbook counter-clockwise
/// rotation (`x' = x·cos θ - y·sin θ`, `y' = x·sin θ + y·cos θ`), which
/// this test proved *wrong*: it predicted pad 2 at
/// `(160.02, 93.359)` mm, but `pcbnew` reports the true absolute
/// position as `(160.02, 63.359)` mm -- the opposite sign on the
/// cross-terms. This function now matches that ground truth (see
/// `rotation_matches_pcbnew_ground_truth_on_a_real_kicad_footprint`
/// below).
fn rotate(offset: Point, deg: f64) -> Point {
    if deg == 0.0 {
        return offset;
    }
    let rad = deg.to_radians();
    let (sin, cos) = rad.sin_cos();
    let dx = offset.x as f64;
    let dy = offset.y as f64;
    Point::new(
        (dx * cos + dy * sin).round() as Unit,
        (-dx * sin + dy * cos).round() as Unit,
    )
}

fn layer_matches(token: &str, target: &str) -> bool {
    token == target || token == "*.Cu"
}

fn pad_layers(pad: &SExpr) -> (bool, bool) {
    let tokens: Vec<&str> = pad
        .child("layers")
        .and_then(|l| l.tagged("layers"))
        .map(|args| args.iter().filter_map(SExpr::text).collect())
        .unwrap_or_default();
    (
        tokens.iter().any(|t| layer_matches(t, "F.Cu")),
        tokens.iter().any(|t| layer_matches(t, "B.Cu")),
    )
}

fn import_footprint(footprint: &SExpr, node: &mut Node) {
    let (origin, rotation) = at_with_rotation(footprint, "at");

    for pad in footprint.children("pad") {
        let (local_offset, _pad_rotation) = at_with_rotation(pad, "at");
        let placed = origin.add(rotate(local_offset, rotation));

        // A genuine unplated mechanical hole (`np_thru_hole`, see
        // `alladin_core::Item::Hole`'s own doc comment) has no net and
        // no copper at all -- reading it as an ordinary `Item::Pad`
        // (this flat `Node`'s only other option) would wrongly let a
        // route land on it as if it were a real electrical connection.
        // `writer::import_footprints`'s own structured import path
        // makes the exact same `np_thru_hole` -> hole distinction; see
        // that function's doc comment for why this crate keeps *two*
        // separate import paths at all.
        let mount = pad.tagged("pad").and_then(|a| a.get(1)).and_then(SExpr::text);
        if mount == Some("np_thru_hole") {
            let drill = pad
                .child("drill")
                .and_then(|d| d.tagged("drill"))
                .and_then(|args| args.first().and_then(SExpr::as_f64))
                .map(mm)
                .unwrap_or(0);
            node.add(Item::Hole { position: placed, drill });
            continue;
        }

        let radius = pad
            .child("size")
            .and_then(|s| s.tagged("size"))
            .map(|args| {
                let w = args.first().and_then(SExpr::as_f64).unwrap_or(0.0);
                let h = args.get(1).and_then(SExpr::as_f64).unwrap_or(w);
                mm(w.max(h) / 2.0)
            })
            .unwrap_or(0);

        // Always a bounding circle here, never the pad's true polygon
        // shape (`alladin_core::PadShape::Polygon`) -- this flat,
        // per-footprint-structure-free `Node` is exactly right for
        // `alladin-router`'s own examples/routing obstacles, but
        // `alladin-pcb::kicad_import` (the interactive editor's real
        // import path) discards every `Item::Pad` this function
        // produces and rebuilds true pad geometry itself from
        // `import_footprints`'s full per-pad shape/rotation data
        // instead -- see that module's doc comment.
        let circle = Circle::new(placed, radius);
        let net = net_of(pad);
        let (front, back) = pad_layers(pad);

        if front {
            node.add(Item::Pad { shape: PadShape::Circle(circle), net, layer: LayerId::FCu });
        }
        if back {
            node.add(Item::Pad { shape: PadShape::Circle(circle), net, layer: LayerId::BCu });
        }
        if !front && !back {
            // Missing/unrecognised layer list: assume front copper
            // rather than silently dropping a real obstacle.
            node.add(Item::Pad { shape: PadShape::Circle(circle), net, layer: LayerId::FCu });
        }
    }
}

fn import_segment(
    segment: &SExpr,
    node: &mut Node,
    nets: &HashMap<u32, String>,
    classify: &dyn Fn(&str) -> NetClass,
) {
    let start = point_of(segment, "start").unwrap_or(Point::new(0, 0));
    let end = point_of(segment, "end").unwrap_or(Point::new(0, 0));
    let width = segment
        .child("width")
        .and_then(|w| w.tagged("width"))
        .and_then(|args| args.first().and_then(SExpr::as_f64))
        .map(mm)
        .unwrap_or(0);
    let layer_name = segment
        .child("layer")
        .and_then(|l| l.tagged("layer"))
        .and_then(|args| args.first().and_then(SExpr::text))
        .unwrap_or("F.Cu");
    let layer = if layer_name == "B.Cu" { LayerId::BCu } else { LayerId::FCu };
    let net = net_of(segment);
    // Unconnected ("no net") tracks have no name to classify by --
    // `NetClass::C` (lowest priority) is the only sensible default.
    let class = net
        .and_then(|n| nets.get(&n.0))
        .map(|name| classify(name))
        .unwrap_or(NetClass::C);

    node.add(Item::Track {
        shape: Segment::new(start, end, width),
        net,
        layer,
        class,
    });
}

fn import_via(via: &SExpr, node: &mut Node) {
    let at = point_of(via, "at").unwrap_or(Point::new(0, 0));
    let radius = via
        .child("size")
        .and_then(|s| s.tagged("size"))
        .and_then(|args| args.first().and_then(SExpr::as_f64))
        .map(|d| mm(d / 2.0))
        .unwrap_or(0);
    // The real drill diameter, *not* derived from `radius` by a fixed
    // ratio -- see `Item::Via::drill`'s doc comment for why that
    // assumption is wrong on real boards (confirmed against a real
    // KiCad demo file: 1.397mm pad / 0.6mm drill, a ~0.43 ratio).
    let drill = via
        .child("drill")
        .and_then(|d| d.tagged("drill"))
        .and_then(|args| args.first().and_then(SExpr::as_f64))
        .map(mm)
        .unwrap_or(radius); // no `(drill ...)` form: fall back to a solid-looking via rather than a zero-size hole

    node.add(Item::Via {
        shape: Circle::new(at, radius),
        drill,
        net: net_of(via),
    });
}

/// Imports a `(zone ...)` form's *already-filled* copper areas as
/// static `Item::Zone` obstacles -- one per `(filled_polygon (layer
/// ..) (pts ...))` sub-form, since a single zone can legitimately fill
/// into several disconnected copper islands (confirmed on a real file,
/// see below: a zone can and does produce dozens of separate
/// `filled_polygon` blocks, e.g. one ring per through-hole pad it
/// routes thermal-relief-free copper around).
///
/// **Explicitly documented limitations, not hidden** (see
/// the development log's Phase C entry for the full rationale):
/// - **No live re-fill.** The imported outline is exactly what KiCad's
///   own fill pass already computed, frozen at import time -- if
///   Alladin itself adds new tracks/vias afterwards, the zone's shape
///   does *not* shrink/grow around them the way a real KiCad re-fill
///   would. This can only make Alladin *more* conservative (it keeps
///   treating the old, possibly now-stale-but-still-valid-as-copper
///   shape as occupied), never less -- consistent with this codebase's
///   "never under-clear" bias.
/// - **No thermal-relief spoke geometry.** A same-net item is simply
///   exempted from colliding with the zone at all (via the existing
///   same-net fast path in `alladin_core::Node::query_colliding`)
///   rather than modelling the real spoke/gap shape a thermal relief
///   actually cuts into the copper around a same-net pad.
/// - **No zone-priority modelling.** Real KiCad resolves overlapping
///   zones of different priority by letting the higher-priority one
///   "win" the overlapping area; each zone is imported here completely
///   independently, so an overlap between two zones is simply two
///   separate `Item::Zone` obstacles occupying the same space.
///
/// Deliberately reads `filled_polygon` (the already-computed, real
/// copper shape a fill pass produced), **not** the zone's own bare
/// `(polygon (pts ...))` sibling form (the *unfilled* outline the user
/// drew before any fill ever ran, which does not account for clearance
/// to other copper, thermal-relief gaps, or `min_thickness` -- using it
/// would silently under- or over-state the real obstacle shape). This
/// also means an unfilled zone (a `(zone ...)` with no `filled_polygon`
/// sub-forms at all, e.g. one that hasn't had a fill pass run in KiCad
/// since it was drawn) contributes no obstacle at all -- consistent
/// with "there is currently no real copper there to collide with",
/// which is exactly true for a zone that's never been filled.
///
/// **S-expression shape verified against a real file**, per this
/// crate's project convention of checking real `.kicad_pcb` output
/// rather than assuming a schema: `/usr/share/kicad/demos/interf_u/interf_u.kicad_pcb`
/// (a genuine KiCad-shipped demo board) has a real `B.Cu` GND pour with
/// this exact shape --
/// `(zone (net 100) (net_name "GND") (layer "B.Cu") ... (polygon (pts
/// ...)) (filled_polygon (layer "B.Cu") (pts (xy ..) (xy ..) ...))
/// (filled_polygon (layer "B.Cu") (pts ...)) ...)` -- confirming both
/// that `net`/`filled_polygon`/`pts`/`xy` nest exactly as assumed here
/// and that a single zone really does produce many `filled_polygon`
/// blocks (that file's GND zone has dozens: one small ring per
/// through-hole pad it clears around, plus the large main pour).
fn import_zone(zone: &SExpr, node: &mut Node) {
    let net = net_of(zone);

    for filled in zone.children("filled_polygon") {
        let layer_name = filled
            .child("layer")
            .and_then(|l| l.tagged("layer"))
            .and_then(|args| args.first().and_then(SExpr::text))
            .unwrap_or("F.Cu");
        let layer = if layer_name == "B.Cu" { LayerId::BCu } else { LayerId::FCu };

        let Some(points) = filled.child("pts").map(pts_to_points) else {
            continue; // malformed `filled_polygon` with no `pts` at all
        };
        if points.len() < 3 {
            continue; // degenerate (line/point), not a real filled area
        }

        node.add(Item::Zone {
            outline: alladin_geom::Polygon::new(points),
            layer,
            net,
        });
    }
}

/// Reads a `(pts (xy x1 y1) (xy x2 y2) ...)` form's points, converted to
/// internal nanometre units. Shared by [`import_zone`] today; the same
/// `pts`/`xy` shape KiCad uses for a zone's `polygon`/`filled_polygon`
/// forms (this function deliberately doesn't distinguish which caller
/// it came from -- see [`import_zone`]'s doc comment for why only
/// `filled_polygon` is actually read).
fn pts_to_points(pts: &SExpr) -> Vec<Point> {
    pts.children("xy")
        .filter_map(|xy| {
            let args = xy.tagged("xy")?;
            let x = args.first()?.as_f64()?;
            let y = args.get(1)?.as_f64()?;
            Some(Point::new(mm(x), mm(y)))
        })
        .collect()
}

#[derive(Debug, Error)]
pub enum ExportError {
    #[error("failed to parse the original file as an S-expression: {0}")]
    Parse(#[from] ParseError),
    #[error("original source is not a `(kicad_pcb ...)` list")]
    NotAKicadPcb,
}

/// Append newly-routed items to an existing `.kicad_pcb` file's text,
/// returning the new file contents. This is `import_kicad_pcb`'s
/// counterpart: it closes the import -> route -> export loop so a
/// routed board can actually be written back out.
///
/// `nets` should be the net id -> name table from the original
/// [`ImportedBoard`] (or built by hand). Any item referencing a net id
/// not already in that table -- checked against both `nets` *and* any
/// `(net ...)` forms already present in `original_source` itself, so
/// this stays correct even if `nets` was assembled by hand rather than
/// sourced from a prior `import_kicad_pcb` call -- gets a synthesized
/// `(net id "Alladin_Net_id")` entry appended to the board's net table.
/// Without this, the output would reference net ids KiCad has never
/// heard of, which is exactly the kind of self-inconsistency
/// "Correct-by-Construction" is supposed to rule out.
///
/// [`Item::Pad`] entries in `items` are silently skipped: a bare `pad`
/// form is not valid syntax outside a `footprint`, and Alladin's router
/// never creates new pads -- only tracks and vias -- so this is a real
/// constraint on the expected input, not a silent data-loss risk in
/// practice.
///
/// The output is a syntactically valid S-expression -- confirmed by this
/// crate's own round-trip tests, which re-parse it with
/// [`import_kicad_pcb`] -- but single-line, not pretty-printed the way
/// KiCad's own writer indents its output. Since `.kicad_pcb`'s grammar
/// is whitespace-insensitive, this is *believed* to still be loadable by
/// real KiCad, but that has **not yet been confirmed** by actually
/// opening exported output in the KiCad application (same open item as
/// the importer's footprint-rotation sign convention -- both are
/// tracked in the development log pending access to real KiCad files/the
/// application itself).
pub fn export_appending_items(
    original_source: &str,
    items: &[Item],
    nets: &HashMap<u32, String>,
) -> Result<String, ExportError> {
    let root = parse(original_source)?;
    let mut top_level = root.as_list().ok_or(ExportError::NotAKicadPcb)?.to_vec();
    if !matches!(top_level.first(), Some(SExpr::Sym(s)) if s == "kicad_pcb") {
        return Err(ExportError::NotAKicadPcb);
    }

    let mut known_nets: std::collections::HashSet<u32> = nets.keys().copied().collect();
    for net_form in root.children("net") {
        if let Some(id) = net_form
            .tagged("net")
            .and_then(|args| args.first())
            .and_then(SExpr::as_i64)
        {
            known_nets.insert(id as u32);
        }
    }

    for item in items {
        let net_id = match item {
            Item::Track { net: Some(n), .. } | Item::Via { net: Some(n), .. } => Some(n.0),
            _ => None,
        };
        if let Some(id) = net_id {
            if known_nets.insert(id) {
                top_level.push(SExpr::List(vec![
                    SExpr::Sym("net".to_string()),
                    SExpr::Sym(id.to_string()),
                    SExpr::Str(format!("Alladin_Net_{id}")),
                ]));
            }
        }
    }

    for item in items {
        match item {
            Item::Track { shape, net, layer, .. } => {
                top_level.push(track_to_sexpr(shape, *net, *layer));
            }
            Item::Via { shape, drill, net } => {
                top_level.push(via_to_sexpr(shape, *drill, *net));
            }
            Item::Pad { .. } => {} // see doc comment: not valid outside a footprint
            // A zone is only ever imported (see `import_zone`), never
            // created by Alladin itself -- it's already present,
            // unmodified, in `original_source`, so re-emitting it here
            // would just duplicate the existing `(zone ...)` form.
            Item::Zone { .. } => {}
            // Same as `Item::Pad` above: a mounting hole is footprint-
            // owned (a real `np_thru_hole` pad form lives inside its
            // `(footprint ...)` block), never a free top-level form.
            Item::Hole { .. } => {}
        }
    }

    Ok(SExpr::List(top_level).to_string())
}

/// Formats a millimetre value the way a bare (unquoted) S-expression
/// numeric token needs to look: no unnecessary trailing zeros (KiCad's
/// own writer doesn't emit `5.000000`, and while a re-parse would accept
/// either, keeping output human-plausible costs nothing).
fn format_mm(v: f64) -> String {
    let s = format!("{v:.6}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s.is_empty() || s == "-" {
        "0".to_string()
    } else {
        s.to_string()
    }
}

fn mm_str(u: Unit) -> String {
    format_mm(u as f64 / MM as f64)
}

fn point_form(head: &str, p: Point) -> SExpr {
    SExpr::List(vec![
        SExpr::Sym(head.to_string()),
        SExpr::Sym(mm_str(p.x)),
        SExpr::Sym(mm_str(p.y)),
    ])
}

fn layer_name(layer: LayerId) -> &'static str {
    match layer {
        LayerId::FCu => "F.Cu",
        LayerId::BCu => "B.Cu",
    }
}

fn track_to_sexpr(shape: &Segment, net: Option<NetId>, layer: LayerId) -> SExpr {
    let mut form = vec![
        SExpr::Sym("segment".to_string()),
        point_form("start", shape.a),
        point_form("end", shape.b),
        SExpr::List(vec![SExpr::Sym("width".to_string()), SExpr::Sym(mm_str(shape.width))]),
        SExpr::List(vec![
            SExpr::Sym("layer".to_string()),
            SExpr::Str(layer_name(layer).to_string()),
        ]),
    ];
    if let Some(n) = net {
        form.push(SExpr::List(vec![
            SExpr::Sym("net".to_string()),
            SExpr::Sym(n.0.to_string()),
        ]));
    }
    SExpr::List(form)
}

fn via_to_sexpr(shape: &Circle, drill: Unit, net: Option<NetId>) -> SExpr {
    let diameter = shape.radius * 2;
    let mut form = vec![
        SExpr::Sym("via".to_string()),
        point_form("at", shape.center),
        SExpr::List(vec![SExpr::Sym("size".to_string()), SExpr::Sym(mm_str(diameter))]),
        SExpr::List(vec![SExpr::Sym("drill".to_string()), SExpr::Sym(mm_str(drill))]),
        SExpr::List(vec![
            SExpr::Sym("layers".to_string()),
            SExpr::Str("F.Cu".to_string()),
            SExpr::Str("B.Cu".to_string()),
        ]),
    ];
    if let Some(n) = net {
        form.push(SExpr::List(vec![
            SExpr::Sym("net".to_string()),
            SExpr::Sym(n.0.to_string()),
        ]));
    }
    SExpr::List(form)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writes_silk_lines_as_gr_lines_on_the_right_silk_layer() {
        let front = WriteSilkLine {
            start: Point::new(mm(1.0), mm(2.0)),
            end: Point::new(mm(3.0), mm(4.0)),
            width: mm(0.2),
            layer: LayerId::FCu,
        };
        let back = WriteSilkLine {
            start: Point::new(mm(-1.0), mm(0.0)),
            end: Point::new(mm(0.0), mm(1.0)),
            width: mm(0.15),
            layer: LayerId::BCu,
        };
        let text = write_kicad_pcb(&[], &[], &Node::new(), &[], &[], &[front, back], &[]);
        assert!(
            text.contains(r#"(gr_line (start 1 2) (end 3 4) (stroke (width 0.2) (type solid)) (layer "F.SilkS")"#),
            "front silk stroke missing/malformed:\n{text}"
        );
        assert!(
            text.contains(r#"(gr_line (start -1 0) (end 0 1) (stroke (width 0.15) (type solid)) (layer "B.SilkS")"#),
            "back silk stroke missing/malformed:\n{text}"
        );
        assert!(!text.contains("gr_text"), "Alladin must not emit gr_text for baked silk strokes");
    }

    #[test]
    fn imports_external_gr_text_on_silk_layers() {
        // External KiCad boards still use gr_text; Alladin's own export
        // no longer writes them, but import must keep recovering them.
        let text = write_kicad_pcb(&[], &[], &Node::new(), &[], &[], &[], &[]);
        let with_text = text.replacen(
            "(embedded_fonts no)",
            r#"(gr_text "HELLO" (at 10 5 -90) (layer "F.SilkS") (effects (font (size 1.2 1.2) (thickness 0.2)))) (gr_text "REV B" (at -3 2) (layer "B.SilkS") (effects (font (size 1 1) (thickness 0.2)))) (embedded_fonts no)"#,
            1,
        );
        let board = import_kicad_pcb(&with_text).expect("must parse a board with silk text");
        assert_eq!(board.silk_texts.len(), 2);
        assert_eq!(board.silk_texts[0].text, "HELLO");
        assert_eq!(board.silk_texts[0].position, Point::new(mm(10.0), mm(5.0)));
        assert_eq!(board.silk_texts[0].rotation_deg, 90.0, "must recover the original rotation, not the negated on-disk angle");
        assert_eq!(board.silk_texts[0].layer, LayerId::FCu);
        assert_eq!(board.silk_texts[0].height, mm(1.2));
        assert_eq!(board.silk_texts[0].line_width, mm(0.2));
        assert_eq!(board.silk_texts[1].text, "REV B");
        assert_eq!(board.silk_texts[1].layer, LayerId::BCu);
        assert_eq!(board.silk_texts[1].position, Point::new(mm(-3.0), mm(2.0)));
    }

    #[test]
    fn writes_a_silk_dot_as_a_filled_gr_circle_on_the_right_side() {
        let dot = WriteSilkDot { center: Point::new(mm(3.0), mm(-2.0)), diameter: mm(0.4), layer: LayerId::FCu };
        let back = WriteSilkDot { center: Point::new(mm(1.0), mm(1.0)), diameter: mm(1.0), layer: LayerId::BCu };
        let text = write_kicad_pcb(&[], &[], &Node::new(), &[], &[], &[], &[dot, back]);
        // Center + a point on the circle at center.x + radius, filled,
        // zero-width stroke -- the printed ink is exactly the diameter.
        assert!(text.contains(r#"(gr_circle (center 3 -2) (end 3.2 -2) (stroke (width 0) (type solid)) (fill yes) (layer "F.SilkS")"#), "front dot missing/malformed:\n{text}");
        assert!(text.contains(r#"(gr_circle (center 1 1) (end 1.5 1) (stroke (width 0) (type solid)) (fill yes) (layer "B.SilkS")"#), "back dot missing/malformed:\n{text}");
    }

    #[test]
    fn ignores_gr_text_on_a_non_silk_layer() {
        let text = write_kicad_pcb(&[], &[], &Node::new(), &[], &[], &[], &[]);
        // A `gr_text` on `Cmts.User` (a real, common KiCad layer for
        // free-text notes) is not one of ours -- must be silently
        // skipped, not misread as front-silk.
        let with_comment = text.replacen(
            "(embedded_fonts no)",
            r#"(gr_text "note to self" (at 1 1) (layer "Cmts.User") (effects (font (size 1 1) (thickness 0.15)))) (embedded_fonts no)"#,
            1,
        );
        let board = import_kicad_pcb(&with_comment).expect("must still parse");
        assert!(board.silk_texts.is_empty());
    }

    /// A hand-written, structurally-representative `.kicad_pcb` snippet
    /// -- not a byte-for-byte real KiCad export, but exercising every
    /// shape this importer handles: a net table, an unrotated footprint
    /// with two SMD pads on different nets, a *rotated* footprint with a
    /// through-hole pad (net on both copper layers), an unconnected pad,
    /// a routed segment, and a via.
    const FIXTURE: &str = r#"
        (kicad_pcb
          (version 20221018)
          (generator "pcbnew")
          (net 0 "")
          (net 1 "GND")
          (net 2 "VCC")
          (net 3 "SIG")
          (footprint "TestFP:R1" (layer "F.Cu") (at 0 0)
            (pad "1" smd rect (at -1 0) (size 0.9 0.95) (layers "F.Cu") (net 1 "GND"))
            (pad "2" smd rect (at 1 0) (size 0.9 0.95) (layers "F.Cu") (net 2 "VCC"))
          )
          (footprint "TestFP:R2" (layer "F.Cu") (at 5 5 90)
            (pad "1" thru_hole circle (at 1 0) (size 1.5 1.5) (drill 0.8) (layers "*.Cu") (net 3 "SIG"))
          )
          (footprint "TestFP:NC" (layer "F.Cu") (at 20 20)
            (pad "1" smd circle (at 0 0) (size 1 1) (layers "F.Cu"))
          )
          (segment (start 0 0) (end 5 0) (width 0.25) (layer "F.Cu") (net 1))
          (via (at 10 10) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net 2))
        )
    "#;

    #[test]
    fn parses_the_net_table() {
        let board = import_kicad_pcb(FIXTURE).unwrap();
        assert_eq!(board.nets.get(&1), Some(&"GND".to_string()));
        assert_eq!(board.nets.get(&2), Some(&"VCC".to_string()));
        assert_eq!(board.nets.get(&3), Some(&"SIG".to_string()));
        assert_eq!(board.nets.get(&0), Some(&"".to_string()));
    }

    #[test]
    fn imports_unrotated_smd_pads_at_the_right_position_and_net() {
        let board = import_kicad_pcb(FIXTURE).unwrap();
        let pad1 = board
            .node
            .iter()
            .find_map(|item| match item {
                Item::Pad { shape, net: Some(NetId(1)), layer: LayerId::FCu } => Some(shape.clone()),
                _ => None,
            })
            .expect("GND pad must be imported");

        assert_eq!(pad1.center(), Point::new(-1 * MM, 0));
        assert_eq!(pad1.bounding_radius(), mm(0.475)); // max(0.9, 0.95) / 2
    }

    #[test]
    fn footprint_rotation_correctly_transforms_pad_position() {
        let board = import_kicad_pcb(FIXTURE).unwrap();
        // R2 footprint at (5,5)mm, rotated 90 deg, pad locally at (1,0)mm
        // -> rotated offset (0,-1)mm -> absolute (5,4)mm. Sign verified
        // against real KiCad (`pcbnew`) ground truth -- see `rotate`'s
        // doc comment and the dedicated ground-truth test below.
        let found = board.node.iter().any(|item| matches!(
            item,
            Item::Pad { shape, net: Some(NetId(3)), .. }
                if (shape.center().x - 5 * MM).abs() < 100 && (shape.center().y - 4 * MM).abs() < 100
        ));
        assert!(found, "rotated footprint's pad must land at (5mm, 4mm)");
    }

    /// Ground-truth regression test: reproduces the exact scenario that
    /// caught (and fixed) a real sign error in `rotate` (see that
    /// function's doc comment for the full story) -- a real KiCad demo
    /// file's `C103` footprint (`complex_hierarchy.kicad_pcb`), rotated
    /// 90 degrees, whose pad "2" `pcbnew`'s own Python API places at
    /// (160.02, 63.359) mm, *not* (160.02, 93.359) mm (what the
    /// mathematically-standard-but-wrong-for-KiCad rotation direction
    /// would have predicted). The fixture below reproduces just the
    /// footprint/pad shape that mattered, not the whole multi-hundred-KB
    /// original file.
    #[test]
    fn rotation_matches_pcbnew_ground_truth_on_a_real_kicad_footprint() {
        let fixture = r#"
            (kicad_pcb
              (net 0 "")
              (net 1 "GND")
              (net 2 "-VAA")
              (footprint "complex_hierarchy:CP_Axial_L10.0mm_D4.5mm_P15.00mm_Horizontal"
                (layer "F.Cu")
                (at 160.02 78.359 90)
                (attr through_hole)
                (pad "1" thru_hole rect (at 0 0 90) (size 2 2) (drill 1)
                  (layers "*.Cu" "*.Mask") (net 1 "GND"))
                (pad "2" thru_hole oval (at 15 0 90) (size 2 2) (drill 1)
                  (layers "*.Cu" "*.Mask") (net 2 "-VAA"))
              )
            )
        "#;
        let board = import_kicad_pcb(fixture).unwrap();

        let pad1 = board.node.iter().find_map(|item| match item {
            Item::Pad { shape, net: Some(NetId(1)), layer: LayerId::FCu } => Some(shape.clone()),
            _ => None,
        }).expect("pad 1 (GND) must be imported");
        let pad2 = board.node.iter().find_map(|item| match item {
            Item::Pad { shape, net: Some(NetId(2)), layer: LayerId::FCu } => Some(shape.clone()),
            _ => None,
        }).expect("pad 2 (-VAA) must be imported");

        // pcbnew ground truth (`python3 -c "import pcbnew; ..."` against
        // the real file, 2026-07-29): pad 1 at (160.02, 78.359),
        // pad 2 at (160.02, 63.359).
        let close = |a: Point, expected_mm: (f64, f64)| {
            (a.x - mm(expected_mm.0)).abs() < 1000 && (a.y - mm(expected_mm.1)).abs() < 1000
        };
        assert!(close(pad1.center(), (160.02, 78.359)), "pad 1 got {:?}", pad1.center());
        assert!(close(pad2.center(), (160.02, 63.359)), "pad 2 got {:?}", pad2.center());
    }

    #[test]
    fn through_hole_pad_is_imported_on_both_copper_layers() {
        let board = import_kicad_pcb(FIXTURE).unwrap();
        let front = board.node.iter().filter(|item| matches!(
            item, Item::Pad { net: Some(NetId(3)), layer: LayerId::FCu, .. }
        )).count();
        let back = board.node.iter().filter(|item| matches!(
            item, Item::Pad { net: Some(NetId(3)), layer: LayerId::BCu, .. }
        )).count();
        assert_eq!(front, 1);
        assert_eq!(back, 1);
    }

    #[test]
    fn an_np_thru_hole_pad_imports_as_a_mechanical_item_hole_not_a_pad() {
        // A minimal footprint carrying a real `np_thru_hole` pad, the
        // exact form `writer::footprint_to_sexpr` now writes for a
        // mounting hole -- must import as `Item::Hole` (no net, no
        // copper) on this flat routing-obstacle path too, mirroring
        // the structured `import_footprints` path's own
        // `pads`/`holes` split.
        let fixture = r#"
            (kicad_pcb
              (footprint "MountingHole:M3"
                (layer "F.Cu")
                (at 20 15)
                (pad "" np_thru_hole circle (at 0 0) (size 3.2 3.2) (drill 3.2)
                  (layers "*.Cu" "*.Mask"))
              )
            )
        "#;
        let board = import_kicad_pcb(fixture).unwrap();
        let hole = board.node.iter().find_map(|item| match item {
            Item::Hole { position, drill } => Some((*position, *drill)),
            _ => None,
        }).expect("the np_thru_hole pad must import as an Item::Hole");
        assert_eq!(hole.0, Point::new(20 * MM, 15 * MM));
        assert_eq!(hole.1, mm(3.2));
        assert!(!board.node.iter().any(|item| matches!(item, Item::Pad { .. })), "must not also import as a Pad");
    }

    #[test]
    fn pad_without_a_net_form_imports_as_unconnected() {
        let board = import_kicad_pcb(FIXTURE).unwrap();
        let found = board.node.iter().any(|item| matches!(item, Item::Pad { net: None, .. }));
        assert!(found, "the NC footprint's pad must import with net = None");
    }

    #[test]
    fn imports_a_routed_segment() {
        let board = import_kicad_pcb(FIXTURE).unwrap();
        let track = board.node.iter().find_map(|item| match item {
            Item::Track { shape, net: Some(NetId(1)), layer: LayerId::FCu, .. } => Some(*shape),
            _ => None,
        }).expect("segment must be imported as a Track on net 1");

        assert_eq!(track.a, Point::new(0, 0));
        assert_eq!(track.b, Point::new(5 * MM, 0));
        assert_eq!(track.width, mm(0.25));
    }

    #[test]
    fn imports_a_via() {
        let board = import_kicad_pcb(FIXTURE).unwrap();
        let (shape, drill) = board.node.iter().find_map(|item| match item {
            Item::Via { shape, drill, net: Some(NetId(2)) } => Some((*shape, *drill)),
            _ => None,
        }).expect("via must be imported on net 2");

        assert_eq!(shape.center, Point::new(10 * MM, 10 * MM));
        assert_eq!(shape.radius, mm(0.4)); // size 0.8mm / 2
        // FIXTURE's via happens to use a 0.5 size/drill ratio (0.8mm /
        // 0.4mm), but this must come from the real `(drill ...)` form,
        // not an assumed ratio -- see
        // `imports_the_real_drill_diameter_not_a_guessed_ratio` below
        // for a fixture where the ratio is *not* 0.5.
        assert_eq!(drill, mm(0.4));
    }

    #[test]
    fn imports_the_real_drill_diameter_not_a_guessed_ratio() {
        // Regression test for a real bug found via cross-checking
        // against actual KiCad demo files (see `Item::Via::drill`'s doc
        // comment and the development log's corresponding update):
        // `interf_u.kicad_pcb` uses 1.397mm-diameter vias with a 0.6mm
        // drill -- a ~0.43 ratio, not the 0.5 this crate's importer used
        // to silently assume before it read `(drill ...)` at all.
        let fixture = r#"
            (kicad_pcb
              (net 0 "")
              (net 11 "SIG")
              (via (at 178.435 83.82) (size 1.397) (drill 0.6) (layers "F.Cu" "B.Cu") (net 11))
            )
        "#;
        let board = import_kicad_pcb(fixture).unwrap();
        let (shape, drill) = board.node.iter().find_map(|item| match item {
            Item::Via { shape, drill, net: Some(NetId(11)) } => Some((*shape, *drill)),
            _ => None,
        }).expect("via must be imported");

        assert_eq!(shape.radius, mm(1.397 / 2.0));
        assert_eq!(drill, mm(0.6));
    }

    #[test]
    fn chains_a_real_boards_scrambled_edge_cuts_into_one_closed_outline() {
        // The exact 9 `gr_line` forms from `interf_u.kicad_pcb`'s real
        // `Edge.Cuts` layer, in their real file order -- which is *not*
        // walk order (see `import_board_outline`'s doc comment): some
        // need to be walked start->end, others end->start, to chain
        // into a single closed loop. This is the actual scrambled input
        // that motivated writing a chaining algorithm at all, not a
        // pre-ordered convenience fixture.
        let fixture = r#"
            (kicad_pcb
              (net 0 "")
              (gr_line (start 172.085 133.35) (end 194.945 133.35) (layer "Edge.Cuts"))
              (gr_line (start 90.805 140.97) (end 90.805 142.494) (layer "Edge.Cuts"))
              (gr_line (start 79.375 133.35) (end 79.375 34.29) (layer "Edge.Cuts"))
              (gr_line (start 79.375 34.29) (end 194.945 34.29) (layer "Edge.Cuts"))
              (gr_line (start 194.945 133.35) (end 194.945 34.29) (layer "Edge.Cuts"))
              (gr_line (start 90.805 142.494) (end 172.085 142.494) (layer "Edge.Cuts"))
              (gr_line (start 90.805 133.35) (end 90.805 140.97) (layer "Edge.Cuts"))
              (gr_line (start 172.085 142.494) (end 172.085 133.35) (layer "Edge.Cuts"))
              (gr_line (start 90.805 133.35) (end 79.375 133.35) (layer "Edge.Cuts"))
              (gr_line (start 0 0) (end 1 1) (layer "F.SilkS"))
            )
        "#;
        let board = import_kicad_pcb(fixture).unwrap();

        assert_eq!(board.outline.len(), 1, "the 9 edges must chain into exactly one closed loop");
        let outline = &board.outline[0];
        assert_eq!(outline.points.len(), 9);

        // Point-containment results ground-truth verified against
        // `pcbnew`'s own `SHAPE_POLY_SET::Contains()` on the real file
        // (see `Polygon`'s own tests in `alladin-geom` for the same
        // fixture reproduced independently there).
        assert!(outline.contains_point(Point::new(mm(130.0), mm(50.0))));
        assert!(!outline.contains_point(Point::new(mm(0.0), mm(0.0))));
    }

    #[test]
    fn imports_a_real_boards_filled_zone_as_a_static_obstacle() {
        // One real `filled_polygon` block, verbatim from
        // `interf_u.kicad_pcb`'s actual `B.Cu` GND pour (see
        // `import_zone`'s doc comment) -- the exact real ring shape KiCad
        // filled around one of that zone's own through-hole pads,
        // including its real `(net 100) (net_name "GND")` header and the
        // zone's own unfilled `(polygon (pts ...))` sibling (which must
        // be ignored -- only `filled_polygon` counts, see `import_zone`'s
        // doc comment).
        let fixture = r#"
            (kicad_pcb
              (net 0 "")
              (net 100 "GND")
              (zone
                (net 100)
                (net_name "GND")
                (layer "B.Cu")
                (hatch edge 0.508)
                (connect_pads (clearance 0.5))
                (min_thickness 0.2)
                (filled_areas_thickness no)
                (fill yes (thermal_gap 0.508) (thermal_bridge_width 0.508))
                (polygon (pts (xy 193.675 133.35) (xy 193.675 35.56) (xy 80.645 35.56) (xy 80.645 133.35)))
                (filled_polygon
                  (layer "B.Cu")
                  (pts
                    (xy 100.49401 105.21491) (xy 100.455204 105.41) (xy 100.49401 105.60509) (xy 100.533372 105.664)
                    (xy 98.856628 105.664) (xy 98.89599 105.60509) (xy 98.934796 105.41) (xy 98.89599 105.21491) (xy 98.856628 105.156)
                    (xy 100.533372 105.156)
                  )
                )
              )
            )
        "#;
        let board = import_kicad_pcb(fixture).unwrap();

        let zones: Vec<_> = board.node.iter().filter(|item| matches!(item, Item::Zone { .. })).collect();
        assert_eq!(zones.len(), 1, "one `filled_polygon` block -> one `Item::Zone`");

        match zones[0] {
            Item::Zone { outline, layer, net } => {
                assert_eq!(outline.points.len(), 10);
                assert_eq!(*layer, LayerId::BCu);
                assert_eq!(*net, Some(NetId(100)));
                // Every point round-trips to the exact millimetre value
                // KiCad wrote, not just "some polygon of the right size".
                assert_eq!(outline.points[0], Point::new(mm(100.49401), mm(105.21491)));
                assert_eq!(outline.points[9], Point::new(mm(100.533372), mm(105.156)));
            }
            _ => unreachable!("filtered to only Zone items above"),
        }
    }

    #[test]
    fn a_zone_with_several_filled_polygon_islands_imports_as_several_items() {
        // Design point from the plan: "eine Zone kann mehrere getrennte
        // gefüllte Inseln haben" -- each must become its own independent
        // `Item::Zone`, not merged into one (they're genuinely
        // disconnected copper shapes, e.g. isolated by other tracks
        // running through the middle of the pour).
        let fixture = r#"
            (kicad_pcb
              (net 0 "")
              (net 5 "VCC")
              (zone
                (net 5)
                (net_name "VCC")
                (layer "F.Cu")
                (filled_polygon (layer "F.Cu") (pts (xy 0 0) (xy 2 0) (xy 2 2) (xy 0 2)))
                (filled_polygon (layer "F.Cu") (pts (xy 5 0) (xy 7 0) (xy 7 2) (xy 5 2)))
              )
            )
        "#;
        let board = import_kicad_pcb(fixture).unwrap();
        let zone_count = board.node.iter().filter(|item| matches!(item, Item::Zone { .. })).count();
        assert_eq!(zone_count, 2);
    }

    #[test]
    fn a_zone_with_no_filled_polygon_yet_imports_no_obstacle() {
        // A zone the user drew but never actually ran a fill pass on
        // (or one KiCad couldn't fill, e.g. zero clearance left) has no
        // real copper -- see `import_zone`'s doc comment for why this is
        // correct, not a missed case.
        let fixture = r#"
            (kicad_pcb
              (net 0 "")
              (net 5 "VCC")
              (zone
                (net 5)
                (net_name "VCC")
                (layer "F.Cu")
                (polygon (pts (xy 0 0) (xy 2 0) (xy 2 2) (xy 0 2)))
              )
            )
        "#;
        let board = import_kicad_pcb(fixture).unwrap();
        assert_eq!(board.node.len(), 0);
    }

    #[test]
    fn a_netless_zone_imports_with_no_net() {
        // A keepout-style zone (or any zone with `(net 0)`, KiCad's
        // reserved "no net" id) must import with `net: None` -- see
        // `Item::Zone`'s own doc comment for why that's exactly what
        // makes it block every other net unconditionally, no separate
        // keepout mechanism needed.
        let fixture = r#"
            (kicad_pcb
              (net 0 "")
              (zone
                (net 0)
                (net_name "")
                (layer "F.Cu")
                (filled_polygon (layer "F.Cu") (pts (xy 0 0) (xy 2 0) (xy 2 2) (xy 0 2)))
              )
            )
        "#;
        let board = import_kicad_pcb(fixture).unwrap();
        let net = board.node.iter().find_map(|item| match item {
            Item::Zone { net, .. } => Some(*net),
            _ => None,
        }).expect("the zone must still be imported");
        assert_eq!(net, None);
    }

    #[test]
    fn circumcircle_recovers_the_center_and_radius_of_three_points_on_a_known_circle() {
        let center = Point::new(mm(3.0), mm(-2.0));
        let radius_mm = 5.0;
        let point_at = |deg: f64| {
            let a = deg.to_radians();
            Point::new(
                center.x + (radius_mm * MM as f64 * a.cos()).round() as Unit,
                center.y + (radius_mm * MM as f64 * a.sin()).round() as Unit,
            )
        };
        let (a, b, c) = (point_at(0.0), point_at(120.0), point_at(250.0));

        let (found_center, found_radius) = circumcircle(a, b, c).expect("three non-collinear points must have a circumcircle");
        assert!(found_center.distance(center) < 10.0, "found center {found_center:?} too far from real center {center:?}");
        assert!((found_radius - radius_mm * MM as f64).abs() < 10.0, "found radius {found_radius} too far from real radius");
    }

    #[test]
    fn circumcircle_is_none_for_three_collinear_points() {
        let a = Point::new(0, 0);
        let b = Point::new(mm(5.0), mm(5.0));
        let c = Point::new(mm(10.0), mm(10.0));
        assert!(circumcircle(a, b, c).is_none());
    }

    #[test]
    fn arc_polyline_starts_and_ends_exactly_at_the_input_points_and_stays_on_the_fitted_circle() {
        // A quarter circle, center (8mm, 8mm), radius 2mm, start at 0°,
        // end at 90°, `mid` at 45° -- exactly the shape a rounded board
        // corner uses.
        let center = Point::new(mm(8.0), mm(8.0));
        let r = 2.0 * MM as f64;
        let point_at = |deg: f64| {
            let a = deg.to_radians();
            Point::new(center.x + (r * a.cos()).round() as Unit, center.y + (r * a.sin()).round() as Unit)
        };
        let start = point_at(0.0);
        let mid = point_at(45.0);
        let end = point_at(90.0);

        let polyline = arc_polyline(start, mid, end);
        assert_eq!(*polyline.first().unwrap(), start, "polyline must start exactly at the input start point");
        assert_eq!(*polyline.last().unwrap(), end, "polyline must end exactly at the input end point");
        assert!(polyline.len() > 2, "a real arc must produce interior waypoints, not just the two endpoints");

        for p in &polyline {
            let d = p.distance(center);
            assert!((d - r).abs() < r * 0.02, "point {p:?} at distance {d} from center strays too far from radius {r}");
        }

        // The polyline must actually pass near `mid` somewhere along its
        // length -- proof this picked the short 90° sweep through `mid`,
        // not the long 270° way around the circle (which would also
        // start/end at the same two points but never come near `mid`).
        assert!(
            polyline.iter().any(|p| p.distance(mid) < r * 0.1),
            "polyline {polyline:?} never comes close to mid point {mid:?}"
        );
    }

    #[test]
    fn arc_polyline_returns_the_direct_segment_for_collinear_points() {
        let start = Point::new(0, 0);
        let mid = Point::new(mm(2.0), mm(2.0));
        let end = Point::new(mm(4.0), mm(4.0));
        assert_eq!(arc_polyline(start, mid, end), vec![start, end]);
    }

    #[test]
    fn chains_a_rounded_corner_board_mixing_gr_line_and_gr_arc() {
        // A 10x10mm square with one corner (top-right) rounded off with
        // a 2mm-radius `gr_arc`, structurally the same shape as a real
        // rounded-corner board -- see `import_board_outline`'s doc
        // comment for the real board (`RaspberryPi-HAT.kicad_pcb`) this
        // was validated against too, via ground truth from `pcbnew`'s
        // own `SHAPE_POLY_SET::Contains()` (see the development log's
        // "Teil 16" entry).
        let fixture = r#"
            (kicad_pcb
              (net 0 "")
              (gr_line (start 0 0) (end 10 0) (layer "Edge.Cuts"))
              (gr_line (start 10 0) (end 10 8) (layer "Edge.Cuts"))
              (gr_arc (start 10 8) (mid 9.414214 9.414214) (end 8 10) (layer "Edge.Cuts"))
              (gr_line (start 8 10) (end 0 10) (layer "Edge.Cuts"))
              (gr_line (start 0 10) (end 0 0) (layer "Edge.Cuts"))
            )
        "#;
        let board = import_kicad_pcb(fixture).unwrap();
        assert_eq!(board.outline.len(), 1, "the mixed line/arc edges must chain into exactly one closed loop");
        let outline = &board.outline[0];

        let p = |x: f64, y: f64| Point::new(mm(x), mm(y));
        assert!(outline.contains_point(p(5.0, 5.0)), "well inside the square");
        assert!(!outline.contains_point(p(-1.0, -1.0)), "outside the square entirely");
        assert!(outline.contains_point(p(9.0, 9.0)), "inside the rounded corner's radius (dist ~1.41mm < 2mm)");
        assert!(!outline.contains_point(p(9.5, 9.5)), "cut off by the rounded corner (dist ~2.12mm > 2mm)");
    }

    #[test]
    fn chaining_bridges_a_sub_tolerance_gap_but_not_a_real_one() {
        // Regression test for the real bug found via `RaspberryPi-HAT.kicad_pcb`
        // (see `CHAIN_TOLERANCE`'s doc comment): a `gr_arc` endpoint and
        // its neighbouring `gr_line`'s matching endpoint can differ by a
        // sub-micrometre float rounding artifact in a real KiCad-exported
        // file. Two triangles here, identical except for how far off
        // their "shared" corner really is: one by 2 micrometres (well
        // under `CHAIN_TOLERANCE`, must still chain), one by 20
        // micrometres (well over it, must *not* silently bridge what
        // could be a real gap).
        let make_fixture = |gap_um: f64| {
            let gap_mm = gap_um / 1000.0;
            format!(
                r#"(kicad_pcb (net 0 "")
                    (gr_line (start 0 0) (end 5 0) (layer "Edge.Cuts"))
                    (gr_line (start 5 0) (end 2.5 5) (layer "Edge.Cuts"))
                    (gr_line (start {} {}) (end 0 0) (layer "Edge.Cuts"))
                )"#,
                2.5 + gap_mm,
                5.0 + gap_mm
            )
        };

        let within_tolerance = import_kicad_pcb(&make_fixture(2.0)).unwrap();
        assert_eq!(
            within_tolerance.outline.len(),
            1,
            "a 2-micrometre gap (well under CHAIN_TOLERANCE) must still chain into one closed loop"
        );

        let beyond_tolerance = import_kicad_pcb(&make_fixture(20.0)).unwrap();
        assert_eq!(
            beyond_tolerance.outline.len(),
            0,
            "a 20-micrometre gap (well over CHAIN_TOLERANCE) must NOT be silently bridged"
        );
    }

    #[test]
    fn total_item_count_matches_the_fixture() {
        let board = import_kicad_pcb(FIXTURE).unwrap();
        // R1: 2 SMD pads (1 layer each) = 2
        // R2: 1 thru_hole pad on *.Cu = 2 (F.Cu + B.Cu)
        // NC: 1 SMD pad = 1
        // 1 segment + 1 via = 2
        assert_eq!(board.node.len(), 2 + 2 + 1 + 2);
    }

    #[test]
    fn rejects_input_that_is_not_a_kicad_pcb_form() {
        // `ImportedBoard` (the `Ok` side) deliberately doesn't derive
        // `Debug` -- it wraps a `Node`, which doesn't either (see
        // `alladin_core::Node`'s docs) -- so match on the `Result`
        // directly instead of `unwrap_err()`, which requires `T: Debug`.
        match import_kicad_pcb("(kicad_sch (version 1))") {
            Err(ImportError::NotAKicadPcb) => {}
            other => panic!("expected NotAKicadPcb, got a different outcome ({})",
                if other.is_ok() { "Ok" } else { "a different Err" }),
        }
    }

    #[test]
    fn propagates_underlying_parse_errors() {
        match import_kicad_pcb("(kicad_pcb (unterminated") {
            Err(ImportError::Parse(_)) => {}
            other => panic!("expected Parse error, got a different outcome ({})",
                if other.is_ok() { "Ok" } else { "a different Err" }),
        }
    }

    // -- export_appending_items -------------------------------------------

    #[test]
    fn exported_tracks_and_vias_round_trip_through_import_again() {
        let board = import_kicad_pcb(FIXTURE).unwrap();
        let original_count = board.node.len();

        // Deliberately far from any coordinate already in `FIXTURE`, so
        // this can be unambiguously found again after re-import (the
        // fixture's own segment also happens to start at (0,0)).
        let new_track = Item::Track {
            shape: Segment::new(Point::new(50 * MM, 50 * MM), Point::new(53 * MM, 54 * MM), 250_000),
            net: Some(NetId(1)), // an existing net (GND)
            layer: LayerId::FCu,
            class: NetClass::C,
        };
        let new_via = Item::Via {
            shape: Circle::new(Point::new(8 * MM, 8 * MM), 400_000),
            drill: 175_000, // deliberately *not* a round-number ratio of the 400_000 radius
            net: Some(NetId(2)), // also existing (VCC)
        };

        let exported =
            export_appending_items(FIXTURE, &[new_track, new_via], &board.nets).unwrap();

        let reimported = import_kicad_pcb(&exported)
            .expect("exported text must itself be a valid, re-importable .kicad_pcb form");

        assert_eq!(reimported.node.len(), original_count + 2);

        let track = reimported
            .node
            .iter()
            .find_map(|item| match item {
                Item::Track { shape, net: Some(NetId(1)), layer: LayerId::FCu, .. }
                    if shape.a == Point::new(50 * MM, 50 * MM) =>
                {
                    Some(*shape)
                }
                _ => None,
            })
            .expect("the newly-exported track must round-trip back in");
        assert_eq!(track.b, Point::new(53 * MM, 54 * MM));
        assert_eq!(track.width, 250_000);

        let (via, via_drill) = reimported
            .node
            .iter()
            .find_map(|item| match item {
                Item::Via { shape, drill, net: Some(NetId(2)) } if shape.center == Point::new(8 * MM, 8 * MM) => {
                    Some((*shape, *drill))
                }
                _ => None,
            })
            .expect("the newly-exported via must round-trip back in");
        assert_eq!(via.radius, 400_000);
        // The real drill diameter must survive the round-trip too, not
        // get reduced to some assumed ratio of the outer diameter.
        assert_eq!(via_drill, 175_000);
    }

    #[test]
    fn export_backfills_the_net_table_for_a_brand_new_net_id() {
        let board = import_kicad_pcb(FIXTURE).unwrap();
        assert!(!board.nets.contains_key(&42), "net 42 must not already exist in the fixture");

        let new_track = Item::Track {
            shape: Segment::new(Point::new(0, 0), Point::new(MM, 0), 250_000),
            net: Some(NetId(42)),
            layer: LayerId::FCu,
            class: NetClass::C,
        };

        let exported = export_appending_items(FIXTURE, &[new_track], &board.nets).unwrap();
        let reimported = import_kicad_pcb(&exported).unwrap();

        assert!(
            reimported.nets.contains_key(&42),
            "net 42 must have been backfilled into the net table, or KiCad would see a \
             segment referencing an undeclared net"
        );

        let track = reimported.node.iter().any(|item| {
            matches!(item, Item::Track { net: Some(NetId(42)), .. })
        });
        assert!(track, "the track on the new net must still have been exported");
    }

    #[test]
    fn export_skips_pad_items_and_leaves_original_items_untouched() {
        let board = import_kicad_pcb(FIXTURE).unwrap();
        let original_count = board.node.len();

        // A `Pad` in the `items` slice must be silently skipped (see
        // `export_appending_items`'s doc comment): not valid syntax
        // outside a `footprint`.
        let stray_pad = Item::Pad {
            shape: PadShape::Circle(Circle::new(Point::new(0, 0), 500_000)),
            net: None,
            layer: LayerId::FCu,
        };

        let exported = export_appending_items(FIXTURE, &[stray_pad], &board.nets).unwrap();
        let reimported = import_kicad_pcb(&exported).unwrap();

        // Nothing was actually appended -- item count unchanged.
        assert_eq!(reimported.node.len(), original_count);
    }

    #[test]
    fn export_rejects_a_source_that_is_not_a_kicad_pcb_form() {
        match export_appending_items("(kicad_sch (version 1))", &[], &HashMap::new()) {
            Err(ExportError::NotAKicadPcb) => {}
            other => panic!("expected NotAKicadPcb, got a different outcome ({})",
                if other.is_ok() { "Ok" } else { "a different Err" }),
        }
    }
}
