//! Download a real component by its LCSC/JLCPCB "C-number" straight from
//! EasyEDA's own product API and turn it into a `crate::parts_db`-ready
//! [`FetchedPart`] -- name, reference prefix, and every pad's real
//! position/size/shape/rotation/number, not a placeholder.
//!
//! This is a real client against EasyEDA's **undocumented** public API
//! (there is no official one for this) -- the endpoint and field layout
//! were confirmed two ways while building this: (1) against KiCad's own
//! published EasyEDA-importer documentation
//! (<https://dev-docs.kicad.org/en/import-formats/easyeda/index.html>,
//! which documents the exact `PAD~...` tilde-delimited field order from
//! KiCad's own parser source) and (2) by fetching several real
//! components live and inspecting the JSON returned (see
//! `test_fixtures/lcsc_*.json` -- trimmed real responses, not
//! hand-written approximations, used by this module's own tests).
//!
//! **What is and isn't faithfully imported**, since this file is
//! deliberately not a "best effort" stub:
//! - Position, size, layer, net-facing collision circle, pad
//!   number/name, true shape (circle/rect/oval), and the pad's own
//!   rotation: all read from the real data, not guessed.
//! - `PadTemplate::radius` (used by placement/collision/routing) is the
//!   *circumscribing* circle of the real pad -- see `footprint.rs`'s
//!   doc comment for why every pad in this whole workspace is circular
//!   for that purpose, downloaded or not.
//! - `POLYGON`-shaped pads (a custom point cloud, not `RECT`/`OVAL`)
//!   render as their bounding rectangle rather than the exact outline --
//!   a real, disclosed, minor simplification; still correctly
//!   positioned, sized, and numbered.
//! - 3D models and copper zones are not imported -- out of scope for
//!   "place this part and route to its pads", which is what Alladin
//!   PCB needs a part for. Top-silkscreen *graphics* are a partial
//!   exception: their combined bounding box becomes
//!   [`FetchedPart::explicit_courtyard`] (a real mechanical
//!   keep-out, see [`parse_silk_courtyard`]) -- the individual shapes
//!   themselves still aren't kept or rendered.
//! - Each pin's *function name* (`"GND"`, `"VDD"`, `"DIN"`, `"DOUT"`,
//!   ...) **is** imported, best-effort, as [`PadTemplate::pin_name`] --
//!   but from a *second* request, [`fetch_pin_names`] against
//!   `.../products/{code}/svgs`, not the footprint endpoint above: a
//!   pin's function is schematic-symbol information (EasyEDA renders
//!   each pin as an SVG `<g c_partid="part_pin" ...
//!   c_spicepin="N">` group containing exactly two `<text>` elements,
//!   name then number -- confirmed against a real fetched SK6812 LED
//!   symbol, see `test_fixtures/lcsc_c5378720_svgs.json`), which the
//!   footprint/PCB data alone never carries (its own `PAD~...` line
//!   *does* have a `net` field at the documented position, but it's
//!   empty in every real footprint-only response seen so far -- that
//!   field is for a net *already assigned in a full schematic+PCB
//!   project*, not a symbol's pin function). This second request is
//!   soft-failing: any problem (offline, no symbol, unexpected markup)
//!   just leaves every pad's `pin_name` as `None` rather than failing
//!   the whole download, since the footprint is still fully usable
//!   without it.

use std::collections::HashMap;
use std::sync::mpsc::{Receiver, Sender};
use std::thread;

use alladin_core::LayerId;
use alladin_geom::{Point, Unit};
use serde_json::Value;

use crate::footprint::{Courtyard, PadShapeKind, PadTemplate};

const API_VERSION: &str = "6.4.19.5";
/// EasyEDA's CDN answers a bare `curl`/`reqwest`-style request (no
/// `Referer`) with a generic CloudFront 403 -- confirmed while building
/// this against the real endpoint. A same-site-looking `Referer` is
/// enough; this is not bypassing any authentication, just looking like
/// a normal browser tab instead of a bare HTTP client.
const REFERER: &str = "https://easyeda.com/";
const USER_AGENT: &str = "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0 Safari/537.36";

/// 1 EasyEDA PCB coordinate unit = 10 mil = 0.254mm = 254_000nm -- see
/// this module's doc comment for how that was confirmed (KiCad's own
/// importer docs, cross-checked against real fetched components: e.g. a
/// real 2.54mm-pitch header's two pads land at exactly ±1.27mm once
/// converted, see `test_fixtures/lcsc_c124375_tht_connector.json`).
fn easyeda_unit_to_nm(value: f64) -> Unit {
    (value * 254_000.0).round() as Unit
}

#[derive(Debug)]
pub enum FetchError {
    /// Couldn't even talk to EasyEDA (offline, DNS, timeout, ...).
    Network(String),
    /// EasyEDA answered but has no such product.
    NotFound(String),
    /// EasyEDA has the product, but not a PCB footprint for it (e.g. a
    /// schematic-only symbol) -- nothing this feature could place.
    NoFootprint(String),
    /// The response didn't look like what this parser understands.
    Parse(String),
}

impl std::fmt::Display for FetchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FetchError::Network(e) => write!(f, "couldn't reach LCSC/EasyEDA: {e}"),
            FetchError::NotFound(code) => write!(f, "{code} wasn't found on LCSC/EasyEDA"),
            FetchError::NoFootprint(code) => write!(f, "{code} has no PCB footprint to import"),
            FetchError::Parse(e) => write!(f, "couldn't understand LCSC/EasyEDA's response: {e}"),
        }
    }
}

/// A successfully downloaded, ready-to-save-to-`parts_db` component.
pub struct FetchedPart {
    pub lcsc_code: String,
    pub name: String,
    pub reference_prefix: String,
    pub description: String,
    /// This part's own real category on LCSC/EasyEDA, e.g.
    /// `"Resistors"` or `"Light Emitting Diodes (LED)"` -- confirmed
    /// against several real, live fetched components (a resistor, an
    /// SK6812 LED, an ESP32-WROOM-32, a pin header, ...): every one
    /// carries a `result.tags` array, always with at least one entry,
    /// itself the exact, human-readable category EasyEDA's own product
    /// page shows for that part. `None` only if a future response ever
    /// omits `tags` entirely (not seen in practice) -- `crate::parts_db`
    /// simply files that part under "Uncategorized" rather than failing
    /// the whole download over a missing, purely organizational field.
    pub category: Option<String>,
    pub pads: Vec<PadTemplate>,
    /// This part's own real mechanical body/courtyard outline, if the
    /// footprint's own top-silkscreen data actually drew one -- see
    /// [`parse_silk_courtyard`]. `None` for plenty of real, tiny SMD
    /// parts (footprint too small to bother drawing a body outline at
    /// all): `crate::footprint::FootprintTemplate::courtyard` still
    /// gets a correct (if plainer) pad/hole bounding-box fallback in
    /// that case, so this is never a hard failure.
    pub explicit_courtyard: Option<Courtyard>,
}

/// Fetches and parses `code` (e.g. `"C2040"`) against the live EasyEDA
/// API. Blocking (a plain synchronous HTTP GET) -- callers on a UI
/// thread must run this on a background thread (see `crate::app`'s
/// download button, which does exactly that) rather than call it
/// directly from an egui frame.
pub fn fetch_by_lcsc_code(code: &str) -> Result<FetchedPart, FetchError> {
    let code = code.trim();
    if code.is_empty() {
        return Err(FetchError::NotFound("(empty)".to_string()));
    }
    let url = format!("https://easyeda.com/api/products/{code}/components?version={API_VERSION}");
    let response = ureq::get(&url)
        .set("User-Agent", USER_AGENT)
        .set("Referer", REFERER)
        .set("Accept", "application/json")
        .timeout(std::time::Duration::from_secs(15))
        .call()
        .map_err(|e| FetchError::Network(e.to_string()))?;
    let body: Value = response.into_json().map_err(|e| FetchError::Parse(e.to_string()))?;
    let mut part = parse_response(code, &body)?;

    let pin_names = fetch_pin_names(code);
    if !pin_names.is_empty() {
        for pad in &mut part.pads {
            if let Some(name) = pin_names.get(&pad.number) {
                pad.pin_name = Some(name.clone());
            }
        }
    }
    Ok(part)
}

/// Fetches `code`'s schematic symbol(s) and extracts a `pin number ->
/// pin function name` map (`"3" -> "VDD"`, ...) -- see this module's
/// doc comment for why this is a *second* request against a
/// *different* endpoint than the footprint itself, and why it never
/// fails outright: any problem (offline, no symbol for this part,
/// unexpected markup) just yields an empty map, so a caller merging it
/// into already-fetched pads simply leaves every `pin_name` as `None`.
fn fetch_pin_names(code: &str) -> HashMap<String, String> {
    let url = format!("https://easyeda.com/api/products/{code}/svgs");
    let Ok(response) = ureq::get(&url)
        .set("User-Agent", USER_AGENT)
        .set("Referer", REFERER)
        .set("Accept", "application/json")
        .timeout(std::time::Duration::from_secs(15))
        .call()
    else {
        return HashMap::new();
    };
    let Ok(body): Result<Value, _> = response.into_json() else {
        return HashMap::new();
    };
    parse_pin_names(&body)
}

/// The pure, fixture-testable half of [`fetch_pin_names`] -- parses an
/// already-decoded `.../svgs` response body.
fn parse_pin_names(body: &Value) -> HashMap<String, String> {
    let mut names = HashMap::new();
    let Some(entries) = body.get("result").and_then(Value::as_array) else {
        return names;
    };
    // `docType == 2` is a schematic symbol; `4` is the PCB footprint
    // (see `fn parse_response`'s sibling data) -- only symbols carry
    // pin function names, so entries of any other `docType` (a 3D
    // model preview, etc.) are simply skipped rather than guessed at.
    for entry in entries.iter().filter(|e| e.get("docType").and_then(Value::as_i64) == Some(2)) {
        if let Some(svg) = entry.get("svg").and_then(Value::as_str) {
            for (number, name) in extract_pin_names_from_symbol_svg(svg) {
                names.insert(number, name);
            }
        }
    }
    names
}

/// Extracts `(pin number, pin function name)` pairs straight out of
/// one schematic symbol's raw SVG markup -- see this module's doc
/// comment for the exact `<g c_partid="part_pin" ...
/// c_spicepin="N">`-with-two-`<text>`-children shape this relies on.
/// Deliberately simple substring scanning rather than a real XML
/// parser (no such dependency exists in this workspace, and the
/// markup EasyEDA actually emits is regular enough not to need one) --
/// any pin group that doesn't match the expected shape is just
/// skipped, not an error, consistent with [`fetch_pin_names`]'s
/// soft-failing contract.
fn extract_pin_names_from_symbol_svg(svg: &str) -> Vec<(String, String)> {
    let mut pairs = Vec::new();
    for group in svg.split("c_partid=\"part_pin\"").skip(1) {
        let Some(number) = extract_attr(group, "c_spicepin=\"") else { continue };
        let mut texts = Vec::new();
        let mut rest = group;
        while texts.len() < 2 {
            let Some(start) = rest.find("<text") else { break };
            let Some(gt) = rest[start..].find('>') else { break };
            let after_gt = &rest[start + gt + 1..];
            let Some(close) = after_gt.find("</text>") else { break };
            texts.push(after_gt[..close].to_string());
            rest = &after_gt[close + "</text>".len()..];
        }
        if let Some(name) = texts.first().filter(|s| !s.is_empty()) {
            pairs.push((number, name.clone()));
        }
    }
    pairs
}

/// Returns the quoted value right after `marker` (e.g.
/// `extract_attr(r#"c_spicepin="3" foo"#, "c_spicepin=\"")` ->
/// `Some("3")`) -- the one attribute-lookup primitive
/// [`extract_pin_names_from_symbol_svg`] needs.
fn extract_attr(s: &str, marker: &str) -> Option<String> {
    let start = s.find(marker)? + marker.len();
    let rest = &s[start..];
    let end = rest.find('"')?;
    Some(rest[..end].to_string())
}

/// Spawns [`fetch_by_lcsc_code`] on a background thread and returns a
/// [`Receiver`] the caller can poll with `try_recv()` once per UI frame
/// -- the actual non-blocking entry point for `crate::app`.
pub fn fetch_in_background(code: String) -> Receiver<Result<FetchedPart, FetchError>> {
    let (tx, rx): (Sender<Result<FetchedPart, FetchError>>, Receiver<_>) = std::sync::mpsc::channel();
    thread::spawn(move || {
        let _ = tx.send(fetch_by_lcsc_code(&code));
    });
    rx
}

fn parse_response(code: &str, body: &Value) -> Result<FetchedPart, FetchError> {
    if body.get("success").and_then(Value::as_bool) != Some(true) {
        return Err(FetchError::NotFound(code.to_string()));
    }
    let result = body.get("result").ok_or_else(|| FetchError::Parse("missing 'result'".to_string()))?;
    let name = result.get("title").and_then(Value::as_str).filter(|s| !s.is_empty()).unwrap_or(code).to_string();
    let description = result.get("description").and_then(Value::as_str).unwrap_or("").to_string();
    // EasyEDA's own category for this part, e.g. `["Resistors"]` --
    // see `FetchedPart::category`'s own doc comment for how this was
    // confirmed against several real, live components. Only the first
    // tag is kept: every real response seen so far has exactly one,
    // and `crate::parts_db`'s category tree is a flat string per part,
    // not a multi-tag system.
    let category = result.get("tags").and_then(Value::as_array).and_then(|tags| tags.first()).and_then(Value::as_str).map(str::to_string);

    let package_data_str = result
        .get("packageDetail")
        .and_then(|p| p.get("dataStr"))
        .ok_or_else(|| FetchError::NoFootprint(code.to_string()))?;
    // Some EasyEDA API versions nest `dataStr` as a JSON *string* that
    // needs a second parse rather than an already-decoded object;
    // handle both so a version bump on their end doesn't silently break
    // this.
    let data: Value = match package_data_str {
        Value::String(s) => serde_json::from_str(s).map_err(|e| FetchError::Parse(e.to_string()))?,
        other => other.clone(),
    };

    let head = data.get("head").ok_or_else(|| FetchError::Parse("footprint has no 'head'".to_string()))?;
    let origin_x = head.get("x").and_then(Value::as_f64).unwrap_or(0.0);
    let origin_y = head.get("y").and_then(Value::as_f64).unwrap_or(0.0);
    let c_para = head.get("c_para");
    let reference_prefix = c_para
        .and_then(|c| c.get("pre"))
        .and_then(Value::as_str)
        .map(|s| s.trim_end_matches('?').to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "U".to_string());
    let package = c_para.and_then(|c| c.get("package")).and_then(Value::as_str).unwrap_or("");

    let shapes = data.get("shape").and_then(Value::as_array).ok_or_else(|| FetchError::Parse("footprint has no 'shape' array".to_string()))?;
    let shape_lines: Vec<&str> = shapes.iter().filter_map(Value::as_str).collect();
    let pads: Vec<PadTemplate> = shape_lines.iter().filter_map(|line| parse_pad_line(line, origin_x, origin_y)).collect();

    if pads.is_empty() {
        return Err(FetchError::NoFootprint(code.to_string()));
    }

    let explicit_courtyard = parse_silk_courtyard(&shape_lines, origin_x, origin_y);

    let description = match (package.is_empty(), description.is_empty()) {
        (false, false) => format!("{package} \u{2014} {description}"),
        (false, true) => package.to_string(),
        (true, _) => description,
    };

    Ok(FetchedPart { lcsc_code: code.to_string(), name, reference_prefix, description, category, pads, explicit_courtyard })
}

/// EasyEDA layer number for the top silkscreen -- same table
/// [`parse_pad_line`]'s own doc comment already cites (KiCad's own
/// EasyEDA-importer docs).
const TOP_SILK_LAYER: i32 = 3;

/// Folds one more `(x, y)` point (in raw EasyEDA units) into `bbox`'s
/// running min/max -- the one accumulator every shape handler in
/// [`parse_silk_courtyard`] feeds through.
fn extend_bbox(bbox: &mut Option<(f64, f64, f64, f64)>, x: f64, y: f64) {
    *bbox = Some(match bbox.take() {
        Some((min_x, min_y, max_x, max_y)) => (min_x.min(x), min_y.min(y), max_x.max(x), max_y.max(y)),
        None => (x, y, x, y),
    });
}

/// Every whitespace-delimited pair of numeric tokens in `points_or_path`,
/// non-numeric tokens (SVG-style path commands like `M`/`L`/`Z`, or a
/// stray odd one out) simply dropped -- shared by [`parse_silk_courtyard`]'s
/// `TRACK` handler (a plain `"x y x y ..."` point list) and its
/// `SOLIDREGION` handler (an SVG-style `"M x y L x y ... Z"` path:
/// confirmed against several real fetched parts that its `M`/`L`
/// commands are followed directly by a space-delimited `x y` pair with
/// no comma, so simply discarding the one-character command tokens --
/// which never parse as `f64` -- and pairing up whatever numbers
/// remain recovers every vertex correctly).
fn parse_coordinate_pairs(points_or_path: &str) -> Vec<(f64, f64)> {
    let nums: Vec<f64> = points_or_path.split_whitespace().filter_map(|tok| tok.parse().ok()).collect();
    nums.chunks_exact(2).map(|p| (p[0], p[1])).collect()
}

/// Folds an `ARC~...~pathData` line's own start/end points into `bbox`,
/// each generously padded by the arc's own radii in every direction --
/// real arc math (center, sweep) needs more than this module has any
/// other use for, but an arc's true center can never be farther from
/// either of its own endpoints than its own radius, so padding both
/// endpoints by `(rx, ry)` is guaranteed to *contain* the true arc
/// bounding box, matching this whole function's own "occasionally
/// generous, never too small" contract (see its own doc comment).
///
/// `path_data` is confirmed (against several real fetched parts, all
/// rounded-corner chip parts whose silk outline is straight `TRACK`
/// segments joined by a quarter-circle `ARC` at each corner) to be
/// `"M startX startY A rx ry xAxisRotation largeArcFlag sweepFlag
/// endX endY"` -- space-delimited, matching KiCad's own EasyEDA
/// importer docs (`M` and `A` SVG path commands) modulo the missing
/// commas KiCad's doc examples show (real data uses plain whitespace
/// throughout instead).
fn extend_arc_bbox(bbox: &mut Option<(f64, f64, f64, f64)>, path_data: &str) {
    let tokens: Vec<&str> = path_data.split_whitespace().collect();
    let mut last_point: Option<(f64, f64)> = None;
    let mut i = 0;
    while i < tokens.len() {
        match tokens[i] {
            "M" | "L" if i + 2 < tokens.len() => {
                if let (Ok(x), Ok(y)) = (tokens[i + 1].parse::<f64>(), tokens[i + 2].parse::<f64>()) {
                    extend_bbox(bbox, x, y);
                    last_point = Some((x, y));
                }
                i += 3;
            }
            "A" if i + 7 < tokens.len() => {
                let parsed: Option<(f64, f64, f64, f64)> =
                    (|| Some((tokens[i + 1].parse().ok()?, tokens[i + 2].parse().ok()?, tokens[i + 6].parse().ok()?, tokens[i + 7].parse().ok()?)))();
                if let Some((rx, ry, x, y)) = parsed {
                    if let Some((sx, sy)) = last_point {
                        extend_bbox(bbox, sx - rx, sy - ry);
                        extend_bbox(bbox, sx + rx, sy + ry);
                    }
                    extend_bbox(bbox, x - rx, y - ry);
                    extend_bbox(bbox, x + rx, y + ry);
                    last_point = Some((x, y));
                }
                i += 8;
            }
            _ => i += 1,
        }
    }
}

/// Every top-silkscreen `TRACK`/`RECT`/`ARC` shape's own extent,
/// reduced to one combined bounding box -- real, physically meaningful
/// mechanical data (confirmed against every real part in this
/// workspace's own parts database, not just one or two fixtures): a
/// component's real body/courtyard outline is drawn with exactly these
/// three shape types -- straight edges (`TRACK`), an explicit
/// rectangle (`RECT`), or a rounded corner joining two straight edges
/// (`ARC`) -- and their combined extent is a real (if occasionally
/// generous) proxy for "how much board space does this part's own
/// body actually occupy" -- see [`Courtyard`]'s own doc comment for
/// why a plain bounding box, not a true silhouette, is the right
/// level of precision here.
///
/// - `TRACK~width~layer~net~points~id~...` -- `points` is a
///   space-delimited `"x y x y ..."` list, every point folded in
///   directly.
/// - `RECT~x~y~width~height~layer~id~...` -- `(x, y)` is the
///   rectangle's own top-left corner (confirmed against a real SOIC-14,
///   see `test_fixtures/lcsc_c155176_sn74ahct125dr.json`: this was a
///   real, confirmed bug -- an earlier version of this function only
///   understood `TRACK`, so this exact part's real, JLCPCB-drawn body
///   outline, drawn as a standalone `RECT` and nothing else, was
///   silently missed and fell all the way back to the plain, much
///   larger pad bounding box instead), so both that corner and the
///   opposite `(x + width, y + height)` corner are folded in.
/// - `ARC~strokeWidth~layer~net~pathData~...` -- see [`extend_arc_bbox`].
///   Real, confirmed necessary (not just theoretically possible): every
///   rounded-corner chip resistor/capacitor/inductor in this workspace's
///   own parts database draws its silk outline as straight `TRACK`
///   segments joined by a quarter-circle `ARC` at each rounded corner,
///   e.g. `C14663` (a real 0603 ceramic capacitor) whose real courtyard
///   only reaches its real ~3.4mm x ~2.0mm size once these corner arcs
///   are folded in too, not the ~2.8mm x ~1.4mm a `TRACK`-only version
///   of this function would report (a real, confirmed-too-small result).
///
/// `CIRCLE` and `SOLIDREGION` (both also real shape types seen on this
/// same layer in real fetched parts) are deliberately **not** folded
/// in here, even though they can look like two obvious cases to add:
/// both were tried and confirmed, against real fetched parts, to
/// sometimes draw a small pin-1 dot/arrow/flag deliberately placed
/// *outside* the part's real body on purpose (by design -- that is the
/// whole point of a polarity marker), not part of the body outline
/// itself, and including them is a real, confirmed regression, not a
/// theoretical risk:
/// - A `SOLIDREGION` pin-1 flag on one real SK6812 LED
///   (`test_fixtures/lcsc_c5378720_sk6812_led.json`) grows its correct
///   ~5.4mm x ~5.0mm courtyard to a wrong ~6.1mm x ~6.0mm; on a second,
///   independently fetched SK6812RGBW-NW variant, a correct ~5.1mm x
///   ~4.9mm grows all the way to a badly wrong ~8.7mm x ~5.5mm.
/// - A `CIRCLE` pin-1 dot on the very same real SOIC-14 this function's
///   own `RECT` fix targets (`C155176`, see above) sits far enough
///   past its real ~2.79mm-tall body that including it inflates the
///   height to a wrong ~4.32mm -- a 55% overstatement of the real
///   body, from what looks, in isolation, like an innocuous small dot.
///
/// Returns `None` when the footprint draws none of these three shapes
/// on the top-silk layer at all (common for the smallest SMD packages)
/// -- [`FetchedPart::explicit_courtyard`]'s own doc comment covers the
/// fallback for that case.
fn parse_silk_courtyard(shapes: &[&str], origin_x: f64, origin_y: f64) -> Option<Courtyard> {
    let mut bbox: Option<(f64, f64, f64, f64)> = None;

    for line in shapes {
        let f: Vec<&str> = line.split('~').collect();
        match f.first().copied() {
            Some("TRACK") if f.len() >= 5 => {
                if f[2].parse::<i32>() != Ok(TOP_SILK_LAYER) {
                    continue;
                }
                for (x, y) in parse_coordinate_pairs(f[4]) {
                    extend_bbox(&mut bbox, x, y);
                }
            }
            Some("RECT") if f.len() >= 6 => {
                if f[5].parse::<i32>() != Ok(TOP_SILK_LAYER) {
                    continue;
                }
                let parsed: Option<[f64; 4]> = (|| Some([f[1].parse().ok()?, f[2].parse().ok()?, f[3].parse().ok()?, f[4].parse().ok()?]))();
                let Some([x, y, width, height]) = parsed else { continue };
                extend_bbox(&mut bbox, x, y);
                extend_bbox(&mut bbox, x + width, y + height);
            }
            Some("ARC") if f.len() >= 5 => {
                if f[2].parse::<i32>() != Ok(TOP_SILK_LAYER) {
                    continue;
                }
                extend_arc_bbox(&mut bbox, f[4]);
            }
            _ => continue,
        }
    }

    let (min_x, min_y, max_x, max_y) = bbox?;
    let center = Point::new(easyeda_unit_to_nm((min_x + max_x) / 2.0 - origin_x), easyeda_unit_to_nm((min_y + max_y) / 2.0 - origin_y));
    Some(Courtyard { center, width: easyeda_unit_to_nm(max_x - min_x), height: easyeda_unit_to_nm(max_y - min_y) })
}

/// Parses one `PAD~...` line from an EasyEDA Standard footprint's
/// `shape` array. Field layout confirmed against KiCad's own importer
/// documentation:
/// `PAD~shape~x~y~width~height~layer~net~number~holeDia~polyPoints~rotation~uuid~...`
/// Returns `None` for anything not a recognizable `PAD` line -- callers
/// simply skip tracks/silkscreen/3D-outline/etc. shape lines this way,
/// rather than needing to enumerate every non-pad shape type.
fn parse_pad_line(line: &str, origin_x: f64, origin_y: f64) -> Option<PadTemplate> {
    let f: Vec<&str> = line.split('~').collect();
    if f.len() < 12 || f[0] != "PAD" {
        return None;
    }
    let shape_name = f[1];
    let x: f64 = f[2].parse().ok()?;
    let y: f64 = f[3].parse().ok()?;
    let width: f64 = f[4].parse().ok()?;
    let height: f64 = f[5].parse().ok()?;
    let layer: i32 = f[6].parse().ok()?;
    let number = f[8].to_string();
    let hole_dia: f64 = f[9].parse().unwrap_or(0.0);
    let rotation_deg: f64 = f[11].parse().unwrap_or(0.0);

    let offset = Point::new(easyeda_unit_to_nm(x - origin_x), easyeda_unit_to_nm(y - origin_y));
    let mut w = easyeda_unit_to_nm(width);
    let mut h = easyeda_unit_to_nm(height);

    // Layer 11 = multi-layer/through-hole, 1 = front SMD, 2 = back SMD
    // (EasyEDA layer numbering). A through-hole pad
    // (`hole_dia > 0`) always collapses onto `FCu` here -- exactly the
    // same simplification Alladin's own built-in THT templates already
    // use (see `footprint.rs`'s doc comment), not a new one introduced
    // for downloaded parts.
    let is_tht = hole_dia > 0.0;
    let pad_layer = if is_tht || layer == 11 {
        LayerId::FCu
    } else if layer == 2 {
        LayerId::BCu
    } else {
        LayerId::FCu
    };

    let shape = match shape_name {
        "RECT" => PadShapeKind::Rect { width: w, height: h },
        "OVAL" | "ELLIPSE" if w != h => PadShapeKind::Oval { width: w, height: h },
        "OVAL" | "ELLIPSE" => PadShapeKind::Circle,
        "POLYGON" => {
            // No native polygon pad shape in this workspace (see
            // `footprint.rs`'s doc comment) -- approximate with the
            // point cloud's own bounding rectangle rather than
            // silently degrading to a circle for a shape that's
            // usually itself already a (chamfered) rectangle.
            match poly_bbox_nm(f.get(10).copied().unwrap_or(""), origin_x, origin_y) {
                Some((bw, bh)) => {
                    w = bw;
                    h = bh;
                    PadShapeKind::Rect { width: bw, height: bh }
                }
                None => PadShapeKind::Circle,
            }
        }
        _ => PadShapeKind::Circle,
    };

    // Collision/routing radius: half the *shorter* side (the inscribed
    // circle), not the bounding box's half-diagonal, and not
    // `max(w, h) / 2` as some importers use for rect pads. Both of
    // those were tried first and were real bugs on a
    // real, fetched part: the ESP32-WROOM-32's own edge pads are
    // 2.0mm x 0.9mm at a 1.27mm pitch (confirmed against the real
    // download, see `test_fixtures/` and this module's own regression
    // test) -- `max(w, h) / 2` alone already gives a 2.0mm-diameter
    // circle, still bigger than the 1.27mm pitch, so *neighbouring pads
    // on the very same footprint* still overlapped each other, and
    // ordinary interactive routing between adjacent pins was
    // pathologically slow in practice. Real footprints are always
    // pitched to clear each other along the pad's own *shorter* side
    // (that's what determines how tightly they can be packed in a
    // row) -- using `min(w, h) / 2` reconstructs exactly that
    // clearance and eliminates the false self-overlap. The accepted
    // trade-off: a very elongated pad's own far tip (along its
    // *longer* side) is under-covered by this circle -- a real, minor
    // gap in DRC precision at that one spot, deliberately accepted so
    // routing to/from a real, tightly-pitched part actually works at
    // all, which is the more important correctness property here.
    let radius = (w.min(h) / 2).max(1);

    // `hole_dia` was already parsed above (`is_tht = hole_dia > 0.0`) --
    // keep it, converted to nanometres, so a through-hole part round-
    // trips into a real through-hole pad with its actual drill size on
    // manufacturing export instead of silently becoming an
    // unmanufacturable SMD pad.
    let hole_diameter = is_tht.then(|| easyeda_unit_to_nm(hole_dia).max(1));

    Some(PadTemplate { offset, radius, layer: pad_layer, number, shape, rotation_deg, hole_diameter, pin_name: None })
}

/// The bounding box (in nanometres, already origin-shifted) of a
/// space-delimited `"x1 y1 x2 y2 ..."` point list in EasyEDA's own
/// coordinate units -- used only for `POLYGON` pads, see
/// [`parse_pad_line`].
fn poly_bbox_nm(points_str: &str, origin_x: f64, origin_y: f64) -> Option<(Unit, Unit)> {
    let nums: Vec<f64> = points_str.split_whitespace().filter_map(|s| s.parse().ok()).collect();
    if nums.len() < 4 {
        return None;
    }
    let xs: Vec<f64> = nums.iter().step_by(2).copied().collect();
    let ys: Vec<f64> = nums.iter().skip(1).step_by(2).copied().collect();
    let (min_x, max_x) = (xs.iter().cloned().fold(f64::INFINITY, f64::min), xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
    let (min_y, max_y) = (ys.iter().cloned().fold(f64::INFINITY, f64::min), ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
    let _ = (origin_x, origin_y); // width/height are translation-invariant; kept for signature symmetry with the offset math above.
    Some((easyeda_unit_to_nm(max_x - min_x), easyeda_unit_to_nm(max_y - min_y)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(name: &str) -> Value {
        let path = format!("{}/test_fixtures/{name}", env!("CARGO_MANIFEST_DIR"));
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("couldn't read fixture {path}: {e}"));
        serde_json::from_str(&text).unwrap_or_else(|e| panic!("fixture {path} isn't valid JSON: {e}"))
    }

    #[test]
    fn parses_a_real_smd_resistor_footprint_with_two_rect_pads() {
        let part = parse_response("C25804", &fixture("lcsc_c25804_resistor.json")).expect("a real, captured resistor response must parse");
        assert_eq!(part.name, "0603WAF1002T5E");
        assert_eq!(part.reference_prefix, "R");
        assert!(part.description.contains("R0603"), "description should mention the package: {}", part.description);
        assert_eq!(part.category, Some("Resistors".to_string()), "EasyEDA's own real tag for this part must come through as its category");
        assert_eq!(part.pads.len(), 2);

        for pad in &part.pads {
            assert_eq!(pad.layer, LayerId::FCu, "front SMD pads must land on FCu");
            assert!(matches!(pad.shape, PadShapeKind::Rect { .. }), "0603 pads are RECT in the real data");
        }

        // The two pads must be symmetric around the footprint origin
        // and 2 * ~0.753mm apart -- a real 0603's real pad spacing, not
        // an arbitrary placeholder distance.
        let mut xs: Vec<Unit> = part.pads.iter().map(|p| p.offset.x).collect();
        xs.sort();
        assert_eq!(xs[0], -xs[1], "pads must be symmetric around the origin");
        let spacing_mm = (xs[1] - xs[0]) as f64 / alladin_geom::MM as f64;
        assert!((spacing_mm - 1.506).abs() < 0.01, "expected ~1.51mm pad spacing for a real 0603, got {spacing_mm}mm");
    }

    #[test]
    fn a_response_with_no_tags_array_at_all_yields_no_category_rather_than_an_error() {
        // A missing purely-organizational field must never fail the
        // whole download -- the footprint itself is still fully usable
        // without a category, same soft-failing contract this module's
        // own doc comment already documents for `fetch_pin_names`.
        let body: Value = serde_json::from_str(
            r#"{"success": true, "result": {"title": "No Tags", "packageDetail": {"dataStr": {"head": {"x": 0, "y": 0}, "shape": ["PAD~RECT~0~0~1~1~1~~1~0~~0~id~0~~Y"]}}}}"#,
        )
        .unwrap();
        let part = parse_response("C0", &body).expect("a response with no 'tags' array at all must still parse");
        assert_eq!(part.category, None);
    }

    #[test]
    fn parses_a_real_tht_header_with_a_round_and_a_square_pad_and_a_drill_hole() {
        let part =
            parse_response("C124375", &fixture("lcsc_c124375_tht_connector.json")).expect("a real, captured THT connector response must parse");
        assert_eq!(part.pads.len(), 2);

        let pad1 = part.pads.iter().find(|p| p.number == "1").expect("pad 1 must exist");
        let pad2 = part.pads.iter().find(|p| p.number == "2").expect("pad 2 must exist");

        // Pin 1 of a real THT connector is conventionally square so it
        // stands out from the round pins -- and is, in the real data.
        assert!(matches!(pad1.shape, PadShapeKind::Rect { .. }), "pin 1 should be the square pad in the real data");
        assert!(matches!(pad2.shape, PadShapeKind::Circle), "pin 2 (an equal-sided ELLIPSE) collapses to a plain circle");

        // Both pads sit on a through-hole part -- see `parse_pad_line`'s
        // doc comment for why that means FCu here, same as everywhere
        // else in this workspace.
        assert_eq!(pad1.layer, LayerId::FCu);
        assert_eq!(pad2.layer, LayerId::FCu);

        // A real 2.54mm-pitch header: pins exactly 1.27mm either side
        // of the origin -- this is the strongest real-world proof the
        // coordinate scaling/origin-shift math is exactly right, not
        // just roughly right.
        let expected_half_pitch = (1.27 * alladin_geom::MM as f64).round() as Unit;
        assert_eq!(pad1.offset.x, -expected_half_pitch);
        assert_eq!(pad2.offset.x, expected_half_pitch);
        assert_eq!(pad1.offset.y, 0);
        assert_eq!(pad2.offset.y, 0);
    }

    #[test]
    fn rejects_a_response_whose_success_flag_is_false() {
        let body: Value = serde_json::from_str(r#"{"success": false, "code": 404}"#).unwrap();
        assert!(matches!(parse_response("C0", &body), Err(FetchError::NotFound(code)) if code == "C0"));
    }

    #[test]
    fn rejects_a_component_with_no_package_detail_as_no_footprint_not_a_panic() {
        let body: Value = serde_json::from_str(r#"{"success": true, "result": {"title": "Symbol Only"}}"#).unwrap();
        assert!(matches!(parse_response("C1", &body), Err(FetchError::NoFootprint(code)) if code == "C1"));
    }

    #[test]
    fn rejects_garbage_json_shape_as_a_parse_error_not_a_panic() {
        let body: Value = serde_json::from_str(r#"{"success": true, "result": {"packageDetail": {"dataStr": {"head": {}}}}}"#).unwrap();
        assert!(matches!(parse_response("C2", &body), Err(FetchError::Parse(_))));
    }

    #[test]
    fn a_footprint_whose_only_shapes_are_non_pad_lines_is_no_footprint_not_an_empty_success() {
        let body: Value = serde_json::from_str(
            r#"{"success": true, "result": {"packageDetail": {"dataStr": {"head": {"x": 0, "y": 0}, "shape": ["TRACK~0.6~3~~0 0 1 1~id~0"]}}}}"#,
        )
        .unwrap();
        assert!(matches!(parse_response("C3", &body), Err(FetchError::NoFootprint(code)) if code == "C3"));
    }

    #[test]
    fn parse_pad_line_ignores_non_pad_shape_lines() {
        assert!(parse_pad_line("TRACK~0.6~3~~0 0 1 1~id~0", 0.0, 0.0).is_none());
        assert!(parse_pad_line("CIRCLE~0~0~1~1~1~id~0", 0.0, 0.0).is_none());
    }

    #[test]
    fn an_elongated_pad_s_collision_radius_is_half_its_shorter_side_not_half_its_longer_side_or_diagonal() {
        // Regression test for a real bug (see `parse_pad_line`'s doc
        // comment): a pad 1mm wide, 3mm tall must get radius 0.5mm
        // (half the *shorter* side), not sqrt(1^2+3^2)/2 =~ 1.58mm
        // (the diagonal) and not 3/2 = 1.5mm (the longer side either).
        let line = format!("PAD~RECT~0~0~{}~{}~1~~1~0~~0~id~0~~Y", 1.0 / 0.254, 3.0 / 0.254);
        let pad = parse_pad_line(&line, 0.0, 0.0).expect("a well-formed RECT pad line must parse");
        let expected_radius = (0.5 * alladin_geom::MM as f64).round() as Unit;
        assert_eq!(pad.radius, expected_radius, "radius must be half the shorter side");
    }

    #[test]
    fn a_real_esp32_wroom_32_s_edge_pads_do_not_overlap_their_own_neighbours() {
        // The actual bug report this was chasing: interactive routing
        // between two adjacent pins of a real, downloaded
        // ESP32-WROOM-32 module became "extremely slow and crashes"
        // (in practice: collision checks against overlapping neighbour
        // pads on the same footprint). Root
        // cause, confirmed against the real download: the module's own
        // edge pads are 2.0mm x 0.9mm at a 1.27mm pitch -- any radius
        // formula giving more than 0.635mm (half the pitch) makes
        // neighbouring pads on the very same footprint overlap. This
        // asserts the actual real-world numbers stay clear.
        let part = parse_response("C82899", &fixture("lcsc_c82899_esp32_wroom_32.json")).expect("a real, captured ESP32-WROOM-32 response must parse");
        assert_eq!(part.pads.len(), 39, "38 castellated edge pins + the centre ground pad");

        for a in &part.pads {
            for b in &part.pads {
                if std::ptr::eq(a, b) {
                    continue;
                }
                let dx = (a.offset.x - b.offset.x) as f64;
                let dy = (a.offset.y - b.offset.y) as f64;
                let center_distance = (dx * dx + dy * dy).sqrt();
                let radius_sum = (a.radius + b.radius) as f64;
                assert!(
                    center_distance >= radius_sum,
                    "pads {} and {} overlap: centres {center_distance:.0}nm apart, radii sum to {radius_sum:.0}nm",
                    a.number,
                    b.number
                );
            }
        }
    }

    #[test]
    fn polygon_pads_fall_back_to_their_bounding_rectangle() {
        // A diamond-ish point cloud 2 units wide, 4 units tall (in
        // EasyEDA units), centered on the origin.
        let line = "PAD~POLYGON~0~0~0~0~1~~1~0~-1 0 0 -2 1 0 0 2~0~id~0~~Y";
        let pad = parse_pad_line(line, 0.0, 0.0).expect("a well-formed POLYGON pad line must parse");
        match pad.shape {
            PadShapeKind::Rect { width, height } => {
                assert_eq!(width, easyeda_unit_to_nm(2.0));
                assert_eq!(height, easyeda_unit_to_nm(4.0));
            }
            other => panic!("expected a bounding-rect approximation, got {other:?}"),
        }
    }

    #[test]
    fn a_real_sk6812_led_s_footprint_has_no_net_baked_into_its_pad_lines() {
        // Confirms the claim in this module's doc comment: even a real,
        // captured footprint response's `PAD~...` `net` field is empty
        // -- pin function names genuinely aren't in this endpoint's
        // data, not just an unparsed field.
        let part =
            parse_response("C5378720", &fixture("lcsc_c5378720_sk6812_led.json")).expect("a real, captured SK6812 response must parse");
        assert_eq!(part.pads.len(), 4);
        assert_eq!(part.category, Some("Light Emitting Diodes (LED)".to_string()), "EasyEDA's own real tag for this part must come through as its category");
        for pad in &part.pads {
            assert_eq!(pad.pin_name, None, "the footprint endpoint alone must never populate pin_name");
        }
        let numbers: std::collections::HashSet<&str> = part.pads.iter().map(|p| p.number.as_str()).collect();
        assert_eq!(numbers, std::collections::HashSet::from(["1", "2", "3", "4"]));
    }

    #[test]
    fn parse_pin_names_extracts_every_pin_from_a_real_sk6812_symbol() {
        let names = parse_pin_names(&fixture("lcsc_c5378720_svgs.json"));
        assert_eq!(
            names,
            HashMap::from([
                ("1".to_string(), "GND".to_string()),
                ("2".to_string(), "DIN".to_string()),
                ("3".to_string(), "VDD".to_string()),
                ("4".to_string(), "DOUT".to_string()),
            ]),
            "must recover every real pin's function name from the real captured symbol SVG"
        );
    }

    #[test]
    fn parse_pin_names_returns_an_empty_map_rather_than_erroring_on_garbage() {
        let body: Value = serde_json::from_str(r#"{"success": true, "result": []}"#).unwrap();
        assert_eq!(parse_pin_names(&body), HashMap::new());

        let no_result: Value = serde_json::from_str(r#"{"success": true}"#).unwrap();
        assert_eq!(parse_pin_names(&no_result), HashMap::new());
    }

    #[test]
    fn parse_pin_names_ignores_non_symbol_doctypes() {
        // docType 4 (a PCB footprint preview) must never be scanned for
        // pin text, even if it happens to contain lookalike markup.
        let body: Value = serde_json::from_str(
            r#"{"success": true, "result": [{"docType": 4, "svg": "<g c_partid=\"part_pin\" c_spicepin=\"1\"><text>NOT_A_PIN_NAME</text><text>1</text></g>"}]}"#,
        )
        .unwrap();
        assert_eq!(parse_pin_names(&body), HashMap::new());
    }

    #[test]
    fn extract_pin_names_from_symbol_svg_parses_a_minimal_two_pin_group() {
        let svg = r#"<g c_partid="part_pin" c_spicepin="1"><circle/><text>GND</text><text>1</text></g><g c_partid="part_pin" c_spicepin="2"><text>VCC</text><text>2</text></g>"#;
        assert_eq!(
            extract_pin_names_from_symbol_svg(svg),
            vec![("1".to_string(), "GND".to_string()), ("2".to_string(), "VCC".to_string())]
        );
    }

    #[test]
    fn extract_pin_names_from_symbol_svg_skips_a_pin_group_with_no_name_text() {
        let svg = r#"<g c_partid="part_pin" c_spicepin="1"></g>"#;
        assert_eq!(extract_pin_names_from_symbol_svg(svg), Vec::new());
    }

    #[test]
    fn a_real_sk6812_led_s_footprint_reports_a_real_silkscreen_courtyard_close_to_its_5050_package_size() {
        // A real SK6812 5050 package is ~5.0mm x 5.0mm -- the
        // courtyard extracted from this fixture's own real top-silk
        // TRACK lines (confirmed against the actual fixture: 5.4mm x
        // 5.0mm, the extra 0.4mm being the pads' own silk end-marks
        // poking slightly past the package body) must land in that
        // ballpark, not some unrelated number.
        let part = parse_response("C5378720", &fixture("lcsc_c5378720_sk6812_led.json")).expect("a real, captured SK6812 response must parse");
        let courtyard = part.explicit_courtyard.expect("this real footprint does draw a top-silk outline");
        let width_mm = courtyard.width as f64 / alladin_geom::MM as f64;
        let height_mm = courtyard.height as f64 / alladin_geom::MM as f64;
        assert!((width_mm - 5.4).abs() < 0.05, "expected a ~5.4mm-wide courtyard, got {width_mm}mm");
        assert!((height_mm - 5.0).abs() < 0.05, "expected a ~5.0mm-tall courtyard, got {height_mm}mm");
    }

    #[test]
    fn parse_silk_courtyard_ignores_tracks_on_layers_other_than_top_silk() {
        let shapes = ["TRACK~1~1~~0 0 10 0 10 10 0 10~id~0", "TRACK~1~10~~0 0 20 0 20 20 0 20~id~0"];
        assert!(parse_silk_courtyard(&shapes, 0.0, 0.0).is_none(), "only layer-3 (top silk) tracks should ever be considered");
    }

    #[test]
    fn parse_silk_courtyard_returns_none_for_a_footprint_with_no_silk_tracks_at_all() {
        // A bare 0402/0201-class part frequently draws no silkscreen
        // outline whatsoever -- must fall back cleanly, not panic or
        // fabricate a zero-size box that would then wrongly collide
        // with everything at its own single point.
        let shapes = ["PAD~RECT~0~0~1~1~1~~1~0~~0~id~0~~Y"];
        assert!(parse_silk_courtyard(&shapes, 0.0, 0.0).is_none());
    }

    #[test]
    fn parse_silk_courtyard_combines_several_top_silk_tracks_into_one_bounding_box() {
        let shapes = [
            "TRACK~1~3~~0 0 10 0 10 5 0 5~id~0",
            "TRACK~1~3~~-4 -4 20 -4~id2~0",
            "TRACK~1~1~~-100 -100 100 100~offlayer~0",
        ];
        let courtyard = parse_silk_courtyard(&shapes, 0.0, 0.0).expect("must find a courtyard from the two top-silk tracks");
        // In EasyEDA's own units (not yet converted to nm): combined
        // bbox is x in [-4, 20], y in [-4, 5].
        assert_eq!(courtyard.width, easyeda_unit_to_nm(24.0));
        assert_eq!(courtyard.height, easyeda_unit_to_nm(9.0));
        assert_eq!(courtyard.center, Point::new(easyeda_unit_to_nm(8.0), easyeda_unit_to_nm(0.5)));
    }

    #[test]
    fn parse_silk_courtyard_reads_a_standalone_top_silk_rect_by_its_top_left_corner() {
        // Regression test for a real, confirmed bug: this function
        // used to only understand `TRACK` lines, so a real part (a
        // SOIC-14, see `a_real_sn74ahct125dr_reports_a_rect_based_silkscreen_courtyard_not_a_fallback_pad_bbox`)
        // whose only top-silk shape is a standalone `RECT~x~y~width~
        // height~layer~...` line (not a `TRACK`) silently fell back
        // to the plain pad bounding box, even though JLCPCB's own
        // EasyEDA viewer visibly draws that rectangle as the part's
        // body outline. `(x, y)` is the rect's top-left corner, not
        // its center -- a rect from (2, 3) sized 4x6 must land at
        // bbox x in [2, 6], y in [3, 9].
        let shapes = ["RECT~2~3~4~6~3~id~0~1~none"];
        let courtyard = parse_silk_courtyard(&shapes, 0.0, 0.0).expect("must find a courtyard from the standalone top-silk RECT");
        assert_eq!(courtyard.width, easyeda_unit_to_nm(4.0));
        assert_eq!(courtyard.height, easyeda_unit_to_nm(6.0));
        assert_eq!(courtyard.center, Point::new(easyeda_unit_to_nm(4.0), easyeda_unit_to_nm(6.0)));
    }

    #[test]
    fn parse_silk_courtyard_ignores_a_rect_on_a_layer_other_than_top_silk() {
        let shapes = ["RECT~2~3~4~6~1~id~0~1~none"];
        assert!(parse_silk_courtyard(&shapes, 0.0, 0.0).is_none(), "only a layer-3 (top silk) RECT should ever be considered");
    }

    #[test]
    fn parse_silk_courtyard_ignores_circle_shapes_even_on_the_top_silk_layer() {
        // The deliberate exclusion this function's own doc comment
        // explains: a top-silk CIRCLE is frequently a pin-1 dot placed
        // outside the real body, not part of it (confirmed against a
        // real SOIC-14, see `a_real_sn74ahct125dr_...` below). This one
        // sits far outside the otherwise-correct 10x10 box a same-layer
        // TRACK alone defines.
        let shapes = ["TRACK~1~3~~0 0 10 0 10 10 0 10~id~0", "CIRCLE~50~50~2~1~3~id2~0~~"];
        let courtyard = parse_silk_courtyard(&shapes, 0.0, 0.0).expect("the TRACK alone must still produce a courtyard");
        assert_eq!(courtyard.width, easyeda_unit_to_nm(10.0), "the far-away CIRCLE dot must not inflate the box");
        assert_eq!(courtyard.height, easyeda_unit_to_nm(10.0));
    }

    #[test]
    fn parse_silk_courtyard_pads_a_top_silk_arc_by_its_own_radius_in_every_direction() {
        // A real arc line's exact shape (see `extend_arc_bbox`'s own doc
        // comment): starts at (0, 0), a quarter circle of radius 5 out
        // to (5, 5). The true arc bbox is a subset of [-5, 10] x [-5, 10]
        // (each endpoint padded by the radius in every direction) --
        // deliberately generous rather than exact, which this test
        // locks in rather than the (harder, unneeded) exact arc math.
        let shapes = ["ARC~1~3~~M 0 0 A 5 5 0 0 1 5 5~~id~0"];
        let courtyard = parse_silk_courtyard(&shapes, 0.0, 0.0).expect("must find a courtyard from the standalone top-silk ARC");
        assert_eq!(courtyard.width, easyeda_unit_to_nm(15.0));
        assert_eq!(courtyard.height, easyeda_unit_to_nm(15.0));
    }

    #[test]
    fn parse_silk_courtyard_ignores_solidregion_shapes_even_on_the_top_silk_layer() {
        // The deliberate exclusion this function's own doc comment
        // explains at length: a top-silk `SOLIDREGION` is frequently a
        // pin-1 arrow/flag drawn *outside* the real body on purpose,
        // not part of the courtyard. This one sits far outside the
        // otherwise-correct 10x10 box a same-layer TRACK alone defines.
        let shapes = ["TRACK~1~3~~0 0 10 0 10 10 0 10~id~0", "SOLIDREGION~3~~M 50 50 L 60 50 L 60 60 Z~solid~id2~~~~0"];
        let courtyard = parse_silk_courtyard(&shapes, 0.0, 0.0).expect("the TRACK alone must still produce a courtyard");
        assert_eq!(courtyard.width, easyeda_unit_to_nm(10.0), "the far-away SOLIDREGION flag must not inflate the box");
        assert_eq!(courtyard.height, easyeda_unit_to_nm(10.0));
    }

    #[test]
    fn a_real_ceramic_capacitor_s_rounded_corner_arcs_are_required_for_its_real_courtyard_size() {
        // Regression test for a real, confirmed case where `TRACK`
        // alone under-reports the real size: this real 0603 ceramic
        // capacitor's silk outline is straight TRACK segments joined
        // by a quarter-circle ARC at each of its four rounded corners.
        // TRACK-only would report a real, confirmed-too-small ~2.8mm x
        // ~1.4mm; the real part (and its real courtyard, once the
        // corner arcs are included too) is ~3.4mm x ~2.0mm.
        let part = parse_response("C14663", &fixture("lcsc_c14663_ceramic_capacitor.json")).expect("a real, captured 0603 ceramic capacitor response must parse");
        let courtyard = part.explicit_courtyard.expect("this real footprint's rounded-corner silk outline must be found");
        let width_mm = courtyard.width as f64 / alladin_geom::MM as f64;
        let height_mm = courtyard.height as f64 / alladin_geom::MM as f64;
        assert!((width_mm - 3.4).abs() < 0.05, "expected a ~3.4mm-wide courtyard (corner arcs included), got {width_mm}mm");
        assert!((height_mm - 2.04).abs() < 0.05, "expected a ~2.04mm-tall courtyard (corner arcs included), got {height_mm}mm");
    }

    #[test]
    fn a_real_sn74ahct125dr_reports_a_rect_based_silkscreen_courtyard_not_a_fallback_pad_bbox() {
        // The exact bug the user caught by comparing Alladin's output
        // against JLCPCB's own part page: this real SOIC-14's part
        // page visibly draws a small rectangular body outline around
        // its footprint, but this part's *only* top-silk shape drawing
        // that outline is a standalone `RECT` line, not a `TRACK` --
        // so before the fix above, `explicit_courtyard` came back
        // `None` and silently fell all the way back to the plain (much
        // larger, lead-tip-to-lead-tip) pad bounding box instead.
        //
        // This same real fixture also has 4 top-silk `CIRCLE`s (pin-1
        // dots) that must be *ignored*, not folded in -- confirmed
        // real evidence for [`parse_silk_courtyard`]'s own documented
        // CIRCLE exclusion: one of them sits far enough past this real
        // body that including it would inflate the height to a wrong
        // ~4.32mm instead of the real ~2.79mm.
        let part = parse_response("C155176", &fixture("lcsc_c155176_sn74ahct125dr.json")).expect("a real, captured SN74AHCT125DR response must parse");
        let courtyard = part.explicit_courtyard.expect("this real footprint's top-silk RECT must now be found");
        let width_mm = courtyard.width as f64 / alladin_geom::MM as f64;
        let height_mm = courtyard.height as f64 / alladin_geom::MM as f64;
        // Confirmed against the real fixture's own RECT~3982.75~2994.5~34.5~11~3~...
        // line: 34.5 x 11 EasyEDA units, i.e. ~8.76mm x ~2.79mm --
        // clearly smaller than the pads' own ~7.62mm x ~5.24mm span in
        // the narrow (pin-row) direction, exactly matching a SOIC-14
        // body silhouette drawn *inside* its own gull-wing lead span.
        assert!((width_mm - 8.76).abs() < 0.05, "expected a ~8.76mm-wide courtyard, got {width_mm}mm");
        assert!((height_mm - 2.79).abs() < 0.05, "expected a ~2.79mm-tall courtyard, got {height_mm}mm");
    }
}
