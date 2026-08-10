//! MCP helpers for AI-driven manual routing: scene snapshot, batched
//! clearance probes, commit, and rip-up. Reuses the same gates as the
//! GUI's interactive router (`Node::path_is_clear`, edge clearance,
//! `try_add_via`) — not an autorouter.

use alladin_core::{Item, ItemId, JlcpcbDfm, LayerId, NetClass, NetId};
use alladin_geom::{Point, Segment, Unit, MM};
use serde_json::{json, Value};
use std::collections::HashSet;

use crate::board_doc::{BoardDoc, PlacementError, DEFAULT_VIA_DIAMETER, DEFAULT_VIA_DRILL};
use crate::footprint::FootprintTemplate;
use crate::routing::{path_keeps_edge_margin, DEFAULT_TRACE_WIDTH};

/// How far MCP-committed copper stays from the board edge by default --
/// deliberately wider than [`JlcpcbDfm::COPPER_TO_ROUTED_EDGE`]'s hard
/// 0.2mm fab minimum. Routing exactly at the fab limit is legal but
/// leaves zero reserve (JLCDFM's own measurement of a gerber can come
/// out a few hundredths shorter and flag a warning); a human keeps
/// comfortable distance from the cut line unless space forces the
/// issue, so the AI router does too. A candidate can lower it per-call
/// via `edge_margin_mm`, but never below the fab minimum.
pub const EDGE_COMFORT_MARGIN: Unit = MM;

fn mm(v: f64) -> Unit {
    (v * MM as f64).round() as Unit
}

fn unit_mm(u: Unit) -> f64 {
    u as f64 / MM as f64
}

fn layer_name(layer: LayerId) -> &'static str {
    match layer {
        LayerId::FCu => "FCu",
        LayerId::BCu => "BCu",
    }
}

pub(crate) fn parse_layer(s: &str) -> Result<LayerId, String> {
    match s.trim() {
        "FCu" | "F.Cu" | "fcu" | "F" | "top" => Ok(LayerId::FCu),
        "BCu" | "B.Cu" | "bcu" | "B" | "bottom" => Ok(LayerId::BCu),
        other => Err(format!("unknown layer \"{other}\" -- use FCu or BCu")),
    }
}

pub(crate) fn pad_label(doc: &BoardDoc, templates: &[FootprintTemplate], pad_id: ItemId) -> Option<(String, String)> {
    let footprint = doc.footprints.iter().find(|f| f.pad_item_ids.contains(&pad_id))?;
    let index = footprint.pad_item_ids.iter().position(|&id| id == pad_id)?;
    let pin = templates
        .iter()
        .find(|t| t.name == footprint.template_name)
        .and_then(|t| t.pads.get(index))
        .map(|p| p.number.clone())
        .unwrap_or_else(|| (index + 1).to_string());
    Some((footprint.reference.clone(), pin))
}

fn net_by_name<'a>(doc: &'a BoardDoc, name: &str) -> Result<&'a crate::board_doc::NetRecord, String> {
    doc.nets.iter().find(|n| n.name == name).ok_or_else(|| format!("no net named \"{name}\" -- get_routing_scene / get_nets list current names"))
}

/// One-call geometry snapshot for AI routing (pads, copper, open bridges, rules).
pub fn routing_scene_json(doc: &BoardDoc, templates: &[FootprintTemplate]) -> Value {
    let pads: Vec<Value> = doc
        .footprints
        .iter()
        .flat_map(|fp| {
            let template = templates.iter().find(|t| t.name == fp.template_name);
            fp.pad_item_ids.iter().enumerate().filter_map(move |(index, &pad_id)| {
                let (center, layer, net_id) = doc.pad_endpoint(pad_id)?;
                let pin = template.and_then(|t| t.pads.get(index)).map(|p| p.number.clone()).unwrap_or_else(|| (index + 1).to_string());
                let net = net_id.map(|id| doc.nets.iter().find(|n| n.id == id).map(|n| n.name.as_str()).unwrap_or("?").to_string());
                Some(json!({
                    "ref": fp.reference,
                    "pin": pin,
                    "net": net,
                    "x_mm": unit_mm(center.x),
                    "y_mm": unit_mm(center.y),
                    "layer": layer_name(layer),
                }))
            })
        })
        .collect();

    let mut tracks = Vec::new();
    let mut vias = Vec::new();
    for (id, item) in doc.node.iter_with_ids() {
        match item {
            Item::Track { shape, net, layer, .. } => {
                let net_name = net.and_then(|nid| doc.nets.iter().find(|n| n.id == nid).map(|n| n.name.clone()));
                tracks.push(json!({
                    "id": id.0,
                    "net": net_name,
                    "layer": layer_name(*layer),
                    "width_mm": unit_mm(shape.width),
                    "a_mm": [unit_mm(shape.a.x), unit_mm(shape.a.y)],
                    "b_mm": [unit_mm(shape.b.x), unit_mm(shape.b.y)],
                }));
            }
            Item::Via { shape, drill, net } => {
                let net_name = net.and_then(|nid| doc.nets.iter().find(|n| n.id == nid).map(|n| n.name.clone()));
                vias.push(json!({
                    "id": id.0,
                    "net": net_name,
                    "x_mm": unit_mm(shape.center.x),
                    "y_mm": unit_mm(shape.center.y),
                    "diameter_mm": unit_mm(shape.radius * 2),
                    "drill_mm": unit_mm(*drill),
                }));
            }
            _ => {}
        }
    }

    let open_bridges = open_bridges_json(doc, templates);

    json!({
        "pads": pads,
        "tracks": tracks,
        "vias": vias,
        "open_bridges": open_bridges,
        "rules": {
            "min_copper_clearance_mm": unit_mm(doc.pad_to_pad_clearance()),
            "copper_to_board_edge_mm": unit_mm(JlcpcbDfm::COPPER_TO_ROUTED_EDGE),
            "edge_comfort_margin_mm": unit_mm(EDGE_COMFORT_MARGIN),
            "default_trace_width_mm": unit_mm(DEFAULT_TRACE_WIDTH),
            "default_via_diameter_mm": unit_mm(DEFAULT_VIA_DIAMETER),
            "default_via_drill_mm": unit_mm(DEFAULT_VIA_DRILL),
        },
    })
}

/// Closest pad-to-pad links between copper islands that still need joining.
pub(crate) fn open_bridges_json(doc: &BoardDoc, templates: &[FootprintTemplate]) -> Vec<Value> {
    let mut bridges = Vec::new();
    for net in &doc.nets {
        if doc.pads_on_net(net.id).len() < 2 {
            continue;
        }
        let components = doc.node.net_copper_components(net.id);
        if components.len() <= 1 {
            continue;
        }
        let island_pads: Vec<Vec<(ItemId, Point)>> = components
            .iter()
            .map(|ids| {
                ids.iter()
                    .filter_map(|&id| {
                        let Item::Pad { shape, .. } = doc.node.get(id)? else { return None };
                        Some((id, shape.center()))
                    })
                    .collect()
            })
            .collect();
        // Skip empty-pad islands (track-only debris); still report if ≥2 pad islands remain.
        let pad_islands: Vec<(usize, &Vec<(ItemId, Point)>)> =
            island_pads.iter().enumerate().filter(|(_, pads)| !pads.is_empty()).collect();
        if pad_islands.len() < 2 {
            continue;
        }
        for i in 0..pad_islands.len() {
            for j in (i + 1)..pad_islands.len() {
                let (_, pads_a) = pad_islands[i];
                let (_, pads_b) = pad_islands[j];
                let mut best: Option<(f64, ItemId, ItemId, Point, Point)> = None;
                for &(id_a, pa) in pads_a.iter() {
                    for &(id_b, pb) in pads_b.iter() {
                        let dx = (pa.x - pb.x) as f64;
                        let dy = (pa.y - pb.y) as f64;
                        let dist = dx.hypot(dy);
                        if best.map(|b| dist < b.0).unwrap_or(true) {
                            best = Some((dist, id_a, id_b, pa, pb));
                        }
                    }
                }
                if let Some((dist, id_a, id_b, pa, pb)) = best {
                    let (ref_a, pin_a) = pad_label(doc, templates, id_a).unwrap_or_else(|| ("?".into(), "?".into()));
                    let (ref_b, pin_b) = pad_label(doc, templates, id_b).unwrap_or_else(|| ("?".into(), "?".into()));
                    bridges.push((
                        dist,
                        json!({
                            "net": net.name,
                            "distance_mm": dist / MM as f64,
                            "a": { "ref": ref_a, "pin": pin_a, "x_mm": unit_mm(pa.x), "y_mm": unit_mm(pa.y) },
                            "b": { "ref": ref_b, "pin": pin_b, "x_mm": unit_mm(pb.x), "y_mm": unit_mm(pb.y) },
                        }),
                    ));
                }
            }
        }
    }
    bridges.sort_by(|a, b| a.0.total_cmp(&b.0));
    bridges.into_iter().take(100).map(|(_, v)| v).collect()
}

/// Compact open-bridge score for AI floorplanning: sum/max/count plus the
/// shortest `top_n` bridges (capped at 20). Distances are millimetres.
pub(crate) fn open_bridges_score_json(doc: &BoardDoc, templates: &[FootprintTemplate], top_n: usize) -> Value {
    let bridges = open_bridges_json(doc, templates);
    let distances: Vec<f64> = bridges.iter().filter_map(|b| b.get("distance_mm").and_then(|v| v.as_f64())).collect();
    let sum_mm: f64 = distances.iter().sum();
    let max_mm = distances.iter().copied().fold(0.0_f64, f64::max);
    let top_n = top_n.min(20).min(bridges.len());
    json!({
        "sum_mm": sum_mm,
        "max_mm": max_mm,
        "count": bridges.len(),
        "top": bridges.into_iter().take(top_n).collect::<Vec<_>>(),
    })
}

/// Parsed route candidate (one or more layer segments + vias at junctions).
#[derive(Debug, Clone)]
pub struct ParsedRoute {
    pub net: NetId,
    pub net_name: String,
    pub width: Unit,
    pub via_diameter: Unit,
    pub via_drill: Unit,
    pub segments: Vec<(LayerId, Vec<Point>)>,
    pub vias: Vec<Point>,
    pub edge_margin: Unit,
}

pub fn parse_route_candidate(doc: &BoardDoc, candidate: &Value) -> Result<ParsedRoute, String> {
    let net_name = candidate
        .get("net")
        .and_then(|v| v.as_str())
        .ok_or_else(|| "candidate missing string \"net\"".to_string())?;
    let net = net_by_name(doc, net_name)?.id;
    let width = candidate.get("width_mm").and_then(|v| v.as_f64()).map(mm).unwrap_or(DEFAULT_TRACE_WIDTH);
    let via_diameter = candidate.get("via_diameter_mm").and_then(|v| v.as_f64()).map(mm).unwrap_or(DEFAULT_VIA_DIAMETER);
    let via_drill = candidate.get("via_drill_mm").and_then(|v| v.as_f64()).map(mm).unwrap_or(DEFAULT_VIA_DRILL);
    let edge_margin = match candidate.get("edge_margin_mm").and_then(|v| v.as_f64()) {
        Some(v) => {
            let margin = mm(v);
            if margin < JlcpcbDfm::COPPER_TO_ROUTED_EDGE {
                return Err(format!(
                    "edge_margin_mm {v} is below JLCPCB's hard copper-to-edge minimum of {}mm",
                    unit_mm(JlcpcbDfm::COPPER_TO_ROUTED_EDGE)
                ));
            }
            margin
        }
        None => EDGE_COMFORT_MARGIN,
    };

    let segments_val = candidate
        .get("segments")
        .and_then(|v| v.as_array())
        .ok_or_else(|| "candidate needs a non-empty \"segments\" array".to_string())?;
    if segments_val.is_empty() {
        return Err("candidate \"segments\" must not be empty".into());
    }

    let mut segments = Vec::with_capacity(segments_val.len());
    for (i, seg) in segments_val.iter().enumerate() {
        let layer = parse_layer(seg.get("layer").and_then(|v| v.as_str()).ok_or_else(|| format!("segments[{i}] missing \"layer\""))?)?;
        let points_raw = seg
            .get("points_mm")
            .and_then(|v| v.as_array())
            .ok_or_else(|| format!("segments[{i}] missing \"points_mm\" array"))?;
        if points_raw.len() < 2 {
            return Err(format!("segments[{i}].points_mm needs at least 2 points"));
        }
        let mut points = Vec::with_capacity(points_raw.len());
        for (j, p) in points_raw.iter().enumerate() {
            let arr = p.as_array().ok_or_else(|| format!("segments[{i}].points_mm[{j}] must be [x_mm, y_mm]"))?;
            if arr.len() != 2 {
                return Err(format!("segments[{i}].points_mm[{j}] must be [x_mm, y_mm]"));
            }
            let x = arr[0].as_f64().ok_or_else(|| format!("segments[{i}].points_mm[{j}][0] must be a number"))?;
            let y = arr[1].as_f64().ok_or_else(|| format!("segments[{i}].points_mm[{j}][1] must be a number"))?;
            points.push(Point::new(mm(x), mm(y)));
        }
        segments.push((layer, points));
    }

    let vias_raw = candidate.get("vias_mm").and_then(|v| v.as_array());
    let vias = if let Some(arr) = vias_raw {
        let mut vias = Vec::with_capacity(arr.len());
        for (j, p) in arr.iter().enumerate() {
            let xy = p.as_array().ok_or_else(|| format!("vias_mm[{j}] must be [x_mm, y_mm]"))?;
            if xy.len() != 2 {
                return Err(format!("vias_mm[{j}] must be [x_mm, y_mm]"));
            }
            let x = xy[0].as_f64().ok_or_else(|| format!("vias_mm[{j}][0] must be a number"))?;
            let y = xy[1].as_f64().ok_or_else(|| format!("vias_mm[{j}][1] must be a number"))?;
            vias.push(Point::new(mm(x), mm(y)));
        }
        vias
    } else {
        Vec::new()
    };

    if segments.len() > 1 && vias.len() != segments.len() - 1 {
        return Err(format!(
            "multi-layer route needs vias_mm with {} entries (one at each layer junction); got {}",
            segments.len() - 1,
            vias.len()
        ));
    }
    if segments.len() == 1 && !vias.is_empty() {
        return Err("single-layer route must not include vias_mm (or add a second segment on the other layer)".into());
    }

    for (i, via) in vias.iter().enumerate() {
        let end_a = *segments[i].1.last().unwrap();
        let start_b = segments[i + 1].1[0];
        if end_a != *via || start_b != *via {
            return Err(format!(
                "vias_mm[{i}] must equal the junction: last point of segments[{i}] and first point of segments[{}]",
                i + 1
            ));
        }
        if segments[i].0 == segments[i + 1].0 {
            return Err(format!("segments[{i}] and segments[{}] share a via but are on the same layer", i + 1));
        }
    }

    Ok(ParsedRoute {
        net,
        net_name: net_name.to_string(),
        width,
        via_diameter,
        via_drill,
        segments,
        vias,
        edge_margin,
    })
}

/// Compact description of an existing board item, for blocker feedback:
/// what kind it is, whose net, which layer, roughly where, and (for pads)
/// which footprint owns it.
fn describe_item(doc: &BoardDoc, id: ItemId) -> Option<Value> {
    let item = doc.node.get(id)?;
    let net_name = item.net().and_then(|nid| doc.nets.iter().find(|n| n.id == nid).map(|n| n.name.clone()));
    let (kind, center, layer) = match item {
        Item::Pad { shape, layer, .. } => ("pad", shape.center(), Some(*layer)),
        Item::Via { shape, .. } => ("via", shape.center, None),
        Item::Track { shape, layer, .. } => {
            ("track", Point::new((shape.a.x + shape.b.x) / 2, (shape.a.y + shape.b.y) / 2), Some(*layer))
        }
        Item::Zone { outline, layer, .. } => ("zone", outline.points.first().copied().unwrap_or(Point::new(0, 0)), Some(*layer)),
        Item::Hole { position, .. } => ("hole", *position, None),
    };
    let footprint = doc.footprints.iter().find(|f| f.pad_item_ids.contains(&id)).map(|f| f.reference.clone());
    Some(json!({
        "kind": kind,
        "net": net_name,
        "footprint": footprint,
        "layer": layer.map(layer_name),
        "x_mm": unit_mm(center.x),
        "y_mm": unit_mm(center.y),
    }))
}

/// Up to three colliding items a probe track on `layer` would hit along
/// `from`→`to` — the detail the GUI shows as "red preview" implicitly.
fn leg_blockers_json(doc: &BoardDoc, from: Point, to: Point, width: Unit, net: NetId, layer: LayerId) -> Vec<Value> {
    let probe = Item::Track {
        shape: Segment::new(from, to, width),
        net: Some(net),
        layer,
        class: NetClass::C,
    };
    doc.node
        .query_colliding(&probe, doc.resolver())
        .into_iter()
        .take(3)
        .filter_map(|id| describe_item(doc, id))
        .collect()
}

fn leg_mm(from: Point, to: Point) -> Value {
    json!([[unit_mm(from.x), unit_mm(from.y)], [unit_mm(to.x), unit_mm(to.y)]])
}

/// Per-leg clearance + edge gates for one segment path; on blockage
/// returns `{blocked, leg_index, leg_mm, colliding?}` naming the exact
/// leg and the first items in the way.
fn path_block_json(doc: &BoardDoc, path: &[Point], width: Unit, net: NetId, layer: LayerId, edge_margin: Unit) -> Option<Value> {
    if path.len() < 2 {
        return Some(json!({ "blocked": "path needs at least 2 points" }));
    }
    let resolver = doc.resolver();
    for (i, leg) in path.windows(2).enumerate() {
        if !doc.node.path_is_clear(leg[0], leg[1], width, Some(net), layer, NetClass::C, resolver) {
            return Some(json!({
                "blocked": "clearance",
                "leg_index": i,
                "leg_mm": leg_mm(leg[0], leg[1]),
                "colliding": leg_blockers_json(doc, leg[0], leg[1], width, net, layer),
            }));
        }
    }
    for (i, leg) in path.windows(2).enumerate() {
        if !path_keeps_edge_margin(leg, width, &doc.outline, edge_margin) {
            return Some(json!({
                "blocked": "edge",
                "leg_index": i,
                "leg_mm": leg_mm(leg[0], leg[1]),
                "edge_margin_mm": unit_mm(edge_margin),
                "hint": format!(
                    "comfort margin is {}mm by default; pass edge_margin_mm (min {}mm, the fab limit) to route closer to the edge on purpose",
                    unit_mm(EDGE_COMFORT_MARGIN),
                    unit_mm(JlcpcbDfm::COPPER_TO_ROUTED_EDGE)
                ),
            }));
        }
    }
    None
}

/// Read-only via gates matching [`BoardDoc::try_add_via`] (no mutation).
/// On blockage names the reason and, for collisions, the items in the way.
fn via_block_json(doc: &BoardDoc, center: Point, net: NetId, diameter: Unit, drill: Unit, edge_margin: Unit) -> Option<Value> {
    if let Err(v) = JlcpcbDfm::check_via(diameter, drill) {
        return Some(json!({ "blocked": format!("via: {v}") }));
    }
    let radius = diameter / 2;
    if !alladin_geom::circle_within_outline(center, radius + edge_margin, &doc.outline) {
        return Some(json!({ "blocked": "via: too close to board edge", "edge_margin_mm": unit_mm(edge_margin) }));
    }
    if doc.violates_hole_to_hole(center, drill, Some(net)) {
        return Some(json!({ "blocked": "via: hole-to-hole spacing" }));
    }
    if doc.via_too_close_to_any_track(center, diameter) {
        return Some(json!({
            "blocked": "via: on or too close to a track",
            "hint": "a drill through a trace severs that copper (same-net included); place the via beside the track, or end the track at the via",
        }));
    }
    let resolver = doc.resolver();
    let candidate = Item::Via {
        shape: alladin_geom::Circle { center, radius },
        drill,
        net: Some(net),
    };
    let colliding: Vec<Value> = doc
        .node
        .query_colliding(&candidate, resolver)
        .into_iter()
        .take(3)
        .filter_map(|id| describe_item(doc, id))
        .collect();
    if !colliding.is_empty() {
        return Some(json!({ "blocked": "via: clearance", "colliding": colliding }));
    }
    None
}

/// Probe one parsed candidate against the live board (no mutation).
pub fn probe_one(doc: &BoardDoc, route: &ParsedRoute) -> Value {
    for (i, (layer, path)) in route.segments.iter().enumerate() {
        if let Some(mut detail) = path_block_json(doc, path, route.width, route.net, *layer, route.edge_margin) {
            let obj = detail.as_object_mut().unwrap();
            obj.insert("ok".into(), json!(false));
            obj.insert("segment_index".into(), json!(i));
            obj.insert("layer".into(), json!(layer_name(*layer)));
            obj.insert("net".into(), json!(route.net_name));
            return detail;
        }
    }
    for (i, via) in route.vias.iter().enumerate() {
        if let Some(mut detail) = via_block_json(doc, *via, route.net, route.via_diameter, route.via_drill, route.edge_margin) {
            let obj = detail.as_object_mut().unwrap();
            obj.insert("ok".into(), json!(false));
            obj.insert("via_index".into(), json!(i));
            obj.insert("net".into(), json!(route.net_name));
            return detail;
        }
    }
    json!({ "ok": true, "net": route.net_name, "segment_count": route.segments.len(), "via_count": route.vias.len() })
}

pub fn probe_routes_json(doc: &BoardDoc, candidates: &[Value]) -> Value {
    let mut results = Vec::with_capacity(candidates.len());
    for (index, cand) in candidates.iter().enumerate() {
        match parse_route_candidate(doc, cand) {
            Ok(route) => {
                let mut r = probe_one(doc, &route);
                if let Some(obj) = r.as_object_mut() {
                    obj.insert("index".into(), json!(index));
                }
                results.push(r);
            }
            Err(e) => results.push(json!({ "ok": false, "index": index, "blocked": e })),
        }
    }
    json!({ "results": results })
}

/// One-line human summary of a `probe_one` blockage, for commit errors.
fn block_summary(detail: &Value) -> String {
    let mut s = detail.get("blocked").and_then(|b| b.as_str()).unwrap_or("blocked").to_string();
    if let Some(idx) = detail.get("segment_index").and_then(|v| v.as_u64()) {
        s.push_str(&format!(" (segment {idx}"));
        if let Some(leg) = detail.get("leg_index").and_then(|v| v.as_u64()) {
            s.push_str(&format!(", leg {leg}"));
        }
        s.push(')');
    }
    if let Some(hit) = detail.get("colliding").and_then(|c| c.as_array()).and_then(|a| a.first()) {
        let kind = hit["kind"].as_str().unwrap_or("item");
        let net = hit["net"].as_str().unwrap_or("no-net");
        let x = hit["x_mm"].as_f64().unwrap_or(0.0);
        let y = hit["y_mm"].as_f64().unwrap_or(0.0);
        s.push_str(&format!(" — hits {kind} on net {net} near ({x:.2}, {y:.2}) mm"));
    }
    s
}

/// Apply a cleared route, then verify it actually joined two of the net's
/// copper islands. A geometrically clean route that lands in free space or
/// on the wrong layer is rolled back and reported as an error — no more
/// false-positive commits.
pub fn commit_route(doc: &mut BoardDoc, route: &ParsedRoute) -> Result<Value, String> {
    let probe = probe_one(doc, route);
    if probe.get("ok").and_then(|v| v.as_bool()) != Some(true) {
        return Err(block_summary(&probe));
    }
    let pieces_before = doc.node.net_copper_components(route.net).len();
    let ids_before: HashSet<usize> = doc.node.iter_with_ids().map(|(id, _)| id.0).collect();

    let rollback = |doc: &mut BoardDoc, ids_before: &HashSet<usize>| {
        let added: Vec<ItemId> =
            doc.node.iter_with_ids().map(|(id, _)| id).filter(|id| !ids_before.contains(&id.0)).collect();
        for id in added {
            doc.node.remove(id);
        }
    };

    // Via before the track that ends at it -- same order as the GUI's
    // mid-route via drop. `try_add_via` refuses landing on any existing
    // track (same-net included), so laying the stub first would always
    // self-block at the junction.
    for (i, (layer, path)) in route.segments.iter().enumerate() {
        if i < route.vias.len() {
            if let Err(e) = doc.try_add_via(route.vias[i], route.net, route.via_diameter, route.via_drill) {
                rollback(doc, &ids_before);
                let e: PlacementError = e;
                return Err(format!("via after segment {i}: {e} — rolled back"));
            }
        }
        doc.add_track_path(path, route.net, *layer, route.width, NetClass::C);
    }

    let pieces_after = doc.node.net_copper_components(route.net).len();
    let joined = pieces_after < pieces_before;
    let redundant_but_attached = pieces_before == 1 && pieces_after == 1;
    if !joined && !redundant_but_attached {
        rollback(doc, &ids_before);
        return Err(format!(
            "route was clear but did not join {}'s copper ({pieces_before} island(s) before, \
             {pieces_after} after) — both route ends must land on existing copper of that \
             net, on the right layer; rolled back",
            route.net_name
        ));
    }
    Ok(json!({
        "segment_count": route.segments.len(),
        "via_count": route.vias.len(),
        "copper_pieces_before": pieces_before,
        "copper_pieces_after": pieces_after,
        "bridge_closed": pieces_after < pieces_before,
    }))
}

// ---------------------------------------------------------------------------
// suggest_route: server-side octilinear A* pathfinder
// ---------------------------------------------------------------------------

/// Knobs for [`suggest_route`]. Defaults live in the MCP handler so the
/// tool schema documents them; this struct always carries resolved values.
pub struct SuggestOptions {
    pub layer: LayerId,
    pub width: Unit,
    pub edge_margin: Unit,
    /// Lattice pitch of the search. Smaller squeezes through tighter
    /// gaps but costs quadratically more probes.
    pub step: Unit,
    /// Extra cost per 45° direction change -- higher means straighter,
    /// calmer traces with fewer kinks.
    pub bend_penalty: Unit,
    /// Search budget; exceeded means "no path found" rather than a hang.
    pub max_expansions: usize,
}

/// A found path plus search diagnostics.
#[derive(Debug)]
pub struct Suggestion {
    /// Merged waypoints from start to goal; every leg is horizontal,
    /// vertical, or 45°, and consecutive legs never meet at 90°.
    pub points: Vec<Point>,
    pub expansions: usize,
}

/// The eight octilinear directions, indexed so that neighbours in the
/// array are 45° apart (the no-90°-corner rule becomes "index differs
/// by at most 1 mod 8").
const OCT_DIRS: [(i64, i64); 8] = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)];

/// Sentinel "no incoming direction yet" for the start state.
const NO_DIR: usize = 8;

/// Which of [`OCT_DIRS`] a delta points along, if it is exactly
/// octilinear (axis-aligned or a perfect 45° diagonal).
fn octant_of(dx: i64, dy: i64) -> Option<usize> {
    if dx == 0 && dy == 0 {
        return None;
    }
    if dx != 0 && dy != 0 && dx.abs() != dy.abs() {
        return None;
    }
    let key = (dx.signum(), dy.signum());
    OCT_DIRS.iter().position(|&d| d == key)
}

/// A turn is legal when the direction stays or changes by one octant
/// (45°). This forbids both 90° corners and 135° hairpins.
fn turn_ok(from: usize, to: usize) -> bool {
    if from == NO_DIR {
        return true;
    }
    let d = (8 + to - from) % 8;
    d == 0 || d == 1 || d == 7
}

/// The same two gates every probed candidate leg passes: copper
/// clearance on `layer` and the (comfort) edge margin.
fn leg_is_legal(doc: &BoardDoc, from: Point, to: Point, width: Unit, net: NetId, layer: LayerId, edge_margin: Unit) -> bool {
    doc.node.path_is_clear(from, to, width, Some(net), layer, NetClass::C, doc.resolver())
        && path_keeps_edge_margin(&[from, to], width, &doc.outline, edge_margin)
}

/// Candidate octilinear joins from an on-lattice point to the off-lattice
/// goal: one straight leg when the delta already is octilinear, otherwise
/// diagonal-then-axis and axis-then-diagonal (both meet at 135°, legal).
fn oct_joins(from: Point, to: Point) -> Vec<Vec<Point>> {
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    if dx == 0 && dy == 0 {
        return vec![Vec::new()];
    }
    if dx == 0 || dy == 0 || dx.abs() == dy.abs() {
        return vec![vec![to]];
    }
    let d = dx.abs().min(dy.abs());
    let (sx, sy) = (dx.signum(), dy.signum());
    let diag_first = Point::new(from.x + sx * d, from.y + sy * d);
    let axis_first = if dx.abs() > dy.abs() { Point::new(to.x - sx * d, from.y) } else { Point::new(from.x, to.y - sy * d) };
    vec![vec![diag_first, to], vec![axis_first, to]]
}

/// Drop interior waypoints where the direction doesn't change, so the
/// returned polyline has one point per actual bend.
fn merge_collinear(points: Vec<Point>) -> Vec<Point> {
    let mut merged: Vec<Point> = Vec::with_capacity(points.len());
    for p in points {
        if merged.last() == Some(&p) {
            continue;
        }
        while merged.len() >= 2 {
            let a = merged[merged.len() - 2];
            let b = merged[merged.len() - 1];
            let d1 = octant_of(b.x - a.x, b.y - a.y);
            let d2 = octant_of(p.x - b.x, p.y - b.y);
            if d1.is_some() && d1 == d2 {
                merged.pop();
            } else {
                break;
            }
        }
        merged.push(p);
    }
    merged
}

/// Octilinear A* over a lattice anchored at `start`: finds a legal
/// 45°-style path from `start` to `goal` on one layer, using exactly the
/// clearance and edge gates `probe_route` applies -- so the result is
/// commit-ready by construction. No 90° corners, no vias. States are
/// (lattice point, incoming direction); the goal, generally off-lattice,
/// is joined by a final one- or two-leg octilinear decomposition.
pub fn suggest_route(doc: &BoardDoc, net: NetId, start: Point, goal: Point, opts: &SuggestOptions) -> Result<Suggestion, String> {
    use std::cmp::Reverse;
    use std::collections::{BinaryHeap, HashMap};

    let step = opts.step.max(MM / 20);
    let join_radius = (step * 8) as f64;
    let bend_penalty = opts.bend_penalty.max(0) as f64;
    let sqrt2 = std::f64::consts::SQRT_2;

    let pos_of = |ix: i64, iy: i64| Point::new(start.x + ix * step, start.y + iy * step);
    let heuristic = |p: Point| {
        let dx = ((goal.x - p.x) as f64).abs();
        let dy = ((goal.y - p.y) as f64).abs();
        dx.max(dy) + (sqrt2 - 1.0) * dx.min(dy)
    };
    let legal = |from: Point, to: Point| leg_is_legal(doc, from, to, opts.width, net, opts.layer, opts.edge_margin);

    // Try to finish from `state`; on success returns the join waypoints.
    let try_join = |pos: Point, dir: usize| -> Option<Vec<Point>> {
        if pos.distance(goal) > join_radius {
            return None;
        }
        'variant: for join in oct_joins(pos, goal) {
            let mut prev = pos;
            let mut prev_dir = dir;
            for &p in &join {
                let Some(d) = octant_of(p.x - prev.x, p.y - prev.y) else { continue 'variant };
                if !turn_ok(prev_dir, d) || !legal(prev, p) {
                    continue 'variant;
                }
                prev = p;
                prev_dir = d;
            }
            return Some(join);
        }
        None
    };

    type State = (i64, i64, usize);
    let mut g: HashMap<State, f64> = HashMap::new();
    let mut parent: HashMap<State, State> = HashMap::new();
    let mut open: BinaryHeap<(Reverse<i64>, i64, i64, usize)> = BinaryHeap::new();

    let start_state: State = (0, 0, NO_DIR);
    g.insert(start_state, 0.0);
    open.push((Reverse(heuristic(start) as i64), 0, 0, NO_DIR));

    let mut expansions = 0usize;
    while let Some((Reverse(f_key), ix, iy, dir)) = open.pop() {
        let state = (ix, iy, dir);
        let g_here = match g.get(&state) {
            Some(&v) => v,
            None => continue,
        };
        // Stale heap entry (a cheaper path to this state was found later).
        if ((g_here + heuristic(pos_of(ix, iy))) as i64) < f_key {
            continue;
        }
        let pos = pos_of(ix, iy);

        if let Some(join) = try_join(pos, dir) {
            let mut points = vec![pos];
            let mut cur = state;
            while let Some(&prev) = parent.get(&cur) {
                points.push(pos_of(prev.0, prev.1));
                cur = prev;
            }
            points.reverse();
            points.extend(join);
            return Ok(Suggestion { points: merge_collinear(points), expansions });
        }

        expansions += 1;
        if expansions > opts.max_expansions {
            return Err(format!(
                "no path found within the search budget ({} expansions) -- the corridor may be \
                 blocked; try a larger max_expansions, a smaller step_mm, another layer, or rip \
                 up blocking copper",
                opts.max_expansions
            ));
        }

        let dirs: &[usize] = if dir == NO_DIR { &[0, 1, 2, 3, 4, 5, 6, 7] } else { &[dir, (dir + 1) % 8, (dir + 7) % 8] };
        for &nd in dirs {
            let (dx, dy) = OCT_DIRS[nd];
            let (nix, niy) = (ix + dx, iy + dy);
            let npos = pos_of(nix, niy);
            let leg_len = if dx != 0 && dy != 0 { step as f64 * sqrt2 } else { step as f64 };
            let cost = g_here + leg_len + if dir != NO_DIR && nd != dir { bend_penalty } else { 0.0 };
            let nstate: State = (nix, niy, nd);
            if g.get(&nstate).map(|&old| cost >= old).unwrap_or(false) {
                continue;
            }
            if !legal(pos, npos) {
                continue;
            }
            g.insert(nstate, cost);
            parent.insert(nstate, state);
            open.push((Reverse((cost + heuristic(npos)) as i64), nix, niy, nd));
        }
    }

    Err(format!(
        "no legal octilinear path from ({:.2}, {:.2}) to ({:.2}, {:.2}) on {} (searched {} states) \
         -- the corridor is blocked; rip up blocking copper, try the other layer, or route manually \
         with probe_route",
        unit_mm(start.x),
        unit_mm(start.y),
        unit_mm(goal.x),
        unit_mm(goal.y),
        layer_name(opts.layer),
        expansions
    ))
}

fn nearest_routed_item(doc: &BoardDoc, point: Point) -> Option<ItemId> {
    let mut best: Option<(f64, ItemId)> = None;
    for (id, item) in doc.node.iter_with_ids() {
        let dist = match item {
            Item::Track { shape, .. } => {
                let ax = shape.a.x as f64;
                let ay = shape.a.y as f64;
                let bx = shape.b.x as f64;
                let by = shape.b.y as f64;
                let px = point.x as f64;
                let py = point.y as f64;
                let abx = bx - ax;
                let aby = by - ay;
                let apx = px - ax;
                let apy = py - ay;
                let ab2 = abx * abx + aby * aby;
                let t = if ab2 <= 0.0 { 0.0 } else { ((apx * abx + apy * aby) / ab2).clamp(0.0, 1.0) };
                let cx = ax + t * abx;
                let cy = ay + t * aby;
                (px - cx).hypot(py - cy)
            }
            Item::Via { shape, .. } => {
                let dx = (point.x - shape.center.x) as f64;
                let dy = (point.y - shape.center.y) as f64;
                dx.hypot(dy)
            }
            _ => continue,
        };
        if best.map(|b| dist < b.0).unwrap_or(true) {
            best = Some((dist, id));
        }
    }
    best.map(|(_, id)| id)
}

/// Remove the whole electrically-continuous wire nearest to `(x_mm, y_mm)`.
pub fn ripup_wire_near(doc: &mut BoardDoc, x_mm: f64, y_mm: f64) -> Result<Value, String> {
    let point = Point::new(mm(x_mm), mm(y_mm));
    let id = nearest_routed_item(doc, point).ok_or_else(|| "no track or via on the board to rip up".to_string())?;
    let wire = doc.connected_wire(id);
    if wire.is_empty() {
        return Err("nearest copper item is not a removable wire".into());
    }
    let count = wire.len();
    let net = doc.node.get(id).and_then(|item| item.net());
    let net_name = net.and_then(|nid| doc.nets.iter().find(|n| n.id == nid).map(|n| n.name.clone()));
    doc.remove_wire(id);
    Ok(json!({ "ok": true, "removed_items": count, "net": net_name }))
}

/// Remove every track and via on a named net (pads stay).
pub fn ripup_net_copper(doc: &mut BoardDoc, net_name: &str) -> Result<Value, String> {
    let net = net_by_name(doc, net_name)?.id;
    let mut ids: Vec<ItemId> = doc
        .node
        .iter_with_ids()
        .filter(|(_, item)| matches!(item, Item::Track { .. } | Item::Via { .. }) && item.net() == Some(net))
        .map(|(id, _)| id)
        .collect();
    let mut removed = 0usize;
    while let Some(id) = ids.pop() {
        if doc.node.get(id).is_none() {
            continue;
        }
        let wire = doc.connected_wire(id);
        if wire.is_empty() {
            continue;
        }
        removed += wire.len();
        doc.remove_wire(id);
        ids.retain(|i| doc.node.get(*i).is_some());
    }
    Ok(json!({ "ok": true, "net": net_name, "removed_items": removed }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board_doc::{CopperWeight, LayerCount, NewBoardParams};
    use crate::footprint;

    fn board_with_two_pads() -> (BoardDoc, Vec<FootprintTemplate>, String, String, NetId, Point, Point) {
        let mut doc = NewBoardParams {
            width_mm: 40.0,
            height_mm: 40.0,
            layer_count: LayerCount::Two,
            copper_weight: CopperWeight::OneOz,
            corner_radius_mm: 1.0,
        }
        .create();
        let templates = footprint::builtin_templates();
        let template = templates.iter().find(|t| t.pads.len() == 1 && t.holes.is_empty()).expect("single-pad smd template").clone();
        let a = doc.try_place_footprint(&template, Point::new(-10 * MM, 0), 0.0).unwrap();
        let b = doc.try_place_footprint(&template, Point::new(10 * MM, 0), 0.0).unwrap();
        let pad_a = doc.footprints.iter().find(|f| f.id == a).unwrap().pad_item_ids[0];
        let pad_b = doc.footprints.iter().find(|f| f.id == b).unwrap().pad_item_ids[0];
        let ca = doc.pad_center(pad_a).unwrap();
        let cb = doc.pad_center(pad_b).unwrap();
        let net = doc.connect_pads(pad_a, pad_b).unwrap();
        let ref_a = doc.footprints.iter().find(|f| f.id == a).unwrap().reference.clone();
        let ref_b = doc.footprints.iter().find(|f| f.id == b).unwrap().reference.clone();
        (doc, templates, ref_a, ref_b, net, ca, cb)
    }

    fn straight_candidate(net_name: &str, a: Point, b: Point) -> Value {
        json!({
            "net": net_name,
            "segments": [{
                "layer": "FCu",
                "points_mm": [[unit_mm(a.x), unit_mm(a.y)], [unit_mm(b.x), unit_mm(b.y)]]
            }]
        })
    }

    #[test]
    fn scene_lists_open_bridge_between_unconnected_pads() {
        let (doc, templates, ref_a, ref_b, _, ca, cb) = board_with_two_pads();
        let scene = routing_scene_json(&doc, &templates);
        assert!(scene["pads"].as_array().unwrap().len() >= 2);
        let bridges = scene["open_bridges"].as_array().unwrap();
        assert_eq!(bridges.len(), 1);
        let ends = [bridges[0]["a"]["ref"].as_str().unwrap(), bridges[0]["b"]["ref"].as_str().unwrap()];
        assert!(ends.contains(&ref_a.as_str()) && ends.contains(&ref_b.as_str()), "{bridges:?}");
        let expected = ((ca.x - cb.x) as f64).hypot((ca.y - cb.y) as f64) / MM as f64;
        assert!((bridges[0]["distance_mm"].as_f64().unwrap() - expected).abs() < 0.05);
    }

    #[test]
    fn probe_and_commit_straight_route_closes_the_bridge() {
        let (mut doc, templates, _, _, net, ca, cb) = board_with_two_pads();
        let net_name = doc.nets.iter().find(|n| n.id == net).unwrap().name.clone();
        let cand = straight_candidate(&net_name, ca, cb);
        let route = parse_route_candidate(&doc, &cand).unwrap();
        assert_eq!(probe_one(&doc, &route)["ok"], true);
        let committed = commit_route(&mut doc, &route).unwrap();
        assert_eq!(committed["bridge_closed"], true, "{committed}");
        assert_eq!(committed["copper_pieces_before"], 2, "{committed}");
        assert_eq!(committed["copper_pieces_after"], 1, "{committed}");
        assert_eq!(doc.node.net_copper_components(net).len(), 1);
        let scene = routing_scene_json(&doc, &templates);
        assert!(scene["open_bridges"].as_array().unwrap().is_empty());
    }

    #[test]
    fn blocked_probe_names_the_item_in_the_way() {
        let (mut doc, _, _, _, net, ca, cb) = board_with_two_pads();
        let templates = footprint::builtin_templates();
        let template = templates.iter().find(|t| t.pads.len() == 1 && t.holes.is_empty()).unwrap().clone();
        let blocker = doc.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let blocker_ref = doc.footprints.iter().find(|f| f.id == blocker).unwrap().reference.clone();
        let net_name = doc.nets.iter().find(|n| n.id == net).unwrap().name.clone();
        let route = parse_route_candidate(&doc, &straight_candidate(&net_name, ca, cb)).unwrap();
        let result = probe_one(&doc, &route);
        assert_eq!(result["ok"], false, "{result}");
        assert_eq!(result["blocked"], "clearance", "{result}");
        let hits = result["colliding"].as_array().unwrap();
        assert!(!hits.is_empty(), "{result}");
        assert!(
            hits.iter().any(|h| h["kind"] == "pad" && h["footprint"] == blocker_ref.as_str()),
            "{result}"
        );
        assert!(result["leg_mm"].is_array(), "{result}");
    }

    #[test]
    fn commit_rolls_back_a_route_that_misses_the_target_copper() {
        let (mut doc, _, _, _, net, ca, cb) = board_with_two_pads();
        let net_name = doc.nets.iter().find(|n| n.id == net).unwrap().name.clone();
        // Clearance-clean, but on the wrong layer: the SMD pads live on FCu,
        // so a BCu track underneath them joins nothing.
        let cand = json!({
            "net": net_name,
            "segments": [{
                "layer": "BCu",
                "points_mm": [[unit_mm(ca.x), unit_mm(ca.y)], [unit_mm(cb.x), unit_mm(cb.y)]]
            }]
        });
        let route = parse_route_candidate(&doc, &cand).unwrap();
        assert_eq!(probe_one(&doc, &route)["ok"], true, "BCu under SMD pads must be clearance-clean");
        let err = commit_route(&mut doc, &route).unwrap_err();
        assert!(err.contains("did not join"), "{err}");
        assert!(err.contains("rolled back"), "{err}");
        assert_eq!(
            doc.node.iter().filter(|i| matches!(i, Item::Track { .. })).count(),
            0,
            "rollback must remove the useless track again"
        );
        assert_eq!(doc.node.net_copper_components(net).len(), 2);
    }

    #[test]
    fn edge_comfort_margin_blocks_an_edge_hugging_route_unless_relaxed_per_call() {
        let (doc, _, _, _, net, ca, cb) = board_with_two_pads();
        let net_name = doc.nets.iter().find(|n| n.id == net).unwrap().name.clone();
        // 40mm board -> bottom edge at y = 20mm. A detour along
        // y = 19.5mm leaves ~0.375mm of copper-to-edge: legal for the
        // fab (0.2mm hard minimum) but under the 1.0mm comfort margin.
        let cand = json!({
            "net": net_name,
            "segments": [{
                "layer": "FCu",
                "points_mm": [
                    [unit_mm(ca.x), unit_mm(ca.y)],
                    [-10.0, 19.5],
                    [10.0, 19.5],
                    [unit_mm(cb.x), unit_mm(cb.y)]
                ]
            }]
        });
        let route = parse_route_candidate(&doc, &cand).unwrap();
        let result = probe_one(&doc, &route);
        assert_eq!(result["ok"], false, "{result}");
        assert_eq!(result["blocked"], "edge", "{result}");
        assert!(result["hint"].as_str().unwrap().contains("edge_margin_mm"), "{result}");

        // The same route with an explicit, deliberate 0.3mm margin passes.
        let mut relaxed = cand.clone();
        relaxed["edge_margin_mm"] = json!(0.3);
        let route = parse_route_candidate(&doc, &relaxed).unwrap();
        assert_eq!(probe_one(&doc, &route)["ok"], true);

        // Below the fab minimum is refused outright, at parse time.
        let mut illegal = cand;
        illegal["edge_margin_mm"] = json!(0.1);
        let err = parse_route_candidate(&doc, &illegal).unwrap_err();
        assert!(err.contains("minimum"), "{err}");
    }

    #[test]
    fn probe_rejects_path_off_the_board() {
        let (doc, _, _, _, net, ca, cb) = board_with_two_pads();
        let net_name = doc.nets.iter().find(|n| n.id == net).unwrap().name.clone();
        let cand = json!({
            "net": net_name,
            "segments": [{
                "layer": "FCu",
                "points_mm": [
                    [unit_mm(ca.x), unit_mm(ca.y)],
                    [0.0, 100.0],
                    [unit_mm(cb.x), unit_mm(cb.y)]
                ]
            }]
        });
        let route = parse_route_candidate(&doc, &cand).unwrap();
        let result = probe_one(&doc, &route);
        assert_eq!(result["ok"], false);
        let blocked = result["blocked"].as_str().unwrap();
        assert!(blocked.contains("edge") || blocked.contains("clearance"), "{blocked}");
    }

    fn assert_octilinear_no_90(points: &[Point]) {
        assert!(points.len() >= 2, "path needs at least 2 points: {points:?}");
        let mut prev_dir = NO_DIR;
        for leg in points.windows(2) {
            let dir = octant_of(leg[1].x - leg[0].x, leg[1].y - leg[0].y)
                .unwrap_or_else(|| panic!("leg {:?} -> {:?} is not octilinear", leg[0], leg[1]));
            assert!(turn_ok(prev_dir, dir), "90°/135° corner before leg {:?} -> {:?} in {points:?}", leg[0], leg[1]);
            prev_dir = dir;
        }
    }

    fn default_opts() -> SuggestOptions {
        SuggestOptions {
            layer: LayerId::FCu,
            width: DEFAULT_TRACE_WIDTH,
            edge_margin: EDGE_COMFORT_MARGIN,
            step: MM / 2,
            bend_penalty: 2 * MM / 5,
            max_expansions: 200_000,
        }
    }

    #[test]
    fn suggest_route_finds_a_straight_run_that_commits_cleanly() {
        let (mut doc, _, _, _, net, ca, cb) = board_with_two_pads();
        let found = suggest_route(&doc, net, ca, cb, &default_opts()).unwrap();
        assert_octilinear_no_90(&found.points);
        assert_eq!(*found.points.first().unwrap(), ca);
        assert_eq!(*found.points.last().unwrap(), cb);
        let net_name = doc.nets.iter().find(|n| n.id == net).unwrap().name.clone();
        let cand = json!({
            "net": net_name,
            "segments": [{
                "layer": "FCu",
                "points_mm": found.points.iter().map(|p| json!([unit_mm(p.x), unit_mm(p.y)])).collect::<Vec<_>>()
            }]
        });
        let route = parse_route_candidate(&doc, &cand).unwrap();
        let committed = commit_route(&mut doc, &route).unwrap();
        assert_eq!(committed["bridge_closed"], true, "{committed}");
    }

    #[test]
    fn suggest_route_detours_around_a_blocker_with_only_45_degree_bends() {
        let (mut doc, _, _, _, net, ca, cb) = board_with_two_pads();
        let templates = footprint::builtin_templates();
        let template = templates.iter().find(|t| t.pads.len() == 1 && t.holes.is_empty()).unwrap().clone();
        doc.try_place_footprint(&template, Point::new(0, 0), 0.0).unwrap();
        let found = suggest_route(&doc, net, ca, cb, &default_opts()).unwrap();
        assert_octilinear_no_90(&found.points);
        assert!(found.points.len() > 2, "a detour needs bends: {:?}", found.points);
        let net_name = doc.nets.iter().find(|n| n.id == net).unwrap().name.clone();
        let cand = json!({
            "net": net_name,
            "segments": [{
                "layer": "FCu",
                "points_mm": found.points.iter().map(|p| json!([unit_mm(p.x), unit_mm(p.y)])).collect::<Vec<_>>()
            }]
        });
        let route = parse_route_candidate(&doc, &cand).unwrap();
        assert_eq!(probe_one(&doc, &route)["ok"], true, "suggested path must probe clean");
        commit_route(&mut doc, &route).unwrap();
        assert_eq!(doc.node.net_copper_components(net).len(), 1);
    }

    #[test]
    fn suggest_route_reports_a_blocked_corridor_instead_of_hanging() {
        let (doc, _, _, _, net, ca, cb) = board_with_two_pads();
        // A tiny budget can't reach the far pad: must fail with a clear message.
        let mut opts = default_opts();
        opts.max_expansions = 5;
        let err = suggest_route(&doc, net, ca, cb, &opts).unwrap_err();
        assert!(err.contains("search budget"), "{err}");
    }

    #[test]
    fn ripup_removes_committed_wire() {
        let (mut doc, _, _, _, net, ca, cb) = board_with_two_pads();
        let net_name = doc.nets.iter().find(|n| n.id == net).unwrap().name.clone();
        let route = parse_route_candidate(&doc, &straight_candidate(&net_name, ca, cb)).unwrap();
        commit_route(&mut doc, &route).unwrap();
        assert_eq!(doc.node.iter().filter(|i| matches!(i, Item::Track { .. })).count(), 1);
        let mid_x = unit_mm((ca.x + cb.x) / 2);
        let mid_y = unit_mm((ca.y + cb.y) / 2);
        let result = ripup_wire_near(&mut doc, mid_x, mid_y).unwrap();
        assert_eq!(result["ok"], true);
        assert_eq!(doc.node.iter().filter(|i| matches!(i, Item::Track { .. })).count(), 0);
        assert_eq!(doc.node.net_copper_components(net).len(), 2);
    }
}
