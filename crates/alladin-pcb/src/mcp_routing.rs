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
use crate::routing::{path_keeps_edge_clearance, DEFAULT_TRACE_WIDTH};

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

fn parse_layer(s: &str) -> Result<LayerId, String> {
    match s.trim() {
        "FCu" | "F.Cu" | "fcu" | "F" | "top" => Ok(LayerId::FCu),
        "BCu" | "B.Cu" | "bcu" | "B" | "bottom" => Ok(LayerId::BCu),
        other => Err(format!("unknown layer \"{other}\" -- use FCu or BCu")),
    }
}

fn pad_label(doc: &BoardDoc, templates: &[FootprintTemplate], pad_id: ItemId) -> Option<(String, String)> {
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
            "default_trace_width_mm": unit_mm(DEFAULT_TRACE_WIDTH),
            "default_via_diameter_mm": unit_mm(DEFAULT_VIA_DIAMETER),
            "default_via_drill_mm": unit_mm(DEFAULT_VIA_DRILL),
        },
    })
}

/// Closest pad-to-pad links between copper islands that still need joining.
fn open_bridges_json(doc: &BoardDoc, templates: &[FootprintTemplate]) -> Vec<Value> {
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
fn path_block_json(doc: &BoardDoc, path: &[Point], width: Unit, net: NetId, layer: LayerId) -> Option<Value> {
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
        if !path_keeps_edge_clearance(leg, width, &doc.outline) {
            return Some(json!({
                "blocked": "edge",
                "leg_index": i,
                "leg_mm": leg_mm(leg[0], leg[1]),
            }));
        }
    }
    None
}

/// Read-only via gates matching [`BoardDoc::try_add_via`] (no mutation).
/// On blockage names the reason and, for collisions, the items in the way.
fn via_block_json(doc: &BoardDoc, center: Point, net: NetId, diameter: Unit, drill: Unit) -> Option<Value> {
    if let Err(v) = JlcpcbDfm::check_via(diameter, drill) {
        return Some(json!({ "blocked": format!("via: {v}") }));
    }
    let radius = diameter / 2;
    if !alladin_geom::circle_within_outline(center, radius + JlcpcbDfm::COPPER_TO_ROUTED_EDGE, &doc.outline) {
        return Some(json!({ "blocked": "via: too close to board edge" }));
    }
    if doc.violates_hole_to_hole(center, drill, Some(net)) {
        return Some(json!({ "blocked": "via: hole-to-hole spacing" }));
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
        if let Some(mut detail) = path_block_json(doc, path, route.width, route.net, *layer) {
            let obj = detail.as_object_mut().unwrap();
            obj.insert("ok".into(), json!(false));
            obj.insert("segment_index".into(), json!(i));
            obj.insert("layer".into(), json!(layer_name(*layer)));
            obj.insert("net".into(), json!(route.net_name));
            return detail;
        }
    }
    for (i, via) in route.vias.iter().enumerate() {
        if let Some(mut detail) = via_block_json(doc, *via, route.net, route.via_diameter, route.via_drill) {
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

    for (i, (layer, path)) in route.segments.iter().enumerate() {
        doc.add_track_path(path, route.net, *layer, route.width, NetClass::C);
        if i < route.vias.len() {
            if let Err(e) = doc.try_add_via(route.vias[i], route.net, route.via_diameter, route.via_drill) {
                rollback(doc, &ids_before);
                let e: PlacementError = e;
                return Err(format!("via after segment {i}: {e} — rolled back"));
            }
        }
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
