//! MCP helpers for AI floorplanning: placement probe, atomic batch
//! place/move, and open-bridge scores after a layout change.

use alladin_geom::{Point, Unit, MM};
use serde_json::{json, Value};
use std::collections::BTreeMap;

use crate::board_doc::{BatchMoveSpec, BatchPlaceSpec, BoardDoc, FootprintId};
use crate::footprint::FootprintTemplate;
use crate::mcp::{MovePartSpec, PlacePartSpec, ProbePlacementArgs};
use crate::mcp_routing::open_bridges_score_json;

const MAX_BATCH_PARTS: usize = 50;
const MAX_SEARCH_RADIUS_MM: f64 = 10.0;
const MIN_SEARCH_STEP_MM: f64 = 0.25;
const DEFAULT_SEARCH_STEP_MM: f64 = 0.5;

fn mm(v: f64) -> Unit {
    (v * MM as f64).round() as Unit
}

fn unit_mm(u: Unit) -> f64 {
    u as f64 / MM as f64
}

/// Dry-run placement/move probe (+ optional nearest-legal search).
pub(crate) fn probe_placement_json(
    doc: &BoardDoc,
    templates: &[FootprintTemplate],
    args: &ProbePlacementArgs,
) -> Value {
    let rotation = args.rotation_deg.unwrap_or(0.0);
    let requested = Point::new(mm(args.x_mm), mm(args.y_mm));

    let (template, moving, keep_rotation): (FootprintTemplate, Option<FootprintId>, f64) =
        if let Some(reference) = args.reference.as_deref() {
            let Some(fp) = doc.footprints.iter().find(|f| f.reference == reference) else {
                return json!({ "error": format!("no footprint with reference \"{reference}\" on the board") });
            };
            let Some(template) = templates.iter().find(|t| t.name == fp.template_name).cloned() else {
                return json!({
                    "error": format!(
                        "{reference}'s template \"{}\" is missing from the parts library",
                        fp.template_name
                    )
                });
            };
            let rot = args.rotation_deg.unwrap_or(fp.rotation_deg);
            (template, Some(fp.id), rot)
        } else if let Some(name) = args.template.as_deref() {
            let Some(template) = templates.iter().find(|t| t.name == name).cloned() else {
                return json!({
                    "error": format!(
                        "no template named \"{name}\" in the parts library -- call list_parts for the exact names"
                    )
                });
            };
            (template, None, rotation)
        } else {
            return json!({ "error": "pass template=\"...\" to probe a new place, or reference=\"R1\" to probe a move" });
        };

    let search_radius = args.search_radius_mm.map(|r| mm(r.clamp(0.0, MAX_SEARCH_RADIUS_MM)));
    let search_step = args.search_step_mm.map(|s| mm(s.max(MIN_SEARCH_STEP_MM))).or_else(|| {
        search_radius.map(|_| mm(DEFAULT_SEARCH_STEP_MM))
    });

    let requested_ok = doc
        .find_nearest_legal_placement(&template, requested, keep_rotation, moving, None, None)
        .is_ok();

    match doc.find_nearest_legal_placement(&template, requested, keep_rotation, moving, search_radius, search_step) {
        Ok((pos, rot)) => {
            let mut out = json!({
                "ok": true,
                "legal": true,
                "x_mm": unit_mm(pos.x),
                "y_mm": unit_mm(pos.y),
                "rotation_deg": rot,
                "requested": {
                    "x_mm": args.x_mm,
                    "y_mm": args.y_mm,
                    "rotation_deg": keep_rotation,
                    "legal": requested_ok,
                },
            });
            if !requested_ok {
                out["suggested"] = json!({
                    "x_mm": unit_mm(pos.x),
                    "y_mm": unit_mm(pos.y),
                    "rotation_deg": rot,
                });
                out["requested"]["reason"] = json!("requested pose is illegal; suggested is the nearest legal within search_radius_mm");
            }
            out
        }
        Err(e) => json!({
            "ok": true,
            "legal": false,
            "x_mm": args.x_mm,
            "y_mm": args.y_mm,
            "rotation_deg": keep_rotation,
            "reason": e.to_string(),
            "requested": {
                "x_mm": args.x_mm,
                "y_mm": args.y_mm,
                "rotation_deg": keep_rotation,
                "legal": false,
                "reason": e.to_string(),
            },
        }),
    }
}

/// Atomic multi-place with optional pin→net maps; one caller undo frame.
pub(crate) fn place_parts_on_doc(
    doc: &mut BoardDoc,
    templates: &[FootprintTemplate],
    parts: &[PlacePartSpec],
) -> Result<Value, String> {
    if parts.is_empty() {
        return Err("parts must contain at least one entry".into());
    }
    if parts.len() > MAX_BATCH_PARTS {
        return Err(format!("parts is capped at {MAX_BATCH_PARTS} entries per call"));
    }

    let mut pin_storage: Vec<Vec<(String, String)>> = Vec::with_capacity(parts.len());
    let mut resolved: Vec<(&FootprintTemplate, Point, f64)> = Vec::with_capacity(parts.len());

    for part in parts {
        let template = templates
            .iter()
            .find(|t| t.name == part.template)
            .ok_or_else(|| {
                format!(
                    "no template named \"{}\" in the parts library -- call list_parts for the exact names",
                    part.template
                )
            })?;
        let pins: Vec<(String, String)> = part
            .pins
            .as_ref()
            .map(|m| m.iter().map(|(k, v)| (k.clone(), v.clone())).collect())
            .unwrap_or_default();
        pin_storage.push(pins);
        resolved.push((
            template,
            Point::new(mm(part.x_mm), mm(part.y_mm)),
            part.rotation_deg.unwrap_or(0.0),
        ));
    }

    let specs: Vec<BatchPlaceSpec<'_>> = resolved
        .iter()
        .zip(pin_storage.iter())
        .map(|((template, position, rotation), pins)| BatchPlaceSpec {
            template,
            position: *position,
            rotation_deg: *rotation,
            pins: pins.as_slice(),
        })
        .collect();

    let ids = doc.place_batch(&specs).map_err(|e| e.to_string())?;

    let mut placed = Vec::with_capacity(ids.len());
    for (part, &id) in parts.iter().zip(ids.iter()) {
        let fp = doc
            .footprints
            .iter()
            .find(|f| f.id == id)
            .ok_or_else(|| "internal: placed footprint missing".to_string())?;
        let mut entry = json!({
            "reference": fp.reference,
            "template": part.template,
            "x_mm": unit_mm(fp.position.x),
            "y_mm": unit_mm(fp.position.y),
            "rotation_deg": fp.rotation_deg,
        });
        if let Some(pins) = &part.pins {
            let mut nets = BTreeMap::new();
            for (pin, net) in pins {
                nets.insert(pin.clone(), net.clone());
            }
            entry["nets"] = json!(nets);
        }
        placed.push(entry);
    }

    Ok(json!({
        "ok": true,
        "placed": placed,
        "zones_stale": doc.zones_are_stale(),
        "open_bridges": open_bridges_score_json(doc, templates, 20),
    }))
}

/// Atomic multi-move; one caller undo frame.
pub(crate) fn move_parts_on_doc(
    doc: &mut BoardDoc,
    templates: &[FootprintTemplate],
    parts: &[MovePartSpec],
) -> Result<Value, String> {
    if parts.is_empty() {
        return Err("parts must contain at least one entry".into());
    }
    if parts.len() > MAX_BATCH_PARTS {
        return Err(format!("parts is capped at {MAX_BATCH_PARTS} entries per call"));
    }

    let mut owned_templates: Vec<FootprintTemplate> = Vec::new();
    let mut resolved: Vec<(FootprintId, usize, Point, f64, String)> = Vec::with_capacity(parts.len());

    for part in parts {
        let fp = doc
            .footprints
            .iter()
            .find(|f| f.reference == part.reference)
            .ok_or_else(|| format!("no footprint with reference \"{}\" on the board", part.reference))?;
        let template = templates
            .iter()
            .find(|t| t.name == fp.template_name)
            .ok_or_else(|| {
                format!(
                    "{}'s template \"{}\" is missing from the parts library",
                    part.reference, fp.template_name
                )
            })?;
        let idx = owned_templates.len();
        owned_templates.push(template.clone());
        let rotation = part.rotation_deg.unwrap_or(fp.rotation_deg);
        resolved.push((
            fp.id,
            idx,
            Point::new(mm(part.x_mm), mm(part.y_mm)),
            rotation,
            part.reference.clone(),
        ));
    }

    let specs: Vec<BatchMoveSpec<'_>> = resolved
        .iter()
        .map(|(id, idx, position, rotation, _)| BatchMoveSpec {
            id: *id,
            template: &owned_templates[*idx],
            position: *position,
            rotation_deg: *rotation,
        })
        .collect();

    doc.move_batch(&specs).map_err(|e| e.to_string())?;

    let moved: Vec<Value> = resolved
        .iter()
        .map(|(_, _, position, rotation, reference)| {
            json!({
                "reference": reference,
                "x_mm": unit_mm(position.x),
                "y_mm": unit_mm(position.y),
                "rotation_deg": rotation,
            })
        })
        .collect();

    Ok(json!({
        "ok": true,
        "moved": moved,
        "zones_stale": doc.zones_are_stale(),
        "open_bridges": open_bridges_score_json(doc, templates, 20),
    }))
}
