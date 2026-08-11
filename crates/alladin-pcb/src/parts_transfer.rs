//! Portable parts snapshots: library JSON export/import and board-embedded parts.
//!
//! Desktop exports via "Export parts library…"; boards embed only the templates
//! they use so a colleague can open the file (incl. WASM) without a separate
//! library. Opening merges the difference into the local PartsDb (LCSC / name
//! dedupe — no duplicates).

use std::collections::{BTreeMap, BTreeSet};

use alladin_core::{LayerId, ZoneConnection};
use alladin_geom::{Point, Unit};
use serde::{Deserialize, Serialize};

use crate::board_doc::BoardDoc;
use crate::footprint::{builtin_templates, Courtyard, FootprintTemplate, HoleTemplate, PadShapeKind, PadTemplate};
use crate::parts_db::{PartRecord, PartsDb, PartsDbError};

pub const FORMAT: &str = "alladin-parts";
pub const FORMAT_VERSION: u32 = 1;

#[derive(Debug, Serialize, Deserialize)]
struct LibraryFile {
    format: String,
    format_version: u32,
    parts: Vec<PartSnapshot>,
}

/// One part's portable footprint + metadata (library file or board embed).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PartSnapshot {
    pub lcsc_code: Option<String>,
    pub description: String,
    pub category: Option<String>,
    pub name: String,
    pub reference_prefix: String,
    pub exclude_from_bom: bool,
    pub pads: Vec<PadDto>,
    #[serde(default)]
    pub holes: Vec<HoleDto>,
    pub courtyard: Option<CourtyardDto>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PadDto {
    pub offset_x: Unit,
    pub offset_y: Unit,
    pub radius: Unit,
    pub layer: String,
    pub number: String,
    pub shape_kind: String,
    pub shape_width: Unit,
    pub shape_height: Unit,
    pub pad_rotation_deg: f64,
    pub hole_diameter: Unit,
    pub pin_name: Option<String>,
    /// Pour join style. Missing in older library JSON → Thermal.
    #[serde(default)]
    pub zone_connection: ZoneConnection,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HoleDto {
    pub offset_x: Unit,
    pub offset_y: Unit,
    pub drill: Unit,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CourtyardDto {
    pub center_x: Unit,
    pub center_y: Unit,
    pub width: Unit,
    pub height: Unit,
}

fn layer_to_text(layer: LayerId) -> &'static str {
    match layer {
        LayerId::FCu => "FCu",
        LayerId::BCu => "BCu",
    }
}

fn layer_from_text(text: &str) -> LayerId {
    match text {
        "BCu" => LayerId::BCu,
        _ => LayerId::FCu,
    }
}

fn shape_to_row(shape: PadShapeKind) -> (&'static str, Unit, Unit) {
    match shape {
        PadShapeKind::Circle => ("circle", 0, 0),
        PadShapeKind::Rect { width, height } => ("rect", width, height),
        PadShapeKind::Oval { width, height } => ("oval", width, height),
    }
}

fn shape_from_row(kind: &str, width: Unit, height: Unit) -> PadShapeKind {
    match kind {
        "rect" => PadShapeKind::Rect { width, height },
        "oval" => PadShapeKind::Oval { width, height },
        _ => PadShapeKind::Circle,
    }
}

pub fn snapshot_from_record(part: &PartRecord) -> PartSnapshot {
    snapshot_from_template(
        &part.template,
        part.lcsc_code.clone(),
        part.description.clone(),
        part.category.clone(),
    )
}

pub fn snapshot_from_template(
    template: &FootprintTemplate,
    lcsc_code: Option<String>,
    description: String,
    category: Option<String>,
) -> PartSnapshot {
    PartSnapshot {
        lcsc_code,
        description,
        category,
        name: template.name.clone(),
        reference_prefix: template.reference_prefix.clone(),
        exclude_from_bom: template.exclude_from_bom,
        pads: template
            .pads
            .iter()
            .map(|p| {
                let (kind, w, h) = shape_to_row(p.shape);
                PadDto {
                    offset_x: p.offset.x,
                    offset_y: p.offset.y,
                    radius: p.radius,
                    layer: layer_to_text(p.layer).to_string(),
                    number: p.number.clone(),
                    shape_kind: kind.to_string(),
                    shape_width: w,
                    shape_height: h,
                    pad_rotation_deg: p.rotation_deg,
                    hole_diameter: p.hole_diameter.unwrap_or(0),
                    pin_name: p.pin_name.clone(),
                    zone_connection: p.zone_connection,
                }
            })
            .collect(),
        holes: template
            .holes
            .iter()
            .map(|h| HoleDto { offset_x: h.offset.x, offset_y: h.offset.y, drill: h.drill })
            .collect(),
        courtyard: template.explicit_courtyard.map(|c| CourtyardDto {
            center_x: c.center.x,
            center_y: c.center.y,
            width: c.width,
            height: c.height,
        }),
    }
}

pub fn template_from_snapshot(dto: &PartSnapshot) -> FootprintTemplate {
    FootprintTemplate {
        name: dto.name.clone(),
        reference_prefix: dto.reference_prefix.clone(),
        pads: dto
            .pads
            .iter()
            .map(|p| PadTemplate {
                offset: Point::new(p.offset_x, p.offset_y),
                radius: p.radius,
                layer: layer_from_text(&p.layer),
                number: p.number.clone(),
                shape: shape_from_row(&p.shape_kind, p.shape_width, p.shape_height),
                rotation_deg: p.pad_rotation_deg,
                hole_diameter: (p.hole_diameter > 0).then_some(p.hole_diameter),
                pin_name: p.pin_name.clone(),
                zone_connection: p.zone_connection,
            })
            .collect(),
        holes: dto
            .holes
            .iter()
            .map(|h| HoleTemplate { offset: Point::new(h.offset_x, h.offset_y), drill: h.drill })
            .collect(),
        exclude_from_bom: dto.exclude_from_bom,
        explicit_courtyard: dto.courtyard.map(|c| Courtyard {
            center: Point::new(c.center_x, c.center_y),
            width: c.width,
            height: c.height,
        }),
    }
}

/// Built-in template names never need embedding (every Alladin has them).
fn builtin_names() -> BTreeSet<String> {
    builtin_templates().into_iter().map(|t| t.name).collect()
}

/// Snapshots for every non-builtin template used on `doc`, with PartsDb metadata when known.
pub fn snapshots_used_on_board(
    doc: &BoardDoc,
    templates: &[FootprintTemplate],
    template_origin: &[Option<i64>],
    parts_db: &PartsDb,
) -> Result<Vec<PartSnapshot>, PartsDbError> {
    let builtins = builtin_names();
    let mut needed: BTreeSet<String> = BTreeSet::new();
    for fp in &doc.footprints {
        if !builtins.contains(&fp.template_name) {
            needed.insert(fp.template_name.clone());
        }
    }
    let by_name: BTreeMap<&str, &FootprintTemplate> = templates.iter().map(|t| (t.name.as_str(), t)).collect();
    let db_by_name: BTreeMap<String, PartRecord> =
        parts_db.list_parts()?.into_iter().map(|p| (p.template.name.clone(), p)).collect();
    let mut out = Vec::new();
    for name in needed {
        if let Some(origin_id) = templates
            .iter()
            .position(|t| t.name == name)
            .and_then(|i| template_origin.get(i).copied().flatten())
        {
            if let Ok(record) = parts_db.get_part(origin_id) {
                out.push(snapshot_from_record(&record));
                continue;
            }
        }
        if let Some(record) = db_by_name.get(&name) {
            out.push(snapshot_from_record(record));
            continue;
        }
        if let Some(template) = by_name.get(name.as_str()) {
            out.push(snapshot_from_template(template, None, String::new(), None));
        }
    }
    out.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(out)
}

/// Insert snapshots that are not already in the DB (by LCSC code, else by template name).
/// Returns `(imported, skipped)`.
pub fn merge_snapshots_into_db(parts_db: &PartsDb, parts: &[PartSnapshot]) -> Result<(usize, usize), PartsDbError> {
    let existing_names: BTreeSet<String> = parts_db.list_parts()?.into_iter().map(|p| p.template.name).collect();
    let mut imported = 0usize;
    let mut skipped = 0usize;
    for dto in parts {
        if let Some(code) = dto.lcsc_code.as_deref() {
            if parts_db.find_by_lcsc_code(code)?.is_some() {
                skipped += 1;
                continue;
            }
        } else if existing_names.contains(&dto.name) {
            skipped += 1;
            continue;
        }
        let pads = dto
            .pads
            .iter()
            .map(|p| PadTemplate {
                offset: Point::new(p.offset_x, p.offset_y),
                radius: p.radius,
                layer: layer_from_text(&p.layer),
                number: p.number.clone(),
                shape: shape_from_row(&p.shape_kind, p.shape_width, p.shape_height),
                rotation_deg: p.pad_rotation_deg,
                hole_diameter: (p.hole_diameter > 0).then_some(p.hole_diameter),
                pin_name: p.pin_name.clone(),
                zone_connection: p.zone_connection,
            })
            .collect::<Vec<_>>();
        let holes = dto
            .holes
            .iter()
            .map(|h| HoleTemplate { offset: Point::new(h.offset_x, h.offset_y), drill: h.drill })
            .collect::<Vec<_>>();
        let courtyard = dto.courtyard.map(|c| Courtyard {
            center: Point::new(c.center_x, c.center_y),
            width: c.width,
            height: c.height,
        });
        match parts_db.insert_part_categorized(
            &dto.name,
            &dto.reference_prefix,
            &dto.description,
            dto.lcsc_code.as_deref(),
            &pads,
            &holes,
            dto.exclude_from_bom,
            courtyard,
            dto.category.as_deref(),
        ) {
            Ok(_) => imported += 1,
            Err(PartsDbError::DuplicateLcscCode(_)) => skipped += 1,
            Err(e) => return Err(e),
        }
    }
    Ok((imported, skipped))
}

/// Serialize the full personal parts library to the portable JSON format.
pub fn export_library_json(parts_db: &PartsDb) -> Result<String, PartsDbError> {
    let parts = parts_db.list_parts()?;
    let file = LibraryFile {
        format: FORMAT.to_string(),
        format_version: FORMAT_VERSION,
        parts: parts.iter().map(snapshot_from_record).collect(),
    };
    Ok(serde_json::to_string_pretty(&file).map_err(|e| PartsDbError::Message(e.to_string()))?)
}

/// Import parts from a portable JSON export. Existing LCSC codes / names are skipped.
pub fn import_library_json(parts_db: &PartsDb, json: &str) -> Result<(usize, usize), PartsDbError> {
    let file: LibraryFile = serde_json::from_str(json).map_err(|e| PartsDbError::Message(e.to_string()))?;
    if file.format != FORMAT {
        return Err(PartsDbError::Message(format!(
            "not an Alladin parts library (format {:?})",
            file.format
        )));
    }
    if file.format_version > FORMAT_VERSION {
        return Err(PartsDbError::Message(format!(
            "parts library format_version {} is newer than this build supports ({})",
            file.format_version, FORMAT_VERSION
        )));
    }
    merge_snapshots_into_db(parts_db, &file.parts)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_core::LayerId;
    use alladin_geom::{Point, MM};
    use crate::footprint::{PadShapeKind, PadTemplate};

    fn sample_pad() -> PadTemplate {
        PadTemplate {
            offset: Point::new(0, 0),
            radius: MM / 2,
            layer: LayerId::FCu,
            number: "1".into(),
            shape: PadShapeKind::Circle,
            rotation_deg: 0.0,
            hole_diameter: None,
            pin_name: Some("VCC".into()),
            zone_connection: ZoneConnection::Thermal,
        }
    }

    #[test]
    fn pad_dto_missing_zone_connection_defaults_to_thermal() {
        let json = r#"{
            "offset_x": 0, "offset_y": 0, "radius": 500000, "layer": "FCu",
            "number": "1", "shape_kind": "circle", "shape_width": 0, "shape_height": 0,
            "pad_rotation_deg": 0.0, "hole_diameter": 0, "pin_name": null
        }"#;
        let dto: PadDto = serde_json::from_str(json).unwrap();
        assert_eq!(dto.zone_connection, ZoneConnection::Thermal);
    }

    #[test]
    fn parts_library_json_round_trips() {
        let db = PartsDb::open_in_memory().unwrap();
        db.insert_part_categorized(
            "R_0402",
            "R",
            "10k",
            Some("C123"),
            &[sample_pad()],
            &[],
            false,
            None,
            Some("Resistors"),
        )
        .unwrap();
        let json = export_library_json(&db).unwrap();
        assert!(json.contains("alladin-parts"));
        assert!(json.contains("C123"));

        let db2 = PartsDb::open_in_memory().unwrap();
        let (n, skip) = import_library_json(&db2, &json).unwrap();
        assert_eq!((n, skip), (1, 0));
        let part = db2.find_by_lcsc_code("C123").unwrap().unwrap();
        assert_eq!(part.template.name, "R_0402");
        assert_eq!(part.category.as_deref(), Some("Resistors"));
        assert_eq!(part.template.pads[0].pin_name.as_deref(), Some("VCC"));
        assert_eq!(part.template.pads[0].zone_connection, ZoneConnection::Thermal);
        assert!(json.contains("\"zone_connection\": \"thermal\"") || json.contains("\"zone_connection\":\"thermal\""));

        let (n2, skip2) = import_library_json(&db2, &json).unwrap();
        assert_eq!((n2, skip2), (0, 1));
    }

    #[test]
    fn merge_skips_existing_lcsc() {
        let db = PartsDb::open_in_memory().unwrap();
        db.insert_part_categorized("R_0402", "R", "10k", Some("C123"), &[sample_pad()], &[], false, None, None)
            .unwrap();
        let snap = snapshot_from_template(
            &FootprintTemplate {
                name: "R_0402_other".into(),
                reference_prefix: "R".into(),
                pads: vec![sample_pad()],
                holes: vec![],
                exclude_from_bom: false,
                explicit_courtyard: None,
            },
            Some("C123".into()),
            "dup".into(),
            None,
        );
        let (n, skip) = merge_snapshots_into_db(&db, &[snap]).unwrap();
        assert_eq!((n, skip), (0, 1));
    }
}
