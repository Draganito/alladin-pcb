//! JLCPCB-format BOM (Bill of Materials) CSV export -- `Comment,
//! Designator, Footprint, LCSC Part #`, ground-truthed against JLCPCB's
//! own published requirements
//! (<https://jlcpcb.com/help/article/bill-of-materials-for-pcb-assembly>),
//! not guessed: exactly these four columns, one row per *distinct*
//! component with its designators grouped together (their own sample:
//! `100nF 50V X7R,C1 C2 C5,0805,C49678`), and every designator
//! uppercased ("we will turn all the letters into Capital case... If
//! you happen to use case distinction... please do correct it" -- done
//! here rather than left for JLCPCB's own upload step to silently
//! rewrite).
//!
//! Deliberately **not** produced via `kicad-cli`/KiCad's own BOM
//! exporter: that tool reads a *schematic*'s netlist (a `.kicad_sch`
//! this editor has no equivalent of -- no schematic editor; netting
//! happens by direct pin-to-net assignment on the layout itself). Every fact a BOM
//! actually needs (reference, footprint/value, LCSC part number)
//! already lives in [`BoardDoc`]/[`FootprintTemplate`]/
//! `crate::parts_db::PartsDb` instead, so building the CSV directly from
//! those is simpler and more honest than faking a schematic just to
//! satisfy a tool that doesn't need one for this.
//!
//! **Known simplification, stated rather than hidden:** `Comment` and
//! `Footprint` both currently come from just two fields
//! ([`FootprintTemplate::name`] and `crate::parts_db::PartRecord::
//! description`) because neither `FootprintTemplate` nor the parts
//! database schema splits "package" (e.g. `0805`) from "value/spec"
//! (e.g. `10k 1%`) into two separate columns yet -- `crate::lcsc`'s own
//! downloader already receives both separately from EasyEDA's API but
//! currently folds them into one `description` string before saving.
//! Good enough for JLCPCB to still uniquely match every line via its
//! `LCSC Part #` column (the one column that actually decides assembly,
//! per their own docs) -- not yet as tidy as a real two-column split
//! would be.

use std::collections::BTreeMap;

use crate::board_doc::BoardDoc;
use crate::footprint::FootprintTemplate;
use crate::parts_db::PartsDb;

/// One line of the exported BOM -- several placed parts sharing the
/// same template collapse into one `BomRow` with several `designators`,
/// never one row per placed part (see this module's doc comment for
/// why, and JLCPCB's own sample format).
pub struct BomRow {
    pub comment: String,
    pub designators: Vec<String>,
    pub footprint: String,
    pub lcsc_part_number: Option<String>,
}

/// Groups every placed footprint on `doc` by its template name into one
/// [`BomRow`] each, resolving each group's real LCSC part number/
/// description via `template_origin`/`parts_db` when the template came
/// from the user's database (`None` for built-ins/hand-added templates,
/// exactly like every other `template_origin` consumer). A placed
/// footprint whose `template_name` no longer resolves against
/// `templates` is silently skipped -- same "a stale reference must not
/// crash the whole export" precedent as other manufacturing exporters.
/// A template with
/// [`FootprintTemplate::exclude_from_bom`] set (mounting holes, solder
/// wire pads, ...) is skipped too, on purpose -- see that field's own
/// doc comment: it's never a purchasable JLCPCB assembly line item,
/// matching how the original board this pipeline targets already
/// marks those parts "Exclude from BOM" in KiCad. Rows come back
/// sorted by footprint name for a stable, diffable CSV.
pub fn build_bom_rows(doc: &BoardDoc, templates: &[FootprintTemplate], template_origin: &[Option<i64>], parts_db: &PartsDb) -> Vec<BomRow> {
    let mut designators_by_template: BTreeMap<&str, Vec<String>> = BTreeMap::new();
    for fp in &doc.footprints {
        designators_by_template.entry(fp.template_name.as_str()).or_default().push(fp.reference.to_uppercase());
    }

    let mut rows: Vec<BomRow> = designators_by_template
        .into_iter()
        .filter_map(|(template_name, mut designators)| {
            let index = templates.iter().position(|t| t.name == template_name)?;
            if templates[index].exclude_from_bom {
                return None;
            }
            designators.sort();

            let part = template_origin.get(index).copied().flatten().and_then(|id| parts_db.get_part(id).ok());
            let comment = part.as_ref().filter(|p| !p.description.is_empty()).map(|p| p.description.clone()).unwrap_or_else(|| template_name.to_string());
            let lcsc_part_number = part.and_then(|p| p.lcsc_code);

            Some(BomRow { comment, designators, footprint: template_name.to_string(), lcsc_part_number })
        })
        .collect();
    rows.sort_by(|a, b| a.footprint.cmp(&b.footprint));
    rows
}

/// Quotes a CSV field only when it actually needs it (contains a comma,
/// quote, or newline) -- matches every spreadsheet tool's own default
/// behaviour, so a plain BOM (the common case: no commas in a footprint
/// name) stays perfectly human-readable rather than quoted everywhere.
pub(crate) fn csv_field(s: &str) -> String {
    if s.contains(',') || s.contains('"') || s.contains('\n') {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

/// Renders `rows` as a JLCPCB-ready CSV, header included.
pub fn to_csv(rows: &[BomRow]) -> String {
    let mut out = String::from("Comment,Designator,Footprint,LCSC Part #\n");
    for row in rows {
        let designators = row.designators.join(", ");
        out.push_str(&format!(
            "{},{},{},{}\n",
            csv_field(&row.comment),
            csv_field(&designators),
            csv_field(&row.footprint),
            csv_field(row.lcsc_part_number.as_deref().unwrap_or(""))
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board_doc::{CopperWeight, LayerCount, NewBoardParams};
    use alladin_geom::Point;

    fn test_board() -> BoardDoc {
        NewBoardParams { width_mm: 60.0, height_mm: 60.0, layer_count: LayerCount::Two, copper_weight: CopperWeight::OneOz, corner_radius_mm: 0.0 }.create()
    }

    #[test]
    fn groups_parts_sharing_a_template_into_one_row_with_sorted_uppercase_designators() {
        let mut board = test_board();
        let template = crate::footprint::straight_row_template(
            "2-pin test".into(),
            "P".into(),
            2,
            2.54,
            0.45,
        );
        board.try_place_footprint(&template, Point::new(-20_000_000, 0), 0.0).unwrap();
        board.try_place_footprint(&template, Point::new(20_000_000, 0), 0.0).unwrap();
        let templates = vec![template.clone()];
        let db = PartsDb::open_in_memory().unwrap();

        let rows = build_bom_rows(&board, &templates, &vec![None; templates.len()], &db);

        assert_eq!(rows.len(), 1, "two placements of the same template must collapse into one BOM row");
        assert_eq!(rows[0].designators, vec!["P1".to_string(), "P2".to_string()]);
        assert_eq!(rows[0].footprint, template.name);
        assert_eq!(rows[0].comment, template.name, "no database description available: falls back to the template name");
        assert_eq!(rows[0].lcsc_part_number, None);
    }

    #[test]
    fn resolves_the_real_lcsc_part_number_and_description_for_a_database_backed_template() {
        let db = PartsDb::open_in_memory().unwrap();
        let mut generic_template = crate::footprint::straight_row_template("tmp".to_string(), "R".to_string(), 2, 2.0, 0.5);
        let pads = vec![generic_template.pads.remove(0)];
        let record = db.insert_part("0603 10k", "R", "0603 \u{2014} 10k\u{03a9} 1%", Some("C25804"), &pads, &[], false).unwrap();

        let mut board = test_board();
        board.try_place_footprint(&record.template, Point::new(0, 0), 0.0).unwrap();
        let templates = vec![record.template];
        let origin = vec![Some(record.id)];

        let rows = build_bom_rows(&board, &templates, &origin, &db);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].lcsc_part_number, Some("C25804".to_string()));
        assert_eq!(rows[0].comment, "0603 \u{2014} 10k\u{03a9} 1%");
    }

    #[test]
    fn a_mechanical_template_marked_exclude_from_bom_produces_no_row() {
        let mut board = test_board();
        let hole_template = crate::footprint::builtin_templates().into_iter().find(|t| t.exclude_from_bom).expect("a builtin mechanical template must exist");
        board.try_place_footprint(&hole_template, Point::new(0, 0), 0.0).unwrap();
        let templates = vec![hole_template];
        let db = PartsDb::open_in_memory().unwrap();

        let rows = build_bom_rows(&board, &templates, &vec![None; templates.len()], &db);
        assert!(rows.is_empty(), "a mounting hole / wire pad must never appear as a BOM line item");
    }

    #[test]
    fn a_mix_of_electrical_and_mechanical_parts_only_boms_the_electrical_one() {
        let mut board = test_board();
        let electrical = crate::footprint::straight_row_template(
            "2-pin test".into(),
            "P".into(),
            2,
            2.54,
            0.45,
        );
        let mechanical = crate::footprint::builtin_templates().into_iter().find(|t| t.exclude_from_bom).expect("a builtin mechanical template must exist");
        board.try_place_footprint(&electrical, Point::new(-20_000_000, 0), 0.0).unwrap();
        board.try_place_footprint(&mechanical, Point::new(20_000_000, 0), 0.0).unwrap();
        let templates = vec![electrical.clone(), mechanical];
        let db = PartsDb::open_in_memory().unwrap();

        let rows = build_bom_rows(&board, &templates, &vec![None; templates.len()], &db);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].footprint, electrical.name);
    }

    #[test]
    fn a_footprint_whose_template_was_deleted_is_skipped_not_a_panic() {
        let mut board = test_board();
        let templates = crate::footprint::builtin_templates();
        board.try_place_footprint(&templates[0], Point::new(0, 0), 0.0).unwrap();
        let db = PartsDb::open_in_memory().unwrap();

        let rows = build_bom_rows(&board, &[], &[], &db);
        assert!(rows.is_empty());
    }

    #[test]
    fn to_csv_has_the_exact_header_jlcpcb_expects() {
        assert!(to_csv(&[]).starts_with("Comment,Designator,Footprint,LCSC Part #\n"));
    }

    #[test]
    fn to_csv_quotes_a_designator_list_only_because_it_contains_a_comma() {
        let rows = vec![BomRow { comment: "10k".to_string(), designators: vec!["R1".to_string(), "R2".to_string()], footprint: "0603".to_string(), lcsc_part_number: Some("C25804".to_string()) }];
        let csv = to_csv(&rows);
        assert_eq!(csv, "Comment,Designator,Footprint,LCSC Part #\n10k,\"R1, R2\",0603,C25804\n");
    }

    #[test]
    fn to_csv_leaves_the_lcsc_column_empty_for_a_part_with_no_known_part_number() {
        let rows = vec![BomRow { comment: "2-pin header".to_string(), designators: vec!["P1".to_string()], footprint: "2-pin THT".to_string(), lcsc_part_number: None }];
        let csv = to_csv(&rows);
        assert!(csv.ends_with("2-pin header,P1,2-pin THT,\n"));
    }
}
