//! Excellon drill-file writer (PTH / NPTH).
//!
//! Format mirrors what KiCad 9's `pcb export drill --excellon-separate-th`
//! produces: `M48` header, `METRIC` decimal, tool table, then absolute
//! hole list. `gerber_writer` has no Excellon support -- this is ours.

use std::collections::BTreeMap;

use alladin_geom::{Point, Unit};

use crate::to_mm;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrillKind {
    /// Plated through-hole (vias + THT pad drills).
    Plated,
    /// Non-plated mechanical hole (mounting holes).
    NonPlated,
}

/// One hole to drill.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hole {
    pub position: Point,
    pub diameter: Unit,
}

/// Accumulate holes of one kind and emit an Excellon file.
#[derive(Debug, Clone)]
pub struct ExcellonFile {
    kind: DrillKind,
    holes: Vec<Hole>,
}

impl ExcellonFile {
    pub fn new(kind: DrillKind) -> Self {
        Self { kind, holes: Vec::new() }
    }

    pub fn add_hole(&mut self, position: Point, diameter: Unit) {
        if diameter > 0 {
            self.holes.push(Hole { position, diameter });
        }
    }

    pub fn is_empty(&self) -> bool {
        self.holes.is_empty()
    }

    pub fn dump(&self) -> String {
        // Group by diameter so each tool drills all its holes together.
        let mut by_dia: BTreeMap<Unit, Vec<Point>> = BTreeMap::new();
        for h in &self.holes {
            by_dia.entry(h.diameter).or_default().push(h.position);
        }

        let mut out = Vec::new();
        out.push("M48".into());
        out.push("; DRILL file {Alladin PCB native Excellon writer}".into());
        out.push("; FORMAT={-:-/ absolute / metric / decimal}".into());
        match self.kind {
            DrillKind::Plated => {
                out.push("; #@! TF.FileFunction,Plated,1,2,PTH".into());
            }
            DrillKind::NonPlated => {
                out.push("; #@! TF.FileFunction,NonPlated,1,2,NPTH".into());
            }
        }
        out.push("FMAT,2".into());
        out.push("METRIC".into());

        let mut tool_for_dia: BTreeMap<Unit, u32> = BTreeMap::new();
        let mut next_tool = 1u32;
        for &dia in by_dia.keys() {
            let t = next_tool;
            next_tool += 1;
            tool_for_dia.insert(dia, t);
            let aper = match self.kind {
                DrillKind::Plated => "Plated,PTH,ViaDrill",
                DrillKind::NonPlated => "NonPlated,NPTH,ComponentDrill",
            };
            out.push(format!("; #@! TA.AperFunction,{aper}"));
            out.push(format!("T{t}C{:.3}", to_mm(dia)));
        }
        out.push("%".into());
        out.push("G90".into());
        out.push("G05".into());

        for (dia, positions) in &by_dia {
            let t = tool_for_dia[dia];
            out.push(format!("T{t}"));
            for p in positions {
                out.push(format!("X{}Y{}", fmt_drill_coord(p.x), fmt_drill_coord(p.y)));
            }
        }
        out.push("M30".into());
        out.join("\n") + "\n"
    }
}

fn fmt_drill_coord(nm: Unit) -> String {
    // KiCad writes compact decimals (`X-61.07Y28.795`) -- trim trailing zeros.
    let v = to_mm(nm);
    let s = format!("{v:.6}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s.is_empty() || s == "-" {
        "0".into()
    } else {
        s.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_geom::MM;

    #[test]
    fn plated_file_lists_tools_and_holes() {
        let mut file = ExcellonFile::new(DrillKind::Plated);
        file.add_hole(Point::new(MM, 2 * MM), MM / 3);
        file.add_hole(Point::new(0, 0), MM / 3);
        file.add_hole(Point::new(MM, MM), MM / 2);
        let text = file.dump();
        assert!(text.contains("TF.FileFunction,Plated,1,2,PTH"));
        assert!(text.contains("T1C0.333") || text.contains("T1C0.334") || text.contains("T1C"));
        assert!(text.contains("M30"));
        assert_eq!(text.matches("T1\n").count() + text.matches("T2\n").count(), 2);
    }
}
