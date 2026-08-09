//! [`GerberLayer`] -- accumulate graphics, emit RS-274X with X2 attributes.
//!
//! Oriented on Karel Tavernier's `gerber_writer.DataLayer` (Apache 2.0):
//! pads / traces / regions, automatic aperture table, generation-software
//! attribute. Differs in using Alladin nanometre units and modern `%TF`/
//! `%TA` attribute syntax (same as KiCad 9) rather than `G04 #@!` comments.

use std::collections::HashMap;
use std::sync::Mutex;

use alladin_geom::{Point, Unit};

use crate::padmasters::PadMaster;
use crate::path::{Path, PathOp};
use crate::{gerber_coord, mm_str};

static GENERATION_SOFTWARE: Mutex<Option<(String, String, String)>> = Mutex::new(None);

/// Identify the software that generated subsequent Gerber files
/// (`TF.GenerationSoftware`). Call once at process start.
pub fn set_generation_software(vendor: impl Into<String>, application: impl Into<String>, version: impl Into<String>) {
    *GENERATION_SOFTWARE.lock().unwrap() = Some((vendor.into(), application.into(), version.into()));
}

#[derive(Debug, Clone)]
struct PadFlash {
    master: PadMaster,
    position: Point,
    angle_deg: f64,
}

#[derive(Debug, Clone)]
struct TracePath {
    path: Path,
    width: Unit,
    function: String,
    negative: bool,
}

#[derive(Debug, Clone)]
struct Region {
    path: Path,
    function: String,
    negative: bool,
}

#[derive(Debug, Clone)]
enum Graphics {
    Pad(PadFlash),
    Trace(TracePath),
    Region(Region),
}

struct EmitState {
    apertures: HashMap<String, u32>,
    next_dcode: u32,
    next_poly_macro: u32,
    macros: Vec<String>,
    ad_commands: Vec<String>,
    body: Vec<String>,
    current_dcode: Option<u32>,
    current_lp_neg: Option<bool>,
    current_point: Option<Point>,
    in_g01: bool,
}

impl EmitState {
    fn new() -> Self {
        Self {
            apertures: HashMap::new(),
            next_dcode: 10,
            next_poly_macro: 1,
            macros: Vec::new(),
            ad_commands: Vec::new(),
            body: Vec::new(),
            current_dcode: None,
            current_lp_neg: None,
            current_point: None,
            in_g01: false,
        }
    }

    fn ensure_g01(&mut self) {
        if !self.in_g01 {
            self.body.push("G01*".into());
            self.in_g01 = true;
        }
    }

    fn set_lp(&mut self, negative: bool) {
        if self.current_lp_neg != Some(negative) {
            self.body.push(if negative { "%LPC*%".into() } else { "%LPD*%".into() });
            self.current_lp_neg = Some(negative);
        }
    }

    fn select_dcode(&mut self, key: String, function: &str, ad_body: String) {
        let dcode = if let Some(&d) = self.apertures.get(&key) {
            d
        } else {
            let d = self.next_dcode;
            self.next_dcode += 1;
            if !function.is_empty() {
                self.ad_commands.push(format!("%TA.AperFunction,{function}*%"));
            }
            self.ad_commands.push(format!("%ADD{d}{ad_body}*%"));
            if !function.is_empty() {
                self.ad_commands.push("%TD*%".into());
            }
            self.apertures.insert(key, d);
            d
        };
        if self.current_dcode != Some(dcode) {
            self.body.push(format!("D{dcode}*"));
            self.current_dcode = Some(dcode);
        }
    }

    fn flash_at(&mut self, position: Point) {
        self.body.push(format!("X{}Y{}D03*", gerber_coord(position.x), gerber_coord(position.y)));
        self.current_point = Some(position);
    }

    fn emit_path(&mut self, path: &Path, always_d02: bool) {
        for op in &path.ops {
            match op {
                PathOp::MoveTo(p) => {
                    if always_d02 || self.current_point != Some(*p) {
                        self.body.push(format!("X{}Y{}D02*", gerber_coord(p.x), gerber_coord(p.y)));
                        self.current_point = Some(*p);
                    }
                }
                PathOp::LineTo(p) => {
                    self.ensure_g01();
                    self.body.push(format!("X{}Y{}D01*", gerber_coord(p.x), gerber_coord(p.y)));
                    self.current_point = Some(*p);
                }
            }
        }
    }

    fn add_polygon_macro(&mut self, points: &[Point]) -> String {
        let name = format!("UserPoly_{}", self.next_poly_macro);
        self.next_poly_macro += 1;
        self.macros.push(polygon_macro(&name, points));
        name
    }
}

/// One PCB image layer (copper, mask, silk, profile, …).
#[derive(Debug, Clone)]
pub struct GerberLayer {
    function: String,
    negative: bool,
    stream: Vec<Graphics>,
}

impl GerberLayer {
    /// `function` is a Gerber `.FileFunction` value, e.g.
    /// `"Copper,L1,Top,Signal"` or `"Profile,NP"`.
    pub fn new(function: impl Into<String>, negative: bool) -> Self {
        Self { function: function.into(), negative, stream: Vec::new() }
    }

    pub fn len(&self) -> usize {
        self.stream.len()
    }

    pub fn is_empty(&self) -> bool {
        self.stream.is_empty()
    }

    pub fn add_pad(&mut self, master: PadMaster, position: Point, angle_deg: f64) {
        self.stream.push(Graphics::Pad(PadFlash { master, position, angle_deg }));
    }

    pub fn add_trace_line(&mut self, start: Point, end: Point, width: Unit, function: impl Into<String>) {
        let mut path = Path::new();
        path.moveto(start);
        path.lineto(end);
        self.add_traces_path(path, width, function, false);
    }

    pub fn add_traces_path(&mut self, path: Path, width: Unit, function: impl Into<String>, negative: bool) {
        self.stream.push(Graphics::Trace(TracePath { path, width, function: function.into(), negative }));
    }

    /// Filled region (copper pour island, soldermask cutout, …).
    pub fn add_region(&mut self, path: Path, function: impl Into<String>, negative: bool) {
        self.stream.push(Graphics::Region(Region { path, function: function.into(), negative }));
    }

    /// Emit the complete Gerber file as a string.
    pub fn dump(&self) -> String {
        let mut st = EmitState::new();

        for g in &self.stream {
            match g {
                Graphics::Trace(t) => {
                    st.set_lp(t.negative);
                    let key = format!("C,{},{},{}", t.width, t.function, t.negative);
                    st.select_dcode(key, &t.function, format!("C,{}", mm_str(t.width)));
                    st.emit_path(&t.path, false);
                }
                Graphics::Region(r) => {
                    st.set_lp(r.negative);
                    if !r.function.is_empty() {
                        st.body.push(format!("%TA.AperFunction,{}*%", r.function));
                    }
                    st.body.push("G36*".into());
                    st.emit_path(&r.path, true);
                    st.body.push("G37*".into());
                    if !r.function.is_empty() {
                        st.body.push("%TD*%".into());
                    }
                }
                Graphics::Pad(p) => {
                    st.set_lp(p.master.negative());
                    match &p.master {
                        PadMaster::Circle(c) => {
                            let key = format!("C,{},{},{}", c.diameter, c.function, c.negative);
                            st.select_dcode(key, &c.function, format!("C,{}", mm_str(c.diameter)));
                            st.flash_at(p.position);
                        }
                        PadMaster::Rectangle(r) => {
                            let angle = normalize_angle(p.angle_deg);
                            if almost_multiple_of_180(angle) {
                                let key = format!("R,{},{},0,{},{}", r.x_size, r.y_size, r.function, r.negative);
                                st.select_dcode(key, &r.function, format!("R,{}X{}", mm_str(r.x_size), mm_str(r.y_size)));
                            } else if almost_multiple_of_90(angle) {
                                let key = format!("R,{},{},90,{},{}", r.y_size, r.x_size, r.function, r.negative);
                                st.select_dcode(key, &r.function, format!("R,{}X{}", mm_str(r.y_size), mm_str(r.x_size)));
                            } else {
                                let pts = rotated_rect_points(r.x_size, r.y_size, p.angle_deg);
                                let name = st.add_polygon_macro(&pts);
                                let key = format!("AM,{name},{},{}", r.function, r.negative);
                                st.select_dcode(key, &r.function, format!("{name},0"));
                            }
                            st.flash_at(p.position);
                        }
                        PadMaster::Oblong(o) => {
                            let angle = normalize_angle(p.angle_deg);
                            if almost_multiple_of_180(angle) {
                                let key = format!("O,{},{},0,{},{}", o.x_size, o.y_size, o.function, o.negative);
                                st.select_dcode(key, &o.function, format!("O,{}X{}", mm_str(o.x_size), mm_str(o.y_size)));
                            } else if almost_multiple_of_90(angle) {
                                let key = format!("O,{},{},90,{},{}", o.y_size, o.x_size, o.function, o.negative);
                                st.select_dcode(key, &o.function, format!("O,{}X{}", mm_str(o.y_size), mm_str(o.x_size)));
                            } else {
                                let pts = rotated_oblong_points(o.x_size, o.y_size, p.angle_deg);
                                let name = st.add_polygon_macro(&pts);
                                let key = format!("AM,{name},{},{}", o.function, o.negative);
                                st.select_dcode(key, &o.function, format!("{name},0"));
                            }
                            st.flash_at(p.position);
                        }
                        PadMaster::UserPolygon(poly) => {
                            let pts: Vec<Point> = if almost_zero(p.angle_deg) {
                                poly.points.clone()
                            } else {
                                poly.points.iter().map(|pt| rotate_point(*pt, p.angle_deg)).collect()
                            };
                            let name = st.add_polygon_macro(&pts);
                            let key = format!("AM,{name},{},{}", poly.function, poly.negative);
                            st.select_dcode(key, &poly.function, format!("{name},0"));
                            st.flash_at(p.position);
                        }
                    }
                }
            }
        }

        let mut out: Vec<String> = Vec::new();
        out.push("G04 Created by Alladin PCB native Gerber writer*".into());
        if let Some((vendor, app, ver)) = GENERATION_SOFTWARE.lock().unwrap().as_ref() {
            out.push(format!("%TF.GenerationSoftware,{vendor},{app},{ver}*%"));
        }
        if !self.function.is_empty() {
            out.push(format!("%TF.FileFunction,{}*%", self.function));
        }
        out.push(if self.negative {
            "%TF.FilePolarity,Negative*%".into()
        } else {
            "%TF.FilePolarity,Positive*%".into()
        });
        out.push("%FSLAX46Y46*%".into());
        out.push("%MOMM*%".into());
        out.push("%LPD*%".into());
        out.push("G01*".into());
        out.push("G75*".into());
        out.extend(st.macros);
        out.push("G04 APERTURE LIST*".into());
        out.extend(st.ad_commands);
        out.push("G04 APERTURE END LIST*".into());
        out.extend(st.body);
        out.push("M02*".into());
        out.join("\n") + "\n"
    }
}

fn normalize_angle(deg: f64) -> f64 {
    let mut a = deg % 360.0;
    if a < 0.0 {
        a += 360.0;
    }
    a
}

fn almost_zero(deg: f64) -> bool {
    deg.abs() < 1e-9 || (deg.abs() - 360.0).abs() < 1e-9
}

fn almost_multiple_of_180(deg: f64) -> bool {
    let a = normalize_angle(deg);
    a < 1e-6 || (a - 180.0).abs() < 1e-6 || (a - 360.0).abs() < 1e-6
}

fn almost_multiple_of_90(deg: f64) -> bool {
    let a = normalize_angle(deg);
    (a - 90.0).abs() < 1e-6 || (a - 270.0).abs() < 1e-6
}

fn rotate_point(p: Point, deg: f64) -> Point {
    let rad = deg.to_radians();
    let (s, c) = (rad.sin(), rad.cos());
    let x = p.x as f64;
    let y = p.y as f64;
    Point::new((x * c - y * s).round() as Unit, (x * s + y * c).round() as Unit)
}

fn rotated_rect_points(x_size: Unit, y_size: Unit, deg: f64) -> Vec<Point> {
    let hx = x_size as f64 / 2.0;
    let hy = y_size as f64 / 2.0;
    let corners = [
        Point::new(hx.round() as Unit, hy.round() as Unit),
        Point::new((-hx).round() as Unit, hy.round() as Unit),
        Point::new((-hx).round() as Unit, (-hy).round() as Unit),
        Point::new(hx.round() as Unit, (-hy).round() as Unit),
    ];
    corners.into_iter().map(|p| rotate_point(p, deg)).collect()
}

fn rotated_oblong_points(x_size: Unit, y_size: Unit, deg: f64) -> Vec<Point> {
    let (major, minor, along_x) = if x_size >= y_size {
        (x_size as f64, y_size as f64, true)
    } else {
        (y_size as f64, x_size as f64, false)
    };
    let r = minor / 2.0;
    let straight = (major - minor).max(0.0) / 2.0;
    let mut pts = Vec::new();
    for i in 0..=8 {
        let a = -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * (i as f64) / 8.0;
        let (dx, dy) = (straight + r * a.cos(), r * a.sin());
        let local = if along_x {
            Point::new(dx.round() as Unit, dy.round() as Unit)
        } else {
            Point::new(dy.round() as Unit, dx.round() as Unit)
        };
        pts.push(rotate_point(local, deg));
    }
    for i in 0..=8 {
        let a = std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * (i as f64) / 8.0;
        let (dx, dy) = (-straight + r * a.cos(), r * a.sin());
        let local = if along_x {
            Point::new(dx.round() as Unit, dy.round() as Unit)
        } else {
            Point::new(dy.round() as Unit, dx.round() as Unit)
        };
        pts.push(rotate_point(local, deg));
    }
    pts
}

fn polygon_macro(name: &str, points: &[Point]) -> String {
    let mut pts = points.to_vec();
    if pts.first() != pts.last() {
        if let Some(first) = pts.first().copied() {
            pts.push(first);
        }
    }
    let n = pts.len().saturating_sub(1);
    let mut lines = Vec::new();
    lines.push(format!("%AM{name}*"));
    lines.push(format!("4,1,{n},"));
    for p in &pts {
        lines.push(format!("{},{},", mm_str(p.x), mm_str(p.y)));
    }
    lines.push("$1*%".into());
    lines.join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::padmasters::Circle;
    use alladin_geom::MM;

    #[test]
    fn dumps_a_via_and_a_trace_with_x2_attributes() {
        set_generation_software("Dragan Bojovic", "Alladin PCB", "0.2.0-beta.1");
        let mut layer = GerberLayer::new("Copper,L1,Top,Signal", false);
        layer.add_pad(PadMaster::Circle(Circle::new(MM / 2, "ViaPad")), Point::new(MM, MM), 0.0);
        layer.add_trace_line(Point::new(0, 0), Point::new(2 * MM, 0), MM / 4, "Conductor");
        let gbr = layer.dump();
        assert!(gbr.contains("%TF.FileFunction,Copper,L1,Top,Signal*%"));
        assert!(gbr.contains("%TF.FilePolarity,Positive*%"));
        assert!(gbr.contains("%FSLAX46Y46*%"));
        assert!(gbr.contains("%TA.AperFunction,ViaPad*%"));
        assert!(gbr.contains("C,0.500000"));
        assert!(gbr.contains("D03*"));
        assert!(gbr.contains("D01*"));
        assert!(gbr.contains("M02*"));
    }

    #[test]
    fn profile_layer_strokes_a_closed_outline() {
        let mut layer = GerberLayer::new("Profile,NP", false);
        let mut path = Path::new();
        path.moveto(Point::new(0, 0));
        path.lineto(Point::new(10 * MM, 0));
        path.lineto(Point::new(10 * MM, 10 * MM));
        path.lineto(Point::new(0, 10 * MM));
        path.lineto(Point::new(0, 0));
        layer.add_traces_path(path, MM / 10, "Profile", false);
        let gbr = layer.dump();
        assert!(gbr.contains("%TF.FileFunction,Profile,NP*%"));
        assert!(gbr.matches("D01*").count() >= 4);
    }
}
