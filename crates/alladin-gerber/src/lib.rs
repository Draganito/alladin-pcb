//! Native RS-274X Gerber and Excellon writers for Alladin PCB.
//!
//! The public API is deliberately modelled on Karel Tavernier's
//! [`gerber_writer`](https://github.com/Karel-Tavernier/gerber_writer)
//! (Apache 2.0): a [`GerberLayer`] accumulates pads / traces / regions
//! with X2 attributes, then [`GerberLayer::dump`] emits a conservative,
//! fab-friendly Gerber string. Coordinates are Alladin's own nanometre
//! [`alladin_geom::Unit`] values, written with `%FSLAX46Y46*%` so the
//! integer Gerber digits *are* nanometres (same convention KiCad 9 uses).
//!
//! Drill files live in [`excellon`] -- `gerber_writer` has no Excellon
//! support, so that side is Alladin-owned.

mod excellon;
mod padmasters;
mod path;
mod writer;

pub use excellon::{DrillKind, ExcellonFile};
pub use padmasters::{Circle, Oblong, PadMaster, Rectangle, UserPolygon};
pub use path::Path;
pub use writer::{set_generation_software, GerberLayer};

/// Format an Alladin nanometre coordinate as a Gerber integer under
/// `%FSLAX46Y46*%` (unit = 1 nm). Leading zeros omitted.
pub fn gerber_coord(nm: alladin_geom::Unit) -> String {
    nm.to_string()
}

/// Format a length in millimetres for aperture definitions (`C,0.250000`).
pub fn mm_str(nm: alladin_geom::Unit) -> String {
    format!("{:.6}", nm as f64 / alladin_geom::MM as f64)
}

/// Convert Alladin nanometres to millimetres (Excellon decimal format).
pub fn to_mm(nm: alladin_geom::Unit) -> f64 {
    nm as f64 / alladin_geom::MM as f64
}
