//! Pad masters -- shapes that can be flashed with `D03`.
//!
//! Mirrors `gerber_writer.padmasters` (Circle / Rectangle / UserPolygon)
//! plus Oblong for Alladin's oval pads (`O` aperture).

use alladin_geom::{Point, Unit};

/// Something that can be flashed as a pad on a [`crate::GerberLayer`].
#[derive(Debug, Clone, PartialEq)]
pub enum PadMaster {
    Circle(Circle),
    Rectangle(Rectangle),
    Oblong(Oblong),
    UserPolygon(UserPolygon),
}

impl PadMaster {
    pub fn function(&self) -> &str {
        match self {
            PadMaster::Circle(p) => &p.function,
            PadMaster::Rectangle(p) => &p.function,
            PadMaster::Oblong(p) => &p.function,
            PadMaster::UserPolygon(p) => &p.function,
        }
    }

    pub fn negative(&self) -> bool {
        match self {
            PadMaster::Circle(p) => p.negative,
            PadMaster::Rectangle(p) => p.negative,
            PadMaster::Oblong(p) => p.negative,
            PadMaster::UserPolygon(p) => p.negative,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Circle {
    pub diameter: Unit,
    pub function: String,
    pub negative: bool,
}

impl Circle {
    pub fn new(diameter: Unit, function: impl Into<String>) -> Self {
        Self { diameter, function: function.into(), negative: false }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Rectangle {
    pub x_size: Unit,
    pub y_size: Unit,
    pub function: String,
    pub negative: bool,
}

impl Rectangle {
    pub fn new(x_size: Unit, y_size: Unit, function: impl Into<String>) -> Self {
        Self { x_size, y_size, function: function.into(), negative: false }
    }
}

/// Axis-aligned oblong / oval pad (`O` aperture). For rotated ovals,
/// expand to [`UserPolygon`] at the call site instead.
#[derive(Debug, Clone, PartialEq)]
pub struct Oblong {
    pub x_size: Unit,
    pub y_size: Unit,
    pub function: String,
    pub negative: bool,
}

impl Oblong {
    pub fn new(x_size: Unit, y_size: Unit, function: impl Into<String>) -> Self {
        Self { x_size, y_size, function: function.into(), negative: false }
    }
}

/// A filled polygon flash, vertices relative to the flash origin.
/// First and last point should form a closed ring (last == first is fine
/// but not required -- the writer closes the contour).
#[derive(Debug, Clone, PartialEq)]
pub struct UserPolygon {
    pub points: Vec<Point>,
    pub function: String,
    pub negative: bool,
}

impl UserPolygon {
    pub fn new(points: Vec<Point>, function: impl Into<String>) -> Self {
        Self { points, function: function.into(), negative: false }
    }
}
