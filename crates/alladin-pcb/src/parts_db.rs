//! Parts library facade: SQLite on desktop, in-memory JSON on WASM.
#![allow(unused_imports)]

#[cfg(not(target_arch = "wasm32"))]
#[path = "parts_db_native.rs"]
mod imp;

#[cfg(target_arch = "wasm32")]
#[path = "parts_db_wasm.rs"]
mod imp;

pub use imp::*;
