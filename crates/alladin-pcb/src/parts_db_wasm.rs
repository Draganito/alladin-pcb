//! In-memory parts library for WASM — filled by importing a JSON export
//! from the desktop app (no SQLite / no LCSC in the browser).

use std::cell::RefCell;
use std::path::{Path, PathBuf};

use crate::footprint::{Courtyard, FootprintTemplate, HoleTemplate, PadTemplate};

pub const UNCATEGORIZED_LABEL: &str = "Uncategorized";

#[derive(Debug)]
pub enum PartsDbError {
    Message(String),
    DuplicateLcscCode(String),
}

impl std::fmt::Display for PartsDbError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PartsDbError::Message(m) => write!(f, "{m}"),
            PartsDbError::DuplicateLcscCode(code) => write!(f, "{code} is already in your parts library"),
        }
    }
}

#[derive(Debug, Clone)]
pub struct PartRecord {
    pub id: i64,
    pub lcsc_code: Option<String>,
    pub description: String,
    pub category: Option<String>,
    pub template: FootprintTemplate,
}

struct Inner {
    next_id: i64,
    parts: Vec<PartRecord>,
}

pub struct PartsDb {
    inner: RefCell<Inner>,
}

impl PartsDb {
    pub fn open(_path: &Path) -> Result<Self, PartsDbError> {
        Self::open_in_memory()
    }

    pub fn open_in_memory() -> Result<Self, PartsDbError> {
        Ok(Self {
            inner: RefCell::new(Inner { next_id: 1, parts: Vec::new() }),
        })
    }

    pub fn default_path() -> PathBuf {
        PathBuf::from("alladin-parts.json")
    }

    pub fn open_default() -> Result<Self, PartsDbError> {
        Self::open_in_memory()
    }

    pub fn list_parts(&self) -> Result<Vec<PartRecord>, PartsDbError> {
        Ok(self.inner.borrow().parts.clone())
    }

    pub fn get_part(&self, id: i64) -> Result<PartRecord, PartsDbError> {
        self.inner
            .borrow()
            .parts
            .iter()
            .find(|p| p.id == id)
            .cloned()
            .ok_or_else(|| PartsDbError::Message(format!("no part with id {id}")))
    }

    pub fn find_by_lcsc_code(&self, code: &str) -> Result<Option<PartRecord>, PartsDbError> {
        Ok(self.inner.borrow().parts.iter().find(|p| p.lcsc_code.as_deref() == Some(code)).cloned())
    }

    pub fn delete_part(&self, id: i64) -> Result<(), PartsDbError> {
        self.inner.borrow_mut().parts.retain(|p| p.id != id);
        Ok(())
    }

    pub fn delete_category_tree(&self, prefix: &str) -> Result<usize, PartsDbError> {
        let mut inner = self.inner.borrow_mut();
        let before = inner.parts.len();
        if prefix == UNCATEGORIZED_LABEL {
            inner.parts.retain(|p| p.category.as_deref().filter(|c| !c.is_empty()).is_some());
        } else {
            inner.parts.retain(|p| !p.category.as_deref().unwrap_or("").starts_with(prefix));
        }
        Ok(before - inner.parts.len())
    }

    pub fn insert_part(
        &self,
        name: &str,
        reference_prefix: &str,
        description: &str,
        lcsc_code: Option<&str>,
        pads: &[PadTemplate],
        holes: &[HoleTemplate],
        exclude_from_bom: bool,
    ) -> Result<PartRecord, PartsDbError> {
        self.insert_part_categorized(name, reference_prefix, description, lcsc_code, pads, holes, exclude_from_bom, None, None)
    }

    pub fn insert_part_with_courtyard(
        &self,
        name: &str,
        reference_prefix: &str,
        description: &str,
        lcsc_code: Option<&str>,
        pads: &[PadTemplate],
        holes: &[HoleTemplate],
        exclude_from_bom: bool,
        explicit_courtyard: Option<Courtyard>,
    ) -> Result<PartRecord, PartsDbError> {
        self.insert_part_categorized(name, reference_prefix, description, lcsc_code, pads, holes, exclude_from_bom, explicit_courtyard, None)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn insert_part_categorized(
        &self,
        name: &str,
        reference_prefix: &str,
        description: &str,
        lcsc_code: Option<&str>,
        pads: &[PadTemplate],
        holes: &[HoleTemplate],
        exclude_from_bom: bool,
        explicit_courtyard: Option<Courtyard>,
        category: Option<&str>,
    ) -> Result<PartRecord, PartsDbError> {
        if let Some(code) = lcsc_code {
            if self.find_by_lcsc_code(code)?.is_some() {
                return Err(PartsDbError::DuplicateLcscCode(code.to_string()));
            }
        }
        let category = category.filter(|c| !c.is_empty()).map(|s| s.to_string());
        let mut inner = self.inner.borrow_mut();
        let id = inner.next_id;
        inner.next_id += 1;
        let record = PartRecord {
            id,
            lcsc_code: lcsc_code.map(|s| s.to_string()),
            description: description.to_string(),
            category,
            template: FootprintTemplate {
                name: name.to_string(),
                reference_prefix: reference_prefix.to_string(),
                pads: pads.to_vec(),
                holes: holes.to_vec(),
                exclude_from_bom,
                explicit_courtyard,
            },
        };
        inner.parts.push(record.clone());
        Ok(record)
    }
}
