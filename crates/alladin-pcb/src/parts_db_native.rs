//! The user's own SQL parts library. Backed by SQLite (`rusqlite`,
//! bundled -- no system library dependency) at a per-user data
//! directory, so it persists across every board, independent of any
//! single `.json` save file.
//!
//! Parts arrive either by hand ("Add part..." for simple through-hole
//! shapes) or via `crate::lcsc` (download by LCSC C-number). A
//! [`PartRecord`] is little more than a persisted [`FootprintTemplate`]
//! plus metadata a template itself has no room for (`lcsc_code`,
//! `description`) -- everything that already knows how to
//! place/drag/route/save a `FootprintTemplate` keeps working unchanged
//! for a database-backed one.

use std::path::{Path, PathBuf};

use rusqlite::{params, Connection, OptionalExtension};

use alladin_core::LayerId;
use alladin_geom::{Point, Unit};

use crate::footprint::{Courtyard, FootprintTemplate, HoleTemplate, PadShapeKind, PadTemplate};

/// The GUI's "Place part" category tree (`crate::app`) groups every
/// part with no real `category` (`None`, see [`PartRecord::category`]'s
/// own doc comment) under this label -- purely a *display* fallback,
/// never a string this module itself writes into the `category`
/// column (a part is either given a real category or left `NULL`/
/// empty, normalized the same way by [`PartsDb::insert_part_categorized`]
/// and [`PartsDb::load_part`]). [`PartsDb::delete_category_tree`] treats
/// this exact label specially for that reason -- see its own doc
/// comment.
pub const UNCATEGORIZED_LABEL: &str = "Uncategorized";

#[derive(Debug)]
pub enum PartsDbError {
    Sqlite(rusqlite::Error),
    /// [`PartsDb::insert_part`] refuses a second part with an
    /// already-registered LCSC code -- once the real downloader exists,
    /// this is exactly the check that turns "download C123456 again"
    /// into "already in your library", not a silent duplicate.
    DuplicateLcscCode(String),
    /// Non-SQLite failures (e.g. parts-library JSON import/export).
    Message(String),
}

impl From<rusqlite::Error> for PartsDbError {
    fn from(e: rusqlite::Error) -> Self {
        PartsDbError::Sqlite(e)
    }
}

impl std::fmt::Display for PartsDbError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PartsDbError::Sqlite(e) => write!(f, "parts database error: {e}"),
            PartsDbError::DuplicateLcscCode(code) => write!(f, "{code} is already in your parts library"),
            PartsDbError::Message(m) => write!(f, "{m}"),
        }
    }
}

/// One row in the user's parts library.
#[derive(Debug)]
pub struct PartRecord {
    pub id: i64,
    pub lcsc_code: Option<String>,
    pub description: String,
    /// This part's place in the "Place part" category tree, e.g.
    /// `"Resistors"` (an LCSC download's EasyEDA tag) or a nested path
    /// like `"Passives/Resistors"` — `/` is the GUI tree separator.
    /// `None` for parts that were never given a category (grouped under
    /// "Uncategorized" in the GUI).
    pub category: Option<String>,
    pub template: FootprintTemplate,
}

pub struct PartsDb {
    conn: Connection,
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

/// `(kind, width, height)` -- `width`/`height` are `0` and ignored for
/// `Circle` (kept simple: one row, three columns, no nullable pair that
/// only sometimes means something).
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

impl PartsDb {
    fn init(conn: Connection) -> Result<Self, PartsDbError> {
        conn.execute_batch(
            "CREATE TABLE IF NOT EXISTS parts (
                id INTEGER PRIMARY KEY,
                lcsc_code TEXT UNIQUE,
                name TEXT NOT NULL,
                reference_prefix TEXT NOT NULL,
                description TEXT NOT NULL DEFAULT '',
                exclude_from_bom INTEGER NOT NULL DEFAULT 0,
                courtyard_center_x INTEGER,
                courtyard_center_y INTEGER,
                courtyard_width INTEGER,
                courtyard_height INTEGER
            );
            CREATE TABLE IF NOT EXISTS part_pads (
                part_id INTEGER NOT NULL REFERENCES parts(id),
                pad_index INTEGER NOT NULL,
                offset_x INTEGER NOT NULL,
                offset_y INTEGER NOT NULL,
                radius INTEGER NOT NULL,
                layer TEXT NOT NULL,
                number TEXT NOT NULL DEFAULT '',
                shape_kind TEXT NOT NULL DEFAULT 'circle',
                shape_width INTEGER NOT NULL DEFAULT 0,
                shape_height INTEGER NOT NULL DEFAULT 0,
                pad_rotation_deg REAL NOT NULL DEFAULT 0.0,
                hole_diameter INTEGER NOT NULL DEFAULT 0,
                pin_name TEXT
            );
            CREATE TABLE IF NOT EXISTS part_holes (
                part_id INTEGER NOT NULL REFERENCES parts(id),
                hole_index INTEGER NOT NULL,
                offset_x INTEGER NOT NULL,
                offset_y INTEGER NOT NULL,
                drill INTEGER NOT NULL
            );",
        )?;
        Self::migrate_part_pads_columns(&conn)?;
        Self::migrate_parts_columns(&conn)?;
        Ok(Self { conn })
    }

    /// Adds the `number`/`shape_kind`/`shape_width`/`shape_height`/
    /// `pad_rotation_deg` columns to `part_pads` if they're missing.
    /// `CREATE TABLE IF NOT EXISTS` above only sets up a *brand-new*
    /// database with the current schema -- it's a no-op against an
    /// already-existing `parts.sqlite3` predating these columns (every
    /// user who tried the parts database before this change has one),
    /// so every `INSERT`/`SELECT` against it would otherwise fail with
    /// "table part_pads has no column named ..." forever. This makes
    /// opening an old database self-healing instead.
    fn migrate_part_pads_columns(conn: &Connection) -> Result<(), PartsDbError> {
        let mut existing = std::collections::HashSet::new();
        {
            let mut stmt = conn.prepare("PRAGMA table_info(part_pads)")?;
            let mut rows = stmt.query([])?;
            while let Some(row) = rows.next()? {
                let name: String = row.get(1)?;
                existing.insert(name);
            }
        }
        let missing_columns: [(&str, &str); 7] = [
            ("number", "TEXT NOT NULL DEFAULT ''"),
            ("shape_kind", "TEXT NOT NULL DEFAULT 'circle'"),
            ("shape_width", "INTEGER NOT NULL DEFAULT 0"),
            ("shape_height", "INTEGER NOT NULL DEFAULT 0"),
            ("pad_rotation_deg", "REAL NOT NULL DEFAULT 0.0"),
            ("hole_diameter", "INTEGER NOT NULL DEFAULT 0"),
            ("pin_name", "TEXT"),
        ];
        for (name, declaration) in missing_columns {
            if !existing.contains(name) {
                conn.execute(&format!("ALTER TABLE part_pads ADD COLUMN {name} {declaration}"), [])?;
            }
        }
        Ok(())
    }

    /// Adds the `exclude_from_bom` column to `parts` if it's missing --
    /// the same self-healing reasoning as
    /// [`Self::migrate_part_pads_columns`] (whose own doc comment
    /// explains why `CREATE TABLE IF NOT EXISTS` alone can't add a
    /// column to an already-existing table), just for `parts` instead
    /// of `part_pads`. `part_holes` needs no such migration: it's a
    /// brand-new *table*, not a new column on an old one, so `CREATE
    /// TABLE IF NOT EXISTS` above already handles an old database
    /// correctly on its own.
    fn migrate_parts_columns(conn: &Connection) -> Result<(), PartsDbError> {
        let mut existing = std::collections::HashSet::new();
        {
            let mut stmt = conn.prepare("PRAGMA table_info(parts)")?;
            let mut rows = stmt.query([])?;
            while let Some(row) = rows.next()? {
                let name: String = row.get(1)?;
                existing.insert(name);
            }
        }
        if !existing.contains("exclude_from_bom") {
            conn.execute("ALTER TABLE parts ADD COLUMN exclude_from_bom INTEGER NOT NULL DEFAULT 0", [])?;
        }
        // The four `courtyard_*` columns (see [`Courtyard`]) are all
        // nullable and only added together -- a database predating
        // this feature has none of them, so checking just the first
        // is enough to know all four are missing.
        if !existing.contains("courtyard_center_x") {
            conn.execute_batch(
                "ALTER TABLE parts ADD COLUMN courtyard_center_x INTEGER;
                 ALTER TABLE parts ADD COLUMN courtyard_center_y INTEGER;
                 ALTER TABLE parts ADD COLUMN courtyard_width INTEGER;
                 ALTER TABLE parts ADD COLUMN courtyard_height INTEGER;",
            )?;
        }
        // Same self-healing reasoning again, this time for the
        // "Place part" category tree (see `PartRecord::category`'s own
        // doc comment) -- a database predating this feature simply has
        // every existing part come back `category: None`, landing in
        // the GUI's "Uncategorized" bucket rather than failing to open.
        if !existing.contains("category") {
            conn.execute("ALTER TABLE parts ADD COLUMN category TEXT", [])?;
        }
        Ok(())
    }

    pub fn open(path: &Path) -> Result<Self, PartsDbError> {
        Self::init(Connection::open(path)?)
    }

    pub fn open_in_memory() -> Result<Self, PartsDbError> {
        Self::init(Connection::open_in_memory()?)
    }

    /// A per-user data directory (`~/.local/share/alladin-pcb/` on
    /// Linux, the platform equivalent elsewhere) -- this is meant to be
    /// *the* personal parts library, not a per-project/per-board file,
    /// so it has to live somewhere board-independent.
    pub fn default_path() -> PathBuf {
        let base = dirs::data_dir().unwrap_or_else(std::env::temp_dir);
        base.join("alladin-pcb").join("parts.sqlite3")
    }

    /// Opens (creating if necessary, including the containing directory)
    /// the parts library at [`Self::default_path`].
    pub fn open_default() -> Result<Self, PartsDbError> {
        let path = Self::default_path();
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        Self::open(&path)
    }

    /// Registers a new part with `pads` (in the exact order they'll be
    /// placed/routed in, same convention as
    /// [`crate::footprint::builtin_templates`]), plus its own
    /// mechanical `holes` (see [`crate::footprint::HoleTemplate`] --
    /// empty for every ordinary electrical part) and whether it should
    /// be `exclude_from_bom` (see [`FootprintTemplate::exclude_from_bom`]'s
    /// own doc comment -- an explicit caller choice, not inferred from
    /// e.g. "has holes but no pads", so a caller can still opt a
    /// pads-and-holes part out of the BOM if it isn't purchasable).
    /// Refuses a second part under the same non-`None` `lcsc_code` -- see
    /// [`PartsDbError::DuplicateLcscCode`].
    ///
    /// Production callers use [`Self::insert_part_categorized`]
    /// directly; this shorthand survives for tests that exercise the library.
    #[cfg_attr(not(test), allow(dead_code))]
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
        self.insert_part_with_courtyard(name, reference_prefix, description, lcsc_code, pads, holes, exclude_from_bom, None)
    }

    /// Same as [`Self::insert_part`], plus a real
    /// [`Courtyard`] when one was actually found in the source data
    /// (currently only `crate::lcsc::FetchedPart::explicit_courtyard`,
    /// for a real download) -- kept as a separate method rather than
    /// a new required parameter on [`Self::insert_part`] so callers that
    /// only need the bbox fallback pass `None` and stay simple.
    #[cfg_attr(not(test), allow(dead_code))]
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

    /// The *full* constructor every other `insert_part*` variant
    /// delegates to -- same as [`Self::insert_part_with_courtyard`],
    /// plus this part's own `category` in the "Place part" tree (see
    /// [`PartRecord::category`]'s own doc comment for what a caller
    /// should actually put there). Kept as one more method rather than
    /// a new required parameter on the two existing constructors so
    /// every caller that doesn't care about categories (every existing
    /// test, `bom.rs`, every hand-added built-in-style part) keeps
    /// compiling unchanged.
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
        // Same normalization as `Self::load_part` -- an empty-string
        // category must round-trip as `None` from this constructor's
        // own return value too, not just after a reload through
        // `get_part`/`list_parts`.
        let category = category.filter(|c| !c.is_empty());
        if let Some(code) = lcsc_code {
            if self.find_by_lcsc_code(code)?.is_some() {
                return Err(PartsDbError::DuplicateLcscCode(code.to_string()));
            }
        }

        let (cx, cy, cw, ch) = match explicit_courtyard {
            Some(c) => (Some(c.center.x), Some(c.center.y), Some(c.width), Some(c.height)),
            None => (None, None, None, None),
        };
        self.conn.execute(
            "INSERT INTO parts (lcsc_code, name, reference_prefix, description, exclude_from_bom, courtyard_center_x, courtyard_center_y, courtyard_width, courtyard_height, category)
             VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10)",
            params![lcsc_code, name, reference_prefix, description, exclude_from_bom, cx, cy, cw, ch, category],
        )?;
        let id = self.conn.last_insert_rowid();
        self.insert_pads_and_holes(id, pads, holes)?;

        Ok(PartRecord {
            id,
            lcsc_code: lcsc_code.map(str::to_string),
            description: description.to_string(),
            category: category.map(str::to_string),
            template: FootprintTemplate {
                name: name.to_string(),
                reference_prefix: reference_prefix.to_string(),
                pads: pads.to_vec(),
                holes: holes.to_vec(),
                exclude_from_bom,
                explicit_courtyard,
            },
        })
    }

    /// Inserts every `pads`/`holes` row for an already-`INSERT`ed (or,
    /// for [`Self::update_part_by_lcsc_code`], just-cleared) `parts`
    /// row -- the one `part_pads`/`part_holes` write path both
    /// [`Self::insert_part_with_courtyard`] and
    /// [`Self::update_part_by_lcsc_code`] share, so the two never
    /// silently drift apart on column list or value mapping.
    fn insert_pads_and_holes(&self, part_id: i64, pads: &[PadTemplate], holes: &[HoleTemplate]) -> Result<(), PartsDbError> {
        for (index, pad) in pads.iter().enumerate() {
            let (shape_kind, shape_width, shape_height) = shape_to_row(pad.shape);
            self.conn.execute(
                "INSERT INTO part_pads (part_id, pad_index, offset_x, offset_y, radius, layer, number, shape_kind, shape_width, shape_height, pad_rotation_deg, hole_diameter, pin_name)
                 VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12, ?13)",
                params![
                    part_id,
                    index as i64,
                    pad.offset.x,
                    pad.offset.y,
                    pad.radius,
                    layer_to_text(pad.layer),
                    pad.number,
                    shape_kind,
                    shape_width,
                    shape_height,
                    pad.rotation_deg,
                    pad.hole_diameter.unwrap_or(0),
                    pad.pin_name,
                ],
            )?;
        }
        for (index, hole) in holes.iter().enumerate() {
            self.conn.execute(
                "INSERT INTO part_holes (part_id, hole_index, offset_x, offset_y, drill) VALUES (?1, ?2, ?3, ?4, ?5)",
                params![part_id, index as i64, hole.offset.x, hole.offset.y, hole.drill],
            )?;
        }
        Ok(())
    }

    /// Re-fetched, in-place update of an *already-downloaded* part,
    /// keyed by its own `lcsc_code` -- unlike [`Self::insert_part_with_courtyard`]
    /// (which refuses a duplicate `lcsc_code` outright), this is
    /// exactly for backfilling a genuinely new field onto parts
    /// downloaded *before* that field existed (this session's own
    /// `explicit_courtyard`, extracted from data `crate::lcsc` always
    /// fetches anyway but didn't used to keep) without losing the
    /// part's own database row id -- and so every board that already
    /// references it by `template_origin` -- without a
    /// delete-then-reinsert round trip. Refuses with a plain SQLite
    /// "no rows" error if `lcsc_code` isn't already in the library --
    /// this is deliberately never how a *new* part gets added.
    #[cfg_attr(not(test), allow(dead_code))]
    #[allow(clippy::too_many_arguments)]
    pub fn update_part_by_lcsc_code(
        &self,
        lcsc_code: &str,
        name: &str,
        reference_prefix: &str,
        description: &str,
        pads: &[PadTemplate],
        holes: &[HoleTemplate],
        explicit_courtyard: Option<Courtyard>,
        category: Option<&str>,
    ) -> Result<PartRecord, PartsDbError> {
        let id: i64 = self.conn.query_row("SELECT id FROM parts WHERE lcsc_code = ?1", params![lcsc_code], |row| row.get(0))?;

        let (cx, cy, cw, ch) = match explicit_courtyard {
            Some(c) => (Some(c.center.x), Some(c.center.y), Some(c.width), Some(c.height)),
            None => (None, None, None, None),
        };
        self.conn.execute(
            "UPDATE parts SET name = ?1, reference_prefix = ?2, description = ?3,
             courtyard_center_x = ?4, courtyard_center_y = ?5, courtyard_width = ?6, courtyard_height = ?7, category = ?8
             WHERE id = ?9",
            params![name, reference_prefix, description, cx, cy, cw, ch, category, id],
        )?;
        self.conn.execute("DELETE FROM part_pads WHERE part_id = ?1", params![id])?;
        self.conn.execute("DELETE FROM part_holes WHERE part_id = ?1", params![id])?;
        self.insert_pads_and_holes(id, pads, holes)?;

        self.load_part(id)
    }

    /// Looks up one part by its database row id -- what
    /// `crate::app`'s `template_origin[i]: Option<i64>` (the id a
    /// template was loaded from, see that field's doc comment) actually
    /// stores, so anything needing the full [`PartRecord`] back from a
    /// live template list (currently just `crate::bom`, for the
    /// per-line LCSC part number/description) goes through this rather
    /// than re-deriving it some other way.
    pub fn get_part(&self, id: i64) -> Result<PartRecord, PartsDbError> {
        self.load_part(id)
    }

    pub fn find_by_lcsc_code(&self, code: &str) -> Result<Option<PartRecord>, PartsDbError> {
        let id: Option<i64> =
            self.conn.query_row("SELECT id FROM parts WHERE lcsc_code = ?1", params![code], |row| row.get(0)).optional()?;
        id.map(|id| self.load_part(id)).transpose()
    }

    #[allow(clippy::type_complexity)]
    fn load_part(&self, id: i64) -> Result<PartRecord, PartsDbError> {
        let (lcsc_code, name, reference_prefix, description, exclude_from_bom, cx, cy, cw, ch, category): (
            Option<String>,
            String,
            String,
            String,
            bool,
            Option<Unit>,
            Option<Unit>,
            Option<Unit>,
            Option<Unit>,
            Option<String>,
        ) = self.conn.query_row(
            "SELECT lcsc_code, name, reference_prefix, description, exclude_from_bom, courtyard_center_x, courtyard_center_y, courtyard_width, courtyard_height, category
             FROM parts WHERE id = ?1",
            params![id],
            |row| Ok((row.get(0)?, row.get(1)?, row.get(2)?, row.get(3)?, row.get(4)?, row.get(5)?, row.get(6)?, row.get(7)?, row.get(8)?, row.get(9)?)),
        )?;
        // All four columns are always written together (see
        // `Self::insert_part_with_courtyard`) -- either every one is
        // `Some` (a real silkscreen courtyard was found) or every one
        // is `NULL` (fall back to `FootprintTemplate::courtyard`'s own
        // pad/hole bounding box).
        let explicit_courtyard = match (cx, cy, cw, ch) {
            (Some(cx), Some(cy), Some(width), Some(height)) => Some(Courtyard { center: Point::new(cx, cy), width, height }),
            _ => None,
        };

        let mut stmt = self.conn.prepare(
            "SELECT offset_x, offset_y, radius, layer, number, shape_kind, shape_width, shape_height, pad_rotation_deg, hole_diameter, pin_name
             FROM part_pads WHERE part_id = ?1 ORDER BY pad_index",
        )?;
        let pads = stmt
            .query_map(params![id], |row| {
                let x: Unit = row.get(0)?;
                let y: Unit = row.get(1)?;
                let radius: Unit = row.get(2)?;
                let layer: String = row.get(3)?;
                let number: String = row.get(4)?;
                let shape_kind: String = row.get(5)?;
                let shape_width: Unit = row.get(6)?;
                let shape_height: Unit = row.get(7)?;
                let rotation_deg: f64 = row.get(8)?;
                let hole_diameter: Unit = row.get(9)?;
                let pin_name: Option<String> = row.get(10)?;
                Ok(PadTemplate {
                    offset: Point::new(x, y),
                    radius,
                    layer: layer_from_text(&layer),
                    number,
                    shape: shape_from_row(&shape_kind, shape_width, shape_height),
                    rotation_deg,
                    hole_diameter: (hole_diameter > 0).then_some(hole_diameter),
                    pin_name,
                })
            })?
            .collect::<Result<Vec<_>, _>>()?;

        let mut hole_stmt = self.conn.prepare("SELECT offset_x, offset_y, drill FROM part_holes WHERE part_id = ?1 ORDER BY hole_index")?;
        let holes = hole_stmt
            .query_map(params![id], |row| {
                let x: Unit = row.get(0)?;
                let y: Unit = row.get(1)?;
                let drill: Unit = row.get(2)?;
                Ok(HoleTemplate { offset: Point::new(x, y), drill })
            })?
            .collect::<Result<Vec<_>, _>>()?;

        // Normalize a stored empty string the same as `NULL` -- a
        // caller passing `Some("")` (e.g. an unfilled `AddPartForm`
        // category box) means exactly the same "no category" as never
        // setting the column at all.
        let category = category.filter(|c| !c.is_empty());

        Ok(PartRecord { id, lcsc_code, description, category, template: FootprintTemplate { name, reference_prefix, pads, holes, exclude_from_bom, explicit_courtyard } })
    }

    /// Every part in the library, ordered by insertion (id) order.
    pub fn list_parts(&self) -> Result<Vec<PartRecord>, PartsDbError> {
        let ids: Vec<i64> = {
            let mut stmt = self.conn.prepare("SELECT id FROM parts ORDER BY id")?;
            let rows = stmt.query_map([], |row| row.get(0))?.collect::<Result<Vec<_>, _>>()?;
            rows
        };
        ids.into_iter().map(|id| self.load_part(id)).collect()
    }

    pub fn delete_part(&self, id: i64) -> Result<(), PartsDbError> {
        self.conn.execute("DELETE FROM part_pads WHERE part_id = ?1", params![id])?;
        self.conn.execute("DELETE FROM part_holes WHERE part_id = ?1", params![id])?;
        self.conn.execute("DELETE FROM parts WHERE id = ?1", params![id])?;
        Ok(())
    }

    /// Every distinct, non-empty `category` in the library, sorted --
    /// what the GUI's "Place part" tree (`crate::app`) and any future
    /// headless listing group parts by. A part with no category (see
    /// [`PartRecord::category`]'s own doc comment) is never returned
    /// here -- it lands in the GUI's own separate "Uncategorized"
    /// bucket instead of a real database value. (The GUI currently
    /// derives its tree from `list_parts` records directly; only the
    /// tests below read categories through this.)
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn list_categories(&self) -> Result<Vec<String>, PartsDbError> {
        let mut stmt = self.conn.prepare("SELECT DISTINCT category FROM parts WHERE category IS NOT NULL AND category != '' ORDER BY category")?;
        let rows = stmt.query_map([], |row| row.get(0))?.collect::<Result<Vec<String>, _>>()?;
        Ok(rows)
    }

    /// Deletes every part whose `category` is exactly `prefix`, *or*
    /// starts with `"{prefix}/"` -- one call covers both "delete this
    /// one leaf category" (e.g. `"Imported files/my_board"`, deleting
    /// only that one board's parts) and "delete this whole top-level
    /// bucket and everything nested under it" (e.g. `"Imported
    /// files"`, deleting every board's parts at once) -- the GUI's
    /// category tree (`crate::app`) picks which prefix to pass
    /// depending on which header's own delete button was clicked.
    ///
    /// `prefix == UNCATEGORIZED_LABEL` is special-cased to also match a
    /// real `NULL` category, not just literal rows that happen to be
    /// named that string (there are none in practice -- nothing ever
    /// writes that literal text into the column, see
    /// [`UNCATEGORIZED_LABEL`]'s own doc comment): the GUI's category
    /// tree only ever *displays* a `NULL`/no-category part under that
    /// label, it never stores it as a real value, so without this a
    /// click on that bucket's own delete button always reported
    /// "deleted 0" and left every part exactly where it was --
    /// silently doing nothing rather than what its own button said.
    /// Returns how many parts were actually deleted, so a caller can
    /// show e.g. "Deleted 12 parts" rather than a silent no-op when
    /// `prefix` didn't match anything.
    pub fn delete_category_tree(&self, prefix: &str) -> Result<usize, PartsDbError> {
        let ids: Vec<i64> = {
            let like_pattern = format!("{}/%", prefix.replace('\\', "\\\\").replace('%', "\\%").replace('_', "\\_"));
            let mut stmt =
                self.conn.prepare("SELECT id FROM parts WHERE category = ?1 OR category LIKE ?2 ESCAPE '\\' OR (?1 = ?3 AND category IS NULL)")?;
            let rows = stmt.query_map(params![prefix, like_pattern, UNCATEGORIZED_LABEL], |row| row.get(0))?.collect::<Result<Vec<_>, _>>()?;
            rows
        };
        for id in &ids {
            self.delete_part(*id)?;
        }
        Ok(ids.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alladin_geom::MM;

    fn pad(offset: Point, radius: Unit, number: &str) -> PadTemplate {
        PadTemplate {
            offset,
            radius,
            layer: LayerId::FCu,
            number: number.to_string(),
            shape: PadShapeKind::Circle,
            rotation_deg: 0.0,
            hole_diameter: None,
            pin_name: None,
        }
    }

    fn two_pin_pads() -> Vec<PadTemplate> {
        vec![pad(Point::new(-MM, 0), 400_000, "1"), pad(Point::new(MM, 0), 400_000, "2")]
    }

    #[test]
    fn insert_and_list_round_trips_a_part_with_its_pads() {
        let db = PartsDb::open_in_memory().unwrap();
        db.insert_part("My Resistor", "R", "a 0805 resistor", None, &two_pin_pads(), &[], false).unwrap();

        let parts = db.list_parts().unwrap();
        assert_eq!(parts.len(), 1);
        assert_eq!(parts[0].template.name, "My Resistor");
        assert_eq!(parts[0].template.reference_prefix, "R");
        assert_eq!(parts[0].description, "a 0805 resistor");
        assert_eq!(parts[0].template.pads.len(), 2);
        assert_eq!(parts[0].template.pads[0].offset, Point::new(-MM, 0));
        assert!(parts[0].lcsc_code.is_none());
    }

    #[test]
    fn insert_preserves_pad_order() {
        let db = PartsDb::open_in_memory().unwrap();
        let pads = vec![pad(Point::new(0, 0), 100, "1"), pad(Point::new(MM, 0), 200, "2"), pad(Point::new(2 * MM, 0), 300, "3")];
        db.insert_part("Row", "U", "", None, &pads, &[], false).unwrap();

        let loaded = db.list_parts().unwrap();
        let radii: Vec<Unit> = loaded[0].template.pads.iter().map(|p| p.radius).collect();
        assert_eq!(radii, vec![100, 200, 300], "pad order must survive the round trip");
    }

    #[test]
    fn opening_a_pre_existing_database_with_the_old_part_pads_schema_self_heals() {
        // Reproduces a real bug: a `parts.sqlite3` created before pad
        // number/shape/rotation existed has a `part_pads` table
        // *without* those columns. `CREATE TABLE IF NOT EXISTS` is a
        // no-op against it, so every insert used to fail with "table
        // part_pads has no column named number" forever, on every
        // single existing user's database.
        let conn = Connection::open_in_memory().unwrap();
        conn.execute_batch(
            "CREATE TABLE parts (
                id INTEGER PRIMARY KEY,
                lcsc_code TEXT UNIQUE,
                name TEXT NOT NULL,
                reference_prefix TEXT NOT NULL,
                description TEXT NOT NULL DEFAULT ''
            );
            CREATE TABLE part_pads (
                part_id INTEGER NOT NULL REFERENCES parts(id),
                pad_index INTEGER NOT NULL,
                offset_x INTEGER NOT NULL,
                offset_y INTEGER NOT NULL,
                radius INTEGER NOT NULL,
                layer TEXT NOT NULL
            );",
        )
        .unwrap();

        let db = PartsDb::init(conn).expect("opening an old-schema database must not fail outright");
        db.insert_part("Old-Schema Survivor", "U", "", None, &two_pin_pads(), &[], false).expect("insert must work after self-healing migration");
        assert_eq!(db.list_parts().unwrap()[0].template.pads[0].number, "1");
    }

    #[test]
    fn insert_preserves_pad_numbers_shapes_and_local_rotation() {
        let db = PartsDb::open_in_memory().unwrap();
        let pads = vec![
            PadTemplate {
                offset: Point::new(0, 0),
                radius: 500_000,
                layer: LayerId::FCu,
                number: "1".to_string(),
                shape: PadShapeKind::Rect { width: 900_000, height: 700_000 },
                rotation_deg: 90.0,
                hole_diameter: None,
                pin_name: Some("GND".to_string()),
            },
            PadTemplate {
                offset: Point::new(MM, 0),
                radius: 300_000,
                layer: LayerId::BCu,
                number: "A2".to_string(),
                shape: PadShapeKind::Oval { width: 600_000, height: 300_000 },
                rotation_deg: 0.0,
                hole_diameter: Some(400_000),
                pin_name: None,
            },
        ];
        db.insert_part("QFN-ish", "U", "", None, &pads, &[], false).unwrap();

        let loaded = &db.list_parts().unwrap()[0].template.pads;
        assert_eq!(loaded[0].number, "1");
        assert_eq!(loaded[0].shape, PadShapeKind::Rect { width: 900_000, height: 700_000 });
        assert_eq!(loaded[0].rotation_deg, 90.0);
        assert_eq!(loaded[0].layer, LayerId::FCu);
        assert_eq!(loaded[0].hole_diameter, None, "an SMD pad must round-trip with no hole");
        assert_eq!(loaded[0].pin_name, Some("GND".to_string()), "a pin's schematic function name must round-trip");
        assert_eq!(loaded[1].number, "A2");
        assert_eq!(loaded[1].shape, PadShapeKind::Oval { width: 600_000, height: 300_000 });
        assert_eq!(loaded[1].layer, LayerId::BCu);
        assert_eq!(loaded[1].hole_diameter, Some(400_000), "a through-hole pad's drill size must round-trip");
        assert_eq!(loaded[1].pin_name, None, "a pin with no known function name must round-trip as None");
    }

    #[test]
    fn insert_and_list_round_trips_a_mechanical_part_with_holes_and_no_pads() {
        let db = PartsDb::open_in_memory().unwrap();
        let holes = vec![HoleTemplate { offset: Point::new(0, 0), drill: 2_200_000 }];
        db.insert_part("M2 Mounting Hole", "H", "2.2mm NPTH", None, &[], &holes, true).unwrap();

        let parts = db.list_parts().unwrap();
        assert_eq!(parts.len(), 1);
        assert!(parts[0].template.pads.is_empty(), "a mechanical hole part has no electrical pads");
        assert_eq!(parts[0].template.holes.len(), 1);
        assert_eq!(parts[0].template.holes[0].offset, Point::new(0, 0));
        assert_eq!(parts[0].template.holes[0].drill, 2_200_000);
        assert!(parts[0].template.exclude_from_bom, "a mounting hole must round-trip as excluded from the BOM");
    }

    #[test]
    fn insert_preserves_hole_order_and_a_part_can_mix_pads_and_holes() {
        let db = PartsDb::open_in_memory().unwrap();
        let holes = vec![HoleTemplate { offset: Point::new(0, 0), drill: 100 }, HoleTemplate { offset: Point::new(MM, 0), drill: 200 }];
        db.insert_part("Wire Pad + Hole", "H", "", None, &two_pin_pads(), &holes, false).unwrap();

        let loaded = &db.list_parts().unwrap()[0].template;
        assert_eq!(loaded.pads.len(), 2, "pads must survive alongside holes on the same part");
        let drills: Vec<Unit> = loaded.holes.iter().map(|h| h.drill).collect();
        assert_eq!(drills, vec![100, 200], "hole order must survive the round trip");
        assert!(!loaded.exclude_from_bom);
    }

    #[test]
    fn delete_part_removes_its_holes_too() {
        let db = PartsDb::open_in_memory().unwrap();
        let holes = vec![HoleTemplate { offset: Point::new(0, 0), drill: 2_200_000 }];
        let record = db.insert_part("Doomed Hole", "H", "", None, &[], &holes, true).unwrap();

        db.delete_part(record.id).unwrap();

        assert!(db.list_parts().unwrap().is_empty());
        let leftover: i64 = db.conn.query_row("SELECT COUNT(*) FROM part_holes WHERE part_id = ?1", params![record.id], |row| row.get(0)).unwrap();
        assert_eq!(leftover, 0, "deleting a part must also delete its part_holes rows");
    }

    #[test]
    fn opening_a_pre_existing_database_without_exclude_from_bom_or_part_holes_self_heals() {
        // Reproduces the same class of bug as
        // `opening_a_pre_existing_database_with_the_old_part_pads_schema_self_heals`,
        // but for this session's schema additions: a `parts.sqlite3`
        // created before `exclude_from_bom` and `part_holes` existed.
        let conn = Connection::open_in_memory().unwrap();
        conn.execute_batch(
            "CREATE TABLE parts (
                id INTEGER PRIMARY KEY,
                lcsc_code TEXT UNIQUE,
                name TEXT NOT NULL,
                reference_prefix TEXT NOT NULL,
                description TEXT NOT NULL DEFAULT ''
            );
            CREATE TABLE part_pads (
                part_id INTEGER NOT NULL REFERENCES parts(id),
                pad_index INTEGER NOT NULL,
                offset_x INTEGER NOT NULL,
                offset_y INTEGER NOT NULL,
                radius INTEGER NOT NULL,
                layer TEXT NOT NULL,
                number TEXT NOT NULL DEFAULT '',
                shape_kind TEXT NOT NULL DEFAULT 'circle',
                shape_width INTEGER NOT NULL DEFAULT 0,
                shape_height INTEGER NOT NULL DEFAULT 0,
                pad_rotation_deg REAL NOT NULL DEFAULT 0.0,
                hole_diameter INTEGER NOT NULL DEFAULT 0
            );",
        )
        .unwrap();

        let db = PartsDb::init(conn).expect("opening a database predating exclude_from_bom/part_holes must not fail outright");
        let holes = vec![HoleTemplate { offset: Point::new(0, 0), drill: 2_200_000 }];
        db.insert_part("Old-Schema Hole Survivor", "H", "", None, &[], &holes, true).expect("insert must work after self-healing migration");

        let loaded = &db.list_parts().unwrap()[0].template;
        assert_eq!(loaded.holes.len(), 1);
        assert!(loaded.exclude_from_bom);
    }

    #[test]
    fn insert_part_with_courtyard_round_trips_the_explicit_courtyard() {
        let db = PartsDb::open_in_memory().unwrap();
        let courtyard = Courtyard { center: Point::new(100, -200), width: 3 * MM, height: 2 * MM };
        let record = db.insert_part_with_courtyard("Real Part", "U", "", Some("C1"), &two_pin_pads(), &[], false, Some(courtyard)).unwrap();
        assert_eq!(record.template.explicit_courtyard, Some(courtyard));

        let loaded = db.get_part(record.id).unwrap();
        assert_eq!(loaded.template.explicit_courtyard, Some(courtyard), "a real silkscreen courtyard must round-trip through SQLite");
    }

    #[test]
    fn insert_part_without_a_courtyard_round_trips_as_none() {
        let db = PartsDb::open_in_memory().unwrap();
        let record = db.insert_part("Plain Part", "U", "", None, &two_pin_pads(), &[], false).unwrap();
        assert_eq!(record.template.explicit_courtyard, None);
        assert_eq!(db.get_part(record.id).unwrap().template.explicit_courtyard, None);
    }

    #[test]
    fn update_part_by_lcsc_code_backfills_a_courtyard_without_changing_the_row_id() {
        let db = PartsDb::open_in_memory().unwrap();
        let original = db.insert_part("Old Data", "U", "old description", Some("C1"), &two_pin_pads(), &[], false).unwrap();
        assert_eq!(original.template.explicit_courtyard, None, "no courtyard yet, matching a part downloaded before this feature existed");

        let new_pads = vec![pad(Point::new(-MM, 0), 500_000, "1"), pad(Point::new(MM, 0), 500_000, "2")];
        let courtyard = Courtyard { center: Point::new(0, 0), width: 3 * MM, height: 2 * MM };
        let updated = db.update_part_by_lcsc_code("C1", "Old Data", "U", "refreshed description", &new_pads, &[], Some(courtyard), None).unwrap();

        assert_eq!(updated.id, original.id, "an update must never change the part's own database row id");
        assert_eq!(updated.description, "refreshed description");
        assert_eq!(updated.template.explicit_courtyard, Some(courtyard));
        assert_eq!(updated.template.pads[0].radius, 500_000, "the update must also refresh the re-fetched pad data itself");

        let reloaded = db.get_part(original.id).unwrap();
        assert_eq!(reloaded.template.explicit_courtyard, Some(courtyard), "the backfilled courtyard must actually persist");
    }

    #[test]
    fn update_part_by_lcsc_code_fails_for_a_code_that_was_never_downloaded() {
        let db = PartsDb::open_in_memory().unwrap();
        assert!(db.update_part_by_lcsc_code("C-nonexistent", "X", "U", "", &two_pin_pads(), &[], None, None).is_err());
    }

    #[test]
    fn refuses_a_second_part_with_the_same_lcsc_code() {
        let db = PartsDb::open_in_memory().unwrap();
        db.insert_part("Part A", "U", "", Some("C123456"), &two_pin_pads(), &[], false).unwrap();

        let err = db.insert_part("Part A Again", "U", "", Some("C123456"), &two_pin_pads(), &[], false).unwrap_err();
        assert!(matches!(err, PartsDbError::DuplicateLcscCode(code) if code == "C123456"));
        assert_eq!(db.list_parts().unwrap().len(), 1, "the duplicate must not have been inserted");
    }

    #[test]
    fn find_by_lcsc_code_locates_the_right_part() {
        let db = PartsDb::open_in_memory().unwrap();
        db.insert_part("Part A", "U", "", Some("C1"), &two_pin_pads(), &[], false).unwrap();
        db.insert_part("Part B", "U", "", Some("C2"), &two_pin_pads(), &[], false).unwrap();

        let found = db.find_by_lcsc_code("C2").unwrap().expect("C2 was inserted");
        assert_eq!(found.template.name, "Part B");
        assert!(db.find_by_lcsc_code("C-does-not-exist").unwrap().is_none());
    }

    #[test]
    fn delete_part_removes_it_and_its_pads() {
        let db = PartsDb::open_in_memory().unwrap();
        let record = db.insert_part("Doomed", "U", "", None, &two_pin_pads(), &[], false).unwrap();

        db.delete_part(record.id).unwrap();
        assert!(db.list_parts().unwrap().is_empty());
    }

    #[test]
    fn insert_part_categorized_round_trips_its_category() {
        let db = PartsDb::open_in_memory().unwrap();
        let record = db.insert_part_categorized("0603 10k", "R", "", None, &two_pin_pads(), &[], false, None, Some("Resistors")).unwrap();
        assert_eq!(record.category, Some("Resistors".to_string()));
        assert_eq!(db.get_part(record.id).unwrap().category, Some("Resistors".to_string()));
    }

    #[test]
    fn plain_insert_part_leaves_category_as_none() {
        let db = PartsDb::open_in_memory().unwrap();
        let record = db.insert_part("Plain Part", "U", "", None, &two_pin_pads(), &[], false).unwrap();
        assert_eq!(record.category, None, "insert_part must never invent a category on its own");
    }

    #[test]
    fn a_part_categorized_with_an_empty_string_round_trips_as_no_category() {
        // An unfilled `AddPartForm::category` box passes `Some("")`,
        // not `None` -- must be treated identically to never setting a
        // category at all, not a real (if blank-looking) database
        // value that would still show up in `list_categories`.
        let db = PartsDb::open_in_memory().unwrap();
        let record = db.insert_part_categorized("Plain Part", "U", "", None, &two_pin_pads(), &[], false, None, Some("")).unwrap();
        assert_eq!(record.category, None);
        assert_eq!(db.get_part(record.id).unwrap().category, None);
        assert!(db.list_categories().unwrap().is_empty());
    }

    #[test]
    fn list_categories_returns_every_distinct_non_empty_category_sorted() {
        let db = PartsDb::open_in_memory().unwrap();
        db.insert_part_categorized("A", "R", "", None, &two_pin_pads(), &[], false, None, Some("Resistors")).unwrap();
        db.insert_part_categorized("B", "R", "", None, &two_pin_pads(), &[], false, None, Some("Resistors")).unwrap();
        db.insert_part_categorized("C", "U", "", None, &two_pin_pads(), &[], false, None, Some("Imported files/board_a")).unwrap();
        db.insert_part("Uncategorized One", "U", "", None, &two_pin_pads(), &[], false).unwrap();

        assert_eq!(db.list_categories().unwrap(), vec!["Imported files/board_a".to_string(), "Resistors".to_string()]);
    }

    #[test]
    fn delete_category_tree_removes_only_the_exact_leaf_category() {
        let db = PartsDb::open_in_memory().unwrap();
        let board_a = db.insert_part_categorized("D1", "D", "", None, &two_pin_pads(), &[], false, None, Some("Imported files/board_a")).unwrap();
        let board_b = db.insert_part_categorized("D1", "D", "", None, &two_pin_pads(), &[], false, None, Some("Imported files/board_b")).unwrap();

        let deleted = db.delete_category_tree("Imported files/board_a").unwrap();

        assert_eq!(deleted, 1);
        assert!(db.get_part(board_a.id).is_err(), "board_a's own part must be gone");
        assert!(db.get_part(board_b.id).is_ok(), "board_b's identically-named part must survive untouched");
    }

    #[test]
    fn delete_category_tree_on_a_top_level_prefix_also_removes_every_nested_sub_category() {
        let db = PartsDb::open_in_memory().unwrap();
        db.insert_part_categorized("D1", "D", "", None, &two_pin_pads(), &[], false, None, Some("Imported files/board_a")).unwrap();
        db.insert_part_categorized("D1", "D", "", None, &two_pin_pads(), &[], false, None, Some("Imported files/board_b")).unwrap();
        let unrelated = db.insert_part_categorized("R1", "R", "", None, &two_pin_pads(), &[], false, None, Some("Resistors")).unwrap();

        let deleted = db.delete_category_tree("Imported files").unwrap();

        assert_eq!(deleted, 2, "both board_a's and board_b's parts must be gone in one call");
        assert!(db.list_categories().unwrap() == vec!["Resistors".to_string()]);
        assert!(db.get_part(unrelated.id).is_ok(), "an unrelated top-level category must never be touched");
    }

    #[test]
    fn delete_category_tree_on_uncategorized_deletes_every_real_null_category_part() {
        // The GUI's category tree only ever *displays* a `NULL`
        // category as "Uncategorized" -- it never stores that literal
        // string. Without the `UNCATEGORIZED_LABEL` special case,
        // `category = 'Uncategorized'` never matches a real `NULL`
        // row, so clicking that bucket's own delete button always
        // reported "deleted 0" and silently left every part in place.
        let db = PartsDb::open_in_memory().unwrap();
        let uncategorized_a = db.insert_part("Plain Part A", "U", "", None, &two_pin_pads(), &[], false).unwrap();
        let uncategorized_b = db.insert_part_categorized("Plain Part B", "U", "", None, &two_pin_pads(), &[], false, None, Some("")).unwrap();
        let categorized = db.insert_part_categorized("Resistor", "R", "", None, &two_pin_pads(), &[], false, None, Some("Resistors")).unwrap();

        let deleted = db.delete_category_tree(UNCATEGORIZED_LABEL).unwrap();

        assert_eq!(deleted, 2, "both NULL-category parts must be deleted, regardless of which insert helper created them");
        assert!(db.get_part(uncategorized_a.id).is_err());
        assert!(db.get_part(uncategorized_b.id).is_err());
        assert!(db.get_part(categorized.id).is_ok(), "a real, unrelated category must never be swept up by the \"Uncategorized\" bucket's own delete");
    }

    #[test]
    fn delete_category_tree_never_matches_a_category_that_merely_shares_a_prefix_string() {
        // "Imported files 2" must not be treated as nested under
        // "Imported files" just because it starts with the same
        // characters -- only an exact match or a real "prefix/..."
        // sub-category counts.
        let db = PartsDb::open_in_memory().unwrap();
        let lookalike = db.insert_part_categorized("X1", "X", "", None, &two_pin_pads(), &[], false, None, Some("Imported files 2")).unwrap();

        let deleted = db.delete_category_tree("Imported files").unwrap();

        assert_eq!(deleted, 0);
        assert!(db.get_part(lookalike.id).is_ok());
    }

    #[test]
    fn opening_a_pre_existing_database_without_a_category_column_self_heals() {
        // Same class of bug as the schema self-healing tests above,
        // this time for the `category` column this session adds.
        let conn = Connection::open_in_memory().unwrap();
        conn.execute_batch(
            "CREATE TABLE parts (
                id INTEGER PRIMARY KEY,
                lcsc_code TEXT UNIQUE,
                name TEXT NOT NULL,
                reference_prefix TEXT NOT NULL,
                description TEXT NOT NULL DEFAULT '',
                exclude_from_bom INTEGER NOT NULL DEFAULT 0
            );
            CREATE TABLE part_pads (
                part_id INTEGER NOT NULL REFERENCES parts(id),
                pad_index INTEGER NOT NULL,
                offset_x INTEGER NOT NULL,
                offset_y INTEGER NOT NULL,
                radius INTEGER NOT NULL,
                layer TEXT NOT NULL
            );",
        )
        .unwrap();

        let db = PartsDb::init(conn).expect("opening a database predating the category column must not fail outright");
        let record = db
            .insert_part_categorized("Old-Schema Survivor", "U", "", None, &two_pin_pads(), &[], false, None, Some("Resistors"))
            .expect("insert must work after self-healing migration");
        assert_eq!(db.get_part(record.id).unwrap().category, Some("Resistors".to_string()));
    }

    #[test]
    fn default_path_lives_under_an_alladin_pcb_subdirectory() {
        let path = PartsDb::default_path();
        assert_eq!(path.file_name().unwrap(), "parts.sqlite3");
        assert_eq!(path.parent().unwrap().file_name().unwrap(), "alladin-pcb");
    }
}
