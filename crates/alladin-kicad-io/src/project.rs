//! Net-class *assignment*, read from a `.kicad_pro` **project** file --
//! not `.kicad_pcb`, the board file every other module in this crate
//! deals with. This split is not a design choice of this crate's own;
//! it reflects where KiCad 9 itself stores the data.
//!
//! **Discovered empirically, not assumed** (see the development log's
//! corresponding update): this crate's own module docs used to claim
//! `.kicad_pcb` "does define real net classes (`(net_class ...)`)" --
//! true for older KiCad file-format versions, but checked against every
//! real `.kicad_pcb` demo/template file shipped with KiCad 9.0.2 on this
//! machine and found to be **false** for the current format: none of
//! them contain a single `(net_class ...)` form. The actual data lives
//! in the sibling `.kicad_pro` project file, as JSON, under
//! `net_settings`:
//!
//! - `net_settings.classes[]`: each netclass's own name and DFM-ish
//!   parameters (track width, via size, clearance, ...).
//! - `net_settings.netclass_patterns[]`: `{netclass, pattern}` pairs --
//!   `pattern` is a `*`/`?` shell-style glob matched against net names;
//!   any net matching none of them implicitly belongs to `"Default"`.
//!
//! This module does two genuinely different things, kept separate
//! because only one of them has a real ground truth to check against:
//!
//! 1. [`KicadNetClasses::kicad_class_of`]: net name -> **KiCad's own**
//!    netclass name. Ground-truth-verifiable for the *single matching
//!    pattern* case, and verified: cross-checked against `pcbnew`'s own
//!    `NETINFO_ITEM::GetNetClassName()` for *every* net (not a sample)
//!    in two real KiCad demo boards -- `interf_u.kicad_pcb` (173 nets,
//!    only literal/no-wildcard patterns) and the much larger
//!    `vme-wren.kicad_pcb` (real `*`/`?` wildcard patterns, e.g.
//!    `*DDR4-PS.?ASN*`, `*VMEPX.*`) -- 0 mismatches on either. Neither
//!    file has a net matched by more than one pattern, so this doesn't
//!    exercise conflict resolution; for that case this module follows
//!    KiCad's own documented priority rule (lowest `net_settings.classes[]`
//!    priority number wins) read directly from
//!    `common/project/net_settings.cpp`'s source, verified against that
//!    algorithm rather than against `pcbnew` output -- see
//!    [`KicadNetClasses::kicad_class_of`]'s own doc comment.
//! 2. [`KicadNetClasses::alladin_class_of`]: KiCad's netclass name ->
//!    Alladin's own coarse `NetClass` (A/B/C) routing-priority scheme.
//!    **This one has no ground truth at all** -- `NetClass` is Alladin's
//!    own invention, KiCad has no equivalent concept, and real projects
//!    name their classes arbitrarily (`vme-wren.kicad_pcb` alone has
//!    `DDR4_BYTE0..3`, `DDR4_CMD`, `FPGA_HD`, `FPGA_HP`, `VMEPX`,
//!    `zse_50r`; `Edgeberry_cartridge_template.kicad_pro` has `12V`,
//!    `5V`, `HighPower`, `LessPower`). This is therefore a best-effort
//!    keyword heuristic on the class name, explicitly documented as
//!    such rather than dressed up as more principled than it is -- see
//!    [`guess_alladin_class`].

use alladin_core::NetClass;
use alladin_geom::{Unit, MM};
use serde::Deserialize;
use std::collections::HashMap;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ProjectParseError {
    #[error("failed to parse project file as JSON: {0}")]
    Json(#[from] serde_json::Error),
}

#[derive(Debug, Deserialize, Default)]
struct ProjectFile {
    #[serde(default)]
    net_settings: NetSettings,
}

#[derive(Debug, Deserialize, Default)]
struct NetSettings {
    #[serde(default)]
    classes: Vec<NetClassDef>,
    #[serde(default)]
    netclass_patterns: Vec<PatternAssignment>,
}

#[derive(Debug, Deserialize)]
struct NetClassDef {
    name: String,
    /// KiCad's own default for a netclass entry with no explicit
    /// `priority` key, taken straight from `NETCLASS`'s constructor
    /// (`common/netclass.cpp`: `SetPriority(-1)`). In practice every
    /// KiCad 9 project file has this populated for every declared class
    /// (schema-migration `migrateSchema3to4` back-fills it), so this
    /// only matters for a hand-edited or otherwise unusual file.
    #[serde(default = "default_class_priority")]
    priority: i32,
}

fn default_class_priority() -> i32 {
    -1
}

#[derive(Debug, Deserialize)]
struct PatternAssignment {
    netclass: String,
    pattern: String,
}

/// The implicit class every net belongs to unless a pattern says
/// otherwise -- matches KiCad's own fallback, confirmed against
/// `pcbnew`'s `GetNetClassName()` for every unmatched net in both real
/// files this module was validated against.
const DEFAULT_KICAD_CLASS: &str = "Default";

/// Net-class assignment rules parsed from a `.kicad_pro` project file.
/// See this module's docs for what is and isn't ground-truth-verified.
pub struct KicadNetClasses {
    /// `(pattern, netclass name)`, kept in the file's own order (the
    /// order only still matters as a deterministic tie-break of last
    /// resort -- see [`Self::kicad_class_of`]).
    patterns: Vec<(String, String)>,
    /// Netclass name -> its `priority` field from
    /// `net_settings.classes[]`. Read directly from KiCad's own
    /// resolution algorithm (`common/project/net_settings.cpp`,
    /// `NET_SETTINGS::GetEffectiveNetClass`/`makeEffectiveNetclass`):
    /// when a net matches multiple patterns pointing at different
    /// netclasses, the **lowest** priority number wins, not pattern
    /// list order.
    priorities: HashMap<String, i32>,
}

/// [`KicadNetClasses::priority_of`]'s fallback for a netclass name that
/// is referenced by a pattern but never declared in
/// `net_settings.classes[]`. Mirrors real KiCad's own fallback for
/// exactly this case (`NET_SETTINGS::GetEffectiveNetClass`'s
/// `getOrAddImplicitNetcless`, which assigns such an "implicit"
/// netclass `std::numeric_limits<int>::max() - 1`): loses to every
/// declared class, but still wins against the `Default` fallback class.
const IMPLICIT_CLASS_PRIORITY: i32 = i32::MAX - 1;

impl KicadNetClasses {
    /// Parses a `.kicad_pro` file's contents (already read into a
    /// string, matching every other Alladin crate's "pure logic, no
    /// I/O" convention).
    pub fn parse(project_source: &str) -> Result<Self, ProjectParseError> {
        let file: ProjectFile = serde_json::from_str(project_source)?;
        Ok(Self {
            patterns: file
                .net_settings
                .netclass_patterns
                .into_iter()
                .map(|p| (p.pattern, p.netclass))
                .collect(),
            priorities: file
                .net_settings
                .classes
                .into_iter()
                .map(|c| (c.name, c.priority))
                .collect(),
        })
    }

    /// This netclass's own resolution priority (lower wins), per
    /// [`Self::priorities`]'s doc comment. `Default` isn't required to
    /// appear in `net_settings.classes[]` at all (a minimal/older
    /// project might omit it entirely) but must still resolve to the
    /// lowest possible precedence, matching
    /// `NET_SETTINGS`'s own constructor
    /// (`m_defaultNetClass->SetPriority(std::numeric_limits<int>::max())`).
    fn priority_of(&self, kicad_class_name: &str) -> i32 {
        if kicad_class_name == DEFAULT_KICAD_CLASS {
            i32::MAX
        } else {
            self.priorities
                .get(kicad_class_name)
                .copied()
                .unwrap_or(IMPLICIT_CLASS_PRIORITY)
        }
    }

    /// KiCad's own netclass name for `net_name` -- ground-truth
    /// verified for the *single matching pattern* case, see this
    /// module's docs. When **multiple** patterns match the same net
    /// with *different* netclasses, resolves by [`Self::priority_of`]
    /// (lowest number wins), matching real KiCad's conflict-resolution
    /// rule -- not yet ground-truth-verified against `pcbnew` for that
    /// specific multi-match case (no real local KiCad demo project has
    /// a net matched by more than one pattern to test against), only
    /// against `common/project/net_settings.cpp`'s own algorithm read
    /// directly from the KiCad source (see this crate's tests). A
    /// residual tie (two matching patterns whose netclasses share the
    /// *same* priority number) falls back to "first matching pattern in
    /// file order wins" -- a deterministic choice for an edge case real
    /// KiCad 9 project files never produce in practice (every declared
    /// class gets a distinct sequential priority via schema migration).
    pub fn kicad_class_of(&self, net_name: &str) -> &str {
        self.patterns
            .iter()
            .filter(|(pattern, _)| glob_matches(pattern, net_name))
            .map(|(_, class)| class.as_str())
            .min_by_key(|class| self.priority_of(class))
            .unwrap_or(DEFAULT_KICAD_CLASS)
    }

    /// Best-effort guess at which of Alladin's own `NetClass` priority
    /// tiers `net_name` belongs to, based on its KiCad netclass name.
    /// **Not ground-truth-verifiable** -- see this module's docs for
    /// why. Callers with better project-specific knowledge (e.g. a
    /// user-supplied name -> `NetClass` override table) should prefer
    /// that over this guess.
    pub fn alladin_class_of(&self, net_name: &str) -> NetClass {
        guess_alladin_class(self.kicad_class_of(net_name))
    }
}

/// See [`KicadNetClasses::alladin_class_of`]'s doc comment for why this
/// is explicitly a heuristic, not a derivation. Keyword lists are
/// deliberately broad (covers real names seen across the KiCad 9 demo
/// corpus: "Power"/"power"/"pwr"/"POWER"/"HighPower"/"LessPower" for
/// `B`; "usbdiff" for `A`) but **can't** cover genuinely arbitrary names
/// with no lexical hint at all (`vme-wren.kicad_pcb`'s `DDR4_BYTE0`,
/// `FPGA_HD`, `zse_50r`, or `Edgeberry_cartridge_template.kicad_pro`'s
/// `12V`/`5V` all fall through to `C` today, even though a human reading
/// the schematic would likely call the DDR4/FPGA ones high-speed (`A`)
/// and the voltage-named ones power (`B`)). Stated as a known gap
/// rather than silently guessed around.
fn guess_alladin_class(kicad_class_name: &str) -> NetClass {
    let lower = kicad_class_name.to_lowercase();
    const HIGH_SPEED_KEYWORDS: &[&str] = &[
        "highspeed", "high_speed", "diff", "usb", "ddr", "lvds", "hdmi", "clk", "clock",
    ];
    const POWER_KEYWORDS: &[&str] = &["power", "pwr", "supply", "volt"];

    if HIGH_SPEED_KEYWORDS.iter().any(|k| lower.contains(k)) {
        NetClass::A
    } else if POWER_KEYWORDS.iter().any(|k| lower.contains(k)) {
        NetClass::B
    } else {
        NetClass::C
    }
}

/// Renders a minimal-but-valid `.kicad_pro` project file that declares
/// exactly one thing Alladin actually has an opinion about: the
/// `Default` net class's `clearance`/`track_width`/`via_diameter`/
/// `via_drill` numbers -- see this module's own doc comment for why
/// that data lives in the *project* file, not `.kicad_pcb` itself, in
/// current KiCad.
///
/// **Why this exists at all, ground-truth confirmed** (see
/// the development log's corresponding update): exporting a real,
/// already-routed Alladin board to `.kicad_pcb` alone and running
/// `kicad-cli pcb drc` against it produced 48 false `clearance`
/// violations -- every trace/pad pair Alladin itself correctly allowed
/// down to its own real JLCPCB minimum (0.1 mm), but which KiCad's
/// *built-in* fallback `Default` class (0.2 mm, used whenever no
/// project file is present at all) then flagged. Writing exactly this
/// file next to the same, byte-identical `.kicad_pcb` and re-running
/// the same `kicad-cli pcb drc` made all 48 disappear -- 0 change to
/// the board itself, only to what DRC compares it against.
///
/// Deliberately *not* a full re-serialization of everything a
/// real KiCad-authored project can carry (compare a real one, e.g.
/// KiCad's own `interf_u.kicad_pro` demo, which is ~20x longer) --
/// KiCad's project-settings loader independently defaults every field
/// it doesn't find via its own `PARAM<>` mechanism, so a file that only
/// states an opinion where Alladin actually has one is already a
/// complete, valid project on its own; ground-truth confirmed via the
/// same `kicad-cli pcb drc` run above (0 parse errors, 0 "repair this
/// project" prompts, DRC used exactly the declared clearance).
///
/// `clearance`/`track_width`/`via_diameter`/`via_drill` are internal
/// nanometre [`Unit`]s, same convention as every other Alladin API --
/// pass e.g. `alladin_core::JlcpcbClearance::PAD_TO_TRACK` (the
/// smallest clearance Alladin itself ever actually enforces between
/// two different-net copper items) for `clearance`, so the declared
/// class can never be *stricter* than what Alladin already allowed
/// while routing.
pub fn write_kicad_pro(project_filename: &str, clearance: Unit, track_width: Unit, via_diameter: Unit, via_drill: Unit) -> String {
    let to_mm = |u: Unit| u as f64 / MM as f64;
    let value = serde_json::json!({
        "board": {
            "design_settings": {
                "rules": {
                    "min_clearance": to_mm(clearance)
                }
            }
        },
        "meta": {
            "filename": project_filename,
            "version": 3
        },
        "net_settings": {
            "classes": [
                {
                    "clearance": to_mm(clearance),
                    "name": DEFAULT_KICAD_CLASS,
                    "priority": i32::MAX,
                    "track_width": to_mm(track_width),
                    "via_diameter": to_mm(via_diameter),
                    "via_drill": to_mm(via_drill)
                }
            ],
            "meta": {
                "version": 4
            },
            "netclass_patterns": []
        }
    });
    serde_json::to_string_pretty(&value).expect("a serde_json::json! literal of plain strings/numbers is always serializable")
}

/// Minimal shell-style glob matcher: `*` matches any run of characters
/// (including none), `?` matches exactly one, everything else must
/// match literally. Matches KiCad's own project-file pattern syntax
/// (verified against real wildcard patterns like `*DDR4-PS.?ASN*` and
/// `*VMEPX.*` in `vme-wren.kicad_pro`, cross-checked against `pcbnew`'s
/// own resolution -- see this module's docs). Naive recursive
/// backtracking, not a DFA -- fine for the short, sparsely-starred
/// patterns real KiCad projects use, not appropriate for adversarial
/// input.
fn glob_matches(pattern: &str, text: &str) -> bool {
    glob_bytes(pattern.as_bytes(), text.as_bytes())
}

fn glob_bytes(pattern: &[u8], text: &[u8]) -> bool {
    match (pattern.first(), text.first()) {
        (None, None) => true,
        (None, Some(_)) => false,
        (Some(b'*'), _) => {
            // Match zero characters (skip the `*`) or one-or-more (keep
            // the `*`, consume one character of `text`).
            glob_bytes(&pattern[1..], text)
                || (!text.is_empty() && glob_bytes(pattern, &text[1..]))
        }
        (Some(b'?'), Some(_)) => glob_bytes(&pattern[1..], &text[1..]),
        (Some(_), None) => false,
        (Some(p), Some(t)) if p == t => glob_bytes(&pattern[1..], &text[1..]),
        (Some(_), Some(_)) => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PROJECT_FIXTURE: &str = r#"
        {
            "net_settings": {
                "classes": [
                    { "name": "Default" },
                    { "name": "Power" }
                ],
                "netclass_patterns": [
                    { "netclass": "Power", "pattern": "GND" },
                    { "netclass": "Power", "pattern": "VCC" }
                ]
            }
        }
    "#;

    #[test]
    fn exact_pattern_match_resolves_the_right_kicad_class() {
        let classes = KicadNetClasses::parse(PROJECT_FIXTURE).unwrap();
        assert_eq!(classes.kicad_class_of("GND"), "Power");
        assert_eq!(classes.kicad_class_of("VCC"), "Power");
        assert_eq!(classes.kicad_class_of("/SPI_CLK"), "Default"); // no matching pattern
    }

    #[test]
    fn alladin_class_guess_maps_power_keyword_to_b_and_default_to_c() {
        let classes = KicadNetClasses::parse(PROJECT_FIXTURE).unwrap();
        assert_eq!(classes.alladin_class_of("GND"), NetClass::B);
        assert_eq!(classes.alladin_class_of("/SPI_CLK"), NetClass::C);
    }

    #[test]
    fn priority_field_overrides_pattern_file_order_on_a_real_conflict() {
        // Two patterns matching the *same* net with different netclasses
        // -- the exact case the previous "first pattern in file order
        // wins" simplification got wrong. File order lists the
        // low-priority (numerically higher, `5`) class *first* and the
        // high-priority (`0`) class *second*, deliberately the opposite
        // of the correct answer, so a passing test can only mean the
        // priority field, not list position, actually decided this.
        let fixture = r#"
            {
                "net_settings": {
                    "classes": [
                        { "name": "Default" },
                        { "name": "SlowClass", "priority": 5 },
                        { "name": "FastClass", "priority": 0 }
                    ],
                    "netclass_patterns": [
                        { "netclass": "SlowClass", "pattern": "*DDR4*" },
                        { "netclass": "FastClass", "pattern": "*CLK*" }
                    ]
                }
            }
        "#;
        let classes = KicadNetClasses::parse(fixture).unwrap();
        assert_eq!(classes.kicad_class_of("NET_DDR4_CLK"), "FastClass");
        // Sanity: each pattern alone (no conflict) still resolves normally.
        assert_eq!(classes.kicad_class_of("NET_DDR4_ONLY"), "SlowClass");
        assert_eq!(classes.kicad_class_of("NET_CLK_ONLY"), "FastClass");
    }

    #[test]
    fn equal_priority_conflict_falls_back_to_pattern_file_order() {
        // Real KiCad 9 project files never produce this (schema
        // migration assigns every declared class a distinct sequential
        // priority) -- documented, deterministic edge-case behaviour
        // for a hand-edited or otherwise unusual file, not a case with
        // a `pcbnew` ground truth to check against.
        let fixture = r#"
            {
                "net_settings": {
                    "classes": [
                        { "name": "Default" },
                        { "name": "A", "priority": 3 },
                        { "name": "B", "priority": 3 }
                    ],
                    "netclass_patterns": [
                        { "netclass": "A", "pattern": "*X*" },
                        { "netclass": "B", "pattern": "*X*" }
                    ]
                }
            }
        "#;
        let classes = KicadNetClasses::parse(fixture).unwrap();
        assert_eq!(classes.kicad_class_of("FOO_X_BAR"), "A");
    }

    #[test]
    fn a_pattern_referencing_an_undeclared_netclass_loses_to_a_declared_one() {
        // "Ghost" is referenced by a pattern but never appears in
        // `classes[]` -- real KiCad treats that as an "implicit"
        // netclass with priority just below every declared one (see
        // `IMPLICIT_CLASS_PRIORITY`'s doc comment), so a declared class
        // matching the same net -- even with no explicit `priority` key
        // of its own -- must still win.
        let fixture = r#"
            {
                "net_settings": {
                    "classes": [
                        { "name": "Default" },
                        { "name": "NoPriority" }
                    ],
                    "netclass_patterns": [
                        { "netclass": "NoPriority", "pattern": "*X*" },
                        { "netclass": "Ghost", "pattern": "*X*" }
                    ]
                }
            }
        "#;
        let classes = KicadNetClasses::parse(fixture).unwrap();
        assert_eq!(classes.kicad_class_of("FOO_X_BAR"), "NoPriority");
    }

    #[test]
    fn wildcard_patterns_match_like_a_shell_glob() {
        let fixture = r#"
            {
                "net_settings": {
                    "netclass_patterns": [
                        { "netclass": "DDR4_CMD", "pattern": "*DDR4-PS.?ASN*" },
                        { "netclass": "VMEPX", "pattern": "*VMEPX.*" }
                    ]
                }
            }
        "#;
        let classes = KicadNetClasses::parse(fixture).unwrap();
        // Real net names from `vme-wren.kicad_pcb`, ground-truth
        // confirmed against `pcbnew` to resolve to these exact classes.
        assert_eq!(
            classes.kicad_class_of("/vme_interface/vme_buffers_addr/VMEPX.SYSRESET"),
            "VMEPX"
        );
        assert_eq!(classes.kicad_class_of("/some/unrelated/net"), "Default");
    }

    #[test]
    fn high_speed_keyword_guess_catches_usbdiff() {
        // Real netclass name from `tiny_tapeout/tinytapeout-demo.kicad_pro`.
        assert_eq!(guess_alladin_class("usbdiff"), NetClass::A);
    }

    #[test]
    fn unrecognised_project_json_shape_is_a_parse_error_not_a_panic() {
        assert!(KicadNetClasses::parse("not json at all").is_err());
    }

    #[test]
    fn write_kicad_pro_produces_valid_json_parseable_by_serde_json() {
        let text = write_kicad_pro("board.kicad_pro", 100_000, 250_000, 600_000, 300_000);
        let _: serde_json::Value = serde_json::from_str(&text).expect("must be valid JSON");
    }

    #[test]
    fn write_kicad_pro_round_trips_through_its_own_reader_as_the_default_class() {
        // The whole point of this file: a net with no matching pattern
        // must resolve to "Default", and `KicadNetClasses` (the reader
        // this crate already ships) must be able to parse straight
        // back what `write_kicad_pro` wrote, with no reader changes.
        let text = write_kicad_pro("board.kicad_pro", 100_000, 250_000, 600_000, 300_000);
        let classes = KicadNetClasses::parse(&text).expect("must parse with this module's own reader");
        assert_eq!(classes.kicad_class_of("GND"), "Default");
        assert_eq!(classes.alladin_class_of("GND"), NetClass::C);
    }

    #[test]
    fn write_kicad_pro_declares_the_exact_clearance_it_was_given_in_millimetres() {
        // 100_000 nm == 0.1 mm -- the actual JLCPCB minimum this
        // function exists to make KiCad's own DRC respect (see this
        // function's own doc comment for the 48-false-violation
        // ground-truth story). A wrong unit conversion here would
        // silently reintroduce exactly that bug.
        let text = write_kicad_pro("board.kicad_pro", 100_000, 250_000, 600_000, 300_000);
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(value["net_settings"]["classes"][0]["clearance"], 0.1);
        assert_eq!(value["net_settings"]["classes"][0]["track_width"], 0.25);
        assert_eq!(value["net_settings"]["classes"][0]["via_diameter"], 0.6);
        assert_eq!(value["net_settings"]["classes"][0]["via_drill"], 0.3);
        assert_eq!(value["board"]["design_settings"]["rules"]["min_clearance"], 0.1);
    }

    #[test]
    fn missing_net_settings_section_defaults_to_no_patterns() {
        // A minimal/older project file might not have `net_settings` at
        // all -- must not fail to parse, just resolve everything to
        // "Default".
        let classes = KicadNetClasses::parse("{}").unwrap();
        assert_eq!(classes.kicad_class_of("anything"), "Default");
    }
}
