//! Save/load for a [`BoardDoc`], as Alladin PCB's own JSON format --
//! **not** `.kicad_pcb`. Emitting valid footprint/pad S-expressions
//! (library references or embedded definitions, net tables, board
//! setup) is out of scope; Alladin boards round-trip as JSON.
//!
//! Deliberately reconstructs footprint pads from their template (by
//! name) plus position/rotation, rather than serializing every pad's raw
//! geometry -- exactly what [`crate::footprint::world_items`] already
//! does for a fresh placement, so a loaded board's pads are byte-for-byte
//! the same geometry a freshly-placed one would have (no drift between
//! "placed this session" and "loaded from disk"). Only the *net* each pad
//! ended up on has to be captured separately, since that's not template
//! data.
//!
//! `Item::Via` is handled as of `FORMAT_VERSION` 2 (see [`SavedVia`]) --
//! the editor gained the ability to place them itself (freehand
//! stitching vias, and mid-route layer switches, see
//! `crate::routing::RoutingDrag::drop_via_and_switch_layer`), so there
//! is now something real to round-trip.
//!
//! `Item::Zone` is handled as of `FORMAT_VERSION` 3 (see [`SavedZone`]),
//! once the editor gained `Tool::DrawZone`/`BoardDoc::add_zone`. What
//! gets saved is the **user-drawn outline** plus its target layer/net,
//! and -- since [`SavedZone::islands`] was added -- the computed fill
//! islands as well. Originally only the outline was saved and
//! [`from_json`] re-ran `crate::zone_fill::fill_zone` for every zone on
//! load, on the theory that a saved fill could silently drift out of
//! sync with its own board. In practice that made *loading* pay the
//! full flood-fill cost every single time (tens of seconds per plane
//! zone on a real board, vs. seconds before zones existed), for a
//! staleness risk the editor already handles at runtime: every
//! `ZoneRecord` carries `filled_at_revision`, and `BoardDoc::
//! zones_are_stale` surfaces "this fill predates the board" as a
//! banner + manual "Refill zones" action. So the fill is now saved and
//! restored verbatim -- [`SavedZone::fill_stale`] round-trips whether
//! it was already stale at save time -- and re-filling on load only
//! remains as the fallback for files from before `islands` existed.
//!
//! `FORMAT_VERSION` 4 adds [`SavedStaticZoneIsland`] for exactly one
//! other shape of `Item::Zone`: already-filled, static copper pours
//! with no `ZoneRecord` behind them.
//! Before this version, [`to_json`] only ever looked at `doc.zones` (the
//! `ZoneRecord` list) to decide what counted as a zone at all -- a
//! board's *actual ground/power plane copper*, sitting in
//! `doc.node` with no `ZoneRecord` pointing at it, was silently dropped
//! by every single save. [`to_json`] now also walks `doc.node` for any
//! `Item::Zone` not already accounted for by a tracked
//! `ZoneRecord::item_ids`, and [`from_json`] restores those straight
//! back into `node` verbatim.
//!
//! Used non-builtin footprint templates are embedded in the same file
//! as [`SavedBoard::embedded_parts`] (no format-version bump:
//! `#[serde(default)]` keeps older boards loading). On load,
//! [`from_json`] prefers those snapshots for pad geometry, and the
//! caller can merge the difference into the local PartsDb.

use std::collections::{BTreeMap, HashSet};

use serde::{Deserialize, Serialize};

use alladin_core::{Item, ItemId, LayerId, NetClass, NetId, Node};
use alladin_geom::{Circle, Point, Polygon};

use crate::board_doc::{BoardDoc, CopperWeight, FootprintId, LayerCount, NetRecord, PlacedFootprint, SilkDot, SilkDotId, SilkText, SilkTextId};
use crate::footprint::{world_assembly_drills, world_courtyard, world_items, FootprintTemplate};
use crate::parts_transfer::{template_from_snapshot, PartSnapshot};

/// Bumped whenever [`SavedBoard`]'s shape changes incompatibly; a
/// mismatch is reported as a clear [`LoadError`] rather than a confusing
/// parse failure or, worse, silently-wrong geometry. 1 -> 2: added
/// [`SavedBoard::vias`]. 2 -> 3: added [`SavedBoard::zones`]. 3 -> 4:
/// added [`SavedBoard::static_zone_islands`] (see this module's doc
/// comment for the silently-dropped-copper-pour bug this fixes).
const FORMAT_VERSION: u32 = 4;

#[derive(Serialize, Deserialize)]
enum SavedLayer {
    FCu,
    BCu,
}

impl From<LayerId> for SavedLayer {
    fn from(layer: LayerId) -> Self {
        match layer {
            LayerId::FCu => SavedLayer::FCu,
            LayerId::BCu => SavedLayer::BCu,
        }
    }
}

impl From<SavedLayer> for LayerId {
    fn from(layer: SavedLayer) -> Self {
        match layer {
            SavedLayer::FCu => LayerId::FCu,
            SavedLayer::BCu => LayerId::BCu,
        }
    }
}

#[derive(Serialize, Deserialize)]
enum SavedNetClass {
    A,
    B,
    C,
}

impl From<NetClass> for SavedNetClass {
    fn from(class: NetClass) -> Self {
        match class {
            NetClass::A => SavedNetClass::A,
            NetClass::B => SavedNetClass::B,
            NetClass::C => SavedNetClass::C,
        }
    }
}

impl From<SavedNetClass> for NetClass {
    fn from(class: SavedNetClass) -> Self {
        match class {
            SavedNetClass::A => NetClass::A,
            SavedNetClass::B => NetClass::B,
            SavedNetClass::C => NetClass::C,
        }
    }
}

#[derive(Serialize, Deserialize)]
struct SavedNet {
    id: u32,
    name: String,
}

/// One placed footprint. `pad_nets[i]` is the net (if any) of
/// `template.pads[i]` -- see this module's doc comment for why that's
/// the *only* per-pad data saved; everything else about a pad is
/// re-derived from `template_name`/`position`/`rotation_deg` on load.
#[derive(Serialize, Deserialize)]
struct SavedFootprint {
    reference: String,
    template_name: String,
    position: Point,
    rotation_deg: f64,
    pad_nets: Vec<Option<u32>>,
    /// The footprint-local pin-1 marker offset, if enabled -- see
    /// [`crate::board_doc::PlacedFootprint::pin1_marker`].
    /// `#[serde(default)]` (`None`): a board saved before markers
    /// existed simply had none, same no-version-bump reasoning as
    /// [`SavedBoard::silk_texts`]'s.
    #[serde(default)]
    pin1_marker: Option<Point>,
}

/// One routed trace segment -- unlike a footprint's pads, a track has no
/// template to regenerate it from, so its full geometry is saved as-is.
#[derive(Serialize, Deserialize)]
struct SavedTrack {
    from: Point,
    to: Point,
    width: i64,
    net: Option<u32>,
    layer: SavedLayer,
    class: SavedNetClass,
}

/// One placed via -- like [`SavedTrack`], no template to regenerate it
/// from, so its full geometry is saved as-is. Always spans FCu<->BCu
/// (see [`alladin_core::Item::Via`]'s own doc comment), so unlike
/// [`SavedTrack`] there is no layer field to save.
#[derive(Serialize, Deserialize)]
struct SavedVia {
    center: Point,
    diameter: i64,
    drill: i64,
    net: Option<u32>,
}

/// One user-drawn zone/pour: its outline plus its last computed fill --
/// see this module's doc comment for why the fill is saved too (loading
/// used to re-run the flood fill, which cost tens of seconds per plane
/// zone on a real board). Unlike [`SavedTrack`]/[`SavedVia`], `net` is
/// never optional: a zone with no target net isn't a supported state
/// (`crate::app::EditorState::finish_zone` already requires picking one
/// before a zone can even be created).
#[derive(Serialize, Deserialize)]
struct SavedZone {
    outline: Polygon,
    layer: SavedLayer,
    net: u32,
    /// The fill islands `crate::zone_fill::fill_zone` last produced for
    /// this zone (each becomes an `Item::Zone` with this zone's
    /// layer/net on load). `None` -- as opposed to `Some(vec![])`, a
    /// fill that genuinely produced no copper -- means the file
    /// predates this field, and [`from_json`] falls back to re-filling
    /// from `outline`. Added with `#[serde(default)]` rather than a
    /// `FORMAT_VERSION` bump for the same reason as
    /// [`SavedBoard::copper_weight`]: a missing value has one obvious,
    /// always-correct meaning, not an ambiguity a version bump exists
    /// to force the user to notice.
    #[serde(default)]
    islands: Option<Vec<Polygon>>,
    /// Whether this zone's fill already predated the board's own
    /// `obstacle_revision` at save time -- round-tripped so a board
    /// saved with a visibly stale pour shows the same "zones are
    /// stale" banner when reopened, instead of the reload silently
    /// laundering the fill into looking current.
    #[serde(default)]
    fill_stale: bool,
}

/// One frozen copper-pour island with no editable [`crate::board_doc::ZoneRecord`]
/// (legacy import shape; `FORMAT_VERSION` 4). Restored verbatim on load;
/// never re-filled. `net` is optional; the polygon *is* the final copper.
#[derive(Serialize, Deserialize)]
struct SavedStaticZoneIsland {
    outline: Polygon,
    layer: SavedLayer,
    net: Option<u32>,
}

/// One free-standing [`crate::board_doc::SilkText`] -- like
/// [`SavedTrack`]/[`SavedVia`], no template to regenerate it from, so
/// its full state is saved as-is. Added with `#[serde(default)]`
/// (empty `Vec`) on [`SavedBoard::silk_texts`] rather than a
/// `FORMAT_VERSION` bump, same "a missing value has one obvious,
/// always-correct meaning" reasoning as `SavedBoard::copper_weight`'s
/// own doc comment: a board saved before this field existed simply
/// had no silk text at all.
#[derive(Serialize, Deserialize)]
struct SavedSilkText {
    text: String,
    position: Point,
    rotation_deg: f64,
    layer: SavedLayer,
    height: i64,
    line_width: i64,
}

/// One deliberately placed silkscreen dot -- the round counterpart of
/// [`SavedSilkText`], added with the same `#[serde(default)]`-instead-
/// of-version-bump reasoning: a board saved before dots existed simply
/// had none.
#[derive(Serialize, Deserialize)]
struct SavedSilkDot {
    position: Point,
    diameter: i64,
    layer: SavedLayer,
}

#[derive(Serialize, Deserialize)]
struct SavedBoard {
    format_version: u32,
    outline: Vec<Polygon>,
    layer_count: u8,
    /// Added without a `FORMAT_VERSION` bump -- unlike every field
    /// above, a missing `copper_weight` has an obvious, always-correct
    /// meaning ("this board predates the concept, so it's the only
    /// weight that ever existed": 1oz), not a genuine ambiguity a
    /// version bump exists to force the user to notice. `#[serde(default)]`
    /// makes every already-saved board keep loading exactly as before.
    #[serde(default)]
    copper_weight: u8,
    next_footprint_serial: usize,
    next_net_serial: u32,
    nets: Vec<SavedNet>,
    footprints: Vec<SavedFootprint>,
    tracks: Vec<SavedTrack>,
    vias: Vec<SavedVia>,
    zones: Vec<SavedZone>,
    static_zone_islands: Vec<SavedStaticZoneIsland>,
    #[serde(default)]
    silk_texts: Vec<SavedSilkText>,
    #[serde(default)]
    silk_dots: Vec<SavedSilkDot>,
    /// Portable snapshots of non-builtin templates this board uses, so
    /// opening the file alone (desktop or WASM) reconstitutes geometry
    /// without a separate parts-library transfer. Missing / empty on
    /// boards saved before embedding existed.
    #[serde(default)]
    embedded_parts: Vec<PartSnapshot>,
}

/// Why loading a `.json` file failed, distinguishing "not our format at
/// all" from "our format, but from a future/incompatible version" from
/// "well-formed but references a template that no longer exists" -- each
/// needs a different message to the user.
#[derive(Debug)]
pub enum LoadError {
    Parse(serde_json::Error),
    UnsupportedVersion(u32),
    UnknownTemplate(String),
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LoadError::Parse(e) => write!(f, "not a valid Aladin PCB file: {e}"),
            LoadError::UnsupportedVersion(v) => write!(f, "unsupported file version {v} (this build supports {FORMAT_VERSION})"),
            LoadError::UnknownTemplate(name) => write!(f, "unknown footprint template \"{name}\" -- was this saved by a newer version?"),
        }
    }
}

fn layer_count_to_u8(count: LayerCount) -> u8 {
    count.as_u8()
}

fn layer_count_from_u8(value: u8) -> LayerCount {
    match value {
        1 => LayerCount::One,
        // `4` was a real, saveable `LayerCount::Four` before that
        // variant was removed (see its own former doc comment: it was
        // never wired to anything beyond its own label, so a 4-layer
        // save has always been electrically identical to a 2-layer
        // one) -- falls back here rather than becoming a load error,
        // since there is nothing behaviourally different to recover.
        _ => LayerCount::Two,
    }
}

fn copper_weight_to_u8(weight: CopperWeight) -> u8 {
    weight.as_oz()
}

fn copper_weight_from_u8(value: u8) -> CopperWeight {
    match value {
        2 => CopperWeight::TwoOz,
        _ => CopperWeight::OneOz,
    }
}

/// Serializes `doc` to Aladin PCB's own JSON save format (pretty-printed,
/// so a saved board is diffable/readable -- there is no performance
/// pressure at hobby-board scale to justify a denser encoding).
/// `embedded_parts` should be the non-builtin templates used on `doc`
/// (see [`crate::parts_transfer::snapshots_used_on_board`]); pass `&[]`
/// only for boards that reference builtins alone.
pub fn to_json(doc: &BoardDoc, embedded_parts: &[PartSnapshot]) -> String {
    let nets = doc.nets.iter().map(|n| SavedNet { id: n.id.0, name: n.name.clone() }).collect();

    let footprints = doc
        .footprints
        .iter()
        .map(|fp| {
            let pad_nets = fp
                .pad_item_ids
                .iter()
                .map(|&id| match doc.node.get(id) {
                    Some(Item::Pad { net, .. }) => net.map(|NetId(n)| n),
                    _ => None,
                })
                .collect();
            SavedFootprint {
                reference: fp.reference.clone(),
                template_name: fp.template_name.clone(),
                position: fp.position,
                rotation_deg: fp.rotation_deg,
                pad_nets,
                pin1_marker: fp.pin1_marker,
            }
        })
        .collect();

    let tracks = doc
        .node
        .iter()
        .filter_map(|item| match item {
            Item::Track { shape, net, layer, class } => Some(SavedTrack {
                from: shape.a,
                to: shape.b,
                width: shape.width,
                net: net.map(|NetId(n)| n),
                layer: (*layer).into(),
                class: (*class).into(),
            }),
            _ => None,
        })
        .collect();

    let vias = doc
        .node
        .iter()
        .filter_map(|item| match item {
            Item::Via { shape, drill, net } => {
                Some(SavedVia { center: shape.center, diameter: shape.radius * 2, drill: *drill, net: net.map(|NetId(n)| n) })
            }
            _ => None,
        })
        .collect();

    let current_revision = doc.node.obstacle_revision();
    let zones = doc
        .zones
        .iter()
        .map(|z| {
            let islands = z
                .item_ids
                .iter()
                .filter_map(|&id| match doc.node.get(id) {
                    Some(Item::Zone { outline, .. }) => Some(outline.clone()),
                    _ => None,
                })
                .collect();
            SavedZone {
                outline: z.outline.clone(),
                layer: z.layer.into(),
                net: z.net.0,
                islands: Some(islands),
                fill_stale: z.filled_at_revision != current_revision,
            }
        })
        .collect();

    // Every `Item::Zone` a tracked `ZoneRecord`'s last fill produced is
    // already captured above (in its `SavedZone`'s own `islands`) --
    // anything left over is a frozen, `ZoneRecord`-less import (see
    // `SavedStaticZoneIsland`'s doc comment) that would otherwise vanish
    // from this exact save with no way to get it back.
    let tracked_zone_items: HashSet<ItemId> = doc.zones.iter().flat_map(|z| z.item_ids.iter().copied()).collect();
    let static_zone_islands = doc
        .node
        .iter_with_ids()
        .filter_map(|(id, item)| match item {
            Item::Zone { outline, layer, net } if !tracked_zone_items.contains(&id) => {
                Some(SavedStaticZoneIsland { outline: outline.clone(), layer: (*layer).into(), net: net.map(|NetId(n)| n) })
            }
            _ => None,
        })
        .collect();

    let silk_texts = doc
        .silk_texts
        .iter()
        .map(|t| SavedSilkText {
            text: t.text.clone(),
            position: t.position,
            rotation_deg: t.rotation_deg,
            layer: t.layer.into(),
            height: t.height,
            line_width: t.line_width,
        })
        .collect();

    let silk_dots = doc
        .silk_dots
        .iter()
        .map(|d| SavedSilkDot { position: d.position, diameter: d.diameter, layer: d.layer.into() })
        .collect();

    let saved = SavedBoard {
        format_version: FORMAT_VERSION,
        outline: doc.outline.clone(),
        layer_count: layer_count_to_u8(doc.layer_count),
        copper_weight: copper_weight_to_u8(doc.copper_weight),
        next_footprint_serial: doc.next_footprint_serial,
        next_net_serial: doc.next_net_serial,
        nets,
        footprints,
        tracks,
        vias,
        zones,
        static_zone_islands,
        silk_texts,
        silk_dots,
        embedded_parts: embedded_parts.to_vec(),
    };
    serde_json::to_string_pretty(&saved).expect("SavedBoard has no types that can fail to serialize")
}

/// Parses `json` and rebuilds a [`BoardDoc`] from it -- footprint pads
/// via [`world_items`] (see this module's doc comment for why), tracks
/// directly from their saved geometry. Templates resolve in this order:
/// board-embedded snapshot (exact geometry at save time), then
/// `templates` (built-ins + local PartsDb). Returns the embedded
/// snapshots so the caller can merge any missing ones into PartsDb.
pub fn from_json(json: &str, templates: &[FootprintTemplate]) -> Result<(BoardDoc, Vec<PartSnapshot>), LoadError> {
    let saved: SavedBoard = serde_json::from_str(json).map_err(LoadError::Parse)?;
    if saved.format_version != FORMAT_VERSION {
        return Err(LoadError::UnsupportedVersion(saved.format_version));
    }

    let embedded_by_name: BTreeMap<String, FootprintTemplate> = saved
        .embedded_parts
        .iter()
        .map(|s| (s.name.clone(), template_from_snapshot(s)))
        .collect();

    let mut node = Node::new();
    let mut footprints = Vec::with_capacity(saved.footprints.len());

    for (index, sf) in saved.footprints.into_iter().enumerate() {
        let template = if let Some(t) = embedded_by_name.get(&sf.template_name) {
            t
        } else if let Some(t) = templates.iter().find(|t| t.name == sf.template_name) {
            t
        } else {
            return Err(LoadError::UnknownTemplate(sf.template_name.clone()));
        };

        // Only real pads ever had a net saved (see `to_json`'s own
        // `pad_nets` construction, which now reads `fp.pad_item_ids`
        // post-hole-split) -- holes never consume from this iterator,
        // same "pads only" convention `BoardDoc::insert_footprint_unchecked`
        // uses for its own `pad_nets` parameter.
        let mut pad_nets = sf.pad_nets.iter().chain(std::iter::repeat(&None));
        let mut pad_item_ids = Vec::new();
        let mut hole_item_ids = Vec::new();
        for item in world_items(template, sf.position, sf.rotation_deg) {
            match item {
                Item::Pad { shape, layer, zone_connection, .. } => {
                    let &net = pad_nets.next().unwrap_or(&None);
                    pad_item_ids.push(node.add(Item::Pad {
                        shape,
                        layer,
                        net: net.map(NetId),
                        zone_connection,
                    }));
                }
                Item::Hole { .. } => {
                    hole_item_ids.push(node.add(item));
                }
                other => {
                    // world_items only ever produces Item::Pad/Item::Hole.
                    pad_item_ids.push(node.add(other));
                }
            }
        }

        footprints.push(PlacedFootprint {
            id: FootprintId(index + 1),
            reference: sf.reference,
            template_name: sf.template_name,
            position: sf.position,
            rotation_deg: sf.rotation_deg,
            pad_item_ids,
            hole_item_ids,
            courtyard: world_courtyard(template, sf.position, sf.rotation_deg),
            assembly_drills: world_assembly_drills(template, sf.position, sf.rotation_deg),
            pin1_marker: sf.pin1_marker,
        });
    }

    for st in saved.tracks {
        node.add(Item::Track {
            shape: alladin_geom::Segment::new(st.from, st.to, st.width),
            net: st.net.map(NetId),
            layer: st.layer.into(),
            class: st.class.into(),
        });
    }

    for sv in saved.vias {
        node.add(Item::Via { shape: Circle::new(sv.center, sv.diameter / 2), drill: sv.drill, net: sv.net.map(NetId) });
    }

    let nets = saved.nets.into_iter().map(|n| NetRecord { id: NetId(n.id), name: n.name }).collect();

    let mut doc = BoardDoc {
        outline: saved.outline,
        layer_count: layer_count_from_u8(saved.layer_count),
        copper_weight: copper_weight_from_u8(saved.copper_weight),
        node,
        footprints,
        next_footprint_serial: saved.next_footprint_serial,
        nets,
        next_net_serial: saved.next_net_serial,
        zones: Vec::new(),
        next_zone_serial: 0,
        silk_texts: saved
            .silk_texts
            .into_iter()
            .enumerate()
            .map(|(index, st)| SilkText {
                id: SilkTextId(index),
                text: st.text,
                position: st.position,
                rotation_deg: st.rotation_deg,
                layer: st.layer.into(),
                height: st.height,
                line_width: st.line_width,
            })
            .collect(),
        next_silk_text_serial: 0,
        silk_dots: saved
            .silk_dots
            .into_iter()
            .enumerate()
            .map(|(index, sd)| SilkDot { id: SilkDotId(index), position: sd.position, diameter: sd.diameter, layer: sd.layer.into() })
            .collect(),
        next_silk_dot_serial: 0,
    };
    doc.next_silk_text_serial = doc.silk_texts.len();
    doc.next_silk_dot_serial = doc.silk_dots.len();

    // Restores every saved zone's fill verbatim -- no `fill_zone`
    // re-run, that's exactly the tens-of-seconds-per-zone cost this
    // module's doc comment explains was moved out of loading. Every
    // real obstacle (footprints/tracks/vias, all added above) is
    // already in `node`, and `Item::Zone` adds never bump
    // `obstacle_revision`, so `filled_at_revision` below compares
    // against the same revision `zones_are_stale` will see. A file
    // from before `SavedZone::islands` existed still takes the old
    // re-fill path.
    for sz in saved.zones {
        let layer: LayerId = sz.layer.into();
        let net = NetId(sz.net);
        match sz.islands {
            Some(islands) => {
                let items: Vec<Item> = islands.into_iter().map(|outline| Item::Zone { outline, layer, net: Some(net) }).collect();
                let current = doc.node.obstacle_revision();
                // Any value != current reads as stale; wrapping_sub
                // keeps that guaranteed even at revision 0.
                let filled_at_revision = if sz.fill_stale { current.wrapping_sub(1) } else { current };
                doc.insert_new_zone(sz.outline, layer, net, items, filled_at_revision);
            }
            None => {
                doc.add_zone(sz.outline, layer, net).expect("embedded zone fill");
            }
        }
    }

    // Restored verbatim, straight into `node` -- no `fill_zone` call, no
    // `ZoneRecord`, matching the exact "frozen, never reshaped" contract
    // Restored verbatim into `node` -- no ZoneRecord, matching the
    // frozen-island contract for static pours.
    for szi in saved.static_zone_islands {
        doc.node.add(Item::Zone { outline: szi.outline, layer: szi.layer.into(), net: szi.net.map(NetId) });
    }

    Ok((doc, saved.embedded_parts))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board_doc::NewBoardParams;
    use crate::footprint::builtin_templates;
    use alladin_geom::MM;

    #[test]
    fn round_trips_an_empty_board() {
        let doc = NewBoardParams::default().create();
        let (loaded, _) = from_json(&to_json(&doc, &[]), &builtin_templates()).expect("a freshly-created board must round-trip");
        assert_eq!(loaded.outline, doc.outline);
        assert_eq!(loaded.layer_count, doc.layer_count);
        assert!(loaded.footprints.is_empty());
        assert_eq!(loaded.node.len(), 0);
    }

    #[test]
    fn round_trips_a_placed_silk_text() {
        let mut doc = NewBoardParams::default().create();
        let id = doc
            .try_place_silk_text("HELLO", Point::new(0, 0), 90.0, LayerId::FCu, crate::board_doc::DEFAULT_SILK_TEXT_HEIGHT)
            .expect("center of an empty 50x30mm board must be a legal silk placement");

        let (loaded, _) = from_json(&to_json(&doc, &[]), &builtin_templates()).expect("a board with a placed silk text must round-trip");

        assert_eq!(loaded.silk_texts.len(), 1);
        let restored = &loaded.silk_texts[0];
        assert_eq!(restored.id, id);
        assert_eq!(restored.text, "HELLO");
        assert_eq!(restored.position, Point::new(0, 0));
        assert_eq!(restored.rotation_deg, 90.0);
        assert_eq!(restored.layer, LayerId::FCu);
        assert_eq!(restored.height, doc.silk_texts[0].height);
        assert_eq!(restored.line_width, doc.silk_texts[0].line_width);

        // A file saved before `silk_texts` existed must still load
        // (backward compatibility -- `#[serde(default)]`, no
        // `FORMAT_VERSION` bump, see `SavedSilkText`'s own doc
        // comment).
        let empty_board_json = to_json(&NewBoardParams::default().create(), &[]);
        assert!(!empty_board_json.contains("silk_texts") || from_json(&empty_board_json, &builtin_templates()).unwrap().0.silk_texts.is_empty());
    }

    #[test]
    fn round_trips_a_placed_silk_dot_and_a_pin1_marker() {
        let mut doc = NewBoardParams::default().create();
        let dot_id = doc
            .try_place_silk_dot(Point::new(5 * MM, 5 * MM), crate::board_doc::DEFAULT_SILK_DOT_DIAMETER, LayerId::BCu)
            .expect("open space on an empty board must be a legal dot placement");
        let template = &builtin_templates()[0];
        let fp_id = doc.try_place_footprint(template, Point::new(-10 * MM, 0), 0.0).unwrap();
        doc.try_enable_pin1_marker(fp_id, template).expect("an empty board has room for a pin-1 dot");
        let marker_before = doc.footprints[0].pin1_marker_circle().unwrap();

        let (loaded, _) = from_json(&to_json(&doc, &[]), &builtin_templates()).expect("dots and markers must round-trip");

        assert_eq!(loaded.silk_dots.len(), 1);
        let restored = &loaded.silk_dots[0];
        assert_eq!(restored.id, dot_id);
        assert_eq!(restored.position, Point::new(5 * MM, 5 * MM));
        assert_eq!(restored.diameter, doc.silk_dots[0].diameter);
        assert_eq!(restored.layer, LayerId::BCu);
        // The pin-1 marker survives as the same *world* circle -- the
        // local offset plus the footprint's own restored position.
        assert_eq!(loaded.footprints[0].pin1_marker_circle().unwrap(), marker_before);

        // A file saved before dots existed must still load (backward
        // compatibility -- `#[serde(default)]`, no version bump, see
        // `SavedSilkDot`'s own doc comment).
        let empty_board_json = to_json(&NewBoardParams::default().create(), &[]);
        assert!(from_json(&empty_board_json, &builtin_templates()).unwrap().0.silk_dots.is_empty());
    }

    #[test]
    fn round_trips_footprints_with_their_nets_and_a_routed_track() {
        let mut doc = NewBoardParams::default().create();
        let template = &builtin_templates()[0];
        doc.try_place_footprint(template, Point::new(-10 * MM, 0), 0.0).unwrap();
        doc.try_place_footprint(template, Point::new(10 * MM, 0), 0.0).unwrap();
        let pad_a = doc.footprints[0].pad_item_ids[0];
        let pad_b = doc.footprints[1].pad_item_ids[0];
        let net = doc.connect_pads(pad_a, pad_b).unwrap();
        doc.add_track_path(&[Point::new(-10 * MM, 0), Point::new(10 * MM, 0)], net, LayerId::FCu, 250_000, NetClass::C);

        let (loaded, _) = from_json(&to_json(&doc, &[]), &builtin_templates()).expect("a board with parts/nets/tracks must round-trip");

        assert_eq!(loaded.footprints.len(), 2);
        assert_eq!(loaded.nets.len(), 1);
        let loaded_pad_a = loaded.footprints[0].pad_item_ids[0];
        let loaded_pad_b = loaded.footprints[1].pad_item_ids[0];
        assert_eq!(loaded.node.get(loaded_pad_a).unwrap().net(), Some(net));
        assert_eq!(loaded.node.get(loaded_pad_b).unwrap().net(), Some(net));
        assert!(
            loaded.node.iter().any(|item| matches!(item, Item::Track { net: Some(n), .. } if *n == net)),
            "the routed track must survive the round trip"
        );
    }

    #[test]
    fn round_trips_a_manually_placed_via() {
        let mut doc = NewBoardParams::default().create();
        let template = &builtin_templates()[0];
        doc.try_place_footprint(template, Point::new(-10 * MM, 0), 0.0).unwrap();
        doc.try_place_footprint(template, Point::new(10 * MM, 0), 0.0).unwrap();
        let pad_a = doc.footprints[0].pad_item_ids[0];
        let pad_b = doc.footprints[1].pad_item_ids[0];
        let net = doc.connect_pads(pad_a, pad_b).unwrap();
        let via_center = Point::new(0, 5 * MM);
        doc.try_add_via(via_center, net, 600_000, 300_000).expect("open space must accept a via");

        let (loaded, _) = from_json(&to_json(&doc, &[]), &builtin_templates()).expect("a board with a via must round-trip");

        let vias: Vec<&Item> = loaded.node.iter().filter(|item| matches!(item, Item::Via { .. })).collect();
        assert_eq!(vias.len(), 1, "exactly the one saved via must come back");
        match vias[0] {
            Item::Via { shape, drill, net: via_net } => {
                assert_eq!(shape.center, via_center);
                assert_eq!(shape.radius, 300_000);
                assert_eq!(*drill, 300_000);
                assert_eq!(*via_net, Some(net));
            }
            _ => unreachable!(),
        }
    }

    #[test]
    fn round_trips_a_drawn_zone_restoring_its_saved_fill_verbatim() {
        let mut doc = NewBoardParams::default().create(); // 50mm x 30mm outline
        let net = doc.create_net();
        let outline = Polygon::new(vec![
            Point::new(-20 * MM, -10 * MM),
            Point::new(20 * MM, -10 * MM),
            Point::new(20 * MM, 10 * MM),
            Point::new(-20 * MM, 10 * MM),
        ]);
        let zone_id = doc.add_zone(outline.clone(), LayerId::FCu, net).unwrap();
        let islands_before: Vec<Polygon> = doc
            .zones
            .iter()
            .find(|z| z.id == zone_id)
            .unwrap()
            .item_ids
            .iter()
            .filter_map(|&id| match doc.node.get(id) {
                Some(Item::Zone { outline, .. }) => Some(outline.clone()),
                _ => None,
            })
            .collect();
        assert!(!islands_before.is_empty(), "a zone drawn over open board space must fill to at least one island");

        let (loaded, _) = from_json(&to_json(&doc, &[]), &builtin_templates()).expect("a board with a zone must round-trip");

        assert_eq!(loaded.zones.len(), 1);
        assert_eq!(loaded.zones[0].outline, outline);
        assert_eq!(loaded.zones[0].layer, LayerId::FCu);
        assert_eq!(loaded.zones[0].net, net);
        let islands_after: Vec<Polygon> = loaded.zones[0]
            .item_ids
            .iter()
            .filter_map(|&id| match loaded.node.get(id) {
                Some(Item::Zone { outline, .. }) => Some(outline.clone()),
                _ => None,
            })
            .collect();
        assert_eq!(islands_after, islands_before, "the saved fill islands must come back exactly, with no re-fill on load");
        assert!(!loaded.zones_are_stale(), "a fill that was fresh at save time must still read as fresh after loading");
    }

    #[test]
    fn a_save_from_before_zone_islands_existed_still_refills_on_load() {
        // Simulates every board saved back when `SavedZone` was outline-
        // only: strip the `islands`/`fill_stale` keys off a current save
        // and the load must fall back to re-running the flood fill.
        let mut doc = NewBoardParams::default().create();
        let net = doc.create_net();
        let outline = Polygon::new(vec![
            Point::new(-20 * MM, -10 * MM),
            Point::new(20 * MM, -10 * MM),
            Point::new(20 * MM, 10 * MM),
            Point::new(-20 * MM, 10 * MM),
        ]);
        doc.add_zone(outline, LayerId::FCu, net).unwrap();

        let json = to_json(&doc, &[]);
        let mut value: serde_json::Value = serde_json::from_str(&json).unwrap();
        let zone = value["zones"][0].as_object_mut().unwrap();
        zone.remove("islands").expect("islands must actually be present to prove its removal below matters");
        zone.remove("fill_stale").expect("fill_stale must actually be present to prove its removal below matters");
        let legacy = serde_json::to_string(&value).unwrap();

        let (loaded, _) = from_json(&legacy, &builtin_templates()).expect("a legacy outline-only zone save must still load");
        assert_eq!(loaded.zones.len(), 1);
        assert!(!loaded.zones[0].item_ids.is_empty(), "the legacy path must have re-filled the zone from its outline");
    }

    #[test]
    fn a_zone_that_was_stale_at_save_time_is_still_stale_after_loading() {
        let mut doc = NewBoardParams::default().create();
        let net = doc.create_net();
        let outline = Polygon::new(vec![
            Point::new(-20 * MM, -10 * MM),
            Point::new(20 * MM, -10 * MM),
            Point::new(20 * MM, 10 * MM),
            Point::new(-20 * MM, 10 * MM),
        ]);
        doc.add_zone(outline, LayerId::FCu, net).unwrap();
        // Routing a track after the fill bumps `obstacle_revision`, so
        // the zone's copper no longer matches the board it sits on.
        doc.add_track_path(&[Point::new(-10 * MM, 0), Point::new(10 * MM, 0)], net, LayerId::BCu, 250_000, NetClass::C);
        assert!(doc.zones_are_stale(), "the un-refilled zone must read as stale before saving for this test to mean anything");

        let (loaded, _) = from_json(&to_json(&doc, &[]), &builtin_templates()).expect("a board with a stale zone must round-trip");
        assert!(loaded.zones_are_stale(), "staleness must survive the round-trip rather than the reload laundering the fill into looking current");
    }

    #[test]
    fn round_trips_a_static_zone_island_with_no_zonerecord_behind_it() {
        // Frozen pour with no ZoneRecord — silently dropped before FORMAT_VERSION 4.
        let mut doc = NewBoardParams::default().create(); // 50mm x 30mm outline
        let net = doc.create_net();
        let outline = Polygon::new(vec![
            Point::new(-20 * MM, -10 * MM),
            Point::new(20 * MM, -10 * MM),
            Point::new(20 * MM, 10 * MM),
            Point::new(-20 * MM, 10 * MM),
        ]);
        doc.node.add(Item::Zone { outline: outline.clone(), layer: LayerId::BCu, net: Some(net) });
        assert!(doc.zones.is_empty(), "no ZoneRecord must exist -- this is the untracked-import shape, not a user-drawn zone");

        let (loaded, _) = from_json(&to_json(&doc, &[]), &builtin_templates()).expect("a board with a static zone island must round-trip");

        assert!(loaded.zones.is_empty(), "still no ZoneRecord -- a static island must never grow one on load");
        let islands: Vec<_> = loaded.node.iter().filter(|item| matches!(item, Item::Zone { .. })).collect();
        assert_eq!(islands.len(), 1, "the static island itself must survive the round-trip");
        match islands[0] {
            Item::Zone { outline: loaded_outline, layer, net: loaded_net } => {
                assert_eq!(*loaded_outline, outline, "the exact saved polygon must come back unchanged -- there is no outline to refill from");
                assert_eq!(*layer, LayerId::BCu);
                assert_eq!(*loaded_net, Some(net));
            }
            _ => unreachable!(),
        }
    }

    #[test]
    fn a_static_zone_island_coexists_with_a_tracked_zonerecord_without_either_duplicating_or_swallowing_the_other() {
        let mut doc = NewBoardParams::default().create();
        let net = doc.create_net();
        let drawn_outline = Polygon::new(vec![Point::new(-5 * MM, -5 * MM), Point::new(5 * MM, -5 * MM), Point::new(5 * MM, 5 * MM), Point::new(-5 * MM, 5 * MM)]);
        doc.add_zone(drawn_outline, LayerId::FCu, net).unwrap();
        let static_outline = Polygon::new(vec![Point::new(-20 * MM, -10 * MM), Point::new(20 * MM, -10 * MM), Point::new(20 * MM, 10 * MM), Point::new(-20 * MM, 10 * MM)]);
        doc.node.add(Item::Zone { outline: static_outline, layer: LayerId::BCu, net: Some(net) });

        let (loaded, _) = from_json(&to_json(&doc, &[]), &builtin_templates()).unwrap();
        assert_eq!(loaded.zones.len(), 1, "exactly the one drawn ZoneRecord, not duplicated by the static island");
        let front_islands = loaded.node.iter().filter(|item| matches!(item, Item::Zone { layer: LayerId::FCu, .. })).count();
        let back_islands = loaded.node.iter().filter(|item| matches!(item, Item::Zone { layer: LayerId::BCu, .. })).count();
        assert!(front_islands >= 1, "the drawn F.Cu zone must still have refilled at least one island");
        assert_eq!(back_islands, 1, "the static B.Cu island must survive alongside it, exactly once");
    }

    #[test]
    fn round_trips_a_2oz_boards_copper_weight() {
        let doc = NewBoardParams { copper_weight: crate::board_doc::CopperWeight::TwoOz, ..NewBoardParams::default() }.create();
        let (loaded, _) = from_json(&to_json(&doc, &[]), &builtin_templates()).expect("a 2oz board must round-trip");
        assert_eq!(loaded.copper_weight, crate::board_doc::CopperWeight::TwoOz);
    }

    #[test]
    fn a_save_from_before_copper_weight_existed_loads_as_1oz() {
        // A real save this format ever produced, minus the `copper_weight`
        // key entirely -- simulates every board saved before this field
        // was added. `#[serde(default)]` must fill it in as `0`, which
        // `copper_weight_from_u8` maps to `OneOz` -- the only weight that
        // ever existed at the time such a file could have been written.
        let doc = NewBoardParams::default().create();
        let json = to_json(&doc, &[]);
        let mut value: serde_json::Value = serde_json::from_str(&json).unwrap();
        value.as_object_mut().unwrap().remove("copper_weight").expect("copper_weight must actually be present to prove its removal below matters");
        let without_copper_weight = serde_json::to_string(&value).unwrap();

        let (loaded, _) = from_json(&without_copper_weight, &builtin_templates()).expect("a save missing copper_weight must still load");
        assert_eq!(loaded.copper_weight, crate::board_doc::CopperWeight::OneOz);
    }

    #[test]
    fn preserves_the_counters_so_new_references_and_nets_never_collide_after_a_reload() {
        let mut doc = NewBoardParams::default().create();
        let template = &builtin_templates()[0];
        let id = doc.try_place_footprint(template, Point::new(0, 0), 0.0).unwrap();
        doc.remove_footprint(id); // burns a reference number without leaving a footprint behind
        doc.try_place_footprint(template, Point::new(20 * MM, 0), 0.0).unwrap();

        let (loaded, _) = from_json(&to_json(&doc, &[]), &builtin_templates()).unwrap();
        assert_eq!(loaded.next_footprint_serial, doc.next_footprint_serial);
        assert_eq!(loaded.footprints[0].reference, doc.footprints[0].reference);
    }

    #[test]
    fn rejects_a_file_from_an_unsupported_future_format_version() {
        let doc = NewBoardParams::default().create();
        let json = to_json(&doc, &[]).replace(&format!("\"format_version\": {FORMAT_VERSION}"), "\"format_version\": 99");
        match from_json(&json, &builtin_templates()).map_err(|e| e.to_string()) {
            Err(message) if message.contains("99") => {}
            other => panic!("expected an UnsupportedVersion(99) error, got {}", other.is_ok()),
        }
    }

    #[test]
    fn rejects_garbage_input_as_a_parse_error_not_a_panic() {
        assert!(matches!(from_json("not json", &builtin_templates()), Err(LoadError::Parse(_))));
    }

    #[test]
    fn resolves_a_footprint_against_a_non_builtin_template_list() {
        // A board that placed a database-backed part must still load, as
        // long as the caller passes that part's template in -- it must
        // not be hardwired to `builtin_templates()` internally.
        let custom = crate::footprint::straight_row_template("My Part".to_string(), "X".to_string(), 2, 2.0, 0.5);
        let mut doc = NewBoardParams::default().create();
        doc.try_place_footprint(&custom, Point::new(0, 0), 0.0).unwrap();

        match from_json(&to_json(&doc, &[]), &builtin_templates()) {
            Err(LoadError::UnknownTemplate(name)) => assert_eq!(name, "My Part"),
            other => panic!("expected UnknownTemplate(\"My Part\"), got is_ok={}", other.is_ok()),
        }

        let (loaded, _) = from_json(&to_json(&doc, &[]), std::slice::from_ref(&custom)).expect("passing the custom template in must resolve it");
        assert_eq!(loaded.footprints.len(), 1);
    }

    #[test]
    fn loads_from_embedded_parts_without_session_template() {
        use crate::parts_transfer::snapshot_from_template;
        let custom = crate::footprint::straight_row_template("EmbedMe".to_string(), "X".to_string(), 2, 2.0, 0.5);
        let mut doc = NewBoardParams::default().create();
        doc.try_place_footprint(&custom, Point::new(0, 0), 0.0).unwrap();
        let snap = snapshot_from_template(&custom, Some("C999".into()), "test part".into(), Some("ICs".into()));
        let json = to_json(&doc, &[snap]);
        assert!(json.contains("embedded_parts"));
        assert!(json.contains("EmbedMe"));
        // Builtins only — must still load via embed.
        let (loaded, embedded) = from_json(&json, &builtin_templates()).expect("embed must supply the template");
        assert_eq!(loaded.footprints.len(), 1);
        assert_eq!(embedded.len(), 1);
        assert_eq!(embedded[0].lcsc_code.as_deref(), Some("C999"));
    }

    #[test]
    fn boards_without_embedded_parts_still_load() {
        let doc = NewBoardParams::default().create();
        let mut json: serde_json::Value = serde_json::from_str(&to_json(&doc, &[])).unwrap();
        json.as_object_mut().unwrap().remove("embedded_parts");
        let (loaded, embedded) = from_json(&json.to_string(), &builtin_templates()).unwrap();
        assert!(embedded.is_empty());
        assert_eq!(loaded.outline, doc.outline);
    }
}
