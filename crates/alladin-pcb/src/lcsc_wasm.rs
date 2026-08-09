//! WASM stub: no network LCSC download.

#[derive(Debug)]
pub enum FetchError {
    Message(String),
}

impl std::fmt::Display for FetchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FetchError::Message(m) => write!(f, "{m}"),
        }
    }
}

#[derive(Debug, Clone)]
pub struct FetchedPart {
    pub lcsc_code: String,
    pub name: String,
    pub reference_prefix: String,
    pub description: String,
    pub category: Option<String>,
    pub pads: Vec<crate::footprint::PadTemplate>,
    pub explicit_courtyard: Option<crate::footprint::Courtyard>,
}

pub fn fetch_in_background(
    _code: String,
) -> std::sync::mpsc::Receiver<Result<FetchedPart, FetchError>> {
    let (tx, rx) = std::sync::mpsc::channel();
    let _ = tx.send(Err(FetchError::Message(
        "LCSC download is desktop-only — export parts from the desktop app and import the file here".into(),
    )));
    rx
}
