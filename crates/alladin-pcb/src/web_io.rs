//! Browser file pick / download helpers for the WASM build.
//! Uses `rfd::AsyncFileDialog` so Open/Save/Import work without a proxy.

use std::sync::mpsc;

pub enum PickedFile {
    Ok { name: String, bytes: Vec<u8> },
    #[allow(dead_code)]
    Err(String),
    Cancelled,
}

/// Opens the browser file picker; result arrives on the returned channel.
pub fn pick_file(filter_name: &str, extensions: &[&str]) -> mpsc::Receiver<PickedFile> {
    let (tx, rx) = mpsc::channel();
    let filter_name = filter_name.to_string();
    let extensions: Vec<String> = extensions.iter().map(|s| (*s).to_string()).collect();
    wasm_bindgen_futures::spawn_local(async move {
        let exts: Vec<&str> = extensions.iter().map(|s| s.as_str()).collect();
        let dialog = rfd::AsyncFileDialog::new().add_filter(&filter_name, &exts);
        match dialog.pick_file().await {
            Some(handle) => {
                let name = handle.file_name();
                let bytes = handle.read().await;
                let _ = tx.send(PickedFile::Ok { name, bytes });
            }
            None => {
                let _ = tx.send(PickedFile::Cancelled);
            }
        }
    });
    rx
}

/// Triggers a browser download of `bytes` under `filename`.
pub fn download_bytes(filename: &str, bytes: Vec<u8>) {
    let filename = filename.to_string();
    wasm_bindgen_futures::spawn_local(async move {
        let dialog = rfd::AsyncFileDialog::new().set_file_name(&filename);
        if let Some(handle) = dialog.save_file().await {
            let _ = handle.write(&bytes).await;
        }
    });
}
