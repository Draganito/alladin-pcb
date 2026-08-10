//! `alladin-pcb`: correct-by-construction 2-layer PCB editor (egui/eframe).
//! Manual 45°-guided tracks + segment drag; native Gerber/BOM/CPL export.
//! Desktop: SQLite parts DB, LCSC download, mini MCP. WASM: board/parts via
//! file upload/download (no LCSC proxy, no MCP).

mod app;
mod board_doc;
mod bom;
mod dxf_outline;
mod footprint;
mod native_gerber;
mod persistence;
mod ratsnest;
mod routing;
mod stroke_font;
mod stroke_font_data;
mod zone_fill;

#[cfg(not(target_arch = "wasm32"))]
mod cli;
#[cfg(not(target_arch = "wasm32"))]
mod lcsc;
#[cfg(target_arch = "wasm32")]
#[path = "lcsc_wasm.rs"]
mod lcsc;
#[cfg(not(target_arch = "wasm32"))]
mod mcp;
#[cfg(not(target_arch = "wasm32"))]
mod mcp_routing;
mod parts_db;
mod parts_transfer;
#[cfg(target_arch = "wasm32")]
mod web_io;

#[cfg(not(target_arch = "wasm32"))]
fn take_flag(args: &mut Vec<String>, flag: &str) -> bool {
    match args.iter().position(|a| a == flag) {
        Some(pos) => {
            args.remove(pos);
            true
        }
        None => false,
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn main() -> std::process::ExitCode {
    let mut args: Vec<String> = std::env::args().collect();
    let allow_ai_write = take_flag(&mut args, "--allow-ai-write");

    if args.len() > 1 {
        use clap::Parser;
        return match cli::run(cli::Cli::parse_from(args)) {
            Ok(()) => std::process::ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("error: {e}");
                std::process::ExitCode::FAILURE
            }
        };
    }

    let native_options = eframe::NativeOptions::default();
    let result = eframe::run_native(
        "Alladin PCB",
        native_options,
        Box::new(move |cc| {
            // Prefer dark panels so a light desktop theme does not
            // wash out the board canvas contrast.
            cc.egui_ctx.set_visuals(eframe::egui::Visuals::dark());
            Ok(Box::new(app::PcbApp::new(allow_ai_write)))
        }),
    );
    match result {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::ExitCode::FAILURE
        }
    }
}

#[cfg(target_arch = "wasm32")]
fn main() {
    use wasm_bindgen::JsCast;
    console_error_panic_hook::set_once();
    let web_options = eframe::WebOptions::default();
    wasm_bindgen_futures::spawn_local(async {
        let document = web_sys::window()
            .expect("no window")
            .document()
            .expect("no document");
        let canvas = document
            .get_element_by_id("alladin_pcb_canvas")
            .expect("missing #alladin_pcb_canvas")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("element is not a canvas");
        eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|cc| {
                    // Browser egui defaults to the OS/light theme; force
                    // the same dark panels as the desktop build.
                    cc.egui_ctx.set_visuals(eframe::egui::Visuals::dark());
                    Ok(Box::new(app::PcbApp::new(false)))
                }),
            )
            .await
            .expect("failed to start eframe");
    });
}
