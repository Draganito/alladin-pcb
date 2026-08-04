//! `alladin-pcb`: a minimalist, correct-by-construction PCB layout editor
//! for hobbyist boards -- egui/eframe, built on the same `alladin-geom`/
//! `alladin-core` geometry and collision model `alladin-router` already
//! trusts, and sharing its board-rendering code (`Camera`, `draw_board`)
//! with `alladin-viewer` via the `alladin-render` crate.
//!
//! Deliberately **not** an autorouter (see the development log's "Teil 29"
//! entry for the full pivot rationale): net creation and routing decisions
//! stay with the human or an AI driving this tool's future CLI, with
//! interactive walkaround/shove assistance while dragging a track --
//! `alladin-router`'s existing `walkaround`/`shove`/`capsule_walkaround`
//! primitives (proven against real boards already) are the mechanism this
//! is headed toward, just triggered by a mouse drag instead of a fully
//! automatic search.
//!
//! Also runs **headless**, as a scripting/AI command-line interface --
//! see `cli`'s module doc comment -- for the "board erstellen, parts
//! platzieren, netzliste machen, alles über die Kommandozeile"
//! interface the project's vision explicitly calls for. Launched by
//! running this same binary with a subcommand (`--help` lists them);
//! with no arguments at all, it starts the GUI below as always.

mod app;
mod background;
mod board_doc;
mod bom;
mod cli;
mod external_router;
mod footprint;
mod kicad_export;
mod kicad_import;
mod native_gerber;
mod lcsc;
mod mcp;
mod parts_db;
mod persistence;
mod ratsnest;
mod routing;
mod stroke_font;
mod stroke_font_data;
mod zone_fill;

/// Strips `flag` out of `args` (wherever it appears) and reports whether
/// it was present -- used for `--allow-ai-write` below, which needs to
/// be recognized by the GUI launch path itself, *before* the "any
/// argument at all means run headless" check `fn main` makes right
/// after. Ordinary CLI subcommands never see this flag (clap would
/// otherwise reject it as unrecognized on subcommands that don't
/// declare it).
fn take_flag(args: &mut Vec<String>, flag: &str) -> bool {
    match args.iter().position(|a| a == flag) {
        Some(pos) => {
            args.remove(pos);
            true
        }
        None => false,
    }
}

fn main() -> std::process::ExitCode {
    let mut args: Vec<String> = std::env::args().collect();
    // Opt-in switch for the embedded MCP server's *write* tools (see
    // `crate::mcp`'s module doc comment) -- an AI can only ever place
    // parts/route/save through MCP if you launched the GUI with this
    // flag; otherwise every write tool call is refused with a clear
    // message. The read-only introspection tools are unaffected -- those
    // are always on regardless of this flag. Stripped out of `args`
    // before the CLI-dispatch check below, since it's a GUI-only switch,
    // not a `clap` subcommand.
    let allow_ai_write = take_flag(&mut args, "--allow-ai-write");

    // Any argument at all (even just `--help`) means "run headless as a
    // CLI", not "launch the GUI" -- see `cli`'s module doc comment.
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
    let result = eframe::run_native("Alladin PCB", native_options, Box::new(move |_cc| Ok(Box::new(app::PcbApp::new(allow_ai_write)))));
    match result {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::ExitCode::FAILURE
        }
    }
}
