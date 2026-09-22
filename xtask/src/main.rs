//! Repository automation entry point: `cargo xtask <command>`.
//!
//! Commands are added alongside the deliverables they serve (codegen for D2/D18,
//! coverage for D18, graph/licence invariants for D1).

fn main() {
    let cmd = std::env::args().nth(1).unwrap_or_default();
    match cmd.as_str() {
        "" | "help" => usage(),
        other => {
            eprintln!("unknown command: {other}\n");
            usage();
            std::process::exit(2);
        }
    }
}

fn usage() {
    println!("cargo xtask <command>\n\ncommands:\n  help    show this message");
}
