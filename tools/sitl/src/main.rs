use std::env;
use std::error::Error;
use std::io;
use std::path::PathBuf;

use rip_sitl::{execute, write_artifacts, RunContext, Scenario};

const DEFAULT_SYSTEM_IDENTIFIER: &str = "rotary-inverted-pendulum";

#[derive(Debug)]
struct Cli {
    scenario: PathBuf,
    output: PathBuf,
    system_identifier: String,
    git_commit: String,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("rip-sitl: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let cli =
        parse_cli().map_err(|message| io::Error::new(io::ErrorKind::InvalidInput, message))?;
    let scenario = Scenario::load(&cli.scenario)?;
    let context = RunContext::stage1(cli.system_identifier, cli.git_commit);
    let artifacts = execute(&context, &scenario)?;
    write_artifacts(&cli.output, &artifacts)?;

    println!("SITL deterministic skeleton");
    println!("scenario............... {}", scenario.id);
    println!("duration_us............ {}", scenario.duration_us);
    println!("system................. {}", context.system_identifier);
    println!("git_commit............. {}", context.git_commit);
    println!("output................. {}", cli.output.display());
    Ok(())
}

fn parse_cli() -> Result<Cli, String> {
    let mut args = env::args().skip(1);
    let mut scenario = None;
    let mut output = None;
    let mut system_identifier = None;
    let mut git_commit = None;

    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--scenario" => {
                scenario = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| "--scenario requires a path".to_string())?,
                ));
            }
            "--output" => {
                output = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| "--output requires a path".to_string())?,
                ));
            }
            "--system-identifier" => {
                system_identifier = Some(
                    args.next()
                        .ok_or_else(|| "--system-identifier requires a value".to_string())?,
                );
            }
            "--git-commit" => {
                git_commit = Some(
                    args.next()
                        .ok_or_else(|| "--git-commit requires a value".to_string())?,
                );
            }
            "--help" | "-h" => {
                println!(
                    "usage: rip-sitl --scenario <scenario.toml> --output <directory> \
                     [--system-identifier <id>] [--git-commit <sha>]"
                );
                std::process::exit(0);
            }
            _ => return Err(format!("unknown argument: {argument}")),
        }
    }

    Ok(Cli {
        scenario: scenario.ok_or_else(|| "missing --scenario".to_string())?,
        output: output.ok_or_else(|| "missing --output".to_string())?,
        system_identifier: system_identifier
            .unwrap_or_else(|| DEFAULT_SYSTEM_IDENTIFIER.to_string()),
        git_commit: git_commit
            .or_else(|| env::var("GITHUB_SHA").ok())
            .unwrap_or_else(|| "unknown".to_string()),
    })
}
