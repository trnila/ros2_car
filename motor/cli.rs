use std::time::Duration;

use car_controller::CarController;
use clap::{Parser, Subcommand};
use reedline::{DefaultPrompt, DefaultPromptSegment, Reedline, Signal};

#[derive(Parser)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run
    Test,
    Stop,
    Steering {
        /// Angle (-100 to 100)
        angle: i8,
    },
    Motor {
        /// Speed (-100 to 100)
        speed: i8,
    },
}

fn test(controller: &CarController) {
    for value in (-100..=100).chain((-100..=100).rev()) {
        controller.set_motor_power(value);
        controller.set_steering(value);
        std::thread::sleep(Duration::from_millis(50));
    }
    controller.stop();
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let controller = CarController::new();

    let mut line_editor = Reedline::create();
    let prompt = DefaultPrompt::new(
        DefaultPromptSegment::Basic("car".to_string()),
        DefaultPromptSegment::Empty,
    );

    loop {
        let sig = line_editor.read_line(&prompt);
        match sig {
            Ok(Signal::Success(buffer)) => {
                let mut args = vec!["myapp"];
                args.extend(buffer.split_whitespace());
                let cmd: Result<Cli, _> = Parser::try_parse_from(args);
                match cmd {
                    Ok(cmd) => match cmd.command {
                        Commands::Test => test(&controller),
                        Commands::Stop => controller.stop(),
                        Commands::Steering { angle } => controller.set_steering(angle),
                        Commands::Motor { speed } => controller.set_motor_power(speed),
                    },
                    Err(err) => {
                        println!("{err}");
                    }
                }

                println!("We processed: {}", buffer);
            }
            Ok(Signal::CtrlD) | Ok(Signal::CtrlC) => {
                println!("\nAborted!");
                break;
            }
            x => {
                println!("Event: {:?}", x);
            }
        }
    }

    Ok(())
}
