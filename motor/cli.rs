use std::time::Duration;

use car_controller::CarController;
use clap::{Parser, Subcommand};
use clap_repl::reedline::{
    DefaultPrompt, DefaultPromptSegment, FileBackedHistory, Reedline, Signal,
};
use clap_repl::ClapEditor;

#[derive(Debug, Parser)]
#[command(name = "")]
enum Command {
    /// Run
    Test,
    Stop,
    Steering {
        /// Angle (-100 to 100)
        #[arg(allow_hyphen_values = true)]
        angle: i8,
    },
    Motor {
        /// Speed (-100 to 100)
        #[arg(allow_hyphen_values = true)]
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

    let prompt = DefaultPrompt {
        left_prompt: DefaultPromptSegment::Basic("car".to_owned()),
        right_prompt: DefaultPromptSegment::Empty,
    };
    let rl = ClapEditor::<Command>::builder()
        .with_prompt(Box::new(prompt))
        .build();
    rl.repl(|cmd| match cmd {
        Command::Test => test(&controller),
        Command::Stop => controller.stop(),
        Command::Steering { angle } => controller.set_steering(angle),
        Command::Motor { speed } => controller.set_motor_power(speed),
    });
    Ok(())
}
