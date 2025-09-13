use gpiocdev::line::EdgeDetection;
use gpiocdev::line::EdgeEvent;
use gpiocdev::line::EdgeKind;
use gpiocdev::line::Value;
use gpiocdev::line::Values;
use gpiocdev::{FoundLine, Request};
//use rclrs::{log_info, ToLogParams};
use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use sysfs_pwm::Pwm;

/*
pub struct Ultrasonic {
    trigger_line: FoundLine,
    req: Request,
}

impl Ultrasonic {
    pub fn new() -> Self {
        let trigger_line = gpiocdev::find_named_line("GPIO23").unwrap();
        let echo_line = gpiocdev::find_named_line("GPIO24").unwrap();

        let req = gpiocdev::Request::builder()
            .with_consumer("ultrasonic")
            .with_found_line(&trigger_line)
            .as_output(Value::Inactive)
            .with_found_line(&echo_line)
            .as_input()
            .with_edge_detection(EdgeDetection::BothEdges)
            .request()
            .unwrap();

        Ultrasonic { trigger_line, req }
    }

    fn wait_event(&self, kind: EdgeKind, timeout: Duration) -> Option<EdgeEvent> {
        if self.req.wait_edge_event(timeout).unwrap() {
            let event = self.req.read_edge_event().unwrap();
            assert!(event.kind == kind);
            Some(event)
        } else {
            None
        }
    }

    fn measure(&self) -> f32 {
        // clear all echo events
        while self.req.has_edge_event().unwrap() {
            self.req.read_edge_event().unwrap();
        }

        // trigger ultrasonic beam
        self.req
            .set_value(self.trigger_line.info.offset, Value::Active)
            .unwrap();
        self.req
            .set_value(self.trigger_line.info.offset, Value::Inactive)
            .unwrap();

        // wait for the start
        let timeout = Duration::from_millis(1000);
        let transmitted = self.wait_event(EdgeKind::Rising, timeout);
        if let Some(transmitted) = transmitted {
            let received = self.wait_event(EdgeKind::Falling, timeout);
            match received {
                Some(received) => {
                    let time_diff = received.timestamp_ns - transmitted.timestamp_ns;
                    let cm = time_diff as f32 * 1e-9 * 17150f32;
                    println!("{transmitted:?} {received:?} {time_diff} {cm}cm");
                    return cm;
                }
                None => {}
            };
        } else {
        }

        0.0
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let shut_down = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shut_down))?;

    let context = rclrs::Context::new(env::args())?;
    let node = rclrs::create_node(&context, "ultrasonic")?;

    let publisher =
        node.create_publisher::<sensor_msgs::msg::Range>("topic", rclrs::QOS_PROFILE_DEFAULT)?;
    let mut message = sensor_msgs::msg::Range::default();
    message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    message.field_of_view = 0.0;
    message.min_range = 0.0;
    message.max_range = 30.0;

    let ultrasonic = Ultrasonic::new();
    while !shut_down.load(Ordering::Relaxed) && context.ok() {
        let distance_cm = ultrasonic.measure();

        message.range = distance_cm;

        //log_info!(node.logger(), "Publishing: {}", message.data);
        publisher.publish(&message)?;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    Ok(())
}

*/

fn steering_angle_to_duty(percent: i32) -> u32 {
    let servo_min: i32 = 800000;
    let servo_max: i32 = 1100000;
    ((servo_min + servo_max) / 2 + (servo_max - servo_min) / 2 * percent.clamp(-100, 100) / 100)
        as u32
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let motor_dir_line = gpiocdev::find_named_line("GPIO6").unwrap();
    let motor_disable_line = gpiocdev::find_named_line("GPIO19").unwrap();

    let actuator_en_gpio = gpiocdev::find_named_line("SPI_MOSI").unwrap();

    let req = gpiocdev::Request::builder()
        .with_consumer("motor")
        .with_found_line(&motor_dir_line)
        .as_output(Value::Inactive)
        .with_found_line(&motor_disable_line)
        .as_output(Value::Inactive)
        .with_found_line(&actuator_en_gpio)
        .as_output(Value::Active)
        .request()
        .unwrap();

    let period = 20_000_000;

    let servo_pwm = Pwm::new(0, 1).unwrap();
    servo_pwm
        .with_exported(|| {
            servo_pwm.enable(false).unwrap();
            servo_pwm.set_period_ns(period).unwrap();
            servo_pwm.enable(true).unwrap();

            let duty = steering_angle_to_duty(-100);
            println!("{duty}");
            servo_pwm.set_duty_cycle_ns(duty as u32).unwrap();

            loop {}
        })
        .unwrap();

    println!("fc");

    let motor_max_perc = 20f32;

    let motor_pwm = Pwm::new(0, 0).unwrap();
    motor_pwm
        .with_exported(|| {
            motor_pwm.enable(false).unwrap();
            motor_pwm.set_period_ns(period).unwrap();
            motor_pwm.enable(true).unwrap();

            let speed: i32 = 20;

            let percent = speed.clamp(-100, 100).abs() as f32 / 100.0 * motor_max_perc;
            let duty = percent / 100.0 * period as f32;
            println!("{duty}");

            req.set_values(
                Values::default()
                    .set(motor_disable_line.info.offset, (speed == 0).into())
                    .set(motor_dir_line.info.offset, (speed < 0).into()),
            )
            .unwrap();
            motor_pwm.set_duty_cycle_ns(duty as u32).unwrap();

            loop {}
        })
        .unwrap();
    Ok(())
}
