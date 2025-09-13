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

struct CarController {
    motor_dir_line: FoundLine,
    motor_disable_line: FoundLine,
    gpios: Request,
    motor_pwm: Pwm,
    servo_pwm: Pwm,
    period_pwm: u32,
    max_motor_speed: f32,
}

impl CarController {
    pub fn new() -> Self {
        let motor_dir_line: FoundLine = gpiocdev::find_named_line("GPIO6").unwrap();
        let motor_disable_line = gpiocdev::find_named_line("GPIO19").unwrap();
        let actuator_en_gpio = gpiocdev::find_named_line("SPI_MOSI").unwrap();

        let gpios = gpiocdev::Request::builder()
            .with_consumer("motor_controller")
            .with_found_line(&motor_dir_line)
            .as_output(Value::Inactive)
            .with_found_line(&motor_disable_line)
            .as_output(Value::Inactive)
            .with_found_line(&actuator_en_gpio)
            .as_output(Value::Active)
            .request()
            .unwrap();

        let period_pwm = 20_000_000;
        let servo_pwm = Pwm::new(0, 1).unwrap();
        let motor_pwm = Pwm::new(0, 0).unwrap();

        let configure_pwm = |pwm: &Pwm| {
            pwm.export().unwrap();
            pwm.enable(false).unwrap();
            pwm.set_period_ns(period_pwm).unwrap();
            pwm.enable(true).unwrap();
        };

        configure_pwm(&motor_pwm);
        configure_pwm(&servo_pwm);

        CarController {
            motor_dir_line,
            motor_disable_line,
            gpios,
            motor_pwm,
            servo_pwm,
            period_pwm,
            max_motor_speed: 20.0,
        }
    }

    /// speed: -100 to 100
    pub fn set_motor_power(&self, speed: i8) {
        let percent = speed.clamp(-100, 100).abs() as f32 / 100.0 * self.max_motor_speed;
        let duty = (percent / 100.0 * self.period_pwm as f32) as u32;
        println!("{duty}");

        self.gpios
            .set_values(
                Values::default()
                    .set(self.motor_disable_line.info.offset, (speed == 0).into())
                    .set(self.motor_dir_line.info.offset, (speed < 0).into()),
            )
            .unwrap();
        self.motor_pwm.set_duty_cycle_ns(duty as u32).unwrap();
    }

    /// angle: -100 to 100
    pub fn set_steering(&self, angle: i8) {
        let duty = self.steering_angle_to_duty(angle as i32);
        println!("{duty}");
        self.servo_pwm.set_duty_cycle_ns(duty as u32).unwrap();
    }

    fn steering_angle_to_duty(&self, percent: i32) -> u32 {
        let servo_min: i32 = 800000;
        let servo_max: i32 = 1100000;
        ((servo_min + servo_max) / 2 + (servo_max - servo_min) / 2 * percent.clamp(-100, 100) / 100)
            as u32
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let controller = CarController::new();

    for value in (-100..=100).chain((-100..=100).rev()) {
        controller.set_motor_power(value);
        controller.set_steering(value);
        std::thread::sleep(Duration::from_millis(50));
    }

    Ok(())
}
