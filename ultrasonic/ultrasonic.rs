use gpiocdev::line::EdgeDetection;
use gpiocdev::line::EdgeEvent;
use gpiocdev::line::EdgeKind;
use gpiocdev::line::Value;
use gpiocdev::{FoundLine, Request};
use rclrs::ToLogParams;
use rclrs::{log_error, log_warn};
use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use std::time::Instant;

pub struct Ultrasonic {
    trigger_line: FoundLine,
    req: Request,
}

pub enum MeasurementError {
    /// ultrasonic beam was not sent
    NoTrigger,
    /// ultrasonic beam was not received back
    NoEcho,
    /// Unexpected edge received
    WrongEdge,
}

enum EdgeError {
    Timeout,
    WrongEdge,
}

impl Default for Ultrasonic {
    fn default() -> Self {
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
}

impl Ultrasonic {
    fn wait_event(&self, kind: EdgeKind, timeout: Duration) -> Result<EdgeEvent, EdgeError> {
        if self.req.wait_edge_event(timeout).unwrap() {
            let event = self.req.read_edge_event().unwrap();
            if event.kind == kind {
                Ok(event)
            } else {
                Err(EdgeError::WrongEdge)
            }
        } else {
            Err(EdgeError::Timeout)
        }
    }

    fn measure(&self) -> Result<f32, MeasurementError> {
        // clear all echo events
        while self.req.has_edge_event().unwrap() {
            self.req.read_edge_event().unwrap();
        }

        // trigger ultrasonic beam
        self.req
            .set_value(self.trigger_line.info.offset, Value::Active)
            .unwrap();
        thread::sleep(Duration::from_micros(10));
        self.req
            .set_value(self.trigger_line.info.offset, Value::Inactive)
            .unwrap();

        // wait for the start of ultrasound transmission
        let transmitted = self
            .wait_event(EdgeKind::Rising, Duration::from_millis(1))
            .map_err(|err| match err {
                EdgeError::Timeout => MeasurementError::NoTrigger,
                EdgeError::WrongEdge => MeasurementError::WrongEdge,
            })?;
        // wait when signals is returned back
        let received = self
            .wait_event(EdgeKind::Falling, Duration::from_millis(45))
            .map_err(|err| match err {
                EdgeError::Timeout => MeasurementError::NoEcho,
                EdgeError::WrongEdge => MeasurementError::WrongEdge,
            })?;
        let time_diff = received.timestamp_ns - transmitted.timestamp_ns;
        let cm = time_diff as f32 * 1e-9 * 17150f32;
        Ok(cm)
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let shut_down = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shut_down))?;

    let context = rclrs::Context::new(env::args())?;
    let node = rclrs::create_node(&context, "ultrasonic")?;

    let publisher =
        node.create_publisher::<sensor_msgs::msg::Range>("topic", rclrs::QOS_PROFILE_DEFAULT)?;
    let mut message = sensor_msgs::msg::Range {
        radiation_type: sensor_msgs::msg::Range::ULTRASOUND,
        field_of_view: 0.0,
        min_range: 0.0,
        max_range: 450.0,
        ..Default::default()
    };

    let ultrasonic = Ultrasonic::default();
    let rate = Duration::from_millis(50);
    let mut next_tick = Instant::now();
    while !shut_down.load(Ordering::Relaxed) && context.ok() {
        let measurement = ultrasonic.measure();
        message.range = match measurement {
            Ok(cm) => cm,
            Err(MeasurementError::NoTrigger) => {
                log_error!(node.logger(), "No trigger pulse detected");
                f32::NAN
            }
            Err(MeasurementError::NoEcho) => {
                log_warn!(node.logger(), "No echo pulse received");
                f32::NAN
            }
            Err(MeasurementError::WrongEdge) => {
                log_error!(node.logger(), "Unexpected signal edge detected");
                f32::NAN
            }
        };
        publisher.publish(&message)?;

        next_tick += rate;
        let now = Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        } else {
            next_tick = now;
            log_warn!(node.logger(), "Ultrasonic measurement missed its deadline");
        }
    }

    Ok(())
}
