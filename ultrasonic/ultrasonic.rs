use gpiocdev::line::EdgeDetection;
use gpiocdev::line::EdgeEvent;
use gpiocdev::line::EdgeKind;
use gpiocdev::line::Value;
use gpiocdev::{FoundLine, Request};
use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

pub struct Ultrasonic {
    trigger_line: FoundLine,
    req: Request,
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
        thread::sleep(Duration::from_micros(10));
        self.req
            .set_value(self.trigger_line.info.offset, Value::Inactive)
            .unwrap();

        // wait for the start of ultrasound transmission
        let transmitted = self.wait_event(EdgeKind::Rising, Duration::from_millis(1));
        if let Some(transmitted) = transmitted {
            // wait when signals is returned back
            let received = self.wait_event(EdgeKind::Falling, Duration::from_millis(50));
            if let Some(received) = received {
                let time_diff = received.timestamp_ns - transmitted.timestamp_ns;
                let cm = time_diff as f32 * 1e-9 * 17150f32;
                return cm;
            }
        }
        f32::NAN
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
    while !shut_down.load(Ordering::Relaxed) && context.ok() {
        let distance_cm = ultrasonic.measure();

        message.range = distance_cm;

        //log_info!(node.logger(), "Publishing: {}", message.data);
        publisher.publish(&message)?;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    Ok(())
}
