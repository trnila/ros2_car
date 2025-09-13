use gpiocdev::line::EdgeDetection;
use gpiocdev::line::EdgeEvent;
use gpiocdev::line::EdgeKind;
use gpiocdev::line::Value;
use gpiocdev::{FoundLine, Request};
use rclrs::{log_info, ToLogParams};
use std::env;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

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

    fn measure(&self) -> f64 {
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
                    let cm = time_diff as f64 * 1e-9 * 17150f64;
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
    let ultrasonic = Ultrasonic::new();

    let shut_down = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shut_down))?;

    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "minimal_publisher")?;

    let publisher =
        node.create_publisher::<std_msgs::msg::String>("topic", rclrs::QOS_PROFILE_DEFAULT)?;

    let mut message = std_msgs::msg::String::default();

    let mut publish_count: u32 = 1;

    while !shut_down.load(Ordering::Relaxed) && context.ok() {
        ultrasonic.measure();

        message.data = format!("Hello, world! {}", publish_count);
        //log_info!(node.logger(), "Publishing: {}", message.data);
        publisher.publish(&message)?;
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }

    Ok(())
}
