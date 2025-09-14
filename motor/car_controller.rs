use gpiocdev::line::Value;
use gpiocdev::line::Values;
use gpiocdev::{FoundLine, Request};
use sysfs_pwm::Pwm;

fn steering_angle_to_duty(percent: i32) -> u32 {
    let servo_min: i32 = 800000;
    let servo_max: i32 = 1100000;
    ((servo_min + servo_max) / 2 + (servo_max - servo_min) / 2 * percent.clamp(-100, 100) / 100)
        as u32
}

pub struct CarController {
    motor_dir_line: FoundLine,
    motor_disable_line: FoundLine,
    gpios: Request,
    motor_pwm: Pwm,
    servo_pwm: Pwm,
    period_pwm: u32,
    max_motor_speed: f32,
}

impl Default for CarController {
    fn default() -> Self {
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

        let configure_pwm = |pwm: &Pwm, zero: u32| {
            pwm.export().unwrap();
            pwm.enable(false).unwrap();
            pwm.set_period_ns(period_pwm).unwrap();
            pwm.set_duty_cycle_ns(zero).unwrap();
            pwm.enable(true).unwrap();
        };

        configure_pwm(&motor_pwm, 0);
        configure_pwm(&servo_pwm, steering_angle_to_duty(0));

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
}

impl CarController {
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
        self.motor_pwm.set_duty_cycle_ns(duty).unwrap();
    }

    /// angle: -100 to 100
    pub fn set_steering(&self, angle: i8) {
        let duty = steering_angle_to_duty(angle as i32);
        println!("{duty}");
        self.servo_pwm.set_duty_cycle_ns(duty).unwrap();
    }

    pub fn stop(&self) {
        self.set_motor_power(0);
        self.set_steering(0);
        println!("stopped");
    }
}
