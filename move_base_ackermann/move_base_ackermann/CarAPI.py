#!/usr/bin/env python3
import os
import time
from pathlib import Path
import atexit
import gpiod

def clamp(val, min_val, max_val):
    return max(min(max_val, val), min_val)

class GpioOut:
    def __init__(self, name):
        line = gpiod.find_line(name)
        assert line
        self.gpio = line.owner().get_lines([line.offset()])
        self.gpio.request('motor control', type=gpiod.LINE_REQ_DIR_OUT)

    def write(self, val):
        self.gpio.set_values([val])

class Pwm:
    def __init__(self, chip, num, period, initial_duty=None):
        self.period = period
        self.path = Path("/sys/class/pwm") / f"pwmchip{chip}" / f"pwm{num}"
        try:
            self._write("enable", 0)
        except OSError as e:
            # may fail if period and duty_cycle is not configured yet
            pass
        self._write("period", period)
        self.set_duty(initial_duty or 0)
        self._write("enable", 1)

    def _write(self, name, value):
        with open(self.path / name, "w") as f:
            f.write(str(value))

    def set_percent(self, percent):
        assert percent >= 0
        assert percent <= 100
        self.set_duty(round(percent / 100 * self.period))

    def set_duty(self, duty):
        assert duty >= 0
        assert duty < self.period
        self._write("duty_cycle", duty)

class CarApi:
    def __init__(self):
        self.servo_pwm_gpio = Pwm(chip=0, num=1, period=20 * 1000 * 1000, initial_duty=self.steering_angle_to_duty(0))
        self.actuator_en_gpio = GpioOut("SPI_MOSI")
        self.actuator_en_gpio.write(1)

        self.motor_max_perc = 20
        self.motor_pwm_gpio = Pwm(chip=0, num=0, period=20 * 1000 * 1000)
        self.motor_dir_gpio = GpioOut("GPIO6")
        self.motor_disable_gpio = GpioOut("GPIO19")

        atexit.register(self.stop)

    def stop(self):
        self.set_motor_power(0)
        self.actuator_en_gpio.write(0)

    def set_motor_power(self, speed):
        """-100 to +100"""
        perc = clamp(speed, -100, 100)
        self.motor_disable_gpio.write(perc == 0)
        self.motor_dir_gpio.write(perc < 0)
        self.motor_pwm_gpio.set_percent(abs(perc) / 100 * self.motor_max_perc)

    def steering_angle_to_duty(self, percent):
        perc = clamp(percent, -100, 100)
        servo_min = 800000
        servo_max = 1100000
        return round((servo_min + servo_max) / 2 + (servo_max - servo_min) / 2 * perc / 100)

    def set_steering_angle(self, percent):
        """-100 to +100"""
        self.servo_pwm_gpio.set_duty(self.steering_angle_to_duty(percent))

if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    subparsers = p.add_subparsers(dest='action', required=True)
    parser = subparsers.add_parser('set')
    parser.add_argument("--servo", help="Set servo position from -100% to 100%", type=int)
    parser.add_argument("--servo-duty", help="Set servo duty", type=int)
    parser.add_argument("--motor", help="Set motor speed from -100% to 100%", type=int)
    parser = subparsers.add_parser('test')
    parser.add_argument("--servo", action='store_true')
    parser.add_argument("--motor", action='store_true')

    args = p.parse_args()
    motor = CarApi()

    if args.action == 'test':
        test_all = not args.servo and not args.motor
        while True:
            t = [
                (0, 100),
                (100, -100),
                (-100, 0),
            ]
            if args.servo or test_all:
                for a, b in t:
                    for i in range(a, b + 1, 1 if a < b else -1):
                        print("servo:", i)
                        motor.set_steering_angle(i)
                        time.sleep(0.01)

            if args.motor or test_all:
                for a, b in t:
                    for i in range(a, b + 1, 1 if a < b else -1):
                        print("motor:", i)
                        motor.set_motor_power(i)
                        time.sleep(0.1)
    else:
        if args.servo:
            motor.set_steering_angle(args.servo)
        if args.servo_duty:
            motor.servo_pwm_gpio.set_duty(args.servo_duty)
        if args.motor:
            motor.set_motor_power(args.motor)
        input()
