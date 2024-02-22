import gpiod

class Ultrasonic:
    def __init__(self):
        self.line = gpiod.find_line("GPIO24")
        assert self.line
        trig = self.line.owner().get_lines([self.line.offset()])
        trig.request('SONIC trigger', type=gpiod.LINE_REQ_EV_BOTH_EDGES)

        line1 = gpiod.find_line("GPIO23")
        assert line1
        self.echo = line1.owner().get_lines([line1.offset()])
        self.echo.request('SONIC echo', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

    def edge_time(self, t):
        if not self.line.event_wait(sec=1):
            print(f"{t} expected")
            return
        evt = self.line.event_read()
        if evt.type != t:
            print(f"{t} expected")
            return

        return evt.sec + evt.nsec * 10**-9

    def measure(self):
        self.echo.set_values([1])
        self.echo.set_values([0])

        started = self.edge_time(gpiod.LineEvent.RISING_EDGE)
        ended = self.edge_time(gpiod.LineEvent.FALLING_EDGE)
        return (ended - started) * 17150


if __name__ == "__main__":
    import time

    sensor = Ultrasonic()
    while True:
        print(sensor.measure())
        time.sleep(.1)
