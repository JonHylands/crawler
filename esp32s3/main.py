
from telemetry import Telemetry
from machine import UART, Pin
import machine
import time
import json


#================================================
#
#       Class HeartbeatLED
#

class HeartbeatLED:

    def __init__(self, ledPin):
        self.led = ledPin
        self.led.value(1)
        self.timer = Metro(0)
        self.set(100, 900)

    def set(self, newOnInterval, newOffInterval):
        self.onInterval = newOnInterval
        self.offInterval = newOffInterval
        self.timer.setInterval(self.offInterval)
        self.ledState = 0
        self.led.value(1)

    def update(self):
        if self.timer.check():
            if self.ledState:
                self.ledState = 0
                self.led.value(1)
                if self.onInterval != self.offInterval:
                    self.timer.setInterval(self.offInterval)
            else:
                self.ledState = 1
                self.led.value(0)
                if self.onInterval != self.offInterval:
                    self.timer.setInterval(self.onInterval)

    def shutdown(self):
        self.led.value(0)
    

class Metro:
  def __init__(self, interval_millis):
    self.interval = interval_millis
    self.previous = time.ticks_ms()

  def setInterval(self, interval_millis):
    self.interval = interval_millis

  def check(self):
    now = time.ticks_ms()
    if self.interval == 0:
      self.previous = now
      return True
    if (now - self.previous) >= self.interval:
      self.previous = now
      return True
    return False

  def reset(self):
    self.previous = time.ticks_ms()


class RobotComms:
    def __init__(self):
        self.comms = Telemetry()
        self.comms.register_telemetry_callback(self.command_callback)
        self.robot_port = UART(1, baudrate=1000000, tx=43, rx=44, rxbuf=1000, timeout=1000)

    def command_callback(self, packet):
        print('Got command from URC: {}'.format(packet))
        packet_bytes = json.dumps(packet)
        print('Length: {}'.format(len(packet_bytes)))
        self.robot_port.write(packet_bytes)
        self.robot_port.write('\n')

    def update(self):
        if self.robot_port.any():
            time.sleep_ms(10)
            bytes = self.robot_port.readline()
            print('Got command from robot: {}'.format(bytes))
            self.comms.send_packet(bytes)


print('URC Telemetry on Robot (ESP32)')

led_pin = Pin(21, Pin.OUT)
led = HeartbeatLED(led_pin)

rc = RobotComms()
while True:
    led.update()
    rc.update()
    time.sleep_ms(1)
