
from telemetry import Telemetry
from machine import UART, Pin
import machine
import time
import json


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

led = Pin(21, Pin.OUT)
led.off()

led_value = 0
led_metro = Metro(250)

rc = RobotComms()
try:
    while True:
        if led_metro.check():
            led.value(led_value)
            led_value = led_value ^ 1
        rc.update()
        time.sleep_ms(1)
except:
   machine.reset()
