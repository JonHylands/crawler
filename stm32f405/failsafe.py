
from machine import UART
import time
import json


class Failsafe:
    def __init__(self, owner_callback, port_number):
        self.owner_callback = owner_callback
        self.uart = UART(port_number, 1000000, rxbuf=1000)

    def update(self):
        if self.uart.any():
            time.sleep_ms(10)
            bytes = self.uart.readline()
            print('Got command from URC: {}'.format(bytes))
            try:
                packet = json.loads(bytes)
            except Exception as ex:
                print('Exception parsing json packet from URC: {}'.format(ex))
            else:
                self.owner_callback(packet)

    def send_packet(self, packet):
        byte_string = json.dumps(packet)
        self.uart.write(json.dumps(packet) + '\n')
