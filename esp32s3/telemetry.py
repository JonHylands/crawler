
import network
import espnow
import json
from binascii import hexlify


class Telemetry:
    def __init__(self):
        # A WLAN interface must be active to send()/recv()
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)

        print('MAC Address:')
        print(self.wlan.config('mac'))
        print(hexlify(self.wlan.config('mac'),':').decode())

        self.esp_now = espnow.ESPNow()
        self.esp_now.active(True)
        self.esp_now.irq(self.receive_callback)

        self.telemetry_callback = None
        self.return_mac = None

    def register_telemetry_callback(self, callback):
        self.telemetry_callback = callback

    # Packets are JSON strings
    def process_packet(self, packet_bytes):
        print('got telemetry: {}'.format(packet_bytes))
        packet = json.loads(packet_bytes)
        if self.telemetry_callback is not None:
            self.telemetry_callback(packet)

    def send_packet(self, packet_bytes):
        if self.return_mac is not None:
            print('Telemetry - sending response to {}'.format(self.return_mac))
            self.esp_now.send(self.return_mac, packet_bytes, True)
        else:
            print('Telemetry - no return mac')

    def receive_callback(self, interface):
        while True:
            mac, msg = interface.recv(0)  # 0 timeout == no waiting
            if mac is None:
                return
            if self.return_mac is None:
                self.return_mac = mac
                self.esp_now.add_peer(self.return_mac)
            self.process_packet(msg)

    def shutdown(self):
        self.esp_now.active(False)
        self.wlan.active(False)
