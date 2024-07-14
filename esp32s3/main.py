
from Wifi import Sta

#
#   The Robot's ESP32 (that this code is running on)
#       MAC Address:    b'4\x85\x18\xab\xf6P'
#                                 34:85:18:AB:F6:50
#
#   The Handheld ESP32
#       MAC Address:    b'4\x85\x18\xab\xf3\xec'
#                                 34:85:18:AB:F3:EC
#

import network
import espnow
import time

# A WLAN interface must be active to send()/recv()
sta = network.WLAN(network.STA_IF)  # Or network.AP_IF
sta.active(True)
# sta.disconnect()      # For ESP8266

e = espnow.ESPNow()
e.active(True)
peer = b'4\x85\x18\xab\xf3\xec'   # MAC address of peer's wifi interface
e.add_peer(peer)      # Must add_peer() before send()

print('Starting ESPNow test')

e.send(peer, "Starting...")
for i in range(100):
    e.send(peer, str(i)*20, True)
    time.sleep_ms(10)
e.send(peer, b'end')

print('Done ESPNow test')
