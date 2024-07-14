
from Wifi import Sta
import time
import machine
import requests


wifi = Sta()
wifi.connect()

r = requests.get('http://worldtimeapi.org/api/timezone/America/Toronto')

time_string = r.json()['datetime']

#  '2024-02-19T17:56:24.993029-05:00'

parts = time_string.split('T')
date = parts[0]
time = parts[1].split('.')[0]

date_parts = date.split('-')
time_parts = time.split(':')

year = int(date_parts[0])
month = int(date_parts[1])
day = int(date_parts[2])

hour = int(time_parts[0])
minute = int(time_parts[1])
second = int(time_parts[2])

rtc = machine.RTC()

rtc.init((year, month, day, hour, minute, second,0,0))

print('')
print('Time: {}'.format(rtc.datetime()))
