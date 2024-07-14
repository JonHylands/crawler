
import time

#================================================
#
#     Class Metro
#

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

