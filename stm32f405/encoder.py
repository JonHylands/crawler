#
#   Encoder Class
#
#   For standard quadrature encoder, hooked to two pins on an STM32F405
#
#   This encoder class uses Timer 5
#

import pyb
from machine import Pin


class Encoder:

    def __init__(self, a_pin, b_pin):
        self.pin_a = a_pin
        self.pin_b = b_pin
        self.pin_a.init(Pin.AF_PP, pull=Pin.PULL_NONE, alt=Pin.AF2_TIM5)
        self.pin_b.init(Pin.AF_PP, pull=Pin.PULL_NONE, alt=Pin.AF2_TIM5)
        # The prescaler needs to be 0. When incrementing, the counter will count up-to
        # and including the period value, and then reset to 0.
        self.timer = pyb.Timer(5, prescaler=0, period=0x7FFFFFFF)
        # ENC_AB will increment/decrement on the rising edge of either the A channel or the B
        # channel.
        self.channel = self.timer.channel(1, pyb.Timer.ENC_AB)

    def value(self):
        return self.timer.counter()
