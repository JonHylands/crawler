#
#   Display Class
#
#   For the Waveshare 1.44" round cornered display
#
#   Uses the ST7789 driver (must be baked into the micropython build)
#

import st7789
from machine import SPI


custom_rotations = [
    (0x00, 172, 320, 34, 0),
    (0x60, 320, 172, 0, 34),
    (0xc0, 172, 320, 34, 0),
    (0xa0, 320, 172, 0, 34),
]


class Display:

    WHITE = st7789.WHITE
    BLACK = st7789.BLACK
    YELLOW = st7789.YELLOW
    BLUE = st7789.BLUE
    CYAN = st7789.CYAN
    GREEN = st7789.GREEN
    MAGENTA = st7789.MAGENTA
    RED = st7789.RED

    def __init__(self, spi_port, reset_pin, cs_pin, dc_pin):
        self.screen = st7789.ST7789(
            SPI(spi_port, baudrate=40000000),
            172, 320,
            reset=reset_pin,
            cs=cs_pin,
            dc=dc_pin,
            rotations=custom_rotations,
            rotation=1,
            options=0,
            buffer_size=0)
        self.screen.offset(35, 0)
        self.screen.init()

