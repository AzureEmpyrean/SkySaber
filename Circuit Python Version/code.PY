# pylint: disable=bare-except

import time
import math
from digitalio import DigitalInOut, Direction, Pull
import audioio
import busio
import board
import neopixel
import adafruit_lis3dh
import gc

NUM_PIXELS = 85
# pins

# audio = audioio.AudioOut(board.A0)     # Speaker

LED_PIN = board.D5
LED_PIN2 = board.D6

BTN_PIN = board.D9
RGB_PIN = board.D10


while True:
    btns = neopixel.NeoPixel(LED_PIN2, 2, brightness=1, auto_write=False)
    btns.fill((0, 255, 0))                          # NeoPixels off ASAP on startup
    btns.show()

    strip = neopixel.NeoPixel(LED_PIN, NUM_PIXELS, brightness=1, auto_write=False)
    strip.fill((255, 0, 0))                          # NeoPixels off ASAP on startup
    strip.show()
