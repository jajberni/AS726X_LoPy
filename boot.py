from machine import UART
from network import WLAN
import os

uart = UART(0, 115200)
os.dupterm(uart)

wlan = WLAN() # get current object, without changing the mode
