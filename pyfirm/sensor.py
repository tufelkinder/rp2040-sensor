import machine
import uasyncio as asyncio
import utime
import queue
import json
from machine import UART, Pin

uart_c = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart_s = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))

# Settings
led1 = machine.Pin(9, machine.Pin.OUT)
led2 = machine.Pin(10, machine.Pin.OUT)

# setup:
# set adxl thresholds and activate

# task 1
# listen for adxl interrupts and report via uart_c

# task 2
# listen for uart_s interrupts, read, increment id, and report via uart_c

# task 3
# listen for uart_c incoming messages, update and pass along thresholds accordingly

