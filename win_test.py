import serial
import json

ser = serial.Serial(
        port='/dev/ttyS0', #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = 115200
)

data = []

def compare(cur, last):
	result = 'No motion'
	return result

while True:
	data = ser.readline()
	print(data)
	l = data.encode('utf-8').replace('\r', '').replace('\n', '').replace(': ', '": "').replace(', ', '", "').replace('{', '{"').replace('}', '"}')
	print(l)
	last_read = data[-1]
	compare(l, last_read)
	data.append(json.loads(l))



