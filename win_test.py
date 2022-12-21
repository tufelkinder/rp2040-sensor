import serial
import json


s = serial.Serial('COM4')
data = []

def compare(cur, last):
	result = 'No motion'
	return result

while True:
	data = s.read()
	print(data)
	l = data.encode('utf-8').replace('\r', '').replace('\n', '').replace(': ', '": "').replace(', ', '", "').replace('{', '{"').replace('}', '"}')
	print(l)
	last_read = data[-1]
	compare(l, last_read)
	data.append(json.loads(l))



