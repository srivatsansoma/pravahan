import serial
import time
from collections import deque
import matplotlib.pyplot as plt
import numpy
import threading

serial_port = serial.Serial(
	"/dev/ttyACM0",
	115200,
	timeout=5
)

is_running = True

radio = deque(maxlen=300)
distance = deque(maxlen=300)

start_time = time.time()

lps_ratio = 0.9

def collect_data():
	while(is_running):
		
		if serial_port.in_waiting > 0:
			line = serial_port.readline().decode("utf-8")
			if line[0:2] == "//":
				line = line[2:-4]
				signals = [int(x) for x in line.split()]
				radio.append([signals, time.time() - start_time])
				print(signals)
			else:
				distance.append([int(line). time.time() - start_time])

def plot_data():
	data.set_xdata([i[1] for i in distance])
	data.set_ydata([i[0] for i in distance])
	plt.draw()
	time.sleep(0.1)

if __name__ == "__main__":
	collect_data_thread = threading.Thread(target = collect_data)

	collect_data_thread.start()
	collect_data_thread.join()
