import serial
import time
from collections import deque
import matplotlib.pyplot as plt
import numpy
import threading

serial_port = serial.Serial(
	baud_rate = 115200,
	port = "/dev/ttyACM0",
	timeout=5
)

is_running = True

radio = deque(maxlen=300)
distance = deque(maxlen=300)

start_time = time.time()

lps_ratio = 0.9

data = plt.plot([], [])

def collect_data():
	while(is_running):
		
		if serial_port.is_waiting() > 0:
			line = serial_port.readline()
			line = serial_port.readline()
			if line[0:2]=="/." and line[-2:]==".\\":
				line = line[2:-3]
				signal = []
				temp = ""
				for i in line:
					if i != " ":
						signal.append(int(temp))
					else:
						temp += i
				radio.append([signal, time.time()-start_time])

			else:
				if distance != []:
					distance.append(int(line)*lps_ratio + distance[-1]*(1-lps_ratio))

def plot_data():
	data.set_xdata([i[1] for i in distance])
	data.set_ydata([i[0] for i in distance])
	plt.draw()
	time.sleep(0.1)

if __name__ == "__main__":
	collect_data_thread = threading.Thread(collect_data)
	plot_data_thread = threading.Thread(plot_data)

	collect_data_thread.join()
	plot_data_thread.join()

	collect_data_thread.join()
	plot_data_thread.join()