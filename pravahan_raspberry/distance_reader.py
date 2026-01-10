import serial
import time

ser = serial.Serial("/dev/ttyACM1", 115200, timeout=1)
time.sleep(2)

while True:
    line = ser.readline().decode(errors='ignore').strip()
    if line.startswith("DIST:"):
        val = line.split(":")[1]
        if val != "ERR":
            distance_mm = int(val)
            print(f"Distance: {distance_mm} mm ({distance_mm/10:.1f} cm)")
        else:
            print("Error reading distance")

