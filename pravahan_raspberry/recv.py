from pymavlink import mavutil
import time
import serial
import serial.tools.list_ports
import threading
from collections import deque
import math

# Filtering / state
prev_height_filt = None
prev_time = None

def interpolate_attitude(target_time):
    if len(attitude_data) < 2:
        return None

    for i in range(len(attitude_data) - 1):
        a0 = attitude_data[i]
        a1 = attitude_data[i + 1]

        if a0["time"] <= target_time <= a1["time"]:
            t0, t1 = a0["time"], a1["time"]
            w = (target_time - t0) / (t1 - t0)

            return {
                "roll":  a0["roll"]  + w * (a1["roll"]  - a0["roll"]),
                "pitch": a0["pitch"] + w * (a1["pitch"] - a0["pitch"]),
                "yaw":   a0["yaw"]   + w * (a1["yaw"]   - a0["yaw"])
            }

    return None


# Deques
attitude_data = deque(maxlen=1000)
gyro_data     = deque(maxlen=1000)
acc_data      = deque(maxlen=1000)
serial_data   = deque(maxlen=1000)

run_program = True

def mavlink_reader():
    try:
        master = mavutil.mavlink_connection(
            '/dev/ttyACM0',
            baud=115200
        )

        master.wait_heartbeat()
        print("heartbeat received")
    except Exception as e:
        print(f"ERROR: Failed to connect to MAVLink device: {e}")
        return

    # Request messages
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
        10000,
        0, 0, 0, 0, 0
    )

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        10000,
        0, 0, 0, 0, 0
    )

    while run_program:
        try:
            msg = master.recv_match(blocking=True)
            if msg is None:
                continue

            ts = time.time()

            mtype = msg.get_type()

            if mtype == "ATTITUDE":
                attitude_data.append({
                    "time": ts,
                    "roll": msg.roll,
                    "pitch": msg.pitch,
                    "yaw": msg.yaw
                })
                #print(f"[ATTITUDE] roll={msg.roll:.3f} pitch={msg.pitch:.3f} yaw={msg.yaw:.3f}")

            elif mtype == "HIGHRES_IMU":
                gyro_data.append({
                    "time": ts,
                    "gx": msg.xgyro,
                    "gy": msg.ygyro,
                    "gz": msg.zgyro
                })

                acc_data.append({
                    "time": ts,
                    "ax": msg.xacc,
                    "ay": msg.yacc,
                    "az": msg.zacc
                })
                #print(f"[HIGHRES_IMU] acc=({msg.xacc:.3f},{msg.yacc:.3f},{msg.zacc:.3f}) gyro=({msg.xgyro:.3f},{msg.ygyro:.3f},{msg.zgyro:.3f})")
        except Exception as e:
            print(f"ERROR in mavlink_reader: {e}")
            #time.sleep(1)


def serial_reader():
    try:
        ser = serial.Serial("/dev/ttyACM1", 115200, timeout=1)
        time.sleep(2)
        print("Serial reader connected to /dev/ttyACM1")
    except Exception as e:
        print(f"ERROR: Failed to connect to serial device /dev/ttyACM1: {e}")
        return

    while run_program:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            ts = time.time()

            if line.startswith("DIST:"):
                val = line.split(":")[1]
                if val != "ERR":
                    serial_data.append({
                        "time": ts,
                        "distance_mm": int(val)
                    })
                    #print(f"[DISTANCE] {val} mm")
                else:
                    #print(f"[DISTANCE] ERR received")
                    pass
        except Exception as e:
            print(f"ERROR in serial_reader: {e}")
            #time.sleep(1)

def get_latest_sensor_data():
    global prev_height_filt, prev_time

    # ---- SENSOR OFFSET (mm) ----
    x_s = 55.8     # forward
    y_s = 0.0
    z_s = 1.362    # down

    # ---- FILTERING ----
    alpha = 0.3    # low-pass factor

    if not serial_data or not gyro_data:
        return None

    dist = serial_data[-1]
    ts   = dist["time"]
    d_mm = dist["distance_mm"]

    # ---- OUTLIER REJECTION ----
    if d_mm < 40 or d_mm > 4000:
        return None

    att = interpolate_attitude(ts)
    if att is None:
        return None

    roll  = att["roll"]
    pitch = att["pitch"]

    x0 = 55.8   # mm (forward from COM)
    z0 = 0.5    # mm (down from COM)

    height_mm = (
        d_mm * math.cos(roll) * math.cos(pitch)
        + z0 * math.cos(roll) * math.cos(pitch)
        - x0 * math.sin(pitch)
    )
    # ---- LOW PASS FILTER ----
    if prev_height_filt is None:
        height_filt = height_mm
        vertical_vel = 0.0
    else:
        height_filt = alpha * height_mm + (1 - alpha) * prev_height_filt
        dt = ts - prev_time if prev_time else 0.0
        vertical_vel = (height_filt - prev_height_filt) / dt if dt > 0 else 0.0

    prev_height_filt = height_filt
    prev_time = ts

    print(
        f"h={height_filt:7.1f} mm | "
        f"vz={vertical_vel:6.1f} mm/s | "
        f"roll={roll:+.3f} pitch={pitch:+.3f}"
    )

    return height_filt, vertical_vel


t1 = threading.Thread(target=mavlink_reader, daemon=True)
t2 = threading.Thread(target=serial_reader, daemon=True)

t1.start()
t2.start()

while True:
    data = get_latest_sensor_data()
    time.sleep(0.25)

