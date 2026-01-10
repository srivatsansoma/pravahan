import threading
import time
import logging
import math
from collections import deque

from pymavlink import mavutil
from dash import Dash, dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go


# ---------- MAVLink receiver (based on recv.py) ----------

# Serial ports: Cube (MAVLink) on ttyACM0, Arduino (distance) on ttyACM1
MAVLINK_DEVICE = "/dev/ttyACM3"  # Cube
DISTANCE_DEVICE = "/dev/ttyACM2"  # Arduino
BAUD_RATE = 115200

# Distance from center of mass to sensor (in mm) - adjust as needed
DISTANCE_COM_TO_SENSOR = 55.35  # Change this to your actual value

# Enable/disable console debugging of incoming MAVLink data
DEBUG_PRINT = True

# Store the last N seconds of data
# At ~160 Hz IMU rate, 8000 points ≈ 50 s of history
MAX_POINTS = 8000

data_lock = threading.Lock()
# IMU data: store initial time and use relative time (current - initial)
imu_initial_time = None  # Will be set when first IMU data arrives
imu_relative_times = deque(maxlen=MAX_POINTS)  # Relative time in seconds
roll_list = deque(maxlen=MAX_POINTS)
pitch_list = deque(maxlen=MAX_POINTS)
yaw_list = deque(maxlen=MAX_POINTS)
xacc_list = deque(maxlen=MAX_POINTS)
yacc_list = deque(maxlen=MAX_POINTS)
zacc_list = deque(maxlen=MAX_POINTS)

# Distance data: store time when received and value (step function - constant until next update)
distance_times = deque(maxlen=MAX_POINTS)  # Absolute time when received
distance_values = deque(maxlen=MAX_POINTS)  # Raw distance in mm
corrected_distance_values = deque(maxlen=MAX_POINTS)  # Corrected distance in mm


def distance_reader():
    """
    Background thread that reads distance messages from Arduino serial port.
    Reads text lines like "DIST:1234" from the Arduino connection.
    """
    import serial
    
    try:
        ser = serial.Serial(DISTANCE_DEVICE, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        print("Distance reader started on", DISTANCE_DEVICE)
    except Exception as e:
        print(f"ERROR: Could not open distance reader on {DISTANCE_DEVICE}: {e}")
        return

    buffer = ""
    while True:
        try:
            # Read available bytes
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting).decode(errors='ignore')
                buffer += data
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if line.startswith("DIST:"):
                        val = line.split(":")[1]
                        # Treat ERR as 0 and continue calculating
                        if val == "ERR":
                            distance_mm = 0
                        else:
                            try:
                                distance_mm = int(val)
                            except ValueError:
                                if DEBUG_PRINT:
                                    print(f"[DISTANCE] Error parsing distance value: {val}, using 0")
                                distance_mm = 0
                        
                        # Use device system time - log the moment it's received
                        now = time.time()
                        
                        with data_lock:
                            # Store time when received and value (step function - constant until next update)
                            distance_times.append(now)
                            distance_values.append(distance_mm)
                            
                            # Get latest pitch and roll for calculation (use most recent available)
                            if len(pitch_list) > 0 and len(roll_list) > 0:
                                pitch = pitch_list[-1]
                                roll = roll_list[-1]
                                
                                # Calculate: distance*cos(pitch)*cos(roll) - (distance_com_to_sensor)*sin(pitch)
                                corrected = (distance_mm * math.cos(pitch) * math.cos(roll) - 
                                            DISTANCE_COM_TO_SENSOR * math.sin(pitch))
                                corrected_distance_values.append(corrected)
                                
                                if DEBUG_PRINT:
                                    print(
                                        f"[DISTANCE] t={now:.3f} "
                                        f"raw={distance_mm/10:.1f}cm corrected={corrected/10:.2f}cm "
                                        f"(pitch={pitch:.4f} roll={roll:.4f})"
                                    )
                            else:
                                # No attitude data yet, store raw distance (will be corrected when attitude data arrives)
                                corrected_distance_values.append(float(distance_mm))
                                
                                if DEBUG_PRINT:
                                    print(f"[DISTANCE] t={now:.3f} raw={distance_mm/10:.1f}cm (no attitude data)")
            else:
                time.sleep(0.01)  # Small delay when no data
        except Exception as e:
            print(f"ERROR in distance reader: {e}")
            time.sleep(1)


def mavlink_reader():
    """
    Background thread that mirrors the behavior of recv.py:
    - Opens the MAVLink serial connection
    - Requests HIGHRES_IMU and ATTITUDE streams
    - Continuously reads and stores the latest values
    """
    try:
        master = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=BAUD_RATE)
        master.wait_heartbeat()
        print("heartbeat received from cube on", MAVLINK_DEVICE)
    except ModuleNotFoundError as e:
        # Typically raised when underlying 'serial' (pyserial) is missing
        print(
            "ERROR: Required serial backend not found. "
            "Install it with: pip install pyserial\n"
            f"Details: {e}"
        )
        return
    except Exception as e:
        print(f"ERROR: Failed to open MAVLink connection on {MAVLINK_DEVICE}: {e}")
        return

    # Request HIGHRES_IMU stream
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
        10000,  # µs → 400 Hz
        0,
        0,
        0,
        0,
        0,
    )

    # Request ATTITUDE stream
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        10000,  # µs → 400 Hz
        0,
        0,
        0,
        0,
        0,
    )

    while True:
        msg = master.recv_match(blocking=True)
        if msg is None:
            continue

        # Use device system time, not sensor timestamps
        now = time.time()

        with data_lock:
            # Set initial time on first IMU data
            global imu_initial_time
            if imu_initial_time is None:
                imu_initial_time = now
                if DEBUG_PRINT:
                    print(f"[IMU] Initial time set: {imu_initial_time:.3f}")
            
            # Calculate relative time (current - initial)
            relative_time = now - imu_initial_time
            
            msg_type = msg.get_type()
            if msg_type == "ATTITUDE":
                roll_list.append(msg.roll)
                pitch_list.append(msg.pitch)
                yaw_list.append(msg.yaw)
                imu_relative_times.append(relative_time)
                if DEBUG_PRINT:
                    print(
                        f"[ATTITUDE] t_rel={relative_time:.3f}s "
                        f"roll={msg.roll:.4f} pitch={msg.pitch:.4f} yaw={msg.yaw:.4f}"
                    )
            elif msg_type == "HIGHRES_IMU":
                xacc_list.append(msg.xacc)
                yacc_list.append(msg.yacc)
                zacc_list.append(msg.zacc)
                imu_relative_times.append(relative_time)
                if DEBUG_PRINT:
                    print(
                        f"[HIGHRES_IMU] t_rel={relative_time:.3f}s "
                        f"xacc={msg.xacc:.4f} yacc={msg.yacc:.4f} zacc={msg.zacc:.4f}"
                    )


# Start background MAVLink reader thread
reader_thread = threading.Thread(target=mavlink_reader, daemon=True)
reader_thread.start()

# Start background distance reader thread
distance_thread = threading.Thread(target=distance_reader, daemon=True)
distance_thread.start()


# ---------- Dash app ----------

app = Dash(__name__)
app.title = "AQ Dashboard"

# Reduce HTTP request logging noise from the underlying server
werk_log = logging.getLogger("werkzeug")
werk_log.setLevel(logging.ERROR)


def build_layout():
    return html.Div(
        style={
            "backgroundColor": "#0b1020",
            "minHeight": "100vh",
            "padding": "20px 40px",
            "fontFamily": "system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif",
            "color": "#f5f7ff",
        },
        children=[
            html.Div(
                style={"display": "flex", "justifyContent": "space-between", "alignItems": "center"},
                children=[
                    html.Div(
                        children=[
                            html.H1(
                                "AQ IMU Live Dashboard",
                                style={
                                    "margin": 0,
                                    "fontWeight": 700,
                                    "fontSize": "28px",
                                },
                            ),
                            html.Div(
                                "Live accelerometer and attitude data from Raspberry Pi",
                                style={
                                    "opacity": 0.75,
                                    "marginTop": "4px",
                                    "fontSize": "13px",
                                },
                            ),
                        ]
                    ),
                    html.Div(
                        style={
                            "display": "flex",
                            "gap": "12px",
                            "alignItems": "center",
                        },
                        children=[
                            html.Label(
                                "Time window (seconds):",
                                style={"fontSize": "12px", "opacity": 0.7},
                            ),
                            dcc.Slider(
                                id="time-window-slider",
                                min=5,
                                max=60,
                                step=5,
                                value=20,
                                marks={
                                    5: "5",
                                    20: "20",
                                    40: "40",
                                    60: "60",
                                },
                                tooltip={"placement": "bottom", "always_visible": False},
                                updatemode="drag",
                            ),
                        ],
                    ),
                ],
            ),
            html.Hr(style={"borderColor": "#252b3d"}),
            html.Div(
                style={
                    "display": "grid",
                    "gridTemplateColumns": "1fr",
                    "gridTemplateRows": "1fr 1fr 1fr 1fr",
                    "gap": "18px",
                },
                children=[
                    html.Div(
                        style={
                            "backgroundColor": "#14192b",
                            "borderRadius": "14px",
                            "padding": "14px 16px 8px",
                            "boxShadow": "0 12px 25px rgba(0,0,0,0.35)",
                        },
                        children=[
                            html.Div(
                                "Linear Acceleration (m/s²)",
                                style={
                                    "fontSize": "14px",
                                    "fontWeight": 600,
                                    "marginBottom": "4px",
                                },
                            ),
                            html.Div(
                                "X / Y / Z accelerations over time",
                                style={"fontSize": "11px", "opacity": 0.65},
                            ),
                            dcc.Graph(
                                id="acc-graph",
                                config={
                                    "displaylogo": False,
                                    "modeBarButtonsToRemove": [
                                        "lasso2d",
                                        "select2d",
                                        "autoScale2d",
                                    ],
                                },
                                style={"height": "42vh"},
                            ),
                        ],
                    ),
                    html.Div(
                        style={
                            "backgroundColor": "#14192b",
                            "borderRadius": "14px",
                            "padding": "14px 16px 8px",
                            "boxShadow": "0 12px 25px rgba(0,0,0,0.35)",
                        },
                        children=[
                            html.Div(
                                "Attitude Angles (rad)",
                                style={
                                    "fontSize": "14px",
                                    "fontWeight": 600,
                                    "marginBottom": "4px",
                                },
                            ),
                            html.Div(
                                "Roll / Pitch / Yaw over time",
                                style={"fontSize": "11px", "opacity": 0.65},
                            ),
                            dcc.Graph(
                                id="att-graph",
                                config={
                                    "displaylogo": False,
                                    "modeBarButtonsToRemove": [
                                        "lasso2d",
                                        "select2d",
                                        "autoScale2d",
                                    ],
                                },
                                style={"height": "42vh"},
                            ),
                        ],
                    ),
                    html.Div(
                        style={
                            "backgroundColor": "#14192b",
                            "borderRadius": "14px",
                            "padding": "14px 16px 8px",
                            "boxShadow": "0 12px 25px rgba(0,0,0,0.35)",
                        },
                        children=[
                            html.Div(
                                "Original Distance (cm)",
                                style={
                                    "fontSize": "14px",
                                    "fontWeight": 600,
                                    "marginBottom": "4px",
                                },
                            ),
                            html.Div(
                                "Raw distance from sensor (step function - constant until next update)",
                                style={"fontSize": "11px", "opacity": 0.65, "marginBottom": "8px"},
                            ),
                            html.Div(
                                id="raw-distance-display",
                                children="-- cm",
                                style={
                                    "fontSize": "20px",
                                    "fontWeight": 600,
                                    "color": "#fbbf24",
                                    "marginBottom": "8px",
                                },
                            ),
                            dcc.Graph(
                                id="raw-dist-graph",
                                config={
                                    "displaylogo": False,
                                    "modeBarButtonsToRemove": [
                                        "lasso2d",
                                        "select2d",
                                        "autoScale2d",
                                    ],
                                },
                                style={"height": "42vh"},
                            ),
                        ],
                    ),
                    html.Div(
                        style={
                            "backgroundColor": "#14192b",
                            "borderRadius": "14px",
                            "padding": "14px 16px 8px",
                            "boxShadow": "0 12px 25px rgba(0,0,0,0.35)",
                        },
                        children=[
                            html.Div(
                                "Corrected Distance (cm)",
                                style={
                                    "fontSize": "14px",
                                    "fontWeight": 600,
                                    "marginBottom": "4px",
                                },
                            ),
                            html.Div(
                                "Distance corrected using pitch/roll compensation (step function)",
                                style={"fontSize": "11px", "opacity": 0.65, "marginBottom": "8px"},
                            ),
                            html.Div(
                                id="corrected-distance-display",
                                children="-- cm",
                                style={
                                    "fontSize": "20px",
                                    "fontWeight": 600,
                                    "color": "#10b981",
                                    "marginBottom": "8px",
                                },
                            ),
                            dcc.Graph(
                                id="corrected-dist-graph",
                                config={
                                    "displaylogo": False,
                                    "modeBarButtonsToRemove": [
                                        "lasso2d",
                                        "select2d",
                                        "autoScale2d",
                                    ],
                                },
                                style={"height": "42vh"},
                            ),
                        ],
                    ),
                ],
            ),
            dcc.Interval(id="update-interval", interval=50, n_intervals=0),  # 50ms for real-time updates
        ],
    )


app.layout = build_layout


def _get_windowed_data(window_sec: float):
    """Return data limited to the last window_sec seconds.
    IMU: uses relative time (current - initial)
    Distance: uses absolute time for step function plotting
    """
    with data_lock:
        # Get IMU data with relative times (already relative to initial time)
        imu_rel_times = list(imu_relative_times)
        r_list = list(roll_list)
        p_list = list(pitch_list)
        y_list = list(yaw_list)
        xa_list = list(xacc_list)
        ya_list = list(yacc_list)
        za_list = list(zacc_list)
        
        # Get distance data with absolute times (for step function)
        dist_times_abs = list(distance_times)
        dist_values = list(distance_values)
        corr_dist_values = list(corrected_distance_values)
        
        # Get initial IMU time for converting distance times to relative
        imu_init = imu_initial_time

    # Window IMU data by relative time
    if not imu_rel_times:
        imu_t_windowed = []
        xa_windowed = []
        ya_windowed = []
        za_windowed = []
        r_windowed = []
        p_windowed = []
        y_windowed = []
    else:
        cutoff_rel = imu_rel_times[-1] - window_sec
        imu_start_idx = 0
        for i, t in enumerate(imu_rel_times):
            if t >= cutoff_rel:
                imu_start_idx = i
                break
        
        imu_t_windowed = imu_rel_times[imu_start_idx:]
        # Align data lists to timestamps (they may have different lengths)
        xa_windowed = xa_list[-len(imu_t_windowed):] if len(xa_list) >= len(imu_t_windowed) else (xa_list if xa_list else [])
        ya_windowed = ya_list[-len(imu_t_windowed):] if len(ya_list) >= len(imu_t_windowed) else (ya_list if ya_list else [])
        za_windowed = za_list[-len(imu_t_windowed):] if len(za_list) >= len(imu_t_windowed) else (za_list if za_list else [])
        r_windowed = r_list[-len(imu_t_windowed):] if len(r_list) >= len(imu_t_windowed) else (r_list if r_list else [])
        p_windowed = p_list[-len(imu_t_windowed):] if len(p_list) >= len(imu_t_windowed) else (p_list if p_list else [])
        y_windowed = y_list[-len(imu_t_windowed):] if len(y_list) >= len(imu_t_windowed) else (y_list if y_list else [])
        
        # Ensure all lists have same length as timestamps (pad with last value if needed)
        def align_to_timestamps(data_list, target_len):
            if not data_list:
                return []
            if len(data_list) >= target_len:
                return data_list[-target_len:]
            # Pad with last value
            last_val = data_list[-1] if data_list else 0.0
            return [last_val] * (target_len - len(data_list)) + list(data_list)
        
        xa_windowed = align_to_timestamps(xa_windowed, len(imu_t_windowed))
        ya_windowed = align_to_timestamps(ya_windowed, len(imu_t_windowed))
        za_windowed = align_to_timestamps(za_windowed, len(imu_t_windowed))
        r_windowed = align_to_timestamps(r_windowed, len(imu_t_windowed))
        p_windowed = align_to_timestamps(p_windowed, len(imu_t_windowed))
        y_windowed = align_to_timestamps(y_windowed, len(imu_t_windowed))
    
    # Window distance data by absolute time, then convert to relative
    if imu_init and dist_times_abs:
        cutoff_abs = time.time() - window_sec
        dist_start_idx = 0
        for i, t in enumerate(dist_times_abs):
            if t >= cutoff_abs:
                dist_start_idx = i
                break
        
        dist_times_windowed = dist_times_abs[dist_start_idx:]
        dist_values_windowed = dist_values[dist_start_idx:] if len(dist_values) > dist_start_idx else []
        corr_dist_windowed = corr_dist_values[dist_start_idx:] if len(corr_dist_values) > dist_start_idx else []
        
        # Convert distance absolute times to relative (using IMU initial time)
        dist_times_rel = [t - imu_init for t in dist_times_windowed] if imu_init else []
    else:
        dist_times_rel = []
        dist_values_windowed = []
        corr_dist_windowed = []

    return (imu_t_windowed, xa_windowed, ya_windowed, za_windowed, r_windowed, p_windowed, y_windowed,
            dist_times_rel, dist_values_windowed, corr_dist_windowed)


@app.callback(
    Output("acc-graph", "figure"),
    Output("att-graph", "figure"),
    Output("raw-dist-graph", "figure"),
    Output("corrected-dist-graph", "figure"),
    Input("update-interval", "n_intervals"),
    Input("time-window-slider", "value"),
)
def update_graphs(_, window_sec):
    # Get data: IMU uses relative time, distance uses relative time (converted from absolute)
    imu_t_rel, xa, ya, za, r, p, y, dist_t_rel, dist_values, corr_dist_values = _get_windowed_data(float(window_sec))

    # Diagnostic output
    if DEBUG_PRINT:
        with data_lock:
            imu_count = len(imu_relative_times)
            dist_count = len(distance_values)
        print(f"[PLOT] IMU times: {len(imu_t_rel)}, X acc: {len(xa)}, Y acc: {len(ya)}, Z acc: {len(za)}, "
              f"Roll: {len(r)}, Pitch: {len(p)}, Yaw: {len(y)}, "
              f"Distance times: {len(dist_t_rel)}, Dist values: {len(dist_values)}, "
              f"Total IMU: {imu_count}, Total Dist: {dist_count}")

    # Create empty layout for graphs with no data
    empty_layout = go.Layout(
        template="plotly_dark",
        paper_bgcolor="#14192b",
        plot_bgcolor="#14192b",
        margin=dict(l=40, r=20, t=10, b=40),
        xaxis=dict(showgrid=False),
        yaxis=dict(showgrid=False),
    )

    # Helper function to create step function (constant until next update)
    def create_step_function(times, values):
        """Create step function data for distance plots - value constant until next update."""
        if not times or not values or len(times) != len(values):
            return [], []
        step_t = []
        step_v = []
        for i in range(len(times)):
            step_t.append(times[i])
            step_v.append(values[i])
            if i < len(times) - 1:
                step_t.append(times[i + 1] - 0.001)  # Just before next update
                step_v.append(values[i])
        return step_t, step_v

    # Get common time range for x-axis synchronization
    all_times = []
    if imu_t_rel:
        all_times.extend(imu_t_rel)
    if dist_t_rel:
        all_times.extend(dist_t_rel)
    
    if not all_times:
        return (go.Figure(layout=empty_layout), go.Figure(layout=empty_layout), 
                go.Figure(layout=empty_layout), go.Figure(layout=empty_layout))
    
    t_min = min(all_times)
    t_max = max(all_times)
    x_range = [t_min, t_max] if t_max > t_min else [0, 1]
    

    # Helper function to calculate dynamic y-axis range with padding
    def calc_y_range(data_lists, padding_factor=0.1):
        """Calculate y-axis range from multiple data lists with padding."""
        all_values = []
        for data_list in data_lists:
            if data_list:
                all_values.extend(data_list)
        if not all_values:
            return None
        y_min = min(all_values)
        y_max = max(all_values)
        y_range = y_max - y_min
        if y_range == 0:
            # If all values are the same, add some padding
            y_min -= abs(y_min) * 0.1 if y_min != 0 else 1
            y_max += abs(y_max) * 0.1 if y_max != 0 else 1
        else:
            padding = y_range * padding_factor
            y_min -= padding
            y_max += padding
        return [y_min, y_max]

    # Acceleration graph - use IMU relative time directly
    acc_data = []
    if imu_t_rel and len(imu_t_rel) > 0:
        # Plot data if we have matching lengths or at least some data
        if xa and len(xa) > 0:
            # Use the shorter length to avoid index errors
            plot_len = min(len(imu_t_rel), len(xa))
            if plot_len > 0:
                acc_data.append(go.Scatter(x=imu_t_rel[-plot_len:], y=xa[-plot_len:], mode="lines", name="X", line=dict(color="#4fd1c5", width=2)))
        if ya and len(ya) > 0:
            plot_len = min(len(imu_t_rel), len(ya))
            if plot_len > 0:
                acc_data.append(go.Scatter(x=imu_t_rel[-plot_len:], y=ya[-plot_len:], mode="lines", name="Y", line=dict(color="#f6ad55", width=2)))
        if za and len(za) > 0:
            plot_len = min(len(imu_t_rel), len(za))
            if plot_len > 0:
                acc_data.append(go.Scatter(x=imu_t_rel[-plot_len:], y=za[-plot_len:], mode="lines", name="Z", line=dict(color="#fc8181", width=2)))
    
    acc_y_range = calc_y_range([xa, ya, za]) if (xa or ya or za) else None
    acc_fig = go.Figure(
        data=acc_data if acc_data else [
            go.Scatter(x=[], y=[], mode="lines", name="X", line=dict(color="#4fd1c5", width=2)),
            go.Scatter(x=[], y=[], mode="lines", name="Y", line=dict(color="#f6ad55", width=2)),
            go.Scatter(x=[], y=[], mode="lines", name="Z", line=dict(color="#fc8181", width=2)),
        ],
        layout=go.Layout(
            template="plotly_dark",
            paper_bgcolor="#14192b",
            plot_bgcolor="#14192b",
            margin=dict(l=40, r=20, t=10, b=40),
            legend=dict(
                orientation="h",
                yanchor="bottom",
                y=1.02,
                xanchor="right",
                x=1,
                font=dict(size=10),
            ),
            xaxis=dict(
                title="Time (s)",
                showgrid=True,
                gridcolor="rgba(255,255,255,0.04)",
                range=x_range,
            ),
            yaxis=dict(
                title="Acceleration (m/s²)",
                showgrid=True,
                gridcolor="rgba(255,255,255,0.04)",
                range=acc_y_range,
                autorange=False if acc_y_range else True,
            ),
        ),
    )

    # Attitude graph - use IMU relative time directly
    att_data = []
    if imu_t_rel and len(imu_t_rel) > 0:
        # Plot data if we have matching lengths or at least some data
        if r and len(r) > 0:
            plot_len = min(len(imu_t_rel), len(r))
            if plot_len > 0:
                att_data.append(go.Scatter(x=imu_t_rel[-plot_len:], y=r[-plot_len:], mode="lines", name="Roll", line=dict(color="#63b3ed", width=2)))
        if p and len(p) > 0:
            plot_len = min(len(imu_t_rel), len(p))
            if plot_len > 0:
                att_data.append(go.Scatter(x=imu_t_rel[-plot_len:], y=p[-plot_len:], mode="lines", name="Pitch", line=dict(color="#b794f4", width=2)))
        if y and len(y) > 0:
            plot_len = min(len(imu_t_rel), len(y))
            if plot_len > 0:
                att_data.append(go.Scatter(x=imu_t_rel[-plot_len:], y=y[-plot_len:], mode="lines", name="Yaw", line=dict(color="#9ae6b4", width=2)))
    
    att_y_range = calc_y_range([r, p, y]) if (r or p or y) else None
    att_fig = go.Figure(
        data=att_data if att_data else [
            go.Scatter(x=[], y=[], mode="lines", name="Roll", line=dict(color="#63b3ed", width=2)),
            go.Scatter(x=[], y=[], mode="lines", name="Pitch", line=dict(color="#b794f4", width=2)),
            go.Scatter(x=[], y=[], mode="lines", name="Yaw", line=dict(color="#9ae6b4", width=2)),
        ],
        layout=go.Layout(
            template="plotly_dark",
            paper_bgcolor="#14192b",
            plot_bgcolor="#14192b",
            margin=dict(l=40, r=20, t=10, b=40),
            legend=dict(
                orientation="h",
                yanchor="bottom",
                y=1.02,
                xanchor="right",
                x=1,
                font=dict(size=10),
            ),
            xaxis=dict(
                title="Time (s)",
                showgrid=True,
                gridcolor="rgba(255,255,255,0.04)",
                range=x_range,
            ),
            yaxis=dict(
                title="Angle (rad)",
                showgrid=True,
                gridcolor="rgba(255,255,255,0.04)",
                range=att_y_range,
                autorange=False if att_y_range else True,
            ),
        ),
    )

    # Original Distance graph - step function (constant until next update)
    dist_cm = [d / 10.0 for d in dist_values] if dist_values else []
    dist_step_t, dist_step_v = create_step_function(dist_t_rel, dist_cm) if dist_t_rel and dist_cm else ([], [])
    
    raw_dist_y_range = calc_y_range([dist_cm]) if dist_cm else None
    raw_dist_fig = go.Figure(
        data=[go.Scatter(x=dist_step_t, y=dist_step_v, mode="lines", name="Original Distance",
                        line=dict(color="#fbbf24", width=2))] if dist_step_t else [
            go.Scatter(x=[], y=[], mode="lines", name="Original Distance", line=dict(color="#fbbf24", width=2))
        ],
        layout=go.Layout(
            template="plotly_dark",
            paper_bgcolor="#14192b",
            plot_bgcolor="#14192b",
            margin=dict(l=40, r=20, t=10, b=40),
            legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1, font=dict(size=10)),
            xaxis=dict(title="Time (s)", showgrid=True, gridcolor="rgba(255,255,255,0.04)", range=x_range),
            yaxis=dict(title="Distance (cm)", showgrid=True, gridcolor="rgba(255,255,255,0.04)",
                      range=raw_dist_y_range, autorange=False if raw_dist_y_range else True),
        ),
    )

    # Corrected Distance graph - step function (constant until next update)
    corr_dist_cm = [d / 10.0 for d in corr_dist_values] if corr_dist_values else []
    corr_dist_step_t, corr_dist_step_v = create_step_function(dist_t_rel, corr_dist_cm) if dist_t_rel and corr_dist_cm else ([], [])
    
    corr_dist_y_range = calc_y_range([corr_dist_cm]) if corr_dist_cm else None
    corr_dist_fig = go.Figure(
        data=[go.Scatter(x=corr_dist_step_t, y=corr_dist_step_v, mode="lines", name="Corrected Distance",
                        line=dict(color="#10b981", width=2))] if corr_dist_step_t else [
            go.Scatter(x=[], y=[], mode="lines", name="Corrected Distance", line=dict(color="#10b981", width=2))
        ],
        layout=go.Layout(
            template="plotly_dark",
            paper_bgcolor="#14192b",
            plot_bgcolor="#14192b",
            margin=dict(l=40, r=20, t=10, b=40),
            legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1, font=dict(size=10)),
            xaxis=dict(title="Time (s)", showgrid=True, gridcolor="rgba(255,255,255,0.04)", range=x_range),
            yaxis=dict(title="Distance (cm)", showgrid=True, gridcolor="rgba(255,255,255,0.04)",
                      range=corr_dist_y_range, autorange=False if corr_dist_y_range else True),
        ),
    )

    return acc_fig, att_fig, raw_dist_fig, corr_dist_fig


@app.callback(
    Output("raw-distance-display", "children"),
    Output("corrected-distance-display", "children"),
    Input("update-interval", "n_intervals"),
)
def update_distance_displays(_):
    """Update the numerical displays with the latest distance values."""
    with data_lock:
        # Get the most recent values
        raw_dist = distance_values[-1] if distance_values else None
        corr_dist = corrected_distance_values[-1] if corrected_distance_values else None
    
    # Format the displays
    if raw_dist is not None:
        raw_display = f"{raw_dist / 10.0:.2f} cm"
    else:
        raw_display = "-- cm"
    
    if corr_dist is not None:
        corr_display = f"{corr_dist / 10.0:.2f} cm"
    else:
        corr_display = "-- cm"
    
    return raw_display, corr_display


if __name__ == "__main__":
    # Host on all interfaces so you can view it from another device if needed
    app.run(host="0.0.0.0", port=8050, debug=False)


