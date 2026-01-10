import tkinter as tk
from tkinter import ttk
import ctypes
import mmap
import os


class data_shared(ctypes.Structure):
    _fields_ = [
        ("vertical_velocity", ctypes.c_float),
        ("throttle", ctypes.c_float),
        ("copy_now", ctypes.c_int),
    ]


class cpp_data_shared(ctypes.Structure):
    _fields_ = [
        ("time", ctypes.c_float),
        ("x", ctypes.c_float * 4),  # x[0],x[1] for heave, x[2],x[3] for pitch
        ("u_base", ctypes.c_float),
        ("u_diff_pitch", ctypes.c_float),
        ("ref", ctypes.c_float),  # Reference value for heave controller
        ("r", ctypes.c_float * 2),
        ("copy_now", ctypes.c_int),
    ]


class UnifiedDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("Control & Monitor Dashboard")
        self.root.geometry("900x700")
        self.root.configure(bg='#f5f5f5')
        
        # Initialize velocity tracking (matching C++ code)
        self.velocity = 2.0  # Start at 2 (matching C++ float velocity = 2;)
        self.acc_factor = 3.0  # Matching C++ acc_factor = 3
        self.dt = 0.1  # Time step matching C++ code (float dt = 0.1;)
        self.last_copy_now = 0  # Track when throttle data is available
        
        # Track which slider is being dragged
        self.dragging_slider = None
        
        # Initialize shared memory for sending controls
        try:
            self.fd_controls = os.open("/dev/shm/controls", os.O_RDWR | os.O_CREAT)
            os.ftruncate(self.fd_controls, ctypes.sizeof(data_shared))
            self.mm_controls = mmap.mmap(
                self.fd_controls, ctypes.sizeof(data_shared), mmap.MAP_SHARED, 
                mmap.PROT_READ | mmap.PROT_WRITE
            )
            self.controls_data = data_shared.from_buffer(self.mm_controls)
            self.controls_data.copy_now = 0
            self.controls_mem_available = True
            print("Successfully opened shared memory /dev/shm/controls")
        except (OSError, FileNotFoundError) as e:
            print(f"Warning: Could not open shared memory /dev/shm/controls: {e}")
            self.controls_mem_available = False
        
        # Initialize shared memory for reading from C++
        try:
            self.fd_cpp = os.open("/dev/shm/cpp_to_py", os.O_RDWR)
            file_size = os.fstat(self.fd_cpp).st_size
            struct_size = ctypes.sizeof(cpp_data_shared)
            if file_size < struct_size:
                os.ftruncate(self.fd_cpp, struct_size)
            self.mm_cpp = mmap.mmap(
                self.fd_cpp, struct_size, mmap.MAP_SHARED, 
                mmap.PROT_READ | mmap.PROT_WRITE
            )
            self.cpp_data = cpp_data_shared.from_buffer(self.mm_cpp)
            self.cpp_mem_available = True
            print("Successfully opened shared memory /dev/shm/cpp_to_py")
        except (OSError, FileNotFoundError) as e:
            print(f"Warning: Could not open shared memory /dev/shm/cpp_to_py: {e}")
            self.cpp_mem_available = False
        
        # Create UI
        self.create_ui()
        
        # Start update loops
        if self.controls_mem_available:
            self.update_controls()
        self.update_monitor()
        
        # Bind global mouse release
        root.bind('<ButtonRelease-1>', self.global_release)
    
    def create_ui(self):
        """Create the unified user interface"""
        # Main container with padding
        main_container = tk.Frame(self.root, bg='#f5f5f5', padx=20, pady=20)
        main_container.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title = tk.Label(
            main_container,
            text="Control & Monitor Dashboard",
            font=('Arial', 24, 'bold'),
            bg='#f5f5f5',
            fg='#2c3e50'
        )
        title.pack(pady=(0, 20))
        
        # Create two main sections side by side
        content_frame = tk.Frame(main_container, bg='#f5f5f5')
        content_frame.pack(fill=tk.BOTH, expand=True)
        
        # Left panel - Controls
        left_panel = tk.Frame(content_frame, bg='#ffffff', relief=tk.RAISED, bd=2)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # Right panel - Monitor
        right_panel = tk.Frame(content_frame, bg='#ffffff', relief=tk.RAISED, bd=2)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))
        
        # Create control section
        self.create_control_section(left_panel)
        
        # Create monitor section
        self.create_monitor_section(right_panel)
    
    def create_control_section(self, parent):
        """Create the control input section"""
        # Section title
        control_title = tk.Label(
            parent,
            text="Control Inputs",
            font=('Arial', 18, 'bold'),
            bg='#ffffff',
            fg='#34495e'
        )
        control_title.pack(pady=20)
        
        # Container for controls
        controls_container = tk.Frame(parent, bg='#ffffff', padx=30, pady=20)
        controls_container.pack(fill=tk.BOTH, expand=True)
        
        # Store slider values
        self.vertical_velocity_value = tk.DoubleVar(value=0.0001)
        self.throttle_value = tk.DoubleVar(value=0.0)
        
        # Vertical Velocity Control
        self.create_slider_control(
            controls_container,
            "Vertical Velocity",
            self.vertical_velocity_value,
            -1.0,
            1.0
        )
        
        # Throttle Command Control
        self.create_slider_control(
            controls_container,
            "Throttle Command",
            self.throttle_value,
            -1.0,
            1.0
        )
        
        # Reset button
        reset_btn = tk.Button(
            controls_container,
            text="Reset to Zero",
            command=self.reset_controls,
            font=('Arial', 12, 'bold'),
            bg='#e74c3c',
            fg='white',
            activebackground='#c0392b',
            activeforeground='white',
            relief=tk.FLAT,
            padx=30,
            pady=12,
            cursor='hand2'
        )
        reset_btn.pack(pady=30)
    
    def create_slider_control(self, parent, label_text, var, min_val, max_val):
        """Create a slider control"""
        # Container
        control_frame = tk.Frame(parent, bg='#ffffff')
        control_frame.pack(fill=tk.X, pady=20)
        
        # Label and value
        label_frame = tk.Frame(control_frame, bg='#ffffff')
        label_frame.pack(fill=tk.X, pady=(0, 10))
        
        label = tk.Label(
            label_frame,
            text=label_text,
            font=('Arial', 14, 'bold'),
            bg='#ffffff',
            fg='#2c3e50'
        )
        label.pack(side=tk.LEFT)
        
        value_label = tk.Label(
            label_frame,
            textvariable=var,
            font=('Courier', 12, 'bold'),
            bg='#ecf0f1',
            fg='#27ae60',
            relief=tk.SUNKEN,
            padx=15,
            pady=5,
            width=10
        )
        value_label.pack(side=tk.RIGHT)
        
        # Format value
        var.trace('w', lambda *args, v=var: v.set(round(v.get(), 2)))
        
        # Slider
        slider = tk.Scale(
            control_frame,
            from_=min_val,
            to=max_val,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            variable=var,
            length=300,
            bg='#ffffff',
            fg='#2c3e50',
            troughcolor='#ecf0f1',
            activebackground='#3498db',
            highlightthickness=0,
            font=('Arial', 10)
        )
        slider.pack(fill=tk.X)
        
        # Bind events
        def on_press(event):
            self.dragging_slider = var
        
        def on_release(event):
            if self.dragging_slider == var:
                var.set(0.0)
                self.dragging_slider = None
            return "break"
        
        slider.bind('<ButtonPress-1>', on_press)
        slider.bind('<ButtonRelease-1>', on_release)
        
        # Range labels
        range_frame = tk.Frame(control_frame, bg='#ffffff')
        range_frame.pack(fill=tk.X, pady=5)
        
        tk.Label(range_frame, text=f"{min_val:.1f}", font=('Arial', 9), 
                bg='#ffffff', fg='#7f8c8d').pack(side=tk.LEFT)
        tk.Label(range_frame, text="0.0", font=('Arial', 9), 
                bg='#ffffff', fg='#7f8c8d').pack(side=tk.LEFT, expand=True)
        tk.Label(range_frame, text=f"{max_val:.1f}", font=('Arial', 9), 
                bg='#ffffff', fg='#7f8c8d').pack(side=tk.RIGHT)
    
    def create_monitor_section(self, parent):
        """Create the monitoring section"""
        # Section title
        monitor_title = tk.Label(
            parent,
            text="System Monitor",
            font=('Arial', 18, 'bold'),
            bg='#ffffff',
            fg='#34495e'
        )
        monitor_title.pack(pady=20)
        
        # Container for monitor data
        monitor_container = tk.Frame(parent, bg='#ffffff', padx=30, pady=20)
        monitor_container.pack(fill=tk.BOTH, expand=True)
        
        # Velocity display
        self.create_value_display(monitor_container, "Velocity", "velocity_var", '#e8f5e9', '#2e7d32')
        
        # State displays
        self.create_state_section(monitor_container, "Heave Controller", ["x[0]", "x[1]"], [0, 1])
        self.create_state_section(monitor_container, "Pitch Controller", ["x[2]", "x[3]"], [2, 3])
        
        # Control inputs
        self.create_value_display(monitor_container, "u_base (Heave)", "u_base_var", '#e3f2fd', '#1976d2')
        self.create_value_display(monitor_container, "u_diff_pitch (Pitch)", "u_pitch_var", '#fce4ec', '#c2185b')
        
        # Reference value
        self.create_value_display(monitor_container, "ref (Heave Reference)", "ref_var", '#fff3e0', '#f57c00')
        
        # Status
        status_frame = tk.Frame(monitor_container, bg='#ffffff')
        status_frame.pack(fill=tk.X, pady=20)
        
        tk.Label(status_frame, text="Status:", font=('Arial', 12), 
                bg='#ffffff', fg='#7f8c8d').pack(side=tk.LEFT)
        
        self.status_var = tk.StringVar(value="Initializing...")
        status_label = tk.Label(
            status_frame,
            textvariable=self.status_var,
            font=('Arial', 11, 'bold'),
            bg='#ffffff',
            fg='#e67e22'
        )
        status_label.pack(side=tk.LEFT, padx=10)
    
    def create_value_display(self, parent, label, var_name, bg_color, fg_color):
        """Create a simple value display"""
        frame = tk.Frame(parent, bg='#ffffff')
        frame.pack(fill=tk.X, pady=8)
        
        tk.Label(frame, text=f"{label}:", font=('Arial', 11), 
                bg='#ffffff', fg='#2c3e50', width=20, anchor='w').pack(side=tk.LEFT)
        
        var = tk.StringVar(value="0.00")
        setattr(self, var_name, var)
        
        tk.Label(frame, textvariable=var, font=('Courier', 12, 'bold'),
                bg=bg_color, fg=fg_color, relief=tk.SUNKEN, padx=15, pady=5,
                width=15, anchor='e').pack(side=tk.RIGHT)
    
    def create_state_section(self, parent, title, labels, indices):
        """Create a state display section"""
        section_frame = tk.Frame(parent, bg='#f8f9fa', relief=tk.RAISED, bd=1)
        section_frame.pack(fill=tk.X, pady=10)
        
        tk.Label(section_frame, text=title, font=('Arial', 12, 'bold'),
                bg='#f8f9fa', fg='#2c3e50').pack(pady=8)
        
        values_frame = tk.Frame(section_frame, bg='#f8f9fa')
        values_frame.pack(fill=tk.X, padx=15, pady=8)
        
        if not hasattr(self, 'state_vars'):
            self.state_vars = {}
        
        for label, idx in zip(labels, indices):
            row = tk.Frame(values_frame, bg='#f8f9fa')
            row.pack(fill=tk.X, pady=3)
            
            tk.Label(row, text=f"{label}:", font=('Arial', 10),
                    bg='#f8f9fa', fg='#7f8c8d', width=8, anchor='w').pack(side=tk.LEFT)
            
            var = tk.StringVar(value="0.0000")
            self.state_vars[idx] = var
            
            tk.Label(row, textvariable=var, font=('Courier', 11, 'bold'),
                    bg='#ffffff', fg='#2c3e50', relief=tk.SUNKEN, padx=12, pady=4,
                    width=12, anchor='e').pack(side=tk.RIGHT)
    
    def reset_controls(self):
        """Reset all controls to zero"""
        self.vertical_velocity_value.set(0.0)
        self.throttle_value.set(0.0)
    
    def global_release(self, event):
        """Handle global mouse release"""
        if self.dragging_slider is not None:
            self.dragging_slider.set(0.0)
            self.dragging_slider = None
    
    def update_controls(self):
        """Update control outputs to shared memory"""
        if not self.controls_mem_available:
            return
        
        try:
            if self.controls_data.copy_now == 0:
                throttle_value = self.throttle_value.get()
                self.controls_data.vertical_velocity = ctypes.c_float(
                    self.vertical_velocity_value.get()
                )
                self.controls_data.throttle = ctypes.c_float(throttle_value)
                self.controls_data.copy_now = 1
                
                # Update velocity when throttle data is sent (matching C++ behavior)
                # C++ code: if (throttle_controls->copy_now){
                #              velocity += throttle_controls->throttle * acc_factor * dt;
                #          }
                # We update here when we send the data, matching when C++ would read it
                self.velocity += throttle_value * self.acc_factor * self.dt
        except Exception as e:
            print(f"Error updating controls: {e}")
        
        self.root.after(5, self.update_controls)
    
    def update_monitor(self):
        """Update monitor displays from shared memory"""
        try:
            # Update velocity display (velocity is updated in update_controls when data is sent)
            if hasattr(self, 'velocity_var'):
                self.velocity_var.set(f"{self.velocity:.2f}")
            
            # Update C++ data displays - always show current values
            if self.cpp_mem_available:
                try:
                    # Always update state values (regardless of copy_now)
                    if hasattr(self, 'state_vars'):
                        if 0 in self.state_vars:
                            self.state_vars[0].set(f"{self.cpp_data.x[0]:.4f}")
                        if 1 in self.state_vars:
                            self.state_vars[1].set(f"{self.cpp_data.x[1]:.4f}")
                        if 2 in self.state_vars:
                            self.state_vars[2].set(f"{self.cpp_data.x[2]:.4f}")
                        if 3 in self.state_vars:
                            self.state_vars[3].set(f"{self.cpp_data.x[3]:.4f}")
                    
                    # Always update control inputs
                    self.u_base_var.set(f"{self.cpp_data.u_base:.4f}")
                    self.u_pitch_var.set(f"{self.cpp_data.u_diff_pitch:.4f}")
                    
                    # Update reference value
                    if hasattr(self, 'ref_var'):
                        self.ref_var.set(f"{self.cpp_data.ref:.4f}")
                    
                    # Update status and handle copy_now
                    # Since copy_now logic in C++ has race conditions, we'll work around it
                    # by always displaying data and only resetting copy_now when it's actually 1
                    
                    # Check if data looks valid (non-zero values suggest data is being written)
                    has_data = (abs(self.cpp_data.x[0]) > 0.001 or 
                               abs(self.cpp_data.x[1]) > 0.001 or
                               abs(self.cpp_data.x[2]) > 0.001 or
                               abs(self.cpp_data.x[3]) > 0.001 or
                               abs(self.cpp_data.u_base) > 0.001 or
                               abs(self.cpp_data.u_diff_pitch) > 0.001)
                    
                    if self.cpp_data.copy_now == 1:
                        self.status_var.set(f"✓ Data received")
                        # Reset copy_now to signal data was read (only when it's actually 1)
                        self.cpp_data.copy_now = 0
                    elif has_data:
                        # Data is present but copy_now is 0 - C++ race condition issue
                        # Just show that we're receiving data
                        self.status_var.set(f"✓ Receiving data (time: {self.cpp_data.time:.2f}s)")
                    else:
                        self.status_var.set("Waiting for data...")
                except Exception as e:
                    self.status_var.set(f"Error: {e}")
                    print(f"Error updating monitor: {e}")
                    import traceback
                    traceback.print_exc()
            else:
                self.status_var.set("C++ memory not available")
        except Exception as e:
            print(f"Error in update_monitor: {e}")
        
        self.root.after(10, self.update_monitor)


if __name__ == "__main__":
    root = tk.Tk()
    app = UnifiedDashboard(root)
    root.mainloop()
