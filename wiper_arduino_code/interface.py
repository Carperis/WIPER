import serial
import threading
import tkinter as tk
import atexit
import time
import csv
import queue
import re
import os
import datetime
import numpy as np
import platform

from lqr import generate_reference_trajectory, RecedingTVLQRController, TVLQRController
from apriltag_tracker import AprilTagTracker
from shared_state import state as shared_state, state_lock as shared_state_lock


# Define global variables
if platform.system() == "Windows":                  # Windows
    SERIAL_PORT = "COM3"
else:
    SERIAL_PORT = "/dev/tty.usbserial-A1080DBC"     # Mac

BAUD_RATE = 9600
power = 0
mode = "default"
flag_terminate = False
state = shared_state
reference = {'x': 0.0, 'y': 0.0}

class SerialInterface:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=1)
        self.message_queue = queue.Queue()
        self.receive_thread_stop = threading.Event()
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        self.last_print_time = None
        self.log_filename = None
        self.last_power = 0
        self.log_folder = r"C:/Users/19536/OneDrive/Current Semester/16745 OCRL/WIPER/WIPER/Logs"
        os.makedirs(self.log_folder, exist_ok=True)
        atexit.register(self.close_serial)

    def send_message(self, message):
        print(f"[PYTHON → ARDUINO] {message.strip()}")  # Debug print
        self.serial_port.write(message.encode())

    def update_state(self, message):
        try:
            message = message.decode(errors='replace').replace('\r', '').replace('\x00', '').strip()
            if message.startswith("TEST") or not message:
                return

            values = message.split()
            if len(values) < 4:
                print("[WARNING] Not enough values in message")
                return

            ax_raw, ay_raw, az, dt = map(float, values[:4])
            acc_x = ay_raw
            acc_y = -ax_raw

            vx = acc_x * dt
            vy = acc_y * dt

            # Compute IMU-estimated fallback state
            imu_x = state.get('x', 0.0) + vx
            imu_y = state.get('y', 0.0) + vy
            imu_theta = np.degrees(np.arctan2(ay_raw, ax_raw)) % 360

            with shared_state_lock:
                cam_x = state.get('cam_x')
                cam_y = state.get('cam_y')
                cam_theta = state.get('cam_theta')
                cam_ts = state.get('cam_timestamp', 0)

            cam_age = time.time() - cam_ts

            if cam_age < 0.1 and all(np.isfinite(v) for v in [cam_x, cam_y, cam_theta]):
                with shared_state_lock:
                    state['x'] = cam_x
                    state['y'] = cam_y
                    state['theta'] = cam_theta
                source = "CAM"
            else:
                with shared_state_lock:
                    state['x'] = imu_x
                    state['y'] = imu_y
                    state['theta'] = imu_theta
                source = "IMU"
            print(f"[STATE] x: {state['x']:.3f}, y: {state['y']:.3f}, θ: {state['theta']:.2f} ({source}), "
                f"ax: {acc_x:.3f}, ay: {acc_y:.3f}, dt: {dt:.3f}")

        except Exception as e:
            print(f"[ERROR] Failed to update state: {e}")

    def receive_data(self):
        while not self.receive_thread_stop.is_set():
            if self.serial_port.in_waiting > 0:
                received_data = self.serial_port.readline()
                self.update_state(received_data)
                self.message_queue.put(received_data)

    def receive_message(self):
        try:
            return self.message_queue.get_nowait()
        except queue.Empty:
            return ""

    def close_serial(self):
        self.send_message("0,0\n")
        self.receive_thread_stop.set()
        self.receive_thread.join()
        if self.serial_port.is_open:
            self.serial_port.close()


class App:
    def __init__(self, master, serial_interface):
        self.master = master
        self.serial_interface = serial_interface
        self.previous_messages = []
        self.previous_received_messages = []
        self.current_message_index = -1
        self.data_queue = queue.Queue()  # Initialize the data_queue
        self.log_buffer = []  # Initialize the log buffer
        self.selected_mode = 1  # Default mode is 1

        # === Layout Start ===
        self.frame = tk.Frame(self.master)
        self.frame.pack(padx=20, pady=20)

        # === Top row: Mode + Power ===
        self.mode_label = tk.Label(self.frame, text="Select Mode:")
        self.mode_label.grid(row=0, column=0, sticky='w')

        self.mode_var = tk.StringVar(self.master)
        self.mode_var.set("1")
        self.mode_dropdown = tk.OptionMenu(self.frame, self.mode_var, "1", "2", "3", "4", command=self.update_mode)
        self.mode_dropdown.grid(row=0, column=1, sticky='w')

        self.power_on_button = tk.Button(self.frame, text="Power ON", bg="green", fg="white", command=self.send_power_on)
        self.power_on_button.grid(row=0, column=2, padx=10)

        self.power_off_button = tk.Button(self.frame, text="Power OFF", bg="red", fg="white", command=self.send_power_off)
        self.power_off_button.grid(row=0, column=3, padx=5)

        # === Message input ===
        self.message_label = tk.Label(self.frame, text="Message:")
        self.message_label.grid(row=1, column=0, sticky='w')

        self.message_entry = tk.Entry(self.frame, width=30)
        self.message_entry.grid(row=1, column=1, columnspan=2, sticky='w')
        self.message_entry.bind("<Return>", self.send_message_event)

        self.send_button = tk.Button(self.frame, text="Send", command=self.send_message)
        self.send_button.grid(row=1, column=3, padx=5)

        # === Message History ===
        self.message_history_label = tk.Label(self.frame, text="Message History:")
        self.message_history_label.grid(row=2, column=0, columnspan=4, sticky="w")

        self.message_history_text = tk.Text(self.frame, height=6, width=80)
        self.message_history_text.grid(row=3, column=0, columnspan=4)

        # === Received Messages ===
        self.received_message_label = tk.Label(self.frame, text="Received Message:")
        self.received_message_label.grid(row=4, column=0, columnspan=4, sticky="w")

        self.received_message_text = tk.Text(self.frame, height=10, width=120)
        self.received_message_text.grid(row=5, column=0, columnspan=4)

        # === Status at the bottom ===
        self.status_label = tk.Label(self.frame, text="Status:")
        self.status_label.grid(row=6, column=0, columnspan=4, sticky="w", pady=(10, 0))

        # Start the update checker
        self.check_for_updates()

        # Initialize the TVLQR controller
        self.lqr_controller = None  # Holds an active TVLQRController instance
        self.use_imu_heading = False  # Set True if using IMU for heading in future

    def send_power_on(self):
        global power
        power = 1
        print("[INFO] Power ON activated.")

    def send_power_off(self):
        global power
        power = 0
        print("[INFO] Power OFF activated.")
        self.serial_interface.send_message("0.0,0.0\n")  # stop motors just in case

    def update_mode(self, mode):
        """Update the selected mode for the trajectory."""
        self.selected_mode = int(mode)
        print(f"Mode updated to: {self.selected_mode}")

    def cmd_write_thread(self):
        global power, mode, flag_terminate, state, reference
        logging_active = False  # Flag to control logging state
        rpm_m1, rpm_m2 = 0.0, 0.0  # Always define defaults

        while not flag_terminate:
            # On power-on: initialize controller
            if power == 1 and not logging_active:
                logging_active = True
                print("[INFO] Power ON: Logging started.")
                start_state = (
                    state['x'],
                    state['y'],
                    state.get('theta', 0.0)
                )
                # OPTION 1: Traditional TVLQR
                # self.lqr_controller = TVLQRController(start=start_state, mode=self.selected_mode, N=50)

                # OPTION 2: Receding-Horizon TVLQR
                full_ref = generate_reference_trajectory(start_state, N=50, mode=self.selected_mode)
                self.lqr_controller = RecedingTVLQRController(full_ref_traj=full_ref, N=10)

            # On power-off: stop controller
            if power == 0 and logging_active:
                logging_active = False
                print("[INFO] Power OFF: Logging stopped.")
                self.lqr_controller = None

            # Run LQR controller if active
            if self.lqr_controller is not None:
                with shared_state_lock:
                    x_curr = np.array([
                        state['x'],
                        state['y'],
                        state.get('theta', 0.0)
                    ])
                rpm_m1, rpm_m2 = self.lqr_controller.get_control(x_curr)

                # Clamp RPM to avoid overspeeding
                # rpm_m1 = max(min(rpm_m1, 300), -300)
                # rpm_m2 = max(min(rpm_m2, 300), -300)

                # === ONLY SEND RAW RPMs ===
                rpm_cmd = f"{rpm_m1:.1f},{rpm_m2:.1f}\n"
                self.serial_interface.send_message(rpm_cmd)

                print(f"[MOTOR OUTPUT] RPM_M1: {rpm_m1:.1f}, RPM_M2: {rpm_m2:.1f}")

            time.sleep(0.1)  # Control loop rate


    def send_message(self):
        message = self.message_entry.get()
        self.serial_interface.send_message(message)  # Use serial_interface instead of bluetooth_interface
        self.previous_messages.insert(0, message)
        self.current_message_index = -1
        self.message_entry.delete(0, tk.END)
        self.update_message_history()

    def send_message_event(self, event):
        self.send_message()

    def load_previous_message(self, event):
        if self.current_message_index < len(self.previous_messages) - 1:
            self.current_message_index += 1
        self.update_message_entry()

    def load_next_message(self, event):
        if self.current_message_index > 0:
            self.current_message_index -= 1
        elif self.current_message_index == 0:
            self.current_message_index = -1
        self.update_message_entry()

    def update_message_entry(self):
        if self.current_message_index == -1:
            self.message_entry.delete(0, tk.END)
        else:
            self.message_entry.delete(0, tk.END)
            self.message_entry.insert(
                0, self.previous_messages[self.current_message_index])

    def update_message_history(self):
        self.message_history_text.delete("1.0", tk.END)
        for message in reversed(self.previous_messages):
            self.message_history_text.insert(tk.END, message + "\n")

    def update_received_message(self, received_message):
        self.previous_received_messages.insert(0, received_message)
        if (len(self.previous_received_messages) > 10):
            self.previous_received_messages.pop()
        self.received_message_text.delete("1.0", tk.END)
        for message in reversed(self.previous_received_messages):
            self.received_message_text.insert(tk.END, message + "\n")

    def check_for_updates(self):
        """ Check the queue for new data and update the map if necessary """
        try:
            map_corners, plot_para, path, status, erasiable_corners = self.data_queue.get_nowait()
            self.status_label.config(
                text=f"Power: {power}, Mode: {mode}\n"
                     f"Current: x={state['x']:.2f}, y={state['y']:.2f}, θ={state['theta']:.2f}\n"
                     f"Target:  x={reference['x']:.2f}, y={reference['y']:.2f}"
            )
            self.draw_map(map_corners, plot_para, path, erasiable_corners)
        except queue.Empty:
            pass
        finally:
            self.master.after(1, self.check_for_updates)


def main():
    serial_interface = SerialInterface(port=SERIAL_PORT, baudrate=BAUD_RATE)

    # Motor Test
    #serial_interface.send_message("400.0,-400.0,3000\n")
    #time.sleep(3.2)

    tag_tracker = AprilTagTracker(tag_size=0.045, tag_robot=0, tag_ref=1, state_ref=shared_state)
    tag_tracker.start()

    # === GUI Setup ===
    root = tk.Tk()
    root.title("WIPER CONTROL")
    app = App(root, serial_interface)

    window_width = 800
    window_height = 600
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x_coordinate = (screen_width - window_width) // 2
    y_coordinate = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x_coordinate}+{y_coordinate}")
    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)
    root.protocol("WM_DELETE_WINDOW", root.quit)

    # === Start Control Thread ===
    thread1 = threading.Thread(target=app.cmd_write_thread)
    thread1.daemon = True
    thread1.start()

    root.mainloop()

if __name__ == "__main__":
    main()
