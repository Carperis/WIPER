import serial
import threading
import tkinter as tk
import atexit
import time
import queue
import os
import numpy as np
import platform

from lqr import TVLQRController
from apriltag_tracker import AprilTagTracker
from shared_state import state as shared_state, state_lock as shared_state_lock

dt = 0.05  # Time step for control loop

# Define global variables
if platform.system() == "Windows":                  # Windows
    SERIAL_PORT = "COM5"
else:
    SERIAL_PORT = "/dev/tty.usbserial-A1080DBC"     # Mac

BAUD_RATE = 9600
power = 0
mode = "default"
flag_terminate = False
camera_available = False
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
        print(f"[SEND] RPM Command: {message.strip()}")
        self.serial_port.write(message.encode())

    def update_state(self, message):
        global camera_available

        try:
            message = message.decode(errors='replace').replace('\r', '').replace('\x00', '').strip()

            if not message or message.startswith(("TEST", "Power", "READY")):
                return

            values = message.split()
            ax_raw, ay_raw, az, dt_val = map(float, values[:4])

            # Reorient IMU axes (depends on your mounting!)
            acc_x = ay_raw
            acc_y = -ax_raw

            # Compute delta position (velocity × time)
            dx = acc_x * dt_val
            dy = acc_y * dt_val

            # IMU heading estimate
            imu_theta = np.arctan2(ay_raw, ax_raw) % (2 * np.pi)

            with shared_state_lock:
                cam_x = state.get('cam_x')
                cam_y = state.get('cam_y')
                cam_theta = state.get('cam_theta')
                cam_ts = state.get('cam_timestamp', 0)

            cam_age = time.time() - cam_ts

            if cam_age < 0.1 and all(np.isfinite(v) for v in [cam_x, cam_y, cam_theta]):
                # Use camera: absolute pose
                with shared_state_lock:
                    if 'prev_cam_x' not in state or cam_age > 1.0:
                        # First valid CAM reading — initialize
                        state['prev_cam_x'] = cam_x
                        state['prev_cam_y'] = cam_y
                    delta_x = cam_x - state['prev_cam_x']
                    delta_y = cam_y - state['prev_cam_y']
                    state['x'] = state.get('x', 0.0) + delta_x
                    state['y'] = state.get('y', 0.0) + delta_y
                    state['theta'] = np.radians(cam_theta)  # Use absolute heading
                    state['prev_cam_x'] = cam_x
                    state['prev_cam_y'] = cam_y
                source = "CAM"
            else:
                # Use IMU: accumulate pose estimate
                with shared_state_lock:
                    state['x'] = state.get('x', 0.0) + dx
                    state['y'] = state.get('y', 0.0) + dy
                    state['theta'] = imu_theta
                source = "IMU"

            #print(f"[{source}] state: x = {state['x']:.3f}, y = {state['y']:.3f}, θ = {state['theta']:.2f}")


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
        logging_active = False
        rpm_m1, rpm_m2 = 0.0, 0.0
        dt_local = dt  # Time step
        next_loop_time = time.time()

        while not flag_terminate:
            loop_start = time.time()

            # === Power ON: Initialize controller ===
            if power == 1 and not logging_active:
                logging_active = True
                print("[INFO] Power ON: Logging started.")

                with shared_state_lock:
                    x0 = state.get('x', 0.0)
                    y0 = state.get('y', 0.0)
                    theta0 = state.get('theta', 0.0)
                start_state = (x0, y0, theta0)

                self.lqr_controller = TVLQRController(start=(0, 0, theta0), mode=self.selected_mode, N=30, dt=dt_local)

                shifted_traj = [
                    (x0 + dx, y0 + dy, dtheta)
                    for dx, dy, dtheta in self.lqr_controller.reference_trajectory
                ]
                self.lqr_controller.reference_trajectory = shifted_traj
                self.lqr_controller.step_counter = 0

            # === Power OFF: Clean up ===
            if power == 0 and logging_active:
                logging_active = False
                print("[INFO] Power OFF: Logging stopped.")
                self.lqr_controller = None

            # === Control Loop ===
            prev_time = time.time()
            if self.lqr_controller is not None:
                k = self.lqr_controller.step_counter

                with shared_state_lock:
                    x_curr = np.array([state['x'], state['y'], state['theta']])
                    source = "CAM" if (time.time() - state.get('cam_timestamp', 0) < 0.1 and 
                                    all(np.isfinite([state.get('cam_x'), state.get('cam_y'), state.get('cam_theta')]))) else "IMU"

                x_ref = self.lqr_controller.reference_trajectory[k+1]
                rpm_m2, rpm_m1 = self.lqr_controller.get_control(x_curr)

                # print dim of x_ref
                #print(f"[DEBUG] x_ref: {x_ref}, dim: {np.shape(x_ref)}")

                print(f"[{source}][CONTROL] x = {x_curr[0]:.3f}, y = {x_curr[1]:.3f}, θ = {x_curr[2]:.2f} | "
                    f"xref = {x_ref[0]:.3f}, yref = {x_ref[1]:.3f}, θref = {x_ref[2]:.2f}")

                rpm_cmd = f"{rpm_m1:.1f},{rpm_m2:.1f}\n"
                self.serial_interface.send_message(rpm_cmd)

                pos_error = np.linalg.norm(x_curr[0] - x_ref[0])
                #pos_error = np.linalg.norm(x_curr[:2] - x_ref[:2])
                theta_error = abs((x_curr[2] - x_ref[2] + np.pi) % (2 * np.pi) - np.pi)
                print(f"[ERROR] pos = {pos_error:.4f}, theta = {theta_error:.4f}")

                pos_thres = 0.05
                #theta_thres = 0.05

                if k == self.lqr_controller.N - 2 and pos_error < pos_thres: #and theta_error < theta_thres:
                    self.serial_interface.send_message("0.0,0.0\n")
                    print("[INFO] Final step reached and within threshold. Stopping.")
                    self.lqr_controller = None
                    continue

                if pos_error < pos_thres: #and theta_error < theta_thres:
                    if self.lqr_controller.step_counter < self.lqr_controller.N - 1:
                        self.lqr_controller.step_counter += 1

            # === Sleep until next loop ===
            if power == 1:
                now = time.time()
                loop_hz = 1.0 / (now - prev_time) if now != prev_time else 0.0
                #print(f"[LOOP] Rate: {loop_hz:.2f} Hz")
                prev_time = now

            sleep_time = max(0.0, next_loop_time - time.time())
            time_del = 0.01  # Adjust this value to control the delay between commands
            time.sleep(sleep_time + time_del)



    def send_message(self):
        message = self.message_entry.get()
        self.serial_interface.send_message(message)
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

    try:
        tag_tracker = AprilTagTracker(tag_size=0.041, tag_robot=0, tag_ref=1, state_ref=shared_state)
        tag_tracker.start()
        print("[INFO] AprilTag tracker started.")
    except Exception as e:
        print(f"[WARNING] AprilTag tracker could not be initialized: {e}")
        tag_tracker = None

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
