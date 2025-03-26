import serial
import threading
import tkinter as tk
import atexit
import time
import csv
import queue
import re
import os

# Define global variables
power = 0
mode = "default"
flag_terminate = False
current_position = {'x': 0.0, 'y': 0.0}
target_position = {'x': 0.0, 'y': 0.0}

class BluetoothInterface:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = serial.Serial(
            port=self.port, baudrate=self.baudrate, timeout=1)
        self.message_queue = queue.Queue()  # Queue to store received messages
        # Event to signal the thread to stop
        self.receive_thread_stop = threading.Event()
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        # Register close_serial to be called when the program exits
        atexit.register(self.close_serial)

    def send_message(self, message):
        self.serial_port.write(message.encode())

    def receive_data(self):
        while not self.receive_thread_stop.is_set():
            if self.serial_port.in_waiting > 0:
                received_data = self.serial_port.readline().decode().strip()
                self.message_queue.put(received_data)  # Store received data in the queue
                print("WIPER:", len(received_data))
                self.log_to_csv(received_data)
            time.sleep(0.1)  # Add a small delay to avoid busy waiting

    def receive_message(self):
        """ Retrieve a message from the queue """
        try:
            return self.message_queue.get_nowait()
        except queue.Empty:
            return ""
    def log_to_csv(self, received_data):
        """ Parse and log structured data from Arduino to CSV """
        pattern = re.compile(
            r"(?P<Power>ON|OFF)\s+"
            r"M(?P<Mode>\d+)\s+"
            r"(?P<BatteryVoltage>[\d.]+)V\s*\|\s*"
            r"\(\s*(?P<CurrX>[-\d.]+)m,\s*(?P<CurrY>[-\d.]+)m\s*\)\s*=>\s*"
            r"\(\s*(?P<TargetX>[-\d.]+)m,\s*(?P<TargetY>[-\d.]+)m\s*\)\s*\|\s*"
            r"L\s*(?P<RPM_M1>[-\d.]+)\s*=\[\s*(?P<DriveM1>-?\d+)\s*\]=>\s*(?P<TargetM1>[-\d.]+)\s*"
            r"R\s*(?P<RPM_M2>[-\d.]+)\s*=\[\s*(?P<DriveM2>-?\d+)\s*\]=>\s*(?P<TargetM2>[-\d.]+)\s*\|\s*"
            r"TarDeg\s*(?P<TargetDeg>[-\d.]+)°\s*"
            r"TarDis\s*(?P<TargetDis>[-\d.]+)m\s*\|\s*"
            r"CurrDeg:\s*(?P<CurrDeg>[-\d.]+)°\s*"
            r"CurrDis:\s*(?P<CurrDis>[-\d.]+)m"
        )

        match = pattern.search(received_data)
        if not match:
            print("Warning: Unable to parse line:", received_data)
            return

        data = match.groupdict()

        fieldnames = [
            "Power", "Mode", "BatteryVoltage",
            "CurrX", "CurrY", "TargetX", "TargetY",
            "RPM_M1", "DriveM1", "TargetM1",
            "RPM_M2", "DriveM2", "TargetM2",
            "TargetDeg", "TargetDis",
            "CurrDeg", "CurrDis"
        ]

        file_exists = os.path.isfile('motor_log.csv')
        with open('motor_log.csv', mode='a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerow(data)


    def close_serial(self):
        self.receive_thread_stop.set()  # Signal the receive thread to stop
        self.receive_thread.join()  # Wait for the receive thread to stop
        if self.serial_port.is_open:
            self.serial_port.close()

class App:
    def __init__(self, master, bluetooth_interface):
        self.master = master
        self.bluetooth_interface = bluetooth_interface
        self.previous_messages = []
        self.previous_received_messages = []
        self.current_message_index = -1
        self.data_queue = queue.Queue()  # Initialize the data_queue
        self.log_buffer = []  # Initialize the log buffer

        self.frame = tk.Frame(self.master)
        self.frame.pack()

        self.message_label = tk.Label(self.frame, text="Message:")
        self.message_label.grid(row=0, column=0)

        self.message_entry = tk.Entry(self.frame)
        self.message_entry.grid(row=0, column=1)
        # Bind Return key to send_message_event
        self.message_entry.bind("<Return>", self.send_message_event)

        self.send_button = tk.Button(
            self.frame, text="Send", command=self.send_message)
        self.send_button.grid(row=0, column=2)

        self.message_history_label = tk.Label(
            self.frame, text="Message History:")
        self.message_history_label.grid(
            row=1, column=0, columnspan=3, sticky="w")

        self.message_history_text = tk.Text(self.frame, height=10, width=40)
        self.message_history_text.grid(row=2, column=0, columnspan=3)

        self.received_message_label = tk.Label(
            self.frame, text="Received Message:")
        self.received_message_label.grid(row=3, column=0, sticky="w")

        self.received_message_text = tk.Text(self.frame, height=10, width=150)
        self.received_message_text.grid(row=4, column=0, columnspan=3)

        self.status_label = tk.Label(self.frame, text="Status:")
        self.status_label.grid(row=5, column=0, columnspan=3, sticky="w")

        # Bind up and down arrow keys to load previous messages
        self.master.bind("<Up>", self.load_previous_message)
        self.master.bind("<Down>", self.load_next_message)

        # Start the update checker
        self.check_for_updates()

    def read_rpm_from_arduino(self):
        """ Read RPM values from the Arduino via Bluetooth """
        response = self.bluetooth_interface.receive_data()
        print("response",response)
        if response:
            rpm_m1, rpm_m2 = map(float, response.split(","))
            return rpm_m1, rpm_m2
        return 0.0, 0.0

    def cmd_write_thread(self):
        global power, mode, flag_terminate, current_position, target_position
        logging_active = False  # Flag to control logging state
        while not flag_terminate:
            timestamp = time.time()
            #rpm_m1, rpm_m2 = self.read_rpm_from_arduino()  # Read RPM values from Arduino
            cmd = f"{current_position['x']:.3f},{current_position['y']:.3f}|{target_position['x']:.3f},{target_position['y']:.3f}|{mode}|{power}\n"
            #self.bluetooth_interface.send_message(cmd)
            # Check if the motor input is 1 to start logging
            if power == 1 and not logging_active:
                logging_active = True
                print("Logging started")
            
            # Check if the motor input is 0 to stop logging
            if power == 0 and logging_active:
                logging_active = False
                print("Logging stopped")
            
            # Log data to the buffer if logging is active
            if logging_active:
                self.log_buffer.append([timestamp, current_position['x'], current_position['y'], target_position['x'], target_position['y'], mode, power, rpm_m1, rpm_m2])
            
            time.sleep(0.5)  # Adjust the delay to ensure commands are sent at an appropriate interval

    def send_message(self):
        message = self.message_entry.get()
        self.bluetooth_interface.send_message(message)
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
                text=f"Quadrant: {status[2]}\nPower: {status[0]}\nMode: {status[1]}\nCurrent Position: ({status[3]['x']:.2f}, {status[3]['y']:.2f})\nTarget Position: \t({status[4]['x']:.2f}, {status[4]['y']:.2f})")
            self.draw_map(map_corners, plot_para, path, erasiable_corners)
        except queue.Empty:
            pass
        finally:
            self.master.after(1, self.check_for_updates)

def main():
    bluetooth_port = 'COM4'  # for Windows
    
    bluetooth_interface = BluetoothInterface(
        port=bluetooth_port, baudrate=9600)
    root = tk.Tk()
    root.title("WIPER CONTROL")
    app = App(root, bluetooth_interface)

    # Calculate the position to center the window
    window_width = 400  # Adjust width as needed
    window_height = 200  # Adjust height as needed
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x_coordinate = (screen_width - window_width) // 2
    y_coordinate = (screen_height - window_height) // 2

    # Set window dimensions and position
    root.geometry(
        f"{window_width}x{window_height}+{x_coordinate}+{y_coordinate}")
    # Properly close the GUI window
    root.protocol("WM_DELETE_WINDOW", root.quit)

    # Start the background threads
    thread1 = threading.Thread(
        target=app.cmd_write_thread)
    thread1.daemon = True
    thread1.start()

    root.mainloop()

if __name__ == "__main__":
    main()