#!/usr/bin/env python3
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import yaml
import importlib
import math
import os
import tkinter as tk

# --------------------------
# Load config.yaml
# --------------------------
with open("config.yaml", "r") as f:
    CONFIG = yaml.safe_load(f)

# --------------------------
# Globals
# --------------------------
topic_status = {}    # {"topic": {"last_time": float, "is_corrupted": bool}}
sensor_status = {}   # {"sensor_name": bool}
status_lock = threading.Lock()  # Thread-safe access
SENSOR_TIMEOUT = 2.0
LED_RADIUS = 20

# Debounce history
SENSOR_STABLE_COUNT = 3
sensor_color_history = {}  # {"sensor_name": [last_colors...]}

# --------------------------
# Detect ROS2 topic type dynamically
# --------------------------
def get_topic_type(topic_name):
    """Return the ROS2 type string for a topic, e.g., 'sensor_msgs/msg/Imu'"""
    result = subprocess.run(
        ["ros2", "topic", "type", topic_name],
        capture_output=True, text=True
    )
    if result.returncode == 0:
        return result.stdout.strip()
    return None

def get_msg_class(topic_name):
    """Dynamically import the message class for a topic"""
    type_str = get_topic_type(topic_name)
    if not type_str:
        return None
    module_name, class_name = type_str.replace('/', '.').rsplit('.', 1)
    module = importlib.import_module(module_name)
    return getattr(module, class_name)

# --------------------------
# NaN / Inf check
# --------------------------
def is_corrupted(value):
    if isinstance(value, (float, int)):
        return math.isnan(value) or math.isinf(value)
    elif hasattr(value, '__slots__'):
        for field in value.__slots__:
            if is_corrupted(getattr(value, field)):
                return True
    return False

# --------------------------
# ROS2 Topic Monitor
# --------------------------
class TopicMonitor(Node):
    def __init__(self, config):
        super().__init__('topic_monitor')

        for group, sensors in config["sensors"].items():
            for sensor in sensors:
                if "topics" in sensor:
                    for t in sensor["topics"]:
                        topic = t["name"]
                        msg_type = get_msg_class(topic)
                        if msg_type:
                            print(f"[INFO] Subscriping to: {topic}")
                            self.create_subscription(
                                msg_type,
                                topic,
                                lambda msg, top=topic: self.callback(msg, top),
                                qos_profile=qos_profile_sensor_data
                            )
                            with status_lock:
                                topic_status[topic] = {'last_time': None, 'is_corrupted': False}
                        else:
                            print(f"[WARNING] Could not detect type for topic: {topic}")

    def callback(self, msg, topic_name):
        corrupted = is_corrupted(msg)
        with status_lock:
            topic_status[topic_name]['is_corrupted'] = corrupted
            if not corrupted:
                topic_status[topic_name]['last_time'] = time.time()

# --------------------------
# USB / Ethernet checks
# --------------------------
def sensor_check_thread():
    while True:
        for group, sensors in CONFIG["sensors"].items():
            for sensor in sensors:
                name = sensor["name"]
                stype = sensor["type"]

                if stype == "usb":
                    status = os.path.exists(sensor["device_path"])
                elif stype == "ethernet":
                    ip = sensor["ip"]
                    result = subprocess.run(
                        ["ping", "-c", "2", "-W", "0.1", ip],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                    status = (result.returncode == 0)
                else:
                    status = True

                with status_lock:
                    sensor_status[name] = status
        time.sleep(0.1)

# --------------------------
# Debounce LED color
# --------------------------
def get_stable_color(name, color):
    history = sensor_color_history.get(name, [])
    history.append(color)
    if len(history) > SENSOR_STABLE_COUNT:
        history.pop(0)
    sensor_color_history[name] = history
    if all(c == history[-1] for c in history):
        return history[-1]
    return '#FFD93D'  # amber as transitional

# --------------------------
# GUI with Tkinter
# --------------------------
class Dashboard(tk.Tk):
    def __init__(self, config):
        super().__init__()
        self.title("NavINST Sensor Launch")
        self.configure(bg="#6389AC")

        title = tk.Label(self, text="Sensor Dashboard",
                         font=("Arial", 20, "bold"),
                         fg="white", bg="#6389AC")
        title.pack(pady=10)

        container = tk.Frame(self, bg="#6389AC")
        container.pack(fill="both", expand=True)

        self.sensor_widgets = {}

        # Groups horizontally, sensors vertically inside each group
        for group, sensors in config["sensors"].items():
            group_frame = tk.LabelFrame(container, text=group.upper(),
                                        font=("Arial", 14, "bold"),
                                        fg="white", bg="#6389AC",
                                        labelanchor="n", bd=2, relief="groove")
            group_frame.pack(side="left", padx=15, pady=5, fill="y")

            for sensor in sensors:
                row = tk.Frame(group_frame, bg="#6389AC")
                row.pack(anchor="w", pady=4)

                tk.Label(row, text=sensor["name"],
                         font=("Arial", 12), width=22,
                         anchor="w", bg="#6389AC", fg="white", padx=30, pady=5).pack(side="left")

                canvas = tk.Canvas(row, width=LED_RADIUS*2,
                                   height=LED_RADIUS*2, bg="#6389AC",
                                   highlightthickness=0)
                canvas.pack(side="left", padx=5)
                circle = canvas.create_oval(2, 2, LED_RADIUS*2, LED_RADIUS*2,
                                            fill="#FF0000", outline="black")

                self.sensor_widgets[sensor["name"]] = (canvas, circle)

        self.after(200, self.update_loop)

    def update_sensor(self, name, color):
        if name in self.sensor_widgets:
            canvas, circle = self.sensor_widgets[name]
            canvas.itemconfig(circle, fill=color)

    def update_loop(self):
        now = time.time()
        with status_lock:
            for group, sensors in CONFIG["sensors"].items():
                for sensor in sensors:
                    name = sensor["name"]
                    device_ok = sensor_status.get(name, False)

                    topics_ok = True
                    if "topics" in sensor and len(sensor["topics"]) > 0:
                        for t in sensor["topics"]:
                            topic = t["name"]
                            if topic not in topic_status:
                                topics_ok = False
                                break
                            last = topic_status[topic]['last_time']
                            corrupted = topic_status[topic]['is_corrupted']
                            if corrupted or last is None or (now - last) > SENSOR_TIMEOUT:
                                topics_ok = False
                                break

                    if not device_ok:
                        color = "#FF0000"   # bright red
                    elif not topics_ok:
                        color = "#FFFB00"    # amber/yellow
                    else:
                        color = "#00FF00"   # bright green

                    stable_color = get_stable_color(name, color)
                    self.update_sensor(name, stable_color)

        self.after(200, self.update_loop)

# --------------------------
# Main
# --------------------------
def main():
    rclpy.init()

    monitor = TopicMonitor(CONFIG)
    ros_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    ros_thread.start()

    threading.Thread(target=sensor_check_thread, daemon=True).start()

    dashboard = Dashboard(CONFIG)
    dashboard.mainloop()

    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

