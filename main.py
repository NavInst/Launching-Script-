#!/usr/bin/env python3
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import yaml
import importlib
import PySimpleGUI as sg
import math
import os

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
                            # Use sensor QoS to avoid RELIABILITY_QOS_POLICY warnings
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
                        ["ping", "-c", "1", "-W", "1", ip],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                    status = (result.returncode == 0)
                else:
                    status = True

                with status_lock:
                    sensor_status[name] = status
        time.sleep(1)

# --------------------------
# Debounce LED color
# --------------------------
def get_stable_color(name, color):
    history = sensor_color_history.get(name, [])
    history.append(color)
    if len(history) > SENSOR_STABLE_COUNT:
        history.pop(0)
    sensor_color_history[name] = history
    # Only change color if last N readings are same, otherwise yellow
    if all(c == history[-1] for c in history):
        return history[-1]
    return 'yellow'

# --------------------------
# GUI
# --------------------------
def gui_thread():
    sg.theme('DarkBlue3')
    group_columns = []

    for group, sensors in CONFIG["sensors"].items():
        col_layout = [[sg.Text(group.upper(), font=('Any', 14), text_color='yellow')]]
        for sensor in sensors:
            col_layout.append([
                sg.Text(sensor["name"], size=(18,1), font=('Any',12)),
                sg.Graph(
                    canvas_size=(LED_RADIUS*2,LED_RADIUS*2),
                    graph_bottom_left=(0,0),
                    graph_top_right=(LED_RADIUS*2,LED_RADIUS*2),
                    background_color='black',
                    key=sensor["name"]
                )
            ])
        col_layout.append([sg.HorizontalSeparator()])
        group_columns.append(
            sg.Column(col_layout, vertical_scroll_only=False, expand_y=True, expand_x=True, pad=(5,5))
        )

    layout = [
        [sg.Text('Sensor Dashboard', font=('Any', 18), justification='center', expand_x=True)],
        [sg.HorizontalSeparator()],
        group_columns
    ]

    window = sg.Window(
        'Sensor Dashboard',
        layout,
        finalize=True,
        resizable=True,
        element_justification="center",
        font=('Any', 11)
    )

    while True:
        event, _ = window.read(timeout=200)
        if event == sg.WINDOW_CLOSED:
            break

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
                        color = "red"
                    elif not topics_ok:
                        color = "yellow"
                    else:
                        color = "green"

                    stable_color = get_stable_color(name, color)
                    g = window[name]
                    g.erase()
                    g.draw_circle((LED_RADIUS,LED_RADIUS), LED_RADIUS,
                                  fill_color=stable_color, line_color='black', line_width=2)

    window.close()

# --------------------------
# Main
# --------------------------
def main():
    rclpy.init()

    # Start topic monitor in background
    monitor = TopicMonitor(CONFIG)
    ros_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    ros_thread.start()

    # Start device checks
    threading.Thread(target=sensor_check_thread, daemon=True).start()

    # Run GUI in main thread
    gui_thread()

    # Cleanup
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()