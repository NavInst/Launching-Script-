#!/usr/bin/env python3
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
import yaml
import os
import math
import importlib
import PySimpleGUI as sg

# --------------------------
# Load config.yaml
# --------------------------
with open("config.yaml", "r") as f:
    CONFIG = yaml.safe_load(f)

# --------------------------
# Globals
# --------------------------
topic_status = {}   # {"topic": {"last_time": float, "is_corrupted": bool}}
sensor_status = {}  # {"sensor_name": bool}
SENSOR_TIMEOUT = 2.0
LED_RADIUS = 20

# --------------------------
# Dynamic ROS msg type import
# --------------------------
def get_msg_type(type_str: str):
    module_name, class_name = type_str.rsplit(".", 1)
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
                        msg_type = get_msg_type(t["msg_type"])
                        self.create_subscription(
                            msg_type,
                            topic,
                            lambda msg, top=topic: self.callback(msg, top),
                            10
                        )
                        topic_status[topic] = {'last_time': None, 'is_corrupted': False}

    def callback(self, msg, topic_name):
        if is_corrupted(msg):
            topic_status[topic_name]['is_corrupted'] = True
        else:
            topic_status[topic_name]['is_corrupted'] = False
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
                    sensor_status[name] = os.path.exists(sensor["device_path"])
                elif stype == "ethernet":
                    ip = sensor["ip"]
                    result = subprocess.run(
                        ["ping", "-c", "1", "-W", "1", ip],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                    sensor_status[name] = (result.returncode == 0)
                else:
                    sensor_status[name] = True
        time.sleep(1)

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

        for group, sensors in CONFIG["sensors"].items():
            for sensor in sensors:
                name = sensor["name"]
                device_ok = sensor_status.get(name, False)

                topics_ok = True
                if "topics" in sensor and len(sensor["topics"]) > 0:
                    for t in sensor["topics"]:
                        topic = t["name"]
                        if topic not in topic_status:
                            continue
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

                g = window[name]
                g.erase()
                g.draw_circle((LED_RADIUS,LED_RADIUS), LED_RADIUS, fill_color=color, line_color='black', line_width=2)

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

