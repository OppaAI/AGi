#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json

class ServerVitalPulse(Node):
    def __init__(self):
        super().__init__('server_vital_pulse')

        # Publisher: send feedback to robot
        self.feedback_pub = self.create_publisher(String, 'vital_feedback', 10)

        # Subscriber: receive robot pulse
        self.pulse_sub = self.create_subscription(
            String,
            'vital_pulse',
            self.pulse_callback,
            10
        )

        self.last_pulse_time = None
        self.bpm = 0

    def pulse_callback(self, msg):
        now = time.time()
        data = json.loads(msg.data)
        robot_time = data.get("robot_time", None)

        if self.last_pulse_time is not None:
            interval = now - self.last_pulse_time
            if interval > 0:
                hz = 1.0 / interval
                self.bpm = hz * 60

        self.last_pulse_time = now

        # Send feedback back to robot
        feedback_msg = String()
        feedback_msg.data = json.dumps({
            "server_time": now,
            "bpm": self.bpm
        })
        self.feedback_pub.publish(feedback_msg)
        print(f"Pulse received. BPM: {self.bpm:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = ServerVitalPulse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Server Vital Pulse stopped.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
