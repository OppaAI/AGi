import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json
import sys

class GraceUI(Node):
    def __init__(self):
        super().__init__('grace_ui')
        self.pub = self.create_publisher(String, '/cns/neural_input', 10)
        self.sub = self.create_subscription(String, '/gce/response', self.callback, 10)
        print("\n" + "="*40)
        print("🌸 GRACE COGNITIVE INTERFACE ONLINE")
        print("="*40 + "\n")

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            msg_type = data.get("type")
            content = data.get("content", "")

            if msg_type == "start":
                print(f"\n🌸 GRACE: {content}", end="", flush=True)
            elif msg_type == "delta":
                print(content, end="", flush=True)
            elif msg_type == "done":
                print("\n" + "-"*20 + "\n> ", end="", flush=True)
            elif msg_type == "error":
                print(f"\n❌ SYSTEM ERROR: {content}\n> ", end="", flush=True)

        except json.JSONDecodeError:
            # Fallback if raw text is sent
            print(f"\n🌸 GRACE (raw): {msg.data}\n> ", end="")

    def send(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = GraceUI()
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            user_input = input("> ")
            if user_input.lower() in ['exit', 'quit', 'clear']:
                break
            if user_input.strip():
                node.send(user_input)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()