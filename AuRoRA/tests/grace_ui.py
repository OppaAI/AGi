import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json
import time

class GraceUI(Node):
    def __init__(self):
        super().__init__('grace_ui')
        self.pub = self.create_publisher(String, '/cns/neural_input', 10)
        self.sub = self.create_subscription(String, '/gce/response', self.callback, 10)

        # Timing + token tracking state
        self._turn_start: float | None = None
        self._token_count: int = 0

        print("\n" + "="*40)
        print("🌸 GRACE COGNITIVE INTERFACE ONLINE")
        print("="*40 + "\n")

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            msg_type = data.get("type")
            content = data.get("content", "")

            if msg_type == "start":
                self._turn_start = time.monotonic()
                self._token_count = 0
                print(f"\n🌸 GRACE: {content}", end="", flush=True)

            elif msg_type == "delta":
                # Approximate token count: split on whitespace as a rough proxy.
                # Replace with an exact count if your backend sends token metadata.
                self._token_count += max(1, len(content.split()))
                print(content, end="", flush=True)

            elif msg_type == "done":
                elapsed = (
                    time.monotonic() - self._turn_start
                    if self._turn_start is not None
                    else 0.0
                )
                tps = self._token_count / elapsed if elapsed > 0 else 0.0
                stats = (
                    f"[⏱ {elapsed:.2f}s | "
                    f"~{self._token_count} tok | "
                    f"{tps:.1f} tok/s]"
                )
                print(f"\n{stats}")
                print("-" * 20 + "\n> ", end="", flush=True)
                self._turn_start = None

            elif msg_type == "error":
                print(f"\n❌ SYSTEM ERROR: {content}\n> ", end="", flush=True)
                self._turn_start = None

        except json.JSONDecodeError:
            print(f"\n🌸 GRACE (raw): {msg.data}\n> ", end="")

    def send(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GraceUI()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
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
