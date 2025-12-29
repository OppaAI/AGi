# Vital Terminal Core (VTC)
# - Broadcast vital pulse to ROS network to ping the server
# - Sends vital data and timestamp to the server for analysis
# - Receives feedback from the server to verify connection
# - Detects timeouts and disconnections
# - Displays vital signs in the terminal

class VitalTerminalCore:
    
    def __init__(self, interval=1.0, blink_duration=1.0):
        self.interval = interval
        self.blink_duration = blink_duration
        self.last_feedback_time = None
        self.step = 0
        self.current_opm = 0.0
        self.current_rtt = 0.0
        self.fire_timestamp = None

    def get_now_sec(self, node):
        now = node.get_clock().now()
        return now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9

    def compute_opm(self):
        self.current_opm = (1 / self.interval) * 60
        return self.current_opm

    def compute_rtt(self, fire_timestamp, last_feedback_time):
        if fire_timestamp and last_feedback_time:
            self.current_rtt = (last_feedback_time - fire_timestamp) * 1000
        else:
            self.current_rtt = 0
        return self.current_rtt

    def blink_color(self, now):
        if self.last_feedback_time and 0 <= (now - self.last_feedback_time) <= self.blink_duration:
            return "\033[38;5;51m"  # BRIGHT_CYAN
        return "\033[38;5;30m"      # DARK_CYAN
