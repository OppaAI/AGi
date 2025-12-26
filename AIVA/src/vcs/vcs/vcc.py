import time

# ANSI colors
DARK_CYAN = "\033[38;5;30m"
BRIGHT_CYAN = "\033[38;5;51m"

class VitalCentralCore:
    def __init__(self, blink_duration=1.0):
        self.blink_duration = blink_duration

        self.last_pulse_time = None
        self.current_opm = 0.0
        self.current_rtt = 0.0

        # ⭐ identity
        self.robot_id = "Unknown"
        self.user_id = "Unknown"

        self._blink_step = 0
        
    def get_now_sec(self):
        """Return current ROS time in seconds as float"""
        try:
            # if rclpy is initialized
            from rclpy.clock import Clock
            return Clock().now().nanoseconds / 1e9
        except Exception:
            # fallback to system time
            return time.time()

    def record_identity(self, robot_id, user_id):
        self.robot_id = robot_id
        self.user_id = user_id

    def record_remote_pulse(self, opm, remote_ts):
        now = time.time()

        self.last_pulse_time = now
        self.current_opm = float(opm)

        if remote_ts is not None:
            try:
                self.current_rtt = max(0.0, (now - float(remote_ts)) * 1000.0)
            except Exception:
                self.current_rtt = 0.0

    def blink_color(self, now):
        if self.last_pulse_time is None:
            return DARK_CYAN

        if (now - self.last_pulse_time) <= self.blink_duration:
            return BRIGHT_CYAN

        return DARK_CYAN

    def get_status(self):
        self._blink_step += 1
        return {
            "opm": self.current_opm,
            "rtt": self.current_rtt,
            "blink": self._blink_step % 2,
            "color": self.blink_color(time.time())
        }

    # ⭐ 新增方法，斷線或沒收到 pulse 時清除
    def clear_identity_and_pulse(self):
        self.robot_id = "Unknown"
        self.user_id = "Unknown"
        self.current_opm = 0.0
        self.current_rtt = 0.0
        self.last_pulse_time = None
