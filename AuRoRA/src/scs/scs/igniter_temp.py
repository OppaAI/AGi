"""
Igniter (Bootstrap) Module

Welcome to my Igniter Module!
This module handles the ignition sequence (bootstrap) of the robot's system.

Startup sequence:
    1. TallEEE: Centralized Triple-Aggregation Logger & Ledger Emergency and Exception Entry
    2. ROS Bridge WebSocket Server: Enables web interface communication
    3. Flask Web Server: Serves the Grace web interface
    4. SearXNG MCP Server: Enhanced web search with full content extraction
    5. Nature Skills MCP Server: Outdoor/nature exploration tools
    6. ROS2 Bridge: Connects all components
"""
# System modules
from enum import Enum
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
import os

# ROS2 modules
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import Log
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# AGi modules
from scs.eee import get_logger, EEEAggregator
from scs.eee_ros_bridge import EEEROSBridge, ReflexPlugin, AwarenessPlugin

# State of all modules for the robot
class StateOfModule(str, Enum):
    INIT = "INIT"
    RUN = "RUN"
    DEGRADED = "DEGRADED"
    OFF = "OFF"


class ServerManager:
    """
    Manages external servers (ROS Bridge, Flask, MCP, Skills) for the robot.
    Starts all servers in parallel and polls for readiness instead of sleeping.
    """
    # How long to poll for each server before giving up (seconds)
    ROSBRIDGE_TIMEOUT = 5
    FLASK_TIMEOUT     = 5
    MCP_TIMEOUT       = 3
    SKILLS_TIMEOUT    = 3
    POLL_INTERVAL     = 0.1  # Check every 100ms

    def __init__(self, logger):
        self.logger = logger
        self.rosbridge_process = None
        self.flask_process     = None
        self.mcp_process       = None
        self.skills_process    = None
        self.base_dir          = Path.home() / "AGi"

    # -------------------------------------------------------------------------
    # Internal helper: poll a process until it's alive or times out
    # -------------------------------------------------------------------------
    def _wait_for_process(self, process, timeout: float) -> bool:
        """Poll process every POLL_INTERVAL seconds up to timeout. Returns True if alive."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is None:
                return True          # process is running
            time.sleep(self.POLL_INTERVAL)
        return process.poll() is None  # final check

    # -------------------------------------------------------------------------
    # Individual server launchers (just Popen, no sleeping)
    # -------------------------------------------------------------------------
    def _launch_rosbridge(self) -> bool:
        try:
            self.logger.info("🌉 Starting ROS Bridge WebSocket server...")
            self.rosbridge_process = subprocess.Popen(
                ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid
            )
            if self._wait_for_process(self.rosbridge_process, self.ROSBRIDGE_TIMEOUT):
                self.logger.info("✅ ROS Bridge started on ws://aurora-usb:9090")
                threading.Thread(target=self._log_rosbridge_output, daemon=True).start()
                return True
            else:
                stderr_output = self.rosbridge_process.stderr.read() if self.rosbridge_process.stderr else ""
                self.logger.error("❌ ROS Bridge failed to start")
                if stderr_output:
                    self.logger.error(f"   Error: {stderr_output}")
                return False
        except FileNotFoundError:
            self.logger.error("❌ rosbridge_server not found!")
            self.logger.error("   Install with: sudo apt install ros-humble-rosbridge-suite")
            return False
        except Exception as e:
            self.logger.error(f"❌ ROS Bridge error: {e}")
            return False

    def _launch_flask(self) -> bool:
        try:
            flask_locations = [
                self.base_dir / "flask_server" / "flask_server.py",
                self.base_dir / "flask_server.py",
            ]
            flask_script = flask_dir = None
            for loc in flask_locations:
                if loc.exists():
                    flask_script = loc
                    flask_dir    = loc.parent
                    break

            if not flask_script:
                self.logger.error("❌ Flask server not found in:")
                for loc in flask_locations:
                    self.logger.error(f"   - {loc}")
                return False

            self.logger.info(f"🌸 Starting Flask web server from {flask_dir}...")
            self.flask_process = subprocess.Popen(
                ["python3", str(flask_script)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                cwd=str(flask_dir),
                preexec_fn=os.setsid
            )
            if self._wait_for_process(self.flask_process, self.FLASK_TIMEOUT):
                self.logger.info("✅ Flask server started on http://localhost:9413")
                threading.Thread(target=self._log_flask_output, daemon=True).start()
                return True
            else:
                stderr_output = self.flask_process.stderr.read() if self.flask_process.stderr else ""
                self.logger.error("❌ Flask server failed to start")
                if stderr_output:
                    self.logger.error(f"   Error: {stderr_output}")
                return False
        except Exception as e:
            self.logger.error(f"❌ Flask server error: {e}")
            return False

    def _launch_mcp(self) -> bool:
        try:
            mcp_locations = [
                self.base_dir / "mcp_server" / "searxng-mcp-server" / "searxng-mcp-server.js",
                self.base_dir / "searxng-mcp-server.js",
                "/usr/local/bin/searxng-mcp-server",
            ]
            mcp_script = None
            for loc in mcp_locations:
                if Path(loc).exists():
                    mcp_script = loc
                    break

            if not mcp_script:
                self.logger.warning("⚠️  SearXNG MCP server not found, skipping")
                return False

            self.logger.info("🔍 Starting SearXNG MCP server...")
            env = os.environ.copy()
            env.update({
                'SEARXNG_URL':        os.getenv('SEARXNG_URL',        'http://127.0.0.1:8080'),
                'MAX_CONTENT_LENGTH': os.getenv('MAX_CONTENT_LENGTH', '3000'),
                'REQUEST_TIMEOUT':    os.getenv('REQUEST_TIMEOUT',    '10000'),
            })
            cmd = ["node", str(mcp_script)] if str(mcp_script).endswith('.js') else [str(mcp_script)]
            self.mcp_process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=env,
                cwd=str(self.base_dir),
                preexec_fn=os.setsid
            )
            if self._wait_for_process(self.mcp_process, self.MCP_TIMEOUT):
                self.logger.info("✅ SearXNG MCP server started")
                threading.Thread(target=self._log_mcp_output, daemon=True).start()
                return True
            else:
                stderr_output = self.mcp_process.stderr.read() if self.mcp_process.stderr else ""
                self.logger.error("❌ SearXNG MCP server failed to start")
                if stderr_output:
                    self.logger.error(f"   Error: {stderr_output}")
                return False
        except Exception as e:
            self.logger.error(f"❌ MCP server error: {e}")
            return False

    def _launch_skills(self) -> bool:
        try:
            skills_locations = [
                self.base_dir / "nature_skills_server.py",
                self.base_dir / "mcp_server" / "nature_skills_server.py",
                self.base_dir / "mcp_server" / "skills_server" / "nature_skills_server.py",
            ]
            skills_script = None
            for loc in skills_locations:
                if loc.exists():
                    skills_script = loc
                    break

            if not skills_script:
                self.logger.warning("⚠️  Nature Skills MCP server not found, skipping")
                return False

            self.logger.info("🌲 Starting Nature Skills MCP server...")
            env = os.environ.copy()
            env.update({
                'USER_LATITUDE':  os.getenv('USER_LATITUDE',  '49.2827'),
                'USER_LONGITUDE': os.getenv('USER_LONGITUDE', '-123.1207'),
                'USER_LOCATION':  os.getenv('USER_LOCATION',  'Vancouver, BC, Canada'),
                'PYTHONUNBUFFERED': '1',
            })
            self.skills_process = subprocess.Popen(
                ["python3", str(skills_script)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=env,
                cwd=str(self.base_dir),
                preexec_fn=os.setsid
            )
            if self._wait_for_process(self.skills_process, self.SKILLS_TIMEOUT):
                self.logger.info("✅ Nature Skills MCP server started")
                threading.Thread(target=self._log_skills_output, daemon=True).start()
                return True
            else:
                stderr_output = self.skills_process.stderr.read() if self.skills_process.stderr else ""
                self.logger.error("❌ Nature Skills MCP server failed to start")
                if stderr_output:
                    self.logger.error(f"   Error: {stderr_output}")
                return False
        except Exception as e:
            self.logger.error(f"❌ Nature Skills MCP server error: {e}")
            return False

    # -------------------------------------------------------------------------
    # PARALLEL startup - launch all servers at once, collect results
    # -------------------------------------------------------------------------
    def start_all(self) -> dict:
        """
        Launch all servers in parallel using threads.
        Returns dict of {name: bool} results.
        Total startup time = slowest server, not sum of all servers.
        """
        results = {}

        def run(name, launcher):
            results[name] = launcher()

        threads = [
            threading.Thread(target=run, args=("rosbridge", self._launch_rosbridge), daemon=True),
            threading.Thread(target=run, args=("flask",     self._launch_flask),     daemon=True),
            threading.Thread(target=run, args=("mcp",       self._launch_mcp),       daemon=True),
            threading.Thread(target=run, args=("skills",    self._launch_skills),    daemon=True),
        ]

        for t in threads:
            t.start()
        for t in threads:
            t.join()  # wait for all launchers to finish

        return results

    # -------------------------------------------------------------------------
    # PARALLEL shutdown - terminate all at once, then wait
    # -------------------------------------------------------------------------
    def stop_all(self):
        """
        Kill all servers - with timing debug to find bottleneck
        """
        t0 = time.monotonic()
        self.logger.info(f"[STOP] stop_all() started at t=0")

        processes = {
            "ROS Bridge":  self.rosbridge_process,
            "Flask":       self.flask_process,
            "SearXNG MCP": self.mcp_process,
            "Skills MCP":  self.skills_process,
        }

        # Log which processes actually exist
        for name, proc in processes.items():
            if proc is None:
                self.logger.info(f"[STOP] {name}: not started (None)")
            elif proc.poll() is not None:
                self.logger.info(f"[STOP] {name}: already dead (returncode={proc.poll()})")
            else:
                self.logger.info(f"[STOP] {name}: ALIVE pid={proc.pid}")

        self.logger.info(f"[STOP] t={time.monotonic()-t0:.2f}s - sending SIGKILL to all groups...")

        # SIGKILL entire process group directly - no SIGTERM polite wait
        for name, proc in processes.items():
            if proc and proc.poll() is None:
                try:
                    pgid = os.getpgid(proc.pid)
                    self.logger.info(f"[STOP] killpg({pgid}, SIGKILL) for {name}")
                    os.killpg(pgid, signal.SIGKILL)
                    self.logger.info(f"[STOP] {name} SIGKILL sent t={time.monotonic()-t0:.2f}s")
                except ProcessLookupError:
                    self.logger.info(f"[STOP] {name} already gone")
                except Exception as e:
                    self.logger.warning(f"[STOP] {name} killpg failed: {e}, trying proc.kill()")
                    try:
                        proc.kill()
                    except Exception as e2:
                        self.logger.warning(f"[STOP] {name} proc.kill() also failed: {e2}")

        self.logger.info(f"[STOP] t={time.monotonic()-t0:.2f}s - waiting for confirmations...")

        logger_ref = self.logger

        def wait_dead(name, proc):
            tw = time.monotonic()
            if proc is None:
                return
            try:
                proc.wait(timeout=1.5)
                logger_ref.info(f"[STOP] {name} confirmed dead in {time.monotonic()-tw:.2f}s")
            except Exception as e:
                logger_ref.warning(f"[STOP] {name} still alive after 1.5s wait: {e}")

        waiters = [
            threading.Thread(target=wait_dead, args=(name, proc), daemon=True)
            for name, proc in processes.items()
            if proc is not None
        ]
        for t in waiters:
            t.start()
        for t in waiters:
            t.join(timeout=2.0)

        self.logger.info(f"[STOP] stop_all() DONE at t={time.monotonic()-t0:.2f}s")

    def check_health(self) -> dict:
        return {
            "rosbridge": self.rosbridge_process.poll() is None if self.rosbridge_process else False,
            "flask":     self.flask_process.poll()     is None if self.flask_process     else False,
            "mcp":       self.mcp_process.poll()       is None if self.mcp_process       else False,
            "skills":    self.skills_process.poll()    is None if self.skills_process    else False,
        }

    # -------------------------------------------------------------------------
    # Log capture threads
    # -------------------------------------------------------------------------
    def _log_rosbridge_output(self):
        try:
            for line in self.rosbridge_process.stderr:
                if line.strip():
                    self.logger.info(f"[ROS Bridge] {line.strip()}")
        except Exception:
            pass

    def _log_flask_output(self):
        try:
            for line in self.flask_process.stderr:
                if line.strip():
                    self.logger.info(f"[Flask] {line.strip()}")
        except Exception:
            pass

    def _log_mcp_output(self):
        try:
            for line in self.mcp_process.stderr:
                if line.strip():
                    self.logger.info(f"[MCP] {line.strip()}")
        except Exception:
            pass

    def _log_skills_output(self):
        try:
            for line in self.skills_process.stderr:
                if line.strip():
                    self.logger.info(f"[Skills] {line.strip()}")
        except Exception:
            pass


class Igniter(Node):
    """
    ROS2 node that bootstraps all external servers for the robot.
    """
    def __init__(self):
        super().__init__('igniter')
        self.eee = get_logger("SCS.IGNITER")

        self.server_manager = ServerManager(self.eee)
        self._start_servers()

        # Health check every 30 seconds
        self.create_timer(30.0, self._health_check)

    def _start_servers(self):
        self.eee.info("=" * 60)
        self.eee.info("STARTING EXTERNAL SERVERS (parallel)")
        self.eee.info("=" * 60)

        results = self.server_manager.start_all()

        self.eee.info("=" * 60)
        self.eee.info("SERVER STARTUP SUMMARY")
        self.eee.info("=" * 60)
        self.eee.info(f"ROS Bridge (WebSocket):    {'✅ RUNNING' if results.get('rosbridge') else '❌ FAILED'}")
        self.eee.info(f"Flask Web Server:          {'✅ RUNNING' if results.get('flask')     else '❌ FAILED'}")
        self.eee.info(f"SearXNG MCP Server:        {'✅ RUNNING' if results.get('mcp')       else '❌ FAILED'}")
        self.eee.info(f"Nature Skills MCP Server:  {'✅ RUNNING' if results.get('skills')    else '❌ FAILED'}")
        self.eee.info("=" * 60)

        if results.get('rosbridge'):
            self.eee.info("🌉 WebSocket available at: ws://localhost:9090")
        else:
            self.eee.error("⚠️  CRITICAL: Web interface will NOT work without ROS Bridge!")

        if results.get('flask'):
            self.eee.info("🌸 Access Grace UI at: http://localhost:9413")

        if not any(results.values()):
            self.eee.warning("⚠️  No external servers started - robot will run in basic mode")

    def _health_check(self):
        health = self.server_manager.check_health()
        if not health['rosbridge']:
            self.eee.warning("⚠️  ROS Bridge is down - web interface will not work!")
        if not health['flask']:
            self.eee.warning("⚠️  Flask server is down")

    def shutdown(self):
        self.eee.info("=" * 60)
        self.eee.info("SHUTDOWN SEQUENCE")
        self.eee.info("=" * 60)
        self.server_manager.stop_all()
        print("[SHUTDOWN] calling EEEAggregator.shutdown() with 1s timeout")
        eee_thread = threading.Thread(target=EEEAggregator.shutdown, daemon=True)
        eee_thread.start()
        eee_thread.join(timeout=1.0)
        if eee_thread.is_alive():
            print("[SHUTDOWN] EEEAggregator.shutdown() timed out - skipping")
        print("[SHUTDOWN] EEE done, calling rclpy.shutdown()")
        self.eee.info("=" * 60)
        self.eee.info("SHUTDOWN COMPLETE")
        self.eee.info("=" * 60)


#--- THE BOOTSTRAP (IGNITER) ---

def main(args=None):
    rclpy.init(args=args)
    
    bridge_node  = EEEROSBridge()
    igniter_node = Igniter()
    
    executor = MultiThreadedExecutor()
    executor.add_node(bridge_node)
    executor.add_node(igniter_node)

    logger = get_logger("SCS.BOOTSTRAP")

    _shutdown_called = False

    def handle_shutdown():
        nonlocal _shutdown_called
        if _shutdown_called:
            logger.info("[SHUTDOWN] Already called, skipping")
            return
        _shutdown_called = True

        t0 = time.monotonic()
        logger.info(f"[SHUTDOWN] Starting at t=0")

        # Step 1: Kill child processes
        logger.info(f"[SHUTDOWN] t={time.monotonic()-t0:.2f}s calling stop_all()")
        igniter_node.server_manager.stop_all()
        logger.info(f"[SHUTDOWN] t={time.monotonic()-t0:.2f}s stop_all() returned")

        # Step 2: EEE logger
        logger.info(f"[SHUTDOWN] t={time.monotonic()-t0:.2f}s calling EEEAggregator.shutdown()")
        EEEAggregator.shutdown()
        logger.info(f"[SHUTDOWN] t={time.monotonic()-t0:.2f}s EEEAggregator done")

        # Step 3: ROS shutdown
        logger.info(f"[SHUTDOWN] t={time.monotonic()-t0:.2f}s calling rclpy.shutdown()")
        try:
            executor.shutdown(wait=False)
            rclpy.shutdown()
        except Exception as e:
            logger.info(f"[SHUTDOWN] rclpy.shutdown() exception (ok): {e}")
        logger.info(f"[SHUTDOWN] t={time.monotonic()-t0:.2f}s COMPLETE")

    def handle_sigterm(signum, frame):
        logger.warning(f"🛑 Signal {signum} (SIGTERM) received!")
        handle_shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, handle_sigterm)
    signal.signal(signal.SIGINT,  handle_sigterm)
    
    try:
        executor.spin()
    except Exception as e:
        logger.error(f"😵 Robot Brain Error: {e}")
    finally:
        handle_shutdown()

if __name__ == "__main__":
    main()