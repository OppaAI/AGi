"""
Igniter (Bootstrap) Module

Welcome to my Igniter Module!
This module handles the ignition sequence (bootstrap) of the robot's system.

Startup sequence:
    1. TallEEE: Centralized Triple-Aggregation Logger & Ledger Emergency and Exception Entry
    2. ROS Bridge WebSocket Server: Enables web interface communication
    3. Nature Skills MCP Server: Outdoor/nature exploration tools
    4. ROS2 Bridge: Connects all components

Note: Web interface served by Caddy (systemd service - no startup needed)
Note: Web search handled by Ollama Cloud API (no SearXNG/MCP required)
      Set OLLAMA_API_KEY in .env to enable web search.
      To re-enable SearXNG: docker compose up -d searxng
"""
# System modules
from enum import Enum
import json
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


# ─────────────────────────────────────────────────────────────────────────────
# NOTE: MCPSearchProxy removed — web search now via Ollama Cloud API in cnc.py
# To restore SearXNG: docker compose up -d searxng
#   and recover MCPSearchProxy + _launch_mcp() from git history.
# ─────────────────────────────────────────────────────────────────────────────

class _MCPSearchProxy_DISABLED:
    """
    Thin HTTP proxy that sits in front of the igniter-owned MCP process.

    cnc.py calls  POST http://127.0.0.1:51622/search  {"query": "...", "num_results": 5}
    This class forwards the request over the existing mcp_process stdin/stdout
    JSON-RPC channel and returns full-content results as JSON.

    Why HTTP and not direct stdin/stdout sharing?
    - The mcp_process belongs to igniter; passing the raw Popen handle into
      cnc would create a shared-mutable-state nightmare across two ROS nodes.
    - A local loopback HTTP call costs ~0.2 ms — negligible vs a web search.
    - cnc.web_search() stays a simple requests.post(), easy to test/mock.
    """

    PORT = 51622
    REQUEST_TIMEOUT = 30   # seconds to wait for MCP response

    def __init__(self, logger):
        self.logger = logger
        self._mcp_process = None          # set by attach() once igniter starts MCP
        self._mcp_lock = threading.Lock() # serialize stdin/stdout access
        self._server: HTTPServer | None = None
        self._thread: threading.Thread | None = None

    def attach(self, mcp_process) -> bool:
        """Point the proxy at igniter's already-running MCP process."""
        if mcp_process is None or mcp_process.poll() is not None:
            self.logger.error("❌ MCPSearchProxy: MCP process is dead or None — cannot attach")
            return False
        self._mcp_process = mcp_process
        self.logger.info("✅ MCPSearchProxy: attached to MCP process")
        return True

    def start(self):
        """Start the HTTP listener in a daemon thread."""
        proxy_ref = self   # captured by handler class below

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                if self.path != "/search":
                    self.send_response(404)
                    self.end_headers()
                    return
                try:
                    length = int(self.headers.get("Content-Length", 0))
                    body   = json.loads(self.rfile.read(length))
                    query       = body.get("query", "")
                    num_results = int(body.get("num_results", 5))
                    results = proxy_ref._search(query, num_results)
                    payload = json.dumps(results).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                except Exception as e:
                    proxy_ref.logger.error(f"MCPSearchProxy handler error: {e}")
                    self.send_response(500)
                    self.end_headers()

            def log_message(self, *args):
                pass  # silence HTTP access log — already logged at search level

        self._server = HTTPServer(("127.0.0.1", self.PORT), Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True, name="mcp-proxy"
        )
        self._thread.start()
        self.logger.info(f"✅ MCPSearchProxy listening on http://127.0.0.1:{self.PORT}/search")

    def stop(self):
        if self._server:
            self._server.shutdown()
            self.logger.info("🛑 MCPSearchProxy stopped")

    # ------------------------------------------------------------------ #
    #  Internal: talk to the MCP process over JSON-RPC stdin/stdout       #
    # ------------------------------------------------------------------ #
    def _search(self, query: str, num_results: int) -> list:
        with self._mcp_lock:
            if self._mcp_process is None or self._mcp_process.poll() is not None:
                self.logger.error("MCPSearchProxy: MCP process gone during search")
                return []
            try:
                request = {
                    "jsonrpc": "2.0",
                    "id": 1,
                    "method": "tools/call",
                    "params": {
                        "name": "search_web",
                        "arguments": {
                            "query": query,
                            "num_results": min(num_results, 20),
                            "fetch_content": True,
                        },
                    },
                }
                self._mcp_process.stdin.write(json.dumps(request) + "\n")
                self._mcp_process.stdin.flush()

                deadline = time.monotonic() + self.REQUEST_TIMEOUT
                while time.monotonic() < deadline:
                    ready = select.select(
                        [self._mcp_process.stdout], [], [],
                        deadline - time.monotonic()
                    )[0]
                    if not ready:
                        break
                    line = self._mcp_process.stdout.readline()
                    if not line:
                        break
                    try:
                        response = json.loads(line)
                        if response.get("id") == 1:
                            return self._parse(response)
                    except json.JSONDecodeError:
                        continue  # skip non-JSON debug lines

                self.logger.error("MCPSearchProxy: MCP search timed out")
                return []

            except Exception as e:
                self.logger.error(f"MCPSearchProxy._search error: {e}")
                return []

    def _parse(self, response: dict) -> list:
        """Parse the MCP markdown-formatted response into a results list."""
        try:
            content = response.get("result", {}).get("content", [])
            if not content:
                return []
            text = content[0].get("text", "")

            results, current, content_lines, in_content = [], {}, [], False

            def _save(cur, lines):
                if cur and "title" in cur:
                    cur["full_content"]     = "\n".join(lines)
                    cur["has_full_content"] = bool(lines)
                    cur.setdefault("url", "")
                    cur.setdefault("snippet", "")
                    cur.setdefault("engine", "unknown")
                    results.append(cur)

            for line in text.split("\n"):
                line = line.strip()
                if line.startswith("## [") and "]" in line:
                    _save(current, content_lines)
                    parts = line.split("]", 1)
                    current = {
                        "number":           len(results) + 1,
                        "title":            parts[1].strip() if len(parts) > 1 else "No title",
                        "url":              "",
                        "snippet":          "",
                        "engine":           "unknown",
                        "full_content":     "",
                        "has_full_content": False,
                    }
                    content_lines, in_content = [], False
                elif line.startswith("**URL:**"):
                    current["url"]     = line.replace("**URL:**", "").strip()
                elif line.startswith("**Snippet:**"):
                    current["snippet"] = line.replace("**Snippet:**", "").strip()
                elif line.startswith("**Source:**"):
                    current["engine"]  = line.replace("**Source:**", "").strip()
                elif line.startswith("**Full Content:**"):
                    in_content = True
                elif line == "---":
                    in_content = False
                elif in_content and line:
                    content_lines.append(line)

            _save(current, content_lines)
            self.logger.info(f"MCPSearchProxy: parsed {len(results)} results")
            return results

        except Exception as e:
            self.logger.error(f"MCPSearchProxy._parse error: {e}")
            return []


class ServerManager:
    """
    Manages external servers (ROS Bridge, Skills) for the robot.
    Web interface served by Caddy (systemd - always on, not managed here).
    Web search handled by Ollama Cloud API in cnc.py (no SearXNG needed).
    Starts all servers in parallel and polls for readiness instead of sleeping.
    """
    # How long to poll for each server before giving up (seconds)
    ROSBRIDGE_TIMEOUT = 5
    MCP_TIMEOUT       = 3
    SKILLS_TIMEOUT    = 3
    POLL_INTERVAL     = 0.1  # Check every 100ms

    def __init__(self, logger):
        self.logger = logger
        self.rosbridge_process = None
        # mcp_process removed — search handled by Ollama Cloud API in cnc.py
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

    # _launch_mcp() removed — SearXNG MCP disabled.
    # To re-enable: docker compose up -d searxng
    # and restore _launch_mcp() from git history.

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
            # "SearXNG MCP": disabled — re-enable with docker compose up -d searxng
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
            # mcp removed — search via Ollama Cloud API
            "skills":    self.skills_process.poll()    is None if self.skills_process     else False,
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
        self._health_timer = self.create_timer(30.0, self._health_check)

    def _start_servers(self):
        self.eee.info("=" * 60)
        self.eee.info("STARTING EXTERNAL SERVERS (parallel)")
        self.eee.info("=" * 60)

        results = self.server_manager.start_all()

        self.eee.info("=" * 60)
        self.eee.info("SERVER STARTUP SUMMARY")
        self.eee.info("=" * 60)
        self.eee.info(f"ROS Bridge (WebSocket):    {'✅ RUNNING' if results.get('rosbridge') else '❌ FAILED'}")
        self.eee.info(f"Nature Skills MCP Server:  {'✅ RUNNING' if results.get('skills')    else '❌ FAILED'}")
        self.eee.info(f"Web Search:                ✅ Ollama Cloud API (OLLAMA_API_KEY in .env)")
        self.eee.info(f"Web Interface (Caddy):     ✅ systemd service (always on)")
        self.eee.info("=" * 60)

        if results.get('rosbridge'):
            self.eee.info("🌉 WebSocket available at: ws://localhost:9090")
        else:
            self.eee.error("⚠️  CRITICAL: Web interface will NOT work without ROS Bridge!")

        self.eee.info("🌐 Access Grace UI at: http://localhost:9413")

        if not any(results.values()):
            self.eee.warning("⚠️  No external servers started - robot will run in basic mode")

    def _health_check(self):
        health = self.server_manager.check_health()
        # Only warn — don't restart, that's out of scope here
        if not health['rosbridge']:
            self.eee.warning("⚠️  ROS Bridge is down - web interface will not work!")
        # Cancel timer if everything is permanently dead (optional but clean)
        if not any(health.values()):
            self.eee.warning("⚠️  All servers down - stopping health check timer")
            self._health_timer.cancel()   # store timer ref: self._health_timer = self.create_timer(...)

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
    
    executor = MultiThreadedExecutor(num_threads=2)
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
