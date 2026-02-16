"""
Igniter (Bootstrap) Module

Welcome to my Igniter Module!
This module handles the ignition sequence (bootstrap) of the robot's system.

Startup sequence:
    1. TallEEE: Centralized Triple-Aggregation Logger & Ledger Emergency and Exception Entry
    2. Flask Web Server: Serves the Grace web interface
    3. SearXNG MCP Server: Enhanced web search with full content extraction
    4. ROS2 Bridge: Connects all components
"""
# System modules
from enum import Enum
import subprocess
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
    Manages external servers (Flask, MCP) for the robot
    """
    def __init__(self, logger):
        self.logger = logger
        self.flask_process = None
        self.mcp_process = None
        self.base_dir = Path.home() / "AGi"
        
    def start_flask_server(self) -> bool:
        """Start Flask web server for Grace UI"""
        try:
            # Try multiple locations for Flask server
            flask_locations = [
                self.base_dir / "flask_server" / "flask_server.py",  # In subdirectory
                self.base_dir / "flask_server.py",                   # Main directory
            ]
            
            flask_script = None
            flask_dir = None
            for loc in flask_locations:
                if loc.exists():
                    flask_script = loc
                    flask_dir = loc.parent
                    break
            
            if not flask_script:
                self.logger.error(f"❌ Flask server not found in:")
                for loc in flask_locations:
                    self.logger.error(f"   - {loc}")
                return False
            
            self.logger.info(f"🌸 Starting Flask web server from {flask_dir}...")
            
            self.flask_process = subprocess.Popen(
                ["python3", str(flask_script)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                cwd=str(flask_dir)  # Run from Flask directory
            )
            
            # Wait a moment and check if it started
            time.sleep(2)
            
            if self.flask_process.poll() is None:
                self.logger.info("✅ Flask server started on http://localhost:9413")
                
                # Start thread to capture output
                threading.Thread(
                    target=self._log_flask_output,
                    daemon=True
                ).start()
                
                return True
            else:
                self.logger.error("❌ Flask server failed to start")
                return False
                
        except Exception as e:
            self.logger.error(f"❌ Flask server error: {e}")
            return False
    
    def start_mcp_server(self) -> bool:
        """Start SearXNG MCP server for enhanced search"""
        try:
            # Try to find the MCP server
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
                self.logger.warn("⚠️  MCP server not found, skipping")
                self.logger.warn("   Locations checked:")
                for loc in mcp_locations:
                    self.logger.warn(f"     - {loc}")
                return False
            
            self.logger.info("🔍 Starting SearXNG MCP server...")
            
            # Set environment variables
            env = os.environ.copy()
            env.update({
                'SEARXNG_URL': os.getenv('SEARXNG_URL', 'http://127.0.0.1:8080'),
                'MAX_CONTENT_LENGTH': os.getenv('MAX_CONTENT_LENGTH', '3000'),
                'REQUEST_TIMEOUT': os.getenv('REQUEST_TIMEOUT', '10000')
            })
            
            # Start MCP server
            if str(mcp_script).endswith('.js'):
                cmd = ["node", str(mcp_script)]
            else:
                cmd = [str(mcp_script)]
            
            self.mcp_process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=env,
                cwd=str(self.base_dir)
            )
            
            # Wait and check if it started
            time.sleep(1)
            
            if self.mcp_process.poll() is None:
                self.logger.info("✅ SearXNG MCP server started")
                
                # Start thread to capture output
                threading.Thread(
                    target=self._log_mcp_output,
                    daemon=True
                ).start()
                
                return True
            else:
                self.logger.error("❌ MCP server failed to start")
                return False
                
        except Exception as e:
            self.logger.error(f"❌ MCP server error: {e}")
            return False
    
    def _log_flask_output(self):
        """Log Flask server output"""
        if not self.flask_process:
            return
        
        for line in self.flask_process.stderr:
            if line.strip():
                self.logger.info(f"[Flask] {line.strip()}")
    
    def _log_mcp_output(self):
        """Log MCP server output"""
        if not self.mcp_process:
            return
        
        for line in self.mcp_process.stderr:
            if line.strip():
                self.logger.info(f"[MCP] {line.strip()}")
    
    def stop_all(self):
        """Stop all managed servers"""
        self.logger.info("Stopping servers...")
        
        if self.flask_process:
            self.logger.info("🛑 Stopping Flask server...")
            self.flask_process.terminate()
            try:
                self.flask_process.wait(timeout=5)
            except:
                self.flask_process.kill()
            self.logger.info("✅ Flask server stopped")
        
        if self.mcp_process:
            self.logger.info("🛑 Stopping MCP server...")
            self.mcp_process.terminate()
            try:
                self.mcp_process.wait(timeout=5)
            except:
                self.mcp_process.kill()
            self.logger.info("✅ MCP server stopped")
    
    def check_health(self) -> dict:
        """Check health of all servers"""
        return {
            "flask": self.flask_process.poll() is None if self.flask_process else False,
            "mcp": self.mcp_process.poll() is None if self.mcp_process else False
        }


class Igniter(Node):
    """
    This is the ROS 2 Wrapper. It acts as a bridge between ROS and your TALLEEE system.
    Also manages external servers (Flask, MCP) for the robot.
    """
    def __init__(self):
        super().__init__('igniter')
        self.eee = get_logger("SCS.IGNITER")
        
        # Server manager
        self.server_manager = ServerManager(self.eee)
        
        # Start servers
        self._start_servers()
        
        # Create health check timer (every 30 seconds)
        self.create_timer(30.0, self._health_check)
    
    def _start_servers(self):
        """Start all external servers"""
        self.eee.info("=" * 60)
        self.eee.info("STARTING EXTERNAL SERVERS")
        self.eee.info("=" * 60)
        
        # Start Flask server
        flask_ok = self.server_manager.start_flask_server()
        
        # Start MCP server
        mcp_ok = self.server_manager.start_mcp_server()
        
        # Summary
        self.eee.info("=" * 60)
        self.eee.info("SERVER STARTUP SUMMARY")
        self.eee.info("=" * 60)
        self.eee.info(f"Flask Web Server:    {'✅ RUNNING' if flask_ok else '❌ FAILED'}")
        self.eee.info(f"SearXNG MCP Server:  {'✅ RUNNING' if mcp_ok else '❌ FAILED'}")
        self.eee.info("=" * 60)
        
        if flask_ok:
            self.eee.info("🌐 Access Grace UI at: http://localhost:9413")
        
        if not flask_ok and not mcp_ok:
            self.eee.warn("⚠️  No external servers started - robot will run in basic mode")
    
    def _health_check(self):
        """Periodic health check of servers"""
        health = self.server_manager.check_health()
        
        # Log any failures
        if not health['flask']:
            self.eee.warn("⚠️  Flask server is down")
        
        if not health['mcp']:
            self.eee.warn("⚠️  MCP server is down")
    
    def shutdown(self):
        """Cleanup plugins and stop servers"""
        self.eee.info("=" * 60)
        self.eee.info("SHUTDOWN SEQUENCE")
        self.eee.info("=" * 60)
        
        # Stop external servers
        self.server_manager.stop_all()
        
        # Shutdown EEE
        EEEAggregator.shutdown()
        
        self.eee.info("=" * 60)
        self.eee.info("SHUTDOWN COMPLETE")
        self.eee.info("=" * 60)


#--- THE BOOTSTRAP (IGNITER) ---

def main(args=None):
    """
    This is the Ignition Sequence.
    
    Startup order:
    1. ROS2 initialization
    2. EEE Bridge (logging)
    3. Igniter (server management)
    4. Flask web server
    5. SearXNG MCP server
    6. Spin all nodes
    """
    rclpy.init(args=args)

    # 1. Instantiate both nodes
    # The Bridge handles the ROS network plumbing
    bridge_node = EEEROSBridge()

    # The Igniter handles the robot's logic, plugin management, and servers
    igniter_node = Igniter()
    
    # 2. Use the Executor to run them together
    # This is the "Party Bus" that carries both nodes
    executor = MultiThreadedExecutor()
    executor.add_node(bridge_node)
    executor.add_node(igniter_node)
    
    logger = get_logger("SCS.BOOTSTRAP")
    logger.info("=" * 60)
    logger.info("🤖 AGi ROBOT IGNITION SEQUENCE")
    logger.info("=" * 60)
    logger.info("Ignition sequence started. 🔥")
    logger.info("=" * 60)
    
    try:
        # This spins BOTH nodes simultaneously
        executor.spin()

    except KeyboardInterrupt:
        logger.warning("=" * 60)
        logger.warning("🛑 Shutdown requested by human meatbag.")
        logger.warning("=" * 60)
    finally:
        # Cleanup everything
        logger.info("Executing graceful shutdown...")
        
        executor.shutdown()
        igniter_node.shutdown()
        bridge_node.destroy_node()
        igniter_node.destroy_node()
        
        rclpy.shutdown()
        
        logger.info("✅ All systems offline. Goodbye! 👋")


if __name__ == "__main__":
    main()