"""
AuRoRA — Igniter (Bootstrap)
=============================
Starts the full AuRoRA stack:
  1. EEE logging bridge (from scs package)
  2. CNSBridge — Grace's cognitive node

Replaces igniter_temp.py with a production-ready ignition sequence.

Usage:
    python3 igniter.py
    # or as a ROS2 node:
    ros2 run aurora igniter
"""

import sys
import logging

import rclpy
from rclpy.executors import MultiThreadedExecutor

log = logging.getLogger("aurora.igniter")


def main(args=None):
    # ── Basic logging ─────────────────────────────────────────────────────────
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    log.info("=" * 60)
    log.info("AuRoRA — IGNITION SEQUENCE 🚀")
    log.info("=" * 60)

    rclpy.init(args=args)

    nodes = []

    # ── EEE bridge (optional — graceful skip if scs not installed) ────────────
    eee_bridge = None
    try:
        from scs.eee_ros_bridge import EEEROSBridge
        eee_bridge = EEEROSBridge()
        nodes.append(eee_bridge)
        log.info("✅ EEE ROS bridge initialized")
    except ImportError:
        log.warning("⚠️  scs.eee_ros_bridge not found — EEE logging disabled")
    except Exception as e:
        log.error(f"❌ EEE bridge init failed: {e}")

    # ── CNS Bridge — Grace's cognitive core ───────────────────────────────────
    from cns_node import CNSBridge
    cns_node = CNSBridge()
    nodes.append(cns_node)

    # ── Executor ──────────────────────────────────────────────────────────────
    executor = MultiThreadedExecutor(num_threads=4)
    for node in nodes:
        executor.add_node(node)

    log.info("🍺 All nodes online — spinning")

    try:
        executor.spin()
    except KeyboardInterrupt:
        log.warning("🛑 Shutdown requested (Ctrl+C)")
    except Exception as e:
        log.error(f"😵 Executor error: {e}")
    finally:
        log.info("🔧 Graceful shutdown...")
        try:
            cns_node.shutdown()
        except Exception as e:
            log.error(f"CNS shutdown error: {e}")

        executor.shutdown()

        for node in nodes:
            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()

        log.info("💤 AuRoRA offline.")
        sys.exit(0)


if __name__ == "__main__":
    main()
