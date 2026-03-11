"""
GRACE — Igniter
================
Startup orchestration for Grace.
Boots all systems in order before handing off to ROS2 spin.

Boot sequence:
  1. Check Jetson hardware (temps, GPU)
  2. Wait for vLLM / Cosmos to be ready
  3. Init memory (SQLite, RAG)
  4. Start CNS ROS2 node
  5. Announce online
  6. Spin until shutdown
  7. Graceful consolidation on exit

Usage:
  python3 igniter.py
"""

import logging
import signal
import sys
import time

import requests
import rclpy

from config import (
    VLLM_URL,
    COSMOS_MODEL,
    VLLM_WARMUP_RETRIES,
    VLLM_WARMUP_INTERVAL,
)
from cns_node import CNSNode

# ─── Logging ──────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(name)-20s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("grace.igniter")


# ─── Banner ───────────────────────────────────────────────────────────────────

BANNER = """
╔══════════════════════════════════════════════════════════╗
║          GRACE — Amazing Grace Infrastructure            ║
║     Generative Reasoning Agentic Cognitive Edge          ║
╚══════════════════════════════════════════════════════════╝
"""


# ─── Hardware check ───────────────────────────────────────────────────────────

def check_hardware() -> bool:
    """Check Jetson temps and GPU availability."""
    log.info("🔧 Checking hardware...")
    try:
        # Check CPU temp
        cpu_temp_path = "/sys/devices/virtual/thermal/thermal_zone0/temp"
        try:
            with open(cpu_temp_path) as f:
                cpu_temp = int(f.read().strip()) / 1000
            log.info(f"   CPU temp: {cpu_temp:.1f}°C")
            if cpu_temp > 85:
                log.warning(f"   ⚠️  CPU temp high: {cpu_temp:.1f}°C")
        except FileNotFoundError:
            log.info("   CPU temp: unavailable (not on Jetson?)")

        # Check GPU
        try:
            import subprocess
            result = subprocess.run(
                ["nvidia-smi", "--query-gpu=temperature.gpu,memory.used,memory.total",
                 "--format=csv,noheader,nounits"],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                parts = result.stdout.strip().split(",")
                gpu_temp = parts[0].strip()
                mem_used = parts[1].strip()
                mem_total = parts[2].strip()
                log.info(f"   GPU temp: {gpu_temp}°C  |  VRAM: {mem_used}MB / {mem_total}MB")
            else:
                log.info("   GPU: nvidia-smi unavailable")
        except Exception:
            log.info("   GPU: check skipped")

        log.info("✅ Hardware check passed")
        return True

    except Exception as e:
        log.error(f"Hardware check failed: {e}")
        return False


# ─── vLLM warmup ──────────────────────────────────────────────────────────────

def wait_for_vllm() -> bool:
    """Wait for vLLM to be ready, retrying up to VLLM_WARMUP_RETRIES times."""
    log.info(f"⏳ Waiting for vLLM at {VLLM_URL}...")

    for attempt in range(1, VLLM_WARMUP_RETRIES + 1):
        try:
            r = requests.get(f"{VLLM_URL}/v1/models", timeout=5)
            if r.status_code == 200:
                models = r.json().get("data", [])
                model_ids = [m.get("id", "") for m in models]
                log.info(f"✅ vLLM ready — models: {model_ids}")

                # Check our model is loaded
                if any(COSMOS_MODEL in mid for mid in model_ids):
                    log.info(f"✅ Cosmos model loaded: {COSMOS_MODEL}")
                else:
                    log.warning(f"⚠️  {COSMOS_MODEL} not found in vLLM. Available: {model_ids}")

                # Warmup ping — first inference is slow, do it now
                log.info("🔥 Warming up Cosmos inference...")
                _warmup_ping()
                return True

        except requests.exceptions.ConnectionError:
            log.info(f"   vLLM not ready (attempt {attempt}/{VLLM_WARMUP_RETRIES}) — retrying in {VLLM_WARMUP_INTERVAL}s...")
        except Exception as e:
            log.warning(f"   vLLM check error: {e}")

        time.sleep(VLLM_WARMUP_INTERVAL)

    log.error(f"❌ vLLM not available after {VLLM_WARMUP_RETRIES} attempts")
    log.error("   Start vLLM with: bash launch/cosmos.sh")
    return False


def _warmup_ping():
    """Send a short inference to warm up the model before first real call."""
    try:
        payload = {
            "model": COSMOS_MODEL,
            "messages": [
                {"role": "user", "content": "Hi"}
            ],
            "max_tokens": 8,
            "temperature": 0.1,
        }
        r = requests.post(f"{VLLM_URL}/v1/chat/completions", json=payload, timeout=30)
        if r.status_code == 200:
            log.info("✅ Cosmos warmup complete")
        else:
            log.warning(f"Warmup returned {r.status_code}")
    except Exception as e:
        log.warning(f"Warmup ping failed: {e}")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    print(BANNER)
    log.info("🚀 Igniter starting...")

    # ── Step 1: Hardware ──────────────────────────────────────────────────────
    if not check_hardware():
        log.warning("Hardware check failed — continuing anyway")

    # ── Step 2: vLLM ─────────────────────────────────────────────────────────
    if not wait_for_vllm():
        log.error("Cannot start without vLLM. Exiting.")
        sys.exit(1)

    # ── Step 3: ROS2 init ─────────────────────────────────────────────────────
    log.info("🤖 Initialising ROS2...")
    rclpy.init()

    # ── Step 4: CNS Node ──────────────────────────────────────────────────────
    log.info("🧠 Starting CNS Node...")
    node = CNSNode()

    # ── Step 5: Signal handlers ───────────────────────────────────────────────
    def _shutdown(sig, frame):
        log.info(f"\n⚡ Signal {sig} received — initiating graceful shutdown...")
        node.graceful_shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    # ── Step 6: Announce online ───────────────────────────────────────────────
    log.info("=" * 60)
    log.info("✅ GRACE is awake and listening")
    log.info("   ROS2 topic in:  /grace/input")
    log.info("   ROS2 topic out: /grace/output")
    log.info("   Send a message: ros2 topic pub /grace/input std_msgs/String '{data: \"Hello Grace\"}'")
    log.info("=" * 60)

    # ── Step 7: Spin ──────────────────────────────────────────────────────────
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        _shutdown(signal.SIGINT, None)


if __name__ == "__main__":
    main()
