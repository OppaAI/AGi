"""
test_encoding_engine.py
=======================
AuRoRA · Encoding Engine diagnostic

Mirrors exactly what EncodingEngine._load_engine() does in msb.py.
Run this under the ROS2 environment to isolate the failure:

    source /opt/ros/humble/setup.bash
    source ~/AGi/AuRoRA/install/setup.bash
    python3 test_encoding_engine.py

If this fails but bare python3 works, the issue is ROS2 sys.path ordering.
"""

import sys
import traceback

MODEL = "BAAI/bge-base-en-v1.5"
TEST_CUE = "what did we talk about yesterday"

print(f"\n{'═' * 60}")
print(f"  AuRoRA · Encoding Engine diagnostic")
print(f"  Model : {MODEL}")
print(f"{'═' * 60}\n")

# ── 1. sys.path ───────────────────────────────────────────────
print("── sys.path ──────────────────────────────────────────────")
for i, p in enumerate(sys.path):
    print(f"  [{i:02d}] {p}")

# ── 2. huggingface_hub ────────────────────────────────────────
print("\n── huggingface_hub ───────────────────────────────────────")
try:
    import huggingface_hub
    print(f"  version : {huggingface_hub.__version__}")
    print(f"  path    : {huggingface_hub.__file__}")
except Exception as e:
    print(f"  FAILED: {e}")
    traceback.print_exc()

# ── 3. huggingface_hub.utils ──────────────────────────────────
print("\n── huggingface_hub.utils ─────────────────────────────────")
try:
    import huggingface_hub.utils as hfu
    print(f"  path    : {hfu.__file__}")
    print(f"  OK")
except Exception as e:
    print(f"  FAILED: {e}")
    traceback.print_exc()

# ── 4. sentence_transformers ──────────────────────────────────
print("\n── sentence_transformers ─────────────────────────────────")
try:
    import sentence_transformers
    print(f"  version : {sentence_transformers.__version__}")
    print(f"  path    : {sentence_transformers.__file__}")
except Exception as e:
    print(f"  FAILED: {e}")
    traceback.print_exc()

# ── 5. SentenceTransformer load ───────────────────────────────
print(f"\n── SentenceTransformer('{MODEL}') ─────────────────────────")
try:
    from sentence_transformers import SentenceTransformer
    model = SentenceTransformer(MODEL)
    print(f"  loaded ✓")
except Exception as e:
    print(f"  FAILED: {e}")
    traceback.print_exc()
    sys.exit(1)

# ── 6. encode ─────────────────────────────────────────────────
print(f"\n── encode('{TEST_CUE}') ───────────────────────────────────")
try:
    vec = model.encode(TEST_CUE).tolist()
    print(f"  dim     : {len(vec)}")
    print(f"  sample  : {vec[:4]}")
    print(f"  encode ✓")
except Exception as e:
    print(f"  FAILED: {e}")
    traceback.print_exc()
    sys.exit(1)

print(f"\n{'═' * 60}")
print(f"  Encoding Engine — all checks passed ✓")
print(f"{'═' * 60}\n")
