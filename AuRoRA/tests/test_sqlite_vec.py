#!/usr/bin/env python3
"""
sqlite_vec Jetson validation script.
Tests install, load, insert, and KNN search against EMC's expected schema.
"""

import struct
import sqlite3

ENCODING_DIM = 768


def random_vec(seed: int) -> list[float]:
    import math
    vec = [math.sin(i + seed) for i in range(ENCODING_DIM)]
    mag = math.sqrt(sum(v * v for v in vec))
    return [v / mag for v in vec]


def pack(vec: list[float]) -> bytes:
    return struct.pack(f"{len(vec)}f", *vec)


print("=" * 50)

# 1 — import
try:
    import sqlite_vec
    print("✅ sqlite_vec imported")
except ImportError as e:
    print(f"❌ Import failed: {e}")
    print("   Run: pip3 install sqlite-vec --break-system-packages")
    raise SystemExit(1)

# 2 — load into connection
conn = sqlite3.connect(":memory:")
conn.row_factory = sqlite3.Row
try:
    sqlite_vec.load(conn)
    print("✅ sqlite_vec loaded into connection")
except Exception as e:
    print(f"❌ Load failed: {e}")
    raise SystemExit(1)

# 3 — create virtual table matching EMC schema
conn.execute(f"""
    CREATE VIRTUAL TABLE episode_vectors USING vec0(
        encoding FLOAT[{ENCODING_DIM}]
    )
""")
print(f"✅ vec0 virtual table created  — dim={ENCODING_DIM}")

# 4 — insert test episodes
vectors = {i: random_vec(i) for i in range(10)}
for row_id, vec in vectors.items():
    conn.execute(
        "INSERT INTO episode_vectors (rowid, encoding) VALUES (?, ?)",
        [row_id, pack(vec)],
    )
conn.commit()
print(f"✅ Inserted {len(vectors)} test episodes")

# 5 — KNN search (top 3, matching EMC recall query)
query = random_vec(3)          # episode 3 should be the top hit
rows = conn.execute("""
    SELECT rowid, distance
    FROM episode_vectors
    WHERE encoding MATCH ?
      AND k = 3
    ORDER BY distance
""", [pack(query)]).fetchall()

print(f"✅ KNN search returned {len(rows)} results:")
for r in rows:
    marker = " ← exact match" if r["rowid"] == 3 else ""
    print(f"   rowid={r['rowid']}  dist={r['distance']:.6f}{marker}")

if rows and rows[0]["rowid"] == 3:
    print("✅ Top result is correct — sqlite_vec KNN is working")
else:
    print("⚠️  Top result unexpected — check dim or packing")
    raise SystemExit(1)
    
conn.close()
print("=" * 50)
print("✅ sqlite_vec is fully operational for EMC on this device")
