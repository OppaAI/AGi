#!/usr/bin/env bash
set -euo pipefail

# Stop and remove existing container
+docker stop cosmos-reason2-2b >/dev/null 2>&1 || true
+docker rm   cosmos-reason2-2b >/dev/null 2>&1 || true

# Clear caches
sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'

# Run new container
docker run -d \
  --restart unless-stopped \
  --network host \
  --shm-size=8g \
  --ulimit memlock=-1 \
  --ulimit stack=67108864 \
  --runtime=nvidia \
  --name=cosmos-reason2-2b \
  -v $HOME/.cache/huggingface:/root/.cache/huggingface \
  -e HF_HOME=/root/.cache/huggingface \
  embedl/vllm:latest-jetson-orin-flashhead \
  vllm serve "embedl/Cosmos-Reason2-2B-W4A16-Edge2-FlashHead" \
    --max-model-len 1920 \
    --gpu-memory-utilization 0.45 \
    --max-num-seqs 1 \
    --mm-processor-kwargs '{"max_pixels":256000}' \
    --trust-remote-code
#   --enforce_eager

echo "⏳ Cosmos loading (~3 minutes)..."
echo "Monitor: docker logs -f cosmos-reason2-2b"
echo "Ready when you see: Application startup complete"
