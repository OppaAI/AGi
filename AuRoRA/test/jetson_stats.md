## ✅ All Accessible `jtop` Properties (Flattened List)

---

### 🧬 `board`（Boot-time / Static）

```text
board.hardware.Model
board.hardware.PartNumber_699
board.hardware.PNumber
board.hardware.Module
board.hardware.SoC
board.hardware.CUDA_Arch_BIN
**board.hardware.Serial_Number**
board.hardware.L4T
board.hardware.Jetpack

board.platform.Machine
board.platform.System
board.platform.Distribution
board.platform.Release
board.platform.Python

board.libraries.CUDA
board.libraries.OpenCV
board.libraries.OpenCV_Cuda
board.libraries.cuDNN
board.libraries.TensorRT
board.libraries.VPI
board.libraries.Vulkan
```

---

### 🧠 `cpu`

```text
**cpu.total.user**
cpu.total.nice
**cpu.total.system**
cpu.total.idle
```

Per-core:

```text
cpu.cpu[i].online
cpu.cpu[i].governor
cpu.cpu[i].freq.min
cpu.cpu[i].freq.max
cpu.cpu[i].freq.cur
cpu.cpu[i].info_freq.min
cpu.cpu[i].info_freq.max
cpu.cpu[i].info_freq.cur
cpu.cpu[i].idle_state.WFI
cpu.cpu[i].idle_state.c7
cpu.cpu[i].model
cpu.cpu[i].user
cpu.cpu[i].nice
cpu.cpu[i].system
cpu.cpu[i].idle
```

---

### 🧠 `memory`

```text
**memory.RAM.tot**
**memory.RAM.used**
memory.RAM.free
memory.RAM.buffers
memory.RAM.cached
memory.RAM.shared
memory.RAM.lfb
```

Swap:

```text
**memory.SWAP.tot**
**memory.SWAP.used**
memory.SWAP.cached
memory.SWAP.table[path].type
memory.SWAP.table[path].prio
memory.SWAP.table[path].size
memory.SWAP.table[path].used
memory.SWAP.table[path].boot
```

EMC:

```text
**memory.EMC.cur** (or load% from jetson.stats)
memory.EMC.max
memory.EMC.min
memory.EMC.override
memory.EMC.val
memory.EMC.online
```

---

### 🎮 `gpu`

```text
gpu.gpu.type
gpu.gpu.status.railgate
gpu.gpu.status.tpc_pg_mask
gpu.gpu.status.3d_scaling
**gpu.gpu.status.load**

gpu.gpu.freq.governor
gpu.gpu.freq.cur
gpu.gpu.freq.max
gpu.gpu.freq.min
gpu.gpu.freq.GPC[i]

gpu.gpu.power_control
```

---

### ⚙️ `engine`

```text
engine.APE.APE.online
engine.APE.APE.cur

engine.NVDEC.NVDEC.online
engine.NVDEC.NVDEC.cur

engine.NVJPG.NVJPG.online
engine.NVJPG.NVJPG.cur
engine.NVJPG.NVJPG1.online
engine.NVJPG.NVJPG1.cur

engine.OFA.OFA.online
engine.OFA.OFA.cur

engine.SE.SE.online
engine.SE.SE.cur

engine.VIC.VIC.online
engine.VIC.VIC.cur
```

---

### 🔌 `nvpmodel`

```text
nvpmodel   # e.g. "MAXN_SUPER"
```

---

### ⏱️ `jetson_clocks`

```text
jetson_clocks   # bool
```

---

### 💾 `disk`

```text
**disk.total**
**disk.used**
disk.available
disk.available_no_root
disk.unit
```

---

### 🌬️ `fan`

```text
**fan.pwmfan.speed[i]**
fan.pwmfan.rpm[i]
fan.pwmfan.profile
fan.pwmfan.governor
fan.pwmfan.control
```

---

### ⏳ `uptime`

```text
uptime   # timedelta / string
```

---

### 🌐 `local_interfaces`

```text
local_interfaces.hostname
local_interfaces.interfaces[name]
```

---

### 🌡️ `temperature`

```text
**temperature.cpu.temp**
temperature.cpu.online

**temperature.gpu.temp**
temperature.gpu.online

**temperature.soc0.temp**
temperature.soc0.online
**temperature.soc1.temp**
temperature.soc1.online
**temperature.soc2.temp**
temperature.soc2.online

**temperature.tj.temp**
temperature.tj.online

**temperature.cv0.temp**
temperature.cv0.online
**temperature.cv1.temp**
temperature.cv1.online
temperature.cv2.temp
temperature.cv2.online
```

---

### ⚡ `power`

Rails:

```text
VDD_CPU_GPU_CV / VDD_SOC
power.rail[name].volt 
power.rail[name].curr
power.rail[name].warn
power.rail[name].crit
**power.rail[name].power**
power.rail[name].online
power.rail[name].avg
```

Total:

```text
**power.tot.volt**
**power.tot.curr**
power.tot.warn
power.tot.crit
**power.tot.power**
power.tot.online
power.tot.avg
power.tot.name
```

---

+-------------------+-------------------------+-----------+----------------------------------------+
| Category          | Metric                  | Poll Level| Notes                                  |
+-------------------+-------------------------+-----------+----------------------------------------+
| CPU (MID)         | cpu.total.user          | MID       | User load can spike occasionally       |
|                   | cpu.total.system        | MID       | System load changes slowly             |
+-------------------+-------------------------+-----------+----------------------------------------+
| Memory (MID)      | memory.RAM.tot          | LO        | Static value                           |
|                   | memory.RAM.used         | MID       | Memory usage changes slowly            |
|                   | memory.SWAP.tot         | LO        | Static                                 |
|                   | memory.SWAP.used        | MID       | Track swap usage                       |
|                   | memory.EMC.cur          | MID       | EMC load changes with memory access    |
+-------------------+-------------------------+-----------+----------------------------------------+
| GPU (HI)          | gpu.gpu.status.load     | HI        | GPU load fluctuates moderately         |
+-------------------+-------------------------+-----------+----------------------------------------+
| Disk (LO)         | disk.total              | LO        | Static                                 |
|                   | disk.used               | LO        | Changes slowly                         |
+-------------------+-------------------------+-----------+----------------------------------------+
| Fan (MID)         | fan.pwmfan.speed[i]     | MID       | Fans adjust dynamically                |
+-------------------+-------------------------+-----------+----------------------------------------+
| Temperature  (HI) | temperature.cpu.temp    | HI        | CPU temp can rise quickly              |
|                   | temperature.gpu.temp    | HI        | GPU temp can spike                     |
|                   | temperature.soc0.temp   | HI        | SoC cores heat fast                    |
|                   | temperature.soc1.temp   | HI        | SoC cores heat fast                    |
|                   | temperature.soc2.temp   | HI        | SoC cores heat fast                    |
|                   | temperature.tj.temp     | HI        | Junction temp; critical                |
|                   | temperature.cv0.temp    | MID       | Camera sensor temp, slower             |
|                   | temperature.cv1.temp    | MID       | Only if used                           |
|                   | temperature.cv2.temp    | MID       | Only if used                           |
+-------------------+-------------------------+-----------+----------------------------------------+
| Power (Rails)(MID)| power.rail[name].power  | MID       | Track real-time power consumption      |
+-------------------+-------------------------+-----------+----------------------------------------+
| Power (Total)(MID)| power.tot.volt          | MID       | Voltage is stable                      |
|                   | power.tot.curr          | MID       | Current fluctuates with load           |
|                   | power.tot.power         | MID       | Combined load for health alerting      |
+-------------------+-------------------------+-----------+----------------------------------------+

