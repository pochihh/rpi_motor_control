# rpi_motor_control

Minimal modular C++ codebase for ~1 kHz motor control on Raspberry Pi (Debian 12 + PREEMPT_RT).  
Encoders via **libgpiod** edge events; motor drive via **Pololu Motoron M3H256** over **I²C**.

---

## Structure

```
rpi_motor_control/
├─ .devcontainer        # Docker files
├─ README.md
├─ Makefile
├─ main.cpp
├─ util.h / util.cpp          # RT helper + ThreadMonitor (utilization & deadline stats)
├─ PID.h / PID.cpp
├─ Encoder.h / Encoder.cpp    # ONE encoder, interrupt-driven, internal event thread
├─ Motoron.h / Motoron.cpp
├─ Motor.h / Motor.cpp        # ONE motor, owns an Encoder
```


---

## Threads

- **1 kHz control** → polls encoders (bounded), runs PID, sends Motoron command  
- **200 Hz kinematics** → updates setpoints (`Motor::setReference(revs)`)  
- **Housekeeping** → prints positions/status

> Run with `sudo` for real-time scheduling (SCHED_FIFO).

---

## Requirements

```bash
sudo apt-get update
sudo apt-get install -y g++ make libgpiod-dev i2c-tools
```

Enable/verify I²C:

```bash
ls /dev/i2c-1
sudo i2cdetect -y 1    # expect 0x10 for Motoron default address
```

Check GPIO lines:

```bash
gpioinfo               # confirm chosen lines exist & no conflicting consumer
```

---

## Build & Run

```bash
make
sudo ./motor_ctrl
```

Stop with **Ctrl-C** (avoid Ctrl-Z; it suspends and keeps GPIO lines busy).

---

## Quick Configuration (edit `main.cpp`)

- **Encoder pins**: `enc.add(A_line, B_line)` (defaults: (5,6), (12,13), (16,17))  
- **Counts per rev**: `m.setCountsPerRev(4096)` (1024 CPR × 4)  
- **Gear ratio**: `m.setGear(1.0)` (>1 means reduction)  
- **PID gains**: `m.setPID(Kp, Ki, Kd)` (start small; Ki=0 initially)  
- **Motoron address**: `Motoron("/dev/i2c-1", 0x10)`

---

## Notes & Tips

- Ensure encoder signals are **3.3 V** (level-shift from 5 V if needed).  
- Use twisted pairs/shielding; add pull-ups/Schmitt buffers for noisy lines.  
- On exit or when disabled, outputs **coast** (safe stop).

### Troubleshooting
- **Resource busy**: kill any suspended old run (`ps … | grep motor_ctrl`, then `sudo kill -INT/-TERM/-KILL <PID>`).  
- **No I²C device**: check wiring/power and that `i2cdetect` shows `0x10`.  
- **Jitter**: run as root, close background apps, consider CPU isolation/pinning later.

---

## What’s next

- Add CRC & status reads for Motoron.  
- JSON/YAML config for pins/CPR/gains without recompiling.  
- Watchdogs & CPU affinity for the 1 kHz thread.
