# Motor Test Scripts for Quadruped Robot

This directory contains test scripts for controlling the 12 Dynamixel MX-64 motors of the quadruped robot.

## Motor Layout

```
                        FRONT
            ┌─────────────────────────┐
            │                         │
            │      ┌───────────┐      │
     FL     │      │   BODY    │      │     FR
   ┌─────┐  │      │           │      │  ┌─────┐
   │  1  │──┼──────┤           ├──────┼──│ 10  │
   └──┬──┘  │      │           │      │  └──┬──┘
      │     │      └───────────┘      │     │
   ┌──┴──┐  │                         │  ┌──┴──┐
   │  2  │  │                         │  │ 11  │
   └──┬──┘  │                         │  └──┬──┘
      │     │                         │     │
   ┌──┴──┐  │                         │  ┌──┴──┐
   │  3  │  │                         │  │ 12  │
   └─────┘  │                         │  └─────┘
            │                         │
            │      ┌───────────┐      │
     RL     │      │   BODY    │      │     RR
   ┌─────┐  │      │           │      │  ┌─────┐
   │  4  │──┼──────┤           ├──────┼──│  7  │
   └──┬──┘  │      │           │      │  └──┬──┘
      │     │      └───────────┘      │     │
   ┌──┴──┐  │                         │  ┌──┴──┐
   │  5  │  │                         │  │  8  │
   └──┬──┘  │                         │  └──┬──┘
      │     │                         │     │
   ┌──┴──┐  │                         │  ┌──┴──┐
   │  6  │  │                         │  │  9  │
   └─────┘  │                         │  └─────┘
            │                         │
            └─────────────────────────┘
                        REAR

    FL = Front Left  (1, 2, 3)      FR = Front Right (10, 11, 12)
    RL = Rear Left   (4, 5, 6)      RR = Rear Right  (7, 8, 9)
```

## Leg Structure

Each leg has 3 motors:

```
    LEFT SIDE                           RIGHT SIDE
    FL (1,2,3) & RL (4,5,6)            FR (10,11,12) & RR (7,8,9)

         [1/4]  ← Hip                    [10/7]  ← Hip
           │                                │
         [2/5]  ← Upper Leg              [11/8]  ← Upper Leg
           │                                │
         [3/6]  ← Lower Leg (Knee)       [12/9]  ← Lower Leg (Knee)
           │                                │
          ═╧═   ← Foot                     ═╧═   ← Foot
```

## Motor Movement Convention

**When RAW VALUE INCREASES (+), each motor moves as follows:**

| Motor ID | Location | Joint | Effect on Increase |
|----------|----------|-------|-------------------|
| **1** | Front Left | Hip | ⬆️ UP |
| **2** | Front Left | Upper Leg | ⬆️ UP |
| **3** | Front Left | Lower Leg | 🔒 CLOSE (fold) |
| **4** | Rear Left | Hip | ⬆️ UP |
| **5** | Rear Left | Upper Leg | ⬆️ UP |
| **6** | Rear Left | Lower Leg | 🔒 CLOSE (fold) |
| **7** | Rear Right | Hip | ⬆️ UP |
| **8** | Rear Right | Upper Leg | ⬇️ DOWN |
| **9** | Rear Right | Lower Leg | 🔓 OPEN (extend) |
| **10** | Front Right | Hip | ⬆️ UP |
| **11** | Front Right | Upper Leg | ⬇️ DOWN |
| **12** | Front Right | Lower Leg | 🔓 OPEN (extend) |

### Visual Summary

```
INCREASING RAW VALUE (+) CAUSES:

LEFT SIDE (FL & RL):             RIGHT SIDE (FR & RR):
┌──────────────────┐             ┌──────────────────┐
│  Motor 1,4: UP   │  Hip        │  Motor 10,7: UP  │
│  Motor 2,5: UP   │  Upper      │  Motor 11,8: DOWN│
│  Motor 3,6: CLOSE│  Lower      │  Motor 12,9: OPEN│
└──────────────────┘             └──────────────────┘
```

### Motor Groups by Function

| Group | Motors | On + Raw Value |
|-------|--------|----------------|
| Hip (all legs) | 1, 4, 7, 10 | UP |
| Left Upper | 2, 5 | UP |
| Left Lower | 3, 6 | CLOSE |
| Right Upper | 8, 11 | DOWN |
| Right Lower | 9, 12 | OPEN |

## Scripts

### `test_motors.py` - Interactive Multi-Motor Control
Interactive script for manual control of all discovered motors.

```bash
uv run python test_motors.py
```

Features:
- Auto-discovers all connected motors
- GroupSync for efficient communication
- Reference-based position input (degrees or raw)
- Commands: `q` quit, `s` status, `z` re-zero, `h` home

### `all_motor_control.py` - Oscillation Test
Automated oscillation test for motors 3, 6, 9, 12 (multiples of 3).

```bash
# Default: 5 Hz, ±300r range
uv run python all_motor_control.py

# Custom frequency and range
uv run python all_motor_control.py -f 10 -s 20 --min -500 --max 500
```

Motion convention:
- Motors 3, 6: Move in NORMAL direction
- Motors 9, 12: Move in OPPOSITE direction (opposite phase)

### `basic_sitstand.py` - Sit-Stand Motion
Simple sit-stand sequence using knee motors.

```bash
# Default: 500r offset
uv run python basic_sitstand.py

# Custom offset
uv run python basic_sitstand.py -t 400 -f 10
```

Motion:
- STAND → SIT: Motors 9,12 go +500r, Motors 3,6 go -500r
- SIT → STAND: Return to reference

### `mx64_controller.py` - Base Controller Class
Core controller class providing:
- Port scanning and auto-connection
- Motor discovery via broadcast ping
- Individual and GroupSync read/write
- Position, velocity, current control
- Operating mode switching (Position/Extended Position)

## Hardware Setup

1. **Power**: 12V DC to motors
2. **Communication**: USB to TTL (2Mbps default)
3. **Ports**: `/dev/ttyUSB0` or `/dev/ttyUSB1`

### USB Latency Timer Configuration (Important)

For optimal motor control performance, set the USB latency timer to 1ms.

#### Automated Setup (One-time, Recommended)

Run the automated setup script that detects your device and configures everything:

```bash
# From the repository root directory
./setup_usb_latency.sh
```

The script will:
- Detect your FTDI USB device automatically
- Create a udev rule with the correct IDs
- Install and activate it

Done! The latency timer will be set to 1ms automatically whenever you plug in the device. This persists across reboots.

#### Manual Setup (Temporary, each session)

If you prefer not to install the udev rule:

```bash
sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
```

## Troubleshooting

### Permission denied on /dev/ttyUSB*
```bash
sudo chmod 666 /dev/ttyUSB0
# Or add user to dialout group:
sudo usermod -a -G dialout $USER
```

### No motors found
- Check 12V power supply
- Verify USB connection
- Check baud rate (default: 2Mbps)

### Sync read failures
- The controller includes retry logic with port buffer clearing
- Increase `max_retries` parameter if needed
- Individual read fallback is automatic
