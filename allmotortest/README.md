# Multi-Motor Dynamixel Controller

High-frequency controller for simultaneous control of multiple Dynamixel motors using GroupSyncRead/Write.

## Quick Start

```bash
# Test 3 motors at 20Hz (default step size: 40 units)
python3 multi_motor_controller.py

# Test 12 motors at 50Hz with larger steps (faster cycle)
python3 multi_motor_controller.py -n 12 -r 50 -s 100

# Test with fine control (small steps)
python3 multi_motor_controller.py -s 20 -r 50

# View all options
python3 multi_motor_controller.py -h
```

## What It Does

1. Connects to N motors (IDs 1 to N)
2. Reads current motor positions
3. Calculates next position: `current_pos + step_size` (oscillates between 0-4000)
4. Writes positions to ALL motors in ONE packet (GroupSyncWrite)
5. Reads back positions from ALL motors in ONE packet (GroupSyncRead)
6. Repeats incrementally - no absolute sequence, just relative steps

## Command-Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `-n N` | Number of motors | 3 |
| `-r HZ` | Control frequency (Hz) | 100 |
| `-p PORT` | Serial port | /dev/ttyUSB0 |
| `-b BAUD` | Baudrate (bps) | 2000000 |
| `-d SEC` | Write-read delay (sec) | 0.0005 |
| `-s STEP` | Position step size (units) | 40 |

## Performance

Expected frequencies with 2Mbps baudrate:
- **3 motors**: 100-120 Hz
- **6 motors**: 90-110 Hz
- **12 motors**: 80-100 Hz

## Test Pattern (Incremental Positioning)

**Approach**: Read current position → Add step → Write new position → Repeat

- **Step size**: Configurable (default: 40 units = ~3.5°)
- **Range**: 0 to 4000 units
- **Pattern**: Starts from current position, increments by step_size, reverses at limits

**Benefits**:
- No sudden jumps at sequence boundaries
- Handles intermittent read failures gracefully (continues with last known position)
- Smoother motion
- More robust for multiple motors

**Step Size Examples:**

| Step Size | Degrees/Step | Safe at 20Hz? | Safe at 100Hz? |
|-----------|--------------|---------------|----------------|
| 20 units | 1.8° | ✓ Yes | ✓ Yes |
| 40 units | 3.5° | ✓ Yes | ✓ Yes |
| 100 units | 8.8° | ✓ Yes | ✗ Marginal |
| 200 units | 17.6° | ✓ Yes | ✗ No |

**Safe step sizes** (motors can reach target in 1 cycle):
  - 20Hz: ≤215 units
  - 100Hz: ≤43 units

## Usage in Code

```python
from multi_motor_controller import MultiMotorController

# Initialize
controller = MultiMotorController(motor_ids=[1, 2, 3])
controller.connect()

# Write positions (Dynamixel units)
controller.write_positions({1: 2048, 2: 3000, 3: 1500})

# Read positions
positions = controller.read_positions()  # Returns {1: 2048, 2: 3000, 3: 1500}

# Or use radians
controller.write_positions_radians({1: 0.0, 2: 0.5, 3: -0.3})
positions_rad = controller.read_positions_radians()

controller.disconnect()
```

## Troubleshooting

**No communication**: Check port, power, motor IDs, and baudrate
**Read errors**: Increase delay `-d 0.001` or reduce frequency `-r 20`
**Low frequency**: Use higher baudrate `-b 3000000` or fewer motors

## Files

- `multi_motor_controller.py` - Main controller class and test script
- `fast_dynamixel_controller.py` - Alternative controller (2-motor focused)
- `mx_64.md` - Motor specifications and control table
