#!/usr/bin/env python3
# rumi-custom-quadruped — Reinforcement-learning-based quadruped-robot control framework for custom quadruped
# Copyright (C) 2025  Vishwanath R
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
MX-64 Dynamixel Motor Controller Base Class

A reusable controller for Dynamixel MX-64 motors using Protocol 2.0.
Provides port scanning, motor discovery, and read/write operations.

Supports two position control modes:
- Position Control Mode (mode 3): 0-360 degrees, single rotation
- Extended Position Control Mode (mode 4): Multi-turn, -256 to +256 revolutions

Supports two communication methods:
- Individual read/write: Simple, one motor at a time
- GroupSync read/write: Efficient, all motors in ONE packet (recommended for multi-motor)

Usage (Single Motor):
    controller = MX64Controller()
    controller.connect()
    controller.set_torque(True)
    controller.write_position(2048)
    controller.disconnect()

Usage (Multi-Motor with GroupSync - Recommended):
    controller = MX64Controller()
    controller.connect()

    # Initialize sync for discovered motors
    motors = controller.scan_motors()
    motor_ids = list(motors.keys())  # e.g., [1, 2, 3]
    controller.init_sync(motor_ids)

    # Enable torque for all
    for mid in motor_ids:
        controller.set_torque(True, mid)

    # Write all positions in ONE packet (simultaneous)
    controller.sync_write_positions({1: 2048, 2: 3000, 3: 1500})

    # Read all positions in ONE packet
    positions = controller.sync_read_positions()  # {1: 2048, 2: 3000, 3: 1500}

    controller.disconnect()
"""

import sys
import os
import json

# Add the local DynamixelSDK to path
SDK_PATH = os.path.join(os.path.dirname(__file__), '..', 'DynamixelSDK', 'python', 'src')
if SDK_PATH not in sys.path:
    sys.path.insert(0, SDK_PATH)

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncRead,
    GroupSyncWrite,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_LOWORD,
    DXL_HIWORD,
)


def _load_motor_config(config_path=None):
    """
    Load motor configuration from JSON file.

    Config file format:
    {
        "control": {"rate_hz": 100.0},
        "groups": {"hip": [1,4,7,10], "thigh": [2,5,8,11], "calf": [3,6,9,12], ...},
        "motors": {
            "1": {"initial_position": 2048, "kp": 6.25, "kd": 0.0, "min_position": 1024, "max_position": 3072},
            ...
        }
    }

    Note: kp and kd are ACTUAL values (not raw). Conversions:
        - raw_kp = kp × 128
        - raw_kd = kd × 16

    Args:
        config_path: Path to JSON config file. If None, looks for 'motor_config.json'

    Returns:
        Dictionary with 'control_rate', 'groups', and 'motors'
    """
    # Default configuration
    default_config = {
        'control_rate': 100.0,
        'groups': {
            'hip': [1, 4, 7, 10],
            'thigh': [2, 5, 8, 11],
            'calf': [3, 6, 9, 12],
            'front_left_leg': [1, 2, 3],
            'rear_left_leg': [4, 5, 6],
            'rear_right_leg': [7, 8, 9],
            'front_right_leg': [10, 11, 12],
            'left_side': [1, 2, 3, 4, 5, 6],
            'right_side': [7, 8, 9, 10, 11, 12]
        },
        'motors': {}
    }

    # Initialize default motor configs (1-12)
    for motor_id in range(1, 13):
        default_config['motors'][motor_id] = {
            'initial_position': 2048,
            'kp': 6.25,  # Actual Kp value (raw = 800)
            'kd': 0.0,   # Actual Kd value (raw = 0)
            'min_position': 1024,
            'max_position': 3072
        }

    # Determine config file path
    if config_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, 'motor_config.json')

    # Try to load from file
    if not os.path.exists(config_path):
        print(f"[CONFIG] Config file not found: {config_path}")
        print(f"[CONFIG] Using built-in defaults")
        return default_config

    try:
        with open(config_path, 'r') as f:
            file_config = json.load(f)

        # Extract control rate
        if 'control' in file_config:
            default_config['control_rate'] = file_config['control'].get('rate_hz', 100.0)

        # Load motor groups
        if 'groups' in file_config:
            default_config['groups'].update(file_config['groups'])

        # Load motor configurations
        if 'motors' in file_config:
            for motor_id_str, motor_data in file_config['motors'].items():
                motor_id = int(motor_id_str)
                # Update default with file values
                default_config['motors'][motor_id].update(motor_data)

        print(f"[CONFIG] Loaded configuration from: {config_path}")
        print(f"[CONFIG] Control rate: {default_config['control_rate']} Hz")
        return default_config

    except json.JSONDecodeError as e:
        print(f"[ERROR] Failed to parse config file: {e}")
        print(f"[CONFIG] Using built-in defaults")
        return default_config
    except Exception as e:
        print(f"[ERROR] Failed to load config file: {e}")
        print(f"[CONFIG] Using built-in defaults")
        return default_config


class MX64Controller:
    """
    Base controller class for Dynamixel MX-64 motors.

    Features:
    - Automatic port scanning (/dev/ttyUSB0, /dev/ttyUSB1)
    - Motor discovery via broadcast ping
    - Read/write to any control table address
    - Position, velocity, current control support
    - Unit conversion helpers (degrees, radians)

    Control Table Reference (Protocol 2.0):
        EEPROM (non-volatile, requires torque off):
            7: ID, 8: Baud Rate, 11: Operating Mode
            48: Max Position Limit, 52: Min Position Limit

        RAM (volatile):
            64: Torque Enable, 65: LED
            116: Goal Position (4 bytes)
            132: Present Position (4 bytes)
            126: Present Current (2 bytes)
            144: Present Voltage (2 bytes)
            146: Present Temperature (1 byte)
    """

    # Control Table Addresses (MX-64 Protocol 2.0)
    # EEPROM Area
    ADDR_MODEL_NUMBER = 0
    ADDR_FIRMWARE_VERSION = 6
    ADDR_ID = 7
    ADDR_BAUD_RATE = 8
    ADDR_RETURN_DELAY_TIME = 9
    ADDR_DRIVE_MODE = 10
    ADDR_OPERATING_MODE = 11
    ADDR_SECONDARY_ID = 12
    ADDR_PROTOCOL_TYPE = 13
    ADDR_HOMING_OFFSET = 20
    ADDR_MOVING_THRESHOLD = 24
    ADDR_TEMPERATURE_LIMIT = 31
    ADDR_MAX_VOLTAGE_LIMIT = 32
    ADDR_MIN_VOLTAGE_LIMIT = 34
    ADDR_PWM_LIMIT = 36
    ADDR_CURRENT_LIMIT = 38
    ADDR_ACCELERATION_LIMIT = 40
    ADDR_VELOCITY_LIMIT = 44
    ADDR_MAX_POSITION_LIMIT = 48
    ADDR_MIN_POSITION_LIMIT = 52
    ADDR_SHUTDOWN = 63

    # RAM Area
    ADDR_TORQUE_ENABLE = 64
    ADDR_LED = 65
    ADDR_STATUS_RETURN_LEVEL = 68
    ADDR_REGISTERED_INSTRUCTION = 69
    ADDR_HARDWARE_ERROR_STATUS = 70
    ADDR_VELOCITY_I_GAIN = 76
    ADDR_VELOCITY_P_GAIN = 78
    ADDR_POSITION_D_GAIN = 80
    ADDR_POSITION_I_GAIN = 82
    ADDR_POSITION_P_GAIN = 84
    ADDR_FEEDFORWARD_2ND_GAIN = 88
    ADDR_FEEDFORWARD_1ST_GAIN = 90
    ADDR_BUS_WATCHDOG = 98
    ADDR_GOAL_PWM = 100
    ADDR_GOAL_CURRENT = 102
    ADDR_GOAL_VELOCITY = 104
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY = 112
    ADDR_GOAL_POSITION = 116
    ADDR_REALTIME_TICK = 120
    ADDR_MOVING = 122
    ADDR_MOVING_STATUS = 123
    ADDR_PRESENT_PWM = 124
    ADDR_PRESENT_CURRENT = 126
    ADDR_PRESENT_VELOCITY = 128
    ADDR_PRESENT_POSITION = 132
    ADDR_VELOCITY_TRAJECTORY = 136
    ADDR_POSITION_TRAJECTORY = 140
    ADDR_PRESENT_INPUT_VOLTAGE = 144
    ADDR_PRESENT_TEMPERATURE = 146

    # Data sizes
    LEN_GOAL_POSITION = 4
    LEN_PRESENT_POSITION = 4
    LEN_PRESENT_CURRENT = 2
    LEN_PRESENT_VOLTAGE = 2

    # Protocol settings
    PROTOCOL_VERSION = 2.0

    # Supported baud rates (value -> actual baud rate)
    BAUD_RATE_MAP = {
        0: 9600,
        1: 57600,
        2: 115200,
        3: 1000000,
        4: 2000000,
        5: 3000000,
        6: 4000000,
        7: 4500000,
    }

    # Operating modes
    OPERATING_MODE_CURRENT = 0
    OPERATING_MODE_VELOCITY = 1
    OPERATING_MODE_POSITION = 3
    OPERATING_MODE_EXTENDED_POSITION = 4
    OPERATING_MODE_CURRENT_BASED_POSITION = 5
    OPERATING_MODE_PWM = 16

    # Position constants - Standard Position Control Mode (mode 3)
    POSITION_MIN = 0
    POSITION_MAX = 4095
    POSITION_CENTER = 2048

    # Extended Position Control Mode (mode 4) - Multi-turn
    # Supports -256 to +256 revolutions
    EXTENDED_POSITION_MIN = -1044479
    EXTENDED_POSITION_MAX = 1044479

    # Conversion constants
    POSITION_TO_DEGREES = 360.0 / 4096.0  # ~0.088 degrees per unit
    DEGREES_TO_POSITION = 4096.0 / 360.0
    POSITION_TO_RADIANS = 3.14159265359 * 2 / 4096.0
    RADIANS_TO_POSITION = 4096.0 / (3.14159265359 * 2)

    # Units per revolution (for extended mode calculations)
    UNITS_PER_REVOLUTION = 4096

    # Default ports to scan
    DEFAULT_PORTS = ['/dev/ttyUSB0', '/dev/ttyUSB1']

    # Default baud rates to try
    DEFAULT_BAUDRATES = [4000000, 2000000, 1000000, 57600, 115200]

    def __init__(self, port=None, baudrate=None, motor_id=1, auto_connect=False, config_path=None):
        """
        Initialize the MX-64 controller.

        Args:
            port: Serial port path. If None, will auto-scan.
            baudrate: Communication baud rate. If None, will try common rates.
            motor_id: Target motor ID (default: 1)
            auto_connect: If True, connect immediately on init.
            config_path: Path to motor configuration JSON file. If None, looks for
                        'motor_config.json' in the same directory. Config contains
                        initial positions, PID gains, and position limits per motor.
        """
        self.port = port
        self.baudrate = baudrate
        self.motor_id = motor_id
        self.connected = False

        self.port_handler = None
        self.packet_handler = None

        # Load configuration from JSON file
        self._config = _load_motor_config(config_path)
        self.control_rate = self._config['control_rate']
        self.groups = self._config['groups']

        # Discovered motors cache
        self.discovered_motors = {}  # {id: (model_number, firmware_version)}

        # Operating mode cache per motor {motor_id: mode}
        # Used to determine position handling (standard vs extended)
        self._operating_mode_cache = {}

        # GroupSync handlers for efficient multi-motor communication
        self._sync_read_position = None
        self._sync_write_position = None
        self._sync_motor_ids = []  # Motors registered for sync operations

        # Initial positions read during initialize()
        self.initial_positions = {}

        # Statistics
        self.stats = {
            'read_count': 0,
            'write_count': 0,
            'read_errors': 0,
            'write_errors': 0,
        }

        if auto_connect:
            self.connect()

    # ==================== Connection Methods ====================

    def scan_ports(self):
        """
        Scan for available serial ports.

        Returns:
            list: Available port paths
        """
        available_ports = []
        for port in self.DEFAULT_PORTS:
            if os.path.exists(port):
                available_ports.append(port)
        return available_ports

    def try_connect_port(self, port, baudrate):
        """
        Attempt to connect to a specific port and baudrate.

        Args:
            port: Serial port path
            baudrate: Baud rate to use

        Returns:
            bool: True if connection successful
        """
        try:
            handler = PortHandler(port)

            if not handler.openPort():
                return False

            if not handler.setBaudRate(baudrate):
                handler.closePort()
                return False

            # Store handlers
            self.port_handler = handler
            self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)
            self.port = port
            self.baudrate = baudrate

            return True

        except Exception as e:
            print(f"[ERROR] Exception connecting to {port}: {e}")
            return False

    def connect(self):
        """
        Connect to the motor controller.
        Auto-scans ports if none specified.

        Returns:
            bool: True if connection successful
        """
        # Determine ports to try
        if self.port:
            ports_to_try = [self.port]
        else:
            ports_to_try = self.scan_ports()
            if not ports_to_try:
                print("[ERROR] No serial ports found (/dev/ttyUSB0 or /dev/ttyUSB1)")
                return False
            print(f"[INFO] Found ports: {ports_to_try}")

        # Determine baud rates to try
        if self.baudrate:
            baudrates_to_try = [self.baudrate]
        else:
            baudrates_to_try = self.DEFAULT_BAUDRATES

        # Try each port/baudrate combination
        for port in ports_to_try:
            for baudrate in baudrates_to_try:
                print(f"[INFO] Trying {port} at {baudrate} bps...")

                if self.try_connect_port(port, baudrate):
                    # Try to ping motors to verify connection
                    motors = self.scan_motors(verbose=False)

                    if motors:
                        print(f"[SUCCESS] Connected to {port} at {baudrate} bps")
                        print(f"[SUCCESS] Found {len(motors)} motor(s): {list(motors.keys())}")
                        self.connected = True
                        return True
                    else:
                        # No motors found, try next baudrate
                        self.port_handler.closePort()
                        self.port_handler = None

        print("[ERROR] Could not connect to any motors")
        return False

    def disconnect(self):
        """Disable torque, cleanup sync handlers, and close the serial port."""
        if self.connected and self.port_handler:
            # Disable torque for all sync motors (if initialized)
            if self._sync_motor_ids:
                for mid in self._sync_motor_ids:
                    self.set_torque(False, mid)
            else:
                self.set_torque(False)

            # Cleanup GroupSync handlers
            self.cleanup_sync()

            # Close port
            self.port_handler.closePort()
            self.connected = False

            print(f"[INFO] Disconnected from {self.port}")
            self._print_stats()

    def _print_stats(self):
        """Print communication statistics."""
        if self.stats['read_count'] > 0 or self.stats['write_count'] > 0:
            print(f"\n[STATS] Reads: {self.stats['read_count']} "
                  f"(errors: {self.stats['read_errors']}), "
                  f"Writes: {self.stats['write_count']} "
                  f"(errors: {self.stats['write_errors']})")

    # ==================== Configuration Methods ====================

    def initialize(self, expected_motors=12):
        """
        Full initialization sequence for robot startup.

        Steps:
            1. Connect to motors (auto-scan ports/baudrates)
            2. Discover all motors
            3. Verify expected motor count
            4. Apply PID gains from config file
            5. Initialize GroupSync for efficient communication
            6. Read initial positions
            7. Enable torque on all motors

        After this method returns successfully:
            - All motors are connected and verified
            - PID gains are set from config
            - GroupSync is ready for fast read/write
            - Initial positions are stored in self.initial_positions
            - Torque is ON - motors are holding position

        Args:
            expected_motors: Expected number of motors (default: 12)

        Returns:
            list: List of discovered motor IDs if successful, None if failed
        """
        print("\n" + "=" * 60)
        print("   Motor Initialization")
        print("=" * 60)

        # Step 1: Connect
        print("\n[STEP 1] Connecting to motors...")
        print("-" * 40)

        if not self.connect():
            print("\n[FAILED] Could not connect to any motors.")
            print("Please check:")
            print("  - Motors are powered (12V)")
            print("  - USB cable is connected")
            print("  - Correct permissions on /dev/ttyUSB*")
            return None

        # Step 2: Discover motors
        print("\n[STEP 2] Discovering motors...")
        print("-" * 40)

        motors = self.scan_motors(verbose=True)

        if not motors:
            print("[FAILED] No motors discovered.")
            self.disconnect()
            return None

        motor_ids = sorted(motors.keys())
        print(f"\nFound {len(motor_ids)} motor(s): {motor_ids}")

        # Step 3: Verify motor count
        print("\n[STEP 3] Verifying motor count...")
        print("-" * 40)

        if len(motor_ids) != expected_motors:
            print(f"[WARNING] Expected {expected_motors} motors, found {len(motor_ids)}")
            print(f"  Missing motors: {[m for m in range(1, expected_motors + 1) if m not in motor_ids]}")
            self.disconnect()
            return None

        print(f"[OK] All {expected_motors} motors found")

        # Step 4: Apply PID configuration
        print("\n[STEP 4] Applying PID gains from config...")
        print("-" * 40)

        if not self.apply_pid_config(motor_ids):
            print("[FAILED] Could not apply PID configuration")
            self.disconnect()
            return None

        # Step 5: Initialize GroupSync
        print("\n[STEP 5] Initializing GroupSync...")
        print("-" * 40)

        if not self.init_sync(motor_ids):
            print("[FAILED] Could not initialize GroupSync")
            self.disconnect()
            return None

        print("[OK] GroupSync ready - all motors read/write in ONE packet")

        # Step 6: Read initial positions
        print("\n[STEP 6] Reading initial positions...")
        print("-" * 40)

        positions = self.sync_read_positions(max_retries=10, retry_delay=0.02)
        if positions is None:
            print("[WARNING] Sync read failed, trying individual reads...")
            positions = {}
            for mid in motor_ids:
                pos = self.read_position(mid)
                if pos is not None:
                    positions[mid] = pos
                else:
                    print(f"[FAILED] Could not read position for motor {mid}")
                    self.disconnect()
                    return None

        self.initial_positions = positions.copy()
        for mid in motor_ids:
            print(f"  Motor {mid:2d}: {self.initial_positions[mid]:5d} raw")

        # Step 7: Enable torque on all motors
        print("\n[STEP 7] Enabling torque on all motors...")
        print("-" * 40)

        for mid in motor_ids:
            if self.set_torque(True, mid):
                print(f"  Motor {mid:2d}: Torque ON [OK]")
            else:
                print(f"  Motor {mid:2d}: Torque ON [FAILED]")
                self.disconnect()
                return None

        print("\n" + "=" * 60)
        print(f"[SUCCESS] Initialization complete - {len(motor_ids)} motors ready")
        print("=" * 60)

        return motor_ids

    def apply_pid_config(self, motor_ids=None):
        """
        Apply PID gains (Kp, Kd) from config file to specified motors.

        Config file stores ACTUAL Kp/Kd values (not raw). This method automatically
        converts them to raw register values:
            - raw_kp = kp × 128
            - raw_kd = kd × 16

        Note: PID writes require torque to be OFF. This method will temporarily
        disable torque if needed and restore it afterwards.

        Args:
            motor_ids: List of motor IDs to configure. If None, uses discovered motors.

        Returns:
            bool: True if all motors configured successfully
        """
        if motor_ids is None:
            motor_ids = list(self.discovered_motors.keys())
            if not motor_ids:
                print("[WARNING] No motors discovered. Call scan_motors() first.")
                return False

        success_count = 0
        fail_count = 0

        for mid in motor_ids:
            motor_config = self._config['motors'].get(mid, {})

            # Read actual Kp/Kd values from config
            kp_actual = motor_config.get('kp', 6.25)
            kd_actual = motor_config.get('kd', 0.0)

            # Convert actual values to raw register values
            # Kp: raw = actual × 128
            # Kd: raw = actual × 16
            kp_raw = int(round(kp_actual * 128))
            kd_raw = int(round(kd_actual * 16))

            # Clamp to valid range (0-16383)
            kp_raw = max(0, min(16383, kp_raw))
            kd_raw = max(0, min(16383, kd_raw))

            # PID gains require torque off
            torque_was_on = self.get_torque(mid)
            if torque_was_on:
                self.set_torque(False, mid)

            # Write P and D gains (raw values)
            p_success = self.write_2_bytes(self.ADDR_POSITION_P_GAIN, kp_raw, mid)
            d_success = self.write_2_bytes(self.ADDR_POSITION_D_GAIN, kd_raw, mid)

            if p_success and d_success:
                print(f"  Motor {mid:2d}: Kp={kp_actual:6.2f} (raw={kp_raw:4d}), Kd={kd_actual:6.2f} (raw={kd_raw:4d}) [OK]")
                success_count += 1
            else:
                print(f"  Motor {mid:2d}: Kp={kp_actual:6.2f} (raw={kp_raw:4d}), Kd={kd_actual:6.2f} (raw={kd_raw:4d}) [FAILED]")
                fail_count += 1

            # Restore torque state
            if torque_was_on:
                self.set_torque(True, mid)

        print(f"\n[CONFIG] Applied PID to {success_count}/{len(motor_ids)} motors", end="")
        if fail_count > 0:
            print(f" ({fail_count} failed)")
        else:
            print()

        return fail_count == 0

    def validate_position(self, position, motor_id=None):
        """
        Validate a position against configured limits for a motor.

        Args:
            position: Position value to validate
            motor_id: Motor ID. Uses self.motor_id if None.

        Returns:
            int: Clamped position within configured limits
        """
        mid = motor_id if motor_id is not None else self.motor_id
        motor_config = self._config['motors'].get(mid, {})
        min_pos = motor_config.get('min_position', 1024)
        max_pos = motor_config.get('max_position', 3072)

        # For extended position mode, use hardware limits instead of config limits
        if self.is_extended_position_mode(mid):
            min_pos = self.EXTENDED_POSITION_MIN
            max_pos = self.EXTENDED_POSITION_MAX

        return max(min_pos, min(max_pos, position))

    def get_initial_positions(self):
        """
        Get initial positions from configuration for all motors.

        Returns:
            dict: {motor_id: initial_position}
        """
        return {mid: motor.get('initial_position', 2048)
                for mid, motor in self._config['motors'].items()}

    def move_to_initial_positions(self, motor_ids=None):
        """
        Move specified motors to their configured initial positions.

        Args:
            motor_ids: List of motor IDs. If None, moves all discovered motors.

        Returns:
            bool: True if successful
        """
        if motor_ids is None:
            motor_ids = list(self.discovered_motors.keys())
            if not motor_ids:
                print("[WARNING] No motors discovered. Call scan_motors() first.")
                return False

        initial_positions = self.get_initial_positions()
        target_positions = {mid: initial_positions[mid] for mid in motor_ids if mid in initial_positions}

        if not target_positions:
            print("[WARNING] No initial positions configured for specified motors")
            return False

        print(f"\n[CONFIG] Moving {len(target_positions)} motor(s) to initial positions...")
        for mid, pos in target_positions.items():
            print(f"  Motor {mid}: -> {pos} raw")

        if self._sync_write_position is not None:
            return self.sync_write_positions(target_positions)
        else:
            # Individual writes if sync not initialized
            success = True
            for mid, pos in target_positions.items():
                if not self.write_position(pos, mid):
                    success = False
            return success

    # ==================== Motor Discovery ====================

    def scan_motors(self, verbose=True):
        """
        Scan for all connected motors using broadcast ping.

        Args:
            verbose: Print detailed scan results

        Returns:
            dict: {motor_id: (model_number, firmware_version)}
        """
        if not self.port_handler:
            print("[ERROR] Not connected to any port")
            return {}

        try:
            data_list, comm_result = self.packet_handler.broadcastPing(self.port_handler)

            if comm_result != COMM_SUCCESS:
                if verbose:
                    print(f"[ERROR] Broadcast ping failed: "
                          f"{self.packet_handler.getTxRxResult(comm_result)}")
                return {}

            # Cache results
            self.discovered_motors = {}
            for motor_id, (model_num, fw_version) in data_list.items():
                self.discovered_motors[motor_id] = (model_num, fw_version)

            if verbose and self.discovered_motors:
                print(f"\n[SCAN] Found {len(self.discovered_motors)} motor(s):")
                for motor_id, (model_num, fw_version) in sorted(self.discovered_motors.items()):
                    print(f"  ID {motor_id}: Model {model_num}, Firmware v{fw_version}")

            return self.discovered_motors

        except Exception as e:
            if verbose:
                print(f"[ERROR] Exception during scan: {e}")
            return {}

    def ping(self, motor_id=None):
        """
        Ping a specific motor to check if it's responding.

        Args:
            motor_id: Motor ID to ping. Uses self.motor_id if None.

        Returns:
            tuple: (model_number, firmware_version) or None if failed
        """
        if not self.port_handler:
            return None

        mid = motor_id if motor_id is not None else self.motor_id

        try:
            model_num, comm_result, error = self.packet_handler.ping(
                self.port_handler, mid
            )

            if comm_result != COMM_SUCCESS:
                return None
            if error != 0:
                return None

            return model_num

        except Exception:
            return None

    def select_motor(self, motor_id):
        """
        Select a motor ID for subsequent operations.

        Args:
            motor_id: Motor ID to select

        Returns:
            bool: True if motor responds to ping
        """
        if self.ping(motor_id) is not None:
            self.motor_id = motor_id
            print(f"[INFO] Selected motor ID {motor_id}")
            return True
        else:
            print(f"[ERROR] Motor ID {motor_id} not responding")
            return False

    # ==================== Low-Level Read/Write ====================

    def read_1_byte(self, address, motor_id=None):
        """Read 1 byte from control table."""
        mid = motor_id if motor_id is not None else self.motor_id
        self.stats['read_count'] += 1

        try:
            value, comm_result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, mid, address
            )

            if comm_result != COMM_SUCCESS or error != 0:
                self.stats['read_errors'] += 1
                return None

            return value

        except Exception:
            self.stats['read_errors'] += 1
            return None

    def read_2_bytes(self, address, motor_id=None):
        """Read 2 bytes from control table."""
        mid = motor_id if motor_id is not None else self.motor_id
        self.stats['read_count'] += 1

        try:
            value, comm_result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, mid, address
            )

            if comm_result != COMM_SUCCESS or error != 0:
                self.stats['read_errors'] += 1
                return None

            return value

        except Exception:
            self.stats['read_errors'] += 1
            return None

    def read_4_bytes(self, address, motor_id=None):
        """Read 4 bytes from control table."""
        mid = motor_id if motor_id is not None else self.motor_id
        self.stats['read_count'] += 1

        try:
            value, comm_result, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, mid, address
            )

            if comm_result != COMM_SUCCESS or error != 0:
                self.stats['read_errors'] += 1
                return None

            return value

        except Exception:
            self.stats['read_errors'] += 1
            return None

    def write_1_byte(self, address, value, motor_id=None):
        """Write 1 byte to control table."""
        mid = motor_id if motor_id is not None else self.motor_id
        self.stats['write_count'] += 1

        try:
            comm_result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, mid, address, value
            )

            if comm_result != COMM_SUCCESS or error != 0:
                self.stats['write_errors'] += 1
                return False

            return True

        except Exception:
            self.stats['write_errors'] += 1
            return False

    def write_2_bytes(self, address, value, motor_id=None):
        """Write 2 bytes to control table."""
        mid = motor_id if motor_id is not None else self.motor_id
        self.stats['write_count'] += 1

        try:
            comm_result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, mid, address, value
            )

            if comm_result != COMM_SUCCESS or error != 0:
                self.stats['write_errors'] += 1
                return False

            return True

        except Exception:
            self.stats['write_errors'] += 1
            return False

    def write_4_bytes(self, address, value, motor_id=None):
        """Write 4 bytes to control table."""
        mid = motor_id if motor_id is not None else self.motor_id
        self.stats['write_count'] += 1

        try:
            comm_result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, mid, address, value
            )

            if comm_result != COMM_SUCCESS or error != 0:
                self.stats['write_errors'] += 1
                return False

            return True

        except Exception:
            self.stats['write_errors'] += 1
            return False

    # ==================== High-Level Motor Control ====================

    def set_torque(self, enable, motor_id=None):
        """
        Enable or disable motor torque.

        Args:
            enable: True to enable, False to disable
            motor_id: Motor ID (uses selected motor if None)

        Returns:
            bool: Success status
        """
        return self.write_1_byte(self.ADDR_TORQUE_ENABLE, 1 if enable else 0, motor_id)

    def get_torque(self, motor_id=None):
        """Get torque enable status."""
        value = self.read_1_byte(self.ADDR_TORQUE_ENABLE, motor_id)
        return value == 1 if value is not None else None

    def set_led(self, on, motor_id=None):
        """Turn LED on or off."""
        return self.write_1_byte(self.ADDR_LED, 1 if on else 0, motor_id)

    def get_led(self, motor_id=None):
        """Get LED status."""
        value = self.read_1_byte(self.ADDR_LED, motor_id)
        return value == 1 if value is not None else None

    # ==================== Position Control ====================
    #
    # Position Control supports two modes:
    # - Position Control Mode (3): 0-4095 units (0-360 degrees), single rotation
    # - Extended Position Control Mode (4): -1,044,479 to 1,044,479 units
    #   (~-256 to +256 revolutions), multi-turn capable
    #
    # The methods below automatically handle both modes based on current setting.

    def _read_position_raw(self, motor_id=None):
        """
        Read raw 4-byte position value (internal use).
        Handles signed conversion for extended position mode.
        """
        mid = motor_id if motor_id is not None else self.motor_id
        value = self.read_4_bytes(self.ADDR_PRESENT_POSITION, mid)

        if value is None:
            return None

        # In extended mode, position can be negative (signed 32-bit)
        if self.is_extended_position_mode(mid):
            if value > 2147483647:  # 2^31 - 1
                value = value - 4294967296  # Convert to signed

        return value

    def read_position(self, motor_id=None):
        """
        Read current position in raw units.

        In Position Control Mode (3): Returns 0-4095
        In Extended Position Mode (4): Returns signed value (can be negative)

        Returns:
            int: Position value or None on error
        """
        return self._read_position_raw(motor_id)

    def write_position(self, position, motor_id=None):
        """
        Write goal position in raw units.

        In Position Control Mode (3): Clamps to configured limits (default 0-4095)
        In Extended Position Mode (4): Clamps to hardware limits (-1,044,479 to 1,044,479)

        Position is validated against configured limits from config file.

        Args:
            position: Target position
            motor_id: Motor ID (uses self.motor_id if None)

        Returns:
            bool: Success status
        """
        mid = motor_id if motor_id is not None else self.motor_id
        position = int(position)

        # Validate position against configured limits
        position = self.validate_position(position, mid)

        # Convert negative to unsigned 32-bit for transmission (extended mode)
        if self.is_extended_position_mode(mid) and position < 0:
            position = position + 4294967296

        return self.write_4_bytes(self.ADDR_GOAL_POSITION, position, mid)

    def read_position_degrees(self, motor_id=None):
        """
        Read current position in degrees.

        In Position Control Mode: Returns 0-360 degrees
        In Extended Position Mode: Returns total degrees (can be >360 or negative)
        """
        pos = self.read_position(motor_id)
        return pos * self.POSITION_TO_DEGREES if pos is not None else None

    def write_position_degrees(self, degrees, motor_id=None):
        """
        Write goal position in degrees.

        In Position Control Mode: Use 0-360 degrees
        In Extended Position Mode: Can use any degree value (e.g., 720 for 2 turns)
        """
        position = int(degrees * self.DEGREES_TO_POSITION)
        return self.write_position(position, motor_id)

    def read_position_radians(self, motor_id=None):
        """
        Read current position in radians.

        In Position Control Mode: Returns 0 to 2*pi
        In Extended Position Mode: Returns total radians (can be >2*pi or negative)
        """
        pos = self.read_position(motor_id)
        return pos * self.POSITION_TO_RADIANS if pos is not None else None

    def write_position_radians(self, radians, motor_id=None):
        """
        Write goal position in radians.

        In Position Control Mode: Use 0 to 2*pi
        In Extended Position Mode: Can use any radian value
        """
        position = int(radians * self.RADIANS_TO_POSITION)
        return self.write_position(position, motor_id)

    def read_goal_position(self, motor_id=None):
        """Read goal position in raw units."""
        mid = motor_id if motor_id is not None else self.motor_id
        value = self.read_4_bytes(self.ADDR_GOAL_POSITION, mid)

        if value is None:
            return None

        # Handle signed conversion for extended mode
        if self.is_extended_position_mode(mid):
            if value > 2147483647:
                value = value - 4294967296

        return value

    # ==================== GroupSync Operations ====================
    #
    # Efficient multi-motor communication using GroupSyncRead/Write.
    # All motors are read/written in ONE packet instead of individual packets.
    #
    # Benefits:
    # - Much faster (1 packet vs N packets)
    # - All motors receive commands simultaneously
    # - Better synchronization for robotics applications
    #
    # Usage:
    #   controller.init_sync(motor_ids)  # Initialize once
    #   controller.sync_write_positions({1: 2048, 2: 3000, 3: 1500})
    #   positions = controller.sync_read_positions()  # Returns {1: pos1, 2: pos2, ...}

    def init_sync(self, motor_ids):
        """
        Initialize GroupSync handlers for efficient multi-motor communication.
        Call this once after connecting, before using sync_read/write methods.

        Args:
            motor_ids: List of motor IDs to include in sync operations

        Returns:
            bool: True if successful
        """
        if not self.port_handler or not self.packet_handler:
            print("[ERROR] Not connected. Call connect() first.")
            return False

        self._sync_motor_ids = list(motor_ids)

        # Initialize GroupSyncRead for position
        self._sync_read_position = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            self.ADDR_PRESENT_POSITION,
            self.LEN_PRESENT_POSITION
        )

        # Initialize GroupSyncWrite for position
        self._sync_write_position = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            self.ADDR_GOAL_POSITION,
            self.LEN_GOAL_POSITION
        )

        # Register all motors for sync read
        for mid in self._sync_motor_ids:
            if not self._sync_read_position.addParam(mid):
                print(f"[ERROR] Failed to add motor {mid} to sync read group")
                return False

        print(f"[INFO] GroupSync initialized for motors: {self._sync_motor_ids}")
        return True

    def sync_write_positions(self, positions_dict):
        """
        Write goal positions to multiple motors in ONE packet.
        All motors receive the command simultaneously.

        Args:
            positions_dict: {motor_id: position} mapping
                           Example: {1: 2048, 2: 3000, 3: 1500}

        Returns:
            bool: True if successful
        """
        if self._sync_write_position is None:
            print("[ERROR] Sync not initialized. Call init_sync() first.")
            return False

        self.stats['write_count'] += 1

        # Add each motor's position to the sync write buffer
        for mid, position in positions_dict.items():
            position = int(position)

            # Validate position against configured limits
            position = self.validate_position(position, mid)

            # Convert negative to unsigned 32-bit for transmission (extended mode)
            if self.is_extended_position_mode(mid) and position < 0:
                position = position + 4294967296

            # Convert to 4-byte array (little-endian)
            param = [
                DXL_LOBYTE(DXL_LOWORD(position)),
                DXL_HIBYTE(DXL_LOWORD(position)),
                DXL_LOBYTE(DXL_HIWORD(position)),
                DXL_HIBYTE(DXL_HIWORD(position))
            ]

            if not self._sync_write_position.addParam(mid, param):
                print(f"[ERROR] Failed to add motor {mid} to sync write")
                self._sync_write_position.clearParam()
                self.stats['write_errors'] += 1
                return False

        # Send ONE packet containing all motor commands
        result = self._sync_write_position.txPacket()

        # Clear buffer for next write
        self._sync_write_position.clearParam()

        if result != COMM_SUCCESS:
            print(f"[ERROR] Sync write failed: {self.packet_handler.getTxRxResult(result)}")
            self.stats['write_errors'] += 1
            return False

        return True

    def sync_read_positions(self, max_retries=3, retry_delay=0.005):
        """
        Read current positions from all registered motors in ONE packet.

        Args:
            max_retries: Number of retry attempts on failure (default: 3)
            retry_delay: Delay between retries in seconds (default: 5ms)

        Returns:
            dict: {motor_id: position} or None on error
                  Example: {1: 2048, 2: 3000, 3: 1500}
        """
        import time

        if self._sync_read_position is None:
            print("[ERROR] Sync not initialized. Call init_sync() first.")
            return None

        self.stats['read_count'] += 1

        # Retry loop for transient communication errors
        for attempt in range(max_retries):
            # Clear any leftover data in the port buffer before attempting read
            # This prevents corrupted/partial data from previous failed attempts
            # from interfering with new read requests
            self.port_handler.clearPort()

            # Send read request and receive all responses in ONE transaction
            result = self._sync_read_position.txRxPacket()

            if result != COMM_SUCCESS:
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    print(f"[ERROR] Sync read failed after {max_retries} attempts: "
                          f"{self.packet_handler.getTxRxResult(result)}")
                    self.stats['read_errors'] += 1
                    return None

            # Extract position data for each motor
            positions = {}
            all_available = True

            for mid in self._sync_motor_ids:
                # Check if data is available
                if not self._sync_read_position.isAvailable(
                    mid,
                    self.ADDR_PRESENT_POSITION,
                    self.LEN_PRESENT_POSITION
                ):
                    if attempt < max_retries - 1:
                        all_available = False
                        break
                    else:
                        print(f"[ERROR] Data not available for motor {mid} after {max_retries} attempts")
                        self.stats['read_errors'] += 1
                        return None

                # Get position value
                value = self._sync_read_position.getData(
                    mid,
                    self.ADDR_PRESENT_POSITION,
                    self.LEN_PRESENT_POSITION
                )

                # Handle signed conversion for extended mode
                if self.is_extended_position_mode(mid):
                    if value > 2147483647:
                        value = value - 4294967296

                positions[mid] = value

            if all_available:
                return positions

            # Retry if not all data available
            time.sleep(retry_delay)

        self.stats['read_errors'] += 1
        return None

    def sync_write_positions_degrees(self, degrees_dict):
        """
        Write goal positions in degrees to multiple motors in ONE packet.

        Args:
            degrees_dict: {motor_id: degrees} mapping
                         Example: {1: 180.0, 2: 90.0, 3: 270.0}

        Returns:
            bool: True if successful
        """
        positions = {
            mid: int(deg * self.DEGREES_TO_POSITION)
            for mid, deg in degrees_dict.items()
        }
        return self.sync_write_positions(positions)

    def sync_read_positions_degrees(self):
        """
        Read current positions in degrees from all registered motors in ONE packet.

        Returns:
            dict: {motor_id: degrees} or None on error
        """
        positions = self.sync_read_positions()
        if positions is None:
            return None

        return {
            mid: pos * self.POSITION_TO_DEGREES
            for mid, pos in positions.items()
        }

    def cleanup_sync(self):
        """Clean up GroupSync handlers. Called automatically by disconnect()."""
        if self._sync_read_position is not None:
            self._sync_read_position.clearParam()
            self._sync_read_position = None

        if self._sync_write_position is not None:
            self._sync_write_position.clearParam()
            self._sync_write_position = None

        self._sync_motor_ids = []

    def zero_position(self, motor_id=None):
        """
        Set current position as zero using homing offset.
        Useful in Extended Position Mode to reset multi-turn count.

        Note: This modifies EEPROM (Homing Offset), requires torque off.

        Returns:
            bool: Success status
        """
        mid = motor_id if motor_id is not None else self.motor_id

        # Read current position
        current_pos = self.read_position(mid)
        if current_pos is None:
            return False

        # Disable torque to write EEPROM
        torque_was_on = self.get_torque(mid)
        if torque_was_on:
            self.set_torque(False, mid)

        # Set homing offset to negate current position
        # New position = actual position + homing offset
        # We want new position = 0, so homing offset = -actual position
        success = self.write_4_bytes(self.ADDR_HOMING_OFFSET, -current_pos, mid)

        if success:
            print(f"[INFO] Position zeroed. Homing offset set to {-current_pos}")

        return success

    # ==================== Velocity Control ====================

    def read_velocity(self, motor_id=None):
        """
        Read current velocity in raw units.
        Unit: ~0.229 rev/min per unit
        """
        return self.read_4_bytes(self.ADDR_PRESENT_VELOCITY, motor_id)

    def write_goal_velocity(self, velocity, motor_id=None):
        """
        Write goal velocity (for velocity control mode).
        Unit: ~0.229 rev/min per unit
        """
        return self.write_4_bytes(self.ADDR_GOAL_VELOCITY, velocity, motor_id)

    def set_profile_velocity(self, velocity, motor_id=None):
        """
        Set profile velocity for position control.
        0 = unlimited (max speed)
        """
        return self.write_4_bytes(self.ADDR_PROFILE_VELOCITY, velocity, motor_id)

    def set_profile_acceleration(self, acceleration, motor_id=None):
        """Set profile acceleration for position control."""
        return self.write_4_bytes(self.ADDR_PROFILE_ACCELERATION, acceleration, motor_id)

    # ==================== Current/PWM Control ====================

    def read_current(self, motor_id=None):
        """
        Read current in raw units.
        Unit: ~2.69 mA per unit (signed)
        """
        value = self.read_2_bytes(self.ADDR_PRESENT_CURRENT, motor_id)
        if value is not None and value > 32767:
            value = value - 65536  # Convert to signed
        return value

    def read_current_ma(self, motor_id=None):
        """Read current in milliamps."""
        raw = self.read_current(motor_id)
        return raw * 2.69 if raw is not None else None

    def write_goal_current(self, current, motor_id=None):
        """Write goal current (for current control modes)."""
        return self.write_2_bytes(self.ADDR_GOAL_CURRENT, current, motor_id)

    def read_pwm(self, motor_id=None):
        """Read present PWM value."""
        return self.read_2_bytes(self.ADDR_PRESENT_PWM, motor_id)

    def write_goal_pwm(self, pwm, motor_id=None):
        """Write goal PWM (for PWM control mode)."""
        return self.write_2_bytes(self.ADDR_GOAL_PWM, pwm, motor_id)

    # ==================== Status/Info ====================

    def read_temperature(self, motor_id=None):
        """Read internal temperature in Celsius."""
        return self.read_1_byte(self.ADDR_PRESENT_TEMPERATURE, motor_id)

    def read_voltage(self, motor_id=None):
        """
        Read input voltage in raw units.
        Unit: 0.1V per unit
        """
        return self.read_2_bytes(self.ADDR_PRESENT_INPUT_VOLTAGE, motor_id)

    def read_voltage_v(self, motor_id=None):
        """Read input voltage in Volts."""
        raw = self.read_voltage(motor_id)
        return raw * 0.1 if raw is not None else None

    def is_moving(self, motor_id=None):
        """Check if motor is currently moving."""
        value = self.read_1_byte(self.ADDR_MOVING, motor_id)
        return value == 1 if value is not None else None

    def get_hardware_error(self, motor_id=None):
        """
        Read hardware error status.

        Returns:
            dict: Error status flags or None on error
        """
        value = self.read_1_byte(self.ADDR_HARDWARE_ERROR_STATUS, motor_id)
        if value is None:
            return None

        return {
            'input_voltage_error': bool(value & 0x01),
            'motor_hall_sensor_error': bool(value & 0x02),
            'overheating_error': bool(value & 0x04),
            'motor_encoder_error': bool(value & 0x08),
            'electrical_shock_error': bool(value & 0x10),
            'overload_error': bool(value & 0x20),
            'raw_value': value,
        }

    # ==================== Operating Mode Configuration ====================

    def get_operating_mode(self, motor_id=None):
        """
        Get current operating mode.

        Returns:
            int: Operating mode value or None
            - 0: Current Control
            - 1: Velocity Control
            - 3: Position Control
            - 4: Extended Position Control
            - 5: Current-based Position Control
            - 16: PWM Control
        """
        mid = motor_id if motor_id is not None else self.motor_id
        mode = self.read_1_byte(self.ADDR_OPERATING_MODE, mid)
        if mode is not None:
            self._operating_mode_cache[mid] = mode
        return mode

    def set_operating_mode(self, mode, motor_id=None, auto_torque=True):
        """
        Set operating mode. Automatically handles torque disable/enable.

        Args:
            mode: Operating mode (0, 1, 3, 4, 5, or 16)
            motor_id: Motor ID (uses selected motor if None)
            auto_torque: If True, automatically disable torque before and
                        optionally re-enable after mode change

        Returns:
            bool: Success status
        """
        mid = motor_id if motor_id is not None else self.motor_id

        # Check if torque needs to be disabled
        torque_was_enabled = self.get_torque(mid)

        if torque_was_enabled and auto_torque:
            if not self.set_torque(False, mid):
                print(f"[ERROR] Failed to disable torque for mode change")
                return False

        # Set the operating mode
        success = self.write_1_byte(self.ADDR_OPERATING_MODE, mode, mid)

        if success:
            self._operating_mode_cache[mid] = mode
            mode_names = {
                0: 'Current Control',
                1: 'Velocity Control',
                3: 'Position Control',
                4: 'Extended Position Control',
                5: 'Current-based Position Control',
                16: 'PWM Control',
            }
            print(f"[INFO] Motor {mid}: Operating mode set to {mode_names.get(mode, mode)}")

        return success

    def set_position_control_mode(self, motor_id=None):
        """
        Switch to standard Position Control Mode (0-360 degrees).

        Returns:
            bool: Success status
        """
        return self.set_operating_mode(self.OPERATING_MODE_POSITION, motor_id)

    def set_extended_position_mode(self, motor_id=None):
        """
        Switch to Extended Position Control Mode (multi-turn, -256 to +256 revs).

        Returns:
            bool: Success status
        """
        return self.set_operating_mode(self.OPERATING_MODE_EXTENDED_POSITION, motor_id)

    def set_velocity_control_mode(self, motor_id=None):
        """
        Switch to Velocity Control Mode.

        Returns:
            bool: Success status
        """
        return self.set_operating_mode(self.OPERATING_MODE_VELOCITY, motor_id)

    def set_current_control_mode(self, motor_id=None):
        """
        Switch to Current Control Mode.

        Returns:
            bool: Success status
        """
        return self.set_operating_mode(self.OPERATING_MODE_CURRENT, motor_id)

    def set_pwm_control_mode(self, motor_id=None):
        """
        Switch to PWM Control Mode.

        Returns:
            bool: Success status
        """
        return self.set_operating_mode(self.OPERATING_MODE_PWM, motor_id)

    def is_extended_position_mode(self, motor_id=None):
        """
        Check if motor is in Extended Position Control Mode.

        Returns:
            bool: True if in extended position mode
        """
        mid = motor_id if motor_id is not None else self.motor_id

        # Use cached value if available, otherwise read
        if mid in self._operating_mode_cache:
            return self._operating_mode_cache[mid] == self.OPERATING_MODE_EXTENDED_POSITION

        mode = self.get_operating_mode(mid)
        return mode == self.OPERATING_MODE_EXTENDED_POSITION

    def get_motor_info(self, motor_id=None):
        """
        Get comprehensive motor information.

        Returns:
            dict: Motor information
        """
        mid = motor_id if motor_id is not None else self.motor_id

        return {
            'id': mid,
            'model_number': self.read_2_bytes(self.ADDR_MODEL_NUMBER, mid),
            'firmware_version': self.read_1_byte(self.ADDR_FIRMWARE_VERSION, mid),
            'baud_rate': self.read_1_byte(self.ADDR_BAUD_RATE, mid),
            'operating_mode': self.get_operating_mode(mid),
            'torque_enabled': self.get_torque(mid),
            'led_on': self.get_led(mid),
            'temperature_c': self.read_temperature(mid),
            'voltage_v': self.read_voltage_v(mid),
            'position': self.read_position(mid),
            'velocity': self.read_velocity(mid),
            'current_ma': self.read_current_ma(mid),
            'hardware_error': self.get_hardware_error(mid),
        }

    def print_status(self, motor_id=None):
        """Print formatted motor status."""
        info = self.get_motor_info(motor_id)

        mode_names = {
            0: 'Current Control',
            1: 'Velocity Control',
            3: 'Position Control (0-360°)',
            4: 'Extended Position (Multi-turn)',
            5: 'Current-based Position',
            16: 'PWM Control',
        }

        print(f"\n{'='*60}")
        print(f"Motor ID {info['id']} Status")
        print(f"{'='*60}")
        print(f"Model Number:     {info['model_number']}")
        print(f"Firmware Version: {info['firmware_version']}")
        print(f"Baud Rate Index:  {info['baud_rate']}")
        print(f"Operating Mode:   {mode_names.get(info['operating_mode'], info['operating_mode'])}")
        print(f"Torque Enabled:   {'Yes' if info['torque_enabled'] else 'No'}")
        print(f"LED:              {'On' if info['led_on'] else 'Off'}")
        print(f"Temperature:      {info['temperature_c']}°C")
        print(f"Voltage:          {info['voltage_v']:.1f}V" if info['voltage_v'] else "Voltage: N/A")

        # Position display
        if info['position'] is not None:
            pos = info['position']
            degrees = pos * self.POSITION_TO_DEGREES
            print(f"Position:         {pos} raw ({degrees:.1f}°)")
        else:
            print(f"Position:         N/A")

        print(f"Velocity:         {info['velocity']}")
        print(f"Current:          {info['current_ma']:.1f} mA" if info['current_ma'] else "Current: N/A")

        if info['hardware_error'] and info['hardware_error']['raw_value'] != 0:
            print(f"Hardware Error:   {info['hardware_error']}")
        else:
            print(f"Hardware Error:   None")
        print(f"{'='*60}\n")


# ==================== Convenience Test Function ====================

def main():
    """Test the controller with basic operations."""
    import time

    print("MX-64 Controller Test")
    print("=" * 60)

    # Create controller (will auto-scan ports)
    controller = MX64Controller()

    # Connect
    if not controller.connect():
        print("Failed to connect. Exiting.")
        return

    try:
        # Scan for motors
        motors = controller.scan_motors()

        if not motors:
            print("No motors found!")
            return

        # Select first motor
        first_motor_id = sorted(motors.keys())[0]
        controller.select_motor(first_motor_id)

        # Print initial status
        controller.print_status()

        # Test LED
        print("Testing LED...")
        controller.set_led(True)
        time.sleep(0.5)
        controller.set_led(False)
        print("LED test complete.\n")

        # Show current operating mode
        mode = controller.get_operating_mode()
        mode_names = {
            0: 'Current Control',
            1: 'Velocity Control',
            3: 'Position Control',
            4: 'Extended Position Control',
            5: 'Current-based Position Control',
            16: 'PWM Control',
        }
        print(f"Current operating mode: {mode_names.get(mode, mode)}")

        # Get position info
        pos = controller.read_position()
        if pos is not None:
            print(f"\nPosition Information:")
            print(f"  Raw:     {pos}")
            print(f"  Degrees: {pos * controller.POSITION_TO_DEGREES:.2f}°")

        # Interactive mode selection demo
        print("\n" + "=" * 60)
        print("Mode Switching Demo")
        print("=" * 60)
        print("\nAvailable mode switching methods:")
        print("  controller.set_position_control_mode()   - Standard 0-360°")
        print("  controller.set_extended_position_mode()  - Multi-turn ±256 revs")
        print("  controller.set_velocity_control_mode()   - Velocity control")
        print("  controller.set_current_control_mode()    - Current/torque control")
        print("  controller.set_pwm_control_mode()        - Direct PWM control")

        print("\nPosition methods:")
        print("  controller.read_position()               - Read raw position")
        print("  controller.write_position(2048)          - Write raw position")
        print("  controller.read_position_degrees()       - Read position in degrees")
        print("  controller.write_position_degrees(180)   - Write position in degrees")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
