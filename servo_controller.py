"""
Servo Controller - Wrapper for scservo_sdk
"""
import threading
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass, asdict
from scservo_sdk import *

# SCS Servo Register Addresses (EEPROM - persistent, need unlock to write)
ADDR_SCS_ID = 5                    # Servo ID (1-253)
ADDR_SCS_BAUD_RATE = 6             # Baud rate
ADDR_SCS_MIN_ANGLE_LIMIT = 9       # CW angle limit (2 bytes)
ADDR_SCS_MAX_ANGLE_LIMIT = 11      # CCW angle limit (2 bytes)
ADDR_SCS_OVERCURRENT_PROT = 38     # Overcurrent protection (2 bytes)
ADDR_SCS_VELOCITY_I_GAIN = 39      # Velocity I Gain (1 byte)

# SCS Servo Register Addresses (RAM - volatile)
ADDR_SCS_TORQUE_ENABLE = 40        # Torque enable (1 byte)
ADDR_SCS_GOAL_ACCELERATION = 41    # Goal acceleration (1 byte)
ADDR_SCS_GOAL_POSITION = 42        # Goal position (2 bytes)
ADDR_SCS_GOAL_TIME = 46            # Goal time/speed (2 bytes)
ADDR_SCS_GOAL_SPEED = 46
ADDR_SCS_TORQUE_LIMIT = 48         # Torque limit (2 bytes)
ADDR_SCS_LOCK = 55                 # EEPROM Lock (0=unlock, 1=lock)
ADDR_SCS_PRESENT_POSITION = 56     # Present position (2 bytes, read-only)
ADDR_SCS_PRESENT_SPEED = 58        # Present speed (2 bytes, read-only)
ADDR_SCS_PRESENT_LOAD = 60         # Present load (2 bytes, read-only)
ADDR_SCS_PRESENT_VOLTAGE = 62      # Present voltage (1 byte, read-only)
ADDR_SCS_PRESENT_TEMPERATURE = 63  # Present temperature (1 byte, read-only)

# SCS Servo Register Addresses (DEFAULT - motion profile parameters)
ADDR_SCS_MOVING_THRESHOLD = 80     # Moving threshold (1 byte)
ADDR_SCS_DTS = 81                  # DTs in ms (1 byte)
ADDR_SCS_VK = 82                   # Vk in ms (1 byte)  
ADDR_SCS_VMIN = 83                 # Vmin (1 byte)
ADDR_SCS_VMAX = 84                 # Vmax (1 byte)
ADDR_SCS_AMAX = 85                 # Amax - Max acceleration (1 byte)
ADDR_SCS_KACC = 86                 # KAcc (1 byte)

# Register definitions for UI with descriptions - Complete list from FT SCServo Debug
SERVO_REGISTERS = {
    # ============ EPROM REGISTERS (persistent, need unlock to write) ============
    'firmware_main': {
        'addr': 0, 'size': 1, 'area': 'EPROM', 'rw': 'r', 'name': 'Firmware Main Version',
        'desc': 'Main firmware version number (read-only).', 'unit': '', 'range': '0-255', 'default': '—'
    },
    'firmware_secondary': {
        'addr': 1, 'size': 1, 'area': 'EPROM', 'rw': 'r', 'name': 'Firmware Secondary Version',
        'desc': 'Secondary firmware version number (read-only).', 'unit': '', 'range': '0-255', 'default': '—'
    },
    'servo_main_ver': {
        'addr': 3, 'size': 1, 'area': 'EPROM', 'rw': 'r', 'name': 'Servo Main Version',
        'desc': 'Servo hardware main version (read-only).', 'unit': '', 'range': '0-255', 'default': '—'
    },
    'servo_sub_ver': {
        'addr': 4, 'size': 1, 'area': 'EPROM', 'rw': 'r', 'name': 'Servo Sub Version',
        'desc': 'Servo hardware sub version (read-only).', 'unit': '', 'range': '0-255', 'default': '—'
    },
    'id': {
        'addr': 5, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'ID',
        'desc': 'Unique servo ID (1-253). Each servo on the bus must have different ID.',
        'unit': '', 'range': '1-253', 'default': '1'
    },
    'baud_rate': {
        'addr': 6, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Baud Rate',
        'desc': 'Serial speed. 0=1Mbps, 1=500K, 2=250K, 3=128K, 4=115200, 5=76800, 6=57600, 7=38400',
        'unit': '', 'range': '0-7', 'default': '0'
    },
    'reserved': {
        'addr': 7, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Reserved',
        'desc': 'Reserved register.', 'unit': '', 'range': '0-255', 'default': '0'
    },
    'status_return_level': {
        'addr': 8, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Status Return Level',
        'desc': 'Response mode. 0=No response, 1=Respond to READ only, 2=Respond to all commands',
        'unit': '', 'range': '0-2', 'default': '1'
    },
    'min_angle': {
        'addr': 9, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Min Position Limit',
        'desc': 'Minimum angle limit. Servo will not rotate past this position.',
        'unit': 'steps', 'range': '0-4095', 'default': '0'
    },
    'max_angle': {
        'addr': 11, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Max Position Limit',
        'desc': 'Maximum angle limit. Servo will not rotate past this position.',
        'unit': 'steps', 'range': '0-4095', 'default': '4095'
    },
    'max_temp_limit': {
        'addr': 13, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Max Temperature Limit',
        'desc': 'Maximum temperature limit. Servo disables when exceeded.',
        'unit': '°C', 'range': '0-100', 'default': '70'
    },
    'max_voltage': {
        'addr': 14, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Max Input Voltage',
        'desc': 'Maximum input voltage limit. Value ÷10 = Volts.',
        'unit': '×0.1V', 'range': '0-255', 'default': '140'
    },
    'min_voltage': {
        'addr': 15, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Min Input Voltage',
        'desc': 'Minimum input voltage limit. Value ÷10 = Volts.',
        'unit': '×0.1V', 'range': '0-255', 'default': '40'
    },
    'max_torque': {
        'addr': 16, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Max Torque Limit',
        'desc': 'Maximum torque output limit. 1000=100%.',
        'unit': '‰', 'range': '0-1000', 'default': '1000'
    },
    'setting_byte': {
        'addr': 18, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Setting Byte',
        'desc': 'Configuration flags. Bit0=Direction, Bit1=Mode, etc.',
        'unit': '', 'range': '0-255', 'default': '12'
    },
    'protection_switch': {
        'addr': 19, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Protection Switch',
        'desc': 'Enable/disable various protection features (bitmask).',
        'unit': '', 'range': '0-255', 'default': '44'
    },
    'led_alarm': {
        'addr': 20, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'LED Alarm Condition',
        'desc': 'LED alarm trigger conditions (bitmask).',
        'unit': '', 'range': '0-255', 'default': '47'
    },
    'position_p_gain': {
        'addr': 21, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Position P Gain',
        'desc': 'Proportional gain (P) of position PID controller.',
        'unit': '', 'range': '0-255', 'default': '32'
    },
    'position_d_gain': {
        'addr': 22, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Position D Gain',
        'desc': 'Derivative gain (D) of position PID controller.',
        'unit': '', 'range': '0-255', 'default': '32'
    },
    'position_i_gain': {
        'addr': 23, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Position I Gain',
        'desc': 'Integral gain (I) of position PID controller.',
        'unit': '', 'range': '0-255', 'default': '0'
    },
    'punch': {
        'addr': 24, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Punch',
        'desc': 'Minimum PWM value applied to motor. Helps overcome static friction.',
        'unit': '', 'range': '0-1000', 'default': '16'
    },
    'max_i': {
        'addr': 25, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'MAX I',
        'desc': 'Maximum integral value for PID controller.',
        'unit': '', 'range': '0-255', 'default': '0'
    },
    'cw_dead_band': {
        'addr': 26, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'CW Dead Band',
        'desc': 'Clockwise dead band. Position error within this range is ignored.',
        'unit': 'steps', 'range': '0-255', 'default': '1'
    },
    'ccw_dead_band': {
        'addr': 27, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'CCW Dead Band',
        'desc': 'Counter-clockwise dead band. Position error within this range is ignored.',
        'unit': 'steps', 'range': '0-255', 'default': '1'
    },
    'overload_current': {
        'addr': 28, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Overload Current',
        'desc': 'Overload current threshold for protection.',
        'unit': 'mA', 'range': '0-1000', 'default': '310'
    },
    'angular_resolution': {
        'addr': 30, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Angular Resolution',
        'desc': 'Position resolution multiplier.',
        'unit': '', 'range': '0-255', 'default': '1'
    },
    'position_offset': {
        'addr': 31, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Position Offset Value',
        'desc': 'Position offset for calibration. Added to actual position.',
        'unit': 'steps', 'range': '-2048 to 2047', 'default': '0'
    },
    'work_mode': {
        'addr': 33, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Work Mode',
        'desc': 'Operating mode. 0=Position servo, 1=Wheel mode, 2=PWM mode, 3=Step mode',
        'unit': '', 'range': '0-3', 'default': '0'
    },
    'protect_torque': {
        'addr': 34, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Protect Torque',
        'desc': 'Torque level when protection is triggered.',
        'unit': '%', 'range': '0-100', 'default': '20'
    },
    'overload_protection_time': {
        'addr': 35, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Overload Protection Time',
        'desc': 'Time before overload protection triggers.',
        'unit': '×20ms', 'range': '0-255', 'default': '200'
    },
    'overload_torque': {
        'addr': 36, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Overload Torque',
        'desc': 'Torque threshold for overload detection.',
        'unit': '%', 'range': '0-100', 'default': '80'
    },
    'velocity_p_gain': {
        'addr': 37, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Velocity P Gain',
        'desc': 'Proportional gain (P) of velocity PID controller.',
        'unit': '', 'range': '0-255', 'default': '10'
    },
    'overcurrent': {
        'addr': 38, 'size': 2, 'area': 'EPROM', 'rw': 'rw', 'name': 'Overcurrent Protection',
        'desc': 'Overcurrent protection threshold.',
        'unit': 'mA', 'range': '0-1000', 'default': '200'
    },
    'velocity_i_gain': {
        'addr': 39, 'size': 1, 'area': 'EPROM', 'rw': 'rw', 'name': 'Velocity I Gain',
        'desc': 'Integral gain (I) of velocity PID controller.',
        'unit': '', 'range': '0-255', 'default': '200'
    },
    
    # ============ SRAM REGISTERS (volatile, reset on power cycle) ============
    'torque_enable': {
        'addr': 40, 'size': 1, 'area': 'SRAM', 'rw': 'rw', 'name': 'Torque Enable',
        'desc': 'Enable/disable torque. 0=Off (free), 1=On (hold position)',
        'unit': '', 'range': '0-1', 'default': '0'
    },
    'goal_acceleration': {
        'addr': 41, 'size': 1, 'area': 'SRAM', 'rw': 'rw', 'name': 'Goal Acceleration',
        'desc': 'Target acceleration. 0=No limit, higher=faster acceleration.',
        'unit': '', 'range': '0-255', 'default': '0'
    },
    'goal_position': {
        'addr': 42, 'size': 2, 'area': 'SRAM', 'rw': 'rw', 'name': 'Goal Position',
        'desc': 'Target position for servo to move to.',
        'unit': 'steps', 'range': '0-4095', 'default': '—'
    },
    'goal_pwm': {
        'addr': 44, 'size': 2, 'area': 'SRAM', 'rw': 'rw', 'name': 'Goal PWM',
        'desc': 'Target PWM value (for PWM mode).',
        'unit': '', 'range': '-1000 to 1000', 'default': '0'
    },
    'goal_speed': {
        'addr': 46, 'size': 2, 'area': 'SRAM', 'rw': 'rw', 'name': 'Goal Velocity',
        'desc': 'Target velocity or time to reach position.',
        'unit': 'steps/s or ms', 'range': '0-65535', 'default': '0'
    },
    'torque_limit': {
        'addr': 48, 'size': 2, 'area': 'SRAM', 'rw': 'rw', 'name': 'Torque Limit',
        'desc': 'Runtime torque limit. 1000=100%.',
        'unit': '‰', 'range': '0-1000', 'default': '1000'
    },
    'lock': {
        'addr': 55, 'size': 1, 'area': 'SRAM', 'rw': 'rw', 'name': 'Lock',
        'desc': 'EEPROM lock. 0=Unlocked (allow write), 1=Locked (protect)',
        'unit': '', 'range': '0-1', 'default': '1'
    },
    'present_position': {
        'addr': 56, 'size': 2, 'area': 'SRAM', 'rw': 'r', 'name': 'Present Position',
        'desc': 'Current servo position.',
        'unit': 'steps', 'range': '0-4095', 'default': '—'
    },
    'present_speed': {
        'addr': 58, 'size': 2, 'area': 'SRAM', 'rw': 'r', 'name': 'Present Velocity',
        'desc': 'Current servo velocity. Negative=reverse.',
        'unit': 'steps/s', 'range': '±32767', 'default': '—'
    },
    'present_pwm': {
        'addr': 60, 'size': 2, 'area': 'SRAM', 'rw': 'r', 'name': 'Present PWM',
        'desc': 'Current PWM output value.',
        'unit': '', 'range': '±1000', 'default': '—'
    },
    'present_voltage': {
        'addr': 62, 'size': 1, 'area': 'SRAM', 'rw': 'r', 'name': 'Present Input Voltage',
        'desc': 'Current input voltage. Value ÷10 = Volts.',
        'unit': '×0.1V', 'range': '0-255', 'default': '—'
    },
    'present_temp': {
        'addr': 63, 'size': 1, 'area': 'SRAM', 'rw': 'r', 'name': 'Present Temperature',
        'desc': 'Current temperature.',
        'unit': '°C', 'range': '0-100', 'default': '—'
    },
    'sync_write_flag': {
        'addr': 64, 'size': 1, 'area': 'SRAM', 'rw': 'r', 'name': 'Sync Write Flag',
        'desc': 'Indicates if sync write command was received.',
        'unit': '', 'range': '0-1', 'default': '—'
    },
    'hardware_error': {
        'addr': 65, 'size': 1, 'area': 'SRAM', 'rw': 'r', 'name': 'Hardware Error Status',
        'desc': 'Hardware error flags (bitmask).',
        'unit': '', 'range': '0-255', 'default': '—'
    },
    'moving_status': {
        'addr': 66, 'size': 1, 'area': 'SRAM', 'rw': 'r', 'name': 'Moving Status',
        'desc': '0=Stopped, 1=Moving to goal position.',
        'unit': '', 'range': '0-1', 'default': '—'
    },
    'present_current': {
        'addr': 69, 'size': 2, 'area': 'SRAM', 'rw': 'r', 'name': 'Present Current',
        'desc': 'Current draw.',
        'unit': 'mA', 'range': '0-65535', 'default': '—'
    },
    
    # ============ DEFAULT REGISTERS (motion profile parameters) ============
    'moving_threshold': {
        'addr': 80, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'Moving Threshold',
        'desc': 'Threshold to detect if servo is moving.',
        'unit': '', 'range': '0-255', 'default': '1'
    },
    'dts': {
        'addr': 81, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'DTs(ms)',
        'desc': 'Dead time before movement. Smooths direction changes.',
        'unit': 'ms', 'range': '0-255', 'default': '20'
    },
    'vk': {
        'addr': 82, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'Vk(ms)',
        'desc': 'Velocity constant. Affects velocity curve smoothness.',
        'unit': 'ms', 'range': '0-255', 'default': '50'
    },
    'vmin': {
        'addr': 83, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'Vmin',
        'desc': 'Minimum velocity. Servo wont move slower than this.',
        'unit': 'steps/s', 'range': '0-255', 'default': '1'
    },
    'vmax': {
        'addr': 84, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'Vmax',
        'desc': 'Maximum velocity. Servo wont move faster than this.',
        'unit': 'steps/s ×50', 'range': '0-255', 'default': '65'
    },
    'amax': {
        'addr': 85, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'Amax',
        'desc': 'Maximum acceleration. High=sharp, Low=smooth motion.',
        'unit': '', 'range': '0-254', 'default': '50'
    },
    'kacc': {
        'addr': 86, 'size': 1, 'area': 'DEFAULT', 'rw': 'rw', 'name': 'KAcc',
        'desc': 'Acceleration coefficient. Multiplied with Amax.',
        'unit': '', 'range': '0-255', 'default': '1'
    },
}


@dataclass
class ServoInfo:
    id: int
    model_number: int
    position: int = 0
    speed: int = 0
    load: int = 0
    voltage: float = 0.0
    temperature: int = 0
    present_current: int = 0
    moving_status: int = 0
    goal_position: int = 0
    is_online: bool = True
    min_position: int = 0
    max_position: int = 4095
    center_position: int = 2048


class ServoController:
    def __init__(self, port: str = '/dev/tty.usbmodem5A7C1167091', baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.port_handler: Optional[PortHandler] = None
        self.packet_handler: Optional[PacketHandler] = None
        self.servos: Dict[int, ServoInfo] = {}
        self.is_connected = False
        self._lock = threading.Lock()
    
    def connect(self) -> Tuple[bool, str]:
        """Connect to the serial port"""
        try:
            self.port_handler = PortHandler(self.port)
            self.packet_handler = PacketHandler(0)
            
            if not self.port_handler.openPort():
                return False, f"Cannot open port {self.port}"
            
            if not self.port_handler.setBaudRate(self.baudrate):
                return False, f"Cannot set baudrate {self.baudrate}"
            
            self.is_connected = True
            return True, "Connected successfully"
        except Exception as e:
            return False, str(e)
    
    def disconnect(self):
        """Disconnect from the serial port"""
        if self.port_handler:
            self.port_handler.closePort()
        self.is_connected = False
        self.servos.clear()
    
    def scan_servos(self, start_id: int = 1, end_id: int = 20) -> List[ServoInfo]:
        """Scan for connected servos in ID range"""
        if not self.is_connected:
            return []
        
        found_servos = []
        with self._lock:
            for servo_id in range(start_id, end_id + 1):
                model_number, comm_result, error = self.packet_handler.ping(
                    self.port_handler, servo_id
                )
                if comm_result == COMM_SUCCESS:
                    servo = ServoInfo(id=servo_id, model_number=model_number)
                    self.servos[servo_id] = servo
                    found_servos.append(servo)
        
        return found_servos
    
    def ping(self, servo_id: int) -> Tuple[bool, str]:
        """Ping a specific servo"""
        if not self.is_connected:
            return False, "Not connected"
        
        with self._lock:
            model_number, comm_result, error = self.packet_handler.ping(
                self.port_handler, servo_id
            )
            if comm_result == COMM_SUCCESS:
                return True, f"Servo {servo_id} online, model: {model_number}"
            else:
                return False, f"Servo {servo_id} not responding"
    
    def read_position(self, servo_id: int) -> Optional[int]:
        """Read current position of a servo"""
        if not self.is_connected:
            return None
        
        with self._lock:
            position, comm_result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_POSITION
            )
            if comm_result == COMM_SUCCESS:
                # Handle signed value (position can be negative for some servos)
                if position > 32767:
                    position = position - 65536
                if servo_id in self.servos:
                    self.servos[servo_id].position = position
                return position
            return None
    
    def read_status(self, servo_id: int) -> Optional[Dict]:
        """Read full status of a servo"""
        if not self.is_connected or servo_id not in self.servos:
            return None
        
        servo = self.servos[servo_id]
        
        with self._lock:
            # Read position
            pos, res, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_POSITION
            )
            if res == COMM_SUCCESS:
                if pos > 32767:
                    pos = pos - 65536
                servo.position = pos
            
            # Read speed
            speed, res, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_SPEED
            )
            if res == COMM_SUCCESS:
                if speed > 32767:
                    speed = speed - 65536
                servo.speed = speed
            
            # Read load
            load, res, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_LOAD
            )
            if res == COMM_SUCCESS:
                if load > 32767:
                    load = load - 65536
                servo.load = load
            
            # Read voltage
            voltage, res, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_VOLTAGE
            )
            if res == COMM_SUCCESS:
                servo.voltage = voltage / 10.0
            
            # Read temperature
            temp, res, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_TEMPERATURE
            )
            if res == COMM_SUCCESS:
                servo.temperature = temp
            
            # Read current
            current, res, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, 69  # ADDR_SCS_PRESENT_CURRENT
            )
            if res == COMM_SUCCESS:
                servo.present_current = current
            
            # Read moving status
            moving, res, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, 66  # ADDR_SCS_MOVING_STATUS
            )
            if res == COMM_SUCCESS:
                servo.moving_status = moving
            
            # Read goal position
            goal, res, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_GOAL_POSITION
            )
            if res == COMM_SUCCESS:
                servo.goal_position = goal
        
        return asdict(servo)
    
    def set_position(self, servo_id: int, position: int, time_ms: int = 500) -> Tuple[bool, str]:
        """Set servo position"""
        if not self.is_connected:
            return False, "Not connected"
        
        # Clamp position
        position = max(0, min(4095, position))
        
        with self._lock:
            # Write goal time
            self.packet_handler.write2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_GOAL_TIME, time_ms
            )
            # Write goal position
            comm_result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_GOAL_POSITION, position
            )
            
            if comm_result == COMM_SUCCESS:
                return True, f"Moving servo {servo_id} to {position}"
            else:
                return False, f"Failed to move servo {servo_id}"
    
    def set_torque(self, servo_id: int, enable: bool) -> Tuple[bool, str]:
        """Enable/disable servo torque"""
        if not self.is_connected:
            return False, "Not connected"
        
        with self._lock:
            comm_result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_TORQUE_ENABLE, 1 if enable else 0
            )
            
            if comm_result == COMM_SUCCESS:
                return True, f"Torque {'enabled' if enable else 'disabled'} for servo {servo_id}"
            else:
                return False, f"Failed to set torque for servo {servo_id}"
    
    def center_servo(self, servo_id: int) -> Tuple[bool, str]:
        """Move servo to center position (2048)"""
        return self.set_position(servo_id, 2048, 1000)
    
    def test_range(self, servo_id: int, min_pos: int = 0, max_pos: int = 4095) -> Tuple[bool, str]:
        """Test servo full range of motion"""
        if not self.is_connected:
            return False, "Not connected"
        
        # Move to min
        success, msg = self.set_position(servo_id, min_pos, 1500)
        if not success:
            return False, msg
        
        return True, f"Testing range {min_pos} -> {max_pos} for servo {servo_id}"
    
    def get_all_servos(self) -> List[Dict]:
        """Get list of all known servos"""
        return [asdict(s) for s in self.servos.values()]
    
    def set_calibration(self, servo_id: int, min_pos: int, max_pos: int, center_pos: int):
        """Set calibration values for a servo"""
        if servo_id in self.servos:
            self.servos[servo_id].min_position = min_pos
            self.servos[servo_id].max_position = max_pos
            self.servos[servo_id].center_position = center_pos
    
    def unlock_eeprom(self, servo_id: int) -> bool:
        """Unlock EEPROM for writing (required before changing ID)"""
        with self._lock:
            comm_result, _ = self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_LOCK, 0
            )
            return comm_result == COMM_SUCCESS
    
    def lock_eeprom(self, servo_id: int) -> bool:
        """Lock EEPROM after writing"""
        with self._lock:
            comm_result, _ = self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_LOCK, 1
            )
            return comm_result == COMM_SUCCESS
    
    def change_servo_id(self, old_id: int, new_id: int) -> Tuple[bool, str]:
        """
        Change servo ID. 
        ⚠️ IMPORTANT: Only connect ONE servo when changing ID!
        """
        if not self.is_connected:
            return False, "Not connected"
        
        if new_id < 1 or new_id > 253:
            return False, "ID must be between 1 and 253"
        
        if old_id == new_id:
            return False, "New ID is same as old ID"
        
        # First ping to make sure old_id exists
        success, _ = self.ping(old_id)
        if not success:
            return False, f"Servo ID {old_id} not found"
        
        # Check if new_id already exists
        success, _ = self.ping(new_id)
        if success:
            return False, f"ID {new_id} already in use by another servo!"
        
        try:
            # Disable torque first
            self.set_torque(old_id, False)
            
            # Unlock EEPROM
            if not self.unlock_eeprom(old_id):
                return False, "Failed to unlock EEPROM"
            
            # Write new ID
            with self._lock:
                comm_result, _ = self.packet_handler.write1ByteTxRx(
                    self.port_handler, old_id, ADDR_SCS_ID, new_id
                )
            
            if comm_result != COMM_SUCCESS:
                self.lock_eeprom(old_id)
                return False, "Failed to write new ID"
            
            # Lock EEPROM with new ID
            self.lock_eeprom(new_id)
            
            # Verify new ID works
            import time
            time.sleep(0.1)
            success, _ = self.ping(new_id)
            if success:
                # Update internal servo list
                if old_id in self.servos:
                    servo = self.servos.pop(old_id)
                    servo.id = new_id
                    self.servos[new_id] = servo
                return True, f"Successfully changed ID from {old_id} to {new_id}"
            else:
                return False, "ID changed but verification failed. Try scanning again."
                
        except Exception as e:
            return False, f"Error changing ID: {str(e)}"
    
    def read_servo_id(self, servo_id: int) -> Optional[int]:
        """Read the stored ID from a servo"""
        if not self.is_connected:
            return None
        
        with self._lock:
            read_id, comm_result, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_ID
            )
            if comm_result == COMM_SUCCESS:
                return read_id
            return None
    
    def read_register(self, servo_id: int, addr: int, size: int) -> Optional[int]:
        """Read a register value from servo"""
        if not self.is_connected:
            return None
        
        with self._lock:
            if size == 1:
                value, comm_result, _ = self.packet_handler.read1ByteTxRx(
                    self.port_handler, servo_id, addr
                )
            else:  # size == 2
                value, comm_result, _ = self.packet_handler.read2ByteTxRx(
                    self.port_handler, servo_id, addr
                )
            
            if comm_result == COMM_SUCCESS:
                return value
            return None
    
    def write_register(self, servo_id: int, addr: int, size: int, value: int, unlock_eeprom: bool = False) -> Tuple[bool, str]:
        """Write a value to servo register"""
        if not self.is_connected:
            return False, "Not connected"
        
        try:
            # For EEPROM registers, unlock first
            if unlock_eeprom:
                self.unlock_eeprom(servo_id)
            
            with self._lock:
                if size == 1:
                    comm_result, _ = self.packet_handler.write1ByteTxRx(
                        self.port_handler, servo_id, addr, value
                    )
                else:  # size == 2
                    comm_result, _ = self.packet_handler.write2ByteTxRx(
                        self.port_handler, servo_id, addr, value
                    )
            
            # Lock EEPROM after writing
            if unlock_eeprom:
                self.lock_eeprom(servo_id)
            
            if comm_result == COMM_SUCCESS:
                return True, f"Written {value} to address {addr}"
            else:
                return False, f"Failed to write to address {addr}"
        except Exception as e:
            return False, str(e)
    
    def read_all_registers(self, servo_id: int) -> Dict:
        """Read all defined registers from a servo"""
        if not self.is_connected:
            return {}
        
        result = {}
        for key, reg in SERVO_REGISTERS.items():
            value = self.read_register(servo_id, reg['addr'], reg['size'])
            result[key] = {
                'addr': reg['addr'],
                'name': reg['name'],
                'value': value,
                'area': reg['area'],
                'rw': reg['rw'],
                'size': reg['size'],
                'desc': reg.get('desc', ''),
                'unit': reg.get('unit', ''),
                'range': reg.get('range', ''),
                'default': reg.get('default', '')
            }
        
        return result
    
    def write_motion_params(self, servo_id: int, amax: int = None, vmax: int = None, 
                           vmin: int = None, dts: int = None, vk: int = None) -> Tuple[bool, str]:
        """Write motion profile parameters"""
        if not self.is_connected:
            return False, "Not connected"
        
        results = []
        if amax is not None:
            success, msg = self.write_register(servo_id, ADDR_SCS_AMAX, 1, amax)
            results.append(f"Amax: {'OK' if success else 'FAIL'}")
        
        if vmax is not None:
            success, msg = self.write_register(servo_id, ADDR_SCS_VMAX, 1, vmax)
            results.append(f"Vmax: {'OK' if success else 'FAIL'}")
        
        if vmin is not None:
            success, msg = self.write_register(servo_id, ADDR_SCS_VMIN, 1, vmin)
            results.append(f"Vmin: {'OK' if success else 'FAIL'}")
        
        if dts is not None:
            success, msg = self.write_register(servo_id, ADDR_SCS_DTS, 1, dts)
            results.append(f"DTs: {'OK' if success else 'FAIL'}")
        
        if vk is not None:
            success, msg = self.write_register(servo_id, ADDR_SCS_VK, 1, vk)
            results.append(f"Vk: {'OK' if success else 'FAIL'}")
        
        return True, ", ".join(results)


# Global controller instance
controller = ServoController()

