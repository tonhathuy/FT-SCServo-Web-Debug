"""
Servo Controller - Wrapper for scservo_sdk
"""
import threading
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass, asdict
from scservo_sdk import *

# SCS Servo Register Addresses (EEPROM - persistent)
ADDR_SCS_ID = 5                    # Servo ID (1-253)
ADDR_SCS_BAUD_RATE = 6             # Baud rate
ADDR_SCS_MIN_ANGLE_LIMIT = 9       # CW angle limit
ADDR_SCS_MAX_ANGLE_LIMIT = 11      # CCW angle limit
ADDR_SCS_LOCK = 55                 # EEPROM Lock (0=unlock, 1=lock)

# SCS Servo Register Addresses (RAM - volatile)
ADDR_SCS_TORQUE_ENABLE = 40
ADDR_SCS_GOAL_POSITION = 42
ADDR_SCS_GOAL_TIME = 46
ADDR_SCS_GOAL_SPEED = 46
ADDR_SCS_PRESENT_POSITION = 56
ADDR_SCS_PRESENT_SPEED = 58
ADDR_SCS_PRESENT_LOAD = 60
ADDR_SCS_PRESENT_VOLTAGE = 62
ADDR_SCS_PRESENT_TEMPERATURE = 63


@dataclass
class ServoInfo:
    id: int
    model_number: int
    position: int = 0
    speed: int = 0
    load: int = 0
    voltage: float = 0.0
    temperature: int = 0
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


# Global controller instance
controller = ServoController()

