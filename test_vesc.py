#!/usr/bin/env python3
"""
Simple test script for VESC communication
"""

import asyncio
import serial
import struct
import time
from typing import Optional


class VESCTester:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
    
    def connect(self):
        """Connect to VESC"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"Connected to VESC on {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect to VESC: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from VESC"""
        if self.serial_port:
            self.serial_port.close()
            print("Disconnected from VESC")
    
    def crc16(self, data: bytes) -> int:
        """Calculate CRC16 for VESC packet"""
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc &= 0xFFFF
        return crc
    
    def pack_packet(self, packet_id: int, payload: bytes = b'') -> bytes:
        """Pack a VESC packet with proper formatting and CRC"""
        packet = struct.pack('>B', packet_id) + payload
        crc = self.crc16(packet)
        packet += struct.pack('>H', crc)
        return packet
    
    def send_packet(self, packet_id: int, payload: bytes = b'') -> bool:
        """Send a packet to the VESC"""
        if not self.serial_port or not self.serial_port.is_open:
            print("Serial port not open")
            return False
        
        try:
            packet = self.pack_packet(packet_id, payload)
            self.serial_port.write(packet)
            self.serial_port.flush()
            return True
        except Exception as e:
            print(f"Failed to send packet: {e}")
            return False
    
    def read_packet(self, expected_id: Optional[int] = None) -> Optional[bytes]:
        """Read a packet from the VESC"""
        if not self.serial_port or not self.serial_port.is_open:
            return None
        
        try:
            # Read packet ID
            packet_id = self.serial_port.read(1)
            if not packet_id:
                return None
            
            packet_id = packet_id[0]
            print(f"Received packet ID: 0x{packet_id:02X}")
            
            # Read payload length (2 bytes)
            length_bytes = self.serial_port.read(2)
            if len(length_bytes) != 2:
                return None
            
            payload_length = struct.unpack('>H', length_bytes)[0]
            print(f"Payload length: {payload_length}")
            
            # Read payload
            payload = self.serial_port.read(payload_length)
            if len(payload) != payload_length:
                return None
            
            # Read CRC (2 bytes)
            crc_bytes = self.serial_port.read(2)
            if len(crc_bytes) != 2:
                return None
            
            received_crc = struct.unpack('>H', crc_bytes)[0]
            calculated_crc = self.crc16(struct.pack('>B', packet_id) + struct.pack('>H', payload_length) + payload)
            
            if received_crc != calculated_crc:
                print(f"CRC mismatch: received {received_crc}, calculated {calculated_crc}")
                return None
            
            if expected_id and packet_id != expected_id:
                print(f"Unexpected packet ID: received {packet_id}, expected {expected_id}")
                return None
            
            return payload
            
        except Exception as e:
            print(f"Failed to read packet: {e}")
            return None
    
    def test_connection(self):
        """Test basic VESC connection"""
        print("Testing VESC connection...")
        
        # Test 1: Send GET_VALUES command
        print("\n1. Testing GET_VALUES command...")
        if self.send_packet(0x27):  # COMM_GET_VALUES
            print("GET_VALUES command sent successfully")
            payload = self.read_packet(0x27)
            if payload:
                print(f"Received response: {len(payload)} bytes")
                print(f"Payload: {payload.hex()}")
            else:
                print("No response received")
        else:
            print("Failed to send GET_VALUES command")
        
        # Test 2: Send ALIVE command
        print("\n2. Testing ALIVE command...")
        if self.send_packet(0x3A):  # COMM_ALIVE
            print("ALIVE command sent successfully")
            payload = self.read_packet(0x3A)
            if payload:
                print(f"Received response: {len(payload)} bytes")
                print(f"Payload: {payload.hex()}")
            else:
                print("No response received")
        else:
            print("Failed to send ALIVE command")
        
        # Test 3: Send zero duty cycle
        print("\n3. Testing SET_DUTY command (zero)...")
        payload = struct.pack('>f', 0.0)
        if self.send_packet(0x00, payload):  # COMM_SET_DUTY
            print("SET_DUTY command sent successfully")
        else:
            print("Failed to send SET_DUTY command")


def main():
    """Main test function"""
    import sys
    
    # Get port from command line argument or use default
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    
    print(f"VESC Communication Test")
    print(f"Port: {port}")
    print(f"Baudrate: 115200")
    print("-" * 50)
    
    tester = VESCTester(port=port)
    
    if tester.connect():
        try:
            tester.test_connection()
        finally:
            tester.disconnect()
    else:
        print("Could not connect to VESC. Please check:")
        print("1. Serial port path is correct")
        print("2. VESC is powered on")
        print("3. Serial connection is properly wired")
        print("4. You have permission to access the serial port")


if __name__ == "__main__":
    main() 