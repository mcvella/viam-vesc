#!/usr/bin/env python3
"""
Test script for improved VESC power control
"""

import asyncio
import sys
import os

# Add the src directory to the path so we can import the VESC module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from models.vesc import Vesc
from viam.proto.app.robot import ComponentConfig
from viam.utils import struct_to_dict


async def test_power_control():
    """Test the improved power control functionality"""
    
    # Create a mock config
    config = ComponentConfig()
    config.name = "test_vesc"
    config.attributes = struct_to_dict({
        "port": "/dev/ttyACM0",
        "baudrate": 115200,
        "timeout": 1.0,
        "debug": True
    })
    
    # Create VESC instance
    vesc = Vesc.new(config, {})
    
    try:
        print("Testing improved VESC power control...")
        print("=" * 50)
        
        # Test 1: Forward power
        print("\n1. Testing forward power (0.3)...")
        await vesc.set_power(0.3)
        await asyncio.sleep(3)  # Run for 3 seconds
        
        # Test 2: Stop
        print("\n2. Testing stop...")
        await vesc.stop()
        await asyncio.sleep(1)
        
        # Test 3: Reverse power
        print("\n3. Testing reverse power (-0.3)...")
        await vesc.set_power(-0.3)
        await asyncio.sleep(3)  # Run for 3 seconds
        
        # Test 4: Stop
        print("\n4. Testing stop...")
        await vesc.stop()
        await asyncio.sleep(1)
        
        # Test 5: Higher forward power
        print("\n5. Testing higher forward power (0.6)...")
        await vesc.set_power(0.6)
        await asyncio.sleep(3)  # Run for 3 seconds
        
        # Test 6: Stop
        print("\n6. Testing stop...")
        await vesc.stop()
        
        # Test 7: Quick direction changes
        print("\n7. Testing quick direction changes...")
        for i in range(5):
            print(f"   Direction change {i+1}/5")
            await vesc.set_power(0.2)
            await asyncio.sleep(0.5)
            await vesc.set_power(-0.2)
            await asyncio.sleep(0.5)
        
        await vesc.stop()
        
        print("\nAll tests completed successfully!")
        
    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        # Ensure motor is stopped
        try:
            await vesc.stop()
        except:
            pass


if __name__ == "__main__":
    asyncio.run(test_power_control()) 