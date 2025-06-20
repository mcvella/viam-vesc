#!/usr/bin/env python3
"""
Example usage of the VESC motor module
"""

import asyncio
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions


async def main():
    """Example usage of VESC motor"""
    
    # Connect to the robot
    opts = RobotClient.Options.with_api_key(
        api_key="your_api_key_here",
        api_key_id="your_api_key_id_here"
    )
    
    robot = await RobotClient.at_address("your_robot_address", opts)
    
    try:
        # Get the VESC motor
        vesc_motor = robot.get_motor("vesc_motor")
        
        print("VESC Motor Control Example")
        print("=" * 30)
        
        # Check if motor is powered
        is_powered, power = await vesc_motor.is_powered()
        print(f"Motor powered: {is_powered}, Current power: {power}")
        
        # Check if motor is moving
        is_moving = await vesc_motor.is_moving()
        print(f"Motor moving: {is_moving}")
        
        # Get current position
        position = await vesc_motor.get_position()
        print(f"Current position: {position} revolutions")
        
        # Set motor to 50% power forward
        print("\nSetting motor to 50% power forward...")
        await vesc_motor.set_power(0.5)
        await asyncio.sleep(2)
        
        # Check status again
        is_powered, power = await vesc_motor.is_powered()
        is_moving = await vesc_motor.is_moving()
        print(f"After setting power - Powered: {is_powered}, Power: {power}, Moving: {is_moving}")
        
        # Set motor to 25% power reverse
        print("\nSetting motor to 25% power reverse...")
        await vesc_motor.set_power(-0.25)
        await asyncio.sleep(2)
        
        # Stop the motor
        print("\nStopping motor...")
        await vesc_motor.stop()
        
        # Set RPM to 1000
        print("\nSetting motor to 1000 RPM...")
        await vesc_motor.set_rpm(1000)
        await asyncio.sleep(3)
        
        # Stop again
        await vesc_motor.stop()
        
        # Go for 2 revolutions at 500 RPM
        print("\nGoing for 2 revolutions at 500 RPM...")
        await vesc_motor.go_for(500, 2.0)
        
        # Get final position
        position = await vesc_motor.get_position()
        print(f"Final position: {position} revolutions")
        
        # Example of using custom commands
        print("\nGetting VESC telemetry...")
        result = await vesc_motor.do_command({"command": "get_vesc_values"})
        print(f"Telemetry result: {result}")
        
        # Set motor current to 5A
        print("\nSetting motor current to 5A...")
        result = await vesc_motor.do_command({"command": "set_current", "current": 5.0})
        print(f"Current set result: {result}")
        await asyncio.sleep(2)
        
        # Stop motor
        await vesc_motor.stop()
        
        print("\nExample completed successfully!")
        
    finally:
        # Close the robot connection
        await robot.close()


if __name__ == "__main__":
    asyncio.run(main()) 