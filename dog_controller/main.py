#!/usr/bin/env python3
import asyncio
import aiohttp
from ROS2Controller import ROS2Controller

# Initialize ROS2 controller
ros_controller = ROS2Controller()

# API endpoint of the gesture web server
GESTURE_API_URL = "http://0.0.0.0:8000/gesture"  # Change to the actual IP if running externally

# Track the last sent gesture to avoid redundant messages
previous_gesture = None  

async def fetch_gesture():
    """Fetch gesture from the web API"""
    async with aiohttp.ClientSession() as session:
        try:
            async with session.get(GESTURE_API_URL) as response:
                if response.status == 200:
                    data = await response.json()
                    return data.get("gesture", "none")
                else:
                    print(f"API request failed with status {response.status}")
        except Exception as e:
            print(f"Error fetching gesture: {e}")
    return "none"

async def gesture_to_ros_loop():
    """Continuously fetch gesture and send to ROS2 if changed"""
    global previous_gesture

    while True:
        gesture = await fetch_gesture()

        if gesture != previous_gesture:  # Send only if gesture changed
            print(f"Sending gesture to ROS2: {gesture}")
            ros_controller.publish_to_topic("/active_gesture", "std_msgs/msg/String", f'{{data: "{gesture}"}}')
            previous_gesture = gesture

        await asyncio.sleep(0.2)  # Adjust polling interval if needed

if __name__ == "__main__":
    asyncio.run(gesture_to_ros_loop())
