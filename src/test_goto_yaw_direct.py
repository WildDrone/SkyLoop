#!/usr/bin/env python3
"""
Direct goto_yaw test - sends command directly to drone via HTTP POST.
No ROS required.

Usage:
    python3 test_goto_yaw_direct.py <IP> <YAW>
    
Example:
    python3 test_goto_yaw_direct.py 10.184.11.117 90
"""

import requests
import sys
import time

def send_command(ip: str, endpoint: str, data: str = ""):
    """Send POST command to drone."""
    url = f"http://{ip}:8080{endpoint}"
    try:
        resp = requests.post(url, data=data, timeout=5)
        return resp.text
    except Exception as e:
        return f"Error: {e}"

def send_goto_yaw(ip: str, yaw: float):
    """Send goto_yaw command directly to drone."""
    
    # First enable virtual stick (important for altitude stability)
    print(f"[1/2] Enabling virtual stick...")
    resp = send_command(ip, "/send/enableVirtualStick", "")
    print(f"      Response: {resp}")
    
    time.sleep(0.5)
    
    # Send goto yaw command
    print(f"[2/2] Sending goto_yaw({yaw}°)...")
    resp = send_command(ip, "/send/gotoYaw", str(yaw))
    print(f"      Response: {resp}")
    
    print(f"\n✓ Command sent! Drone should rotate to {yaw}°")


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 test_goto_yaw_direct.py <IP> <YAW>")
        print("Example: python3 test_goto_yaw_direct.py 10.184.11.117 90")
        sys.exit(1)
    
    ip = sys.argv[1]
    yaw = float(sys.argv[2])
    
    print(f"=" * 50)
    print(f"GOTO YAW TEST")
    print(f"Drone IP: {ip}")
    print(f"Target Yaw: {yaw}°")
    print(f"=" * 50)
    
    send_goto_yaw(ip, yaw)


if __name__ == '__main__':
    main()
