#!/usr/bin/env python3
"""
Test script for goto_yaw command.
This script sends a yaw rotation command to a drone and monitors its heading.

Usage:
    ros2 run groundstation test_goto_yaw  (if installed)
    or
    python3 test_goto_yaw.py

Make sure the drone is flying and stable before running this test.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import sys


class GotoYawTester(Node):
    def __init__(self, namespace: str = "drone1"):
        super().__init__('goto_yaw_tester')
        self.namespace = namespace
        
        # Publisher for goto_yaw command
        self.yaw_publisher = self.create_publisher(
            Float64,
            f'{namespace}/command/goto_yaw',
            10
        )
        
        # Store current heading
        self.current_heading = None
        self.current_altitude = None
        
        # Subscribe to telemetry to monitor heading and altitude
        from std_msgs.msg import String
        import json
        
        self.telemetry_sub = self.create_subscription(
            String,
            f'{namespace}/telemetry',
            self.telemetry_callback,
            10
        )
        
        self.get_logger().info(f"GotoYaw Tester initialized for {namespace}")
        self.get_logger().info("Waiting for telemetry...")
        
    def telemetry_callback(self, msg):
        """Parse telemetry to get current heading and altitude."""
        try:
            import json
            data = json.loads(msg.data)
            
            if 'heading' in data:
                self.current_heading = data['heading']
            if 'alt' in data:
                self.current_altitude = data['alt']
                
        except Exception as e:
            pass
    
    def send_goto_yaw(self, target_yaw: float):
        """Send goto_yaw command."""
        msg = Float64()
        msg.data = float(target_yaw)
        
        self.get_logger().info(f"=" * 50)
        self.get_logger().info(f"Sending goto_yaw command: {target_yaw}°")
        if self.current_heading is not None:
            self.get_logger().info(f"Current heading: {self.current_heading:.1f}°")
        if self.current_altitude is not None:
            self.get_logger().info(f"Current altitude: {self.current_altitude:.1f}m")
        self.get_logger().info(f"=" * 50)
        
        self.yaw_publisher.publish(msg)
        
    def monitor_for_duration(self, duration: float = 10.0):
        """Monitor heading and altitude for a duration."""
        start_time = time.time()
        initial_altitude = self.current_altitude
        
        self.get_logger().info(f"Monitoring for {duration} seconds...")
        
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.5)
            
            if self.current_heading is not None and self.current_altitude is not None:
                alt_change = ""
                if initial_altitude is not None:
                    alt_diff = self.current_altitude - initial_altitude
                    alt_change = f" (Δalt: {alt_diff:+.1f}m)"
                    
                self.get_logger().info(
                    f"Heading: {self.current_heading:6.1f}° | "
                    f"Altitude: {self.current_altitude:5.1f}m{alt_change}"
                )
            
            time.sleep(1.0)
        
        # Final report
        if initial_altitude is not None and self.current_altitude is not None:
            total_alt_change = self.current_altitude - initial_altitude
            self.get_logger().info(f"=" * 50)
            self.get_logger().info(f"TEST COMPLETE")
            self.get_logger().info(f"Total altitude change: {total_alt_change:+.1f}m")
            if abs(total_alt_change) > 2.0:
                self.get_logger().warn(f"⚠️  ALTITUDE DRIFT DETECTED!")
            else:
                self.get_logger().info(f"✓ Altitude stable")
            self.get_logger().info(f"=" * 50)


def main():
    rclpy.init()
    
    # Parse arguments
    namespace = "drone1"
    target_yaw = None
    
    args = sys.argv[1:]
    
    # Help
    if '-h' in args or '--help' in args:
        print(__doc__)
        print("\nOptions:")
        print("  --ns NAMESPACE    Drone namespace (default: drone1)")
        print("  --yaw DEGREES     Target yaw angle (0-360)")
        print("  --rotate DEGREES  Relative rotation from current heading")
        print("\nExamples:")
        print("  python3 test_goto_yaw.py --ns drone1 --yaw 90")
        print("  python3 test_goto_yaw.py --ns drone1 --yaw 180")
        print("  python3 test_goto_yaw.py --ns drone1 --rotate 90")
        return
    
    # Parse namespace
    if '--ns' in args:
        idx = args.index('--ns')
        if idx + 1 < len(args):
            namespace = args[idx + 1]
    
    # Parse target yaw
    if '--yaw' in args:
        idx = args.index('--yaw')
        if idx + 1 < len(args):
            target_yaw = float(args[idx + 1])
    
    # Parse relative rotation
    relative_rotation = None
    if '--rotate' in args:
        idx = args.index('--rotate')
        if idx + 1 < len(args):
            relative_rotation = float(args[idx + 1])
    
    # Create tester
    tester = GotoYawTester(namespace)
    
    # Wait for telemetry
    print("Waiting for telemetry (5 seconds)...")
    for _ in range(10):
        rclpy.spin_once(tester, timeout_sec=0.5)
        if tester.current_heading is not None:
            break
    
    if tester.current_heading is None:
        print(f"❌ No telemetry received from {namespace}")
        print("Make sure the drone is connected and publishing telemetry.")
        tester.destroy_node()
        rclpy.shutdown()
        return
    
    print(f"✓ Connected to {namespace}")
    print(f"  Current heading: {tester.current_heading:.1f}°")
    print(f"  Current altitude: {tester.current_altitude:.1f}m")
    
    # Calculate target if relative rotation requested
    if relative_rotation is not None:
        target_yaw = (tester.current_heading + relative_rotation) % 360
        print(f"  Relative rotation: {relative_rotation}°")
        print(f"  Target heading: {target_yaw:.1f}°")
    
    # Interactive mode if no yaw specified
    if target_yaw is None:
        print("\n" + "=" * 50)
        print("INTERACTIVE MODE")
        print("=" * 50)
        print("Commands:")
        print("  Enter a number (0-360) to set absolute yaw")
        print("  +90 or -90 for relative rotation")
        print("  'q' to quit")
        print("=" * 50)
        
        while True:
            try:
                cmd = input("\nEnter yaw command: ").strip()
                
                if cmd.lower() == 'q':
                    break
                
                if cmd.startswith('+') or cmd.startswith('-'):
                    # Relative rotation
                    rotation = float(cmd)
                    target = (tester.current_heading + rotation) % 360
                    print(f"Rotating {rotation}° → target: {target:.1f}°")
                else:
                    # Absolute yaw
                    target = float(cmd) % 360
                    print(f"Going to absolute yaw: {target:.1f}°")
                
                tester.send_goto_yaw(target)
                tester.monitor_for_duration(15.0)
                
            except ValueError:
                print("Invalid input. Enter a number or 'q' to quit.")
            except KeyboardInterrupt:
                break
    else:
        # Single command mode
        print(f"\nSending yaw command to {target_yaw}°...")
        print("⚠️  Make sure the drone is flying and stable!")
        
        try:
            input("Press Enter to send command (Ctrl+C to cancel)...")
        except KeyboardInterrupt:
            print("\nCancelled.")
            tester.destroy_node()
            rclpy.shutdown()
            return
        
        tester.send_goto_yaw(target_yaw)
        tester.monitor_for_duration(15.0)
    
    tester.destroy_node()
    rclpy.shutdown()
    print("\nTest complete.")


if __name__ == '__main__':
    main()
