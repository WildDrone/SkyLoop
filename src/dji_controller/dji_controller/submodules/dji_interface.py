"""
WildBridge - DJI Interface Module

A Python interface for controlling DJI drones through HTTP requests and TCP sockets,
providing seamless integration for drone operations, telemetry retrieval, and video streaming.

Authors: Kilian Meier, Edouard G.A. Rolland
Project: WildDrone - Autonomous Search and Rescue Operations
Institution: University of Bristol, University of Southern Denmark (SDU)
License: MIT

For more information, visit: https://github.com/WildDrone/WildBridge
"""

import requests
import ast
import json
import socket
import threading
from datetime import datetime

# HTTP POST Command Endpoints (port 8080)
EP_STICK = "/send/stick"  # expects a formatted string: "<leftX>,<leftY>,<rightX>,<rightY>"
EP_ZOOM = "/send/camera/zoom"
EP_GIMBAL_SET_PITCH = "/send/gimbal/pitch"
EP_GIMBAL_SET_YAW = "/send/gimbal/yaw"  # !!! This is the yaw joint angle !!!
EP_TAKEOFF = "/send/takeoff"
EP_LAND = "/send/land"
EP_RTH = "/send/RTH"
EP_ENABLE_VIRTUAL_STICK = "/send/enableVirtualStick"
EP_ABORT_MISSION = "/send/abortMission"
EP_GOTO_WP = "/send/gotoWP"
EP_GOTO_YAW = "/send/gotoYaw"
EP_GOTO_WP_PID = "/send/gotoWPwithPID"
EP_GOTO_TRAJECTORY = "/send/navigateTrajectory"
EP_GOTO_ALTITUDE = "/send/gotoAltitude"
EP_CAMERA_START_RECORDING = "/send/camera/startRecording"
EP_CAMERA_STOP_RECORDING = "/send/camera/stopRecording"
EP_GOTO_TRAJECTORY_DJI_NATIVE = "/send/navigateTrajectoryDJINative"
EP_ABORT_DJI_NATIVE_MISSION = "/send/abort/DJIMission"
EP_SET_RTH_ALTITUDE = "/send/setRTHAltitude"

# PID Tuning
EP_TUNING = "/send/gotoWPwithPIDtuning"


class DJIInterface:
    """
    Interface for DJI drone control via HTTP commands (port 8080) and 
    TCP telemetry socket (port 8081).
    """
    
    def __init__(self, IP_RC="192.168.50.27"):
        self.IP_RC = IP_RC
        self.baseCommandUrl = f"http://{IP_RC}:8080"
        self.telemetryPort = 8081
        self.videoSource = f"rtsp://aaa:aaa@{self.IP_RC}:8554/streaming/live/1"
        
        # Telemetry state (updated via TCP socket)
        self._telemetry = {}
        self._telemetry_lock = threading.Lock()
        self._telemetry_socket = None
        self._telemetry_thread = None
        self._running = False

    def getVideoSource(self):
        if self.IP_RC == "":
            return ""
        return self.videoSource

    # ==================== Telemetry (TCP Socket on port 8081) ====================
    
    def startTelemetryStream(self):
        """
        Start receiving telemetry data via TCP socket connection.
        The drone sends JSON telemetry data continuously.
        """
        if self._running:
            return
        
        self._running = True
        self._telemetry_thread = threading.Thread(target=self._telemetry_receiver, daemon=True)
        self._telemetry_thread.start()
    
    def stopTelemetryStream(self):
        """Stop the telemetry stream and close the socket."""
        self._running = False
        if self._telemetry_socket:
            try:
                self._telemetry_socket.close()
            except:
                pass
        if self._telemetry_thread:
            self._telemetry_thread.join(timeout=2)
    
    def _telemetry_receiver(self):
        """Background thread that receives telemetry data from TCP socket."""
        buffer = ""
        while self._running:
            try:
                # Connect to telemetry server
                self._telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._telemetry_socket.settimeout(5.0)
                self._telemetry_socket.connect((self.IP_RC, self.telemetryPort))
                
                while self._running:
                    data = self._telemetry_socket.recv(4096)
                    if not data:
                        break
                    
                    buffer += data.decode('utf-8')
                    
                    # Process complete JSON objects (separated by newlines)
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            try:
                                telemetry = json.loads(line)
                                with self._telemetry_lock:
                                    self._telemetry = telemetry
                                    self._telemetry["timestamp"] = datetime.now().strftime(
                                        "%Y-%m-%d_%H-%M-%S.%f")
                            except json.JSONDecodeError:
                                pass
                                
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Telemetry connection error: {e}")
                import time
                time.sleep(1)  # Wait before reconnecting
            finally:
                if self._telemetry_socket:
                    try:
                        self._telemetry_socket.close()
                    except:
                        pass
    
    def getTelemetry(self):
        """
        Get the latest telemetry data.
        Returns a dictionary with all telemetry fields from the drone.
        """
        with self._telemetry_lock:
            return self._telemetry.copy()
    
    def requestAllStates(self, verbose=False):
        """
        Get all aircraft states from telemetry.
        Note: You must call startTelemetryStream() first.
        """
        telemetry = self.getTelemetry()
        if verbose and telemetry:
            print("Telemetry:", json.dumps(telemetry, indent=2))
        return telemetry
    
    # Telemetry field accessors
    def getSpeed(self):
        """Get aircraft velocity (x, y, z)."""
        return self.getTelemetry().get("speed", {})
    
    def getHeading(self):
        """Get compass heading in degrees."""
        return self.getTelemetry().get("heading", 0.0)
    
    def getAttitude(self):
        """Get aircraft attitude (pitch, roll, yaw)."""
        return self.getTelemetry().get("attitude", {})
    
    def getLocation(self):
        """Get aircraft 3D location (latitude, longitude, altitude)."""
        return self.getTelemetry().get("location", {})
    
    def getGimbalAttitude(self):
        """Get gimbal attitude (pitch, roll, yaw)."""
        return self.getTelemetry().get("gimbalAttitude", {})
    
    def getGimbalJointAttitude(self):
        """Get gimbal joint attitude (pitch, roll, yaw)."""
        return self.getTelemetry().get("gimbalJointAttitude", {})
    
    def getZoomFocalLength(self):
        """Get camera zoom focal length."""
        return self.getTelemetry().get("zoomFl", -1)
    
    def getHybridFocalLength(self):
        """Get camera hybrid focal length."""
        return self.getTelemetry().get("hybridFl", -1)
    
    def getOpticalFocalLength(self):
        """Get camera optical focal length."""
        return self.getTelemetry().get("opticalFl", -1)
    
    def getZoomRatio(self):
        """Get camera zoom ratio."""
        return self.getTelemetry().get("zoomRatio", 1.0)
    
    def getBatteryLevel(self):
        """Get battery level percentage."""
        return self.getTelemetry().get("batteryLevel", -1)
    
    def getSatelliteCount(self):
        """Get GPS satellite count."""
        return self.getTelemetry().get("satelliteCount", -1)
    
    def getHomeLocation(self):
        """Get home location (latitude, longitude)."""
        return self.getTelemetry().get("homeLocation", {})
    
    def getDistanceToHome(self):
        """Get distance to home in meters."""
        return self.getTelemetry().get("distanceToHome", 0.0)
    
    def isWaypointReached(self):
        """Check if the current waypoint has been reached."""
        return self.getTelemetry().get("waypointReached", False)
    
    def isIntermediaryWaypointReached(self):
        """Check if an intermediary waypoint has been reached."""
        return self.getTelemetry().get("intermediaryWaypointReached", False)
    
    def isYawReached(self):
        """Check if the target yaw has been reached."""
        return self.getTelemetry().get("yawReached", False)
    
    def isAltitudeReached(self):
        """Check if the target altitude has been reached."""
        return self.getTelemetry().get("altitudeReached", False)
    
    def isCameraRecording(self):
        """Check if the camera is currently recording."""
        return self.getTelemetry().get("isRecording", False)
    
    def isHomeSet(self):
        """Check if the home location has been set."""
        return self.getTelemetry().get("homeSet", False)
    
    def getRemainingFlightTime(self):
        """Get remaining flight time in minutes."""
        return self.getTelemetry().get("remainingFlightTime", 0)
    
    def getTimeNeededToGoHome(self):
        """Get time needed to return home in minutes."""
        return self.getTelemetry().get("timeNeededToGoHome", 0)
    
    def getMaxRadiusCanFlyAndGoHome(self):
        """Get maximum radius the drone can fly and still return home."""
        return self.getTelemetry().get("maxRadiusCanFlyAndGoHome", 0)

    # ==================== Commands (HTTP on port 8080) ====================

    def requestGet(self, endPoint, verbose=False):
        """Send a GET request to the drone (for backward compatibility)."""
        if self.IP_RC == "":
            print(f"No IP_RC provided, returning empty string for request at {endPoint}")
            return ""
        try:
            response = requests.get(self.baseCommandUrl + endPoint, timeout=5)
            if verbose:
                print("EP : " + endPoint + "\t" + str(response.content, encoding="utf-8"))
            return response.content.decode('utf-8')
        except requests.exceptions.RequestException as e:
            print(f"Request error at {endPoint}: {e}")
            return ""

    def requestSend(self, endPoint, data, verbose=False):
        """Send a POST request to the drone."""
        if self.IP_RC == "":
            print(f"No IP_RC provided, returning empty string for request at {endPoint}")
            return ""
        try:
            response = requests.post(self.baseCommandUrl + endPoint, str(data), timeout=5)
            if verbose:
                print("EP : " + endPoint + "\t" + str(response.content, encoding="utf-8"))
            return response.content.decode('utf-8')
        except requests.exceptions.RequestException as e:
            print(f"Request error at {endPoint}: {e}")
            return ""

    def requestSendStick(self, leftX=0, leftY=0, rightX=0, rightY=0):
        """Send virtual stick commands. Values should be in [-1, 1]."""
        # Saturate values such that they are in [-1;1]
        s = 0.3
        leftX = max(-s, min(s, leftX))
        leftY = max(-s, min(s, leftY))
        rightX = max(-s, min(s, rightX))
        rightY = max(-s, min(s, rightY))
        rep = self.requestSend(
            EP_STICK, f"{leftX:.4f},{leftY:.4f},{rightX:.4f},{rightY:.4f}")
        return rep

    def requestSendGimbalPitch(self, pitch=0):
        """Set gimbal pitch angle."""
        return self.requestSend(EP_GIMBAL_SET_PITCH, f"0,{pitch},0")

    def requestSendGimbalYaw(self, yaw=0):
        """Set gimbal yaw angle."""
        return self.requestSend(EP_GIMBAL_SET_YAW, f"0,0,{yaw}")

    def requestSendZoomRatio(self, zoomRatio=1):
        """Set camera zoom ratio."""
        return self.requestSend(EP_ZOOM, zoomRatio)

    def requestSendTakeOff(self):
        """Command the drone to take off."""
        return self.requestSend(EP_TAKEOFF, "")

    def requestSendLand(self):
        """Command the drone to land."""
        return self.requestSend(EP_LAND, "")

    def requestSendRTH(self):
        """Command the drone to return to home."""
        return self.requestSend(EP_RTH, "")

    def requestSendGoToWP(self, latitude, longitude, altitude):
        """Navigate to a waypoint."""
        return self.requestSend(EP_GOTO_WP, f"{latitude},{longitude},{altitude}")

    def requestSendGoToWPwithPID(self, latitude, longitude, altitude, yaw):
        """Navigate to a waypoint with PID control."""
        return self.requestSend(EP_GOTO_WP_PID, f"{latitude},{longitude},{altitude},{yaw}")
    
    def requestSendGoToWPwithPIDtuning(self, latitude, longitude, altitude, yaw, kp_pos, ki_pos, kd_pos, kp_yaw, ki_yaw, kd_yaw):
        """Navigate to a waypoint with custom PID tuning parameters."""
        return self.requestSend(EP_TUNING, f"{latitude},{longitude},{altitude},{yaw},{kp_pos},{ki_pos},{kd_pos},{kp_yaw},{ki_yaw},{kd_yaw}")

    def requestSendNavigateTrajectory(self, waypoints, finalYaw=None):
        """
        Navigate through a series of waypoints.
        :param waypoints: A list of triples (latitude, longitude, altitude) for each waypoint,
                          or a list of dicts with 'lat', 'lng', 'y', and optionally 'final_heading'.
        :param finalYaw: The final yaw angle at the last waypoint (optional if using dict format).
        :return: The response from the server.
        """
        self.requestSendEnableVirtualStick()
        if not waypoints:
            raise ValueError("No waypoints provided")

        # Handle dict format (backwards compatible with old code)
        if isinstance(waypoints[0], dict):
            if len(waypoints) == 1:
                waypoint_data = waypoints[0]
                return self.requestSendGoToWPwithPID(
                    waypoint_data.get('lat'), 
                    waypoint_data.get('lng'),
                    waypoint_data.get('y'), 
                    waypoint_data.get('final_heading', 0)
                )
            
            segments = []
            for i, waypoint in enumerate(waypoints):
                lat = waypoint.get('lat')
                lon = waypoint.get('lng')
                alt = waypoint.get('y')
                yaw = waypoint.get('final_heading', 0)

                if i < len(waypoints) - 1:
                    segments.append(f"{lat},{lon},{alt}")
                else:
                    segments.append(f"{lat},{lon},{alt},{yaw}")

            message = ";".join(segments)
            return self.requestSend(EP_GOTO_TRAJECTORY, message)

        # Handle tuple format
        segments = []
        for i, (lat, lon, alt) in enumerate(waypoints):
            if i < len(waypoints) - 1:
                segments.append(f"{lat},{lon},{alt}")
            else:
                segments.append(f"{lat},{lon},{alt},{finalYaw}")

        message = ";".join(segments)
        return self.requestSend(EP_GOTO_TRAJECTORY, message)
    
    def requestSendNavigateTrajectoryDJINative(self, waypoints):
        """
        Send waypoints to be executed using DJI's native waypoint mission system.
        :param waypoints: A list of triples (latitude, longitude, altitude) for each waypoint.
        :return: The response from the server.
        """
        if not waypoints:
            raise ValueError("No waypoints provided")

        # Build the message format: "lat,lon,alt; lat,lon,alt; ..."
        segments = []
        for lat, lon, alt in waypoints:
            segments.append(f"{lat},{lon},{alt}")

        message = ";".join(segments)
        return self.requestSend(EP_GOTO_TRAJECTORY_DJI_NATIVE, message)
    
    def requestAbortDJINativeMission(self):
        """Abort the current DJI native waypoint mission."""
        return self.requestSend(EP_ABORT_DJI_NATIVE_MISSION, "")

    def requestAbortMission(self):
        """Abort the current mission and disable virtual stick."""
        return self.requestSend(EP_ABORT_MISSION, "")

    def requestSendEnableVirtualStick(self):
        """Enable virtual stick control mode."""
        return self.requestSend(EP_ENABLE_VIRTUAL_STICK, "")

    def requestSendGotoYaw(self, yaw):
        """Rotate to a specific yaw angle."""
        return self.requestSend(EP_GOTO_YAW, f"{yaw}")

    def requestSendGotoAltitude(self, altitude):
        """Navigate to a specific altitude."""
        return self.requestSend(EP_GOTO_ALTITUDE, f"{altitude}")

    def requestCameraStartRecording(self):
        """Start camera recording."""
        return self.requestSend(EP_CAMERA_START_RECORDING, "")

    def requestCameraStopRecording(self):
        """Stop camera recording."""
        return self.requestSend(EP_CAMERA_STOP_RECORDING, "")
    
    def requestSetRTHAltitude(self, altitude):
        """Set the return-to-home altitude in meters."""
        return self.requestSend(EP_SET_RTH_ALTITUDE, str(altitude))

    # ==================== Deprecated methods (kept for backward compatibility) ====================
    
    def requestSticks(self):
        """Deprecated: RC stick values are now available via getTelemetry()."""
        print("Warning: requestSticks() is deprecated. Use getTelemetry() instead.")
        return ""

    def requestWaypointStatus(self):
        """Deprecated: Use isWaypointReached() instead."""
        return str(self.isWaypointReached()).lower()

    def requestIntermediaryWaypointStatus(self):
        """Deprecated: Use isIntermediaryWaypointReached() instead."""
        return str(self.isIntermediaryWaypointReached()).lower()

    def requestYawStatus(self):
        """Deprecated: Use isYawReached() instead."""
        return str(self.isYawReached()).lower()

    def requestAltitudeStatus(self):
        """Deprecated: Use isAltitudeReached() instead."""
        return str(self.isAltitudeReached()).lower()

    def requestHomePosition(self):
        """Deprecated: Use getHomeLocation() instead."""
        return self.getHomeLocation()

    def requestCameraIsRecording(self):
        """Deprecated: Use isCameraRecording() instead."""
        return self.isCameraRecording()


# Alias for backward compatibility
DJIInterfaceLite = DJIInterface


class DJIControllerLite:
    """High-level controller for DJI drone operations."""
    
    def __init__(self, droneInterface):
        self.droneInterface = droneInterface
        self.GAIN_P = 1e-2

    def center(self, xPixelOffset, yPixelOffset):
        """
        Center the drone based on pixel offset.
        Image x axis is going from left to right,
        y is going from top to bottom,
        origin is at the center of image.
        """
        ctrlX = xPixelOffset * self.GAIN_P
        ctrlY = -yPixelOffset * self.GAIN_P
        self.droneInterface.requestSendStick(rightX=ctrlX, rightY=ctrlY)

    def gotoYaw(self, targetYaw):
        """Rotate to target yaw and wait until reached."""
        import time
        if targetYaw > 179 or targetYaw < -179:
            raise ValueError("Yaw must be in the range [-179, 179]")
        self.droneInterface.requestSendGotoYaw(targetYaw)
        while not self.droneInterface.isYawReached():
            time.sleep(0.1)

    def waitGPSFix(self):
        """Wait for GPS fix before starting waypoint navigation."""
        import time
        while True:
            location = self.droneInterface.getLocation()
            if location.get("latitude", 0) == 0:
                print("Waiting for GPS fix...")
                time.sleep(0.5)
            else:
                break

    def gotoWaypoint(self, latitude, longitude, altitude):
        """Navigate to waypoint and wait until reached."""
        import time
        self.droneInterface.requestSendGoToWP(latitude, longitude, altitude)
        while not self.droneInterface.isWaypointReached():
            time.sleep(0.1)
        print("Waypoint reached")

    def gotoWaypointwithPID(self, latitude, longitude, altitude, yaw):
        """Navigate to waypoint with PID control and wait until reached."""
        import time
        self.droneInterface.requestSendGoToWPwithPID(latitude, longitude, altitude, yaw)
        while not self.droneInterface.isWaypointReached():
            time.sleep(0.1)
        print("Waypoint reached")

    def gotoAltitude(self, altitude):
        """Navigate to altitude and wait until reached."""
        import time
        self.droneInterface.requestSendGotoAltitude(altitude)
        while not self.droneInterface.isAltitudeReached():
            time.sleep(0.1)
        print("Altitude reached")


if __name__ == '__main__':
    import time
    import sys
    
    IP_RC = "192.168.50.27"  # REPLACE WITH YOUR RC IP
    
    if len(sys.argv) > 1:
        IP_RC = sys.argv[1]

    print(f"Connecting to {IP_RC}...")
    dji = DJIInterface(IP_RC)

    # Start telemetry stream (TCP socket on port 8081)
    print("Starting telemetry stream...")
    dji.startTelemetryStream()
    
    # Wait for initial connection
    time.sleep(1)
    
    print("\n" + "="*60)
    print("TCP Telemetry Socket Test - Press Ctrl+C to stop")
    print("="*60 + "\n")
    
    try:
        while True:
            telemetry = dji.getTelemetry()
            
            if telemetry:
                # Clear screen effect by printing separator
                print("-" * 60)
                print(f"[{telemetry.get('timestamp', 'N/A')}]")
                print(f"  Battery:     {dji.getBatteryLevel()}%")
                print(f"  Satellites:  {dji.getSatelliteCount()}")
                print(f"  Heading:     {dji.getHeading():.1f}°")
                print(f"  Location:    {dji.getLocation()}")
                print(f"  Altitude:    {dji.getLocation().get('altitude', 'N/A')} m")
                print(f"  Speed:       {dji.getSpeed()}")
                print(f"  Attitude:    {dji.getAttitude()}")
                print(f"  Gimbal:      {dji.getGimbalAttitude()}")
                print(f"  Home Set:    {dji.isHomeSet()}")
                print(f"  Home Loc:    {dji.getHomeLocation()}")
                print(f"  Dist Home:   {dji.getDistanceToHome():.1f} m")
                print(f"  Recording:   {dji.isCameraRecording()}")
                print(f"  WP Reached:  {dji.isWaypointReached()}")
                print(f"  Yaw Reached: {dji.isYawReached()}")
                print(f"  Alt Reached: {dji.isAltitudeReached()}")
                print(f"  Flight Time: {dji.getRemainingFlightTime()} min remaining")
            else:
                print("Waiting for telemetry data...")
            
            time.sleep(0.1)  # Update every 100ms
            
    except KeyboardInterrupt:
        print("\n\nStopping telemetry stream...")
        dji.stopTelemetryStream()
        print("Done.")
