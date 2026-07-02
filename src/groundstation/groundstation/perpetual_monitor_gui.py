"""
Perpetual Monitoring GUI

NiceGUI-based web interface for the perpetual drone monitoring system.
Uses NiceGUI Events for thread-safe communication between ROS2 callbacks and UI.

Author: Edouard Rolland
Project: WildDrone
"""

import asyncio
import math
import threading
from typing import Dict
from pathlib import Path
from datetime import datetime

import rclpy
import time
from rclpy.executors import ExternalShutdownException

from nicegui import Event, app, ui, ui_run

from groundstation.arrow import Arrow
from groundstation.perpetual_monitor import (
    PerpetualMonitorNode, DroneData, DroneState,
    MonitoringPoint, RelayMission, DroneRTHPredictor
)
from groundstation.mission_controller import MissionController, MissionState, MissionMode


# ============================================================================
# STATIC FILES
# ============================================================================

app.add_static_files('/static', str(Path(__file__).parent / 'static'))


# ============================================================================
# GUI NODE WITH NICEGUI EVENTS
# ============================================================================



def _fullscreen_video_html(namespace):
    return f'''
            <div id="videoContainer_{namespace}" style="position: relative; width: 100vw; height: 100vh;">
                <video id="fullscreenVideo_{namespace}" autoplay playsinline muted 
                       style="width: 100%; height: 100%; object-fit: contain; background: #000;"></video>
                
                <!-- Detection canvas overlay -->
                <canvas id="detectionCanvas_{namespace}" style="
                    position: absolute;
                    top: 0;
                    left: 0;
                    width: 100%;
                    height: 100%;
                    pointer-events: none;
                    z-index: 40;
                "></canvas>
                
                <!-- YOLO Stats overlay - top center -->
                <div id="yoloStats_{namespace}" style="
                    position: absolute;
                    top: 80px;
                    left: 50%;
                    transform: translateX(-50%);
                    background: rgba(128, 0, 128, 0.85);
                    color: #fff;
                    font-family: 'Courier New', monospace;
                    font-size: 12px;
                    padding: 8px 15px;
                    border-radius: 8px;
                    z-index: 50;
                    display: none;
                ">
                    <span id="yoloInferenceTime_{namespace}">Inference: -- ms</span> | 
                    <span id="yoloDetections_{namespace}">Detections: 0</span>
                </div>
                
                <!-- Drone Telemetry overlay - bottom left -->
                <div id="telemetryOverlay_{namespace}" style="
                    position: absolute;
                    bottom: 20px;
                    left: 20px;
                    background: rgba(0, 0, 0, 0.8);
                    color: #00ff00;
                    font-family: 'Courier New', monospace;
                    font-size: 13px;
                    padding: 15px;
                    border-radius: 8px;
                    border: 1px solid rgba(0, 255, 0, 0.3);
                    min-width: 320px;
                    z-index: 50;
                    line-height: 1.6;
                ">
                    <div style="font-weight: bold; margin-bottom: 10px; color: #fff; font-size: 16px;">🚁 Drone Telemetry</div>
                    
                    <!-- GPS Section -->
                    <div style="color: #00ffff;">
                        <div id="telem_gps_{namespace}">📍 GPS: --</div>
                        <div id="telem_altitude_{namespace}">🏔️ Alt: ASL -- AGL --</div>
                        <div id="telem_sats_{namespace}">🛰️ Satellites: --</div>
                    </div>
                    
                    <!-- Gimbal Section -->
                    <div style="margin-top: 8px; color: #ffff00;">
                        <div id="telem_gimbal_{namespace}">🎥 Gimbal P:-- Y:-- R:--</div>
                    </div>
                    
                    <!-- Attitude Section -->
                    <div style="margin-top: 8px; color: #ffa500;">
                        <div id="telem_attitude_{namespace}">✈️ Attitude P:-- Y:-- R:--</div>
                        <div id="telem_heading_{namespace}">🧭 Heading: --°</div>
                    </div>
                    
                    <!-- Velocity Section -->
                    <div style="margin-top: 8px; color: #00bfff;">
                        <div id="telem_velocity_{namespace}">💨 Speed: -- m/s</div>
                    </div>
                    
                    <!-- Battery Section -->
                    <div style="margin-top: 8px;">
                        <div id="telem_battery_{namespace}">🔋 Battery: --%</div>
                    </div>
                    
                    <!-- Frame Info -->
                    <div style="margin-top: 10px; border-top: 1px solid rgba(255,255,255,0.2); padding-top: 8px; color: #aaa;">
                        <div id="telem_frame_{namespace}">📹 Frame: --</div>
                    </div>
                </div>
                
                <!-- Stream Stats overlay - bottom right -->
                <div id="streamStatsOverlay_{namespace}" style="
                    position: absolute;
                    bottom: 20px;
                    right: 20px;
                    background: rgba(0, 0, 0, 0.75);
                    color: #00ff00;
                    font-family: 'Courier New', monospace;
                    font-size: 12px;
                    padding: 12px;
                    border-radius: 8px;
                    border: 1px solid rgba(0, 255, 0, 0.3);
                    min-width: 200px;
                    z-index: 50;
                ">
                    <div style="font-weight: bold; margin-bottom: 8px; color: #fff; font-size: 14px;">📊 Stream Stats</div>
                    <div id="meta_resolution_{namespace}">Resolution: --</div>
                    <div id="meta_bitrate_{namespace}">Bitrate: --</div>
                    <div id="meta_codec_{namespace}">Codec: --</div>
                    <div id="meta_latency_{namespace}">Latency: --</div>
                    <div id="meta_jitter_{namespace}">Jitter: --</div>
                    <div id="meta_frames_{namespace}">Frames: --</div>
                    <div id="meta_dropped_{namespace}">Dropped: --</div>
                </div>
                
                <!-- Connection status indicator - top right -->
                <div id="connectionStatus_{namespace}" style="
                    position: absolute;
                    top: 80px;
                    right: 20px;
                    background: rgba(0, 0, 0, 0.75);
                    color: #fff;
                    font-family: sans-serif;
                    font-size: 14px;
                    padding: 10px 15px;
                    border-radius: 20px;
                    z-index: 50;
                ">
                    ⏳ Connecting...
                </div>
                
                <!-- Data channel status - top left below header -->
                <div id="dataChannelStatus_{namespace}" style="
                    position: absolute;
                    top: 80px;
                    left: 20px;
                    background: rgba(0, 0, 0, 0.75);
                    color: #888;
                    font-family: sans-serif;
                    font-size: 12px;
                    padding: 8px 12px;
                    border-radius: 15px;
                    z-index: 50;
                ">
                    📡 Telemetry: waiting...
                </div>
                
                <!-- Recording status overlay - top center right -->
                <div id="recordingStatus_{namespace}" style="
                    position: absolute;
                    top: 80px;
                    left: 50%;
                    margin-left: 150px;
                    background: rgba(255, 0, 0, 0.85);
                    color: #fff;
                    font-family: 'Courier New', monospace;
                    font-size: 14px;
                    font-weight: bold;
                    padding: 8px 15px;
                    border-radius: 8px;
                    z-index: 50;
                    display: none;
                ">
                    <span id="recordingIndicator_{namespace}">🔴 REC</span>
                    <span id="recordingTime_{namespace}">00:00:00</span>
                    <span id="recordingFrames_{namespace}">(0 frames)</span>
                </div>
            </div>
            '''


def _fullscreen_video_script(namespace, ws_url):
    return f'''
        (async function connectFullscreenStream() {{
            const wsUrl = "{ws_url}";
            const namespace = "{namespace}";
            const remoteVideo = document.getElementById("fullscreenVideo_{namespace}");
            const connectionStatus = document.getElementById("connectionStatus_{namespace}");
            const dataChannelStatus = document.getElementById("dataChannelStatus_{namespace}");
            
            // Telemetry elements
            const telemGps = document.getElementById("telem_gps_{namespace}");
            const telemAltitude = document.getElementById("telem_altitude_{namespace}");
            const telemSats = document.getElementById("telem_sats_{namespace}");
            const telemGimbal = document.getElementById("telem_gimbal_{namespace}");
            const telemAttitude = document.getElementById("telem_attitude_{namespace}");
            const telemHeading = document.getElementById("telem_heading_{namespace}");
            const telemVelocity = document.getElementById("telem_velocity_{namespace}");
            const telemBattery = document.getElementById("telem_battery_{namespace}");
            const telemFrame = document.getElementById("telem_frame_{namespace}");
            
            // Stream stats elements
            const metaResolution = document.getElementById("meta_resolution_{namespace}");
            const metaBitrate = document.getElementById("meta_bitrate_{namespace}");
            const metaCodec = document.getElementById("meta_codec_{namespace}");
            const metaLatency = document.getElementById("meta_latency_{namespace}");
            const metaJitter = document.getElementById("meta_jitter_{namespace}");
            const metaFrames = document.getElementById("meta_frames_{namespace}");
            const metaDropped = document.getElementById("meta_dropped_{namespace}");
            
            let lastBytesReceived = 0;
            let lastTimestamp = Date.now();
            let statsInterval = null;
            let telemetryChannel = null;
            
            function setConnectionStatus(text, color) {{
                if (connectionStatus) {{
                    connectionStatus.textContent = text;
                    connectionStatus.style.background = color;
                }}
            }}
            
            function setDataChannelStatus(text, color) {{
                if (dataChannelStatus) {{
                    dataChannelStatus.textContent = "📡 Telemetry: " + text;
                    dataChannelStatus.style.color = color;
                }}
            }}
            
            function updateTelemetry(meta) {{
                if (!meta) return;
                
                // GPS
                const lat = meta.latitude || 0;
                const lon = meta.longitude || 0;
                if (telemGps) {{
                    if (lat !== 0 || lon !== 0) {{
                        telemGps.textContent = "📍 GPS: " + lat.toFixed(6) + ", " + lon.toFixed(6);
                    }} else {{
                        telemGps.textContent = "📍 GPS: No fix";
                    }}
                }}
                
                // Altitude
                const altASL = meta.altitudeASL || 0;
                const altAGL = meta.altitudeAGL || 0;
                if (telemAltitude) {{
                    telemAltitude.textContent = "🏔️ Alt: ASL " + altASL.toFixed(1) + "m AGL " + altAGL.toFixed(1) + "m";
                }}
                
                // Satellites
                const sats = meta.satelliteCount || 0;
                if (telemSats) {{
                    let satsColor = sats > 10 ? "#00ff00" : sats > 5 ? "#ffa500" : "#ff0000";
                    telemSats.innerHTML = '🛰️ Satellites: <span style="color:' + satsColor + '">' + sats + '</span>';
                }}
                
                // Gimbal
                const gimbalPitch = meta.gimbalPitch || 0;
                const gimbalYaw = meta.gimbalYaw || 0;
                const gimbalRoll = meta.gimbalRoll || 0;
                if (telemGimbal) {{
                    telemGimbal.textContent = "🎥 Gimbal P:" + gimbalPitch.toFixed(1) + "° Y:" + gimbalYaw.toFixed(1) + "° R:" + gimbalRoll.toFixed(1) + "°";
                }}
                
                // Attitude
                const pitch = meta.aircraftPitch || 0;
                const yaw = meta.aircraftYaw || 0;
                const roll = meta.aircraftRoll || 0;
                if (telemAttitude) {{
                    telemAttitude.textContent = "✈️ Attitude P:" + pitch.toFixed(1) + "° Y:" + yaw.toFixed(1) + "° R:" + roll.toFixed(1) + "°";
                }}
                
                // Heading
                if (telemHeading) {{
                    telemHeading.textContent = "🧭 Heading: " + yaw.toFixed(1) + "°";
                }}
                
                // Velocity
                const vx = meta.velocityX || 0;
                const vy = meta.velocityY || 0;
                const vz = meta.velocityZ || 0;
                const speed = Math.sqrt(vx*vx + vy*vy + vz*vz);
                if (telemVelocity) {{
                    telemVelocity.textContent = "💨 Speed: " + speed.toFixed(1) + " m/s";
                }}
                
                // Battery with color coding
                const battery = meta.batteryPercent || 0;
                if (telemBattery) {{
                    let batteryColor = battery > 30 ? "#00ff00" : battery > 15 ? "#ffa500" : "#ff0000";
                    telemBattery.innerHTML = '🔋 Battery: <span style="color:' + batteryColor + '">' + battery + '%</span>';
                }}
                
                // Frame number
                const frameNum = meta.frameNumber || "N/A";
                if (telemFrame) {{
                    telemFrame.textContent = "📹 Frame: " + frameNum;
                }}
            }}
            
            function formatBitrate(bps) {{
                if (bps < 1000) return bps.toFixed(0) + " bps";
                if (bps < 1000000) return (bps / 1000).toFixed(1) + " Kbps";
                return (bps / 1000000).toFixed(2) + " Mbps";
            }}
            
            async function updateStats(pc) {{
                if (!pc) return;
                
                try {{
                    const stats = await pc.getStats();
                    
                    stats.forEach(report => {{
                        if (report.type === "inbound-rtp" && report.kind === "video") {{
                            // Calculate bitrate
                            const now = Date.now();
                            const bytesReceived = report.bytesReceived || 0;
                            const timeDiff = (now - lastTimestamp) / 1000;
                            const bytesDiff = bytesReceived - lastBytesReceived;
                            const bitrate = timeDiff > 0 ? (bytesDiff * 8) / timeDiff : 0;
                            
                            lastBytesReceived = bytesReceived;
                            lastTimestamp = now;
                            
                            if (metaBitrate) metaBitrate.textContent = "Bitrate: " + formatBitrate(bitrate);
                            if (metaJitter) metaJitter.textContent = "Jitter: " + ((report.jitter || 0) * 1000).toFixed(2) + " ms";
                            if (metaFrames) metaFrames.textContent = "Frames: " + (report.framesDecoded || 0).toLocaleString();
                            if (metaDropped) metaDropped.textContent = "Dropped: " + (report.framesDropped || 0);
                            
                            // Get codec info
                            if (report.codecId) {{
                                const codecReport = stats.get(report.codecId);
                                if (codecReport && metaCodec) {{
                                    metaCodec.textContent = "Codec: " + (codecReport.mimeType || "unknown").replace("video/", "");
                                }}
                            }}
                        }}
                        
                        if (report.type === "candidate-pair" && report.state === "succeeded") {{
                            if (metaLatency) {{
                                metaLatency.textContent = "Latency: " + (report.currentRoundTripTime ? (report.currentRoundTripTime * 1000).toFixed(1) + " ms" : "--");
                            }}
                        }}
                    }});
                    
                    // Update video resolution from video element
                    if (remoteVideo && remoteVideo.videoWidth > 0) {{
                        if (metaResolution) metaResolution.textContent = "Resolution: " + remoteVideo.videoWidth + "x" + remoteVideo.videoHeight;
                    }}
                    
                }} catch (e) {{
                    console.error("Error getting stats:", e);
                }}
            }}
            
            function addDebug(msg) {{
                console.log("[WebRTC Fullscreen " + namespace + "] " + msg);
            }}
            
            function setupTelemetryChannel(channel) {{
                addDebug("Setting up telemetry channel: " + channel.label);
                telemetryChannel = channel;
                
                channel.onopen = () => {{
                    addDebug("Telemetry channel opened");
                    setDataChannelStatus("connected", "#00ff00");
                }};
                
                channel.onmessage = (event) => {{
                    try {{
                        if (!event.data) return;
                        const meta = JSON.parse(event.data);
                        updateTelemetry(meta);
                        
                        // Store latest telemetry globally for recording
                        window.latestTelemetry_{namespace} = meta;
                        
                        addDebug("Telemetry received: frame " + (meta.frameNumber || "N/A"));
                    }} catch (e) {{
                        addDebug("Telemetry parse error: " + e.message);
                    }}
                }};
                
                channel.onclose = () => {{
                    addDebug("Telemetry channel closed");
                    setDataChannelStatus("closed", "#888");
                }};
                
                channel.onerror = (error) => {{
                    addDebug("Telemetry channel error: " + error);
                    setDataChannelStatus("error", "#ff0000");
                }};
                
                // If channel is already open
                if (channel.readyState === "open") {{
                    setDataChannelStatus("connected", "#00ff00");
                }}
            }}
            
            if (!remoteVideo) {{
                addDebug("Video element not found");
                setConnectionStatus("❌ Video element not found", "rgba(239, 68, 68, 0.9)");
                return;
            }}
            
            setConnectionStatus("⏳ Connecting...", "rgba(245, 158, 11, 0.9)");
            addDebug("Starting fullscreen connection to " + wsUrl);
            
            const ws = new WebSocket(wsUrl);
            let pc = null;
            
            ws.onopen = async () => {{
                addDebug("WebSocket connected");
                setConnectionStatus("🔗 WebSocket connected", "rgba(59, 130, 246, 0.9)");
                
                const config = {{
                    iceServers: [{{ urls: "stun:stun.l.google.com:19302" }}]
                }};
                
                pc = new RTCPeerConnection(config);
                addDebug("RTCPeerConnection created");
                
                // Create telemetry data channel (negotiated mode to match Android side)
                try {{
                    telemetryChannel = pc.createDataChannel("telemetry", {{
                        negotiated: true,
                        id: 0,
                        ordered: true
                    }});
                    addDebug("Created telemetry data channel (negotiated mode)");
                    setupTelemetryChannel(telemetryChannel);
                }} catch (e) {{
                    addDebug("Error creating telemetry channel: " + e.message);
                }}
                
                // Also handle incoming data channels (if Android creates it differently)
                pc.ondatachannel = (event) => {{
                    addDebug("Received data channel: " + event.channel.label);
                    if (event.channel.label === "telemetry") {{
                        setupTelemetryChannel(event.channel);
                    }}
                }};
                
                pc.onicecandidate = (event) => {{
                    if (event.candidate && ws && ws.readyState === WebSocket.OPEN) {{
                        ws.send(JSON.stringify(event.candidate));
                    }}
                }};
                
                pc.ontrack = (event) => {{
                    addDebug("Track received: " + event.track.kind);
                    
                    if (event.streams && event.streams[0]) {{
                        remoteVideo.srcObject = event.streams[0];
                        
                        // Update frame rate when metadata is loaded
                        remoteVideo.onloadedmetadata = () => {{
                            addDebug("Video metadata loaded: " + remoteVideo.videoWidth + "x" + remoteVideo.videoHeight);
                            if (metaResolution) metaResolution.textContent = "Resolution: " + remoteVideo.videoWidth + "x" + remoteVideo.videoHeight;
                        }};
                        
                        remoteVideo.play().then(() => {{
                            addDebug("Video playback started");
                            setConnectionStatus("✅ Stream playing", "rgba(34, 197, 94, 0.9)");
                            
                            // Start stats monitoring
                            if (statsInterval) clearInterval(statsInterval);
                            statsInterval = setInterval(() => updateStats(pc), 1000);
                        }}).catch(e => {{
                            remoteVideo.muted = true;
                            remoteVideo.play().then(() => {{
                                setConnectionStatus("✅ Stream playing (muted)", "rgba(34, 197, 94, 0.9)");
                                if (statsInterval) clearInterval(statsInterval);
                                statsInterval = setInterval(() => updateStats(pc), 1000);
                            }}).catch(e2 => {{
                                addDebug("Play error: " + e2.message);
                                setConnectionStatus("❌ Play error", "rgba(239, 68, 68, 0.9)");
                            }});
                        }});
                    }} else if (event.track) {{
                        let stream = remoteVideo.srcObject;
                        if (!stream) {{
                            stream = new MediaStream();
                            remoteVideo.srcObject = stream;
                        }}
                        stream.addTrack(event.track);
                        remoteVideo.play().catch(e => addDebug("Play error: " + e.message));
                    }}
                }};
                
                pc.onconnectionstatechange = () => {{
                    addDebug("Connection state: " + pc.connectionState);
                    
                    if (pc.connectionState === "connected") {{
                        setConnectionStatus("✅ Connected", "rgba(34, 197, 94, 0.9)");
                    }} else if (pc.connectionState === "disconnected" || pc.connectionState === "failed") {{
                        setConnectionStatus("❌ " + pc.connectionState, "rgba(239, 68, 68, 0.9)");
                        if (statsInterval) clearInterval(statsInterval);
                    }}
                }};
                
                window.fullscreenWebRTC = {{ pc: pc, ws: ws, statsInterval: statsInterval, telemetryChannel: telemetryChannel }};
            }};
            
            ws.onmessage = async (event) => {{
                const message = JSON.parse(event.data);
                addDebug("Received: " + message.type);
                
                if (message.type === "offer") {{
                    await pc.setRemoteDescription(new RTCSessionDescription(message));
                    const answer = await pc.createAnswer();
                    await pc.setLocalDescription(answer);
                    ws.send(JSON.stringify(pc.localDescription));
                    addDebug("Sent answer");
                }} else if (message.type === "answer") {{
                    await pc.setRemoteDescription(new RTCSessionDescription(message));
                }} else if (message.candidate !== undefined) {{
                    if (message.candidate === null || message.candidate === "") {{
                        try {{ await pc.addIceCandidate(null); }} catch (e) {{}}
                    }} else {{
                        try {{
                            const candidateInit = {{
                                candidate: message.candidate,
                                sdpMid: message.sdpMid !== undefined ? message.sdpMid : "0",
                                sdpMLineIndex: message.sdpMLineIndex !== undefined ? message.sdpMLineIndex : 0
                            }};
                            await pc.addIceCandidate(new RTCIceCandidate(candidateInit));
                        }} catch (e) {{
                            addDebug("ICE error: " + e.message);
                        }}
                    }}
                }}
            }};
            
            ws.onerror = (error) => {{
                addDebug("WebSocket error: " + error);
                setConnectionStatus("❌ WebSocket error", "rgba(239, 68, 68, 0.9)");
            }};
            
            ws.onclose = () => {{
                addDebug("WebSocket closed");
                setConnectionStatus("⚫ Disconnected", "rgba(107, 114, 128, 0.9)");
                if (statsInterval) clearInterval(statsInterval);
            }};
            
            // Cleanup on page unload
            window.addEventListener("beforeunload", () => {{
                if (statsInterval) clearInterval(statsInterval);
                if (window.fullscreenWebRTC) {{
                    if (window.fullscreenWebRTC.ws) window.fullscreenWebRTC.ws.close();
                    if (window.fullscreenWebRTC.pc) window.fullscreenWebRTC.pc.close();
                }}
            }});
        }})();
        '''

class PerpetualMonitorGUI(PerpetualMonitorNode):
    """
    Extended ROS2 Node with NiceGUI Event-based UI updates.
    
    Uses NiceGUI Events to safely communicate between ROS callbacks and UI thread.
    This follows the recommended NiceGUI + ROS2 integration pattern.
    """
    
    _instance = None
    
    @staticmethod
    def get_instance():
        if PerpetualMonitorGUI._instance is None:
            PerpetualMonitorGUI._instance = PerpetualMonitorGUI()
        return PerpetualMonitorGUI._instance
    
    def __init__(self):
        super().__init__()
        
        # Default drone IPs to auto-connect at startup
        self.AUTO_CONNECT_DRONE_IPS = [
            ('10.142.188.57', 'drone_1'),
            ('10.142.188.117', 'drone_2'),
        ]
        
        # NiceGUI Events for thread-safe UI updates
        self.drone_position_update = Event()
        self.drone_heading_update = Event()
        self.drone_battery_update = Event()
        self.drone_flight_time_update = Event()
        self.drone_state_update = Event()
        self.drone_recording_update = Event()
        self.drone_satellite_update = Event()
        self.drone_speed_update = Event()
        self.drone_flight_mode_update = Event()  # Flight mode (virtual_stick, manual, etc.)
        self.drone_connected_event = Event()
        self.drone_disconnected_event = Event()
        self.monitoring_point_update = Event()
        self.relay_countdown_update = Event()
        self.rth_predictor_update = Event()  # RTH predictor info
        self.log_event = Event()
        
        # UI element references (populated when page loads)
        self.map = None
        self.drone_cards: Dict[str, ui.card] = {}
        self.drone_arrows: Dict[str, Arrow] = {}
        self.drone_labels: Dict[str, Dict[str, ui.label]] = {}
        self.drone_buttons: Dict[str, Dict[str, ui.button]] = {}
        self.drone_list_container = None
        
        # Event log message queue (for thread-safe logging)
        self.log_message_queue: list = []
        self._should_start_timer = False  # Flag for starting mission timer from UI thread
        
        # Vertical separation alert queue (for thread-safe UI updates)
        self._vertical_separation_alerts: list = []
        
        # Notification queue (for thread-safe UI notifications from background threads)
        self._notification_queue: list = []
        
        # Takeoff confirmation queue (for thread-safe dialog from background threads)
        self._takeoff_confirmation_queue: list = []
        self._takeoff_confirmation_dialog_open = False
        
        # Monitoring point marker
        self.monitoring_marker = None
        self.monitoring_circle = None
        
        # Mission display elements
        self.mission_status_label = None
        self.countdown_label = None
        self.countdown_progress = None
        self.countdown_container = None  # Container for countdown (hidden during manual swap)
        self.force_swap_button = None  # Button to trigger manual swap
        self.active_drone_label = None
        self.next_drone_label = None
        self.drones_needed_label = None  # Legacy, kept for compatibility
        self.drones_needed_flying_label = None
        self.drones_needed_total_label = None
        self.drones_needed_info_label = None
        self.drones_needed_status_icon = None
        self.drones_needed_ready_label = None
        self.relay_alert_label = None
        self.relay_alert_icon = None
        self.relay_alert_container = None
        self.reconnect_label = None
        self.mission_timer_label = None
        self._mission_start_time = None
        
        # Segmented progress bar elements
        self.progress_segment_1 = None  # 0-1 min (LAUNCH - critical)
        self.progress_segment_2 = None  # 1-3 min (CONNECT - urgent)
        self.progress_segment_3 = None  # 3-5 min (READY - warning)
        self.progress_segment_4 = None  # 5+ min (PREPARE - normal)
        self._mission_timer_task = None
        self._manual_swap_active = False  # True when manual swap in progress
        
        # ROS bag recording
        self._rosbag_process = None  # subprocess.Popen for ros2 bag record
        self._rosbag_recording = False
        self._rosbag_dir = "/WildPerpetua/src/rosbags"  # Mounted to host's src/rosbags/
        
        # Debug console (ROS logs)
        self.debug_mode = False
        self.debug_toggle = None
        self.debug_console = None
        self.debug_console_container = None
        self.debug_scroll = None
        self.normal_logs_container = None
        self.mission_stats_card = None  # Card to hide when debug console is shown
        
        # Silent mode (mute all sounds)
        self.silent_mode = False
        self.silent_toggle = None
        
        # Event log
        self.event_log = None
        
        # Track which drones we've centered on (to center on first position)
        self._centered_on_drone: set = set()
        self.event_scroll = None
        
        # Mission statistics tracking
        self.mission_stats_container = None
        self.mission_stats_scroll = None
        self.mission_stats_history: list = []  # List of {drone, iteration, est_travel, actual_travel, actual_rth}
        self.drone_iteration_counter: Dict[str, int] = {}  # Track iteration per drone
        
        # RTH landing detection
        self.drone_rth_tracking: Dict[str, dict] = {}  # {ns: {start_time, last_alt, stable_count, detected}}
        
        # Battery swap detection (track last battery level per drone)
        self.drone_last_battery: Dict[str, float] = {}
        
        # Trajectory tracking for flight paths
        self.drone_trajectories: Dict[str, list] = {}  # {ns: [(lat, lon), ...]}
        self.drone_trajectory_lines: Dict[str, object] = {}  # {ns: leaflet polyline layer}
        self.drone_is_flying: Dict[str, bool] = {}  # {ns: True if currently flying}
        
        # State machine display elements
        self.state_machine_container = None
        self.state_machine_labels: Dict[str, Dict[str, ui.label]] = {}
        
        # Video feed visibility state tracking
        self.drone_video_visible: Dict[str, bool] = {}  # {drone_ns: is_visible}
        
        # YOLO detection state
        self.yolo_model = None  # Ultralytics YOLO model instance
        self.yolo_model_path = None  # Path to loaded model
        self.yolo_running: Dict[str, bool] = {}  # {namespace: is_running}
        self.yolo_detections: Dict[str, list] = {}  # {namespace: [detections]}
        
        # Video/Telemetry recording state
        self.recording_running: Dict[str, bool] = {}  # {namespace: is_recording}
        self.recording_dir = "/WildPerpetua/recordings"  # Default recording directory
        self.recording_sessions: Dict[str, dict] = {}  # {namespace: session_data}
        self.recording_timers: Dict[str, object] = {}  # {namespace: ui.timer}
        self.latest_telemetry: Dict[str, dict] = {}  # {namespace: telemetry_data} - latest telemetry for recording
        
        # Create default recording directory
        import os
        os.makedirs(self.recording_dir, exist_ok=True)
        
        # UI update throttling - reduces browser lag by limiting update frequency
        # Stores last emit time per namespace per event type
        self._last_emit_time: Dict[str, Dict[str, float]] = {}  # {event_type: {namespace: timestamp}}
        self._throttle_intervals = {
            'position': 0.1,      # 10 Hz max (map arrow updates)
            'heading': 0.1,       # 10 Hz max (arrow rotation)
            'speed': 0.5,         # 2 Hz max (not critical)
            'satellites': 1.0,    # 1 Hz max (slow changing)
            'flight_time': 1.0,   # 1 Hz max (countdown display)
            'flight_mode': 1.0,   # 1 Hz max (rarely changes)
        }
        
        # Connection form elements
        self.ip_input = None
        self.namespace_input = None
        self.lat_input = None
        self.lon_input = None
        self.alt_input = None
        self.heading_input = None
        self.rth_alt_input = None
        self.safety_buffer_input = None
        self.min_battery_input = None
        self.min_satellites_input = None
        self.camera_sync_switch = None
        self.nav_mode_switch = None
        self.nav_mode_label = None
        self.trajectory_mode = None
        self.trajectory_speed_slider = None
        self.trajectory_speed_label = None

        # Rotation UI elements
        self.rotation_order_label = None
        self.rotation_select = None
        self.rotation_next_label = None
        
        # Map settings
        self.map_center = (14.475781, -90.881235)  # Default center
        
        # Drone colors for visualization
        self.drone_colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD', '#98D8C8', '#F7DC6F']
        
        # Pending takeoff drone name (for dialog re-show on cancel->cancel)
        self._pending_takeoff_drone = ""
        self._pending_single_mission = False
        self._pending_relay_mission = False
        self._pending_relay_data = {}
        
        # Vertical separation countdown state
        self._vertical_separation_countdown_active = False
        
        # Register takeoff confirmation callback
        self.mission_controller.on_takeoff_confirmation_request = self._on_takeoff_confirmation_request
        
        # Register vertical separation alert callback
        self.mission_controller.on_vertical_separation_alert = self._on_vertical_separation_alert
        
        # Register vertical separation countdown callbacks
        self.mission_controller.on_vertical_separation_countdown_start = self._on_vertical_separation_countdown_start
        self.mission_controller.on_vertical_separation_countdown_cancel = self._on_vertical_separation_countdown_cancel
        self.mission_controller.on_vertical_separation_mission_stopped = self._on_vertical_separation_mission_stopped
        
        # Register drone flight mode callback for manual flight detection
        self.mission_controller.get_drone_flight_mode = self._get_drone_flight_mode
        
        # Register drone altitude callback for separation check  
        self.mission_controller.get_drone_altitude = self._get_drone_altitude
        
        # Auto-connect drones at startup (in background thread)
        self._auto_connect_on_startup()
        
        # Define the main page
        @ui.page('/')
        def main_page():
            self._build_ui()
        
        # Define fullscreen video page for each drone
        @ui.page('/video/{namespace}')
        def video_page(namespace: str):
            self._build_fullscreen_video_page(namespace)
    
    def _build_fullscreen_video_page(self, namespace: str):
        """Build a fullscreen video page for a drone."""
        # Check if drone exists
        if namespace not in self.drones:
            ui.label(f'Drone {namespace} not found').classes('text-2xl text-red-500')
            return
        
        drone = self.drones[namespace]
        if not drone.ip_address:
            ui.label(f'No IP address for {namespace}').classes('text-2xl text-red-500')
            return
        
        ws_url = f"ws://{drone.ip_address}:8082"
        
        # Dark background fullscreen layout
        ui.query('body').style('margin: 0; padding: 0; background: #000; overflow: hidden;')
        
        with ui.column().classes('w-screen h-screen items-center justify-center').style('background: #000; position: relative;'):
            # Header with drone name and back button
            with ui.row().classes('absolute top-0 left-0 right-0 p-4 items-center justify-between').style('background: rgba(0,0,0,0.7); z-index: 100;'):
                ui.label(f'📹 {namespace} - Live Video Feed').classes('text-white text-xl font-bold')
                with ui.row().classes('gap-2 items-center'):
                    # Recording controls
                    rec_status = ui.label('⚫ Not Recording').classes('text-white text-sm mr-4')
                    rec_status_ref = rec_status
                    rec_btn = ui.button('Start Recording', icon='fiber_manual_record').props('flat color=white size=sm')
                    rec_stop_btn = ui.button('Stop Recording', icon='stop').props('flat color=red size=sm').classes('hidden')
                    
                    ui.label('|').classes('text-gray-500 mx-2')
                    
                    # YOLO Detection controls
                    yolo_status = ui.label('🔴 YOLO Off').classes('text-white text-sm mr-4')
                    yolo_status_ref = yolo_status
                    yolo_btn = ui.button('Load YOLO', icon='psychology').props('flat color=white size=sm')
                    yolo_stop_btn = ui.button('Stop YOLO', icon='stop').props('flat color=white size=sm').classes('hidden')
                    ui.button('Back to Groundstation', icon='arrow_back', on_click=lambda: ui.navigate.to('/')).props('flat color=white')
            
            # Fullscreen video element with metadata overlay and detection canvas
            video_html = _fullscreen_video_html(namespace)
            ui.html(video_html, sanitize=False)
        
        # Initialize YOLO state for this namespace
        self.yolo_running[namespace] = False
        self.yolo_detections[namespace] = []
        
        # Server-side YOLO detection handler
        async def load_yolo_model(e):
            """Handle YOLO model file upload and load with ultralytics."""
            try:
                # NiceGUI UploadEventArguments has 'file' attribute with FileUpload object
                # FileUpload has: name, read(), text(), json(), save(), size()
                # Note: read() is async in NiceGUI
                filename = e.file.name
                content = await e.file.read()
                
                # Save to temp file
                import tempfile
                import os
                temp_dir = tempfile.gettempdir()
                model_path = os.path.join(temp_dir, filename)
                
                with open(model_path, 'wb') as f:
                    f.write(content)
                
                self.get_logger().info(f"[YOLO] Saved model to {model_path}")
                
                # Load model with ultralytics
                try:
                    from ultralytics import YOLO
                    self.yolo_model = YOLO(model_path)
                    self.yolo_model_path = model_path
                    self.yolo_running[namespace] = True
                    
                    # Update UI
                    yolo_status_ref.text = '🟢 YOLO On'
                    yolo_status_ref.style('color: #4ade80')
                    yolo_btn.classes(remove='', add='hidden')
                    yolo_stop_btn.classes(remove='hidden', add='')
                    
                    # Show YOLO stats
                    ui.run_javascript(f'''
                        document.getElementById("yoloStats_{namespace}").style.display = "block";
                    ''')
                    
                    ui.notify(f'YOLO model loaded: {filename}', type='positive')
                    self.get_logger().info(f"[YOLO] Model loaded successfully: {filename}")
                    
                    # Start detection loop
                    start_detection_loop()
                    
                except ImportError:
                    ui.notify('ultralytics not installed. Run: pip install ultralytics', type='negative')
                    self.get_logger().error("[YOLO] ultralytics package not installed")
                except Exception as ex:
                    ui.notify(f'Failed to load model: {str(ex)}', type='negative')
                    self.get_logger().error(f"[YOLO] Failed to load model: {ex}")
                    
            except Exception as ex:
                ui.notify(f'Error uploading file: {str(ex)}', type='negative')
                self.get_logger().error(f"[YOLO] Upload error: {ex}")
        
        def stop_yolo():
            """Stop YOLO detection."""
            self.yolo_running[namespace] = False
            self.yolo_detections[namespace] = []
            
            # Cancel the detection timer if it exists
            if hasattr(self, 'yolo_timer') and self.yolo_timer:
                self.yolo_timer.cancel()
                self.yolo_timer = None
            
            yolo_status_ref.text = '🔴 YOLO Off'
            yolo_status_ref.style('color: white')
            yolo_btn.classes(remove='hidden', add='')
            yolo_stop_btn.classes(remove='', add='hidden')
            
            # Hide YOLO stats and clear canvas
            ui.run_javascript(f'''
                document.getElementById("yoloStats_{namespace}").style.display = "none";
                const canvas = document.getElementById("detectionCanvas_{namespace}");
                if (canvas) {{
                    const ctx = canvas.getContext('2d');
                    ctx.clearRect(0, 0, canvas.width, canvas.height);
                }}
            ''')
            
            ui.notify('YOLO detection stopped', type='info')
            self.get_logger().info(f"[YOLO] Detection stopped for {namespace}")
        
        def start_detection_loop():
            """Start the server-side YOLO detection loop using ui.timer for proper context."""
            import base64
            import json
            
            async def run_detection():
                """Single detection iteration - called by ui.timer."""
                if not self.yolo_running.get(namespace, False) or self.yolo_model is None:
                    return
                
                try:
                    # Request frame capture from browser
                    frame_data = await ui.run_javascript(f'''
                        (function() {{
                            const video = document.getElementById("fullscreenVideo_{namespace}");
                            if (!video || video.readyState < 2) return null;
                            
                            const canvas = document.createElement('canvas');
                            canvas.width = video.videoWidth;
                            canvas.height = video.videoHeight;
                            const ctx = canvas.getContext('2d');
                            ctx.drawImage(video, 0, 0);
                            
                            // Return as base64 JPEG (smaller than PNG)
                            return canvas.toDataURL('image/jpeg', 0.8);
                        }})();
                    ''', timeout=2.0)
                    
                    if frame_data and frame_data.startswith('data:image'):
                        # Decode base64 image
                        import cv2
                        import numpy as np
                        import time
                        
                        # Remove data URL prefix
                        img_data = frame_data.split(',')[1]
                        img_bytes = base64.b64decode(img_data)
                        img_array = np.frombuffer(img_bytes, dtype=np.uint8)
                        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                        
                        if frame is not None:
                            # Run YOLO inference
                            start_time = time.time()
                            
                            results = self.yolo_model(frame, verbose=False, conf=0.41)
                            
                            inference_time = (time.time() - start_time) * 1000
                            
                            # Extract detections
                            detections = []
                            if results and len(results) > 0:
                                result = results[0]
                                if result.boxes is not None:
                                    boxes = result.boxes
                                    for i in range(len(boxes)):
                                        box = boxes[i]
                                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                                        conf = float(box.conf[0])
                                        cls_id = int(box.cls[0])
                                        cls_name = result.names[cls_id] if cls_id in result.names else f'class_{cls_id}'
                                        
                                        detections.append({
                                            'box': [x1, y1, x2, y2],
                                            'score': conf,
                                            'classId': cls_id,
                                            'className': cls_name
                                        })
                            
                            self.yolo_detections[namespace] = detections
                            
                            # Send detections to browser for rendering
                            detections_json = json.dumps(detections)
                            
                            ui.run_javascript(f'''
                                (function() {{
                                    const detections = {detections_json};
                                    const video = document.getElementById("fullscreenVideo_{namespace}");
                                    const canvas = document.getElementById("detectionCanvas_{namespace}");
                                    const inferenceTimeEl = document.getElementById("yoloInferenceTime_{namespace}");
                                    const detectionsEl = document.getElementById("yoloDetections_{namespace}");
                                    
                                    if (!video || !canvas) return;
                                    
                                    // Color palette
                                    const colors = [
                                        '#FF3838', '#FF9D97', '#FF701F', '#FFB21D', '#CFD231', '#48F90A', '#92CC17', '#3DDB86',
                                        '#1A9334', '#00D4BB', '#2C99A8', '#00C2FF', '#344593', '#6473FF', '#0018EC', '#8438FF'
                                    ];
                                    
                                    const ctx = canvas.getContext('2d');
                                    const rect = video.getBoundingClientRect();
                                    canvas.width = rect.width;
                                    canvas.height = rect.height;
                                    
                                    // Calculate video scaling
                                    const videoAspect = video.videoWidth / video.videoHeight;
                                    const containerAspect = rect.width / rect.height;
                                    
                                    let drawWidth, drawHeight, offsetX, offsetY;
                                    if (videoAspect > containerAspect) {{
                                        drawWidth = rect.width;
                                        drawHeight = rect.width / videoAspect;
                                        offsetX = 0;
                                        offsetY = (rect.height - drawHeight) / 2;
                                    }} else {{
                                        drawHeight = rect.height;
                                        drawWidth = rect.height * videoAspect;
                                        offsetX = (rect.width - drawWidth) / 2;
                                        offsetY = 0;
                                    }}
                                    
                                    const scaleX = drawWidth / video.videoWidth;
                                    const scaleY = drawHeight / video.videoHeight;
                                    
                                    ctx.clearRect(0, 0, canvas.width, canvas.height);
                                        
                                        for (const det of detections) {{
                                            const [x1, y1, x2, y2] = det.box;
                                            const sx1 = x1 * scaleX + offsetX;
                                            const sy1 = y1 * scaleY + offsetY;
                                            const sx2 = x2 * scaleX + offsetX;
                                            const sy2 = y2 * scaleY + offsetY;
                                            const w = sx2 - sx1;
                                            const h = sy2 - sy1;
                                            
                                            const color = colors[det.classId % colors.length];
                                            
                                            // Draw box
                                            ctx.strokeStyle = color;
                                            ctx.lineWidth = 3;
                                            ctx.strokeRect(sx1, sy1, w, h);
                                            
                                            // Draw label background
                                            const label = det.className + ' ' + (det.score * 100).toFixed(0) + '%';
                                            ctx.font = 'bold 14px Arial';
                                            const textWidth = ctx.measureText(label).width;
                                            
                                            ctx.fillStyle = color;
                                            ctx.fillRect(sx1, sy1 - 22, textWidth + 10, 22);
                                            
                                            // Draw label text
                                            ctx.fillStyle = '#fff';
                                            ctx.fillText(label, sx1 + 5, sy1 - 6);
                                        }}
                                        
                                        // Update stats
                                        if (inferenceTimeEl) inferenceTimeEl.textContent = "Inference: {inference_time:.0f} ms";
                                        if (detectionsEl) detectionsEl.textContent = "Detections: " + detections.length;
                                    }})();
                                ''')
                
                except Exception as ex:
                    self.get_logger().error(f"[YOLO] Detection error: {ex}")
            
            # Use ui.timer for proper NiceGUI context (runs every 200ms = 5 FPS detection)
            # Store timer reference so we can cancel it when stopping
            self.yolo_timer = ui.timer(0.2, run_detection)
            self.get_logger().info(f"[YOLO] Detection loop started for {namespace}")
        
        # File upload dialog for YOLO model
        def show_yolo_upload_dialog():
            with ui.dialog() as dialog, ui.card().classes('p-4'):
                ui.label('Load YOLO Model').classes('text-xl font-bold mb-2')
                ui.label('Select a YOLOv8 model file (.pt)').classes('text-gray-600 mb-4')
                
                async def handle_upload(e):
                    """Async handler for YOLO model upload."""
                    await load_yolo_model(e)
                    dialog.close()
                
                upload = ui.upload(
                    label='Drop model file here or click to browse',
                    auto_upload=True,
                    on_upload=handle_upload
                ).props('accept=".pt,.onnx" color="primary"').classes('w-full')
                
                with ui.row().classes('w-full justify-end mt-4'):
                    ui.button('Cancel', on_click=dialog.close).props('flat')
            
            dialog.open()
        
        yolo_btn.on_click(show_yolo_upload_dialog)
        yolo_stop_btn.on_click(stop_yolo)
        
        # ====================================================================
        # VIDEO/TELEMETRY RECORDING
        # ====================================================================
        
        # Initialize recording state for this namespace
        self.recording_running[namespace] = False
        self.latest_telemetry[namespace] = {}
        
        def show_recording_dialog():
            """Show dialog to configure and start recording."""
            with ui.dialog() as dialog, ui.card().classes('p-4 w-96'):
                ui.label('📹 Start Recording').classes('text-xl font-bold mb-2')
                ui.label('Configure recording settings').classes('text-gray-600 mb-4')
                
                # Storage path input
                ui.label('Storage Directory:').classes('text-sm font-medium')
                path_input = ui.input(
                    value=self.recording_dir,
                    placeholder='/path/to/recordings'
                ).classes('w-full mb-4')
                
                # Info about what will be recorded
                with ui.column().classes('bg-gray-100 p-3 rounded mb-4'):
                    ui.label('Will record:').classes('font-medium text-sm')
                    ui.label('• Raw video (MP4 @ 30 FPS)').classes('text-xs text-gray-600')
                    ui.label('• Telemetry (CSV synced with frames)').classes('text-xs text-gray-600')
                    if self.yolo_running.get(namespace, False):
                        ui.label('• Annotated video with YOLO boxes').classes('text-xs text-green-600')
                        ui.label('• Detections (JSON + CSV)').classes('text-xs text-green-600')
                    else:
                        ui.label('• YOLO not active - no annotations').classes('text-xs text-gray-400')
                
                async def start_recording_click():
                    """Start the recording session."""
                    self.recording_dir = path_input.value
                    dialog.close()
                    await start_recording()
                
                with ui.row().classes('w-full justify-end gap-2 mt-4'):
                    ui.button('Cancel', on_click=dialog.close).props('flat')
                    ui.button('Start Recording', on_click=start_recording_click).props('color=red')
            
            dialog.open()
        
        async def start_recording():
            """Initialize and start recording session."""
            import os
            import csv

            # Create output directory
            os.makedirs(self.recording_dir, exist_ok=True)
            
            # Generate timestamp for filenames
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            base_filename = f"{namespace}_{timestamp}"
            
            # Initialize session data
            # Use AVI format with XVID codec for better FFmpeg compatibility
            session = {
                'start_time': time.time(),
                'frame_count': 0,
                'base_filename': base_filename,
                'output_dir': self.recording_dir,
                'raw_video_path': os.path.join(self.recording_dir, f"{base_filename}_raw.avi"),
                'annotated_video_path': os.path.join(self.recording_dir, f"{base_filename}_annotated.avi"),
                'telemetry_path': os.path.join(self.recording_dir, f"{base_filename}_telemetry.csv"),
                'detections_json_path': os.path.join(self.recording_dir, f"{base_filename}_detections.json"),
                'detections_csv_path': os.path.join(self.recording_dir, f"{base_filename}_detections.csv"),
                'raw_writer': None,
                'annotated_writer': None,
                'telemetry_file': None,
                'telemetry_writer': None,
                'detections_list': [],
                'video_initialized': False,
                'yolo_active_at_start': self.yolo_running.get(namespace, False)
            }
            
            # Open telemetry CSV
            session['telemetry_file'] = open(session['telemetry_path'], 'w', newline='')
            session['telemetry_writer'] = csv.writer(session['telemetry_file'])
            # Write header
            session['telemetry_writer'].writerow([
                'frame_number', 'timestamp', 'elapsed_seconds',
                'original_width', 'original_height',
                'latitude', 'longitude', 'altitude_asl', 'altitude_agl',
                'satellite_count', 'gimbal_pitch', 'gimbal_yaw', 'gimbal_roll',
                'aircraft_pitch', 'aircraft_yaw', 'aircraft_roll',
                'velocity_x', 'velocity_y', 'velocity_z', 'speed',
                'battery_percent'
            ])
            
            # If YOLO is active, prepare detections CSV
            if session['yolo_active_at_start']:
                session['detections_csv_file'] = open(session['detections_csv_path'], 'w', newline='')
                session['detections_csv_writer'] = csv.writer(session['detections_csv_file'])
                session['detections_csv_writer'].writerow([
                    'frame_number', 'timestamp', 'class_id', 'class_name',
                    'confidence', 'x1', 'y1', 'x2', 'y2'
                ])
            
            self.recording_sessions[namespace] = session
            self.recording_running[namespace] = True
            
            # Update UI
            rec_status_ref.text = '🔴 Recording'
            rec_status_ref.style('color: #ef4444')
            rec_btn.classes(remove='', add='hidden')
            rec_stop_btn.classes(remove='hidden', add='')
            
            # Show recording overlay
            ui.run_javascript(f'''
                document.getElementById("recordingStatus_{namespace}").style.display = "block";
            ''')
            
            ui.notify(f'Recording started: {base_filename}', type='positive')
            self.get_logger().info(f"[Recording] Started session: {session['raw_video_path']}")
            
            # Start recording loop
            start_recording_loop()
        
        def start_recording_loop():
            """Start the recording capture loop at 30 FPS."""
            import base64
            import cv2
            import numpy as np
            
            async def capture_frame():
                """Capture and record a single frame."""
                if not self.recording_running.get(namespace, False):
                    return
                
                session = self.recording_sessions.get(namespace)
                if not session:
                    return
                
                try:
                    # Capture frame and telemetry from browser
                    capture_data = await ui.run_javascript(f'''
                        (function() {{
                            const video = document.getElementById("fullscreenVideo_{namespace}");
                            if (!video || video.readyState < 2 || video.videoWidth === 0) return null;
                            
                            const canvas = document.createElement('canvas');
                            canvas.width = video.videoWidth;
                            canvas.height = video.videoHeight;
                            const ctx = canvas.getContext('2d');
                            ctx.drawImage(video, 0, 0);
                            
                            return {{
                                data: canvas.toDataURL('image/jpeg', 0.95),
                                width: video.videoWidth,
                                height: video.videoHeight,
                                telemetry: window.latestTelemetry_{namespace} || {{}}
                            }};
                        }})();
                    ''', timeout=1.0)
                    
                    if not capture_data or not capture_data.get('data'):
                        return
                    
                    # Store latest telemetry for this namespace
                    if capture_data.get('telemetry'):
                        self.latest_telemetry[namespace] = capture_data['telemetry']
                    
                    # Decode frame
                    img_data = capture_data['data'].split(',')[1]
                    img_bytes = base64.b64decode(img_data)
                    img_array = np.frombuffer(img_bytes, dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    if frame is None:
                        return
                    
                    # Store original frame dimensions for telemetry
                    orig_h, orig_w = frame.shape[:2]
                    
                    # Target output resolution (fixed 1920x1080)
                    TARGET_WIDTH = 1920
                    TARGET_HEIGHT = 1080
                    
                    # Resize frame to target resolution
                    frame_resized = cv2.resize(frame, (TARGET_WIDTH, TARGET_HEIGHT), interpolation=cv2.INTER_LINEAR)
                    
                    # Initialize video writers on first frame
                    if not session['video_initialized']:
                        # Use XVID codec for better compatibility
                        fourcc = cv2.VideoWriter_fourcc(*'XVID')
                        session['raw_writer'] = cv2.VideoWriter(
                            session['raw_video_path'], fourcc, 30.0, (TARGET_WIDTH, TARGET_HEIGHT)
                        )
                        if session['yolo_active_at_start']:
                            session['annotated_writer'] = cv2.VideoWriter(
                                session['annotated_video_path'], fourcc, 30.0, (TARGET_WIDTH, TARGET_HEIGHT)
                            )
                        session['video_initialized'] = True
                        self.get_logger().info(f"[Recording] Video initialized: {TARGET_WIDTH}x{TARGET_HEIGHT} (original: {orig_w}x{orig_h})")
                    
                    # Write resized raw frame
                    session['raw_writer'].write(frame_resized)
                    
                    # Get current telemetry
                    telem = self.latest_telemetry.get(namespace, {})
                    elapsed = time.time() - session['start_time']
                    frame_num = session['frame_count']
                    
                    # Write telemetry row (includes original frame dimensions)
                    vx = telem.get('velocityX', 0)
                    vy = telem.get('velocityY', 0)
                    vz = telem.get('velocityZ', 0)
                    speed = (vx**2 + vy**2 + vz**2) ** 0.5
                    
                    session['telemetry_writer'].writerow([
                        frame_num,
                        datetime.now().isoformat(),
                        f"{elapsed:.3f}",
                        orig_w,
                        orig_h,
                        telem.get('latitude', 0),
                        telem.get('longitude', 0),
                        telem.get('altitudeASL', 0),
                        telem.get('altitudeAGL', 0),
                        telem.get('satelliteCount', 0),
                        telem.get('gimbalPitch', 0),
                        telem.get('gimbalYaw', 0),
                        telem.get('gimbalRoll', 0),
                        telem.get('aircraftPitch', 0),
                        telem.get('aircraftYaw', 0),
                        telem.get('aircraftRoll', 0),
                        vx, vy, vz, f"{speed:.2f}",
                        telem.get('batteryPercent', 0)
                    ])
                    
                    # If YOLO active, create annotated frame and save detections
                    if session['yolo_active_at_start'] and session['annotated_writer']:
                        detections = self.yolo_detections.get(namespace, [])
                        annotated_frame = frame_resized.copy()
                        
                        # Scale factors for bounding boxes (original -> 1920x1080)
                        scale_x = TARGET_WIDTH / orig_w
                        scale_y = TARGET_HEIGHT / orig_h
                        
                        # Draw detections on annotated frame
                        colors = [
                            (56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255),
                            (49, 210, 207), (10, 249, 72), (23, 204, 146), (134, 219, 61),
                            (52, 147, 26), (187, 212, 0), (168, 153, 44), (255, 194, 0),
                            (147, 69, 52), (255, 115, 100), (236, 24, 0), (255, 56, 132)
                        ]
                        
                        for det in detections:
                            # Original coordinates (from YOLO on original frame)
                            ox1, oy1, ox2, oy2 = [int(v) for v in det['box']]
                            # Scale to resized frame
                            x1 = int(ox1 * scale_x)
                            y1 = int(oy1 * scale_y)
                            x2 = int(ox2 * scale_x)
                            y2 = int(oy2 * scale_y)
                            
                            cls_id = det['classId']
                            cls_name = det['className']
                            conf = det['score']
                            color = colors[cls_id % len(colors)]
                            
                            # Draw box on scaled frame
                            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                            
                            # Draw label
                            label = f"{cls_name} {conf*100:.0f}%"
                            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                            cv2.rectangle(annotated_frame, (x1, y1-th-10), (x1+tw+10, y1), color, -1)
                            cv2.putText(annotated_frame, label, (x1+5, y1-5),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                            
                            # Save detection to list and CSV (original coordinates)
                            det_record = {
                                'frame_number': frame_num,
                                'timestamp': datetime.now().isoformat(),
                                'class_id': cls_id,
                                'class_name': cls_name,
                                'confidence': conf,
                                'bbox': [ox1, oy1, ox2, oy2]  # Original coordinates
                            }
                            session['detections_list'].append(det_record)
                            session['detections_csv_writer'].writerow([
                                frame_num, datetime.now().isoformat(),
                                cls_id, cls_name, f"{conf:.4f}",
                                ox1, oy1, ox2, oy2  # Original coordinates
                            ])
                        
                        session['annotated_writer'].write(annotated_frame)
                    
                    session['frame_count'] += 1
                    
                    # Update UI every 10 frames
                    if frame_num % 10 == 0:
                        elapsed_str = time.strftime('%H:%M:%S', time.gmtime(elapsed))
                        ui.run_javascript(f'''
                            document.getElementById("recordingTime_{namespace}").textContent = "{elapsed_str}";
                            document.getElementById("recordingFrames_{namespace}").textContent = "({frame_num} frames)";
                        ''')
                
                except Exception as ex:
                    self.get_logger().error(f"[Recording] Frame capture error: {ex}")
            
            # 30 FPS = ~33ms interval
            timer = ui.timer(1/30, capture_frame)
            self.recording_timers[namespace] = timer
            self.get_logger().info(f"[Recording] Capture loop started at 30 FPS for {namespace}")
        
        def stop_recording():
            """Stop recording and finalize files."""
            import json
            
            self.recording_running[namespace] = False
            
            # Cancel timer
            if namespace in self.recording_timers:
                self.recording_timers[namespace].cancel()
                del self.recording_timers[namespace]
            
            session = self.recording_sessions.get(namespace)
            if session:
                # Close video writers
                if session.get('raw_writer'):
                    session['raw_writer'].release()
                if session.get('annotated_writer'):
                    session['annotated_writer'].release()
                
                # Close telemetry file
                if session.get('telemetry_file'):
                    session['telemetry_file'].close()
                
                # Save detections JSON and close CSV
                if session.get('yolo_active_at_start'):
                    # Save JSON
                    with open(session['detections_json_path'], 'w') as f:
                        json.dump({
                            'drone': namespace,
                            'recording_start': datetime.fromtimestamp(session['start_time']).isoformat(),
                            'total_frames': session['frame_count'],
                            'detections': session['detections_list']
                        }, f, indent=2)
                    
                    # Close CSV
                    if session.get('detections_csv_file'):
                        session['detections_csv_file'].close()
                
                elapsed = time.time() - session['start_time']
                self.get_logger().info(
                    f"[Recording] Stopped: {session['frame_count']} frames, "
                    f"{elapsed:.1f}s, saved to {session['output_dir']}"
                )
                
                ui.notify(
                    f"Recording saved: {session['frame_count']} frames ({elapsed:.1f}s)",
                    type='positive'
                )
                
                del self.recording_sessions[namespace]
            
            # Update UI
            rec_status_ref.text = '⚫ Not Recording'
            rec_status_ref.style('color: white')
            rec_btn.classes(remove='hidden', add='')
            rec_stop_btn.classes(remove='', add='hidden')
            
            # Hide recording overlay
            ui.run_javascript(f'''
                document.getElementById("recordingStatus_{namespace}").style.display = "none";
            ''')
        
        rec_btn.on_click(show_recording_dialog)
        rec_stop_btn.on_click(stop_recording)
        
        # ====================================================================
        # END RECORDING SECTION
        # ====================================================================
        
        # YOLO drawing JavaScript (client-side rendering only)
        ui.run_javascript(f'''
        (function setupYOLOCanvas() {{
            // Just ensure canvas is ready
            const canvas = document.getElementById("detectionCanvas_{namespace}");
            if (canvas) {{
                canvas.style.pointerEvents = "none";
            }}
        }})();
        ''')
        
        # Auto-start WebRTC connection with telemetry data channel
        ui.run_javascript(_fullscreen_video_script(namespace, ws_url))
    
    # ========================================================================
    # OVERRIDE PARENT CALLBACKS TO EMIT EVENTS
    # ========================================================================
    
    def _should_throttle(self, event_type: str, namespace: str) -> bool:
        """Check if an event should be throttled based on time since last emit."""
        if event_type not in self._throttle_intervals:
            return False  # No throttling for this event type
        
        now = time.time()
        interval = self._throttle_intervals[event_type]
        
        if event_type not in self._last_emit_time:
            self._last_emit_time[event_type] = {}
        
        last_time = self._last_emit_time[event_type].get(namespace, 0)
        if now - last_time < interval:
            return True  # Throttle this event
        
        self._last_emit_time[event_type][namespace] = now
        return False
    
    def _on_location(self, namespace: str, msg):
        """Override location callback to emit event."""
        super()._on_location(namespace, msg)
        
        # Throttle position updates to reduce browser load
        if not self._should_throttle('position', namespace):
            self.drone_position_update.emit({
                'namespace': namespace,
                'lat': msg.latitude,
                'lon': msg.longitude,
                'alt': msg.altitude
            })
        
        # Check for RTH landing detection (not throttled - critical for state)
        self._check_rth_landing(namespace, msg.altitude)
    
    def _on_heading(self, namespace: str, heading: float):
        """Override heading callback to emit event."""
        if namespace in self.drones:
            self.drones[namespace].heading = heading
        
        # Throttle heading updates
        if not self._should_throttle('heading', namespace):
            self.drone_heading_update.emit({
                'namespace': namespace,
                'heading': heading
            })
    
    def _on_battery(self, namespace: str, level: float):
        """Override battery callback to emit event."""
        # IMPORTANT: Call parent to update drone data AND RTH predictor
        super()._on_battery(namespace, level)
        
        # Detect battery swap: if battery level increases significantly, drone got a new battery
        # Reset to IDLE so it can be used for next relay
        if namespace in self.drone_last_battery:
            last_level = self.drone_last_battery[namespace]
            battery_increase = level - last_level
            
            # If battery increased by more than 10%, assume battery was swapped
            if battery_increase > 10:
                self._emit_log(f"[{namespace}] Battery swap detected ({last_level:.0f}% → {level:.0f}%)")
                
                # Delegate state change to mission controller
                if self.mission_controller and self.mission_controller.notify_battery_swap(namespace):
                    # Reset RTH predictor for fresh data with new battery
                    if namespace in self.rth_predictors:
                        self.rth_predictors[namespace] = DroneRTHPredictor(namespace=namespace)
                    
                    # Emit state update to refresh GUI
                    self.drone_state_update.emit({
                        'namespace': namespace,
                        'state': DroneState.IDLE
                    })
        
        self.drone_last_battery[namespace] = level
        
        self.drone_battery_update.emit({
            'namespace': namespace,
            'level': level
        })
        
        # Also emit RTH predictor update if available
        if namespace in self.rth_predictors:
            try:
                predictor = self.rth_predictors[namespace]
                debug_info = predictor.get_debug_info()
                
                # Add DJI's remaining_flight_time for comparison
                if namespace in self.drones:
                    debug_info['dji_remaining_flight_time'] = self.drones[namespace].remaining_flight_time
                
                self.rth_predictor_update.emit({
                    'namespace': namespace,
                    'debug_info': debug_info
                })
            except Exception as e:
                # Silently ignore threading issues during RTH predictor updates
                pass
    
    def _on_remaining_flight_time(self, namespace: str, time_remaining: float):
        """Override flight time callback to emit event."""
        if namespace in self.drones:
            self.drones[namespace].remaining_flight_time = time_remaining
        
        # Throttle flight time updates (1 Hz is sufficient for countdown display)
        if not self._should_throttle('flight_time', namespace):
            self.drone_flight_time_update.emit({
                'namespace': namespace,
                'time_remaining': time_remaining
            })
    
    def _on_recording_status(self, namespace: str, is_recording: bool):
        """Override recording status callback to emit event."""
        if namespace in self.drones:
            self.drones[namespace].is_recording = is_recording
        # Recording status not throttled - state change is important
        self.drone_recording_update.emit({
            'namespace': namespace,
            'is_recording': is_recording
        })
    
    def _on_satellite_count(self, namespace: str, count: int):
        """Override satellite count callback to emit event."""
        if namespace in self.drones:
            self.drones[namespace].satellite_count = count
        
        # Throttle satellite updates (slow changing)
        if not self._should_throttle('satellites', namespace):
            self.drone_satellite_update.emit({
                'namespace': namespace,
                'count': count
            })
    
    def _on_speed(self, namespace: str, speed: float):
        """Override speed callback to emit event."""
        if namespace in self.drones:
            self.drones[namespace].speed = speed
        
        # Throttle speed updates
        if not self._should_throttle('speed', namespace):
            self.drone_speed_update.emit({
                'namespace': namespace,
                'speed': speed
            })
    
    def _on_flight_mode(self, namespace: str, mode: str):
        """Override flight mode callback to emit event and track manual control."""
        super()._on_flight_mode(namespace, mode)
        
        # Throttle flight mode updates (rarely changes)
        if not self._should_throttle('flight_mode', namespace):
            self.drone_flight_mode_update.emit({
                'namespace': namespace,
                'flight_mode': mode
            })
    
    def _on_mission_status_update(self, namespace: str, state: MissionState, message: str):
        """Override mission status callback to emit event."""
        super()._on_mission_status_update(namespace, state, message)
        
        # Track when monitoring starts (timer will be started by UI timer)
        if state == MissionState.MONITORING and self._mission_start_time is None:
            self._mission_start_time = time.time()
            self._should_start_timer = True  # Flag for UI thread to pick up
        
        # Track mission statistics per drone iteration
        self._track_mission_stats(namespace, state)
        
        # Map MissionState to DroneState
        state_map = {
            MissionState.IDLE: DroneState.IDLE,
            MissionState.SETTING_RTH_ALTITUDE: DroneState.IDLE,
            MissionState.TAKING_OFF: DroneState.TAKING_OFF,
            MissionState.CLIMBING_TO_ALTITUDE: DroneState.TAKING_OFF,
            MissionState.TRANSIT_TO_MONITORING: DroneState.FLYING_TO_POINT,
            MissionState.APPROACHING_POINT: DroneState.FLYING_TO_POINT,
            MissionState.MONITORING: DroneState.MONITORING,
            MissionState.WAITING_FOR_RELAY: DroneState.WAITING_FOR_RELAY,
            MissionState.CAMERA_SYNC: DroneState.CAMERA_SYNC,
            MissionState.RETURNING_HOME: DroneState.RETURNING_HOME,
            MissionState.COMPLETED: DroneState.IDLE,
            MissionState.ABORTED: DroneState.IDLE,
            MissionState.ERROR: DroneState.EMERGENCY,
        }
        
        if state in state_map:
            self.drone_state_update.emit({
                'namespace': namespace,
                'state': state_map[state]
            })
    
    def _track_mission_stats(self, namespace: str, state: MissionState):
        """Track mission statistics for each drone iteration."""
        if not hasattr(self, 'mission_controller') or not self.mission_controller:
            return
        
        mission = self.mission_controller.drone_missions.get(namespace)
        if not mission:
            return
        
        # When transit starts, create a new entry with estimated travel time
        if state == MissionState.TRANSIT_TO_MONITORING:
            # Increment iteration counter for this drone
            if namespace not in self.drone_iteration_counter:
                self.drone_iteration_counter[namespace] = 0
            self.drone_iteration_counter[namespace] += 1
            iteration = self.drone_iteration_counter[namespace]
            
            # Get estimated travel time from mission
            est_travel = mission.estimated_travel_time
            self._add_mission_stat(namespace, iteration, est_travel)
        
        # When monitoring starts, update with actual travel time
        elif state == MissionState.MONITORING:
            if namespace in self.drone_iteration_counter:
                iteration = self.drone_iteration_counter[namespace]
                actual_travel = mission.actual_travel_time
                self._add_mission_stat(namespace, iteration, 0, actual_travel=actual_travel)
        
        # When RTH starts (normal, aborted, or error), start tracking for landing detection
        elif state in [MissionState.RETURNING_HOME, MissionState.ABORTED, MissionState.ERROR]:
            # Only start tracking if not already tracking this drone
            if namespace not in self.drone_rth_tracking:
                self.drone_rth_tracking[namespace] = {
                    'start_time': time.time(),
                    'last_alt': None,
                    'stable_count': 0,
                    'detected': False
                }
                self._emit_log(f"[{namespace}] RTH tracking started")
    
    def _check_rth_landing(self, namespace: str, altitude: float):
        """Check if a drone in RTH state has landed based on altitude stability."""
        if namespace not in self.drone_rth_tracking:
            return
        
        tracking = self.drone_rth_tracking[namespace]
        if tracking['detected']:
            return  # Already detected landing
        
        # Need low altitude (< 3m) and stable (not changing much)
        LANDING_ALTITUDE_THRESHOLD = 3.0  # meters
        ALTITUDE_STABLE_THRESHOLD = 0.5   # meters - altitude change threshold
        STABLE_COUNT_REQUIRED = 3         # number of consecutive stable readings
        
        if tracking['last_alt'] is not None:
            alt_change = abs(altitude - tracking['last_alt'])
            
            # Check if altitude is low and stable
            if altitude < LANDING_ALTITUDE_THRESHOLD and alt_change < ALTITUDE_STABLE_THRESHOLD:
                tracking['stable_count'] += 1
                
                if tracking['stable_count'] >= STABLE_COUNT_REQUIRED:
                    # Landing detected!
                    tracking['detected'] = True
                    rth_duration = time.time() - tracking['start_time']
                    
                    # Update mission stats with RTH time
                    self._update_mission_stat_rth(namespace, rth_duration)
                    self._emit_log(f"[{namespace}] Landed after RTH ({rth_duration:.1f}s)")
                    
                    # Delegate state change to mission controller
                    if self.mission_controller:
                        self.mission_controller.notify_landing_detected(namespace, rth_duration)
                        # Emit state update to refresh GUI (subscriber will call _update_state_icons)
                        self.drone_state_update.emit({
                            'namespace': namespace,
                            'state': DroneState.IDLE
                        })
                    
                    # Clean up tracking
                    del self.drone_rth_tracking[namespace]
                    return
            else:
                # Reset stable count if altitude is changing or too high
                tracking['stable_count'] = 0
        
        tracking['last_alt'] = altitude
    
    def _on_relay_countdown_update(self, countdown: float, next_drone: str, timing_breakdown: dict = None):
        """Override relay countdown callback to emit event."""
        super()._on_relay_countdown_update(countdown, next_drone, timing_breakdown)
        self.relay_countdown_update.emit({
            'countdown': countdown,
            'next_drone': next_drone,
            'timing_breakdown': timing_breakdown or {}
        })
    
    def _on_takeoff_confirmation_request(self, drone_name: str, callback):
        """Handle takeoff confirmation request from mission controller (relay auto-launch).
        
        This is called from the mission controller thread, so we queue the request
        for processing in the UI thread (similar to other background events).
        """
        # Queue the request for the UI thread to process
        self._takeoff_confirmation_queue.append({
            'drone_name': drone_name,
            'callback': callback
        })
    
    def _process_takeoff_confirmation_sync(self, request: dict):
        """Process a takeoff confirmation request in the UI thread (sync version)."""
        try:
            drone_name = request['drone_name']
            callback = request['callback']
            
            # Show the confirmation dialog using sync version with callback storage
            self._relay_takeoff_callback = callback
            # Use main_row container to ensure proper UI context
            if hasattr(self, 'main_row') and self.main_row:
                with self.main_row:
                    self._show_relay_takeoff_dialog_sync(drone_name)
            else:
                self._show_relay_takeoff_dialog_sync(drone_name)
            # Note: callback will be called from dialog buttons
        except Exception as e:
            self._emit_log(f"[ERROR] Failed to process takeoff confirmation: {e}")
            request['callback'](False)
            self._takeoff_confirmation_dialog_open = False
    
    def _show_relay_takeoff_dialog_sync(self, drone_name: str):
        """Show relay takeoff confirmation dialog (sync version for background thread requests)."""
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label(f'🚁 Relay Takeoff Confirmation').classes('text-xl font-bold text-blue-700')
            ui.separator()
            ui.label(f'Drone "{drone_name}" is ready to relay.').classes('text-lg mt-2')
            ui.label('Confirm to launch relay drone.').classes('text-sm text-gray-600 mt-1')
            
            def on_confirm():
                dialog.close()
                self._takeoff_confirmation_dialog_open = False
                # Play takeoff confirmation sound
                self._play_sound('take_off.mp3')
                # Call the mission controller callback
                if hasattr(self, '_relay_takeoff_callback') and self._relay_takeoff_callback:
                    self._relay_takeoff_callback(True)
                    self._relay_takeoff_callback = None
            
            def on_cancel():
                dialog.close()
                self._takeoff_confirmation_dialog_open = False
                # Call the mission controller callback with False
                if hasattr(self, '_relay_takeoff_callback') and self._relay_takeoff_callback:
                    self._relay_takeoff_callback(False)
                    self._relay_takeoff_callback = None
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=on_cancel, color='red').props('flat')
                ui.button('Confirm', on_click=on_confirm, color='primary')
        
        dialog.open()
    
    def _on_vertical_separation_alert(self, drone1: str, drone2: str, separation: float, alt1: float, alt2: float):
        """Handle vertical separation alert from mission controller.
        
        This is called from a background thread, so we queue the alert
        for processing in the UI thread.
        """
        # Queue the alert for processing in UI thread (thread-safe)
        self._vertical_separation_alerts.append({
            'drone1': drone1,
            'drone2': drone2,
            'separation': separation,
            'alt1': alt1,
            'alt2': alt2
        })
    
    def _on_vertical_separation_countdown_start(self):
        """Handle countdown start - play 20_seconds.mp3.
        
        This is called from a background thread, so we queue it for the UI thread.
        """
        self._emit_log("[DEBUG] _on_vertical_separation_countdown_start called!")
        self._vertical_separation_countdown_active = True
        self._notification_queue.append({
            'message': '⏱️ 20-SECOND COUNTDOWN STARTED!\nRTH will trigger if vertical separation not restored.',
            'type': 'warning',
            'timeout': 20000
        })
        # Queue the audio playback
        self._vertical_separation_alerts.append({
            'action': 'countdown_start'
        })
    
    def _on_vertical_separation_countdown_cancel(self):
        """Handle countdown cancel - play vertical_separation_respected.mp3.
        
        This is called from a background thread, so we queue it for the UI thread.
        """
        self._emit_log("[DEBUG] _on_vertical_separation_countdown_cancel called!")
        self._vertical_separation_countdown_active = False
        self._notification_queue.append({
            'message': '✅ Vertical separation restored!\nCountdown cancelled, mission continues.',
            'type': 'positive',
            'timeout': 5000
        })
        # Queue the audio playback
        self._vertical_separation_alerts.append({
            'action': 'countdown_cancel'
        })
    
    def _on_vertical_separation_mission_stopped(self):
        """Handle mission stopped due to vertical separation countdown expiry.
        
        This is called from a background thread, so we queue UI updates.
        """
        self._emit_log("[CRITICAL] Mission STOPPED due to vertical separation countdown expiry!")
        self._vertical_separation_countdown_active = False
        self._notification_queue.append({
            'message': '🛑 MISSION STOPPED!\nVertical separation countdown expired.\nAll drones returning home.',
            'type': 'negative',
            'timeout': 10000
        })
        # Queue the UI state update
        self._vertical_separation_alerts.append({
            'action': 'mission_stopped'
        })
    
    def _get_drone_flight_mode(self, namespace: str) -> str:
        """Get drone's current flight mode for manual flight detection."""
        if namespace in self.drones:
            return self.drones[namespace].flight_mode or ""
        return ""
    
    def _get_drone_altitude(self, namespace: str) -> float:
        """Get drone's current altitude for vertical separation check."""
        if namespace in self.drones:
            alt = self.drones[namespace].altitude or 0.0
            self.get_logger().debug(f"_get_drone_altitude({namespace}): {alt:.1f}m")
            return alt
        self.get_logger().warning(f"_get_drone_altitude({namespace}): drone not found!")
        return 0.0
    
    def _auto_connect_on_startup(self):
        """Auto-connect to predefined drone IPs at startup."""
        import threading
        
        def connect_drones():
            # Wait a bit for ROS2 to fully initialize
            time.sleep(2.0)
            
            for ip, namespace in self.AUTO_CONNECT_DRONE_IPS:
                self.get_logger().info(f"[AUTO-CONNECT] Connecting to {namespace} at {ip}...")
                try:
                    result = self.connect_drone(ip, namespace)
                    if result:
                        self.get_logger().info(f"[AUTO-CONNECT] Successfully connected {namespace}")
                    else:
                        self.get_logger().warning(f"[AUTO-CONNECT] Failed to connect {namespace} at {ip}")
                except Exception as e:
                    self.get_logger().error(f"[AUTO-CONNECT] Error connecting {namespace}: {e}")
                # Small delay between connections
                time.sleep(1.0)
        
        # Run in background thread to not block GUI startup
        threading.Thread(target=connect_drones, daemon=True).start()
    
    def connect_drone(self, ip_address: str, namespace: str = None) -> bool:
        """Override connect_drone to emit event on success."""
        result = super().connect_drone(ip_address, namespace)
        if result:
            # result is the actual namespace string used (may differ from input if auto-discovered)
            ns = result
            if ns in self.drones:
                self.drone_connected_event.emit({
                    'namespace': ns,
                    'drone': self.drones[ns]
                })
                self._emit_log(f"[CONNECTED] {ns} at {ip_address or self.drones[ns].ip_address}")
                
                # Check if drone was auto-added to relay
                if self.mission_controller.is_drone_in_mission(ns):
                    position = len(self.mission_controller.drone_order)
                    # Queue notification for UI thread (thread-safe)
                    self._notification_queue.append({
                        'message': f'{ns} auto-added to relay queue (position {position})',
                        'type': 'positive',
                        'timeout': 5000
                    })
                    self._emit_log(f"[RELAY] {ns} auto-joined relay mission")
        return result
    
    def disconnect_drone(self, namespace: str) -> bool:
        """Override disconnect_drone to emit event."""
        result = super().disconnect_drone(namespace)
        if result:
            self.drone_disconnected_event.emit({'namespace': namespace})
            self._emit_log(f"[DISCONNECTED] {namespace}")
        return result
    
    def set_monitoring_point(self, lat: float, lon: float, alt: float, heading: float = 0.0, source: str = "manual"):
        """Override to emit event."""
        super().set_monitoring_point(lat, lon, alt, heading, source)
        self.monitoring_point_update.emit({
            'lat': lat,
            'lon': lon,
            'alt': alt,
            'heading': heading
        })
        self._emit_log(f"[POINT] Set: ({lat:.6f}, {lon:.6f}, {alt:.1f}m) heading={heading:.0f}°")
    
    def clear_monitoring_point(self):
        """Override to emit event."""
        super().clear_monitoring_point()
        self.monitoring_point_update.emit({'clear': True})
        self._emit_log("[POINT] Cleared")
    
    def _emit_log(self, message: str):
        """Emit a log event."""
        self.log_event.emit({'message': message})
    
    # ========================================================================
    # UI CONSTRUCTION
    # ========================================================================
    
    def _build_ui(self):
        """Build the main UI layout with event subscriptions."""
        
        # Clear UI references on page refresh (map and arrows need to be recreated)
        self.drone_cards.clear()
        self.drone_arrows.clear()
        self.drone_labels.clear()
        self.drone_buttons.clear()
        self.monitoring_marker = None
        self.monitoring_circle = None
        
        # Clear trajectory data on page refresh (polylines need to be recreated with new map)
        self.drone_trajectory_lines.clear()
        # Keep drone_trajectories data so we can redraw if needed
        # Keep drone_is_flying state
        
        # Prevent browser caching
        ui.add_head_html('''
            <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
            <meta http-equiv="Pragma" content="no-cache">
            <meta http-equiv="Expires" content="0">
            <script>
                // Clear browser storage on page load to prevent stale data
                sessionStorage.clear();
                localStorage.removeItem('wildperpetua_state');
            </script>
        ''')
        
        # Add CSS and JS (cache-busting with version parameter)
        ui.add_head_html("""
            <script src='/static/arrows.js?v=2'></script>
            <style>
                .drone-card { 
                    min-width: 280px; 
                    transition: all 0.3s ease;
                }
                .drone-card:hover {
                    transform: translateY(-2px);
                    box-shadow: 0 4px 12px rgba(0,0,0,0.15);
                }
                .countdown-display { 
                    font-size: 1.5rem; 
                    font-weight: bold; 
                }
                .mission-status { 
                    padding: 10px; 
                    border-radius: 8px; 
                }
                @keyframes blink {
                    0%, 100% { opacity: 1; }
                    50% { opacity: 0.5; }
                }
                .pulse {
                    animation: pulse 2s infinite;
                }
                @keyframes pulse {
                    0% { box-shadow: 0 0 0 0 rgba(76, 175, 80, 0.4); }
                    70% { box-shadow: 0 0 0 10px rgba(76, 175, 80, 0); }
                    100% { box-shadow: 0 0 0 0 rgba(76, 175, 80, 0); }
                }
            </style>
        """)
        
        with ui.row().classes('w-full h-full').style('display: flex; height: 95vh; gap: 10px; padding: 10px;') as self.main_row:
            # Left panel: Drone management
            self._build_left_panel()
            
            # Right panel: Map and mission control
            self._build_right_panel()
        
        # Subscribe to events for UI updates
        self._setup_event_subscriptions()
        
        # Timer to process log message queue (runs in UI thread)
        ui.timer(0.5, self._process_log_queue)
        
        # Populate drone list after map is created (so arrows can be placed)
        self._refresh_drone_list()
        
        # Restore mission state from backend (critical on page refresh)
        self._restore_mission_state_from_backend()
    
    def _restore_mission_state_from_backend(self):
        """Restore mission UI state from backend mission_controller after page refresh.
        
        This ensures that if a mission is running in the backend, the UI displays it correctly
        even after a page refresh.
        """
        if not self.mission_controller:
            return
        
        # 1. Restore monitoring point from mission_controller config
        if self.monitoring_point.is_set:
            lat = self.monitoring_point.latitude
            lon = self.monitoring_point.longitude
            alt = self.monitoring_point.altitude
            heading = self.monitoring_point.heading
            
            # Update input fields
            if self.lat_input:
                self.lat_input.value = f"{lat:.6f}"
            if self.lon_input:
                self.lon_input.value = f"{lon:.6f}"
            if self.alt_input:
                self.alt_input.value = f"{alt:.0f}"
            if self.heading_input:
                self.heading_input.value = f"{heading:.0f}"
            
            # Recreate marker on map
            if self.map:
                self.monitoring_marker = self.map.marker(latlng=[lat, lon])
                self.monitoring_circle = self.map.generic_layer(
                    name='circle',
                    args=[[lat, lon], {'radius': 50, 'color': 'green', 'fillOpacity': 0.2, 'weight': 2}]
                )
        
        # 2. Restore mission parameters from mission_controller config
        if self.mission_controller.config:
            try:
                rth_alt = self.mission_controller.config.rth_altitude
                if self.rth_alt_input and rth_alt > 0:
                    self.rth_alt_input.value = str(int(rth_alt))
            except:
                pass
            
            try:
                safety_buf = self.mission_controller.config.safety_buffer_seconds
                if self.safety_buffer_input and safety_buf >= 0:
                    self.safety_buffer_input.value = str(int(safety_buf))
            except:
                pass
            
            try:
                min_batt = self.mission_controller.config.min_battery_to_launch
                if self.min_battery_input and min_batt > 0:
                    self.min_battery_input.value = str(int(min_batt))
            except:
                pass
            
            try:
                min_sats = self.mission_controller.config.min_satellites
                if self.min_satellites_input and min_sats > 0:
                    self.min_satellites_input.value = str(int(min_sats))
            except:
                pass
        
        # 3. Restore mission mode toggle state
        if self.mission_mode_switch:
            is_free_flight = self.mission_controller.mission_mode == MissionMode.FREE_FLIGHT
            self.mission_mode_switch.value = is_free_flight
            if self.mission_mode_label:
                label_text = '✈️ Free Flight' if is_free_flight else '📍 Monitor'
                label_color = '#d32f2f' if is_free_flight else '#1976d2'
                label_bg = '#ffebee' if is_free_flight else '#e3f2fd'
                self.mission_mode_label.text = label_text
                self.mission_mode_label.style(f'color: {label_color}; background: {label_bg};')
        
        # 4. Restore mission status display if mission is active
        active_drones = list(self.mission_controller.drone_missions.keys())
        if active_drones:
            # Determine mission type
            single_drone_mission = len(active_drones) == 1
            mission_type = "Single Drone" if single_drone_mission else "Relay"
            
            if self.mission_status_label:
                self.mission_status_label.text = mission_type
                status_color = '#e8f5e9' if mission_type == "Single Drone" else '#e3f2fd'
                status_text_color = '#2e7d32' if mission_type == "Single Drone" else '#1565c0'
                self.mission_status_label.style(f'background: {status_color}; color: {status_text_color};')
            
            # Update active drone label
            if self.active_drone_label:
                # Get first drone in mission as active
                active_ns = active_drones[0]
                self.active_drone_label.text = active_ns
                self.active_drone_label.style('background: #e8f5e9; color: #2e7d32;')
            
            # Restore mission timer if we have it
            if active_drones and active_drones[0] in self.mission_controller.drone_missions:
                mission = self.mission_controller.drone_missions[active_drones[0]]
                if mission.mission_start_time and mission.mission_start_time > 0:
                    # Mission has started, will continue to tick from the mission_controller's clock
                    # The timer label will be updated by the relay countdown update events
                    self._should_start_timer = True
        else:
            # No active mission
            if self.mission_status_label:
                self.mission_status_label.text = "Inactive"
                self.mission_status_label.style('background: #eeeeee; color: #616161;')
        
        # 5. Update drones needed estimate
        self._update_drones_needed()
        
        # 6. Build state machine display for any active drones
        if active_drones:
            self._build_state_machine_display()
        
        self._emit_log("[RESTORE] Mission state restored from backend")
    
    def _setup_event_subscriptions(self):
        """Set up event subscriptions for UI updates."""
        
        @self.drone_position_update.subscribe
        def on_position(data: dict):
            ns = data['namespace']
            lat = data['lat']
            lon = data['lon']
            alt = data['alt']
            
            # Center map on first valid position from any new drone
            if ns not in self._centered_on_drone and self.map:
                if lat != 0.0 or lon != 0.0:
                    self.map.set_center((lat, lon))
                    self.map.set_zoom(17)
                    self._centered_on_drone.add(ns)
            
            # Update arrow on map
            if ns in self.drone_arrows:
                heading = self.drones[ns].heading if ns in self.drones else 0.0
                self.drone_arrows[ns].update(lat, lon, heading)
            
            # Update trajectory if drone is flying
            if self.drone_is_flying.get(ns, False) and (lat != 0.0 or lon != 0.0):
                self._update_drone_trajectory(ns, lat, lon)
            
            # Update altitude label
            if ns in self.drone_labels and 'altitude' in self.drone_labels[ns]:
                self.drone_labels[ns]['altitude'].text = f"{alt:.1f}m"
        
        @self.drone_heading_update.subscribe
        def on_heading(data: dict):
            ns = data['namespace']
            heading = data['heading']
            
            # Update arrow rotation
            if ns in self.drone_arrows and ns in self.drones:
                drone = self.drones[ns]
                if drone.latitude != 0 and drone.longitude != 0:
                    self.drone_arrows[ns].update(drone.latitude, drone.longitude, heading)
        
        @self.drone_battery_update.subscribe
        def on_battery(data: dict):
            ns = data['namespace']
            battery = data['level']
            
            if ns in self.drone_labels and 'battery' in self.drone_labels[ns]:
                color = 'green' if battery > 50 else 'orange' if battery > 20 else 'red'
                self.drone_labels[ns]['battery'].text = f"{battery:.0f}%"
                self.drone_labels[ns]['battery'].style(f'color: {color}; font-weight: bold')
        
        @self.drone_flight_time_update.subscribe
        def on_flight_time(data: dict):
            ns = data['namespace']
            time_remaining = data['time_remaining']
            
            if ns in self.drone_labels and 'flight_time' in self.drone_labels[ns]:
                minutes = int(time_remaining // 60)
                seconds = int(time_remaining % 60)
                color = 'green' if time_remaining > 300 else 'orange' if time_remaining > 120 else 'red'
                self.drone_labels[ns]['flight_time'].text = f"{minutes}:{seconds:02d}"
                self.drone_labels[ns]['flight_time'].style(f'color: {color}; font-weight: bold')
        
        @self.drone_recording_update.subscribe
        def on_recording(data: dict):
            ns = data['namespace']
            is_recording = data['is_recording']
            
            if ns in self.drone_labels and 'recording' in self.drone_labels[ns]:
                if is_recording:
                    self.drone_labels[ns]['recording'].text = "REC"
                    self.drone_labels[ns]['recording'].style('color: #c62828; font-weight: bold;')
                    if 'recording_icon' in self.drone_labels[ns]:
                        self.drone_labels[ns]['recording_icon'].style('font-size: 28px; color: #c62828; animation: blink 1s infinite;')
                else:
                    self.drone_labels[ns]['recording'].text = "OFF"
                    self.drone_labels[ns]['recording'].style('color: #bdbdbd; font-weight: bold;')
                    if 'recording_icon' in self.drone_labels[ns]:
                        self.drone_labels[ns]['recording_icon'].style('font-size: 28px; color: #bdbdbd; animation: none;')
        
        @self.drone_satellite_update.subscribe
        def on_satellite(data: dict):
            ns = data['namespace']
            count = data['count']
            
            if ns in self.drone_labels and 'satellites' in self.drone_labels[ns]:
                self.drone_labels[ns]['satellites'].text = f"{count}"
                # Color code based on count
                if count >= 10:
                    color = '#2e7d32'  # green
                elif count >= 6:
                    color = '#ef6c00'  # orange
                else:
                    color = '#c62828'  # red
                self.drone_labels[ns]['satellites'].style(f'color: {color}')
        
        @self.drone_speed_update.subscribe
        def on_speed(data: dict):
            ns = data['namespace']
            speed = data['speed']
            
            if ns in self.drone_labels and 'speed' in self.drone_labels[ns]:
                self.drone_labels[ns]['speed'].text = f"{speed:.1f}m/s"
        
        @self.drone_flight_mode_update.subscribe
        def on_flight_mode(data: dict):
            """Update manual flight indicator when flight mode changes."""
            ns = data['namespace']
            mode = data['flight_mode']
            
            if ns in self.drone_labels and 'manual_indicator' in self.drone_labels[ns]:
                # Show indicator if NOT in virtual_stick mode (pilot has manual control)
                is_manual = mode.lower() != 'virtual_stick' and mode.lower() != 'unknown'
                if is_manual:
                    self.drone_labels[ns]['manual_indicator'].style('display: inline-flex')
                    # Only log once when transitioning to manual mode
                    prev_mode = getattr(self, '_last_flight_mode', {}).get(ns)
                    if prev_mode != mode:
                        if not hasattr(self, '_last_flight_mode'):
                            self._last_flight_mode = {}
                        self._last_flight_mode[ns] = mode
                        self._emit_log(f"[{ns}] Manual control detected: {mode}")
                else:
                    self.drone_labels[ns]['manual_indicator'].style('display: none')
                    # Track mode change so we log again if it returns to manual
                    if not hasattr(self, '_last_flight_mode'):
                        self._last_flight_mode = {}
                    self._last_flight_mode[ns] = mode
        
        @self.rth_predictor_update.subscribe
        def on_rth_predictor(data: dict):
            ns = data['namespace']
            debug_info = data.get('debug_info', {})
            
            predicted_rth = debug_info.get('predicted_rth_seconds', float('inf'))
            drain_rate = debug_info.get('drain_rate_per_min', 0.0) / 60  # Convert back to %/s for existing code
            is_active = debug_info.get('is_active', False)
            data_points = debug_info.get('data_points', 0)
            
            if ns not in self.drone_labels:
                return
            
            labels = self.drone_labels[ns]
            
            # Update RTH debug panel if this is the active monitoring drone
            # (Do this FIRST, before any early returns, so debug info is always shown)
            if hasattr(self, 'rth_debug_container') and self.rth_debug_container:
                # Check if this drone is in MONITORING state
                if ns in self.drones and self.drones[ns].state == DroneState.MONITORING:
                    self._update_rth_debug_panel(ns, debug_info)
            
            # Show/hide the RTH predictor row based on active state
            if 'rth_predictor_row' in labels:
                if is_active and data_points >= 3:  # MIN_DATAPOINTS = 3
                    labels['rth_predictor_row'].style('display: flex')
                else:
                    labels['rth_predictor_row'].style('display: none')
                    return
            
            # Update predicted RTH time
            if 'rth_predicted' in labels:
                if predicted_rth != float('inf') and predicted_rth > 0:
                    minutes = int(predicted_rth // 60)
                    seconds = int(predicted_rth % 60)
                    color = '#2e7d32' if predicted_rth > 300 else '#f57c00' if predicted_rth > 120 else '#c62828'
                    labels['rth_predicted'].text = f"{minutes}:{seconds:02d}"
                    labels['rth_predicted'].style(f'color: {color}; font-weight: bold')
                else:
                    labels['rth_predicted'].text = "--:--"
                    labels['rth_predicted'].style('color: #9e9e9e')
            
            # Update drain rate (%/min)
            if 'rth_drain_rate' in labels:
                drain_per_min = drain_rate * 60  # Convert from %/s to %/min
                if drain_per_min > 0:
                    labels['rth_drain_rate'].text = f"{drain_per_min:.2f}%/min"
                else:
                    labels['rth_drain_rate'].text = "--%/min"
            
            # Update data points count
            if 'rth_data_points' in labels:
                labels['rth_data_points'].text = f"{data_points} pts"
        
        @self.drone_state_update.subscribe
        def on_state(data: dict):
            ns = data['namespace']
            state = data['state']
            
            # Track flying state for trajectory
            flying_states = [
                DroneState.TAKING_OFF,
                DroneState.FLYING_TO_POINT,
                DroneState.MONITORING,
                DroneState.WAITING_FOR_RELAY,
                DroneState.CAMERA_SYNC,
                DroneState.RETURNING_HOME,
            ]
            
            was_flying = self.drone_is_flying.get(ns, False)
            is_flying = state in flying_states
            
            # Start new trajectory on takeoff
            if not was_flying and is_flying:
                self._start_drone_trajectory(ns)
            
            # Clear trajectory with fade on landing
            if was_flying and not is_flying:
                self._fade_and_clear_trajectory(ns)
            
            self.drone_is_flying[ns] = is_flying
            
            if ns in self.drone_labels and 'state' in self.drone_labels[ns]:
                # Use simple text labels - icons are in the card header
                state_colors = {
                    DroneState.DISCONNECTED: "background: #ffebee; color: #c62828;",
                    DroneState.CONNECTED: "background: #e8f5e9; color: #2e7d32;",
                    DroneState.IDLE: "background: #f5f5f5; color: #616161;",
                    DroneState.TAKING_OFF: "background: #e3f2fd; color: #1565c0;",
                    DroneState.FLYING_TO_POINT: "background: #e3f2fd; color: #1565c0;",
                    DroneState.MONITORING: "background: #f3e5f5; color: #7b1fa2;",
                    DroneState.WAITING_FOR_RELAY: "background: #fff8e1; color: #f9a825;",
                    DroneState.CAMERA_SYNC: "background: #e8f5e9; color: #2e7d32;",
                    DroneState.RETURNING_HOME: "background: #fff3e0; color: #ef6c00;",
                    DroneState.LANDING: "background: #e0f2f1; color: #00695c;",
                    DroneState.EMERGENCY: "background: #ffebee; color: #c62828;"
                }
                style = state_colors.get(state, "background: #f5f5f5; color: #616161;")
                self.drone_labels[ns]['state'].text = state.value
                self.drone_labels[ns]['state'].style(style)
                
                # Highlight active monitoring drone
                if state == DroneState.MONITORING and ns in self.drone_cards:
                    self.drone_cards[ns].style('border: 3px solid #4CAF50; box-shadow: 0 0 10px #4CAF50')
                elif ns in self.drone_cards:
                    self.drone_cards[ns].style('border: 1px solid #ddd; box-shadow: none')
            
            # Update state machine display icons (if they exist)
            self._update_state_icons(ns)
        
        @self.drone_connected_event.subscribe
        def on_connected(data: dict):
            self._refresh_drone_list()
            self._build_state_machine_display()  # Update state machine display
            # Try to center map on the newly connected drone (if it has position)
            ns = data.get('namespace')
            drone = data.get('drone')
            if drone and self.map and ns not in self._centered_on_drone:
                lat = drone.latitude
                lon = drone.longitude
                # Only center if we have valid coordinates (not 0,0)
                if lat != 0.0 or lon != 0.0:
                    self.map.set_center((lat, lon))
                    self.map.set_zoom(17)
                    self._centered_on_drone.add(ns)
            # Note: If drone doesn't have position yet, position_update handler will center on first GPS fix
        
        @self.drone_disconnected_event.subscribe
        def on_disconnected(data: dict):
            ns = data['namespace']
            if ns in self.drone_cards:
                self.drone_cards[ns].delete()
                del self.drone_cards[ns]
            if ns in self.drone_arrows:
                self.drone_arrows[ns].destroy()
                del self.drone_arrows[ns]
            if ns in self.drone_labels:
                del self.drone_labels[ns]
            # Clean up trajectory data
            if ns in self.drone_trajectory_lines and self.drone_trajectory_lines[ns]:
                try:
                    self.map.remove_layer(self.drone_trajectory_lines[ns])
                except:
                    pass
                del self.drone_trajectory_lines[ns]
            if ns in self.drone_trajectories:
                del self.drone_trajectories[ns]
            if ns in self.drone_is_flying:
                del self.drone_is_flying[ns]
            self._build_state_machine_display()  # Update state machine display
        
        @self.monitoring_point_update.subscribe
        def on_monitoring_point(data: dict):
            if data.get('clear'):
                if self.monitoring_marker:
                    self.map.remove_layer(self.monitoring_marker)
                    self.monitoring_marker = None
                if self.monitoring_circle:
                    self.map.remove_layer(self.monitoring_circle)
                    self.monitoring_circle = None
                # Reset input fields
                if self.lat_input:
                    self.lat_input.value = '0.0'
                if self.lon_input:
                    self.lon_input.value = '0.0'
                if self.heading_input:
                    self.heading_input.value = '0'
            else:
                lat, lon, alt = data['lat'], data['lon'], data['alt']
                heading = data.get('heading', 0)
                
                # Update input fields
                if self.lat_input:
                    self.lat_input.value = f"{lat:.6f}"
                if self.lon_input:
                    self.lon_input.value = f"{lon:.6f}"
                if self.alt_input:
                    self.alt_input.value = f"{alt:.0f}"
                if self.heading_input:
                    self.heading_input.value = f"{heading:.0f}"
                
                if self.map:
                    if self.monitoring_marker:
                        self.map.remove_layer(self.monitoring_marker)
                    if self.monitoring_circle:
                        self.map.remove_layer(self.monitoring_circle)
                    
                    self.monitoring_marker = self.map.marker(latlng=[lat, lon])
                    self.monitoring_circle = self.map.generic_layer(
                        name='circle',
                        args=[[lat, lon], {'radius': 50, 'color': 'green', 'fillOpacity': 0.2, 'weight': 2}]
                    )
        
        @self.relay_countdown_update.subscribe
        def on_countdown(data: dict):
            countdown = data['countdown']
            next_drone = data['next_drone']
            timing_breakdown = data.get('timing_breakdown', {})
            
            # Check if manual swap just completed - restore UI
            if self._manual_swap_active and self.mission_controller and not self.mission_controller.is_manual_swap_active():
                self._manual_swap_active = False
                # Restore force swap button
                if self.force_swap_button:
                    self.force_swap_button.props(remove='disabled')
                    self.force_swap_button.text = 'SWAP'
                self._emit_log("[SWAP] Swap completed - returning to automatic countdown mode")
            
            # Continue updating countdown display even during manual swap
            # (User wants to see the countdown of the monitoring drone)
            
            # Update timing breakdown display
            if timing_breakdown and hasattr(self, 'timing_breakdown_container'):
                self.timing_breakdown_container.style('display: flex;')
                
                remaining = timing_breakdown.get('remaining_flight_time', 0)
                travel = timing_breakdown.get('avg_travel_time', 0)
                buffer = timing_breakdown.get('safety_buffer', 0)
                
                # Handle "Collecting..." state when remaining is -1
                if remaining == -1:
                    if hasattr(self, 'remaining_time_label'):
                        self.remaining_time_label.text = f"To RTH: --:--"
                    if hasattr(self, 'travel_time_label'):
                        self.travel_time_label.text = f"Travel: --:--"
                else:
                    if hasattr(self, 'remaining_time_label'):
                        self.remaining_time_label.text = f"To RTH: {int(remaining//60)}:{int(remaining%60):02d}"
                    if hasattr(self, 'travel_time_label'):
                        self.travel_time_label.text = f"Travel: {int(travel//60)}:{int(travel%60):02d}"
                if hasattr(self, 'buffer_time_label'):
                    self.buffer_time_label.text = f"Buffer: {int(buffer//60)}:{int(buffer%60):02d}"
            
            if self.countdown_label:
                # Handle "Collecting..." state when countdown is -1
                if countdown == -1:
                    self.countdown_label.text = "Collecting..."
                    self.countdown_label.style('color: #1976d2; font-weight: bold; font-size: 1.3rem;')
                    if self.countdown_progress:
                        self.countdown_progress.value = 0
                    # Hide alert container during collection
                    if hasattr(self, 'relay_alert_container') and self.relay_alert_container:
                        self.relay_alert_container.style('display: none;')
                elif countdown > 0:
                    minutes = int(countdown // 60)
                    seconds = int(countdown % 60)
                    self.countdown_label.text = f"{minutes}:{seconds:02d}"
                    self.countdown_label.style('color: #bf360c; font-weight: bold')
                    
                    if self.countdown_progress:
                        self.countdown_progress.value = max(0, min(1, countdown / 300))
                    
                    # Update segmented progress bar
                    self._update_countdown_segments(countdown)
                    
                    # Threshold-based preparation alerts - use the alert container
                    if hasattr(self, 'relay_alert_container') and self.relay_alert_container:
                        if countdown <= 60:  # 1 minute - CONNECT NOW
                            self.relay_alert_container.style('background: linear-gradient(135deg, #ffebee 0%, #ffcdd2 100%); border: 2px solid #f44336; display: flex;')
                            self.relay_alert_label.text = f"🚨 CONNECT {next_drone} NOW!"
                            self.relay_alert_label.style('color: #c62828; animation: blink 0.5s infinite')
                            self.relay_alert_icon.style('color: #c62828; animation: blink 0.5s infinite')
                        elif countdown <= 180:  # 3 minutes - GET READY
                            self.relay_alert_container.style('background: linear-gradient(135deg, #fff3e0 0%, #ffe0b2 100%); border: 2px solid #ff9800; display: flex;')
                            self.relay_alert_label.text = f"⚠️ GET {next_drone} READY"
                            self.relay_alert_label.style('color: #e65100;')
                            self.relay_alert_icon.style('color: #e65100;')
                        elif countdown <= 300:  # 5 minutes - PREPARE
                            self.relay_alert_container.style('background: linear-gradient(135deg, #e8f5e9 0%, #c8e6c9 100%); border: 2px solid #4caf50; display: flex;')
                            self.relay_alert_label.text = f"📋 Prepare {next_drone}"
                            self.relay_alert_label.style('color: #2e7d32;')
                            self.relay_alert_icon.style('color: #2e7d32;')
                        else:
                            self.relay_alert_container.style('display: none;')
                else:
                    self.countdown_label.text = "LAUNCHING!"
                    self.countdown_label.style('color: #c62828; font-weight: bold; animation: blink 0.5s infinite; font-size: 1.5rem;')
                    if hasattr(self, 'relay_alert_container') and self.relay_alert_container:
                        self.relay_alert_container.style('background: #ffebee; border-left: 4px solid #f44336; display: block;')
                        self.relay_alert_label.text = f"LAUNCHING {next_drone}!"
                        self.relay_alert_label.style('color: #c62828; font-weight: bold; animation: blink 0.5s infinite')
                        self.relay_alert_icon.style('color: #c62828; animation: blink 0.5s infinite')
            
            if self.next_drone_label:
                suffix = ''
                try:
                    override = self.mission_controller.get_next_drone_override() if self.mission_controller else None
                    if override:
                        suffix = ' (overridden)'
                except Exception:
                    pass
                self.next_drone_label.text = f"Next: {next_drone}{suffix}"

            # Update rotation labels
            try:
                if self.rotation_order_label and self.mission_controller:
                    order = self.mission_controller.drone_order
                    if order:
                        display = " → ".join([f"[{ns}]" if ns == next_drone else ns for ns in order])
                        self.rotation_order_label.text = f"Order: {display}"
                if self.rotation_next_label:
                    self.rotation_next_label.text = f"Next: {next_drone}{(' (overridden)' if self.mission_controller and self.mission_controller.get_next_drone_override() else '')}"
            except Exception:
                pass
            
            # Show drones needing reconnection (battery swap)
            if self.reconnect_label:
                needs_reconnect = self.mission_controller.get_drones_needing_reconnection()
                if needs_reconnect:
                    self.reconnect_label.text = f"Swap battery & reconnect: {', '.join(needs_reconnect)}"
                    self.reconnect_label.style('color: #ffcc00; font-weight: bold')
                else:
                    self.reconnect_label.text = ""
        
        @self.log_event.subscribe
        def on_log(data: dict):
            message = data['message']
            timestamp = datetime.now().strftime("%H:%M:%S")
            # Queue the message for the UI timer to process
            self.log_message_queue.append(f"[{timestamp}] {message}")
    
    def _process_log_queue(self):
        """Process queued log messages (called by UI timer, runs in UI thread)."""
        # Check if mission timer should be started
        if self._should_start_timer:
            self._should_start_timer = False
            self._start_mission_timer()
        
        # Process vertical separation alerts (these need UI context)
        while self._vertical_separation_alerts:
            alert = self._vertical_separation_alerts.pop(0)
            self._show_vertical_separation_alert(alert)
        
        # Update vertical separation card
        self._update_vertical_separation_card()
        
        # Process takeoff confirmation requests (from background threads)
        if self._takeoff_confirmation_queue and not self._takeoff_confirmation_dialog_open:
            request = self._takeoff_confirmation_queue.pop(0)
            self._takeoff_confirmation_dialog_open = True
            self._process_takeoff_confirmation_sync(request)
        
        # Process queued notifications (from background threads)
        while self._notification_queue:
            notif = self._notification_queue.pop(0)
            ui.notify(
                notif.get('message', ''),
                type=notif.get('type', 'info'),
                timeout=notif.get('timeout', 3000)
            )
        
        if not self.event_log or not self.log_message_queue:
            return
        
        # Process all queued messages
        while self.log_message_queue:
            message = self.log_message_queue.pop(0)
            with self.event_log:
                ui.label(message).classes('text-sm')
        
        # Scroll to bottom
        if self.event_scroll:
            self.event_scroll.scroll_to(percent=1.0)
    
    def _show_vertical_separation_alert(self, alert: dict):
        """Show vertical separation alert in UI thread.
        
        Can handle different alert types:
        - Regular alert: drone1, drone2, separation, alt1, alt2
        - Countdown start: action='countdown_start'
        - Countdown cancel: action='countdown_cancel'
        """
        action = alert.get('action')
        
        if action == 'countdown_start':
            # Play 20-second countdown audio
            self._play_sound('20_seconds.mp3')
            self._emit_log(f"[ALERT] ⏱️ 20-SECOND COUNTDOWN STARTED - RTH will trigger if separation not restored!")
            return
        
        elif action == 'countdown_cancel':
            # Stop countdown audio and play "respected" sound
            ui.run_javascript('''
                // Stop countdown audio
                if (window.countdownAudio) {
                    window.countdownAudio.pause();
                    window.countdownAudio.currentTime = 0;
                    window.countdownAudio = null;
                }
            ''')
            self._play_sound('vertical_separation_respected.mp3')
            self._emit_log(f"[ALERT] ✅ VERTICAL SEPARATION RESTORED - Countdown cancelled!")
            return
        
        elif action == 'mission_stopped':
            # Stop countdown audio and update UI to stopped state
            ui.run_javascript('''
                // Stop countdown audio
                if (window.countdownAudio) {
                    window.countdownAudio.pause();
                    window.countdownAudio.currentTime = 0;
                    window.countdownAudio = null;
                }
            ''')
            # Update UI state same as Stop button
            self._stop_mission_timer()
            if self.mission_status_label:
                self.mission_status_label.text = "STOPPED (Vert.Sep)"
                self.mission_status_label.style('background: #ffebee; color: #c62828;')
            if self.countdown_label:
                self.countdown_label.text = "--:--"
            if self.active_drone_label:
                self.active_drone_label.text = "--"
                self.active_drone_label.style('background: #e0e0e0; color: #424242;')
            self._emit_log(f"[CRITICAL] 🛑 MISSION STOPPED - Vertical separation countdown expired!")
            return
        
        # Regular vertical separation alert
        drone1 = alert.get('drone1', 'unknown')
        drone2 = alert.get('drone2', 'unknown')
        separation = alert.get('separation', 0)
        alt1 = alert.get('alt1', 0)
        alt2 = alert.get('alt2', 0)
        
        # Play warning sound (not the countdown)
        self._play_sound('vertical_separation.mp3')
        
        # Show critical notification
        ui.notify(
            f'⚠️ CRITICAL: Vertical separation alert!\n'
            f'{drone1} ({alt1:.1f}m) and {drone2} ({alt2:.1f}m)\n'
            f'Separation: {separation:.1f}m (min 5m required)',
            type='negative',
            position='top',
            timeout=10000,
            close_button=True
        )
        
        self._emit_log(f"[ALERT] VERTICAL SEPARATION: {drone1}={alt1:.1f}m, {drone2}={alt2:.1f}m, sep={separation:.1f}m")

    def _update_vertical_separation_card(self):
        """Update the vertical separation information card."""
        if not hasattr(self, 'vertical_sep_status_badge'):
            return
        
        # Get all airborne drones with their altitudes
        airborne_drones = []
        from groundstation.mission_controller import MissionState
        
        for ns, mission in self.mission_controller.drone_missions.items():
            if mission.state not in [MissionState.IDLE, MissionState.ERROR, MissionState.COMPLETED]:
                altitude = self._get_drone_altitude(ns)
                airborne_drones.append((ns, altitude, mission.state))
        
        # Calculate minimum vertical separation between any pair
        min_separation = float('inf')
        violation_pair = None
        MIN_VERTICAL_SEPARATION = 5.0
        
        for i, (ns1, alt1, _) in enumerate(airborne_drones):
            for ns2, alt2, _ in airborne_drones[i+1:]:
                sep = abs(alt1 - alt2)
                if sep < min_separation:
                    min_separation = sep
                    violation_pair = (ns1, ns2, alt1, alt2)
        
        # Update status badge and current separation
        if len(airborne_drones) < 2:
            # Not enough drones to compare
            self.vertical_sep_status_badge.set_text('N/A')
            self.vertical_sep_status_badge.props('color=grey')
            self.vertical_sep_current_label.text = '--'
            self.vertical_sep_current_label.style('color: #9e9e9e;')
        elif min_separation < MIN_VERTICAL_SEPARATION:
            # Violation!
            self.vertical_sep_status_badge.set_text('⚠️ ALERT')
            self.vertical_sep_status_badge.props('color=red')
            self.vertical_sep_current_label.text = f'{min_separation:.1f}m'
            self.vertical_sep_current_label.style('color: #c62828;')
        elif min_separation < MIN_VERTICAL_SEPARATION * 2:
            # Warning (within 10m)
            self.vertical_sep_status_badge.set_text('CAUTION')
            self.vertical_sep_status_badge.props('color=orange')
            self.vertical_sep_current_label.text = f'{min_separation:.1f}m'
            self.vertical_sep_current_label.style('color: #e65100;')
        else:
            # OK
            self.vertical_sep_status_badge.set_text('OK')
            self.vertical_sep_status_badge.props('color=green')
            self.vertical_sep_current_label.text = f'{min_separation:.1f}m' if min_separation != float('inf') else '--'
            self.vertical_sep_current_label.style('color: #2e7d32;')
        
        # Update airborne drones list
        if hasattr(self, 'vertical_sep_drones_list'):
            self.vertical_sep_drones_list.clear()
            with self.vertical_sep_drones_list:
                if not airborne_drones:
                    ui.label("No drones airborne").classes('text-xs text-gray-400 italic')
                else:
                    for ns, alt, state in sorted(airborne_drones, key=lambda x: x[1], reverse=True):
                        # Color based on state
                        color = '#424242'
                        icon_name = 'flight'
                        if state in [MissionState.TAKING_OFF, MissionState.CLIMBING_TO_ALTITUDE]:
                            color = '#1976d2'  # Blue for climbing
                            icon_name = 'trending_up'
                        elif state == MissionState.MONITORING:
                            color = '#2e7d32'  # Green for monitoring
                            icon_name = 'location_on'
                        elif state == MissionState.RETURNING_HOME:
                            color = '#f57c00'  # Orange for RTH
                            icon_name = 'home'
                        
                        with ui.row().classes('items-center gap-1 py-0'):
                            ui.icon(icon_name, size='xs').style(f'color: {color};')
                            ui.label(f'{ns}').classes('text-xs font-bold').style(f'color: {color}; min-width: 50px;')
                            ui.label(f'{alt:.0f}m').classes('text-xs font-mono font-bold').style(f'color: {color};')
        
        # Update countdown row visibility and progress
        if self._vertical_separation_countdown_active and hasattr(self, 'vertical_sep_countdown_row'):
            self.vertical_sep_countdown_row.style('display: flex;')
            
            # Calculate countdown progress
            import time
            elapsed = time.time() - self.mission_controller._vertical_separation_countdown_start
            remaining = max(0, 20.0 - elapsed)
            progress = elapsed / 20.0
            
            self.vertical_sep_countdown_label.text = f'RTH in: {remaining:.0f}s'
            self.vertical_sep_countdown_progress.value = progress
        elif hasattr(self, 'vertical_sep_countdown_row'):
            self.vertical_sep_countdown_row.style('display: none;')

    def _build_state_machine_display(self):
        """Build the state machine visualization."""
        if not self.state_machine_container:
            return
        
        self.state_machine_container.clear()
        self.state_machine_labels.clear()
        
        # Show current mission mode indicator
        from groundstation.mission_controller import MissionMode
        is_free_flight = (hasattr(self, 'mission_controller') and 
                          self.mission_controller.mission_mode == MissionMode.FREE_FLIGHT)
        
        with self.state_machine_container:
            # Mission mode badge
            if is_free_flight:
                with ui.row().classes('w-full items-center gap-1 mb-1'):
                    ui.badge('🆓 FREE FLIGHT', color='orange').classes('text-xs')
                    ui.label('Pilot controls after altitude').classes('text-xs text-gray-500')
            else:
                with ui.row().classes('w-full items-center gap-1 mb-1'):
                    ui.badge('📍 MONITORING', color='blue').classes('text-xs')
                    ui.label('Auto-navigate to point').classes('text-xs text-gray-500')
        
        # Check if camera sync rotation is enabled
        camera_sync_enabled = (hasattr(self, 'mission_controller') and 
                               self.mission_controller.config.camera_sync_enabled)
        # Use different icon/label based on camera sync setting
        sync_icon = '360' if camera_sync_enabled else 'timer'
        sync_label = 'Sync' if camera_sync_enabled else 'Wait'
        
        # Define states based on mission mode
        if is_free_flight:
            # Free Flight: first drone goes to pilot control after climb
            # Relay drones go through transit/approach/sync before pilot control
            # Flow: First drone: IDLE→...→CLIMB→MONITORING→WAITING→RTH
            #       Relay drone: IDLE→...→CLIMB→TRANSIT→APPROACH→SYNC→MONITORING→RTH
            states = [
                ('IDLE', 'hourglass_empty', 'Idle'),
                ('SETTING_RTH_ALTITUDE', 'height', 'RTH'),
                ('TAKING_OFF', 'flight_takeoff', 'T/O'),
                ('CLIMBING_TO_ALTITUDE', 'trending_up', 'Climb'),
                ('MONITORING', 'sports_esports', 'Pilot'),     # First drone: pilot control after climb
                ('WAITING_FOR_RELAY', 'swap_horiz', 'Wait'),   # First drone waiting for relay to arrive
                ('TRANSIT_TO_MONITORING', 'flight', 'Relay'),  # Relay drone flying to first drone
                ('APPROACHING_POINT', 'gps_fixed', 'Appr'),    # Relay drone approaching for handoff
                ('CAMERA_SYNC', sync_icon, sync_label),        # Camera sync (360° rotation or wait only)
                ('RETURNING_HOME', 'home', 'RTH'),
                ('COMPLETED', 'check_circle', 'Done'),
            ]
        else:
            # Monitoring Point: full automated flow
            states = [
                ('IDLE', 'hourglass_empty', 'Idle'),
                ('SETTING_RTH_ALTITUDE', 'height', 'RTH'),
                ('TAKING_OFF', 'flight_takeoff', 'T/O'),
                ('CLIMBING_TO_ALTITUDE', 'trending_up', 'Climb'),
                ('TRANSIT_TO_MONITORING', 'flight', 'Transit'),
                ('APPROACHING_POINT', 'gps_fixed', 'Appr'),
                ('MONITORING', 'videocam', 'Mon'),
                ('WAITING_FOR_RELAY', 'swap_horiz', 'Wait'),
                ('CAMERA_SYNC', sync_icon, sync_label),        # Camera sync (360° rotation or wait only)
                ('RETURNING_HOME', 'home', 'RTH'),
                ('COMPLETED', 'check_circle', 'Done'),
            ]
        
        with self.state_machine_container:
            # Ultra-compact state legend - icons only in a single row
            with ui.row().classes('w-full justify-between gap-0'):
                for state_name, icon, label in states:
                    with ui.column().classes('items-center').style('min-width: 28px;'):
                        ui.icon(icon).style('font-size: 12px; color: #9e9e9e;')
                        ui.label(label).style('font-size: 8px; color: #9e9e9e;')
            
            # Show drones in mission from mission controller
            drones_in_mission = list(self.mission_controller.drone_missions.keys())
            if drones_in_mission:
                for namespace in drones_in_mission:
                    self._add_drone_state_row(namespace, states)
            elif self.drones:
                ui.label("Mission not started").classes('text-gray-400 italic text-xs mt-1')
            else:
                ui.label("No drones connected").classes('text-gray-400 italic text-xs mt-1')
    
    def _add_drone_state_row(self, namespace: str, states: list):
        """Add a state row for a drone."""
        mission = self.mission_controller.get_mission_status(namespace)
        current_state = mission.state.name if mission else 'IDLE'
        
        self.state_machine_labels[namespace] = {}
        
        with ui.row().classes('w-full items-center gap-0 py-1').style('border-top: 1px solid #e0e0e0;'):
            # Drone name - compact
            ui.label(namespace).classes('font-bold text-xs').style('min-width: 60px; max-width: 60px; overflow: hidden; text-overflow: ellipsis;')
            
            # State indicators - smaller and tighter
            with ui.row().classes('flex-grow justify-between gap-0'):
                for state_name, icon, label in states:
                    is_current = (current_state == state_name)
                    is_past = self._is_state_past(current_state, state_name, states)
                    is_error = current_state in ['ERROR', 'ABORTED']
                    
                    if is_current:
                        color = '#4CAF50'  # green - current
                        bg = '#e8f5e9'
                    elif is_past:
                        color = '#2196F3'  # blue - completed
                        bg = '#e3f2fd'
                    elif is_error and state_name == current_state:
                        color = '#f44336'  # red - error
                        bg = '#ffebee'
                    else:
                        color = '#bdbdbd'  # grey - not reached
                        bg = '#fafafa'
                    
                    state_icon = ui.icon(icon).style(f'font-size: 14px; color: {color}; background: {bg}; border-radius: 50%; padding: 2px;')
                    state_icon.tooltip(f"{label}: {state_name}")
                    self.state_machine_labels[namespace][state_name] = state_icon
    
    def _is_state_past(self, current: str, check: str, states: list) -> bool:
        """Check if a state has been passed."""
        state_order = [s[0] for s in states]
        try:
            current_idx = state_order.index(current)
            check_idx = state_order.index(check)
            return check_idx < current_idx
        except ValueError:
            return False
    
    def _update_state_icons(self, namespace: str):
        """Update state machine icons for a specific drone (thread-safe)."""
        if namespace not in self.state_machine_labels:
            return
        
        mission = self.mission_controller.get_mission_status(namespace)
        if not mission:
            return
        
        current_state = mission.state.name
        
        # Check if camera sync rotation is enabled
        camera_sync_enabled = (hasattr(self, 'mission_controller') and 
                               self.mission_controller.config.camera_sync_enabled)
        sync_icon = '360' if camera_sync_enabled else 'timer'
        sync_label = 'Camera Sync (360°)' if camera_sync_enabled else 'Camera Sync (wait only)'
        
        states = [
            ('IDLE', 'hourglass_empty', 'Waiting'),
            ('SETTING_RTH_ALTITUDE', 'height', 'Set RTH Alt'),
            ('TAKING_OFF', 'flight_takeoff', 'Takeoff'),
            ('CLIMBING_TO_ALTITUDE', 'trending_up', 'Climbing'),
            ('TRANSIT_TO_MONITORING', 'flight', 'Transit'),
            ('APPROACHING_POINT', 'gps_fixed', 'Approaching'),
            ('MONITORING', 'videocam', 'Monitoring'),
            ('WAITING_FOR_RELAY', 'swap_horiz', 'Waiting Relay'),
            ('CAMERA_SYNC', sync_icon, sync_label),
            ('RETURNING_HOME', 'home', 'RTH'),
            ('COMPLETED', 'check_circle', 'Done'),
        ]
        
        # Update existing icons with new colors
        for state_name, icon, label in states:
            if state_name not in self.state_machine_labels[namespace]:
                continue
            
            is_current = (current_state == state_name)
            is_past = self._is_state_past(current_state, state_name, states)
            is_error = current_state in ['ERROR', 'ABORTED']
            
            # Special case: when IDLE, reset all icons to grey (ready for new mission)
            # IDLE is special - it means drone is ready, not that it's progressing through mission
            if current_state == 'IDLE':
                if state_name == 'IDLE':
                    color = '#4CAF50'  # green - current/ready
                    bg = '#e8f5e9'
                else:
                    color = '#bdbdbd'  # grey - not started
                    bg = '#fafafa'
            elif is_current:
                color = '#4CAF50'  # green - current
                bg = '#e8f5e9'
            elif is_past:
                color = '#2196F3'  # blue - completed
                bg = '#e3f2fd'
            elif is_error:
                color = '#f44336'  # red - error
                bg = '#ffebee'
            else:
                color = '#bdbdbd'  # grey - not reached
                bg = '#fafafa'
            
            # Update the icon style
            self.state_machine_labels[namespace][state_name].style(
                f'font-size: 22px; color: {color}; background: {bg}; border-radius: 50%; padding: 4px'
            )
    
    def _build_left_panel(self):
        """Build the left panel with drone management."""
        with ui.card().classes('h-full').style('flex: 1.2; min-width: 350px; overflow-y: auto;'):
            with ui.row().classes('items-center gap-3 w-full'):
                ui.image('/static/logo.png').classes('w-16 h-16')
                ui.label("WildPerpetua").classes('text-2xl font-bold').style('flex-grow: 1')
                self.silent_toggle = ui.button(icon='volume_up', on_click=self._toggle_silent_mode).props('flat dense').tooltip('Toggle Silent Mode')
                ui.button(icon='restart_alt', on_click=self._restart_groundstation).props('flat dense color=negative').tooltip('Restart Groundstation')
            
            ui.separator()
            
            # Connection form
            with ui.expansion("Add New Drone", icon='add_circle').classes('w-full'):
                with ui.row().classes('w-full gap-2'):
                    with ui.column().classes('flex-grow'):
                        self.ip_input = ui.input(
                            label='IP Address (Optional)',
                            placeholder='Leave empty for auto-discovery',
                            value='10.142.188.57',
                            validation={'Invalid IP': lambda v: self._validate_ip(v)}
                        ).classes('w-full')
                        ui.label('🔍 Auto-discovery will scan the network for drones').classes('text-xs text-gray-500 mt-1')
                    
                    with ui.column().style('width: 140px'):
                        with ui.row().classes('items-end gap-1'):
                            self.namespace_input = ui.input(
                                label='Name',
                                placeholder='drone_1',
                                value=self._get_next_drone_name()
                            ).classes('w-full').style('flex: 1;')
                            ui.button(icon='add', on_click=self._increment_drone_name).props('flat dense size=sm').tooltip('Next drone name')
                
                with ui.row().classes('w-full gap-2 mt-2'):
                    ui.button('Auto-Discover', icon='radar', on_click=self._autodiscover_drone_ui).props('color=primary')
                    ui.button('Connect', icon='link', on_click=self._connect_drone_ui).props('color=secondary')
                    ui.button('Refresh', icon='refresh', on_click=self._refresh_drone_list).props('flat')
            
            ui.separator()
            
            # Mission Status Card - Modern readable layout
            with ui.card().classes('w-full p-4').style('background: linear-gradient(135deg, #fafafa 0%, #ffffff 100%); border: 1px solid #e0e0e0;'):
                # Header row with status badges
                with ui.row().classes('items-center gap-3 w-full pb-3').style('border-bottom: 1px solid #eeeeee;'):
                    ui.icon('analytics').classes('text-2xl').style('color: #1976d2;')
                    ui.label("Mission Status").classes('text-xl font-bold').style('color: #212121; flex-grow: 1;')
                    self.mission_status_label = ui.label("Inactive").classes('text-sm font-bold px-3 py-1 rounded-full').style('background: #eeeeee; color: #616161;')
                    self.active_drone_label = ui.label("--").classes('text-sm font-bold px-3 py-1 rounded-full').style('background: #e3f2fd; color: #1565c0;')
                
                # Main metrics row - Mission Duration & Relay Countdown
                with ui.row().classes('w-full items-stretch gap-3 mt-4'):
                    # Mission Duration Card
                    with ui.column().classes('items-center justify-center p-4 rounded-lg flex-1').style('background: linear-gradient(135deg, #e3f2fd 0%, #bbdefb 100%); min-height: 90px;'):
                        ui.label("MISSION DURATION").classes('text-xs font-bold tracking-wide').style('color: #1565c0; letter-spacing: 1px;')
                        self.mission_timer_label = ui.label("00:00:00").classes('text-3xl font-bold font-mono mt-1').style('color: #0d47a1;')
                    
                    # Relay Countdown Card
                    with ui.column().classes('items-center justify-center p-4 rounded-lg flex-1').style('background: linear-gradient(135deg, #fff3e0 0%, #ffe0b2 100%); min-height: 90px;') as self.countdown_container:
                        ui.label("NEXT RELAY").classes('text-xs font-bold tracking-wide').style('color: #e65100; letter-spacing: 1px;')
                        self.countdown_label = ui.label("--:--").classes('text-3xl font-bold font-mono mt-1').style('color: #bf360c;')
                    
                    # Force Swap Button
                    with ui.column().classes('items-center justify-center'):
                        self.force_swap_button = ui.button('SWAP', icon='swap_horiz', on_click=self._force_swap_clicked).props('color=warning unelevated').classes('px-4').style('height: 90px; font-weight: bold;').tooltip('Manually trigger relay swap')
                
                # Enhanced Progress Bar with phase indicators
                with ui.column().classes('w-full mt-4 gap-1'):
                    # Phase labels row
                    with ui.row().classes('w-full items-center justify-between px-1'):
                        ui.label("LAUNCH").classes('text-xs font-bold').style('color: #c62828; width: 20%;')
                        ui.label("CONNECT").classes('text-xs font-bold').style('color: #e65100; width: 20%; text-align: center;')
                        ui.label("READY").classes('text-xs font-bold').style('color: #f9a825; width: 20%; text-align: center;')
                        ui.label("PREPARE").classes('text-xs font-bold').style('color: #388e3c; width: 20%; text-align: center;')
                        ui.label("OK").classes('text-xs font-bold').style('color: #1976d2; width: 20%; text-align: right;')
                    
                    # Segmented progress bar container
                    with ui.row().classes('w-full items-center gap-1').style('height: 24px;'):
                        # Segment 1: 0-1 min (LAUNCH - critical)
                        self.progress_segment_1 = ui.html('<div style="width: 100%; height: 100%; border-radius: 4px; background: #ffcdd2; transition: all 0.3s;"></div>', sanitize=False).classes('flex-1').style('height: 20px; border-radius: 4px; overflow: hidden;')
                        # Segment 2: 1-3 min (CONNECT - urgent)  
                        self.progress_segment_2 = ui.html('<div style="width: 100%; height: 100%; border-radius: 4px; background: #ffe0b2; transition: all 0.3s;"></div>', sanitize=False).classes('flex-1').style('height: 20px; border-radius: 4px; overflow: hidden;')
                        # Segment 3: 3-5 min (READY - warning)
                        self.progress_segment_3 = ui.html('<div style="width: 100%; height: 100%; border-radius: 4px; background: #fff9c4; transition: all 0.3s;"></div>', sanitize=False).classes('flex-1').style('height: 20px; border-radius: 4px; overflow: hidden;')
                        # Segment 4: 5+ min (PREPARE - normal)
                        self.progress_segment_4 = ui.html('<div style="width: 100%; height: 100%; border-radius: 4px; background: #c8e6c9; transition: all 0.3s;"></div>', sanitize=False).classes('flex-1').style('height: 20px; border-radius: 4px; overflow: hidden;')
                    
                    # Time markers row
                    with ui.row().classes('w-full items-center justify-between px-1'):
                        ui.label("0:00").classes('text-xs').style('color: #9e9e9e; width: 20%;')
                        ui.label("1:00").classes('text-xs').style('color: #9e9e9e; width: 20%; text-align: center;')
                        ui.label("3:00").classes('text-xs').style('color: #9e9e9e; width: 20%; text-align: center;')
                        ui.label("5:00").classes('text-xs').style('color: #9e9e9e; width: 20%; text-align: center;')
                        ui.label("").classes('text-xs').style('color: #9e9e9e; width: 20%; text-align: right;')
                
                # Hidden original progress (for compatibility)
                self.countdown_progress = ui.linear_progress(value=0).props('instant-feedback').style('display: none;')
                
                # Timing breakdown row (hidden by default)
                with ui.row().classes('w-full gap-2 mt-3 justify-center').style('display: none;') as self.timing_breakdown_container:
                    with ui.row().classes('items-center gap-2 px-3 py-2 rounded-lg').style('background: #e3f2fd; border: 1px solid #90caf9;'):
                        ui.icon('battery_alert', size='sm').style('color: #1565c0;')
                        self.remaining_time_label = ui.label("To RTH: --").classes('text-sm font-medium').style('color: #1565c0;')
                    with ui.row().classes('items-center gap-2 px-3 py-2 rounded-lg').style('background: #fff3e0; border: 1px solid #ffcc80;'):
                        ui.icon('route', size='sm').style('color: #e65100;')
                        self.travel_time_label = ui.label("Travel: --").classes('text-sm font-medium').style('color: #e65100;')
                    with ui.row().classes('items-center gap-2 px-3 py-2 rounded-lg').style('background: #fce4ec; border: 1px solid #f48fb1;'):
                        ui.icon('security', size='sm').style('color: #c2185b;')
                        self.buffer_time_label = ui.label("Buffer: --").classes('text-sm font-medium').style('color: #c2185b;')
                
                # Relay alert (hidden by default)
                with ui.row().classes('w-full items-center gap-3 mt-3 p-3 rounded-lg').style('background: linear-gradient(135deg, #fff3e0 0%, #ffecb3 100%); border: 2px solid #ff9800; display: none;') as self.relay_alert_container:
                    self.relay_alert_icon = ui.icon('notifications_active').classes('text-2xl').style('color: #e65100;')
                    self.relay_alert_label = ui.label("").classes('font-bold text-base').style('color: #bf360c;')
                
                # Bottom info row: Battery swap and Drones needed
                with ui.row().classes('w-full gap-3 mt-4'):
                    with ui.row().classes('items-center gap-3 p-3 rounded-lg flex-1').style('background: linear-gradient(135deg, #e8f5e9 0%, #c8e6c9 100%); border: 1px solid #a5d6a7;'):
                        ui.icon('battery_charging_full').classes('text-xl').style('color: #2e7d32;')
                        with ui.column().classes('gap-0'):
                            ui.label("Battery Swap").classes('text-xs font-bold').style('color: #1b5e20;')
                            self.reconnect_label = ui.label("None").classes('text-sm font-medium').style('color: #2e7d32;')
                    
                    with ui.column().classes('p-3 rounded-lg flex-1 gap-1').style('background: linear-gradient(135deg, #e3f2fd 0%, #bbdefb 100%); border: 1px solid #90caf9;'):
                        with ui.row().classes('items-center gap-2'):
                            ui.icon('group').classes('text-lg').style('color: #1565c0;')
                            ui.label("Drones Required").classes('text-xs font-bold').style('color: #0d47a1;')
                        with ui.row().classes('w-full items-center gap-2'):
                            self.drones_needed_flying_label = ui.label("~--").classes('text-lg font-bold').style('color: #1565c0;')
                            ui.label("flying").classes('text-xs').style('color: #1976d2;')
                            ui.label("/").classes('text-sm').style('color: #90caf9;')
                            self.drones_needed_total_label = ui.label("--").classes('text-lg font-bold').style('color: #0d47a1;')
                            ui.label("total").classes('text-xs').style('color: #1976d2;')
                        with ui.row().classes('w-full items-center gap-1'):
                            self.drones_needed_info_label = ui.label("--").classes('text-xs').style('color: #42a5f5;')
                            self.drones_needed_status_icon = ui.icon('check_circle').style('font-size: 14px; color: #4caf50;')
                        self.drones_needed_ready_label = ui.label("-- ready").classes('text-xs font-medium').style('color: #2e7d32;')
            
            ui.separator()
            
            # Drone list container
            ui.label("Connected Drones").classes('text-lg font-bold')
            self.drone_list_container = ui.column().classes('w-full gap-2')
            
            # Note: _refresh_drone_list() is called after map is created in _build_ui()
    
    def _build_right_panel(self):
        """Build the right panel with map and mission control."""
        with ui.card().classes('h-full').style('flex: 3; display: flex; flex-direction: column; min-width: 0; overflow-x: hidden; overflow-y: auto;'):
            # Map container
            with ui.card().classes('w-full').style('flex: 1; min-height: 400px;'):
                self.map = ui.leaflet(
                    center=self.map_center,
                    zoom=15
                ).style('width: 100%; height: 100%;')
                
                # Map click handler
                self.map.on('map-click', self._on_map_click)
            
            # Control panels - ultra-compact single row layout
            with ui.row().classes('w-full gap-1 items-stretch mt-1'):
                # Card 1: Navigation (compact)
                with ui.card().classes('p-0').style('flex: 1.3; overflow: hidden; border-radius: 6px; box-shadow: 0 1px 3px rgba(0,0,0,0.12); display: flex; flex-direction: column;'):
                    # Header
                    with ui.row().classes('w-full items-center justify-between').style('background: linear-gradient(135deg, #e53935 0%, #c62828 100%); padding: 6px 10px; min-height: 32px;'):
                        with ui.row().classes('items-center gap-2'):
                            ui.icon('navigation').style('color: white; font-size: 16px;')
                            ui.label("Navigation").classes('text-sm font-bold').style('color: white;')
                        with ui.row().classes('items-center gap-0'):
                            ui.button(icon='push_pin', on_click=self._set_monitoring_point_manual).props('round dense size=xs flat').style('color: white; padding: 2px;').tooltip('Set')
                            ui.button(icon='delete_outline', on_click=self._clear_monitoring_point_ui).props('round dense size=xs flat').style('color: rgba(255,255,255,0.7); padding: 2px;').tooltip('Clear')
                            ui.button(icon='cancel', on_click=self._abort_trajectories).props('round dense size=xs flat').style('color: #ffab91; padding: 2px;').tooltip('Abort')
                    
                    # Content - all in minimal space
                    with ui.column().classes('w-full gap-0 justify-center').style('padding: 6px; flex: 1;'):
                        # Inputs row
                        with ui.row().classes('w-full gap-1'):
                            self.lat_input = ui.input(label='Lat', value='0.0', on_change=self._on_monitoring_coords_change).props('dense outlined').style('flex: 1;')
                            self.lon_input = ui.input(label='Lon', value='0.0', on_change=self._on_monitoring_coords_change).props('dense outlined').style('flex: 1;')
                            self.alt_input = ui.input(label='Alt', value='50').props('dense outlined').style('flex: 0.5;')
                            self.heading_input = ui.input(label='Hdg', value='0').props('dense outlined').style('flex: 0.4;')
                        # Mode + Speed row
                        with ui.row().classes('w-full items-center gap-1 mt-1'):
                            self.nav_mode_label = ui.label('PID').classes('text-xs font-bold').style('color: #1976d2; background: #e3f2fd; padding: 2px 6px; border-radius: 4px;')
                            self.nav_mode_switch = ui.switch('DJI', value=False, on_change=self._on_nav_mode_change).props('dense size=xs color=red')
                            self.trajectory_speed_slider = ui.slider(min=1, max=15, value=15, step=1, on_change=self._on_trajectory_speed_change).props('color=red').classes('flex-1')
                            self.trajectory_speed_label = ui.label('15 m/s').classes('text-xs font-bold').style('color: #e53935;')
                
                # Card 2: Vertical Separation (minimal)
                with ui.card().classes('p-0').style('flex: 0.5; overflow: hidden; border-radius: 6px; box-shadow: 0 1px 3px rgba(0,0,0,0.12); display: flex; flex-direction: column;') as self.vertical_sep_card:
                    # Header
                    with ui.row().classes('w-full items-center gap-2').style('background: linear-gradient(135deg, #ff9800 0%, #f57c00 100%); padding: 6px 10px; min-height: 32px;'):
                        ui.icon('height').style('color: white; font-size: 16px;')
                        ui.label("Vertical Sep.").classes('text-sm font-bold').style('color: white;')
                    
                    # Content
                    with ui.column().classes('w-full items-center justify-center gap-1').style('padding: 6px; flex: 1;') as self.vertical_sep_content:
                        # Toggle + Badge row
                        with ui.row().classes('w-full items-center justify-center gap-2'):
                            self.vertical_sep_enabled_switch = ui.switch(value=True, on_change=self._on_vertical_sep_toggle).props('dense size=sm color=orange')
                            self.vertical_sep_status_badge = ui.badge('N/A', color='grey').classes('text-xs')
                        # Values row
                        with ui.row().classes('items-center gap-1'):
                            self.vertical_sep_current_label = ui.label('--').classes('text-lg font-bold').style('color: #424242;')
                            ui.icon('compare_arrows').style('font-size: 14px; color: #bdbdbd;')
                            ui.label('5m').classes('text-lg font-bold').style('color: #2e7d32;')
                        self.vertical_sep_drones_list = ui.column().classes('w-full gap-0')
                        with ui.row().classes('w-full items-center gap-1 p-1 rounded').style('background: #ffebee; display: none;') as self.vertical_sep_countdown_row:
                            ui.icon('warning').style('font-size: 12px; color: #c62828;')
                            self.vertical_sep_countdown_label = ui.label('RTH: --s').classes('text-xs font-bold').style('color: #c62828;')
                            self.vertical_sep_countdown_progress = ui.linear_progress(value=0).props('instant-feedback color=red size=xs').classes('flex-1')
                    self.vertical_sep_disabled_msg = ui.label('Off').classes('w-full text-center text-xs').style('color: #9e9e9e; display: none; flex: 1; display: flex; align-items: center; justify-content: center;')
                
                # Card 3: Mission (larger)
                with ui.card().classes('p-0').style('flex: 1.4; overflow: hidden; border-radius: 6px; box-shadow: 0 1px 3px rgba(0,0,0,0.12); display: flex; flex-direction: column;'):
                    # Header
                    with ui.row().classes('w-full items-center justify-between').style('background: linear-gradient(135deg, #1976d2 0%, #1565c0 100%); padding: 6px 10px; min-height: 32px;'):
                        with ui.row().classes('items-center gap-2'):
                            ui.icon('flag').style('color: white; font-size: 16px;')
                            ui.label("Mission").classes('text-sm font-bold').style('color: white;')
                        with ui.row().classes('items-center gap-2'):
                            # ROS Bag recording toggle
                            with ui.row().classes('items-center gap-1 px-2 rounded').style('background: rgba(255,255,255,0.2);'):
                                ui.icon('fiber_manual_record').style('color: white; font-size: 14px;')
                                self.rosbag_switch = ui.switch('ROS Bag', value=False, on_change=self._on_rosbag_change).props('dense size=xs color=red dark')
                            # Camera sync toggle
                            with ui.row().classes('items-center gap-1 px-2 rounded').style('background: rgba(255,255,255,0.2);'):
                                ui.icon('videocam').style('color: white; font-size: 14px;')
                                self.camera_sync_switch = ui.switch('Sync', value=True, on_change=self._on_camera_sync_change).props('dense size=xs color=white dark')
                    
                    # Content
                    with ui.column().classes('w-full gap-1 justify-center').style('padding: 8px; flex: 1;'):
                        # Row 1: Mode toggle (bigger)
                        with ui.row().classes('w-full items-center gap-2'):
                            self.mission_mode_label = ui.label('📍 Monitor').classes('text-sm font-bold').style('color: #1976d2; background: #e3f2fd; padding: 4px 10px; border-radius: 6px; min-width: 80px; text-align: center;')
                            self.mission_mode_switch = ui.switch('Free', value=False, on_change=self._on_mission_mode_change).props('size=md color=primary')
                        # Row 2: Params (bigger inputs)
                        with ui.row().classes('w-full gap-1'):
                            self.rth_alt_input = ui.input(label='RTH Alt', value='50').props('dense outlined').style('flex: 1;')
                            self.safety_buffer_input = ui.input(label='Buffer', value='60', on_change=self._on_safety_buffer_change).props('dense outlined').style('flex: 1;')
                            self.min_battery_input = ui.input(label='Min Bat%', value='30').props('dense outlined').style('flex: 1;')
                            self.min_satellites_input = ui.input(label='Min Sat', value='8').props('dense outlined').style('flex: 0.8;')
                        # Row 3: Buttons (bigger)
                        with ui.row().classes('w-full gap-2 mt-1'):
                            ui.button('Single', icon='play_arrow', on_click=self._start_single_mission).props('color=green no-caps dense size=sm').style('flex: 1;')
                            ui.button('Relay', icon='sync', on_click=self._start_relay_mission).props('color=primary no-caps dense size=sm').style('flex: 1;')
                            ui.button('Stop', icon='stop', on_click=self._stop_mission_ui).props('color=red no-caps dense size=sm').style('flex: 1;')

                        # Row 4: Rotation view/control
                        with ui.column().classes('w-full gap-1 mt-1'):
                            with ui.row().classes('items-center gap-2'):
                                ui.icon('sync_alt').style('font-size: 16px; color: #1565c0;')
                                ui.label('Rotation').classes('text-sm font-bold').style('color: #1565c0;')
                            with ui.row().classes('w-full items-center gap-2'):
                                self.rotation_order_label = ui.label('Order: --').classes('text-xs').style('color: #1976d2;')
                                self.rotation_next_label = ui.label('Next: --').classes('text-xs font-bold').style('color: #0d47a1;')
                            with ui.row().classes('w-full items-center gap-2'):
                                options = []
                                if self.mission_controller and self.mission_controller.drone_order:
                                    options = self.mission_controller.drone_order.copy()
                                self.rotation_select = ui.select(options=options, value=(self.mission_controller.get_next_drone() if self.mission_controller else None), label='Set next').props('dense outlined').classes('flex-1')
                                ui.button('Apply', icon='check', on_click=self._apply_next_drone_override).props('color=primary no-caps dense size=sm')
                                ui.button('Reset', icon='restart_alt', on_click=self._reset_next_drone_override).props('flat no-caps dense size=sm')
                
                # Card 4: State Machine (compact)
                with ui.card().classes('p-0').style('flex: 1.5; overflow: hidden; border-radius: 6px; box-shadow: 0 1px 3px rgba(0,0,0,0.12); display: flex; flex-direction: column;'):
                    # Header
                    with ui.row().classes('w-full items-center gap-2').style('background: linear-gradient(135deg, #43a047 0%, #388e3c 100%); padding: 6px 10px; min-height: 32px;'):
                        ui.icon('account_tree').style('color: white; font-size: 16px;')
                        ui.label("State Machine").classes('text-sm font-bold').style('color: white;')
                    
                    # Content
                    self.state_machine_container = ui.column().classes('w-full gap-0 justify-center').style('padding: 6px; flex: 1;')
                    with self.state_machine_container:
                        self._build_state_machine_display()
            
            # Bottom row: Event Log and Mission Statistics side by side (or Debug Console when enabled)
            # Normal view container
            with ui.column().classes('w-full gap-2') as self.normal_logs_container:
                with ui.row().classes('w-full gap-2 items-stretch').style('min-width: 0;'):
                    # Event Log
                    with ui.card().classes('p-2').style('flex: 1; min-width: 0; overflow: hidden;'):
                        with ui.row().classes('items-center gap-2 pb-1').style('border-bottom: 1px solid #e0e0e0;'):
                            ui.icon('list_alt').classes('text-lg text-primary')
                            ui.label("Event Log").classes('text-sm font-bold')
                        with ui.scroll_area().classes('w-full').style('height: 100px;').props('id=event-log') as self.event_scroll:
                            self.event_log = ui.column().classes('w-full gap-0')
                    
                    # Mission Statistics (hidden when debug console is shown)
                    with ui.card().classes('p-2').style('flex: 1; min-width: 0; overflow: hidden;') as self.mission_stats_card:
                        with ui.row().classes('items-center gap-2 w-full pb-1').style('border-bottom: 1px solid #e0e0e0;'):
                            ui.icon('analytics').classes('text-lg text-primary')
                            ui.label("Mission Statistics").classes('text-sm font-bold')
                            ui.space()
                            ui.button(icon='delete', on_click=self._clear_mission_stats).props('flat dense size=sm').tooltip('Clear')
                        
                        # Header row
                        with ui.row().classes('w-full text-xs font-bold text-gray-500 gap-0 mt-1'):
                            ui.label("Drone").style('flex: 2;')
                            ui.label("#").style('flex: 0.8; text-align: center;')
                            ui.label("Est.").style('flex: 1.2; text-align: center;')
                            ui.label("Travel").style('flex: 1.2; text-align: center;')
                            ui.label("RTH").style('flex: 1.2; text-align: center;')
                        
                        with ui.scroll_area().classes('w-full').style('height: 80px;') as self.mission_stats_scroll:
                            self.mission_stats_container = ui.column().classes('w-full gap-0')
                    
                    # ROS Console (hidden by default, shown in place of Mission Stats)
                    with ui.card().classes('p-2').style('flex: 1; display: none;') as self.debug_console_container:
                        with ui.row().classes('items-center gap-2 w-full pb-1').style('border-bottom: 1px solid #ffcc80;'):
                            ui.icon('terminal').classes('text-lg').style('color: #e65100;')
                            ui.label("ROS Console").classes('text-sm font-bold').style('color: #e65100;')
                            ui.space()
                            ui.button(icon='delete', on_click=self._clear_debug_console).props('flat dense size=sm').tooltip('Clear console')
                        
                        with ui.scroll_area().classes('w-full').style('height: 100px; background: #1e1e1e; border-radius: 4px;') as self.debug_scroll:
                            self.debug_console = ui.column().classes('w-full gap-0 p-2')
                
                # RTH Prediction Debug Panel (expansion)
                with ui.expansion("RTH Prediction Debug", icon='bug_report').classes('w-full').style('background: #fff8e1;') as self.rth_debug_expansion:
                    with ui.card().classes('w-full p-2').style('background: #fffde7;') as self.rth_debug_container:
                        # Formula explanation
                        with ui.row().classes('w-full items-center gap-2 pb-2').style('border-bottom: 1px solid #ffe082;'):
                            ui.icon('calculate').style('color: #f57c00;')
                            ui.label("Countdown = Predicted RTH - Travel Time - Buffer").classes('text-xs font-mono').style('color: #e65100;')
                        
                        # Debug info grid
                        with ui.grid(columns=2).classes('w-full gap-x-4 gap-y-1 mt-2'):
                            # Left column - Battery & Threshold
                            ui.label("Current Battery:").classes('text-xs font-bold')
                            self.rth_debug_battery = ui.label("--").classes('text-xs font-mono')
                            
                            ui.label("Batt Needed (now):").classes('text-xs font-bold')
                            self.rth_debug_batt_needed = ui.label("--").classes('text-xs font-mono')
                            
                            ui.label("Batt Needed (MAX):").classes('text-xs font-bold').style('color: #d32f2f;')
                            self.rth_debug_batt_max = ui.label("--").classes('text-xs font-mono font-bold').style('color: #d32f2f;')
                            
                            ui.label("RTH Threshold:").classes('text-xs font-bold').tooltip('MAX BattNeeded + 2%')
                            self.rth_debug_threshold = ui.label("--").classes('text-xs font-mono')
                            
                            ui.label("Drain Rate:").classes('text-xs font-bold')
                            self.rth_debug_drain = ui.label("--").classes('text-xs font-mono')
                            
                            ui.label("Data Points:").classes('text-xs font-bold')
                            self.rth_debug_points = ui.label("--").classes('text-xs font-mono')
                            
                            ui.label("Monitoring Time:").classes('text-xs font-bold')
                            self.rth_debug_elapsed = ui.label("--").classes('text-xs font-mono')
                        
                        ui.separator().classes('my-2')
                        
                        # Comparison section
                        with ui.row().classes('w-full gap-4'):
                            with ui.column().classes('flex-1'):
                                ui.label("Predicted RTH:").classes('text-xs font-bold').style('color: #1565c0;')
                                self.rth_debug_predicted = ui.label("--").classes('text-lg font-bold font-mono').style('color: #1565c0;')
                            with ui.column().classes('flex-1'):
                                ui.label("DJI Flight Time:").classes('text-xs font-bold').style('color: #7b1fa2;')
                                self.rth_debug_dji = ui.label("--").classes('text-lg font-bold font-mono').style('color: #7b1fa2;')
                            with ui.column().classes('flex-1'):
                                ui.label("Difference:").classes('text-xs font-bold').style('color: #d32f2f;')
                                self.rth_debug_diff = ui.label("--").classes('text-lg font-bold font-mono').style('color: #d32f2f;')
                        
                        # Regression details
                        ui.separator().classes('my-2')
                        with ui.row().classes('w-full gap-2'):
                            ui.label("Regression:").classes('text-xs font-bold')
                            self.rth_debug_regression = ui.label("battery(t) = slope × t + intercept").classes('text-xs font-mono').style('color: #616161;')
                        
                        # Live regression chart
                        ui.separator().classes('my-2')
                        with ui.row().classes('w-full items-center gap-2'):
                            ui.icon('show_chart').style('color: #1565c0;')
                            ui.label("Live Battery Regression").classes('text-xs font-bold')
                        
                        # Chart using ECharts
                        self.rth_chart = ui.echart({
                            'animation': False,
                            'grid': {'left': '12%', 'right': '5%', 'top': '15%', 'bottom': '18%'},
                            'legend': {
                                'show': True,
                                'top': 0,
                                'textStyle': {'fontSize': 9},
                                'itemWidth': 12,
                                'itemHeight': 8
                            },
                            'xAxis': {
                                'type': 'value',
                                'name': 'Time (min)',
                                'nameLocation': 'middle',
                                'nameGap': 22,
                                'min': 0,
                                'axisLabel': {'fontSize': 9}
                            },
                            'yAxis': {
                                'type': 'value',
                                'name': 'Battery %',
                                'nameLocation': 'middle',
                                'nameGap': 32,
                                'min': 0,
                                'max': 100,
                                'axisLabel': {'fontSize': 9}
                            },
                            'series': [
                                {
                                    'name': 'Battery',
                                    'type': 'scatter',
                                    'data': [],
                                    'symbolSize': 5,
                                    'itemStyle': {'color': '#1976d2'}
                                },
                                {
                                    'name': 'Regression',
                                    'type': 'line',
                                    'data': [],
                                    'lineStyle': {'color': '#4caf50', 'width': 2},
                                    'symbol': 'none'
                                },
                                {
                                    'name': 'MAX Threshold',
                                    'type': 'line',
                                    'data': [],
                                    'lineStyle': {'color': '#f44336', 'width': 2, 'type': 'dashed'},
                                    'symbol': 'none'
                                },
                                {
                                    'name': 'RTH Point',
                                    'type': 'scatter',
                                    'data': [],
                                    'symbolSize': 14,
                                    'symbol': 'triangle',
                                    'itemStyle': {'color': '#f44336'}
                                },
                                {
                                    'name': 'Now',
                                    'type': 'line',
                                    'data': [],
                                    'lineStyle': {'color': '#ff9800', 'width': 2, 'type': 'dotted'},
                                    'symbol': 'none'
                                },
                                {
                                    'name': 'Current Batt',
                                    'type': 'scatter',
                                    'data': [],
                                    'symbolSize': 10,
                                    'symbol': 'circle',
                                    'itemStyle': {'color': '#ff9800', 'borderColor': '#fff', 'borderWidth': 2}
                                }
                            ],
                            'tooltip': {
                                'trigger': 'item',
                                'formatter': '{a}: {c}'
                            }
                        }).classes('w-full').style('height: 200px;')

    def _build_drone_card(self, namespace: str, drone: DroneData):
        """Build an expandable card for a single drone."""
        # Use the drone's assigned color (stored in DroneData)
        color = drone.color if hasattr(drone, 'color') and drone.color else self.drone_colors[0]
        
        with ui.card().classes('drone-card w-full').style('padding: 0; overflow: hidden; border-radius: 12px;') as card:
            self.drone_cards[namespace] = card
            self.drone_labels[namespace] = {}
            self.drone_buttons[namespace] = {}
            
            # ═══════════════════════════════════════════════════════════════
            # ALWAYS VISIBLE HEADER - Clickable to expand/collapse
            # ═══════════════════════════════════════════════════════════════
            with ui.row().classes('w-full items-center justify-between').style(f'background: linear-gradient(135deg, {color}22 0%, {color}11 100%); padding: 10px 14px; border-bottom: 2px solid {color}; cursor: pointer;') as header_row:
                # Left: Drone identifier + key stats
                with ui.row().classes('items-center gap-3'):
                    ui.element('div').style(f'width: 12px; height: 12px; border-radius: 50%; background: {color}; box-shadow: 0 0 6px {color};')
                    ui.label(f"{namespace}").classes('font-bold text-lg').style('color: #333;')
                    # Manual flight indicator
                    manual_badge = ui.badge('🎮', color='orange').tooltip('Manual control')
                    manual_badge.style('display: none')
                    self.drone_labels[namespace]['manual_indicator'] = manual_badge
                
                # Center: Quick telemetry stats (always visible)
                with ui.row().classes('items-center gap-4'):
                    # Altitude
                    with ui.row().classes('items-center gap-1'):
                        ui.icon('height').style('font-size: 18px; color: #1976d2;')
                        self.drone_labels[namespace]['altitude'] = ui.label(f"{drone.altitude:.0f}m").classes('text-sm font-semibold').style('color: #333;')
                    # Speed  
                    with ui.row().classes('items-center gap-1'):
                        ui.icon('speed').style('font-size: 18px; color: #00897b;')
                        self.drone_labels[namespace]['speed'] = ui.label(f"{drone.speed:.1f}m/s").classes('text-sm font-semibold').style('color: #333;')
                    # Satellites
                    with ui.row().classes('items-center gap-1'):
                        ui.icon('satellite_alt').style('font-size: 18px; color: #7b1fa2;')
                        self.drone_labels[namespace]['satellites'] = ui.label(f"{drone.satellite_count}").classes('text-sm font-semibold').style('color: #333;')
                
                # Right: State badge, Battery, Expand icon
                with ui.row().classes('items-center gap-3'):
                    self.drone_labels[namespace]['state'] = ui.label(f"{drone.state.value}").classes('text-xs font-semibold px-2 py-1 rounded-full').style('background: #e8e8e8; color: #555;')
                    # Battery display
                    with ui.row().classes('items-center gap-1').style('background: #f5f5f5; padding: 4px 10px; border-radius: 16px;'):
                        battery_color = '#4caf50' if drone.battery_level > 50 else '#ff9800' if drone.battery_level > 20 else '#f44336'
                        ui.icon('battery_full').style(f'font-size: 18px; color: {battery_color};')
                        self.drone_labels[namespace]['battery'] = ui.label(f"{drone.battery_level:.0f}%").classes('text-sm font-bold').style(f'color: {battery_color};')
                    # Expand indicator
                    expand_icon = ui.icon('expand_more').style('font-size: 24px; color: #666; transition: transform 0.3s;')
                    self.drone_labels[namespace]['expand_icon'] = expand_icon
            
            # ═══════════════════════════════════════════════════════════════
            # EXPANDABLE CONTENT - Hidden by default
            # ═══════════════════════════════════════════════════════════════
            with ui.column().classes('w-full gap-3').style('padding: 14px; display: none;') as content_area:
                self.drone_labels[namespace]['content_area'] = content_area
                
                # ─────────────────────────────────────────────────────────────
                # EXTENDED TELEMETRY - Flight time and Recording
                # ─────────────────────────────────────────────────────────────
                with ui.row().classes('w-full gap-2 items-stretch'):
                    # Flight Time
                    with ui.row().classes('items-center gap-2').style('flex: 1; background: #f8f9fa; padding: 10px 12px; border-radius: 8px; min-height: 44px;'):
                        ui.icon('schedule').style('font-size: 20px; color: #f57c00;')
                        ui.label('Flight Time:').classes('text-xs').style('color: #888;')
                        self.drone_labels[namespace]['flight_time'] = ui.label("0:00").classes('text-sm font-bold').style('color: #f57c00;')
                    
                    # Recording Status
                    with ui.row().classes('items-center gap-2').style(f'flex: 1; background: {"#ffebee" if drone.is_recording else "#f8f9fa"}; padding: 10px 12px; border-radius: 8px; min-height: 44px;'):
                        rec_icon = ui.icon('fiber_manual_record').style(f'font-size: 20px; color: {"#c62828" if drone.is_recording else "#bdbdbd"};{"animation: blink 1s infinite;" if drone.is_recording else ""}')
                        self.drone_labels[namespace]['recording_icon'] = rec_icon
                        ui.label('Recording:').classes('text-xs').style('color: #888;')
                        self.drone_labels[namespace]['recording'] = ui.label("REC" if drone.is_recording else "OFF").classes('text-sm font-bold').style(f'color: {"#c62828" if drone.is_recording else "#9e9e9e"};')
                
                # ─────────────────────────────────────────────────────────────
                # CAMERA CONTROLS - Gimbal and Zoom
                # ─────────────────────────────────────────────────────────────
                with ui.row().classes('w-full items-stretch gap-3'):
                    # Gimbal Control
                    with ui.column().classes('items-center justify-center').style('flex: 1; background: linear-gradient(135deg, #e3f2fd 0%, #bbdefb 100%); padding: 10px; border-radius: 8px;'):
                        gimbal_knob = ui.knob(min=-90, max=0, value=0, step=5, show_value=True).props('size="60px" thickness=0.18 color="primary" font-size="12px"').tooltip('Gimbal Pitch')
                        ui.label('Gimbal').classes('text-xs font-medium mt-1').style('color: #1565c0;')
                        
                        def update_gimbal(e, ns=namespace):
                            val = float(e.args)
                            self.send_gimbal_pitch(ns, val)
                        gimbal_knob.on('update:model-value', update_gimbal)
                    
                    # Zoom Control
                    with ui.column().classes('items-center justify-center').style('flex: 1; background: linear-gradient(135deg, #fff3e0 0%, #ffe0b2 100%); padding: 10px; border-radius: 8px;'):
                        zoom_label = ui.label('1.0x').classes('text-xl font-bold').style('color: #e65100;')
                        zoom_slider = ui.slider(min=1.0, max=2.0, value=1.0, step=0.1).props('color="orange"').style('width: 85%;').tooltip('Camera Zoom')
                        ui.label('Zoom').classes('text-xs font-medium mt-1').style('color: #e65100;')
                        
                        def update_zoom(e, ns=namespace, lbl=zoom_label):
                            val = float(e.args)
                            lbl.text = f'{val:.1f}x'
                            self.send_zoom_ratio(ns, val)
                        zoom_slider.on('update:model-value', update_zoom)
                
                # ─────────────────────────────────────────────────────────────
                # RTH PREDICTOR (hidden by default)
                # ─────────────────────────────────────────────────────────────
                with ui.row().classes('w-full items-center gap-3').style('background: linear-gradient(135deg, #e8f5e9 0%, #c8e6c9 100%); padding: 8px 12px; border-radius: 8px; display: none;') as rth_row:
                    self.drone_labels[namespace]['rth_predictor_row'] = rth_row
                    ui.icon('analytics').style('font-size: 18px; color: #2e7d32;')
                    ui.label('RTH:').classes('text-xs font-semibold').style('color: #2e7d32;')
                    with ui.row().classes('items-center gap-1'):
                        ui.icon('timer').style('font-size: 14px; color: #1565c0;')
                        self.drone_labels[namespace]['rth_predicted'] = ui.label('--:--').classes('text-xs font-bold').style('color: #1565c0;')
                    with ui.row().classes('items-center gap-1'):
                        ui.icon('trending_down').style('font-size: 14px; color: #e65100;')
                        self.drone_labels[namespace]['rth_drain_rate'] = ui.label('--%/min').classes('text-xs').style('color: #e65100;')
                    with ui.row().classes('items-center gap-1'):
                        ui.icon('show_chart').style('font-size: 14px; color: #7b1fa2;')
                        self.drone_labels[namespace]['rth_data_points'] = ui.label('0 pts').classes('text-xs').style('color: #7b1fa2;')
                
                # Hidden position label
                self.drone_labels[namespace]['position'] = ui.label().classes('hidden')
                
                # ─────────────────────────────────────────────────────────────
                # MANUAL STATE CHANGE - Debug/Override Control
                # ─────────────────────────────────────────────────────────────
                with ui.row().classes('w-full items-center gap-2').style('background: linear-gradient(135deg, #fff3e0 0%, #ffe0b2 100%); padding: 8px 12px; border-radius: 8px;'):
                    ui.icon('warning').style('font-size: 18px; color: #e65100;')
                    ui.label('Manual State:').classes('text-xs font-semibold').style('color: #bf360c;')
                    # State selector with all available states
                    state_options = [state.name for state in MissionState]
                    # Get current mission state from mission controller, fallback to IDLE
                    current_mission_state = 'IDLE'
                    if self.mission_controller:
                        mission_status = self.mission_controller.get_mission_status(namespace)
                        if mission_status:
                            current_mission_state = mission_status.state.name
                    state_select = ui.select(
                        options=state_options,
                        value=current_mission_state,
                        label='Change to'
                    ).props('dense outlined size=sm').classes('flex-1')
                    self.drone_labels[namespace]['state_select'] = state_select
                    
                    # Apply button
                    def apply_state_change(ns=namespace, select_elem=state_select):
                        new_state_name = select_elem.value
                        if not new_state_name:
                            ui.notify('No state selected', type='warning')
                            return
                        try:
                            new_state = MissionState[new_state_name]
                            success, message = self.mission_controller.set_drone_mission_state(ns, new_state)
                            if success:
                                ui.notify(message, type='positive')
                                self._emit_log(f"[{ns}] {message}")
                            else:
                                ui.notify(message, type='warning')
                        except Exception as e:
                            ui.notify(f'Error: {str(e)}', type='negative')
                    
                    ui.button('Apply', icon='check').props('flat dense size=sm color=orange').on_click(apply_state_change).tooltip('Change drone state')
                    ui.button('Reset', icon='restart_alt').props('flat dense size=sm').on_click(lambda ns=namespace, sel=state_select: sel.set_value(self.drones[ns].state.name)).tooltip('Reset to current')
                
                # ─────────────────────────────────────────────────────────────
                # VIDEO FEED - Compact inline display
                # ─────────────────────────────────────────────────────────────
                self.drone_video_visible[namespace] = False
                
                with ui.expansion('Video', icon='videocam').classes('w-full').props('dense').style('border-radius: 6px; background: #fafafa;') as video_expansion:
                    def on_expansion_toggle(e, ns=namespace):
                        is_open = e.args
                        if is_open:
                            self.drone_video_visible[ns] = True
                            self._start_webrtc_stream(ns)
                        else:
                            self.drone_video_visible[ns] = False
                            self._stop_webrtc_stream(ns)
                    
                    video_expansion.on('update:model-value', on_expansion_toggle)
                    self.drone_labels[namespace]['video_container'] = video_expansion
                    
                    # Centered video container with controls overlay
                    with ui.column().classes('w-full items-center gap-0').style('position: relative;'):
                        video_html = f'''
                        <video id="remoteVideo_{namespace}" autoplay playsinline muted 
                               style="width: 100%; max-height: 180px; object-fit: contain; border-radius: 6px; background: #1a1a1a; display: block; margin: 0 auto;"></video>
                        '''
                        ui.html(video_html, sanitize=False)
                        
                        # Overlay controls at bottom-right
                        with ui.row().classes('items-center gap-1').style('position: absolute; bottom: 8px; right: 8px;'):
                            ui.button(icon='open_in_new', on_click=lambda ns=namespace: self._open_video_fullscreen(ns)).props('flat dense round size=sm').style('background: rgba(0,0,0,0.5); color: white;').tooltip('Fullscreen')
                    
                    self.drone_labels[namespace]['video_element_id'] = f'remoteVideo_{namespace}'
                    self.drone_labels[namespace]['webrtc_pc'] = None
                    self.drone_labels[namespace]['webrtc_ws'] = None
                
                # ─────────────────────────────────────────────────────────────
                # ACTION BUTTONS
                # ─────────────────────────────────────────────────────────────
                with ui.row().classes('w-full items-center justify-between gap-2').style('flex-wrap: wrap;'):
                    # Flight Controls
                    with ui.row().classes('items-center gap-1'):
                        ui.button(icon='flight_takeoff', on_click=lambda ns=namespace: self.send_takeoff(ns)).props('flat dense round').classes('bg-blue-50').tooltip('Take Off').style('width: 36px; height: 36px;')
                        ui.button(icon='flight_land', on_click=lambda ns=namespace: self.send_land(ns)).props('flat dense round').classes('bg-blue-50').tooltip('Land').style('width: 36px; height: 36px;')
                        ui.button(icon='home', on_click=lambda ns=namespace: self.send_rth(ns)).props('flat dense round').classes('bg-amber-50').tooltip('RTH').style('width: 36px; height: 36px;')
                    
                    # Recording
                    with ui.row().classes('items-center gap-1'):
                        ui.button(icon='videocam', on_click=lambda ns=namespace: self.send_start_recording(ns)).props('flat dense round color=red').tooltip('Record').style('width: 36px; height: 36px;')
                        ui.button(icon='stop', on_click=lambda ns=namespace: self.send_stop_recording(ns)).props('flat dense round').classes('bg-gray-100').tooltip('Stop').style('width: 36px; height: 36px;')
                    
                    # Mission
                    with ui.row().classes('items-center gap-1'):
                        ui.button(icon='push_pin', on_click=lambda ns=namespace: self._pin_drone_location(ns)).props('flat dense round color=purple').tooltip('Pin').style('width: 36px; height: 36px;')
                        self.drone_buttons[namespace]['ready'] = ui.button(icon='check_circle', on_click=lambda ns=namespace: self._mark_drone_ready(ns)).props('flat dense round color=green').tooltip('Ready').style('width: 36px; height: 36px;')
                        ui.button(icon='autorenew', on_click=lambda ns=namespace: self._reconnect_drone_ui(ns)).props('flat dense round color=primary').tooltip('Reconnect drone').style('width: 36px; height: 36px;')
                    
                    # Danger
                    with ui.row().classes('items-center gap-1'):
                        ui.button(icon='warning', on_click=lambda ns=namespace: self.send_abort_mission(ns)).props('flat dense round color=negative').tooltip('Abort').style('width: 36px; height: 36px;')
                        ui.button(icon='link_off', on_click=lambda ns=namespace: self._disconnect_drone_ui(ns)).props('flat dense round color=negative').tooltip('Disconnect').style('width: 36px; height: 36px;')
            
            # Toggle expand/collapse on header click
            self.drone_labels[namespace]['is_expanded'] = False
            
            def toggle_expand(e, ns=namespace):
                content = self.drone_labels[ns]['content_area']
                icon = self.drone_labels[ns]['expand_icon']
                self.drone_labels[ns]['is_expanded'] = not self.drone_labels[ns]['is_expanded']
                if self.drone_labels[ns]['is_expanded']:
                    content.style('padding: 14px; display: flex; flex-direction: column; gap: 12px;')
                    icon.style('font-size: 24px; color: #666; transition: transform 0.3s; transform: rotate(180deg);')
                else:
                    content.style('padding: 14px; display: none;')
                    icon.style('font-size: 24px; color: #666; transition: transform 0.3s; transform: rotate(0deg);')
            
            header_row.on('click', toggle_expand)
            
            # Create arrow on map
            self._add_drone_arrow(namespace, drone.latitude, drone.longitude, drone.heading, color)
    
    def _add_drone_arrow(self, namespace: str, lat: float, lon: float, heading: float, color: str = '#FF6B6B'):
        """Add a drone arrow to the map."""
        if self.map and namespace not in self.drone_arrows:
            try:
                arrow = Arrow(
                    self.map, namespace, lat, lon, heading,
                    drones_arrows=self.drone_arrows,
                    color=color
                )
                self.drone_arrows[namespace] = arrow
                arrow._place_arrow()
            except ValueError as e:
                self.get_logger().warning(f"Could not create arrow: {e}")
    
    # ========================================================================
    # UI EVENT HANDLERS
    # ========================================================================
    
    def _get_next_drone_name(self) -> str:
        """Get the next suggested drone name based on existing drones."""
        # Find the highest drone number currently in use
        max_num = 0
        for ns in self.drones.keys():
            # Try to extract number from drone_X format
            if ns.startswith('drone_'):
                try:
                    num = int(ns.replace('drone_', ''))
                    max_num = max(max_num, num)
                except ValueError:
                    pass
        return f'drone_{max_num + 1}'
    
    def _increment_drone_name(self):
        """Increment the drone name in the input field."""
        current = self.namespace_input.value.strip() if self.namespace_input.value else ''
        
        # Try to extract and increment the number
        if current.startswith('drone_'):
            try:
                num = int(current.replace('drone_', ''))
                self.namespace_input.value = f'drone_{num + 1}'
                return
            except ValueError:
                pass
        
        # If current value doesn't match pattern, use next available
        self.namespace_input.value = self._get_next_drone_name()
    
    def _validate_ip(self, value: str) -> bool:
        """Validate IP address format. Returns True if valid."""
        if not value:
            return True  # Empty is OK (will be caught in connect)
        value = value.strip()
        parts = value.split('.')
        if len(parts) != 4:
            return False
        for part in parts:
            try:
                num = int(part)
                if num < 0 or num > 255:
                    return False
            except ValueError:
                return False
        return True
    
    def _connect_drone_ui(self):
        """Handle drone connection request from UI with optional IP."""
        ip = self.ip_input.value.strip() if self.ip_input.value else ''
        
        # Validate IP if provided
        if ip and not self._validate_ip(ip):
            ui.notify('Please enter a valid IP address', type='warning')
            return
        
        namespace = self.namespace_input.value.strip() or None
        
        # Show appropriate notification based on whether IP was provided
        if ip:
            ui.notify(f'Connecting to {ip}...', type='info')
        else:
            ui.notify('Auto-discovering drone on network...', type='info')
        
        result_ns = self.connect_drone(ip, namespace)
        if result_ns:
            ui.notify(f'Drone "{result_ns}" connected!', type='positive')
            self.ip_input.value = ''
            # Auto-suggest next drone name
            self.namespace_input.value = self._get_next_drone_name()
        else:
            if ip:
                ui.notify('Failed to connect drone', type='negative')
            else:
                ui.notify('No drone found on network. Try entering IP manually.', type='negative')
    
    def _autodiscover_drone_ui(self):
        """Auto-discover and connect drone without requiring IP.
        Clears both IP and namespace so the discovered drone name is used as namespace."""
        # Clear IP input to force autodiscovery
        self.ip_input.value = ''
        # Clear namespace so discovered drone name is used
        self.namespace_input.value = ''
        self._connect_drone_ui()
    
    def _disconnect_drone_ui(self, namespace: str):
        """Handle drone disconnection request from UI."""
        if self.disconnect_drone(namespace):
            ui.notify(f'{namespace} disconnected', type='positive')
            self._refresh_drone_list()
        else:
            ui.notify(f'Cannot disconnect {namespace} (may be in flight)', type='warning')

    def _reconnect_drone_ui(self, namespace: str):
        """Reconnect a drone using its last known IP (disconnect then connect)."""
        drone = self.drones.get(namespace)
        if not drone:
            ui.notify(f'{namespace} not found', type='warning')
            return
        ip = drone.ip_address
        if not ip:
            ui.notify(f'No stored IP for {namespace}', type='warning')
            return

        
        # Stop any ongoing WebRTC stream before reconnecting
        try:
            self._stop_webrtc_stream(namespace)
        except Exception:
            pass
        
        ui.notify(f'Reconnecting {namespace}...', type='info')

        if not self.disconnect_drone(namespace):
            ui.notify(f'Failed to disconnect {namespace}', type='negative')
            return

        # connect_drone triggers _refresh_drone_list via the drone_connected_event,
        # which destroys all drone cards (including the button that invoked us).
        # Queue the notification so it fires after the UI context is restored.
        if self.connect_drone(ip, namespace):
            self._notification_queue.append({
                'message': f'{namespace} reconnected',
                'type': 'positive',
                'timeout': 3000
            })
        else:
            self._notification_queue.append({
                'message': f'Reconnect failed for {namespace}',
                'type': 'negative',
                'timeout': 5000
            })
    
    def _start_webrtc_stream(self, namespace: str):
        """Start WebRTC video stream for a drone using native HTML5 video element."""
        if namespace not in self.drones:
            ui.notify(f'Drone {namespace} not found', type='warning')
            return
        
        drone = self.drones[namespace]
        if not drone.ip_address:
            ui.notify(f'No IP address for {namespace}', type='warning')
            return
        
        if namespace not in self.drone_labels or 'video_element_id' not in self.drone_labels[namespace]:
            ui.notify(f'Video element not found for {namespace}', type='warning')
            return
        
        ws_url = f"ws://{drone.ip_address}:8082"
        video_element_id = self.drone_labels[namespace]['video_element_id']
        
        # Use ui.run_javascript to execute the WebRTC connection
        ui.run_javascript(f'''
        (async function connectStream_{namespace}() {{
            const namespace = "{namespace}";
            const wsUrl = "{ws_url}";
            const remoteVideo = document.getElementById("{video_element_id}");
            
            function addDebug(msg) {{
                console.log("[WebRTC " + namespace + "] " + msg);
            }}
            
            if (!remoteVideo) {{
                addDebug("Video element not found");
                return;
            }}
            
            addDebug("Starting connection to " + wsUrl);
            
            // Create WebSocket connection for signaling
            const ws = new WebSocket(wsUrl);
            let pc = null;
            
            ws.onopen = async () => {{
                addDebug("WebSocket connected");
                
                // Setup WebRTC
                const config = {{
                    iceServers: [{{ urls: "stun:stun.l.google.com:19302" }}]
                }};
                
                pc = new RTCPeerConnection(config);
                addDebug("RTCPeerConnection created");
                
                pc.onicecandidate = (event) => {{
                    if (event.candidate && ws && ws.readyState === WebSocket.OPEN) {{
                        ws.send(JSON.stringify(event.candidate));
                        addDebug("Sent ICE candidate");
                    }}
                }};
                
                pc.ontrack = (event) => {{
                    addDebug("Track received: " + event.track.kind + ", readyState: " + event.track.readyState);
                    
                    // Add video element event listeners for debugging
                    remoteVideo.onloadedmetadata = () => addDebug("Video: loadedmetadata, size: " + remoteVideo.videoWidth + "x" + remoteVideo.videoHeight);
                    remoteVideo.onloadeddata = () => addDebug("Video: loadeddata");
                    remoteVideo.oncanplay = () => addDebug("Video: canplay");
                    remoteVideo.onplaying = () => addDebug("Video: playing");
                    remoteVideo.onstalled = () => addDebug("Video: stalled");
                    remoteVideo.onwaiting = () => addDebug("Video: waiting");
                    remoteVideo.onerror = (e) => addDebug("Video error: " + (remoteVideo.error ? remoteVideo.error.message : e));
                    
                    // Monitor track state
                    event.track.onmute = () => addDebug("Track muted");
                    event.track.onunmute = () => addDebug("Track unmuted");
                    event.track.onended = () => addDebug("Track ended");
                    
                    if (event.streams && event.streams[0]) {{
                        addDebug("Stream has " + event.streams[0].getTracks().length + " tracks, stream active: " + event.streams[0].active);
                        remoteVideo.srcObject = event.streams[0];
                        addDebug("Set srcObject, video.srcObject active: " + (remoteVideo.srcObject ? remoteVideo.srcObject.active : "null"));
                        
                        // Try to play the video
                        remoteVideo.play().then(() => {{
                            addDebug("Video playback started");
                        }}).catch(e => {{
                            addDebug("Play error: " + e.message);
                            // Try playing muted (autoplay policy)
                            remoteVideo.muted = true;
                            remoteVideo.play().then(() => {{
                                addDebug("Video playing (muted)");
                            }}).catch(e2 => addDebug("Play error 2: " + e2.message));
                        }});
                    }} else if (event.track) {{
                        // Fallback: create a new MediaStream from the track
                        addDebug("Using track directly (no stream)");
                        let stream = remoteVideo.srcObject;
                        if (!stream) {{
                            stream = new MediaStream();
                            remoteVideo.srcObject = stream;
                        }}
                        stream.addTrack(event.track);
                        remoteVideo.play().catch(e => addDebug("Play error: " + e.message));
                    }}
                }};
                
                pc.onconnectionstatechange = () => {{
                    addDebug("Connection state: " + pc.connectionState);
                }};
                
                pc.oniceconnectionstatechange = () => {{
                    addDebug("ICE connection state: " + pc.iceConnectionState);
                }};
                
                addDebug("Waiting for server offer...");
                
                // Store for cleanup
                window["webrtc_" + namespace] = {{ pc: pc, ws: ws }};
            }};
            
            ws.onmessage = async (event) => {{
                const message = JSON.parse(event.data);
                addDebug("Received: " + message.type);
                
                if (message.type === "offer") {{
                    addDebug("Processing offer...");
                    await pc.setRemoteDescription(new RTCSessionDescription(message));
                    const answer = await pc.createAnswer();
                    await pc.setLocalDescription(answer);
                    ws.send(JSON.stringify(pc.localDescription));
                    addDebug("Sent answer");
                }} else if (message.type === "answer") {{
                    addDebug("Processing answer...");
                    await pc.setRemoteDescription(new RTCSessionDescription(message));
                }} else if (message.candidate !== undefined) {{
                    // Handle ICE candidates
                    if (message.candidate === null || message.candidate === "") {{
                        addDebug("Received end-of-candidates signal");
                        try {{
                            await pc.addIceCandidate(null);
                        }} catch (e) {{}}
                    }} else {{
                        try {{
                            // Server may send candidates without sdpMid/sdpMLineIndex
                            // Default to sdpMid='0' and sdpMLineIndex=0 for video
                            const candidateInit = {{
                                candidate: message.candidate,
                                sdpMid: message.sdpMid !== undefined ? message.sdpMid : "0",
                                sdpMLineIndex: message.sdpMLineIndex !== undefined ? message.sdpMLineIndex : 0
                            }};
                            await pc.addIceCandidate(new RTCIceCandidate(candidateInit));
                            addDebug("Added ICE candidate");
                        }} catch (e) {{
                            addDebug("ICE error: " + e.message);
                        }}
                    }}
                }} else if (message.type === "welcome") {{
                    addDebug("Server welcome received");
                }} else {{
                    addDebug("Unknown message: " + JSON.stringify(message).substring(0, 100));
                }}
            }};
            
            ws.onerror = (error) => {{
                addDebug("WebSocket error: " + error);
            }};
            
            ws.onclose = () => {{
                addDebug("WebSocket closed");
            }};
        }})();
        ''')
        
        self._emit_log(f"[VIDEO] Starting WebRTC stream for {namespace} at {ws_url}")
    
    def _stop_webrtc_stream(self, namespace: str):
        """Stop video stream for a drone."""
        video_element_id = f'remoteVideo_{namespace}'
        
        ui.run_javascript(f'''
        (function() {{
            try {{
                const conn = window["webrtc_{namespace}"];
                if (conn) {{
                    if (conn.ws) conn.ws.close();
                    if (conn.pc) conn.pc.close();
                    window["webrtc_{namespace}"] = null;
                }}
                
                // Clear video element
                const video = document.getElementById("{video_element_id}");
                if (video) {{
                    video.srcObject = null;
                }}
                
                console.log("[WebRTC] Stream stopped for {namespace}");
            }} catch (error) {{
                console.error("[WebRTC] Error stopping stream: " + error);
            }}
        }})();
        ''')
        
        self._emit_log(f"[VIDEO] Stopped stream for {namespace}")
    
    def _open_video_fullscreen(self, namespace: str):
        """Open WebRTC video stream in a new fullscreen tab."""
        if namespace not in self.drones:
            ui.notify(f'Drone {namespace} not found', type='warning')
            return
        
        drone = self.drones[namespace]
        if not drone.ip_address:
            ui.notify(f'No IP address for {namespace}', type='warning')
            return
        
        # Open fullscreen video page in new tab
        ui.navigate.to(f'/video/{namespace}', new_tab=True)
    
    def _pin_drone_location(self, namespace: str):
        """Pin drone's current location as monitoring point and switch to Monitoring Point mode."""
        if not self.mission_controller:
            ui.notify('No mission controller active', type='warning')
            return
        
        if self.mission_controller.pin_drone_location(namespace):
            # Update mission mode toggle in UI
            if hasattr(self, 'mission_mode_toggle') and self.mission_mode_toggle:
                self.mission_mode_toggle.value = 1  # Switch to "Monitor" mode
            
            # Update monitoring point on map
            lat, lon, alt = 0.0, 0.0, 0.0
            if namespace in self.drones:
                lat = self.drones[namespace].latitude
                lon = self.drones[namespace].longitude
                alt = self.drones[namespace].altitude
                heading = self.drones[namespace].heading
                self.set_monitoring_point(lat, lon, alt, heading, source=f"pinned:{namespace}")
            
            ui.notify(f'📍 Location pinned from {namespace}! Mode switched to Monitoring Point.', type='positive')
            self._emit_log(f"[PIN] Location pinned from {namespace}: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
        else:
            ui.notify(f'Failed to pin location from {namespace}', type='warning')
    
    def _mark_drone_ready(self, namespace: str):
        """Mark a drone as ready (IDLE) for next relay cycle."""
        if not self.mission_controller:
            ui.notify('No mission controller active', type='warning')
            return
        
        if self.mission_controller.mark_drone_ready(namespace):
            ui.notify(f'{namespace} marked as ready', type='positive')
            # Emit state update to refresh GUI
            self.drone_state_update.emit({
                'namespace': namespace,
                'state': DroneState.IDLE
            })
        else:
            ui.notify(f'{namespace} cannot be marked ready (not in COMPLETED state)', type='warning')
    
    # ========================================================================
    # TRAJECTORY TRACKING
    # ========================================================================
    
    def _start_drone_trajectory(self, namespace: str):
        """Start tracking a new trajectory for a drone (called on takeoff)."""
        # Clear any existing trajectory
        self.drone_trajectories[namespace] = []
        
        # Remove existing polyline if any
        if namespace in self.drone_trajectory_lines and self.drone_trajectory_lines[namespace]:
            try:
                self.map.remove_layer(self.drone_trajectory_lines[namespace])
            except:
                pass
            self.drone_trajectory_lines[namespace] = None
        
        self._emit_log(f"[{namespace}] Trajectory tracking started")
    
    def _update_drone_trajectory(self, namespace: str, lat: float, lon: float):
        """Add a point to the drone's trajectory and update the map."""
        if namespace not in self.drone_trajectories:
            self.drone_trajectories[namespace] = []
        
        trajectory = self.drone_trajectories[namespace]
        
        # Only add point if it's far enough from the last one (at least 2 meters)
        # This reduces lag by limiting the number of points
        MIN_DISTANCE_METERS = 2.0
        MAX_POINTS = 500  # Limit total trajectory points
        
        should_add = False
        if not trajectory:
            should_add = True
        else:
            last_lat, last_lon = trajectory[-1]
            # Quick distance check (approximate, good enough for filtering)
            lat_diff = abs(lat - last_lat) * 111320  # meters per degree lat
            lon_diff = abs(lon - last_lon) * 111320 * abs(math.cos(math.radians(lat)))
            distance = math.sqrt(lat_diff**2 + lon_diff**2)
            should_add = distance >= MIN_DISTANCE_METERS
        
        if should_add:
            trajectory.append((lat, lon))
            
            # Trim old points if too many
            if len(trajectory) > MAX_POINTS:
                trajectory[:] = trajectory[-MAX_POINTS:]
            
            # Update polyline on map (need at least 2 points)
            if len(trajectory) >= 2 and self.map:
                # Get drone color
                color = '#FF6B6B'  # default
                if namespace in self.drone_arrows:
                    color = self.drone_arrows[namespace].color
                
                # Remove old polyline
                if namespace in self.drone_trajectory_lines and self.drone_trajectory_lines[namespace]:
                    try:
                        self.map.remove_layer(self.drone_trajectory_lines[namespace])
                    except:
                        pass
                
                # Create new polyline with all points
                self.drone_trajectory_lines[namespace] = self.map.generic_layer(
                    name='polyline',
                    args=[
                        [[p[0], p[1]] for p in trajectory],
                        {'color': color, 'weight': 3, 'opacity': 0.8}
                    ]
                )
    
    def _fade_and_clear_trajectory(self, namespace: str):
        """Fade out and clear a drone's trajectory (called on landing)."""
        if namespace not in self.drone_trajectory_lines or not self.drone_trajectory_lines[namespace]:
            # No trajectory to clear
            if namespace in self.drone_trajectories:
                self.drone_trajectories[namespace] = []
            return
        
        self._emit_log(f"[{namespace}] Trajectory tracking stopped - fading out")
        
        # Fade out over 3 seconds using opacity steps
        polyline = self.drone_trajectory_lines[namespace]
        trajectory = self.drone_trajectories.get(namespace, [])
        
        if not trajectory or len(trajectory) < 2:
            # Nothing to fade, just clear
            try:
                self.map.remove_layer(polyline)
            except:
                pass
            self.drone_trajectory_lines[namespace] = None
            self.drone_trajectories[namespace] = []
            return
        
        # Get drone color
        color = '#FF6B6B'
        if namespace in self.drone_arrows:
            color = self.drone_arrows[namespace].color
        
        # Create fade-out animation using timers
        def fade_step(opacity: float):
            if namespace not in self.drone_trajectory_lines:
                return
            
            try:
                # Remove old polyline
                if self.drone_trajectory_lines[namespace]:
                    self.map.remove_layer(self.drone_trajectory_lines[namespace])
                
                if opacity > 0:
                    # Create new polyline with reduced opacity
                    self.drone_trajectory_lines[namespace] = self.map.generic_layer(
                        name='polyline',
                        args=[
                            [[p[0], p[1]] for p in trajectory],
                            {'color': color, 'weight': 3, 'opacity': opacity}
                        ]
                    )
                else:
                    # Final step - clear everything
                    self.drone_trajectory_lines[namespace] = None
                    self.drone_trajectories[namespace] = []
            except:
                pass
        
        # Schedule fade steps (3 seconds total, 6 steps)
        ui.timer(0.5, lambda: fade_step(0.6), once=True)
        ui.timer(1.0, lambda: fade_step(0.4), once=True)
        ui.timer(1.5, lambda: fade_step(0.3), once=True)
        ui.timer(2.0, lambda: fade_step(0.2), once=True)
        ui.timer(2.5, lambda: fade_step(0.1), once=True)
        ui.timer(3.0, lambda: fade_step(0.0), once=True)
    
    def _refresh_drone_list(self):
        """Refresh the drone list display."""
        if self.drone_list_container is None:
            return
        
        # Clear the tracking dictionaries for cards being removed
        self.drone_cards.clear()
        self.drone_labels.clear()
        self.drone_buttons.clear()
        # Note: Don't clear drone_arrows here - they're on the map
            
        self.drone_list_container.clear()
        
        if not self.drones:
            with self.drone_list_container:
                ui.label("No drones connected").classes('text-gray-500 italic')
        else:
            for namespace, drone in self.drones.items():
                with self.drone_list_container:
                    self._build_drone_card(namespace, drone)
        
        # Update drones needed estimate
        self._update_drones_needed()
    
    def _update_drones_needed(self):
        """Update the estimate of drones needed for continuous coverage."""
        if self.drones_needed_flying_label and self.monitoring_point.is_set:
            result = self.mission_controller.calculate_drones_needed()
            simultaneous, total, travel_time, distance, flight_time, has_actual_data = result
            
            # Format travel time
            travel_min = int(travel_time // 60)
            travel_sec = int(travel_time % 60)
            distance_km = distance / 1000
            # Format flight time
            flight_min = int(flight_time // 60)
            flight_sec = int(flight_time % 60)
            
            connected = len(self.drones)
            
            # Check if we have a valid distance (not fallback 3km/5min)
            is_fallback = (distance == 3000 and travel_time == 300)
            
            # Indicator for estimate source
            source_prefix = "" if has_actual_data else "~"  # ~ = estimated
            
            if is_fallback:
                # Waiting for GPS data
                self.drones_needed_flying_label.set_text("--")
                self.drones_needed_total_label.set_text("--")
                self.drones_needed_info_label.set_text(f"Waiting for GPS...")
                self.drones_needed_info_label.style('color: #90caf9;')
                self.drones_needed_status_icon.props('name=hourglass_empty')
                self.drones_needed_status_icon.style('font-size: 14px; color: #90caf9;')
                self.drones_needed_ready_label.set_text(f"{connected} connected")
                self.drones_needed_ready_label.style('color: #1976d2;')
            elif simultaneous == float('inf'):
                # Point too far
                self.drones_needed_flying_label.set_text("∞")
                self.drones_needed_total_label.set_text("∞")
                self.drones_needed_info_label.set_text(f"Too far! {distance_km:.1f}km")
                self.drones_needed_info_label.style('color: #c62828;')
                self.drones_needed_status_icon.props('name=error')
                self.drones_needed_status_icon.style('font-size: 14px; color: #c62828;')
                self.drones_needed_ready_label.set_text(f"{travel_min}:{travel_sec:02d} travel")
                self.drones_needed_ready_label.style('color: #ef5350;')
            else:
                # Normal display
                self.drones_needed_flying_label.set_text(f"{source_prefix}{simultaneous}")
                self.drones_needed_total_label.set_text(f"{total}")
                self.drones_needed_info_label.set_text(f"{distance_km:.1f}km, {travel_min}min, {flight_min}min flight")
                
                if connected >= total:
                    # All good - enough drones
                    self.drones_needed_info_label.style('color: #42a5f5;')
                    self.drones_needed_status_icon.props('name=check_circle')
                    self.drones_needed_status_icon.style('font-size: 14px; color: #4caf50;')
                    self.drones_needed_ready_label.set_text(f"{connected} ready")
                    self.drones_needed_ready_label.style('color: #2e7d32;')
                elif connected >= simultaneous:
                    # Warning - minimum for flying, not enough for full rotation
                    self.drones_needed_info_label.style('color: #42a5f5;')
                    self.drones_needed_status_icon.props('name=warning')
                    self.drones_needed_status_icon.style('font-size: 14px; color: #ff9800;')
                    self.drones_needed_ready_label.set_text(f"{connected}/{total} ready")
                    self.drones_needed_ready_label.style('color: #ef6c00;')
                else:
                    # Error - not enough drones
                    self.drones_needed_info_label.style('color: #42a5f5;')
                    self.drones_needed_status_icon.props('name=cancel')
                    self.drones_needed_status_icon.style('font-size: 14px; color: #c62828;')
                    self.drones_needed_ready_label.set_text(f"{connected}/{simultaneous}+ needed")
                    self.drones_needed_ready_label.style('color: #c62828;')
    
    def _on_safety_buffer_change(self, e):
        """Handle safety buffer input change - update config and recalculate drones needed."""
        try:
            buffer = float(self.safety_buffer_input.value)
            if buffer >= 0:
                self.mission_controller.config.safety_buffer_seconds = buffer
                self._update_drones_needed()
        except ValueError:
            pass  # Invalid input, ignore
    
    def _on_map_click(self, e):
        """Handle map click for setting monitoring point."""
        try:
            if 'latlng' in e.args:
                lat = e.args['latlng']['lat']
                lon = e.args['latlng']['lng']
            elif 'lat' in e.args:
                lat = e.args['lat']
                lon = e.args['lng']
            else:
                return
        except (KeyError, TypeError):
            return
        
        try:
            alt = float(self.alt_input.value)
        except ValueError:
            alt = 50.0
        
        try:
            heading = float(self.heading_input.value) if self.heading_input else 0.0
            heading = heading % 360
        except ValueError:
            heading = 0.0
        
        self.set_monitoring_point(lat, lon, alt, heading, source="map")
        
        self.lat_input.value = f"{lat:.6f}"
        self.lon_input.value = f"{lon:.6f}"
        
        ui.notify(f'Monitoring point set', type='positive')
        self._update_drones_needed()
    
    def _set_monitoring_point_manual(self):
        """Set monitoring point from manual input."""
        try:
            lat = float(self.lat_input.value)
            lon = float(self.lon_input.value)
            alt = float(self.alt_input.value)
            heading = float(self.heading_input.value) if self.heading_input else 0.0
            
            # Normalize heading to 0-360
            heading = heading % 360
            
            self.set_monitoring_point(lat, lon, alt, heading, source="manual")
            ui.notify(f'Monitoring point set (heading={heading:.0f}°)', type='positive')
            self._update_drones_needed()
        except ValueError:
            ui.notify('Invalid coordinates', type='warning')
    
    def _on_monitoring_coords_change(self, e):
        """Update waypoint marker live when typing lat/lon in the inputs.

        This provides immediate visual feedback without committing the point
        to mission state. It simply refreshes the marker and circle on the map
        if both coordinates parse correctly.
        """
        try:
            lat = float(self.lat_input.value)
            lon = float(self.lon_input.value)
        except (TypeError, ValueError):
            # Ignore while typing incomplete/invalid values
            return

        if not self.map:
            return

        # Recreate marker and circle at the new coordinates
        try:
            if self.monitoring_marker:
                self.map.remove_layer(self.monitoring_marker)
            if self.monitoring_circle:
                self.map.remove_layer(self.monitoring_circle)

            self.monitoring_marker = self.map.marker(latlng=[lat, lon])
            self.monitoring_circle = self.map.generic_layer(
                name='circle',
                args=[[lat, lon], {'radius': 50, 'color': 'green', 'fillOpacity': 0.2, 'weight': 2}]
            )
        except Exception:
            # Be resilient to transient UI/map states
            pass

    def _clear_monitoring_point_ui(self):
        """Clear the monitoring point from UI."""
        self.clear_monitoring_point()
        self.lat_input.value = '0.0'
        self.lon_input.value = '0.0'
        ui.notify('Monitoring point cleared', type='info')

    def _apply_next_drone_override(self):
        """Apply user-selected next drone override."""
        if not self.mission_controller:
            ui.notify('Mission controller not ready', type='warning')
            return
        selection = self.rotation_select.value if self.rotation_select else None
        success, message = self.mission_controller.set_next_drone_override(selection)
        if success:
            ui.notify(message, type='positive')
            self._emit_log(f"[ROTATION] {message}")
        else:
            ui.notify(message, type='warning')
        # Refresh select and labels
        self._refresh_rotation_ui()

    def _reset_next_drone_override(self):
        """Clear any override for next drone."""
        if not self.mission_controller:
            return
        self.mission_controller.set_next_drone_override(None)
        ui.notify('Next-drone override cleared', type='info')
        self._emit_log("[ROTATION] Override cleared")
        self._refresh_rotation_ui()

    def _refresh_rotation_ui(self):
        """Refresh rotation order text and select options."""
        if not self.mission_controller:
            return
        try:
            order = self.mission_controller.drone_order
            if self.rotation_order_label:
                if order:
                    next_ns = self.mission_controller.get_next_drone()
                    display = " → ".join([f"[{ns}]" if ns == next_ns else ns for ns in order])
                    self.rotation_order_label.text = f"Order: {display}"
                else:
                    self.rotation_order_label.text = "Order: --"
            if self.rotation_next_label:
                next_ns = self.mission_controller.get_next_drone()
                suffix = ' (overridden)' if self.mission_controller.get_next_drone_override() else ''
                self.rotation_next_label.text = f"Next: {next_ns if next_ns else '--'}{suffix}"
            if self.rotation_select is not None:
                self.rotation_select.options = order if order else []
                self.rotation_select.value = self.mission_controller.get_next_drone()
        except Exception:
            pass
    
    def _start_single_mission(self):
        """Start a single drone monitoring mission."""
        from groundstation.mission_controller import MissionMode
        
        # Check if monitoring point is required (not in Free Flight mode)
        is_free_flight = (hasattr(self, 'mission_controller') and 
                          self.mission_controller.mission_mode == MissionMode.FREE_FLIGHT)
        
        if not self.monitoring_point.is_set and not is_free_flight:
            ui.notify('Please set a monitoring point first (or use Free Flight mode)', type='warning')
            return
        
        if not self.drones:
            ui.notify('No drones connected', type='warning')
            return
        
        drone_ns = list(self.drones.keys())[0]
        
        # Show takeoff confirmation dialog using run_javascript to stay in UI context
        self._pending_takeoff_drone = drone_ns
        self._pending_single_mission = True
        self._show_takeoff_dialog_sync(drone_ns)
    
    def _do_start_single_mission(self, drone_ns: str):
        """Actually start the single mission after confirmation."""
        try:
            rth_alt = float(self.rth_alt_input.value)
        except ValueError:
            rth_alt = 50.0
        
        try:
            min_battery = float(self.min_battery_input.value)
            self.mission_controller.config.min_battery_to_launch = min_battery
        except ValueError:
            pass
        
        try:
            min_sats = int(self.min_satellites_input.value)
            self.mission_controller.config.min_satellites = min_sats
        except ValueError:
            pass
        
        if self.start_monitoring_mission(drone_ns, rth_alt):
            # Reset mission timer (will start when drone reaches monitoring point)
            self._stop_mission_timer()
            self._mission_start_time = None
            if self.mission_timer_label:
                self.mission_timer_label.text = "00:00:00"
            
            # Disable camera sync switch during mission (can only be changed before start)
            if hasattr(self, 'camera_sync_switch') and self.camera_sync_switch:
                self.camera_sync_switch.disable()
            
            # Disable navigation mode switch during mission
            if hasattr(self, 'nav_mode_switch') and self.nav_mode_switch:
                self.nav_mode_switch.disable()
            
            # Build state machine display for drones in mission
            self._build_state_machine_display()
            
            self.mission_status_label.text = "Single Drone"
            self.mission_status_label.style('background: #e8f5e9; color: #2e7d32;')  # green
            self.active_drone_label.text = drone_ns
            self.active_drone_label.style('background: #e8f5e9; color: #2e7d32;')
            ui.notify(f'Mission started', type='positive')
            self._emit_log(f"Single mission started with {drone_ns}")
        else:
            ui.notify('Failed to start mission', type='negative')
    
    def _start_relay_mission(self):
        """Start a relay mission with all connected drones."""
        from groundstation.mission_controller import MissionMode
        
        # Check if monitoring point is required (not in Free Flight mode)
        is_free_flight = (hasattr(self, 'mission_controller') and 
                          self.mission_controller.mission_mode == MissionMode.FREE_FLIGHT)
        
        if not self.monitoring_point.is_set and not is_free_flight:
            ui.notify('Please set a monitoring point first (or use Free Flight mode)', type='warning')
            return
        
        if len(self.drones) < 1:
            ui.notify('No drones connected', type='warning')
            return
        
        # Check if point is reachable (skip in Free Flight mode)
        if not is_free_flight:
            result = self.mission_controller.calculate_drones_needed()
            simultaneous, total, travel_time, distance, flight_time, has_actual_data = result
            connected = len(self.drones)
            
            if simultaneous == float('inf'):
                ui.notify(f'Point too far! ({distance/1000:.1f}km) - cannot maintain coverage', type='negative')
                return
            
            # Info message about drone requirements (non-blocking)
            if connected < simultaneous:
                ui.notify(
                    f'Need {simultaneous} drones flying simultaneously. Connect more drones soon!',
                    type='warning',
                    timeout=5000
                )
            elif connected < total:
                ui.notify(
                    f'Starting with {connected} drones. {total} recommended for full rotation.',
                    type='info',
                    timeout=3000
                )
            
            travel_time_val = travel_time
            distance_val = distance
        else:
            # Free Flight mode - no distance calculation needed
            travel_time_val = 0
            distance_val = 0
        
        drone_list = list(self.drones.keys())
        first_drone = drone_list[0]
        
        # Store pending relay mission data and show confirmation dialog
        self._pending_takeoff_drone = first_drone
        self._pending_relay_mission = True
        self._pending_relay_data = {'drone_list': drone_list, 'travel_time': travel_time_val, 'distance': distance_val}
        self._show_takeoff_dialog_sync(first_drone)
    
    def _do_start_relay_mission(self, drone_list: list, travel_time: float, distance: float):
        """Actually start the relay mission after confirmation."""
        try:
            rth_alt = float(self.rth_alt_input.value)
        except ValueError:
            rth_alt = 50.0
        
        try:
            buffer = float(self.safety_buffer_input.value)
            self.mission_controller.config.safety_buffer_seconds = buffer
        except ValueError:
            pass
        
        try:
            min_battery = float(self.min_battery_input.value)
            self.mission_controller.config.min_battery_to_launch = min_battery
        except ValueError:
            pass
        
        try:
            min_sats = int(self.min_satellites_input.value)
            self.mission_controller.config.min_satellites = min_sats
        except ValueError:
            pass
        
        if self.start_relay_mission(drone_list, rth_alt):
            # Reset mission timer (will start when drone reaches monitoring point)
            self._stop_mission_timer()
            self._mission_start_time = None
            if self.mission_timer_label:
                self.mission_timer_label.text = "00:00:00"
            
            # Disable camera sync switch during mission (can only be changed before start)
            if hasattr(self, 'camera_sync_switch') and self.camera_sync_switch:
                self.camera_sync_switch.disable()
            
            # Disable navigation mode switch during mission
            if hasattr(self, 'nav_mode_switch') and self.nav_mode_switch:
                self.nav_mode_switch.disable()
            
            # Build state machine display for drones in mission
            self._build_state_machine_display()
            
            travel_min = int(travel_time // 60)
            self.mission_status_label.text = f"Relay ({len(drone_list)})"
            self.mission_status_label.style('background: #e3f2fd; color: #1565c0;')  # blue
            self.active_drone_label.text = drone_list[0]
            self.active_drone_label.style('background: #e3f2fd; color: #1565c0;')
            ui.notify(f'Relay mission started with {len(drone_list)} drones (~{travel_min}min to point)', type='positive')
            self._emit_log(f"Relay mission started: {', '.join(drone_list)} - {distance/1000:.1f}km to point")
        else:
            ui.notify('Failed to start relay mission', type='negative')
    
    def _force_swap_clicked(self):
        """Handle Force Swap button click - manually trigger relay swap."""
        if not self.mission_controller:
            ui.notify('Mission controller not available', type='warning')
            return
        
        # Check if a swap is already in progress
        if self.mission_controller.is_manual_swap_active():
            ui.notify('Swap already in progress', type='warning')
            return
        
        # Get the next drone to take off
        next_drone = self.mission_controller.get_next_drone()
        if not next_drone:
            ui.notify('No next drone available for swap', type='warning')
            return
        
        # Show takeoff confirmation dialog
        self._show_force_swap_dialog(next_drone)
    
    def _show_force_swap_dialog(self, drone_name: str):
        """Show takeoff confirmation dialog for force swap."""
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label(f'🚁 Force Swap - Takeoff Confirmation').classes('text-xl font-bold text-orange-700')
            ui.separator()
            ui.label(f'Drone "{drone_name}" will take off for relay swap.').classes('text-lg mt-2')
            ui.label('This will immediately launch the next drone in sequence.').classes('text-sm text-gray-600 mt-1')
            
            def on_confirm():
                dialog.close()
                # Play takeoff confirmation sound
                self._play_sound('take_off.mp3')
                # Execute the force swap
                self._do_force_swap()
            
            def on_cancel():
                dialog.close()
                ui.notify('Force swap cancelled', type='info')
                self._emit_log("[SWAP] Force swap cancelled by user")
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=on_cancel, color='red').props('flat')
                ui.button('Confirm Takeoff', on_click=on_confirm, color='primary')
        
        dialog.open()
    
    def _do_force_swap(self):
        """Execute the force swap after confirmation."""
        # Try to force the swap
        success, message = self.mission_controller.force_relay_swap()
        
        if success:
            self._manual_swap_active = True
            ui.notify(message, type='positive')
            self._emit_log(f"[SWAP] {message}")
            
            # Update force swap button to show swapping status (countdown continues to update)
            if self.force_swap_button:
                self.force_swap_button.props('disabled')
                self.force_swap_button.text = 'Swapping...'
        else:
            ui.notify(message, type='warning')
            self._emit_log(f"[SWAP] Failed: {message}")
    
    def _start_mission_timer(self):
        """Start the mission elapsed time timer."""
        if self._mission_timer_task is None:
            self._mission_timer_task = ui.timer(1.0, self._update_mission_timer)
    
    def _stop_mission_timer(self):
        """Stop the mission elapsed time timer."""
        if self._mission_timer_task is not None:
            self._mission_timer_task.cancel()
            self._mission_timer_task = None
    
    def _update_mission_timer(self):
        """Update the mission timer display."""
        if self._mission_start_time is not None and self.mission_timer_label:
            elapsed = time.time() - self._mission_start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)
            seconds = int(elapsed % 60)
            self.mission_timer_label.text = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
    
    def _stop_mission_ui(self):
        """Stop the current mission from UI."""
        self.stop_mission()
        self._stop_mission_timer()
        # Keep the final time displayed, just stop updating
        self.mission_status_label.text = "Stopped"
        self.mission_status_label.style('background: #ffebee; color: #c62828;')  # red
        self.countdown_label.text = "--:--"
        self.active_drone_label.text = "--"
        self.active_drone_label.style('background: #e0e0e0; color: #424242;')
        
        # Re-enable camera sync switch (can be changed again for next mission)
        if hasattr(self, 'camera_sync_switch') and self.camera_sync_switch:
            self.camera_sync_switch.enable()
        
        # Re-enable navigation mode switch (can be changed again for next mission)
        if hasattr(self, 'nav_mode_switch') and self.nav_mode_switch:
            self.nav_mode_switch.enable()
        
        ui.notify('Mission stopped', type='info')
        self._emit_log("Mission stopped - drones returning home")
    
    def _on_nav_mode_change(self, e):
        """Handle navigation mode toggle change."""
        use_dji_native = e.value
        self.mission_controller.use_dji_native = use_dji_native
        
        # Update label based on mode (slider range stays 1-15 for both)
        if use_dji_native:
            # DJI Native mode
            if self.nav_mode_label:
                self.nav_mode_label.text = 'DJI'
                self.nav_mode_label.style('color: #e65100; min-width: 25px;')  # Orange for DJI
            # Set default speed for DJI Native
            if self.trajectory_speed_slider:
                self.trajectory_speed_slider.set_value(15)
            if self.trajectory_speed_label:
                self.trajectory_speed_label.text = '15 m/s'
            self.DJI_NATIVE_SPEED = 15.0
            self.mission_controller.dji_native_speed = 15.0  # Sync with mission controller
            ui.notify('✈️ DJI Native (smoother trajectory)', type='positive')
            self._emit_log("[CONFIG] Navigation: DJI NATIVE @ 15 m/s")
        else:
            # PID mode
            if self.nav_mode_label:
                self.nav_mode_label.text = 'PID'
                self.nav_mode_label.style('color: #1976d2; min-width: 25px;')  # Blue for PID
            # Set default speed for PID
            if self.trajectory_speed_slider:
                self.trajectory_speed_slider.set_value(15)
            if self.trajectory_speed_label:
                self.trajectory_speed_label.text = '15 m/s'
            self.PID_SPEED = 15.0
            ui.notify('✈️ PID Control (yaw during transit)', type='info')
            self._emit_log("[CONFIG] Navigation: PID @ 15 m/s")
    
    def _on_camera_sync_change(self, e):
        """Handle camera sync toggle change."""
        enabled = e.value
        self.mission_controller.config.camera_sync_enabled = enabled
        
        if enabled:
            ui.notify('🔄 Camera sync enabled (360° rotation during handoff)', type='positive')
            self._emit_log("[CONFIG] Camera sync rotation ENABLED")
        else:
            ui.notify('🔄 Camera sync disabled (10s waits only, no rotation)', type='info')
            self._emit_log("[CONFIG] Camera sync rotation DISABLED - 10s waits only")
        
        # Refresh state machine display to show updated icon
        self._build_state_machine_display()
    
    def _on_rosbag_change(self, e):
        """Handle ROS bag recording toggle change."""
        import subprocess
        import os
        from datetime import datetime
        
        enabled = e.value
        
        if enabled:
            # Start recording
            try:
                # Create rosbags directory if it doesn't exist
                os.makedirs(self._rosbag_dir, exist_ok=True)
                
                # Generate bag name with timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                bag_name = f"wildperpetua_{timestamp}"
                bag_path = os.path.join(self._rosbag_dir, bag_name)
                
                # Start ros2 bag record in background
                self._rosbag_process = subprocess.Popen(
                    ['ros2', 'bag', 'record', '-a', '-o', bag_path],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                self._rosbag_recording = True
                
                ui.notify(f'🎥 Recording started: {bag_name}', type='positive', timeout=5000)
                self._emit_log(f"[ROSBAG] Recording started - saving to src/rosbags/{bag_name}")
                
            except Exception as ex:
                ui.notify(f'Failed to start recording: {ex}', type='negative')
                self._emit_log(f"[ROSBAG] Error: {ex}")
                if hasattr(self, 'rosbag_switch') and self.rosbag_switch:
                    self.rosbag_switch.value = False
                self._rosbag_recording = False
        else:
            # Stop recording
            if self._rosbag_process:
                try:
                    self._rosbag_process.terminate()
                    self._rosbag_process.wait(timeout=5)
                    self._rosbag_process = None
                    self._rosbag_recording = False
                    
                    ui.notify('🎥 Recording stopped', type='info')
                    self._emit_log("[ROSBAG] Recording stopped")
                    
                except Exception as ex:
                    ui.notify(f'Error stopping recording: {ex}', type='warning')
                    self._emit_log(f"[ROSBAG] Error stopping: {ex}")
    
    def _on_vertical_sep_toggle(self, e):
        """Handle vertical separation check toggle change."""
        enabled = e.value
        self.mission_controller.vertical_separation_enabled = enabled
        
        if enabled:
            ui.notify('✅ Vertical separation check enabled', type='positive')
            self._emit_log("[SAFETY] Vertical separation check ENABLED")
            self.vertical_sep_status_badge.set_text('OK')
            self.vertical_sep_status_badge.props('color=green')
            # Show content, hide disabled message
            self.vertical_sep_content.style('display: block;')
            self.vertical_sep_disabled_msg.style('display: none;')
        else:
            ui.notify('⚠️ Vertical separation check disabled', type='warning')
            self._emit_log("[SAFETY] Vertical separation check DISABLED - manual monitoring required!")
            self.vertical_sep_status_badge.set_text('OFF')
            self.vertical_sep_status_badge.props('color=grey')
            # Hide content, show disabled message
            self.vertical_sep_content.style('display: none;')
            self.vertical_sep_disabled_msg.style('display: block;')
    
    def _on_trajectory_mode_change(self, e):
        """Handle trajectory mode toggle change (legacy - use nav_mode_switch instead)."""
        # This is kept for backward compatibility - use nav_mode_switch for new code
        if hasattr(self, 'mission_controller'):
            # Value True = DJI Native, False = PID
            use_dji = e.value if hasattr(e, 'value') else False
            self.mission_controller.use_dji_native = use_dji
        
        speed = self.trajectory_speed_slider.value if self.trajectory_speed_slider else 10
        mode_str = "DJI Native" if self.mission_controller.use_dji_native else "PID"
        ui.notify(f'Navigation mode: {mode_str} ({speed} m/s)', type='info')
        self._emit_log(f"[CONFIG] Navigation mode set to {mode_str} ({speed} m/s)")
    
    def _on_trajectory_speed_change(self, e):
        """Handle trajectory speed slider change."""
        speed = e.value
        
        # Update label
        if self.trajectory_speed_label:
            self.trajectory_speed_label.set_text(f'{speed} m/s')
        
        # Update speed for the active navigation mode
        if hasattr(self, 'mission_controller') and self.mission_controller.use_dji_native:
            self.DJI_NATIVE_SPEED = float(speed)
            self.mission_controller.dji_native_speed = float(speed)  # Sync with mission controller
            self._emit_log(f"[CONFIG] DJI Native speed: {speed} m/s")
        else:
            self.PID_SPEED = float(speed)
            self._emit_log(f"[CONFIG] PID speed: {speed} m/s")
    
    def _on_mission_mode_change(self, e):
        """Handle mission mode toggle change (Monitoring Point vs Free Flight)."""
        from groundstation.mission_controller import MissionMode
        
        # Switch: False = Monitoring Point, True = Free Flight
        if not e.value:
            mode = MissionMode.MONITORING_POINT
            mode_name = "📍 Monitoring Point"
            mode_desc = "Drone flies to monitoring point and hovers"
            self.mission_mode_label.set_text('📍 Monitor')
            self.mission_mode_label.style('color: #1976d2; background: #e3f2fd; padding: 4px 10px; border-radius: 6px; min-width: 80px; text-align: center;')
        else:
            mode = MissionMode.FREE_FLIGHT
            mode_name = "🆓 Free Flight"
            mode_desc = "Pilot controls drone after reaching altitude"
            self.mission_mode_label.set_text('🆓 Free')
            self.mission_mode_label.style('color: #7b1fa2; background: #f3e5f5; padding: 4px 10px; border-radius: 6px; min-width: 80px; text-align: center;')
        
        # Update mission controller's mode
        if hasattr(self, 'mission_controller'):
            self.mission_controller.mission_mode = mode
        
        # Update state machine display to show new mode
        self._build_state_machine_display()
        
        ui.notify(f'{mode_name}: {mode_desc}', type='info')
        self._emit_log(f"[CONFIG] Mission mode set to {mode_name}")
    
    def _abort_trajectories(self):
        """Abort all trajectories."""
        for namespace in self.drones.keys():
            self.send_abort_mission(namespace)
        ui.notify('All trajectories aborted', type='info')
        self._emit_log("[ABORT] All trajectories aborted")
    
    def _update_countdown_segments(self, countdown: float):
        """Update the segmented countdown progress bar based on current countdown value.
        
        Segments represent urgency phases:
        - Segment 4 (green): 5+ min - PREPARE phase
        - Segment 3 (yellow): 3-5 min - READY phase  
        - Segment 2 (orange): 1-3 min - CONNECT phase
        - Segment 1 (red): 0-1 min - LAUNCH phase (critical)
        """
        if not hasattr(self, 'progress_segment_1'):
            return
        
        # Define phase boundaries (in seconds)
        PHASE_1_END = 60    # 0-1 min: LAUNCH (critical)
        PHASE_2_END = 180   # 1-3 min: CONNECT (urgent)
        PHASE_3_END = 300   # 3-5 min: READY (warning)
        # Phase 4: 5+ min: PREPARE (normal)
        
        # Calculate fill for each segment
        # Segment 1: 0-60s (fills from right to left as countdown decreases)
        if countdown <= PHASE_1_END:
            seg1_fill = countdown / PHASE_1_END
            seg1_color = '#f44336'  # Bright red - active critical
            seg1_bg = f'linear-gradient(to right, #f44336 {seg1_fill*100}%, #ffcdd2 {seg1_fill*100}%)'
        else:
            seg1_fill = 1.0
            seg1_color = '#ef9a9a'  # Light red - not yet reached
            seg1_bg = '#ef9a9a'
        
        # Segment 2: 60-180s
        if countdown <= PHASE_1_END:
            seg2_fill = 0
            seg2_bg = '#ffcc80'  # Depleted
        elif countdown <= PHASE_2_END:
            seg2_fill = (countdown - PHASE_1_END) / (PHASE_2_END - PHASE_1_END)
            seg2_bg = f'linear-gradient(to right, #ff9800 {seg2_fill*100}%, #ffe0b2 {seg2_fill*100}%)'
        else:
            seg2_fill = 1.0
            seg2_bg = '#ffcc80'  # Not yet reached
        
        # Segment 3: 180-300s
        if countdown <= PHASE_2_END:
            seg3_fill = 0
            seg3_bg = '#fff59d'  # Depleted
        elif countdown <= PHASE_3_END:
            seg3_fill = (countdown - PHASE_2_END) / (PHASE_3_END - PHASE_2_END)
            seg3_bg = f'linear-gradient(to right, #fbc02d {seg3_fill*100}%, #fff9c4 {seg3_fill*100}%)'
        else:
            seg3_fill = 1.0
            seg3_bg = '#fff59d'  # Not yet reached
        
        # Segment 4: 300s+ (always full when countdown > 300, otherwise proportional)
        if countdown <= PHASE_3_END:
            seg4_fill = 0
            seg4_bg = '#a5d6a7'  # Depleted
        else:
            # Cap at 600s (10 min) for visualization
            max_display = 600
            seg4_fill = min(1.0, (countdown - PHASE_3_END) / (max_display - PHASE_3_END))
            seg4_bg = f'linear-gradient(to right, #4caf50 {seg4_fill*100}%, #c8e6c9 {seg4_fill*100}%)'
        
        # Apply styles to segments
        try:
            self.progress_segment_1._props['innerHTML'] = f'<div style="width: 100%; height: 100%; border-radius: 4px; background: {seg1_bg}; transition: all 0.3s;"></div>'
            self.progress_segment_1.update()
            
            self.progress_segment_2._props['innerHTML'] = f'<div style="width: 100%; height: 100%; border-radius: 4px; background: {seg2_bg}; transition: all 0.3s;"></div>'
            self.progress_segment_2.update()
            
            self.progress_segment_3._props['innerHTML'] = f'<div style="width: 100%; height: 100%; border-radius: 4px; background: {seg3_bg}; transition: all 0.3s;"></div>'
            self.progress_segment_3.update()
            
            self.progress_segment_4._props['innerHTML'] = f'<div style="width: 100%; height: 100%; border-radius: 4px; background: {seg4_bg}; transition: all 0.3s;"></div>'
            self.progress_segment_4.update()
        except Exception:
            pass  # Ignore errors during UI updates
    
    def _update_rth_debug_panel(self, namespace: str, debug_info: dict):
        """Update the RTH prediction debug panel with current values."""
        # Current battery
        current_batt = debug_info.get('current_battery', 0)
        if hasattr(self, 'rth_debug_battery'):
            self.rth_debug_battery.text = f"{current_batt:.1f}%"
        
        # Battery needed to go home (current value)
        batt_needed = debug_info.get('batt_needed_to_go_home', 0)
        if hasattr(self, 'rth_debug_batt_needed'):
            self.rth_debug_batt_needed.text = f"{batt_needed:.1f}%"
        
        # Battery needed to go home (MAX value - used for prediction)
        max_batt_needed = debug_info.get('max_batt_needed_to_go_home', 0)
        if hasattr(self, 'rth_debug_batt_max'):
            self.rth_debug_batt_max.text = f"{max_batt_needed:.1f}%"
        
        # RTH threshold (MAX batt_needed + 2%)
        threshold = debug_info.get('rth_threshold', 0)
        if hasattr(self, 'rth_debug_threshold'):
            self.rth_debug_threshold.text = f"{threshold:.1f}%"
            # Highlight if close to threshold
            if current_batt > 0 and (current_batt - threshold) < 10:
                self.rth_debug_threshold.style('color: #d32f2f; font-weight: bold;')
            else:
                self.rth_debug_threshold.style('color: inherit;')
        
        # Drain rate
        drain_rate = debug_info.get('drain_rate_per_min', 0)
        if hasattr(self, 'rth_debug_drain'):
            self.rth_debug_drain.text = f"{drain_rate:.2f}%/min"
        
        # Data points
        data_points = debug_info.get('data_points', 0)
        if hasattr(self, 'rth_debug_points'):
            if data_points < 3:
                self.rth_debug_points.text = f"{data_points}/3 (collecting)"
                self.rth_debug_points.style('color: #1976d2; font-weight: bold;')
            else:
                self.rth_debug_points.text = f"{data_points}"
                self.rth_debug_points.style('color: inherit; font-weight: normal;')
        
        # Elapsed monitoring time
        elapsed = debug_info.get('elapsed_since_monitoring', 0)
        if hasattr(self, 'rth_debug_elapsed'):
            mins = int(elapsed // 60)
            secs = int(elapsed % 60)
            self.rth_debug_elapsed.text = f"{mins}:{secs:02d}"
        
        # Predicted RTH time
        predicted = debug_info.get('predicted_rth_seconds', float('inf'))
        if hasattr(self, 'rth_debug_predicted'):
            if predicted != float('inf') and predicted > 0:
                mins = int(predicted // 60)
                secs = int(predicted % 60)
                self.rth_debug_predicted.text = f"{mins}:{secs:02d}"
            else:
                self.rth_debug_predicted.text = "--:--"
        
        # DJI remaining flight time
        dji_time = debug_info.get('dji_remaining_flight_time', 0)
        if hasattr(self, 'rth_debug_dji'):
            mins = int(dji_time // 60)
            secs = int(dji_time % 60)
            self.rth_debug_dji.text = f"{mins}:{secs:02d}"
        
        # Difference (predicted - DJI)
        if hasattr(self, 'rth_debug_diff'):
            if predicted != float('inf') and predicted > 0 and dji_time > 0:
                diff = predicted - dji_time
                mins = int(abs(diff) // 60)
                secs = int(abs(diff) % 60)
                sign = "+" if diff >= 0 else "-"
                self.rth_debug_diff.text = f"{sign}{mins}:{secs:02d}"
                # Color based on sign
                if diff < 0:
                    self.rth_debug_diff.style('color: #d32f2f;')  # Red - prediction is shorter
                else:
                    self.rth_debug_diff.style('color: #2e7d32;')  # Green - prediction is longer
            else:
                self.rth_debug_diff.text = "--:--"
        
        # Regression equation
        slope = debug_info.get('slope', 0)
        intercept = debug_info.get('intercept', 0)
        if hasattr(self, 'rth_debug_regression'):
            if slope != 0:
                self.rth_debug_regression.text = f"batt(t) = {slope:.4f}×t + {intercept:.2f}"
            else:
                self.rth_debug_regression.text = "Not enough data"
        
        # Update live chart
        if hasattr(self, 'rth_chart') and self.rth_chart:
            battery_points = debug_info.get('chart_battery_points', [])
            regression_line = debug_info.get('chart_regression_line', [])
            threshold_line = debug_info.get('chart_threshold_line', [])
            rth_point = debug_info.get('chart_rth_point', [])
            current_time_line = debug_info.get('chart_current_time_line', [])
            current_point = debug_info.get('chart_current_point', [])
            
            # Calculate x-axis max (add some padding)
            if battery_points:
                max_time = max(p[0] for p in battery_points)
                if rth_point:
                    max_time = max(max_time, rth_point[0][0])
                x_max = max(5, max_time + 2)  # At least 5 min, plus 2 min padding
            else:
                x_max = 10
            
            # Update chart options
            self.rth_chart.options['xAxis']['max'] = x_max
            self.rth_chart.options['series'][0]['data'] = battery_points  # Battery scatter (blue dots)
            self.rth_chart.options['series'][1]['data'] = regression_line  # Regression line (green)
            self.rth_chart.options['series'][2]['data'] = threshold_line  # MAX RTH threshold (red dashed)
            self.rth_chart.options['series'][3]['data'] = rth_point  # RTH crossing point (red triangle)
            self.rth_chart.options['series'][4]['data'] = current_time_line  # Current time vertical (orange dotted)
            self.rth_chart.options['series'][5]['data'] = current_point  # Current position (orange circle)
            self.rth_chart.update()
    
    def _clear_mission_stats(self):
        """Clear mission statistics history."""
        self.mission_stats_history.clear()
        self.drone_iteration_counter.clear()
        self.drone_rth_tracking.clear()
        self._refresh_mission_stats_display()
        ui.notify('Mission statistics cleared', type='info')
    
    def _toggle_silent_mode(self):
        """Toggle silent mode (mute all sounds)."""
        self.silent_mode = not self.silent_mode
        
        if self.silent_mode:
            if self.silent_toggle:
                self.silent_toggle.props('icon=volume_off color=negative')
            ui.notify('🔇 Silent mode enabled', type='info')
            self._emit_log('[AUDIO] Silent mode enabled - all sounds muted')
        else:
            if self.silent_toggle:
                self.silent_toggle.props('icon=volume_up')
            ui.notify('🔊 Silent mode disabled', type='info')
            self._emit_log('[AUDIO] Silent mode disabled - sounds enabled')
    
    def _play_sound(self, filename: str):
        """Play a sound file if not in silent mode.
        
        Args:
            filename: Path to sound file relative to /static/ (e.g., 'take_off.mp3')
        """
        if not self.silent_mode:
            ui.run_javascript(f'''
                var audio = new Audio("/static/{filename}");
                audio.play().catch(function(e) {{ console.log("Audio play failed:", e); }});
            ''')
    
    def _toggle_debug_mode(self):
        """Toggle ROS console on/off."""
        self.debug_mode = not self.debug_mode
        
        if self.debug_mode:
            # Hide Mission Statistics, show ROS console
            if self.mission_stats_card:
                self.mission_stats_card.style('display: none;')
            if self.debug_console_container:
                self.debug_console_container.style('display: block; flex: 1;')
            if self.debug_toggle:
                self.debug_toggle.props('color=orange')
            ui.notify('ROS Console enabled', type='warning')
            
            # Set up logging handler to capture output
            self._setup_debug_logging()
            self._add_debug_log('ROS Console enabled - capturing logs', 'INFO')
        else:
            # Show Mission Statistics, hide ROS console
            if self.mission_stats_card:
                self.mission_stats_card.style('display: block; flex: 1;')
            if self.debug_console_container:
                self.debug_console_container.style('display: none;')
            if self.debug_toggle:
                self.debug_toggle.props('color=')
            ui.notify('ROS Console disabled', type='info')
            
            # Remove logging handler
            self._remove_debug_logging()
    
    def _setup_debug_logging(self):
        """Set up logging handlers to capture all console output."""
        import logging
        import sys
        import os
        import io
        import threading
        
        # Create a custom handler that adds to our debug console
        class DebugUIHandler(logging.Handler):
            def __init__(self, gui_instance):
                super().__init__()
                self.gui = gui_instance
            
            def emit(self, record):
                try:
                    msg = self.format(record)
                    self.gui._add_debug_log(msg, record.levelname)
                except Exception:
                    pass
        
        # Store handler reference for later removal
        self._debug_handler = DebugUIHandler(self)
        self._debug_handler.setFormatter(logging.Formatter('%(asctime)s [%(levelname)s] %(name)s: %(message)s', datefmt='%H:%M:%S'))
        self._debug_handler.setLevel(logging.DEBUG)  # Capture all levels
        
        # Add to root logger to capture all logs
        root_logger = logging.getLogger()
        root_logger.addHandler(self._debug_handler)
        # Ensure root logger level allows debug messages through
        if root_logger.level > logging.DEBUG or root_logger.level == 0:
            self._original_root_level = root_logger.level
            root_logger.setLevel(logging.DEBUG)
        
        # Capture stdout/stderr at the file descriptor level to catch ROS2/rclpy output
        # Save original file descriptors
        self._original_stdout_fd = os.dup(1)
        self._original_stderr_fd = os.dup(2)
        
        # Create pipes
        self._stdout_read_fd, self._stdout_write_fd = os.pipe()
        self._stderr_read_fd, self._stderr_write_fd = os.pipe()
        
        # Redirect stdout/stderr to our pipes
        os.dup2(self._stdout_write_fd, 1)
        os.dup2(self._stderr_write_fd, 2)
        
        # Also update Python's sys.stdout/stderr to use the new fd
        sys.stdout = io.TextIOWrapper(os.fdopen(self._stdout_write_fd, 'wb', 0), write_through=True)
        sys.stderr = io.TextIOWrapper(os.fdopen(self._stderr_write_fd, 'wb', 0), write_through=True)
        
        # Start reader threads
        self._stop_capture = False
        
        def read_output(read_fd, original_fd, default_level):
            reader = os.fdopen(read_fd, 'r')
            while not self._stop_capture:
                try:
                    line = reader.readline()
                    if line:
                        # Write to original output
                        os.write(original_fd, line.encode())
                        
                        # Determine level from content
                        line_stripped = line.strip()
                        if not line_stripped:
                            continue
                        
                        level = default_level
                        line_lower = line_stripped.lower()
                        if '[error]' in line_lower or 'error' in line_lower or 'exception' in line_lower:
                            level = 'ERROR'
                        elif '[warn]' in line_lower or 'warning' in line_lower:
                            level = 'WARNING'
                        elif '[info]' in line_lower:
                            level = 'INFO'
                        elif '[debug]' in line_lower:
                            level = 'DEBUG'
                        
                        self._add_debug_log(line_stripped, level)
                except Exception:
                    break
        
        self._stdout_thread = threading.Thread(target=read_output, args=(self._stdout_read_fd, self._original_stdout_fd, 'INFO'), daemon=True)
        self._stderr_thread = threading.Thread(target=read_output, args=(self._stderr_read_fd, self._original_stderr_fd, 'ERROR'), daemon=True)
        self._stdout_thread.start()
        self._stderr_thread.start()
    
    def _remove_debug_logging(self):
        """Remove the debug logging handler and restore stdout/stderr."""
        import logging
        import sys
        import os
        
        # Stop capture threads
        self._stop_capture = True
        
        if hasattr(self, '_debug_handler'):
            logging.getLogger().removeHandler(self._debug_handler)
            
        # Restore original log level if we changed it
        if hasattr(self, '_original_root_level'):
            logging.getLogger().setLevel(self._original_root_level)
            del self._original_root_level
        
        # Restore original file descriptors
        if hasattr(self, '_original_stdout_fd'):
            os.dup2(self._original_stdout_fd, 1)
            os.close(self._original_stdout_fd)
        if hasattr(self, '_original_stderr_fd'):
            os.dup2(self._original_stderr_fd, 2)
            os.close(self._original_stderr_fd)
        
        # Restore Python stdout/stderr
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
    
    def _add_debug_log(self, message: str, level: str = 'INFO'):
        """Add a message to the debug console."""
        if not self.debug_mode or not self.debug_console:
            return
        
        # Color based on level
        level_colors = {
            'DEBUG': '#9e9e9e',    # grey
            'INFO': '#4fc3f7',     # light blue
            'WARNING': '#ffb74d',  # orange
            'ERROR': '#ef5350',    # red
            'CRITICAL': '#f44336', # bright red
        }
        color = level_colors.get(level, '#ffffff')
        
        # Add to console (limit to last 200 lines)
        with self.debug_console:
            ui.label(message).classes('text-xs font-mono').style(f'color: {color}; white-space: pre-wrap; word-break: break-all;')
        
        # Remove old entries if too many
        if len(self.debug_console.default_slot.children) > 200:
            self.debug_console.default_slot.children[0].delete()
        
        # Scroll to bottom
        if hasattr(self, 'debug_scroll'):
            self.debug_scroll.scroll_to(percent=1.0)
    
    def _clear_debug_console(self):
        """Clear the debug console."""
        if self.debug_console:
            self.debug_console.clear()
        ui.notify('Console cleared', type='info')

    def _format_time(self, seconds: float) -> str:
        """Format seconds as MM:SS."""
        if seconds <= 0:
            return "--:--"
        minutes = int(seconds // 60)
        secs = int(seconds % 60)
        return f"{minutes}:{secs:02d}"
    
    def _add_mission_stat(self, drone: str, iteration: int, est_travel: float, actual_travel: float = 0.0, actual_rth: float = 0.0):
        """Add or update a mission statistic entry."""
        # Check if entry exists for this drone and iteration
        for stat in self.mission_stats_history:
            if stat['drone'] == drone and stat['iteration'] == iteration:
                # Update existing entry
                if actual_travel > 0:
                    stat['actual_travel'] = actual_travel
                if actual_rth > 0:
                    stat['actual_rth'] = actual_rth
                self._refresh_mission_stats_display()
                return
        
        # Add new entry
        self.mission_stats_history.append({
            'drone': drone,
            'iteration': iteration,
            'est_travel': est_travel,
            'actual_travel': actual_travel,
            'actual_rth': actual_rth
        })
        self._refresh_mission_stats_display()
    
    def _update_mission_stat_rth(self, drone: str, actual_rth: float):
        """Update RTH time for the most recent mission of a drone."""
        # Find the most recent entry for this drone
        for stat in reversed(self.mission_stats_history):
            if stat['drone'] == drone and stat['actual_rth'] == 0:
                stat['actual_rth'] = actual_rth
                self._refresh_mission_stats_display()
                return
    
    def _refresh_mission_stats_display(self):
        """Refresh the mission statistics display."""
        if not self.mission_stats_container:
            return
        
        self.mission_stats_container.clear()
        
        if not self.mission_stats_history:
            with self.mission_stats_container:
                ui.label("No mission data yet").classes('text-gray-500 italic text-sm')
            return
        
        with self.mission_stats_container:
            for stat in self.mission_stats_history:
                with ui.row().classes('w-full text-base gap-0 px-1 py-1').style('border-bottom: 1px solid #eee'):
                    ui.label(stat['drone']).style('flex: 2; min-width: 60px; overflow: hidden; text-overflow: ellipsis')
                    ui.label(str(stat['iteration'])).style('flex: 0.8; text-align: center; min-width: 30px')
                    ui.label(self._format_time(stat['est_travel'])).style('flex: 1.2; text-align: center; min-width: 50px; color: #666')
                    
                    # Actual travel - color based on comparison with estimate
                    travel_text = self._format_time(stat['actual_travel'])
                    if stat['actual_travel'] > 0:
                        diff = stat['actual_travel'] - stat['est_travel']
                        if diff > 30:  # More than 30s slower
                            travel_color = '#c62828'  # red
                        elif diff < -10:  # More than 10s faster
                            travel_color = '#2e7d32'  # green
                        else:
                            travel_color = '#1565c0'  # blue
                    else:
                        travel_color = '#999'
                    ui.label(travel_text).style(f'flex: 1.2; text-align: center; min-width: 50px; color: {travel_color}; font-weight: bold')
                    
                    # RTH time
                    rth_text = self._format_time(stat['actual_rth'])
                    rth_color = '#1565c0' if stat['actual_rth'] > 0 else '#999'
                    ui.label(rth_text).style(f'flex: 1.2; text-align: center; min-width: 50px; color: {rth_color}; font-weight: bold')
        
        # Scroll to bottom
        if self.mission_stats_scroll:
            self.mission_stats_scroll.scroll_to(percent=1.0)

    async def _show_takeoff_confirmation_dialog(self, drone_name: str) -> bool:
        """Show takeoff confirmation dialog and return user's choice."""
        result = {'confirmed': None}
        
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label(f'🚁 Takeoff Confirmation').classes('text-xl font-bold text-blue-700')
            ui.separator()
            ui.label(f'Drone "{drone_name}" will take off.').classes('text-lg mt-2')
            ui.label('Confirm to proceed with takeoff.').classes('text-sm text-gray-600 mt-1')
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=lambda: self._handle_takeoff_cancel(dialog, result), color='red').props('flat')
                ui.button('Confirm', on_click=lambda: self._handle_takeoff_confirm(dialog, result), color='primary')
        
        dialog.open()
        
        # Wait for user response
        while result['confirmed'] is None:
            await asyncio.sleep(0.1)
        
        return result['confirmed']
    
    async def _show_abort_confirmation_dialog(self) -> bool:
        """Show abort confirmation dialog and return user's choice."""
        result = {'confirmed': None}
        
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label('⚠️ Abort Mission?').classes('text-xl font-bold text-red-700')
            ui.separator()
            ui.label('Are you sure you want to abort the mission?').classes('text-lg mt-2')
            ui.label('The relay system will stop. Airborne drones will continue unaffected.').classes('text-sm text-gray-600 mt-1')
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=lambda: self._handle_abort_cancel(dialog, result)).props('flat')
                ui.button('Confirm Abort', on_click=lambda: self._handle_abort_confirm(dialog, result), color='red')
        
        dialog.open()
        
        # Wait for user response
        while result['confirmed'] is None:
            await asyncio.sleep(0.1)
        
        return result['confirmed']
    
    def _show_takeoff_dialog_sync(self, drone_name: str):
        """Show takeoff confirmation dialog synchronously (non-async version)."""
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label(f'🚁 Takeoff Confirmation').classes('text-xl font-bold text-blue-700')
            ui.separator()
            ui.label(f'Drone "{drone_name}" will take off.').classes('text-lg mt-2')
            ui.label('Confirm to proceed with takeoff.').classes('text-sm text-gray-600 mt-1')
            
            def on_confirm():
                dialog.close()
                # Play takeoff confirmation sound
                self._play_sound('take_off.mp3')
                # Start the appropriate mission
                if self._pending_single_mission:
                    self._pending_single_mission = False
                    self._do_start_single_mission(self._pending_takeoff_drone)
                elif self._pending_relay_mission:
                    self._pending_relay_mission = False
                    data = self._pending_relay_data
                    self._do_start_relay_mission(data['drone_list'], data['travel_time'], data['distance'])
            
            def on_cancel():
                dialog.close()
                self._show_abort_dialog_sync()
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=on_cancel, color='red').props('flat')
                ui.button('Confirm', on_click=on_confirm, color='primary')
        
        dialog.open()
    
    def _show_abort_dialog_sync(self):
        """Show abort confirmation dialog synchronously."""
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label('⚠️ Abort Mission?').classes('text-xl font-bold text-red-700')
            ui.separator()
            ui.label('Are you sure you want to abort the mission?').classes('text-lg mt-2')
            ui.label('The relay system will stop. Airborne drones will continue unaffected.').classes('text-sm text-gray-600 mt-1')
            
            def on_confirm_abort():
                dialog.close()
                self._pending_single_mission = False
                self._pending_relay_mission = False
                ui.notify('Mission cancelled', type='warning')
                self._emit_log("[MISSION] User aborted mission from takeoff confirmation")
            
            def on_cancel_abort():
                dialog.close()
                # Re-show the takeoff dialog
                self._show_takeoff_dialog_sync(self._pending_takeoff_drone)
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=on_cancel_abort).props('flat')
                ui.button('Confirm Abort', on_click=on_confirm_abort, color='red')
        
        dialog.open()
    
    def _handle_takeoff_confirm(self, dialog, result):
        """Handle confirm button click in takeoff dialog."""
        result['confirmed'] = True
        dialog.close()
        # Play takeoff confirmation sound
        ui.run_javascript('''
            var audio = new Audio("/static/take_off.mp3");
            audio.play().catch(function(e) { console.log("Audio play failed:", e); });
        ''')
    
    def _handle_takeoff_cancel(self, dialog, result):
        """Handle cancel button click in takeoff dialog - show abort confirmation."""
        dialog.close()
        
        async def show_abort():
            abort_confirmed = await self._show_abort_confirmation_dialog()
            if abort_confirmed:
                result['confirmed'] = False
                self._emit_log("[MISSION] User aborted mission from takeoff confirmation")
            else:
                # User cancelled the abort, show takeoff dialog again
                result['confirmed'] = None
                confirmed = await self._show_takeoff_confirmation_dialog(self._pending_takeoff_drone if hasattr(self, '_pending_takeoff_drone') else 'drone')
                result['confirmed'] = confirmed
        
        asyncio.create_task(show_abort())
    
    def _handle_abort_confirm(self, dialog, result):
        """Handle confirm button click in abort dialog."""
        result['confirmed'] = True
        dialog.close()
    
    def _handle_abort_cancel(self, dialog, result):
        """Handle cancel button click in abort dialog."""
        result['confirmed'] = False
        dialog.close()
    
    def _restart_groundstation(self):
        """Reset the groundstation state (soft restart)."""
        # Confirm with user
        with ui.dialog() as dialog, ui.card():
            ui.label('Reset Groundstation?').classes('text-lg font-bold')
            ui.label('This will stop all missions and disconnect all drones.').classes('text-sm text-gray-600')
            ui.label('The page will refresh after reset.').classes('text-sm text-gray-500')
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=dialog.close).props('flat')
                ui.button('Reset', on_click=lambda: self._do_soft_restart(dialog), color='negative')
        
        dialog.open()
    
    def _do_soft_restart(self, dialog):
        """Perform a soft restart - reset internal state without killing the process."""
        import os
        import signal
        
        dialog.close()
        
        self._emit_log("[SYSTEM] Resetting groundstation state...")
        ui.notify('Resetting groundstation...', type='warning', timeout=2000)
        
        # Stop any active missions
        try:
            self.stop_mission()
        except:
            pass
        
        # Shutdown mission controller thread
        if hasattr(self, 'mission_controller'):
            self.mission_controller.shutdown()
        
        # Kill all drone controller processes
        for ns, process in list(self.drone_processes.items()):
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=2)
            except:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except:
                    pass
        self.drone_processes.clear()
        
        # Clean up ROS subscribers and publishers
        for ns in list(self.drone_subscribers.keys()):
            for sub in self.drone_subscribers[ns].values():
                try:
                    self.destroy_subscription(sub)
                except:
                    pass
        self.drone_subscribers.clear()
        
        for ns in list(self.drone_publishers.keys()):
            for pub in self.drone_publishers[ns].values():
                try:
                    self.destroy_publisher(pub)
                except:
                    pass
        self.drone_publishers.clear()
        
        # Clear drone data
        self.drones.clear()
        
        # Reset mission state
        self.mission = RelayMission()
        self.monitoring_point = MonitoringPoint()
        
        # Reinitialize mission controller
        self.mission_controller = MissionController()
        self._setup_mission_controller_callbacks()
        self.mission_controller.on_takeoff_confirmation_request = self._on_takeoff_confirmation_request
        
        # Reset UI state
        self._mission_start_time = None
        self._stop_mission_timer()
        
        self._emit_log("[SYSTEM] Groundstation reset complete")
        
        # Refresh the page
        ui.timer(0.5, lambda: ui.run_javascript('location.reload()'), once=True)


# ============================================================================
# ENTRY POINTS
# ============================================================================

def main() -> None:
    """ROS entry point - empty to enable NiceGUI auto-reloading."""
    pass


def ros_main() -> None:
    """Initialize ROS2 and spin the node."""
    # Reset singleton to ensure fresh state on restart
    PerpetualMonitorGUI._instance = None
    
    rclpy.init()
    node = PerpetualMonitorGUI.get_instance()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


# Start ROS2 in background thread on app startup
app.on_startup(lambda: threading.Thread(target=ros_main, daemon=True).start())

# Handle ROS2 module naming conventions
ui_run.APP_IMPORT_STRING = f'{__name__}:app'

ui.run(
    uvicorn_reload_dirs=str(Path(__file__).parent.resolve()),
    reload=False,
    favicon='https://fonts.gstatic.com/s/i/short-term/release/materialsymbolsoutlined/flight/default/48px.svg',
    port=8086,
    title='Perpetual Drone Monitoring'
)
