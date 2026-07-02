"""HTML/JS templates for the video pages.

Pure string builders extracted from perpetual_monitor_gui to separate
presentation markup from GUI logic. Output is byte-identical to the
original inline f-strings.
"""


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



def _webrtc_stream_script(namespace, ws_url, video_element_id):
    return f'''
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
        '''
