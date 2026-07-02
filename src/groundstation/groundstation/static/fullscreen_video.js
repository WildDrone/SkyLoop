
        (async function connectFullscreenStream() {
            const wsUrl = "__WS_URL__";
            const namespace = "__NAMESPACE__";
            const remoteVideo = document.getElementById("fullscreenVideo___NAMESPACE__");
            const connectionStatus = document.getElementById("connectionStatus___NAMESPACE__");
            const dataChannelStatus = document.getElementById("dataChannelStatus___NAMESPACE__");
            
            // Telemetry elements
            const telemGps = document.getElementById("telem_gps___NAMESPACE__");
            const telemAltitude = document.getElementById("telem_altitude___NAMESPACE__");
            const telemSats = document.getElementById("telem_sats___NAMESPACE__");
            const telemGimbal = document.getElementById("telem_gimbal___NAMESPACE__");
            const telemAttitude = document.getElementById("telem_attitude___NAMESPACE__");
            const telemHeading = document.getElementById("telem_heading___NAMESPACE__");
            const telemVelocity = document.getElementById("telem_velocity___NAMESPACE__");
            const telemBattery = document.getElementById("telem_battery___NAMESPACE__");
            const telemFrame = document.getElementById("telem_frame___NAMESPACE__");
            
            // Stream stats elements
            const metaResolution = document.getElementById("meta_resolution___NAMESPACE__");
            const metaBitrate = document.getElementById("meta_bitrate___NAMESPACE__");
            const metaCodec = document.getElementById("meta_codec___NAMESPACE__");
            const metaLatency = document.getElementById("meta_latency___NAMESPACE__");
            const metaJitter = document.getElementById("meta_jitter___NAMESPACE__");
            const metaFrames = document.getElementById("meta_frames___NAMESPACE__");
            const metaDropped = document.getElementById("meta_dropped___NAMESPACE__");
            
            let lastBytesReceived = 0;
            let lastTimestamp = Date.now();
            let statsInterval = null;
            let telemetryChannel = null;
            
            function setConnectionStatus(text, color) {
                if (connectionStatus) {
                    connectionStatus.textContent = text;
                    connectionStatus.style.background = color;
                }
            }
            
            function setDataChannelStatus(text, color) {
                if (dataChannelStatus) {
                    dataChannelStatus.textContent = "📡 Telemetry: " + text;
                    dataChannelStatus.style.color = color;
                }
            }
            
            function updateTelemetry(meta) {
                if (!meta) return;
                
                // GPS
                const lat = meta.latitude || 0;
                const lon = meta.longitude || 0;
                if (telemGps) {
                    if (lat !== 0 || lon !== 0) {
                        telemGps.textContent = "📍 GPS: " + lat.toFixed(6) + ", " + lon.toFixed(6);
                    } else {
                        telemGps.textContent = "📍 GPS: No fix";
                    }
                }
                
                // Altitude
                const altASL = meta.altitudeASL || 0;
                const altAGL = meta.altitudeAGL || 0;
                if (telemAltitude) {
                    telemAltitude.textContent = "🏔️ Alt: ASL " + altASL.toFixed(1) + "m AGL " + altAGL.toFixed(1) + "m";
                }
                
                // Satellites
                const sats = meta.satelliteCount || 0;
                if (telemSats) {
                    let satsColor = sats > 10 ? "#00ff00" : sats > 5 ? "#ffa500" : "#ff0000";
                    telemSats.innerHTML = '🛰️ Satellites: <span style="color:' + satsColor + '">' + sats + '</span>';
                }
                
                // Gimbal
                const gimbalPitch = meta.gimbalPitch || 0;
                const gimbalYaw = meta.gimbalYaw || 0;
                const gimbalRoll = meta.gimbalRoll || 0;
                if (telemGimbal) {
                    telemGimbal.textContent = "🎥 Gimbal P:" + gimbalPitch.toFixed(1) + "° Y:" + gimbalYaw.toFixed(1) + "° R:" + gimbalRoll.toFixed(1) + "°";
                }
                
                // Attitude
                const pitch = meta.aircraftPitch || 0;
                const yaw = meta.aircraftYaw || 0;
                const roll = meta.aircraftRoll || 0;
                if (telemAttitude) {
                    telemAttitude.textContent = "✈️ Attitude P:" + pitch.toFixed(1) + "° Y:" + yaw.toFixed(1) + "° R:" + roll.toFixed(1) + "°";
                }
                
                // Heading
                if (telemHeading) {
                    telemHeading.textContent = "🧭 Heading: " + yaw.toFixed(1) + "°";
                }
                
                // Velocity
                const vx = meta.velocityX || 0;
                const vy = meta.velocityY || 0;
                const vz = meta.velocityZ || 0;
                const speed = Math.sqrt(vx*vx + vy*vy + vz*vz);
                if (telemVelocity) {
                    telemVelocity.textContent = "💨 Speed: " + speed.toFixed(1) + " m/s";
                }
                
                // Battery with color coding
                const battery = meta.batteryPercent || 0;
                if (telemBattery) {
                    let batteryColor = battery > 30 ? "#00ff00" : battery > 15 ? "#ffa500" : "#ff0000";
                    telemBattery.innerHTML = '🔋 Battery: <span style="color:' + batteryColor + '">' + battery + '%</span>';
                }
                
                // Frame number
                const frameNum = meta.frameNumber || "N/A";
                if (telemFrame) {
                    telemFrame.textContent = "📹 Frame: " + frameNum;
                }
            }
            
            function formatBitrate(bps) {
                if (bps < 1000) return bps.toFixed(0) + " bps";
                if (bps < 1000000) return (bps / 1000).toFixed(1) + " Kbps";
                return (bps / 1000000).toFixed(2) + " Mbps";
            }
            
            async function updateStats(pc) {
                if (!pc) return;
                
                try {
                    const stats = await pc.getStats();
                    
                    stats.forEach(report => {
                        if (report.type === "inbound-rtp" && report.kind === "video") {
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
                            if (report.codecId) {
                                const codecReport = stats.get(report.codecId);
                                if (codecReport && metaCodec) {
                                    metaCodec.textContent = "Codec: " + (codecReport.mimeType || "unknown").replace("video/", "");
                                }
                            }
                        }
                        
                        if (report.type === "candidate-pair" && report.state === "succeeded") {
                            if (metaLatency) {
                                metaLatency.textContent = "Latency: " + (report.currentRoundTripTime ? (report.currentRoundTripTime * 1000).toFixed(1) + " ms" : "--");
                            }
                        }
                    });
                    
                    // Update video resolution from video element
                    if (remoteVideo && remoteVideo.videoWidth > 0) {
                        if (metaResolution) metaResolution.textContent = "Resolution: " + remoteVideo.videoWidth + "x" + remoteVideo.videoHeight;
                    }
                    
                } catch (e) {
                    console.error("Error getting stats:", e);
                }
            }
            
            function addDebug(msg) {
                console.log("[WebRTC Fullscreen " + namespace + "] " + msg);
            }
            
            function setupTelemetryChannel(channel) {
                addDebug("Setting up telemetry channel: " + channel.label);
                telemetryChannel = channel;
                
                channel.onopen = () => {
                    addDebug("Telemetry channel opened");
                    setDataChannelStatus("connected", "#00ff00");
                };
                
                channel.onmessage = (event) => {
                    try {
                        if (!event.data) return;
                        const meta = JSON.parse(event.data);
                        updateTelemetry(meta);
                        
                        // Store latest telemetry globally for recording
                        window.latestTelemetry___NAMESPACE__ = meta;
                        
                        addDebug("Telemetry received: frame " + (meta.frameNumber || "N/A"));
                    } catch (e) {
                        addDebug("Telemetry parse error: " + e.message);
                    }
                };
                
                channel.onclose = () => {
                    addDebug("Telemetry channel closed");
                    setDataChannelStatus("closed", "#888");
                };
                
                channel.onerror = (error) => {
                    addDebug("Telemetry channel error: " + error);
                    setDataChannelStatus("error", "#ff0000");
                };
                
                // If channel is already open
                if (channel.readyState === "open") {
                    setDataChannelStatus("connected", "#00ff00");
                }
            }
            
            if (!remoteVideo) {
                addDebug("Video element not found");
                setConnectionStatus("❌ Video element not found", "rgba(239, 68, 68, 0.9)");
                return;
            }
            
            setConnectionStatus("⏳ Connecting...", "rgba(245, 158, 11, 0.9)");
            addDebug("Starting fullscreen connection to " + wsUrl);
            
            const ws = new WebSocket(wsUrl);
            let pc = null;
            
            ws.onopen = async () => {
                addDebug("WebSocket connected");
                setConnectionStatus("🔗 WebSocket connected", "rgba(59, 130, 246, 0.9)");
                
                const config = {
                    iceServers: [{ urls: "stun:stun.l.google.com:19302" }]
                };
                
                pc = new RTCPeerConnection(config);
                addDebug("RTCPeerConnection created");
                
                // Create telemetry data channel (negotiated mode to match Android side)
                try {
                    telemetryChannel = pc.createDataChannel("telemetry", {
                        negotiated: true,
                        id: 0,
                        ordered: true
                    });
                    addDebug("Created telemetry data channel (negotiated mode)");
                    setupTelemetryChannel(telemetryChannel);
                } catch (e) {
                    addDebug("Error creating telemetry channel: " + e.message);
                }
                
                // Also handle incoming data channels (if Android creates it differently)
                pc.ondatachannel = (event) => {
                    addDebug("Received data channel: " + event.channel.label);
                    if (event.channel.label === "telemetry") {
                        setupTelemetryChannel(event.channel);
                    }
                };
                
                pc.onicecandidate = (event) => {
                    if (event.candidate && ws && ws.readyState === WebSocket.OPEN) {
                        ws.send(JSON.stringify(event.candidate));
                    }
                };
                
                pc.ontrack = (event) => {
                    addDebug("Track received: " + event.track.kind);
                    
                    if (event.streams && event.streams[0]) {
                        remoteVideo.srcObject = event.streams[0];
                        
                        // Update frame rate when metadata is loaded
                        remoteVideo.onloadedmetadata = () => {
                            addDebug("Video metadata loaded: " + remoteVideo.videoWidth + "x" + remoteVideo.videoHeight);
                            if (metaResolution) metaResolution.textContent = "Resolution: " + remoteVideo.videoWidth + "x" + remoteVideo.videoHeight;
                        };
                        
                        remoteVideo.play().then(() => {
                            addDebug("Video playback started");
                            setConnectionStatus("✅ Stream playing", "rgba(34, 197, 94, 0.9)");
                            
                            // Start stats monitoring
                            if (statsInterval) clearInterval(statsInterval);
                            statsInterval = setInterval(() => updateStats(pc), 1000);
                        }).catch(e => {
                            remoteVideo.muted = true;
                            remoteVideo.play().then(() => {
                                setConnectionStatus("✅ Stream playing (muted)", "rgba(34, 197, 94, 0.9)");
                                if (statsInterval) clearInterval(statsInterval);
                                statsInterval = setInterval(() => updateStats(pc), 1000);
                            }).catch(e2 => {
                                addDebug("Play error: " + e2.message);
                                setConnectionStatus("❌ Play error", "rgba(239, 68, 68, 0.9)");
                            });
                        });
                    } else if (event.track) {
                        let stream = remoteVideo.srcObject;
                        if (!stream) {
                            stream = new MediaStream();
                            remoteVideo.srcObject = stream;
                        }
                        stream.addTrack(event.track);
                        remoteVideo.play().catch(e => addDebug("Play error: " + e.message));
                    }
                };
                
                pc.onconnectionstatechange = () => {
                    addDebug("Connection state: " + pc.connectionState);
                    
                    if (pc.connectionState === "connected") {
                        setConnectionStatus("✅ Connected", "rgba(34, 197, 94, 0.9)");
                    } else if (pc.connectionState === "disconnected" || pc.connectionState === "failed") {
                        setConnectionStatus("❌ " + pc.connectionState, "rgba(239, 68, 68, 0.9)");
                        if (statsInterval) clearInterval(statsInterval);
                    }
                };
                
                window.fullscreenWebRTC = { pc: pc, ws: ws, statsInterval: statsInterval, telemetryChannel: telemetryChannel };
            };
            
            ws.onmessage = async (event) => {
                const message = JSON.parse(event.data);
                addDebug("Received: " + message.type);
                
                if (message.type === "offer") {
                    await pc.setRemoteDescription(new RTCSessionDescription(message));
                    const answer = await pc.createAnswer();
                    await pc.setLocalDescription(answer);
                    ws.send(JSON.stringify(pc.localDescription));
                    addDebug("Sent answer");
                } else if (message.type === "answer") {
                    await pc.setRemoteDescription(new RTCSessionDescription(message));
                } else if (message.candidate !== undefined) {
                    if (message.candidate === null || message.candidate === "") {
                        try { await pc.addIceCandidate(null); } catch (e) {}
                    } else {
                        try {
                            const candidateInit = {
                                candidate: message.candidate,
                                sdpMid: message.sdpMid !== undefined ? message.sdpMid : "0",
                                sdpMLineIndex: message.sdpMLineIndex !== undefined ? message.sdpMLineIndex : 0
                            };
                            await pc.addIceCandidate(new RTCIceCandidate(candidateInit));
                        } catch (e) {
                            addDebug("ICE error: " + e.message);
                        }
                    }
                }
            };
            
            ws.onerror = (error) => {
                addDebug("WebSocket error: " + error);
                setConnectionStatus("❌ WebSocket error", "rgba(239, 68, 68, 0.9)");
            };
            
            ws.onclose = () => {
                addDebug("WebSocket closed");
                setConnectionStatus("⚫ Disconnected", "rgba(107, 114, 128, 0.9)");
                if (statsInterval) clearInterval(statsInterval);
            };
            
            // Cleanup on page unload
            window.addEventListener("beforeunload", () => {
                if (statsInterval) clearInterval(statsInterval);
                if (window.fullscreenWebRTC) {
                    if (window.fullscreenWebRTC.ws) window.fullscreenWebRTC.ws.close();
                    if (window.fullscreenWebRTC.pc) window.fullscreenWebRTC.pc.close();
                }
            });
        })();
        