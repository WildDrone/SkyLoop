
        (async function connectStream___NAMESPACE__() {
            const namespace = "__NAMESPACE__";
            const wsUrl = "__WS_URL__";
            const remoteVideo = document.getElementById("__VIDEO_ELEMENT_ID__");
            
            function addDebug(msg) {
                console.log("[WebRTC " + namespace + "] " + msg);
            }
            
            if (!remoteVideo) {
                addDebug("Video element not found");
                return;
            }
            
            addDebug("Starting connection to " + wsUrl);
            
            // Create WebSocket connection for signaling
            const ws = new WebSocket(wsUrl);
            let pc = null;
            
            ws.onopen = async () => {
                addDebug("WebSocket connected");
                
                // Setup WebRTC
                const config = {
                    iceServers: [{ urls: "stun:stun.l.google.com:19302" }]
                };
                
                pc = new RTCPeerConnection(config);
                addDebug("RTCPeerConnection created");
                
                pc.onicecandidate = (event) => {
                    if (event.candidate && ws && ws.readyState === WebSocket.OPEN) {
                        ws.send(JSON.stringify(event.candidate));
                        addDebug("Sent ICE candidate");
                    }
                };
                
                pc.ontrack = (event) => {
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
                    
                    if (event.streams && event.streams[0]) {
                        addDebug("Stream has " + event.streams[0].getTracks().length + " tracks, stream active: " + event.streams[0].active);
                        remoteVideo.srcObject = event.streams[0];
                        addDebug("Set srcObject, video.srcObject active: " + (remoteVideo.srcObject ? remoteVideo.srcObject.active : "null"));
                        
                        // Try to play the video
                        remoteVideo.play().then(() => {
                            addDebug("Video playback started");
                        }).catch(e => {
                            addDebug("Play error: " + e.message);
                            // Try playing muted (autoplay policy)
                            remoteVideo.muted = true;
                            remoteVideo.play().then(() => {
                                addDebug("Video playing (muted)");
                            }).catch(e2 => addDebug("Play error 2: " + e2.message));
                        });
                    } else if (event.track) {
                        // Fallback: create a new MediaStream from the track
                        addDebug("Using track directly (no stream)");
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
                };
                
                pc.oniceconnectionstatechange = () => {
                    addDebug("ICE connection state: " + pc.iceConnectionState);
                };
                
                addDebug("Waiting for server offer...");
                
                // Store for cleanup
                window["webrtc_" + namespace] = { pc: pc, ws: ws };
            };
            
            ws.onmessage = async (event) => {
                const message = JSON.parse(event.data);
                addDebug("Received: " + message.type);
                
                if (message.type === "offer") {
                    addDebug("Processing offer...");
                    await pc.setRemoteDescription(new RTCSessionDescription(message));
                    const answer = await pc.createAnswer();
                    await pc.setLocalDescription(answer);
                    ws.send(JSON.stringify(pc.localDescription));
                    addDebug("Sent answer");
                } else if (message.type === "answer") {
                    addDebug("Processing answer...");
                    await pc.setRemoteDescription(new RTCSessionDescription(message));
                } else if (message.candidate !== undefined) {
                    // Handle ICE candidates
                    if (message.candidate === null || message.candidate === "") {
                        addDebug("Received end-of-candidates signal");
                        try {
                            await pc.addIceCandidate(null);
                        } catch (e) {}
                    } else {
                        try {
                            // Server may send candidates without sdpMid/sdpMLineIndex
                            // Default to sdpMid='0' and sdpMLineIndex=0 for video
                            const candidateInit = {
                                candidate: message.candidate,
                                sdpMid: message.sdpMid !== undefined ? message.sdpMid : "0",
                                sdpMLineIndex: message.sdpMLineIndex !== undefined ? message.sdpMLineIndex : 0
                            };
                            await pc.addIceCandidate(new RTCIceCandidate(candidateInit));
                            addDebug("Added ICE candidate");
                        } catch (e) {
                            addDebug("ICE error: " + e.message);
                        }
                    }
                } else if (message.type === "welcome") {
                    addDebug("Server welcome received");
                } else {
                    addDebug("Unknown message: " + JSON.stringify(message).substring(0, 100));
                }
            };
            
            ws.onerror = (error) => {
                addDebug("WebSocket error: " + error);
            };
            
            ws.onclose = () => {
                addDebug("WebSocket closed");
            };
        })();
        