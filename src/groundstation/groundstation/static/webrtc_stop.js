
        (function() {
            try {
                const conn = window["webrtc___NAMESPACE__"];
                if (conn) {
                    if (conn.ws) conn.ws.close();
                    if (conn.pc) conn.pc.close();
                    window["webrtc___NAMESPACE__"] = null;
                }
                
                // Clear video element
                const video = document.getElementById("__VIDEO_ELEMENT_ID__");
                if (video) {
                    video.srcObject = null;
                }
                
                console.log("[WebRTC] Stream stopped for __NAMESPACE__");
            } catch (error) {
                console.error("[WebRTC] Error stopping stream: " + error);
            }
        })();
        