"""HTML/JS templates for the video pages.

The HTML overlay is built inline; the two JavaScript blocks live as real
.js files under static/ and are loaded here with __PLACEHOLDER__ substitution
(byte-identical to the previous inline f-strings).
"""

import functools
from pathlib import Path

_STATIC = Path(__file__).parent / "static"


@functools.lru_cache(maxsize=None)
def _load_js(name: str) -> str:
    return (_STATIC / name).read_text()


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
    return (_load_js('fullscreen_video.js')
            .replace('__NAMESPACE__', namespace)
            .replace('__WS_URL__', ws_url))


def _webrtc_stream_script(namespace, ws_url, video_element_id):
    return (_load_js('webrtc_stream.js')
            .replace('__NAMESPACE__', namespace)
            .replace('__WS_URL__', ws_url)
            .replace('__VIDEO_ELEMENT_ID__', video_element_id))


def _webrtc_stop_script(namespace, video_element_id):
    return (_load_js('webrtc_stop.js')
            .replace('__NAMESPACE__', namespace)
            .replace('__VIDEO_ELEMENT_ID__', video_element_id))
