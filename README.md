# WildPerpetua

Autonomous multi-drone relay system for perpetual wildlife monitoring using DJI drones and ROS 2.

WildPerpetua enables continuous aerial surveillance by automatically swapping drones when batteries run low. Designed for wildlife conservation research, it allows researchers to maintain uninterrupted observation without human presence disrupting natural animal behavior. The system manages multiple DJI drones through a web-based groundstation interface, coordinating takeoff, monitoring, and return-to-home operations in a seamless relay pattern.

## Table of Contents

- [Background](#background)
- [Install](#install)
- [Usage](#usage)
- [Architecture](#architecture)
- [Features](#features)
- [API](#api)
- [Maintainers](#maintainers)
- [Contributing](#contributing)
- [License](#license)

## Background

Traditional drone-based wildlife monitoring is limited by battery life, typically allowing only 20-40 minutes of flight time. WildPerpetua solves this by orchestrating multiple drones in a relay system—when one drone's battery runs low, another automatically takes over the monitoring position.

The system uses:
- **ROS 2 Humble** for robust inter-process communication
- **DJI PSDK** for drone control and telemetry
- **NiceGUI** for the real-time web interface

## Install

### Prerequisites

- Docker and Docker Compose
- (Optional) ROS 2 Humble on host for direct topic communication
- DJI drones with PSDK support

### Quick Start with Docker

1. **Clone the repository:**
   ```bash
   git clone https://github.com/edouardrolland/WildPerpetua.git
   cd WildPerpetua
   ```

2. **Setup host for ROS 2 communication (optional, one-time):**
   ```bash
   ./setup_ros2_host.sh
   source ~/.bashrc
   ```

3. **Build and run:**
   ```bash
   docker compose build
   docker compose up -d
   ```

4. **Access the groundstation UI:**
   
   Open http://localhost:8086 in your browser.

### Manual Installation

1. **Install ROS 2 Humble** following the [official guide](https://docs.ros.org/en/humble/Installation.html).

2. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

3. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Usage

### Starting the System

```bash
# Using Docker
docker compose up

# Or manually with ROS 2
source install/setup.bash
ros2 launch wildview_bringup perpetual_monitoring.launch.py
```

### Connecting Drones

1. Open the web interface at http://localhost:8086
2. Enter the drone's IP address and a unique name
3. Click "Connect"

### Starting a Monitoring Mission

1. Set the monitoring point by clicking on the map or entering coordinates manually
2. Configure altitude and camera heading
3. Choose mission type:
   - **Single**: One drone flies to the point and monitors
   - **Relay**: Multiple drones take turns for continuous coverage
4. Click Start

### CLI Commands

```bash
# List connected drones
ros2 topic echo /groundstation/drone_status

# Monitor mission state
ros2 topic echo /groundstation/mission_state

# Emergency stop all drones
ros2 topic pub /groundstation/emergency_stop std_msgs/msg/Empty
```

## Architecture

```
WildPerpetua/
├── src/
│   ├── groundstation/          # Web UI and mission control
│   │   ├── perpetual_monitor.py      # Core ROS 2 node
│   │   └── perpetual_monitor_gui.py  # NiceGUI web interface
│   ├── dji_controller/         # DJI PSDK drone interface
│   └── wildview_bringup/       # Launch files
├── Dockerfile
├── docker-compose.yml
└── requirements.txt
```

### ROS 2 Nodes

| Node | Description |
|------|-------------|
| `perpetual_monitor_node` | Main groundstation with web UI |
| `dji_controller_node` | Per-drone DJI PSDK interface |
| `safety_node` | Emergency monitoring and failsafes |

## Features

- **Multi-Drone Relay**: Automatic drone swapping based on battery and RTH predictions
- **RTH Predictor**: Machine learning-based battery drain prediction
- **Dual Navigation Modes**: PID control or DJI Native trajectory following
- **Real-Time Web UI**: Live map, telemetry, and mission control
- **ROS Bag Recording**: Mission data logging for analysis
- **Dockerized Deployment**: Easy setup and reproducibility

## API

### ROS 2 Topics

#### Published by Groundstation

| Topic | Type | Description |
|-------|------|-------------|
| `/{drone}/goto_waypoint` | `geographic_msgs/GeoPoint` | Command drone to position |
| `/{drone}/takeoff` | `std_msgs/Empty` | Initiate takeoff |
| `/{drone}/land` | `std_msgs/Empty` | Initiate landing |
| `/{drone}/rth` | `std_msgs/Empty` | Return to home |

#### Subscribed by Groundstation

| Topic | Type | Description |
|-------|------|-------------|
| `/{drone}/gps_position` | `sensor_msgs/NavSatFix` | GPS coordinates |
| `/{drone}/battery_state` | `sensor_msgs/BatteryState` | Battery level |
| `/{drone}/flight_status` | `std_msgs/String` | Current flight state |

### Web Interface

The NiceGUI web interface is available at `http://localhost:8086` and provides:
- Interactive map with drone positions
- Mission control panel
- Per-drone telemetry cards
- Event logging and statistics

## Maintainers

[@edouardrolland](https://github.com/edouardrolland)

## Contributing

PRs are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

For questions, please open an [issue](https://github.com/edouardrolland/WildPerpetua/issues).

## License

[MIT](LICENSE) © Edouard Rolland
