# Running SkyLoop in Docker

## Prerequisites
- Docker installed on your system
- Docker Compose (optional, but recommended)

## Quick Start

### Option 1: Using Docker Compose (Recommended)

1. **Build the Docker image:**
   ```bash
   docker compose build
   ```

2. **Run the container:**
   ```bash
   docker compose up -d
   ```

3. **Access the container:**
   ```bash
   docker compose exec skyloop bash
   ```

4. **Inside the container, you can run ROS2 commands:**
   ```bash
   # The workspace is already sourced
   ros2 pkg list
   
   # Build your workspace
   colcon build
   source install/setup.bash
   
   # Run your nodes
   ros2 run <package_name> <node_name>
   ```

5. **Stop the container:**
   ```bash
   docker compose down
   ```

### Option 2: Using Docker CLI

1. **Build the Docker image:**
   ```bash
   docker build -t skyloop:latest .
   ```

2. **Run the container:**
   ```bash
   docker run -it --rm \
     --name skyloop_ros2 \
     --network host \
     -e DISPLAY=$DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
     -v $(pwd)/src:/ros2_ws/src \
     skyloop:latest
   ```

## Development Workflow

### Rebuilding After Code Changes

If you modify the source code:

```bash
# Inside the container
cd /ros2_ws
colcon build
source install/setup.bash
```

Or rebuild the entire image:
```bash
docker compose build --no-cache
```

### Running GUI Applications

For GUI applications (like RViz or Gazebo), you may need to allow X11 connections:

```bash
xhost +local:docker
```

After you're done:
```bash
xhost -local:docker
```

## Useful Commands

### View container logs:
```bash
docker compose logs -f
```

### Execute commands in running container:
```bash
docker compose exec skyloop ros2 topic list
```

### Open multiple terminals in the same container:
```bash
docker exec -it skyloop_ros2 bash
```

### Clean up build artifacts:
```bash
docker compose exec skyloop bash -c "cd /ros2_ws && rm -rf build install log"
```

## Customization

### Change ROS2 Distribution
Edit the `Dockerfile` and change the first line:
```dockerfile
FROM ros:humble  # Change to ros:foxy, ros:iron, etc.
```

### Add Additional Dependencies
Edit the `Dockerfile` and add packages to the `apt-get install` section or create a `requirements.txt` for Python packages.

### Environment Variables
Edit `docker-compose.yml` to add environment variables:
```yaml
environment:
  - ROS_DOMAIN_ID=42
  - YOUR_CUSTOM_VAR=value
```

## Troubleshooting

### Network Issues
If ROS2 nodes can't communicate:
- Ensure `--network host` is used
- Check `ROS_DOMAIN_ID` matches across containers
- Verify firewall settings

### Permission Issues
If you encounter permission issues:
```bash
docker compose exec skyloop chown -R $(id -u):$(id -g) /ros2_ws
```

### GUI Not Working
```bash
xhost +local:docker
export DISPLAY=:0
```
