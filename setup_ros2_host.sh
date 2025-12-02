#!/bin/bash
# Setup script to enable ROS2 communication between Docker container and host
# Run this script once to configure your host machine

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTDDS_CONFIG="$SCRIPT_DIR/fastdds.xml"
BASHRC="$HOME/.bashrc"

echo "=== WildPerpetua ROS2 Host Setup ==="

# Check if fastdds.xml exists
if [ ! -f "$FASTDDS_CONFIG" ]; then
    echo "Error: fastdds.xml not found at $FASTDDS_CONFIG"
    exit 1
fi

# Check if already configured
if grep -q "FASTRTPS_DEFAULT_PROFILES_FILE.*WildPerpetua" "$BASHRC" 2>/dev/null; then
    echo "✓ ROS2 FastDDS config already in ~/.bashrc"
else
    echo ""
    echo "# WildPerpetua ROS2 Docker communication setup" >> "$BASHRC"
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_CONFIG" >> "$BASHRC"
    echo "✓ Added FastDDS config to ~/.bashrc"
fi

# Export for current session
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_CONFIG"

echo ""
echo "=== Setup Complete ==="
echo ""
echo "To apply changes to your current terminal, run:"
echo "  source ~/.bashrc"
echo ""
echo "Or simply open a new terminal."
echo ""
echo "Then you can communicate with ROS2 nodes in the Docker container:"
echo "  ros2 topic list"
echo "  ros2 topic echo /chatter"
