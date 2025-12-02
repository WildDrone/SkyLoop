#!/bin/bash
# Entrypoint script for WildPerpetua container
# Checks ROS2 communication setup and guides user if needed

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║           WildPerpetua ROS2 Container Started                ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if host has run the setup script by testing ROS2 communication
# We do this by checking if the fastdds.xml is properly mounted
if [ -f "/WildPerpetua/fastdds.xml" ]; then
    echo -e "${GREEN}✓${NC} FastDDS configuration loaded"
else
    echo -e "${RED}✗${NC} FastDDS configuration not found"
fi

echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}  IMPORTANT: To communicate with ROS2 from your HOST machine:${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════${NC}"
echo ""
echo "  Run this ONCE on your host (outside the container):"
echo ""
echo -e "    ${GREEN}./setup_ros2_host.sh && source ~/.bashrc${NC}"
echo ""
echo "  Then you can use ros2 commands from your host:"
echo "    ros2 topic list"
echo "    ros2 topic echo /chatter"
echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# Source ROS2 environment
source /opt/ros/humble/setup.bash
if [ -f /WildPerpetua/install/setup.bash ]; then
    source /WildPerpetua/install/setup.bash
fi

# Execute the command passed to the container
exec "$@"
