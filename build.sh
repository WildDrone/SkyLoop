#!/bin/bash
# Quick build script - run from host to build in container
# Clear Python cache to prevent stale module state
docker compose exec skyloop bash -c "cd /SkyLoop && find . -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null; colcon build --symlink-install && source install/setup.bash"
