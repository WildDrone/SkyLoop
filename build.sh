#!/bin/bash
# Quick build script - run from host to build in container
docker compose exec wildperpetua bash -c "cd /WildPerpetua && colcon build --symlink-install && source install/setup.bash"
