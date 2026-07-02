# Use official ROS 2 Humble Desktop image
FROM osrf/ros:humble-desktop

# Install additional tools (as root before creating user)
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    python3-pip \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Copy and install Python dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt && rm /tmp/requirements.txt

# Arguments to create a user matching the host UID/GID
ARG UID=1000
ARG GID=1000
ARG UNAME=rosuser

# Create a group and user with same UID/GID as host
RUN groupadd -g $GID $UNAME || true && \
    useradd -m -u $UID -g $GID -s /bin/bash $UNAME || true && \
    echo "$UNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to that user
USER $UNAME
WORKDIR /home/$UNAME

# Create workspace
WORKDIR /SkyLoop

# Copy source code (when you add it)
# COPY src ./src

# Copy the entire workspace if src exists
COPY . .

# Install rosdep dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source the workspace automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /SkyLoop/install/setup.bash" >> ~/.bashrc && \
    echo "alias build='cd /SkyLoop && colcon build --symlink-install && source install/setup.bash'" >> ~/.bashrc && \
    echo "alias build-pkg='cd /SkyLoop && colcon build --symlink-install --packages-select'" >> ~/.bashrc

# Copy and set entrypoint
COPY entrypoint.sh /entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["/bin/bash"]
