# The base image pins the ROS 2 distro. Override it to target a different version, e.g.
#   docker build --build-arg BASE_IMAGE=frankjoshua/ros2:humble ...
# Everything below references $ROS_DISTRO (set by the base image), so nothing else needs editing.
ARG BASE_IMAGE=frankjoshua/ros2:jazzy
FROM ${BASE_IMAGE} AS base
# Single source of truth for shared dependencies. Both `dev` and `prod` inherit this stage,
# so they cannot drift apart. Any dependency NOT declared in a src/*/package.xml must be added
# here — never installed ad hoc inside a running dev container (that change would not reach prod).
RUN apt-get update && apt-get install -y \
        python3-pip wget\
    && rm -rf /var/lib/apt/lists/*

# ---- dev: what VS Code opens. Reuse the image's default non-root user (uid 1000, "ubuntu"),
# already in the dialout/video/plugdev groups handy for robotics hardware; just add passwordless
# sudo. VS Code remaps its UID to the host user so bind-mounted files aren't left root-owned.
FROM base AS dev
ARG USERNAME=ubuntu
RUN echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# VS Code terminals open an interactive shell that bypasses the image ENTRYPOINT, so source the ROS
# environment (and the workspace overlay, once built) from .bashrc — otherwise `ros2` isn't on PATH.
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo '[ -f /home/ws/install/setup.bash ] && source /home/ws/install/setup.bash' >> /home/$USERNAME/.bashrc
ENV SHELL=/bin/bash
USER $USERNAME
CMD ["/bin/bash"]

# ---- prod: base + workspace baked and built. ----
FROM base AS prod
WORKDIR /ros2_ws
COPY src ./src
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build --symlink-install
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "run", "example_pkg", "example_node"]
