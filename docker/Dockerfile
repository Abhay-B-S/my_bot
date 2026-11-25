# NavaYantra delivery robot: the LAPTOP side (ROS 2 Foxy) in a container, for any Linux host (e.g. Ubuntu 22.04).
# The Raspberry Pi is unchanged (its own Ubuntu 20.04 + Foxy SD card); the container talks to it over Wi-Fi with
# host networking, exactly like the old Ubuntu 20.04 laptop did.
# Build:  docker build -t navayantra:foxy -f docker/Dockerfile .      (from the my_bot folder)
# Run:    bash docker/run.sh [sim|robot|shell]                         (see docker/README.md)
FROM osrf/ros:foxy-desktop

ENV DEBIAN_FRONTEND=noninteractive
# The same ROS packages the 20.04 laptop had (apt-mark showmanual, 25 Nov 2026) + Python libraries the scripts import
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-slam-toolbox ros-foxy-twist-mux \
      ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-ros2-control \
      ros-foxy-xacro ros-foxy-joint-state-publisher-gui ros-foxy-teleop-twist-keyboard ros-foxy-tf2-tools \
      python3-numpy python3-yaml python3-serial python3-pil python3-colcon-common-extensions \
      git openssh-client iputils-ping iproute2 nano less psmisc \
    && rm -rf /var/lib/apt/lists/*

# Workspace with the same four packages as ~/dev_ws/src on the old laptop
WORKDIR /root/dev_ws/src
RUN git clone https://github.com/Abhay-B-S/my_bot.git my_bot \
 && git clone https://github.com/joshnewans/diffdrive_arduino.git diffdrive_arduino \
 && git clone -b newans_ros2 https://github.com/joshnewans/serial serial \
 && git clone https://github.com/joshnewans/serial_motor_demo.git serial_motor_demo
WORKDIR /root/dev_ws
RUN . /opt/ros/foxy/setup.sh && colcon build --symlink-install

# Same environment as the old laptop's terminals
RUN echo 'source /opt/ros/foxy/setup.bash' >> /root/.bashrc \
 && echo 'source /root/dev_ws/install/setup.bash' >> /root/.bashrc \
 && echo 'export ROS_DOMAIN_ID=0' >> /root/.bashrc \
 && echo 'cd /root/dev_ws' >> /root/.bashrc \
 && ln -s /root/dev_ws/src/my_bot/tools/expo.sh /root/expo
CMD ["bash"]
