FROM ubuntu:jammy

# ==========================================
# 1. Installing ROS Humble
# ==========================================

# -------------
# 1.1 - Locale Setup
# -------------

RUN apt update
RUN apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# To make life easier, we are creating a vagrant user so it
# matches the VM so you can bounce between VM and Docker
# container easily
RUN useradd -m vagrant
RUN usermod -a -G root vagrant
RUN echo 'vagrant ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers
RUN mkdir -p /home/vagrant/ros_ws

# -------------
# 1.2 - Add Additional Repositories
# -------------

RUN apt install -y software-properties-common
RUN add-apt-repository universe
RUN apt update
RUN apt install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
# Finally, refresh the package lists:
RUN apt update
# Recommended as ROS2 is tied tightly to ubuntu releases apparently
RUN apt upgrade -y

# -------------
# 1.3 - Install ROS2
# -------------

ENV LANG en_US.UTF-8
ENV DEBIAN_FRONTEND noninteractive
RUN apt install -y build-essential
RUN apt install -y ros-humble-ros-core
RUN apt install -y python3-colcon-common-extensions

# -------------
# 1.4 - Source ROS
# -------------

RUN echo 'source /opt/ros/humble/setup.bash' >> /home/vagrant/.bashrc
RUN echo "set +e" >> /home/vagrant/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

# ==========================================
# 2. Installing Navigation2
# ==========================================

# -------------
# 2.1 - Install Navigation 2
# -------------

RUN apt install -y ros-humble-navigation2
RUN apt install -y ros-humble-nav2-bringup
RUN apt install -y ros-humble-slam-toolbox
RUN apt install -y ros-humble-tf-transformations

# ==========================================
# 3. Installing other tools
# ==========================================

RUN curl -Lk 'https://code.visualstudio.com/sha/download?build=stable&os=cli-alpine-x64' --output vscode_cli.tar.gz
RUN tar -xf vscode_cli.tar.gz
RUN mv code /usr/local/bin/

COPY ./envs/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENV ROS_DISTRO humble

ENTRYPOINT [ "/bin/bash", "/ros_entrypoint.sh" ]
CMD ["bash"]