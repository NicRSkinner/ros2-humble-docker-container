FROM ubuntu:22.04

LABEL maintainer="f20171569@hyderabad.bits-pilani.ac.in"

#ROS2 Installation Starts
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    bash-completion \
    cmake \
    dirmngr \
    git \
    gnupg2 \
    libssl-dev \
    lsb-release \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

#locale
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

#Setup Sources
RUN apt update && apt install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install bluetooth essentials
#RUN apt-get install -y bluez bluetooth
RUN apt-get install -y nano

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip3 install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures
# This is a workaround for pytest not found causing builds to fail
# Following RUN statements tests for regression of https://github.com/ros2/ros2/issues/722
RUN pip3 freeze | grep pytest \
    && python3 -m pytest --version

#Install ROS2 packages
RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-desktop
RUN apt install -y ros-humble-slam-toolbox
RUN apt install -y ros-humble-cv-bridge ros-humble-librealsense2 ros-humble-message-filters ros-humble-image-transport
RUN apt install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
RUN apt install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
#RUN apt install ros-humble-realsense-camera-msgs ros-humble-realsense-ros2-camera

#Install Intel Realsense
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN apt install -y software-properties-common
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get install -y librealsense2-dkms
RUN apt-get install -y librealsense2-utils
RUN apt-get install -y librealsense2-dbg
RUN apt-get install -y ros-humble-realsense2-camera
RUN apt-get install -y ros-humble-robot-localization
RUN apt-get install -y ros-humble-rtabmap-ros
RUN apt-get install -y ros-humble-joint-state-publisher
RUN apt-get install -y ros-humble-joint-state-publisher-gui
RUN apt-get install -y ros-humble-robot-state-publisher
RUN apt-get install -y ros-humble-xacro
RUN apt-get install -y ros-humble-gazebo-ros
RUN apt-get install -y ros-humble-gazebo-plugins
RUN apt-get install -y ros-humble-camera-info-manager
RUN apt-get install -y ros-humble-nav2-bringup
RUN apt-get install -y ros-humble-navigation2

# Needed because default DDS sucks.
RUN apt install -y ros-humble-rmw-cyclonedds-cpp

#Debug Packages
RUN apt-get install -y ros-humble-rqt-tf-tree
#End of Debug Packages
#End of ROS2 Installation

#Nvidia config for GUI
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

#Make ROS2 Workspace, install packages
WORKDIR /root/dd_ws/src
#RUN git clone https://github.com/ros/ros_tutorials.git -b humble-devel
#RUN git clone https://github.com/ros2-gbp/cartographer-release.git -b release/humble/cartographer

WORKDIR /root/dd_ws
RUN apt-get install python3-rosdep -y
RUN rosdep init
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro humble -y

WORKDIR /root/dd_ws
RUN . /opt/ros/humble/setup.sh && export MAKEFLAGS="-j6" && colcon build --symlink-install

RUN apt install ros-humble-libg2o -y

WORKDIR /root/dd_ws/src
#RUN git clone https://github.com/jdgalviss/realsense_ros2.git
#RUN git clone https://github.com/ros-perception/pointcloud_to_laserscan.git

# 3D rotating LIDAR
#RUN git clone --recursive https://github.com/rsasaki0109/lidarslam_ros2
#RUN git clone --recursive https://github.com/rsasaki0109/ndt_omp_ros2.git

WORKDIR /root/dd_ws
RUN . /opt/ros/humble/setup.sh && colcon build


RUN pip3 install \
    evdev \
    pyPS4Controller \
    odrive

RUN ln -s /usr/bin/python3 /usr/bin/python



# install Gazebo -- THIS IS NTHE NON-ROS SPECIFIC VERSION
#RUN curl -sSL http://get.gazebosim.org | sh

#entrypoint for ROS2
COPY ros2_entrypoint.sh /root/.
ENTRYPOINT ["/root/ros2_entrypoint.sh"]
CMD ["bash"]

