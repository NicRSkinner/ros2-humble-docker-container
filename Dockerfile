# docker build .

FROM ubuntu:22.04

LABEL maintainer="nicholas.skinner95@gmail.com"

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

RUN apt update
RUN apt-get install -y software-properties-common
RUN apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev 
RUN apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

#Install Intel Realsense
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

RUN if [ "$(dpkg --print-architecture)" = "arm64" ] ; then echo "deb https://librealsense.intel.com/Debian/apt-repo bionic main" >> /etc/apt/sources.list ; else add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u ; fi
RUN if [ "$(dpkg --print-architecture)" = "arm64" ] ; then echo "deb https://librealsense.intel.com/Debian/apt-repo focal main" >> /etc/apt/sources.list ; fi
#RUN echo "deb https://librealsense.intel.com/Debian/apt-repo bionic main" >> /etc/apt/sources.list
#RUN echo "deb https://librealsense.intel.com/Debian/apt-repo focal main" >> /etc/apt/sources.list

# Manually set repo, as issues happen on the Jetson nano with Jammy as the container
#RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get update
# DKMS not currently found.
# RUN apt-get install -y librealsense2-dkms
RUN apt-get install -y librealsense2-utils
RUN apt-get install -y librealsense2-dbg

# Install ROS2 Packages
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-desktop
RUN apt-get install -y ros-humble-cv-bridge
RUN apt-get install -y ros-humble-librealsense2
RUN apt-get install -y ros-humble-realsense2-camera
#RUN apt-get install -y ros-humble-realsense-camera-msgs
#RUN apt-get install -y ros-humble-realsense-ros2-camera
RUN apt-get install -y ros-humble-message-filters
RUN apt-get install -y ros-humble-image-transport
RUN apt-get install -y ros-humble-slam-toolbox
RUN apt-get install -y ros-humble-robot-localization
RUN apt-get install -y ros-humble-rtabmap-ros
RUN apt-get install -y ros-humble-joint-state-publisher
RUN if [ "$(dpkg --print-architecture)" = "amd64" ] ; then apt-get install -y ros-humble-joint-state-publisher-gui ; fi
RUN apt-get install -y ros-humble-robot-state-publisher
RUN apt-get install -y ros-humble-xacro
RUN if [ "$(dpkg --print-architecture)" = "amd64" ] ; then apt-get install -y ros-humble-gazebo-ros ; fi
RUN if [ "$(dpkg --print-architecture)" = "amd64" ] ; then apt-get install -y ros-humble-gazebo-plugins ; fi
RUN apt-get install -y ros-humble-camera-info-manager
RUN apt-get install -y ros-humble-nav2-bringup
RUN apt-get install -y ros-humble-navigation2
RUN apt-get install -y ros-humble-libg2o

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

# Make ROS2 Workspace, install packages
WORKDIR /root/dd_ws/src
#RUN git clone https://github.com/ros/ros_tutorials.git -b humble-devel
#RUN git clone https://github.com/ros2-gbp/cartographer-release.git -b release/humble/cartographer

WORKDIR /root/dd_ws
RUN apt-get install -y python3-rosdep
RUN rosdep init
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro humble -y

WORKDIR /root/dd_ws
RUN . /opt/ros/humble/setup.sh && export MAKEFLAGS="-j6" && colcon build --symlink-install

WORKDIR /root/dd_ws
RUN . /opt/ros/humble/setup.sh && colcon build

# Install HAL Python packages
RUN pip3 install \
    evdev \
    pyPS4Controller \
    odrive

RUN ln -s /usr/bin/python3 /usr/bin/python

#entrypoint for ROS2
COPY ros2_entrypoint.sh /root/.
ENTRYPOINT ["/root/ros2_entrypoint.sh"]
CMD ["bash"]
