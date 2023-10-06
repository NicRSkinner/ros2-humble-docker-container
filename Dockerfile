# docker build .

FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04

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
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
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
    python3-numpy \
    python3-dev \
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

RUN apt-get update
RUN apt-get install -y software-properties-common
RUN apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libpng-dev libjpeg-dev libopenexr-dev libtiff-dev libwebp-dev

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

# https://towardsdev.com/installing-opencv-4-with-cuda-in-ubuntu-20-04-fde6d6a0a367
# Build and install OpenCV with CUDA support (GPU 5.0)
RUN apt-get install -y nvidia-cuda-toolkit

#RUN apt-get install -y kmod
#COPY NVIDIA-Linux-x86_64-535.98.run .
#RUN /bin/sh NVIDIA-Linux-x86_64-535.98.run -ui=none --no-drm --no-nvidia-modprobe --no-kernel-modules

#RUN apt-get install -y nvidia-utils-535

WORKDIR /root/dd_ws
RUN apt-get install -y expect
RUN git clone https://github.com/opencv/opencv.git
RUN git clone https://github.com/opencv/opencv_contrib.git
# Use an except script to install cudnn because of stupid prompts.
COPY except_script.exp .
RUN chmod a+x except_script.exp
RUN ./except_script.exp

WORKDIR /root/dd_ws/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_CUDA=ON -D WITH_CUDNN=ON -D WITH_CUBLAS=ON -D WITH_TBB=ON -D OPENCV_DNN_CUDA=ON -D OPENCV_ENABLE_NONFREE=ON -D CUDA_ARCH_BIN=5.0 -D OPENCV_EXTRA_MODULES_PATH=$HOME/dd_ws/opencv_contrib/modules -D BUILD_EXAMPLES=OFF -D HAVE_opencv_python3=ON ..

# Install Eigen with CUDA support
WORKDIR /root/dd_ws
RUN wget -qO- https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz | tar xz
RUN apt-get install -y libblas-dev
WORKDIR /root/dd_ws/eigen-3.3.7/build
RUN cmake ..
RUN make install

# Install FlaNN
RUN apt-get install -y libflann-dev

# Install PCL with CUDA support
RUN apt-get install -y vtk9
RUN apt-get install -y libboost-all-dev
WORKDIR /root/dd_ws
RUN git clone https://github.com/PointCloudLibrary/pcl.git
WORKDIR /root/dd_ws/pcl/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_CUDA=ON ..
RUN make -j4
RUN make install -j4

# Install ROS2 Packages
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-desktop-full
RUN apt-get install -y ros-humble-cv-bridge
RUN apt-get install -y ros-humble-librealsense2
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
#RUN if [ "$(dpkg --print-architecture)" = "amd64" ] ; then apt-get install -y ros-humble-gazebo-ros ; fi
#RUN if [ "$(dpkg --print-architecture)" = "amd64" ] ; then apt-get install -y ros-humble-gazebo-plugins ; fi

#RUN apt-get install -y ros-humble-ros-gz

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y ignition-fortress
RUN apt-get install -y ros-humble-ros-ign-bridge

# TRASH DOESN'T WORK
#RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
#RUN apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
#RUN apt-get update
#RUN apt-get install -y libignition-gui2-dev

RUN apt-get install -y ros-humble-camera-info-manager
RUN apt-get install -y ros-humble-nav2-bringup
RUN apt-get install -y ros-humble-navigation2
RUN apt-get install -y ros-humble-libg2o

# Needed because default DDS sucks.
RUN apt-get install -y ros-humble-rmw-cyclonedds-cpp

#Debug Packages
RUN apt-get install -y ros-humble-rqt-tf-tree
#End of Debug Packages
#End of ROS2 Installation

#Nvidia config for GUI
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,display

# Make ROS2 Workspace, install packages
WORKDIR /root/dd_ws/src
#RUN git clone https://github.com/ros/ros_tutorials.git -b humble-devel
#RUN git clone https://github.com/ros2-gbp/cartographer-release.git -b release/humble/cartographer

WORKDIR /root/dd_ws
RUN apt-get install -y python3-rosdep
RUN rosdep init
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro humble -y

#WORKDIR /root/dd_ws
#RUN . /opt/ros/humble/setup.sh && export MAKEFLAGS="-j6" && colcon build --symlink-install

# Install HAL Python packages
RUN pip3 install \
    evdev \
    pyPS4Controller \
    odrive
    
RUN ln -s /usr/bin/python3 /usr/bin/python

WORKDIR /root/dd_ws
RUN wget https://github.com/foxglove/studio/releases/download/v1.63.0/foxglove-studio-1.63.0-linux-amd64.deb
RUN apt-get install -y ./foxglove-studio-1.63.0-linux-amd64.deb
RUN apt-get install -y ros-humble-rosbridge-server
RUN apt-get install -y ros-humble-foxglove-bridge

RUN rm -rf /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1 /usr/lib/x86_64-linux-gnu/libcuda.so.1 /usr/lib/x86_64-linux-gnu/libcudadebugger.so.1

# Gzweb install (OLD?)
#RUN apt-get install -y libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial
#RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
#RUN . /root/.bashrc && nvm install 18 && nvm use 18
#WORKDIR /root/dd_ws
#RUN apt-get install -y npm
#RUN npm install gzweb
#RUN git clone https://github.com/osrf/gzweb.git
#RUN cd gzweb
#RUN . /usr/share/gazebo/setup.sh
#WORKDIR /root/dd_ws/gzweb
#RUN npm run deploy --- # For some reason this upsets nodeJS
#WORKDIR /root/dd_ws

# Gzweb install (NEW?)
#RUN apt-get install -y npm
#RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.37.0/install.sh | bash
#RUN . /root/.bashrc && nvm install 14 && nvm use 14
#RUN node -v && npm -v

#WORKDIR /root/dd_ws
#RUN git clone https://github.com/gazebo-web/gazebosim-frontend.git
#WORKDIR /root/dd_ws/gazebosim-frontend
#RUN npm install
#RUN npm install gzweb
#RUN npm install -g @angular/cli

RUN apt-get install -y x11vnc xvfb fluxbox #Use export DISPLAY=:1 and Xvfb $DISPLAY -screen 0 1024x768x16 & for headless gzserver

#entrypoint for ROS2
COPY ros2_entrypoint.sh /root/.
ENTRYPOINT ["/root/ros2_entrypoint.sh"]
RUN echo "source /root/ros2_entrypoint.sh" >> /root/.bashrc
CMD ["bash"]
