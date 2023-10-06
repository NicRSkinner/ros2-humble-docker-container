XAUTH=/home/$USER/.Xauthority

docker run -it \
    --net=host \
    --privileged \
    -p $1:8800 \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --gpus all \
    -v /run/udev:/run/udev:ro \
    -v /dev:/dev \
    -v /home/$USER/.gazebo:/root/.gazebo \
    -v /home/$USER/Documents/ardak/:/root/dd_ws/ardak \
    -v /home/$USER/Documents/ros-inc/:/opt/ros/foxy/cp \
    -v /home/$USER/Documents/cp:/root/dd_ws/cp \
    dockros  \
    bash
