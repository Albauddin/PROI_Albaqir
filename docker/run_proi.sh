#!/bin/bash

# Allow X11 access for GUI apps (needed only once per session)
xhost +local:root


# Optional: fix permissions before starting container
echo "Fixing permissions on mounted volume (~/Documents/Docker/Volumes/PROI)..."
sudo chown -R $USER:$USER ~/Documents/Docker/Volumes


# Run Docker container
docker run -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="NVIDIA_VISIBLE_DEVICES=all" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/dev:/dev" \
  --volume="$HOME/Documents/Docker/Volumes/PROI:/workspace/data" \
  --volume="$HOME/.ssh:/root/.ssh:ro" \
  --net=host \
  --privileged \
  --runtime=nvidia \
  --name PROI \
  ros:humble

