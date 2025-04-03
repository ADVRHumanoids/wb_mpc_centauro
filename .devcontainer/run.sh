#!/bin/bash

# check if an image name is given or use default
IMAGE_NAME=${1:-centauro_mpc_image}
CONTAINER_NAME=${2:-centauro_mpc_container}

docker run --rm -it \
 --name="$CONTAINER_NAME" \
 --privileged \
 --network=host \
 --ipc=host \
 --env=DISPLAY \
 -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
 -v $(pwd)/../../wb_mpc_centauro:/workspace/src/wb_mpc_centauro \
 "$IMAGE_NAME" 


