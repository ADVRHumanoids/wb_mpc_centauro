#!/bin/bash

# check if an image name is given or use default
IMAGE_NAME=${1:-centauro_mpc_image}

docker build -t "$IMAGE_NAME" . 


