#!/bin/bash

# Check if a name was provided, otherwise use a default name
CONTAINER_NAME=${1:-centauro_mpc_container}

# attach to the container with the given name
docker exec -it "$CONTAINER_NAME" /bin/bash
