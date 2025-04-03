#!/bin/bash

# go to workspace
cd /workspace

# first build ocs2
catkin build ocs2

# then build the rest
catkin build
source ~/.bashrc
