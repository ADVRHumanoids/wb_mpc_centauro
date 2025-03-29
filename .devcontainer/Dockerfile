FROM osrf/ros:noetic-desktop

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
# ARG USERNAME=ubuntu
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID
# RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
#         groupadd --gid $USER_GID $USERNAME && \
#         useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
#     fi
# # Add sudo support for the non-root user
# RUN apt-get update && \
#     apt-get install -y sudo && \
#     echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
#     chmod 0440 /etc/sudoers.d/$USERNAME

# # Switch from root to user
# USER $USERNAME

# Add user to video group to allow access to webcam
# RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

#############################################
################## OCS2 #####################
#############################################

# install catkin anc others specified by ocs2
RUN apt update \
    && apt install -y libglpk-dev \
    python3-catkin-tools \
    ros-noetic-pybind11-catkin \
    python3-catkin-tools \
    doxygen \
    doxygen-latex \
    liburdfdom-dev \
    liboctomap-dev \
    libassimp-dev \
    && rm -rf /var/lib/apt/lists/*

# install gnome terminal and other dependencies for avoidding errors
RUN apt update \
    && apt install -y dbus-x11 \                  
    libcanberra-gtk-module \
    gnome-terminal \
    && rm -rf /var/lib/apt/lists/*

# install rqt plot tool
RUN apt-get update \
    && apt-get install -y ros-noetic-rqt-multiplot \
    checkinstall \
    wget \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Clone pinocchio, hpp-fcl and raisim
WORKDIR /workspace/src
RUN git clone --branch humanoids2023 https://github.com/IoannisDadiotis/ocs2.git \
    && git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git \
    && git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git \
    && git clone https://github.com/IoannisDadiotis/ocs2_robotic_assets.git \
    && git clone --depth 1 https://github.com/raisimTech/raisimLib.git -b v1.1.01

#install grid map msgs, needed from raisim and rviz-plugin to skip error
RUN sudo apt-get update \
&& apt-get install -y ros-noetic-grid-map-msgs \
ros-${ROS_DISTRO}-grid-map-rviz-plugin \
&& rm -rf /var/lib/apt/lists/*

# install raisim
# Add the following lines to your ~/.bashrc file
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WORKSPACE/raisim/linux/lib' >> ~/.bashrc \
    && echo 'export PYTHONPATH=$PYTHONPATH:$WORKSPACE/raisim/linux/lib' >> ~/.bashrc
RUN cd /workspace/src/raisimLib \
    && mkdir build \
    && cd build \
    && cmake .. -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=$(which python3) \
    && make -j4 && sudo checkinstall

# install mpc-net related
# RUN cd /tmp \
#     && wget https://github.com/microsoft/onnxruntime/releases/download/v1.7.0/onnxruntime-linux-x64-1.7.0.tgz \
#     && tar xf onnxruntime-linux-x64-1.7.0.tgz \
#     && mkdir -p ~/.local/bin ~/.local/include/onnxruntime ~/.local/lib ~/.local/share/cmake/onnxruntime \
#     && rsync -a /tmp/onnxruntime-linux-x64-1.7.0/include/ ~/.local/include/onnxruntime \
#     && rsync -a /tmp/onnxruntime-linux-x64-1.7.0/lib/ ~/.local/lib \
#     && rsync -a /workspace/src/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ ~/.local/share/cmake/onnxruntime
    
#############################################
############# WB_MPC_CENTAURO ###############
#############################################

# install xbot2
# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# sudo apt install curl
# curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# sudo apt update && sudo apt install -y \
# ros-noetic-ros-base \
# libgazebo11-dev
# echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc

# some xbot2 dependencies
RUN apt update \
    && apt install -y ros-$ROS_DISTRO-urdf \
    ros-$ROS_DISTRO-kdl-parser \
    ros-$ROS_DISTRO-eigen-conversions \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-moveit-core \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-interactive-markers \
    ros-$ROS_DISTRO-tf-conversions \
    ros-$ROS_DISTRO-tf2-eigen \
    qttools5-dev \
    libqt5charts5-dev \
    qtdeclarative5-dev \
    && rm -rf /var/lib/apt/lists/*

# install xbot2 full
RUN sh -c 'echo "deb http://xbot.cloud/xbot2/ubuntu/$(lsb_release -sc) /" > /etc/apt/sources.list.d/xbot-latest.list'
RUN wget -q -O - http://xbot.cloud/xbot2/ubuntu/KEY.gpg | sudo apt-key add -
RUN apt update \
    && apt install -y xbot2_desktop_full \
    && rm -rf /var/lib/apt/lists/*
RUN echo ". /opt/xbot/setup.sh" >> ~/.bashrc

# install gazebo
RUN apt update \
    && apt install -y ros-${ROS_DISTRO}-gazebo-ros

# clone base estimation
WORKDIR /workspace/src
RUN git clone --branch ioannis_centauro https://github.com/ADVRHumanoids/base_estimation.git

# centauro, dagana and sensors' models
RUN cd / \
    && mkdir models && cd models \
    && git clone --branch ioannis-devel https://github.com/ADVRHumanoids/iit-centauro-ros-pkg.git \
    && git clone https://github.com/ADVRHumanoids/iit-dagana-ros-pkg.git \
    && git clone https://github.com/m-tartari/realsense_gazebo_description.git \
    && git clone https://github.com/m-tartari/realsense_gazebo_plugin.git \
    && git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git

# Set up catkin workspace
WORKDIR /workspace
RUN catkin init \
    && catkin config --blacklist ocs2_doc -DCMAKE_BUILD_TYPE=RelWithDebInfo

# source catkin pkgs
RUN echo "source /workspace/devel/setup.bash --extend" >> ~/.bashrc

# source models
RUN echo 'export ROS_PACKAGE_PATH=/models:$ROS_PACKAGE_PATH' >> ~/.bashrc

# return to root user for other images
USER root
################################
## ADD ANY CUSTOM SETUP BELOW ##
################################
