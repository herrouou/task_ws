# suing hukble
FROM ros:humble

# dependencies and tools
RUN apt-get update && apt-get install -y \
    libcanberra-gtk-module \
    sudo apt-get install libcanberra-gtk-module libcanberra-gtk3-module \
    sudo apt install python3-autopep8 \
    sudo apt install python3-ament-pycodestyle \
    sudo apt install clang-format \
    ros-humble-plotjuggler-ros \
    ros-humble-generate-parameter-library \
    ros-humble-ament-clang-format \
    ros-humble-cv-bridge \
    python3-colcon-common-extensions \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# workspace
WORKDIR /workspace

# copy to container
COPY . /workspace

# clear the old bbuild, log and install files.
RUN rm -rf /workspace/build /workspace/install /workspace/log

# build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# ROS2 launch
CMD ["/bin/bash", "-c", "source /workspace/install/setup.bash && ros2 launch sine_wave_cpp sine_wave_cpp.launch"]
