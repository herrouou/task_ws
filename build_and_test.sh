#!/bin/bash
set -e

# clear
unset GTK_PATH
rm -rf build install log

# build sine_wave_cpp and sine_wave_py
echo "Start build: sine_wave_cpp and sine_wave_py..."
colcon build --packages-select sine_wave_cpp sine_wave_py --cmake-args -DBUILD_TESTING=ON

# source
source install/setup.bash

# run test
echo "Start testing.."
colcon test --packages-select sine_wave_cpp sine_wave_py

# show test result
echo "Test results："
colcon test-result --verbose
