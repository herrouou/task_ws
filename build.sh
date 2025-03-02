#!/bin/bash
rm -rf build install log && \
colcon build --cmake-args -DBUILD_TESTING=ON && \
colcon test && \
colcon test-result --verbose