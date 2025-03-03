cmake_minimum_required(VERSION 3.10)
project(sine_wave_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies "std_msgs" "rclcpp" "generate_parameter_library")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# srv and msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "sine_wave_interface/srv/ConvertImage.srv"
  "sine_wave_interface/msg/Signal.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(sensor_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


# generate a paramter library from yaml file
generate_parameter_library(
  sine_wave_parameters  #cmake target name for the parameter library
  config/sine_wave_parameters.yaml  #path to yaml file
)


# For publisher
add_executable(sine_wave_publisher_node
  src/sine_wave_publisher_node.cpp
  src/sine_wave_publisher.cpp
)

target_include_directories(sine_wave_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(sine_wave_publisher_node
  sine_wave_parameters  # parmater
  ${cpp_typesupport_target}  # msg
)

ament_target_dependencies(sine_wave_publisher_node # use this to link the basic things
  rclcpp
  std_msgs
)

install(TARGETS
  sine_wave_publisher_node
  sine_wave_parameters  # Also install the generated library
  DESTINATION lib/${PROJECT_NAME}
)

# For receiver
add_executable(sine_wave_receiver_node
  src/sine_wave_receiver_node.cpp
  src/sine_wave_receiver.cpp
)

target_include_directories(sine_wave_receiver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(sine_wave_receiver_node
  sine_wave_parameters  # Link to the generated parameter library
  ${cpp_typesupport_target}
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

ament_target_dependencies(sine_wave_receiver_node
  rclcpp
  std_msgs
  cv_bridge
  sensor_msgs
)

install(TARGETS
  sine_wave_receiver_node
  sine_wave_parameters  # Also install the generated library
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS sine_wave_publisher_node sine_wave_receiver_node sine_wave_parameters
  EXPORT ${PROJECT_NAME}Targets)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)



# add gtest，to make publisher to library
# add library for SineWavePublisher
add_library(sine_wave_publisher_lib
  src/sine_wave_publisher.cpp
)
target_include_directories(sine_wave_publisher_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(sine_wave_publisher_lib
  sine_wave_parameters
  ${cpp_typesupport_target}
)

ament_target_dependencies(sine_wave_publisher_lib
  rclcpp
  std_msgs
)

# add library for SineWaveReceiver
add_library(sine_wave_receiver_lib
  src/sine_wave_receiver.cpp
)
target_include_directories(sine_wave_receiver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sine_wave_receiver_lib
  sine_wave_parameters
  ${cpp_typesupport_target}
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)
ament_target_dependencies(sine_wave_receiver_lib
  rclcpp
  std_msgs
  cv_bridge
  sensor_msgs
  OpenCV
)

install(DIRECTORY config
  DESTINATION share/sine_wave_cpp
)

# test:
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Publisher test
  ament_add_gtest(${PROJECT_NAME}_sine_wave_publisher_test test/sine_wave_publisher_test.cpp)
  target_include_directories(${PROJECT_NAME}_sine_wave_publisher_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_sine_wave_publisher_test
    sine_wave_publisher_lib
    sine_wave_parameters
  )
  ament_target_dependencies(${PROJECT_NAME}_sine_wave_publisher_test
    rclcpp
    std_msgs
  )
  set_target_properties(${PROJECT_NAME}_sine_wave_publisher_test PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$<TARGET_FILE_DIR:sine_wave_publisher_lib>;/opt/ros/humble/lib;/opt/ros/humble/lib/x86_64-linux-gnu"
  )

  # Receiver test
  ament_add_gtest(${PROJECT_NAME}_sine_wave_receiver_test test/sine_wave_receiver_test.cpp)
  target_include_directories(${PROJECT_NAME}_sine_wave_receiver_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_sine_wave_receiver_test
    sine_wave_receiver_lib
    sine_wave_parameters
    ${cpp_typesupport_target}
  )
  ament_target_dependencies(${PROJECT_NAME}_sine_wave_receiver_test
    rclcpp
    std_msgs
    cv_bridge
    sensor_msgs
    OpenCV
  )
  set_target_properties(${PROJECT_NAME}_sine_wave_receiver_test PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$<TARGET_FILE_DIR:sine_wave_receiver_lib>;/opt/ros/humble/lib;/opt/ros/humble/lib/x86_64-linux-gnu"
  )
endif()

ament_package()
