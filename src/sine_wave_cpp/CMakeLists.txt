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

# srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConvertImage.srv"
  DEPENDENCIES sensor_msgs
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
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(sine_wave_publisher_node # use this only to link the sine_wave_parameters
  sine_wave_parameters  # Link to the generated parameter library
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

# For reciever
add_executable(sine_wave_reciever_node
  src/sine_wave_reciever_node.cpp
  src/sine_wave_reciever.cpp
)

target_include_directories(sine_wave_reciever_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(sine_wave_reciever_node
  sine_wave_parameters  # Link to the generated parameter library
  ${cpp_typesupport_target}
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

ament_target_dependencies(sine_wave_reciever_node
  rclcpp
  std_msgs
  cv_bridge
  sensor_msgs
)

install(TARGETS
  sine_wave_reciever_node
  sine_wave_parameters  # Also install the generated library
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS sine_wave_publisher_node sine_wave_reciever_node sine_wave_parameters
  EXPORT ${PROJECT_NAME}Targets)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)



# add gtest，to make publisher to library
add_library(sine_wave_publisher_lib
  src/sine_wave_publisher.cpp
)
target_include_directories(sine_wave_publisher_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sine_wave_publisher_lib
  sine_wave_parameters
)
ament_target_dependencies(sine_wave_publisher_lib
  rclcpp
  std_msgs
)


# test:
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
  # # ignore uncrustify error
  # set(ament_cmake_uncrustify_FOUND TRUE)
  # # ignore xmllint error
  # set(ament_cmake_xmllint_FOUND TRUE)
  # # ignore copyright error
  # set(ament_cmake_copyright_FOUND TRUE)

  ament_add_gtest(${PROJECT_NAME}_sine_wave_publisher_test test/sine_wave_publisher_test.cpp)

  target_include_directories(${PROJECT_NAME}_sine_wave_publisher_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(${PROJECT_NAME}_sine_wave_publisher_test
    sine_wave_publisher_lib
    sine_wave_parameters  # Link to the generated parameter library
  )

  ament_target_dependencies(${PROJECT_NAME}_sine_wave_publisher_test
    rclcpp
    std_msgs
  )
endif()

ament_package()
