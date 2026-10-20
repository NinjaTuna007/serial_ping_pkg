cmake_minimum_required(VERSION 3.8)
project(serial_ping_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AcousticAbort.srv"
  "srv/AcousticCommand.srv"
)

ament_package()
