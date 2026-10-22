# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target mycar_interfaces::mycar_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${mycar_interfaces_TARGETS}.
if(mycar_interfaces_TARGETS AND NOT TARGET mycar_interfaces::mycar_interfaces)
  add_library(mycar_interfaces::mycar_interfaces INTERFACE IMPORTED)
  set_target_properties(mycar_interfaces::mycar_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${mycar_interfaces_TARGETS}")
endif()
