cmake_minimum_required(VERSION 3.12)
project(sowa_ros_adapter LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 3.3 REQUIRED)
find_package(SOWA 0.1 CONFIG REQUIRED)

add_library(sowa_ros_adapter SHARED src/doa_axis_scheduler.cpp)
target_include_directories(sowa_ros_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(sowa_ros_adapter PUBLIC SOWA::core ${catkin_LIBRARIES})
target_compile_features(sowa_ros_adapter PUBLIC cxx_std_17)
target_compile_definitions(sowa_ros_adapter PUBLIC
  EIGEN_MAX_ALIGN_BYTES=16 EIGEN_MAX_STATIC_ALIGN_BYTES=16)
set_target_properties(sowa_ros_adapter PROPERTIES
  BUILD_RPATH "$ORIGIN"
  INSTALL_RPATH "$ORIGIN"
  POSITION_INDEPENDENT_CODE ON
)

include(GNUInstallDirs)
install(TARGETS sowa_ros_adapter
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

include(CTest)
if(BUILD_TESTING)
  add_executable(sowa_adapter_link_smoke tests/link_smoke.cpp)
  target_link_libraries(sowa_adapter_link_smoke PRIVATE sowa_ros_adapter)
  add_test(NAME sowa_adapter_link_smoke COMMAND sowa_adapter_link_smoke)
  add_executable(sowa_core_abi_smoke tests/core_abi_smoke.cpp)
  target_link_libraries(sowa_core_abi_smoke PRIVATE sowa_ros_adapter)
  add_test(NAME sowa_core_abi_smoke COMMAND sowa_core_abi_smoke)
endif()
