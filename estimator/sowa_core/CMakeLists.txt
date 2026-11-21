cmake_minimum_required(VERSION 3.12)
project(SOWA VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 3.3 REQUIRED)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(sowa_core src/core.cpp src/v076_compat.cpp)
add_library(SOWA::core ALIAS sowa_core)
set_target_properties(sowa_core PROPERTIES
  EXPORT_NAME core
  POSITION_INDEPENDENT_CODE ON)
target_include_directories(sowa_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_definitions(sowa_core PRIVATE EIGEN_DONT_PARALLELIZE)
if(TARGET Eigen3::Eigen)
  target_link_libraries(sowa_core PUBLIC Eigen3::Eigen)
else()
  target_include_directories(sowa_core SYSTEM PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  )
endif()
target_compile_features(sowa_core PUBLIC cxx_std_14)

set(SOWA_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/SOWA")
configure_package_config_file(
  cmake/SOWAConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/SOWAConfig.cmake"
  INSTALL_DESTINATION "${SOWA_INSTALL_CMAKEDIR}"
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SOWAConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

install(TARGETS sowa_core EXPORT SOWATargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)
install(DIRECTORY include/sowa DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(EXPORT SOWATargets
  FILE SOWATargets.cmake
  NAMESPACE SOWA::
  DESTINATION "${SOWA_INSTALL_CMAKEDIR}"
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/SOWAConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/SOWAConfigVersion.cmake"
  DESTINATION "${SOWA_INSTALL_CMAKEDIR}"
)

include(CTest)
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE"
  "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/doc/SOWA")
if(BUILD_TESTING)
  add_executable(sowa_core_tests tests/test_core.cpp)
  target_compile_definitions(sowa_core_tests PRIVATE EIGEN_DONT_PARALLELIZE)
  target_link_libraries(sowa_core_tests PRIVATE sowa_core)
  add_test(NAME sowa_core_tests COMMAND sowa_core_tests)
endif()
