cmake_minimum_required(VERSION 3.25)

project(libmmd VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

option(LIBMMD_ENABLE_PHYSICS "Build the optional MMD physics module" ON)
option(LIBMMD_ENABLE_BULLET "Enable Bullet acceleration for the physics module when available" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mmd_core
  src/formats/pmx_reader.cpp
  src/formats/pmx_writer.cpp
  src/formats/vmd.cpp
  src/internal/mapped_file.cpp
)
set_target_properties(mmd_core PROPERTIES EXPORT_NAME core)
add_library(mmd::core ALIAS mmd_core)
target_include_directories(mmd_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE src/internal
)
target_compile_features(mmd_core PUBLIC cxx_std_20)

if(LIBMMD_ENABLE_PHYSICS)
  add_library(mmd_physics src/physics/physics.cpp)
  set_target_properties(mmd_physics PROPERTIES EXPORT_NAME physics)
  add_library(mmd::physics ALIAS mmd_physics)
  target_link_libraries(mmd_physics PUBLIC mmd_core)
  target_include_directories(mmd_physics PRIVATE src/internal)
  set(LIBMMD_HAS_BULLET 0)
  if(LIBMMD_ENABLE_BULLET)
    find_package(Bullet QUIET)
    if(Bullet_FOUND)
      set(LIBMMD_HAS_BULLET 1)
      target_include_directories(mmd_physics PRIVATE ${BULLET_INCLUDE_DIRS})
      target_link_libraries(mmd_physics PRIVATE ${BULLET_LIBRARIES})
    endif()
  endif()
  target_compile_definitions(mmd_physics PRIVATE LIBMMD_HAS_BULLET=${LIBMMD_HAS_BULLET} DAYO_HAS_BULLET=${LIBMMD_HAS_BULLET})

  add_library(mmd_animation src/animation/animation.cpp)
  set_target_properties(mmd_animation PROPERTIES EXPORT_NAME animation)
  add_library(mmd::animation ALIAS mmd_animation)
  target_link_libraries(mmd_animation PUBLIC mmd_core mmd_physics)
  target_include_directories(mmd_animation PRIVATE src/internal)
endif()

if(BUILD_TESTING)
  add_executable(libmmd_pmx_roundtrip_tests tests/pmx_roundtrip_tests.cpp)
  target_link_libraries(libmmd_pmx_roundtrip_tests PRIVATE mmd::core)
  add_test(NAME pmx_roundtrip COMMAND libmmd_pmx_roundtrip_tests)
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS mmd_core EXPORT libmmdTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
if(TARGET mmd_physics)
  install(TARGETS mmd_physics mmd_animation EXPORT libmmdTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
install(EXPORT libmmdTargets NAMESPACE mmd:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libmmd)
configure_package_config_file(cmake/libmmdConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/libmmdConfig.cmake INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libmmd)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/libmmdConfigVersion.cmake VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libmmdConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/libmmdConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libmmd)
