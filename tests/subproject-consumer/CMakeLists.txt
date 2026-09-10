cmake_minimum_required(VERSION 3.25)
project(libmmd_subproject_consumer LANGUAGES CXX)

include(CTest)
include(FetchContent)

set(
  LIBMMD_SOURCE_DIR
  "${CMAKE_CURRENT_LIST_DIR}/../.."
  CACHE PATH
  "Source directory used by the FetchContent consumer test"
)

FetchContent_Declare(libmmd SOURCE_DIR "${LIBMMD_SOURCE_DIR}")
FetchContent_MakeAvailable(libmmd)

if(TARGET libmmd_pmx_roundtrip_tests OR TARGET libmmd_physics_smoke_tests)
  message(FATAL_ERROR "libmmd enabled its top-level tests as a subproject")
endif()

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE mmd::core mmd::animation)
add_test(NAME consumer COMMAND consumer)
