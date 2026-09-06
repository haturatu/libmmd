cmake_minimum_required(VERSION 3.25)
project(libmmd_consumer LANGUAGES CXX)

find_package(libmmd CONFIG REQUIRED)

add_executable(consumer main.cpp)
target_link_libraries(
    consumer
    PRIVATE
        mmd::core
        mmd::animation
        mmd::physics
)
