cmake_minimum_required(VERSION 3.25)
project(libmmd_consumer LANGUAGES CXX)

include(CTest)

option(LIBMMD_CONSUMER_WITH_PHYSICS "Link the optional mmd::physics target" ON)

set(components core animation)
if(LIBMMD_CONSUMER_WITH_PHYSICS)
    list(APPEND components physics)
endif()
option(LIBMMD_CONSUMER_WITH_BULLET "Require the Bullet backend" OFF)
if(LIBMMD_CONSUMER_WITH_BULLET)
    list(APPEND components bullet)
endif()
find_package(libmmd 0.3.2 CONFIG REQUIRED COMPONENTS ${components})

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE mmd::core mmd::animation)
if(LIBMMD_CONSUMER_WITH_PHYSICS)
    if(NOT TARGET mmd::physics)
        message(
            FATAL_ERROR
            "LIBMMD_CONSUMER_WITH_PHYSICS requires mmd::physics"
        )
    endif()
    target_link_libraries(consumer PRIVATE mmd::physics)
    target_compile_definitions(consumer PRIVATE LIBMMD_CONSUMER_WITH_PHYSICS=1)
    if(LIBMMD_CONSUMER_WITH_BULLET)
        target_compile_definitions(
            consumer
            PRIVATE LIBMMD_CONSUMER_WITH_BULLET=1
        )
    endif()
else()
    target_compile_definitions(consumer PRIVATE LIBMMD_CONSUMER_WITH_PHYSICS=0)
endif()

add_test(NAME consumer COMMAND consumer)
