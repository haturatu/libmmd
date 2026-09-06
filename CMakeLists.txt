cmake_minimum_required(VERSION 3.25)

project(libmmd VERSION 0.2.3 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

option(LIBMMD_ENABLE_PHYSICS "Build the optional MMD physics module" ON)
option(
    LIBMMD_ENABLE_BULLET
    "Enable Bullet acceleration for the physics module when available"
    ON
)
option(LIBMMD_BUILD_TESTS "Build libmmd tests" ${PROJECT_IS_TOP_LEVEL})
option(
    LIBMMD_FETCH_DEPENDENCIES
    "Fetch optional source dependencies when unavailable"
    OFF
)
option(
    LIBMMD_INSTALL
    "Generate install rules for libmmd"
    ${PROJECT_IS_TOP_LEVEL}
)
option(
    LIBMMD_WARNINGS_AS_ERRORS
    "Enable strict compiler warnings for libmmd targets"
    OFF
)
option(LIBMMD_BUILD_FUZZERS "Build optional Clang libFuzzer parser targets" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(
    mmd_core
    src/formats/pmx_reader.cpp
    src/formats/pmx_document.cpp
    src/formats/pmx_semantic.cpp
    src/formats/pmx_writer.cpp
    src/formats/vmd.cpp
    src/internal/mapped_file.cpp
)
set_target_properties(mmd_core PROPERTIES EXPORT_NAME core)
add_library(mmd::core ALIAS mmd_core)
target_include_directories(
    mmd_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE src/internal
)
target_compile_features(mmd_core PUBLIC cxx_std_20)

function(libmmd_enable_warnings target)
    if(NOT LIBMMD_WARNINGS_AS_ERRORS)
        return()
    endif()

    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
        target_compile_options(
            ${target}
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wconversion
                -Wsign-conversion
                -Werror
        )
    endif()
endfunction()

libmmd_enable_warnings(mmd_core)

add_library(mmd_animation src/animation/animation.cpp)
set_target_properties(mmd_animation PROPERTIES EXPORT_NAME animation)
add_library(mmd::animation ALIAS mmd_animation)
target_link_libraries(mmd_animation PUBLIC mmd_core)
target_include_directories(mmd_animation PRIVATE src/internal)
target_compile_definitions(
    mmd_animation
    PRIVATE LIBMMD_ENABLE_PHYSICS=$<BOOL:${LIBMMD_ENABLE_PHYSICS}>
)
libmmd_enable_warnings(mmd_animation)

if(LIBMMD_ENABLE_PHYSICS)
    add_library(mmd_physics src/physics/physics.cpp)
    set_target_properties(mmd_physics PROPERTIES EXPORT_NAME physics)
    add_library(mmd::physics ALIAS mmd_physics)
    target_link_libraries(mmd_physics PUBLIC mmd_core)
    target_include_directories(mmd_physics PRIVATE src/internal)
    set(LIBMMD_HAS_BULLET 0)
    set(LIBMMD_BULLET_FETCHED OFF)
    if(LIBMMD_ENABLE_BULLET)
        find_package(Bullet QUIET)
        if(NOT Bullet_FOUND AND LIBMMD_FETCH_DEPENDENCIES)
            set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
            set(BUILD_BULLET3 OFF CACHE BOOL "" FORCE)
            set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
            set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
            set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)
            set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
            set(USE_GRAPHICAL_BENCHMARK OFF CACHE BOOL "" FORCE)
            set(INSTALL_LIBS OFF CACHE BOOL "" FORCE)
            include(FetchContent)
            FetchContent_Declare(
                bullet
                URL
                    https://github.com/bulletphysics/bullet3/archive/refs/tags/3.25.tar.gz
                DOWNLOAD_EXTRACT_TIMESTAMP TRUE
            )
            FetchContent_MakeAvailable(bullet)
            set(BULLET_INCLUDE_DIRS "${bullet_SOURCE_DIR}/src")
            set(BULLET_LIBRARIES BulletDynamics BulletCollision LinearMath)
            set(Bullet_FOUND TRUE)
            set(LIBMMD_BULLET_FETCHED ON)
        endif()
        if(Bullet_FOUND)
            set(LIBMMD_HAS_BULLET 1)
            target_include_directories(
                mmd_physics
                SYSTEM
                PRIVATE ${BULLET_INCLUDE_DIRS}
            )
            target_link_libraries(mmd_physics PRIVATE ${BULLET_LIBRARIES})
        endif()
    endif()
    if(LIBMMD_INSTALL AND LIBMMD_BULLET_FETCHED)
        message(
            FATAL_ERROR
            "Installing libmmd with a FetchContent-provided Bullet is unsupported. "
            "Install Bullet system-wide, disable LIBMMD_ENABLE_BULLET, "
            "or configure with -DLIBMMD_INSTALL=OFF."
        )
    endif()
    set(LIBMMD_CONFIG_NEEDS_BULLET OFF)
    if(LIBMMD_HAS_BULLET AND NOT LIBMMD_BULLET_FETCHED)
        set(LIBMMD_CONFIG_NEEDS_BULLET ON)
    endif()
    target_compile_definitions(
        mmd_physics
        PRIVATE LIBMMD_HAS_BULLET=${LIBMMD_HAS_BULLET}
        INTERFACE LIBMMD_HAS_BULLET=${LIBMMD_HAS_BULLET}
    )
    libmmd_enable_warnings(mmd_physics)
    target_link_libraries(mmd_animation PRIVATE mmd_physics)
endif()

if(BUILD_TESTING AND LIBMMD_BUILD_TESTS)
    add_executable(libmmd_pmx_roundtrip_tests tests/pmx_roundtrip_tests.cpp)
    target_link_libraries(libmmd_pmx_roundtrip_tests PRIVATE mmd::core)
    add_test(NAME pmx_roundtrip COMMAND libmmd_pmx_roundtrip_tests)
    if(LIBMMD_ENABLE_PHYSICS)
        add_executable(libmmd_physics_smoke_tests tests/physics_smoke_tests.cpp)
        target_link_libraries(libmmd_physics_smoke_tests PRIVATE mmd::physics)
        add_test(NAME physics_smoke COMMAND libmmd_physics_smoke_tests)
    endif()

    file(
        GLOB LIBMMD_HEADER_SELF_CONTAINMENT_SOURCES
        CONFIGURE_DEPENDS
        tests/header-self-containment/*.cpp
    )
    foreach(source IN LISTS LIBMMD_HEADER_SELF_CONTAINMENT_SOURCES)
        get_filename_component(name "${source}" NAME_WE)
        add_executable(libmmd_${name} ${source})
        target_include_directories(libmmd_${name} PRIVATE include)
        target_compile_features(libmmd_${name} PRIVATE cxx_std_20)
    endforeach()
endif()

if(LIBMMD_BUILD_FUZZERS)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "LIBMMD_BUILD_FUZZERS requires a Clang compiler")
    endif()
    add_executable(libmmd_pmx_fuzzer tests/fuzz/pmx_fuzzer.cpp)
    target_link_libraries(libmmd_pmx_fuzzer PRIVATE mmd::core)
    target_compile_options(
        libmmd_pmx_fuzzer
        PRIVATE -fsanitize=fuzzer,address,undefined -fno-omit-frame-pointer
    )
    target_link_options(
        libmmd_pmx_fuzzer
        PRIVATE -fsanitize=fuzzer,address,undefined -fno-omit-frame-pointer
    )
endif()

if(LIBMMD_INSTALL)
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    install(
        TARGETS mmd_core mmd_animation
        EXPORT libmmdTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
    if(TARGET mmd_physics)
        install(
            TARGETS mmd_physics
            EXPORT libmmdTargets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )
    endif()
    install(
        EXPORT libmmdTargets
        NAMESPACE mmd::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libmmd
    )
    configure_package_config_file(
        cmake/libmmdConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/libmmdConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libmmd
    )
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/libmmdConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )
    install(
        FILES
            ${CMAKE_CURRENT_BINARY_DIR}/libmmdConfig.cmake
            ${CMAKE_CURRENT_BINARY_DIR}/libmmdConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libmmd
    )
endif()
