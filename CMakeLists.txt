cmake_minimum_required(VERSION 3.22)

project(RealsenseViewer
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

find_package(OpenCV REQUIRED COMPONENTS calib3d core features2d highgui imgcodecs imgproc xfeatures2d)
find_package(PCL REQUIRED COMPONENTS common filters visualization)
find_package(realsense2 CONFIG QUIET
    HINTS
        "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/cmake/realsense2"
        "C:/Program Files/Intel RealSense SDK 2.0/lib/cmake/realsense2"
        /opt/homebrew/opt/librealsense/lib/cmake/realsense2
        /usr/local/opt/librealsense/lib/cmake/realsense2
)

if(NOT realsense2_FOUND AND WIN32)
    find_path(realsense2_INCLUDE_DIR
        NAMES librealsense2/rs.hpp
        HINTS
            "C:/Program Files (x86)/Intel RealSense SDK 2.0/include"
            "C:/Program Files/Intel RealSense SDK 2.0/include"
    )

    find_library(realsense2_LIBRARY
        NAMES realsense2
        HINTS
            "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64"
            "C:/Program Files/Intel RealSense SDK 2.0/lib/x64"
    )

    if(realsense2_INCLUDE_DIR AND realsense2_LIBRARY)
        set(realsense2_FOUND TRUE)
    endif()
endif()

if(NOT realsense2_FOUND)
    message(FATAL_ERROR
        "Intel RealSense SDK 2.0 was not found. Install librealsense2 first, "
        "for example with the Intel RealSense SDK 2.0 installer on Windows "
        "or `brew install librealsense` on macOS, or configure with "
        "-Drealsense2_DIR=/path/to/realsense2Config.cmake."
    )
endif()

add_executable(RealsenseViewer
    RealsenseViewer.cpp
    src/Application.cpp
    src/camera/RealSenseDiagnostics.cpp
    src/camera/RealSenseFrameProcessor.cpp
    src/camera/RealSenseFrameSource.cpp
    src/display/OpenCvFramePresenter.cpp
    src/features/ObjectFeatureMatcher.cpp
)

target_include_directories(RealsenseViewer
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(RealsenseViewer
    PRIVATE
        ${PCL_DEFINITIONS}
)

target_link_libraries(RealsenseViewer
    PRIVATE
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

if(WIN32)
    target_link_libraries(RealsenseViewer PRIVATE Comdlg32)
endif()

if(APPLE)
    target_link_libraries(RealsenseViewer PRIVATE "-framework ApplicationServices")
endif()

if(TARGET realsense2::realsense2)
    target_link_libraries(RealsenseViewer PRIVATE realsense2::realsense2)
elseif(TARGET realsense2)
    target_link_libraries(RealsenseViewer PRIVATE realsense2)
else()
    target_include_directories(RealsenseViewer PRIVATE
        ${realsense2_INCLUDE_DIR}
        ${realsense2_INCLUDE_DIRS}
    )
    target_link_libraries(RealsenseViewer PRIVATE
        ${realsense2_LIBRARY}
        ${realsense2_LIBRARIES}
    )
endif()

if(MSVC)
    target_compile_options(RealsenseViewer PRIVATE /W4 /permissive-)
else()
    target_compile_options(RealsenseViewer PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        $<$<CONFIG:Debug>:-O0>
        $<$<CONFIG:Debug>:-g>
        $<$<CONFIG:Debug>:-fstandalone-debug>
    )
endif()

if(APPLE)
    find_program(DSYMUTIL_EXECUTABLE dsymutil)
endif()

if(APPLE AND DSYMUTIL_EXECUTABLE)
    add_custom_command(TARGET RealsenseViewer POST_BUILD
        COMMAND "${DSYMUTIL_EXECUTABLE}" "$<TARGET_FILE:RealsenseViewer>" -o "$<TARGET_FILE:RealsenseViewer>.dSYM"
        COMMENT "Generating RealsenseViewer.dSYM"
        VERBATIM
    )
endif()
