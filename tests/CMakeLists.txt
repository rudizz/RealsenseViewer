add_library(unity_test_framework STATIC
    vendor/unity/unity.cpp
)

target_include_directories(unity_test_framework
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/unity
)

target_compile_features(unity_test_framework PUBLIC cxx_std_17)

add_executable(realsenseviewer_queue_tests
    test_concurrent_queue.cpp
)

target_include_directories(realsenseviewer_queue_tests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(realsenseviewer_queue_tests
    PRIVATE
        unity_test_framework
)

target_compile_features(realsenseviewer_queue_tests PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(realsenseviewer_queue_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(realsenseviewer_queue_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME realsenseviewer_queue_tests COMMAND realsenseviewer_queue_tests)

find_package(OpenCV QUIET COMPONENTS calib3d core features2d imgcodecs imgproc xfeatures2d)
find_package(PCL QUIET COMPONENTS common filters)
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

if(OpenCV_FOUND AND PCL_FOUND AND realsense2_FOUND)
    add_executable(realsenseviewer_module_tests
        test_application.cpp
        test_command_line_options.cpp
        test_frame_types.cpp
        test_module_main.cpp
        test_object_feature_matcher.cpp
        test_point_cloud_settings.cpp
        ${PROJECT_SOURCE_DIR}/src/Application.cpp
        ${PROJECT_SOURCE_DIR}/src/CommandLineOptions.cpp
        ${PROJECT_SOURCE_DIR}/src/camera/RealSenseFrameProcessor.cpp
        ${PROJECT_SOURCE_DIR}/src/features/ObjectFeatureMatcher.cpp
    )

    target_include_directories(realsenseviewer_module_tests
        PRIVATE
            ${PROJECT_SOURCE_DIR}/include
            ${PCL_INCLUDE_DIRS}
    )

    target_compile_definitions(realsenseviewer_module_tests
        PRIVATE
            ${PCL_DEFINITIONS}
    )

    target_link_libraries(realsenseviewer_module_tests
        PRIVATE
            unity_test_framework
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
    )

    if(TARGET realsense2::realsense2)
        target_link_libraries(realsenseviewer_module_tests PRIVATE realsense2::realsense2)
    elseif(TARGET realsense2)
        target_link_libraries(realsenseviewer_module_tests PRIVATE realsense2)
    else()
        target_include_directories(realsenseviewer_module_tests PRIVATE
            ${realsense2_INCLUDE_DIR}
            ${realsense2_INCLUDE_DIRS}
        )
        target_link_libraries(realsenseviewer_module_tests PRIVATE
            ${realsense2_LIBRARY}
            ${realsense2_LIBRARIES}
        )
    endif()

    target_compile_features(realsenseviewer_module_tests PRIVATE cxx_std_17)

    if(MSVC)
        target_compile_options(realsenseviewer_module_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(realsenseviewer_module_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME realsenseviewer_module_tests COMMAND realsenseviewer_module_tests)
else()
    message(STATUS "Skipping module Unity tests because OpenCV, PCL, or librealsense2 was not found")
endif()
