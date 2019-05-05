cmake_minimum_required(VERSION 3.7.2)
project(575ResearchProject)

set(CMAKE_CXX_STANDARD 14)

#ADDD BACK
find_package(OpenCV REQUIRED)
#ADDD BACK
include_directories(${OpenCV_INCLUDE_DIRS})
#ADDD BACK
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

#add_compile_options(-lpigpio)

# add_compile_options(-lwiringPi)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w -pthread")

include_directories(BEFORE rplidar/sdk/src)
include_directories(BEFORE rplidar/sdk/include)

file(GLOB hitec_control
        "hitec_controller/*.h"
        "hitec_controller/*.cpp"
        )

file(GLOB IMU
        "BNO055/pi-bno055/i2c_bno055.c"
        "BNO055/pi-bno055/include/getbno055.h")

file(GLOB ROBOT
        "robot/*.cpp"
        "robot/*.h")

add_executable(Main ${hitec_control} ${IMU} PID.cpp main.cpp)
add_executable(Main2 ${hitec_control} ${IMU} ${ROBOT} robot_test.cpp)
add_executable(Post post_map_process.cpp)
add_executable(Stream stream_test.cpp)
###### main.cpp ultra_simple.cpp simple.cpp cartesian.cpp telemetry_test.cpp imu_test.cpp test_python.cpp python_common.h

target_link_libraries(Main wiringPi)
target_link_libraries(Main2 wiringPi)
target_link_libraries(Main pigpio)
target_link_libraries(Main2 pigpio)
target_link_libraries(Main pthread)
target_link_libraries(Main2 pthread)
#target_link_libraries(Main ncurses)

if (UNIX)
    target_link_libraries(Main ${CMAKE_SOURCE_DIR}/rplidar/linux_librplidar_sdk.a)
    target_link_libraries(Main2 ${CMAKE_SOURCE_DIR}/rplidar/linux_librplidar_sdk.a)
endif (UNIX)

if (APPLE)
    target_link_libraries(Main ${CMAKE_SOURCE_DIR}/rplidar/librplidar_sdk.a)
    target_link_libraries(Main2 ${CMAKE_SOURCE_DIR}/rplidar/librplidar_sdk.a)
endif (APPLE)

#ADD BACK
target_link_libraries(Main ${OpenCV_LIBS})
target_link_libraries(Main2 ${OpenCV_LIBS})
target_link_libraries(Main ${PYTHON_LIBRARIES})
target_link_libraries(Main2 ${PYTHON_LIBRARIES})
target_link_libraries(Post ${OpenCV_LIBS})
target_link_libraries(Stream ${OpenCV_LIBS})