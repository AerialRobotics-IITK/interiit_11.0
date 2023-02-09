cmake_minimum_required(VERSION 2.8.3)
project(ocam)

add_definitions(-std=c++11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(OpenCV REQUIRED)

add_subdirectory(cpp-python_socket/cpp)
include_directories(
    include
    cpp-python_socket/cpp/cpp_sockets/include
)

add_executable(ocam src/main.cpp src/withrobot_camera.cpp src/withrobot_utility.cpp)
target_link_libraries(ocam ${OpenCV_LIBS} cpp_sockets -lv4l2 -ludev)
