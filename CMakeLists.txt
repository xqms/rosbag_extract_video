
cmake_minimum_required(VERSION 3.11)
project(rosbag_extract_video)

find_package(catkin REQUIRED COMPONENTS
    rosbag
    rosfmt
    sensor_msgs
)

catkin_package()

find_package(Boost REQUIRED COMPONENTS program_options)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
    libavfilter
    libavformat
    libavcodec
    libswresample
    libswscale
    libavutil
)
pkg_check_modules(TURBOJPEG REQUIRED IMPORTED_TARGET
    libturbojpeg
)

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)

add_executable(rosbag_extract_video
    src/rosbag_extract_video.cpp
)
target_link_libraries(rosbag_extract_video
    ${catkin_LIBRARIES}
    Boost::program_options
    PkgConfig::FFMPEG
    PkgConfig::TURBOJPEG
)

install(TARGETS rosbag_extract_video
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
