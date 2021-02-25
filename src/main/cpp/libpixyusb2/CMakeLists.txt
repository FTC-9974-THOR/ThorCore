# buildscript for libpixyusb2

cmake_minimum_required(VERSION 3.4.1)

add_library(
        pixyusb2
        SHARED
        src/libpixyusb2.cpp
        src/usblink.cpp
        src/util.cpp
        common/src/chirp.cpp
)
target_include_directories(
        pixyusb2
        PUBLIC ./ common/inc include
        INTERFACE ./
)
set_target_properties(
        pixyusb2
        PROPERTIES
        COMPILE_FLAGS "-D__LINUX__"
)
if(NOT DEFINED ANDROID_LOG_LIB)
    find_library(ANDROID_LOG_LIB log)
endif()
target_link_libraries(
        pixyusb2
        ${ANDROID_LOG_LIB}
        libusb
)