# buildscript for libusb

cmake_minimum_required(VERSION 3.4.1)
find_library(ANDROID_LOG_LIB log)

add_library(
    libusb
    SHARED
    core.c
    descriptor.c
    hotplug.c
    io.c
    sync.c
    strerror.c
    os/linux_usbfs.c
    os/events_posix.c
    os/threads_posix.c
    os/linux_netlink.c
)
target_include_directories(
    libusb
    # includes needed to compile libusb
    PRIVATE ./ ./os
    # publish the includes needed to compile against libusb
    INTERFACE ./
)
set_target_properties(
        libusb
        PROPERTIES
        COMPILE_FLAGS "-pthread"
)
target_link_libraries(
        libusb
        ${ANDROID_LOG_LIB}
)