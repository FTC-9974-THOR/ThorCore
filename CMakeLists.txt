# top-level buildscript for all native code in ThorCore.

cmake_minimum_required(VERSION 3.4.1)

set(NATIVE_ROOT src/main/cpp)
set(OUTPUT_DIR lib/${ANDROID_ABI})

add_subdirectory(
        # path to libusb source (and the buildscript)
        ${NATIVE_ROOT}/libusb
        # directory to place build outputs in
        ${OUTPUT_DIR}/libusb
)

add_subdirectory(
        ${NATIVE_ROOT}/fmt
        ${OUTPUT_DIR}/fmt
)

add_subdirectory(
        ${NATIVE_ROOT}/libpixyusb2
        ${OUTPUT_DIR}/libpixyusb2
)

# build top-level library object
add_library(
        ThorCore
        SHARED
        ${NATIVE_ROOT}/test.cpp
        ${NATIVE_ROOT}/JNIPixy2USB.cpp
        ${NATIVE_ROOT}/JNIUtilities.cpp
)
target_include_directories(
        ThorCore
        PRIVATE ${NATIVE_ROOT}
)
set_target_properties(
        ThorCore
        PROPERTIES LIBRARY_OUTPUT_DIRECTORY
        ${OUTPUT_DIR}
)
# linking dependencies, as well as INTERFACE_INCLUDE_DIRECTORIES, come from target_link_libraries
target_link_libraries(
        ThorCore
        fmt
        pixyusb2
)