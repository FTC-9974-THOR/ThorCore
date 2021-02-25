# top-level buildscript for all native code in ThorCore.

cmake_minimum_required(VERSION 3.4.1)
enable_language(ASM)

set(NATIVE_ROOT src/main/cpp)
set(OUTPUT_DIR lib/${ANDROID_ABI})

# there's got to be a cleaner way to do this that i'm just unaware of
if(${ANDROID_ABI} STREQUAL "arm64-v8a")
    set(USE_NEON_ACCEL true)
else()
    set(USE_NEON_ACCEL false)
endif()

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
        ${NATIVE_ROOT}/JNIPixy2USB.cpp
        ${NATIVE_ROOT}/JNIUtilities.cpp
        ${NATIVE_ROOT}/JNINEONVision.cpp
        ${NATIVE_ROOT}/ThorCore.cpp
)
if(${USE_NEON_ACCEL})
    target_sources(
            ThorCore
            PRIVATE
            ${NATIVE_ROOT}/NEONVision.s
    )
else()
    target_sources(
            ThorCore
            PRIVATE
            ${NATIVE_ROOT}/NEONVisionStubs.s
    )
endif()
target_include_directories(
        ThorCore
        PRIVATE ${NATIVE_ROOT}
)
set_target_properties(
        ThorCore
        PROPERTIES LIBRARY_OUTPUT_DIRECTORY
        ${OUTPUT_DIR}
)
target_compile_definitions(
        ThorCore
        PRIVATE
        -DUSE_NEON_ACCEL=${USE_NEON_ACCEL}
)
target_compile_options(
        ThorCore
        PRIVATE
        --verbose
)

if(NOT DEFINED ANDROID_LOG_LIB)
    find_library(ANDROID_LOG_LIB, log)
endif()
set_target_properties(
        ThorCore
        PROPERTIES
        LOCAL_CERTIFICATE platform
)
# linking dependencies, as well as INTERFACE_INCLUDE_DIRECTORIES, come from target_link_libraries
target_link_libraries(
        ThorCore
        ${ANDROID_LOG_LIB}
        fmt
        pixyusb2
)