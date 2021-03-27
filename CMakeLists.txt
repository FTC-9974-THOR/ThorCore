# top-level buildscript for all native code in ThorCore.

cmake_minimum_required(VERSION 3.4.1)
enable_language(ASM)

set(NATIVE_ROOT src/main/cpp)
set(ASM_ROOT src/main/asm)
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
        ${NATIVE_ROOT}/JNIPixy2USB.cpp
        ${NATIVE_ROOT}/JNIUtilities.cpp
        ${ASM_ROOT}/neon/NEONCore.s
        ${NATIVE_ROOT}/ThorCore.cpp
)
set_source_files_properties(
        ${ASM_ROOT}/JNI.s
        ${ASM_ROOT}/neon/NEONCore.s
        ${ASM_ROOT}/neon/NEONVision-arm64.s
        ${ASM_ROOT}/neon/NEONVision-arm32.s
        ${ASM_ROOT}/neon/NEONVisionStubs.s
        PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp"
)

#if(${USE_NEON_ACCEL})
#    target_sources(
#            ThorCore
#            PRIVATE
#            ${NATIVE_ROOT}/NEONVision.s
#    )
#    set_source_files_properties(${NATIVE_ROOT}/NEONVision.s PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")
#else()
#    target_sources(
#            ThorCore
#            PRIVATE
#            ${NATIVE_ROOT}/NEONVisionStubs.s
#    )
#endif()

target_include_directories(
        ThorCore
        PRIVATE ${NATIVE_ROOT} ${ASM_ROOT}
)
set_target_properties(
        ThorCore
        PROPERTIES LIBRARY_OUTPUT_DIRECTORY
        ${OUTPUT_DIR}
)
#target_compile_options(
#        ThorCore
#        PRIVATE
#        --verbose
#)

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