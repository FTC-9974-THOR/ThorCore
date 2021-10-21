## core file for assembly code using NEON acceleration. depending on what architecture is being targeted
## at compile time, it will pull in the correct version of NEONVision.

#include "JNI.s"

    .text
    .p2align 2

## Export NEONVision functions for JNI.
EXPORT_SYMBOL(vision_NEONVision_supportsNeonAcceleration)
MARK_FUNCTION(vision_NEONVision_supportsNeonAcceleration)
EXPORT_SYMBOL(vision_NEONVision_processYUY2)
MARK_FUNCTION(vision_NEONVision_processYUY2)
EXPORT_SYMBOL(vision_NEONVision_processYUY2ForDisplay)
MARK_FUNCTION(vision_NEONVision_processYUY2ForDisplay)

#ifndef __ARM_NEON
## if we don't have NEON, it doesn't matter what architecture it is.
#include "neon/NEONVisionStubs.s"
#elif __aarch64__
## armv8-a, aka arm64-v8a
#include "neon/NEONVision-arm64.s"
#include "neon/NEONVisionROI-arm64.s"
#else
## armv7-a, aka armeabi-v7a. currently not supported for NEONVision.
#include "neon/NEONVision-arm32.s"
#endif