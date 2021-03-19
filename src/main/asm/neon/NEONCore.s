## core file for assembly code using NEON acceleration
#include "JNI.s"

    .text
    .p2align 2

## NEONVision functions
EXPORT_SYMBOL(vision_NEONVision_supportsNeonAcceleration)
MARK_FUNCTION(vision_NEONVision_supportsNeonAcceleration)
EXPORT_SYMBOL(vision_NEONVision_processYUY2)
MARK_FUNCTION(vision_NEONVision_processYUY2)
EXPORT_SYMBOL(vision_NEONVision_processYUY2ForDisplay)
MARK_FUNCTION(vision_NEONVision_processYUY2ForDisplay)

#ifndef __ARM_NEON
#include "neon/NEONVisionStubs.s"
#elif __aarch64__
#include "neon/NEONVision-arm64.s"
#else
#include "neon/NEONVision-arm32.s"
#endif