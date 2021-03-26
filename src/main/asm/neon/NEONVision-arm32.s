## NEONVision file for 32-bit ARM processors (armv7-a). At the moment, NEONVision is not supported
## on this architecture. armv7-a doesn't require all processors to have NEON, and those that do are
## significantly weaker than the 64-bit version.
## right now, all methods are just stubs.

#include "JNI.s"

    .arch armv7-a

JNI_EXPAND(vision_NEONVision_supportsNeonAcceleration):
    mov r0, #0
    bx lr

JNI_EXPAND(vision_NEONVision_processYUY2):
    mov r0, #0
    bx lr

JNI_EXPAND(vision_NEONVision_processYUY2ForDisplay):
    mov r0, #0
    bx lr