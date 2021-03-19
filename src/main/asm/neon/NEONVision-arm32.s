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