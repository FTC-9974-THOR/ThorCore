## Stubs file for NEONVision, used in the event that the target architecture doesn't support NEON
## instructions.

#include "JNI.s"

JNI_EXPAND(vision_NEONVision_supportsNeonAcceleration):
    mov r0, #0
    ret

JNI_EXPAND(vision_NEONVision_processYUY2):
    mov r0, #0
    ret

JNI_EXPAND(vision_NEONVision_processYUY2ForDisplay):
    mov r0, #0
    ret