#include "JNI.s"
    .section .text.neonvision, "ax", @progbits
    .arch armv8-a
    .cpu cortex-a53

# args:
# JNIEnv* -> x0
# long imageBufPtr -> x1
# int imageWidth -> x2[0:31]
# int imageHeight -> x3[0:31]
# int startX -> x4[0:31]
# int startY -> x5[0:31]
# int roiWidth -> x6[0:31]
# int roiHeight -> x7[0:31]
#
JNI_EXPAND(vision_NEONVision_processYUY2ROI):
    stp x29, x30, [sp, -16]!



    ldp x29, x30, [sp], 16
    ret