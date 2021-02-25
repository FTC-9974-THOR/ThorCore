# stub symbols file for armeabi-v7a. at the moment, NEONVision does not support armeabi-v7a, but it
# still needs to compile and link. this file just declares stub symbols that Java and C++ can call,
# but actually do nothing.

    .text
    .section .text.neonvision
    .p2align 2
    .globl testcall
    .type testcall, %function
    .globl accumThreshold
    .type accumThreshold, %function
    .globl Java_org_ftc9974_thorcore_vision_NEONVision_processYUY2
    .type Java_org_ftc9974_thorcore_vision_NEONVision_processYUY2, %function

testcall:
    mov r0, #-1
    bx lr
accumThreshold:
    mov r0, #-1
    bx lr
Java_org_ftc9974_thorcore_vision_NEONVision_processYUY2:
    mov r0, #-1
    bx lr