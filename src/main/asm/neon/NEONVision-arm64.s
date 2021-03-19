#include "JNI.s"

    .section .text.neonvision, "ax", @progbits
    .arch armv8-a
    .cpu cortex-a53

JNI_EXPAND(vision_NEONVision_supportsNeonAcceleration):
    mov x0, #1
    ret

    ## processYUY2(long bufferPtr, long bufferLen, long highColor, long lowColor)
JNI_EXPAND(vision_NEONVision_processYUY2):
## USB webcams seem to use the Rec. 601 color space (BT.601). In BT.601, luminace (Y) ranges from
## 16 (black) to 235 (white). U and V are unsigned, with neutral color centered around 128.
##
## symbolic names for registers. i think armasm supports this as an instruction, but i don't have a
## copy of armasm.
#define bufferPtr x2
#define bufferLen x3
#define highColor x4
#define lowColor x5
#define bufferEnd x6

#define Y0 v0.16b
#define U0 v1.16b
#define Y1 v2.16b
#define V0 v3.16b

#define Y0highCmp v16.16b
#define U0highCmp v17.16b
#define Y1highCmp v18.16b
#define V0highCmp v19.16b

#define Y0lowCmp v20.16b
#define U0lowCmp v21.16b
#define Y1lowCmp v22.16b
#define V0lowCmp v23.16b

#define Yhigh v24.16b
#define Uhigh v25.16b
#define Vhigh v26.16b

#define Ylow v27.16b
#define Ulow v28.16b
#define Vlow v29.16b

#define acc v30
#define accScalar d30

    prfm PLDL1STRM, [bufferPtr]

    ## clear x6 for use during threshold loading
    eor x6, x6, x6
    ## load a bitmask into x7. this is used to mask channels out of highColor and lowColor.
    mov x7, #255

    ## load high values from highColor. highColor is in YUV order. we start with the least significant
    ## bits, so we load in VUY order.
    ## clearing the upper bits might be unecessary
    and x6, x7, highColor
    dup Vhigh, w6
    and x6, x7, highColor, LSR #8
    dup Uhigh, w6
    and x6, x7, highColor, LSR #16
    dup Yhigh, w6

    ## load low values from lowColor.
    and x6, x7, lowColor
    dup Vlow, w6
    and x6, x7, lowColor, LSR #8
    dup Ulow, w6
    and x6, x7, lowColor, LSR #16
    dup Ylow, w6

    movi acc.2d, #0

    ## store a pointer to the end of the buffer
    add bufferEnd, bufferPtr, bufferLen

processYUY2_loop:
    cmp bufferPtr, bufferEnd
    b.ge processYUY2_done

    prfm PLDL1STRM, [bufferPtr, #64]

    ld4 {Y0, U0, Y1, V0}, [bufferPtr], #64

    ## compare pixels to high values. high >= pixel
    cmhs Y0highCmp, Yhigh, Y0
    cmhs U0highCmp, Uhigh, U0
    cmhs Y1highCmp, Yhigh, Y1
    cmhs V0highCmp, Vhigh, V0

    ## compare pixels to low values. pixel >= low
    cmhs Y0lowCmp, Y0, Ylow
    cmhs U0lowCmp, U0, Ulow
    cmhs Y1lowCmp, Y1, Ylow
    cmhs V0lowCmp, V0, Vlow

    ## since YUY2 uses macropixels, we're working with 2 pixels here. thus, we'll need 2 conditions
    ## for thresholding.
    ## P0cmp = Y0highCmp && Y0lowCmp && U0highCmp && U0lowCmp && V0highCmp && V0lowCmp
    ## P1cmp = Y1highCmp && Y1lowCmp && U0highCmp && U0lowCmp && V0highCmp && V0lowCmp
    ## we can combine the part into UVcmp:
    ## UVcmp = U0highCmp && U0lowCmp && V0highCmp && V0lowCmp
    ## then each pixel condition has its own luminance threshold:
    ## Y0cmp = Y0highCmp && Y0lowCmp
    ## Y1cmp = Y1highCmp && Y1lowCmp

    ## UVcmp -> v6
    and v6.16b, U0highCmp, U0lowCmp
    and v7.16b, V0highCmp, V0lowCmp
    and v6.16b, v6.16b, v7.16b

    ## Y0cmp -> v4
    and v4.16b, Y0highCmp, Y0lowCmp

    ## Y1cmp -> v5
    and v5.16b, Y1highCmp, Y1lowCmp

    ushr v4.16b, v4.16b, #7
    ushr v5.16b, v5.16b, #7
    ## acc = (Y0cmp + Y1cmp) && (UVcmp)
    add v4.16b, v4.16b, v5.16b
    and v4.16b, v4.16b, v6.16b
    addv b7, v4.16b
    mov acc.B[8], v7.B[0]
    addp accScalar, acc.2d

    b processYUY2_loop

processYUY2_done:
    umov x0, acc.D[0]

    ret
#undef bufferPtr
#undef bufferLen
#undef highColor
#undef lowColor
#undef bufferEnd
#undef Y0
#undef U0
#undef Y1
#undef V0
#undef Y0highCmp
#undef U0highCmp
#undef Y1highCmp
#undef V0highCmp
#undef Y0lowCmp
#undef U0lowCmp
#undef Y1lowCmp
#undef V0lowCmp
#undef Yhigh
#undef Uhigh
#undef Vhigh
#undef Ylow
#undef Ulow
#undef Vlow
#undef acc
#undef accScalar

JNI_EXPAND(vision_NEONVision_processYUY2ForDisplay):
## USB webcams seem to use the Rec. 601 color space (BT.601). In BT.601, luminance (Y) ranges from
## 16 (black) to 235 (white). U and V are unsigned, with neutral color centered around 128.
##
## symbolic names for registers. i think armasm supports this as an instruction, but i don't have a
## copy of armasm.
#define bufferPtr x2
#define bufferLen x3
#define highColor x4
#define lowColor x5
#define bufferEnd x6
#define maskY w9
#define maskU w10
#define maskV w11

#define Y0 v0.16b
#define U0 v1.16b
#define Y1 v2.16b
#define V0 v3.16b

#define Y0highCmp v16.16b
#define U0highCmp v17.16b
#define Y1highCmp v18.16b
#define V0highCmp v19.16b

#define Y0lowCmp v20.16b
#define U0lowCmp v21.16b
#define Y1lowCmp v22.16b
#define V0lowCmp v23.16b

#define Yhigh v24.16b
#define Uhigh v25.16b
#define Vhigh v26.16b

#define Ylow v27.16b
#define Ulow v28.16b
#define Vlow v29.16b

#define acc v30
#define accScalar d30

    prfm PLDL1STRM, [bufferPtr]

    ## clear x6 for use during threshold loading
    eor x6, x6, x6
    ## load a bitmask into x7. this is used to mask channels out of highColor and lowColor.
    mov x7, #255

    ## load high values from highColor. highColor is in YUV order. we start with the least significant
    ## bits, so we load in VUY order.
    ## clearing the upper bits might be unecessary
    and x6, x7, highColor
    dup Vhigh, w6
    and x6, x7, highColor, LSR #8
    dup Uhigh, w6
    and x6, x7, highColor, LSR #16
    dup Yhigh, w6

    ## load low values from lowColor.
    and x6, x7, lowColor
    dup Vlow, w6
    and x6, x7, lowColor, LSR #8
    dup Ulow, w6
    and x6, x7, lowColor, LSR #16
    dup Ylow, w6

    # prefill mask colors
    mov maskY, #16
    mov maskU, #128
    mov maskV, #128

    movi acc.2d, #0

    ## store a pointer to the end of the buffer
    add bufferEnd, bufferPtr, bufferLen

processYUY2ForDisplay_loop:
    cmp bufferPtr, bufferEnd
    b.ge processYUY2ForDisplay_done

    prfm PLDL1STRM, [bufferPtr, #64]

    ld4 {Y0, U0, Y1, V0}, [bufferPtr]

    ## compare pixels to high values. high >= pixel
    cmhs Y0highCmp, Yhigh, Y0
    cmhs U0highCmp, Uhigh, U0
    cmhs Y1highCmp, Yhigh, Y1
    cmhs V0highCmp, Vhigh, V0

    ## compare pixels to low values. pixel >= low
    cmhs Y0lowCmp, Y0, Ylow
    cmhs U0lowCmp, U0, Ulow
    cmhs Y1lowCmp, Y1, Ylow
    cmhs V0lowCmp, V0, Vlow

    ## since YUY2 uses macropixels, we're working with 2 pixels here. thus, we'll need 2 conditions
    ## for thresholding.
    ## P0cmp = Y0highCmp && Y0lowCmp && U0highCmp && U0lowCmp && V0highCmp && V0lowCmp
    ## P1cmp = Y1highCmp && Y1lowCmp && U0highCmp && U0lowCmp && V0highCmp && V0lowCmp
    ## we can combine the part into UVcmp:
    ## UVcmp = U0highCmp && U0lowCmp && V0highCmp && V0lowCmp
    ## then each pixel condition has its own luminance threshold:
    ## Y0cmp = Y0highCmp && Y0lowCmp
    ## Y1cmp = Y1highCmp && Y1lowCmp

    ## UVcmp -> v6
    and v6.16b, U0highCmp, U0lowCmp
    and v7.16b, V0highCmp, V0lowCmp
    and v6.16b, v6.16b, v7.16b

    ## Y0cmp -> v4
    and v4.16b, Y0highCmp, Y0lowCmp

    ## Y1cmp -> v5
    and v5.16b, Y1highCmp, Y1lowCmp

    ## masking logic
    ## fill mask color register with maskY
    dup v31.16b, maskY
    ## v7 = Y0cmp && UVcmp
    and v7.16b, v4.16b, v6.16b
    ## insert mask values into Y0 if pixel 0 doesn't match
    bif Y0, v31.16b, v7.16b
    ## v7 = Y1cmp && UVcmp
    and v7.16b, v5.16b, v6.16b
    ## insert mask values into Y1 if pixel 1 doesn't match
    bif Y1, v31.16b, v7.16b
    ## fill mask color register with maskU
    dup v31.16b, maskU
    ## insert mask values into U0 if UVcmp is false
    bif U0, v31.16b, v6.16b
    ## fill mask color register with maskV
    dup v31.16b, maskV
    ## insert mask values into V0 if UVcmp is false
    bif V0, v31.16b, v6.16b

    st4 {Y0, U0, Y1, V0}, [bufferPtr], #64

    ushr v4.16b, v4.16b, #7
    ushr v5.16b, v5.16b, #7
    ## acc = (Y0cmp + Y1cmp) && (UVcmp)
    add v4.16b, v4.16b, v5.16b
    and v4.16b, v4.16b, v6.16b
    addv b7, v4.16b
    mov acc.B[8], v7.B[0]
    addp accScalar, acc.2d

    b processYUY2ForDisplay_loop

processYUY2ForDisplay_done:
    umov x0, acc.D[0]

    ret
#undef bufferPtr
#undef bufferLen
#undef highColor
#undef lowColor
#undef bufferEnd
#undef maskY
#undef maskU
#undef maskV
#undef Y0
#undef U0
#undef Y1
#undef V0
#undef Y0highCmp
#undef U0highCmp
#undef Y1highCmp
#undef V0highCmp
#undef Y0lowCmp
#undef U0lowCmp
#undef Y1lowCmp
#undef V0lowCmp
#undef Yhigh
#undef Uhigh
#undef Vhigh
#undef Ylow
#undef Ulow
#undef Vlow
#undef acc
#undef accScalar