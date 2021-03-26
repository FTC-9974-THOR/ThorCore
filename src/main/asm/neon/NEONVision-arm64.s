#include "JNI.s"

    ## indicate that this is executable code, stored in its own section of .text
    .section .text.neonvision, "ax", @progbits
    ## indicate that this will be running on armv8-a (or, as Android calls it, arm64-v8a)
    .arch armv8-a
    ## the Rev Control Hub uses an ARM Cortex A53 (specifically, it uses an RK3328, which has an A53
    ## embedded in it), so have the compiler optimize for that processor
    .cpu cortex-a53

    ## Java signature: static boolean supportsNeonAcceleration()
    ## args:
    ## JNIEnv* env -> x0
    ## jclass cls -> x1
JNI_EXPAND(vision_NEONVision_supportsNeonAcceleration):
    ## return true
    mov x0, #1
    ret

    ## Counts the number of pixels in an image that match the given YUV threshold.
    ##
    ## This method iterates through an image buffer in YUY2 format, comparing each pixel in the image
    ## to a color threshold. A pixel matches the threshold if its Y, U, and V channels are lower than
    ## or equal to their respective channels in the highColor argument, and greater than or equal to
    ## their respective channels in the lowColor argument. It's very similar to an RGB or HSV threshold -
    ## just using YUV instead.
    ##
    ## The method returns the number of pixels in the image that match the given threshold.
    ##
    ## The algorithm uses SIMD instructions to parallelize processing.
    ##
    ## The algorithm processes the image in 32 pixel "blocks", which are 64 bytes long. each block
    ## is loaded into the NEON vector registers, and the pixel data is processed in parallel with SIMD
    ## instructions. this allows the algorithm to operate on all 32 pixels at once, compared to the
    ## 2 pixels (a single macropixel) at a time that a non-SIMD implementation would process at - a 16x
    ## performance increase.
    ##
    ## Java signature: long processYUY2(long bufferPtr, long bufferLen, long highColor, long lowColor)
    ## args:
    ## JNIEnv* env -> x0: pointer to JNI function table
    ## jobject obj -> x1: pointer to the calling Java object
    ## byte* bufferPtr -> x2: pointer to the image buffer. the buffer must be in YUY2 format, in
    ##                        YUYV order - that is, bufferPtr[0] = Y0, bufferPtr[1] = U0, bufferPtr[2] = Y1,
    ##                        and so on. each byte is one channel, with 4 bytes encoding 2 pixels.
    ## long bufferLen -> x3: length of the image buffer, in bytes.
    ## long highColor -> x4: high threshold, in YUV, encoded into a 64-bit integer. the V channel is
    ##                       in the least significant byte, the U channel in the second-least significant
    ##                       byte, and the Y channel in the third-least significant byte. in other words,
    ##                       the hexidecimal color in #YYUUVV order.
    ## long lowColor -> x5: low threshold, in YUV, encoded into a 64-bit integer in the same fashion
    ##                      as highColor.
JNI_EXPAND(vision_NEONVision_processYUY2):
## USB webcams seem to use the Rec. 601 color space (BT.601). In BT.601, luminace (Y) ranges from
## 16 (black) to 235 (white). U and V are unsigned, with neutral color centered around 128. That being
## said, the behaviour i've seen in testing doesn't quite match BT.601 - namely, YUV of (0, 0, 0) is
## bright green, but (0, 128, 128) is black. this unfortunately means that the threshold values have
## to be obtained by trial and error using the driver station's camera stream.
##
## symbolic names for registers. i think armasm supports this as an instruction, but i don't have a
## copy of armasm. thus, i'm using the C preprocessor and #define.
#define bufferPtr x2
#define bufferLen x3
#define highColor x4
#define lowColor x5
## pointer to the memory address immediately after the end of the buffer.
#define bufferEnd x6

## Y0 channel of the current pixel block.
#define Y0 v0.16b
## U0 channel of the current pixel block.
#define U0 v1.16b
## Y1 channel of the current pixel block.
#define Y1 v2.16b
## V0 channel of the current pixel block.
#define V0 v3.16b

## stores the result of comparing Y0 to the high Y threshold
#define Y0highCmp v16.16b
## stores the result of comparing U0 to the high U threshold
#define U0highCmp v17.16b
## stores the result of comparing Y1 to the high Y threshold
#define Y1highCmp v18.16b
## stores the result of comparing V0 to the high V threshold
#define V0highCmp v19.16b

## stores the result of comparing Y0 to the low Y threshold
#define Y0lowCmp v20.16b
## stores the result of comparing U0 to the low U threshold
#define U0lowCmp v21.16b
## stores the result of comparing Y1 to the low Y threshold
#define Y1lowCmp v22.16b
## stores the result of comparing V0 to the low V threshold
#define V0lowCmp v23.16b

## stores the high Y threshold
#define Yhigh v24.16b
## stores the high U threshold
#define Uhigh v25.16b
## stores the high V threshold
#define Vhigh v26.16b

## stores the low Y threshold
#define Ylow v27.16b
## stores the low U threshold
#define Ulow v28.16b
## stores the low V threshold
#define Vlow v29.16b

## accumulator for pixel count
#define acc v30
## register view of the low 8 bytes of the accumulator register
#define accScalar d30

    ## prefetch the first block of pixels. on the Cortex A53, cache lines are 64 bytes wide. since
    ## 64 is evenly divisible by 4 (the size in bytes of YUY2 macropixels), it makes a convenient
    ## block size for parallel processing. assuming that bufferPtr is 64-byte aligned, this means
    ## that we can use a PRFM instruction to load a block, starting at bufferPtr.
    ## specifically, load bufferPtr into the L1 cache for streaming.
    prfm PLDL1STRM, [bufferPtr]

    ## clear x6 for use during threshold loading. XOR'ing x6 with itself is functionally equivalent
    ## to mov x6, #0. in some cases, this can even be faster than mov - at least on x86 processors.
    ## truth be told, i don't know if the same can be said for ARM processors, but it makes the syntax
    ## highlighting easier to read.
    eor x6, x6, x6
    ## load a bitmask into x7. this is used to mask channels out of highColor and lowColor, 8 bits at
    ## a time.
    mov x7, #255

    ## load high values from highColor. highColor is in YUV order. we start with the least significant
    ## bits, so we load in VUY order.
    ## put bits 0-7 of highColor in x6
    and x6, x7, highColor
    ## copy that to every byte in the Vhigh register
    dup Vhigh, w6
    ## put bits 8-15 of highColor in x6
    and x6, x7, highColor, LSR #8
    ## copy that to Uhigh
    dup Uhigh, w6
    ## put bits 16-23 of highColor in x6
    and x6, x7, highColor, LSR #16
    ## copy that to Yhigh
    dup Yhigh, w6

    ## now repeat the same process with the low threshold.
    ## put bits 0-7 of lowColor in x6
    and x6, x7, lowColor
    ## copy that to Vlow
    dup Vlow, w6
    ## put bits 8-15 of lowColor in x6
    and x6, x7, lowColor, LSR #8
    ## copy that to Ulow
    dup Ulow, w6
    ## put bits 16-23 of lowColor in x6
    and x6, x7, lowColor, LSR #16
    ## copy that to Ylow
    dup Ylow, w6

    ## clear all bits in acc
    movi acc.2d, #0

    ## store a pointer to the end of the buffer
    add bufferEnd, bufferPtr, bufferLen

processYUY2_loop:
    ## make sure we're not at the end of the buffer (bufferPtr >= bufferEnd)
    cmp bufferPtr, bufferEnd
    b.ge processYUY2_done

    ## ask the memory controller to load the block we'll be processing after the current one
    prfm PLDL1STRM, [bufferPtr, #64]

    ## load the block, deinterlacing into Y0, U0, Y1, and V0.
    ## post-increments bufferPtr by 64
    ld4 {Y0, U0, Y1, V0}, [bufferPtr], #64

    ## compare pixel data to high thresholds, checking if each threshold is greater than or equal to
    ## the pixel data. the result of each channel's comparison is stored into the channel's corresponding
    ## highCmp register.
    cmhs Y0highCmp, Yhigh, Y0
    cmhs U0highCmp, Uhigh, U0
    cmhs Y1highCmp, Yhigh, Y1
    cmhs V0highCmp, Vhigh, V0

    ## compare pixels to low thresholds, checking if each pixel value is greater than or equal to the
    ## low threshold. the result of each channel's comparison is stored into the channel's corresponding
    ## lowCmp register.
    cmhs Y0lowCmp, Y0, Ylow
    cmhs U0lowCmp, U0, Ulow
    cmhs Y1lowCmp, Y1, Ylow
    cmhs V0lowCmp, V0, Vlow

    ## since YUY2 uses macropixels, we're working with 32 pixels here. thus, we'll need 2 conditions
    ## for thresholding each of the 16 macropixels.
    ## P0cmp = Y0highCmp && Y0lowCmp && U0highCmp && U0lowCmp && V0highCmp && V0lowCmp
    ## P1cmp = Y1highCmp && Y1lowCmp && U0highCmp && U0lowCmp && V0highCmp && V0lowCmp
    ## we can combine part of the comparison into UVcmp:
    ## UVcmp = U0highCmp && U0lowCmp && V0highCmp && V0lowCmp
    ## then each pixel condition has its own luminance threshold:
    ## Y0cmp = Y0highCmp && Y0lowCmp
    ## Y1cmp = Y1highCmp && Y1lowCmp

    ## AND together U0highCmp, U0lowCmp, V0highCmp, and V0lowCmp into v6.
    and v6.16b, U0highCmp, U0lowCmp
    and v7.16b, V0highCmp, V0lowCmp
    and v6.16b, v6.16b, v7.16b

    ## store Y0cmp (Y0highCmp && Y0lowCmp) into v4.
    and v4.16b, Y0highCmp, Y0lowCmp

    ## store Y1cmp (Y1highCmp && Y1lowCmp) into v5.
    and v5.16b, Y1highCmp, Y1lowCmp

    ## cmhs sets or clears all bits in the output register. we want to count the number of pixels that
    ## match, so we only need one bit from each comparison, which we can then add together. we can shift
    ## each luminance comparison right 7 bits, leaving only the top bit set.
    ushr v4.16b, v4.16b, #7
    ushr v5.16b, v5.16b, #7
    ## acc = (Y0cmp + Y1cmp) && (UVcmp)
    ## add together Y0cmp and Y1cmp, storing the result in v4
    add v4.16b, v4.16b, v5.16b
    ## AND that with UVcmp. since UVcmp is the same for both pixels in the macropixel, we can add
    ## Y0cmp and Y1cmp, then AND with UVcmp, instead of ANDing twice and adding. this saves an instruction.
    and v4.16b, v4.16b, v6.16b
    ## add across v4 to find the number of pixels in this block that match the threshold, and store it
    ## in the lowest byte of v7.
    addv b7, v4.16b
    ## move the intermediate sum from v7 to the 8th byte of the accumulator.
    mov acc.B[8], v7.B[0]
    ## the accumulator can then be reinterpreted as 2 64-bit integers, with the running accumulator
    ## in the lower half and the intermediate sum in the top half. add across the vector to add the
    ## intermediate sum to the running accumulator.
    addp accScalar, acc.2d

    ## continue the loop
    b processYUY2_loop

processYUY2_done:
    ## move the running accumulator to x0 for returning
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

    ## see the comments for processYUY2. this method does the same thing, but it overwrites pixels
    ## in the image buffer that do not match the threshold.
JNI_EXPAND(vision_NEONVision_processYUY2ForDisplay):
## USB webcams seem to use the Rec. 601 color space (BT.601). In BT.601, luminace (Y) ranges from
## 16 (black) to 235 (white). U and V are unsigned, with neutral color centered around 128. That being
## said, the behaviour i've seen in testing doesn't quite match BT.601 - namely, YUV of (0, 0, 0) is
## bright green, but (0, 128, 128) is black. this unfortunately means that the threshold values have
## to be obtained by trial and error using the driver station's camera stream.
##
## symbolic names for registers. i think armasm supports this as an instruction, but i don't have a
## copy of armasm. thus, i'm using the C preprocessor and #define.
#define bufferPtr x2
#define bufferLen x3
#define highColor x4
#define lowColor x5
## pointer to the memory address immediately after the end of the buffer.
#define bufferEnd x6
## Y value for the mask color
#define maskY w9
## U value for the mask color
#define maskU w10
## V value for the mask color
#define maskV w11

## Y0 channel of the current pixel block.
#define Y0 v0.16b
## U0 channel of the current pixel block.
#define U0 v1.16b
## Y1 channel of the current pixel block.
#define Y1 v2.16b
## V0 channel of the current pixel block.
#define V0 v3.16b

## stores the result of comparing Y0 to the high Y threshold
#define Y0highCmp v16.16b
## stores the result of comparing U0 to the high U threshold
#define U0highCmp v17.16b
## stores the result of comparing Y1 to the high Y threshold
#define Y1highCmp v18.16b
## stores the result of comparing V0 to the high V threshold
#define V0highCmp v19.16b

## stores the result of comparing Y0 to the low Y threshold
#define Y0lowCmp v20.16b
## stores the result of comparing U0 to the low U threshold
#define U0lowCmp v21.16b
## stores the result of comparing Y1 to the low Y threshold
#define Y1lowCmp v22.16b
## stores the result of comparing V0 to the low V threshold
#define V0lowCmp v23.16b

## stores the high Y threshold
#define Yhigh v24.16b
## stores the high U threshold
#define Uhigh v25.16b
## stores the high V threshold
#define Vhigh v26.16b

## stores the low Y threshold
#define Ylow v27.16b
## stores the low U threshold
#define Ulow v28.16b
## stores the low V threshold
#define Vlow v29.16b

## accumulator for pixel count
#define acc v30
## register view of the low 8 bytes of the accumulator register
#define accScalar d30

    ## prefetch the first block of pixels. on the Cortex A53, cache lines are 64 bytes wide. since
    ## 64 is evenly divisible by 4 (the size in bytes of YUY2 macropixels), it makes a convenient
    ## block size for parallel processing. assuming that bufferPtr is 64-byte aligned, this means
    ## that we can use a PRFM instruction to load a block, starting at bufferPtr.
    ## specifically, load bufferPtr into the L1 cache for streaming.
    prfm PLDL1STRM, [bufferPtr]

    ## clear x6 for use during threshold loading. XOR'ing x6 with itself is functionally equivalent
    ## to mov x6, #0. in some cases, this can even be faster than mov - at least on x86 processors.
    ## truth be told, i don't know if the same can be said for ARM processors, but it makes the syntax
    ## highlighting easier to read.
    eor x6, x6, x6
    ## load a bitmask into x7. this is used to mask channels out of highColor and lowColor, 8 bits at
    ## a time.
    mov x7, #255

    ## load high values from highColor. highColor is in YUV order. we start with the least significant
    ## bits, so we load in VUY order.
    ## put bits 0-7 of highColor in x6
    and x6, x7, highColor
    ## copy that to Vhigh
    dup Vhigh, w6
    ## put bits 8-15 of highColor in x6
    and x6, x7, highColor, LSR #8
    ## copy that to Uhigh
    dup Uhigh, w6
    ## put bits 16-23 of highColor in x6
    and x6, x7, highColor, LSR #16
    ## copy that to Yhigh
    dup Yhigh, w6

    ## now repeat the same process with the low threshold.
    ## put bits 0-7 of lowColor in x6
    and x6, x7, lowColor
    ## copy that to Vlow
    dup Vlow, w6
    ## put bits 8-15 of lowColor in x6
    and x6, x7, lowColor, LSR #8
    ## copy that to Ulow
    dup Ulow, w6
    ## put bits 16-23 of lowColor in x6
    and x6, x7, lowColor, LSR #16
    ## copy that to Ylow
    dup Ylow, w6

    ## prefill mask color. any pixel in the image buffer that doesn't match the threshold will be
    ## overwritten with this color. (16, 128, 128) is pure black.
    mov maskY, #16
    mov maskU, #128
    mov maskV, #128

    ## clear all bits in acc
    movi acc.2d, #0

    ## store a pointer to the end of the buffer
    add bufferEnd, bufferPtr, bufferLen

processYUY2ForDisplay_loop:
    ## check if we're at or past the end of the buffer.
    cmp bufferPtr, bufferEnd
    ## if we are, break the loop.
    b.ge processYUY2ForDisplay_done

    ## prefetch the block *after* the current one.
    prfm PLDL1STRM, [bufferPtr, #64]

    ## load the block into Y0, U0, Y1, and V0.
    ld4 {Y0, U0, Y1, V0}, [bufferPtr]

    ## compare pixels to high thresholds, checking if high >= pixel.
    cmhs Y0highCmp, Yhigh, Y0
    cmhs U0highCmp, Uhigh, U0
    cmhs Y1highCmp, Yhigh, Y1
    cmhs V0highCmp, Vhigh, V0

    ## compare pixels to low values, checking if pixel >= low.
    cmhs Y0lowCmp, Y0, Ylow
    cmhs U0lowCmp, U0, Ulow
    cmhs Y1lowCmp, Y1, Ylow
    cmhs V0lowCmp, V0, Vlow

    ## since YUY2 uses macropixels, we're working with 32 pixels here. thus, we'll need 2 conditions
    ## for thresholding each of the 16 macropixels.
    ## P0cmp = Y0highCmp && Y0lowCmp && U0highCmp && U0lowCmp && V0highCmp && V0lowCmp
    ## P1cmp = Y1highCmp && Y1lowCmp && U0highCmp && U0lowCmp && V0highCmp && V0lowCmp
    ## we can combine part of the comparison into UVcmp:
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

    ## store the pixel data back into the buffer, post-incrementing bufferPtr by 64
    st4 {Y0, U0, Y1, V0}, [bufferPtr], #64

    ## cmhs sets or clears all bits in the output register. we want to count the number of pixels that
    ## match, so we only need one bit from each comparison, which we can then add together. we can shift
    ## each luminance comparison right 7 bits, leaving only the top bit set.
    ushr v4.16b, v4.16b, #7
    ushr v5.16b, v5.16b, #7
    ## acc = (Y0cmp + Y1cmp) && (UVcmp)
    add v4.16b, v4.16b, v5.16b
    and v4.16b, v4.16b, v6.16b
    ## add up the number of matches into the lowest byte of v7
    addv b7, v4.16b
    ## move that sum into the 8th byte of acc
    mov acc.B[8], v7.B[0]
    ## add that sum to the running accumulator, storing it back into accc
    addp accScalar, acc.2d

    b processYUY2ForDisplay_loop

processYUY2ForDisplay_done:
    ## move the accumulator to x0 for return
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