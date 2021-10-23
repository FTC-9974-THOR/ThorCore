#include "JNI.s"
    .section .text.neonvision, "ax", @progbits
    .arch armv8-a
    .cpu cortex-a53

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
    ## byte* maskPtr -> x6: pointer to the mask buffer. every pixel in the input image must have a
    ##                      corresponding element in the mask. if a pixel matches the color bounds,
    ##                      the corresponding value in the mask is added to the accumulator.
JNI_EXPAND(vision_NEONVision_processYUY2WithMask):
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
#define acc x10

## weights from the mask
#define Y0weights v8.16b
#define Y1weights v9.16b
## pointer to the current mask block
#define maskPtr x9

    ## push stack frame
    stp x29, x30, [sp, #-16]!
    ## save v8 and v9, as AAPCS64 requires they be callee-saved
    stp q8, q9, [sp, #-32]!
    ## save x9 and x10
    stp x9, x10, [sp, #-16]!

    ## save the pointer to the mask into maskPtr.
    mov maskPtr, x6

    ## prefetch the first block of pixels. on the Cortex A53, cache lines are 64 bytes wide. since
    ## 64 is evenly divisible by 4 (the size in bytes of YUY2 macropixels), it makes a convenient
    ## block size for parallel processing. assuming that bufferPtr is 64-byte aligned, this means
    ## that we can use a PRFM instruction to load a block, starting at bufferPtr.
    ## specifically, load bufferPtr into the L1 cache for streaming.
    prfm PLDL1STRM, [bufferPtr]
    ## load the first mask block
    prfm PLDL1STRM, [maskPtr]

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

    ## set acc to zero
    eor acc, acc, acc

    ## store a pointer to the end of the buffer
    add bufferEnd, bufferPtr, bufferLen

processYUY2Mask_loop:
    ## make sure we're not at the end of the buffer (bufferPtr >= bufferEnd)
    cmp bufferPtr, bufferEnd
    b.ge processYUY2Mask_done

    ## ask the memory controller to load the block we'll be processing after the current one
    prfm PLDL1STRM, [bufferPtr, #64]
    ## also load the next mask block.
    prfm PLDL1STRM, [maskPtr, #32]

    ## load the block, deinterlacing into Y0, U0, Y1, and V0.
    ## post-increments bufferPtr by 64
    ld4 {Y0, U0, Y1, V0}, [bufferPtr], #64
    ## load the mask block, deinterlacing into Y0weights and Y1weights.
    ld2 {Y0weights, Y1weights}, [maskPtr], #32

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
    ## apply Y0weights to Y0
    and v4.16b, v4.16b, Y0weights

    ## store Y1cmp (Y1highCmp && Y1lowCmp) into v5.
    and v5.16b, Y1highCmp, Y1lowCmp
    ## apply Y1Weights to Y1
    and v5.16b, v5.16b, Y1weights

    ## acc += Y0cmp && UVcmp + Y1cmp && UVcmp
    ## since we're using signed arithmetic, we need to handle this a bit differently than the non-mask
    ## version of NEONVision. if we were to just add Y0cmp and Y1cmp, we run the risk of
    ## overflowing and getting the wrong sum. so we add across the 2 vectors and add them to acc
    ## separately.
    ## AND together Y0cmp and UVcmp, placing the result in v4
    and v4.16b, v4.16b, v6.16b
    ## add across v4, placing the result in the lowest halfword of v7
    saddlv h7, v4.16b
    ## sign extend into x7
    smov x7, v7.H[0]
    ## add to the accumulator
    add acc, acc, x7
    ## AND together Y1cmp and UVcmp, placing the result in v5
    and v5.16b, v5.16b, v6.16b
    ## add across v5
    saddlv h7, v5.16b
    ## sign extend into x7
    smov x7, v7.H[0]
    ## add to the accumulator
    add acc, acc, x7

    ## continue the loop
    b processYUY2Mask_loop

processYUY2Mask_done:
    ## move the running accumulator to x0 for returning
    mov x0, acc

    ## restore x9 and x10
    ldp x9, x10, [sp], #16
    ## restore v8 and v9
    ldp q8, q9, [sp], #32
    ## pop stack frame
    ldp x29, x30, [sp], #16

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
#undef Y0weights
#undef Y1weights
#undef maskPtr

JNI_EXPAND(vision_NEONVision_processYUY2WithMaskForDisplay):
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
#define acc x10

## weights from the mask
#define Y0weights v8.16b
#define Y1weights v10.16b
## pointer to the current mask block
#define maskPtr x9

    ## push stack frame
    stp x29, x30, [sp, #-16]!
    ## save v8 and v9, as AAPCS64 requires they be callee-saved
    stp q8, q9, [sp, #-32]!
    ## save v10 and v11
    stp q10, q11, [sp, #-32]!
    ## save x9 and x10
    stp x9, x10, [sp, #-16]!

    ## save the pointer to the mask into maskPtr.
    mov maskPtr, x6

    ## prefetch the first block of pixels. on the Cortex A53, cache lines are 64 bytes wide. since
    ## 64 is evenly divisible by 4 (the size in bytes of YUY2 macropixels), it makes a convenient
    ## block size for parallel processing. assuming that bufferPtr is 64-byte aligned, this means
    ## that we can use a PRFM instruction to load a block, starting at bufferPtr.
    ## specifically, load bufferPtr into the L1 cache for streaming.
    prfm PLDL1STRM, [bufferPtr]
    ## load the first mask block
    prfm PLDL1STRM, [maskPtr]

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

    ## set acc to zero
    eor acc, acc, acc

    ## preload write-back U0 and V0 with 128
    movi v9.16b, #128
    movi v11.16b, #128

    ## store a pointer to the end of the buffer
    add bufferEnd, bufferPtr, bufferLen

processYUY2MaskForDisplay_loop:
    ## make sure we're not at the end of the buffer (bufferPtr >= bufferEnd)
    cmp bufferPtr, bufferEnd
    b.ge processYUY2MaskForDisplay_done

    ## ask the memory controller to load the block we'll be processing after the current one. this
    ## also prevents a cache miss if bufferPtr is not 64-byte aligned, as it'll force the entirety
    ## of the current block to load along with the next block.
    prfm PLDL1STRM, [bufferPtr, #64]
    ## also load the next mask block.
    prfm PLDL1STRM, [maskPtr, #32]

    ## load the block, deinterlacing into Y0, U0, Y1, and V0.
    ## we don't increment the bufferPtr here, since we'll do that during write-back.
    ld4 {Y0, U0, Y1, V0}, [bufferPtr]

    ## tell the cache controller we'll be writing back to bufferPtr soon
    prfm PSTL1STRM, [bufferPtr]

    ## load the mask block, deinterlacing into v16 and v17. those are typically used for storing
    ## Y0highCmp and U0highCmp, so we can overwrite them, as we're going to be storing new values in
    ## them soon anyways. we can't load directly into Y0weights and Y1weights because Load-Structure
    ## instructions require the SIMD registers to be sequential.
    ld2 {Y0highCmp, U0highCmp}, [maskPtr], #32
    ## move mask values into their correct registers
    mov Y0weights, Y0highCmp
    mov Y1weights, U0highCmp

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
    ## apply Y0weights to Y0cmp
    and v4.16b, v4.16b, Y0weights

    ## store Y1cmp (Y1highCmp && Y1lowCmp) into v5.
    and v5.16b, Y1highCmp, Y1lowCmp
    ## apply Y1Weights to Y1cmp
    and v5.16b, v5.16b, Y1weights

    ## write-back logic: v8-v11 are used to write back to the image buffer.
    ## Y0 -> v8
    ## U0 -> v9
    ## Y1 -> v10
    ## V0 -> v11
    ## this is done so they can be written back with a single st4 instruction. Store-Structure
    ## instructions require the registers they use to be sequential. this register layout allows us
    ## to write back to the buffer without needing to shuffle data between registers.

    ## apply UVcmp to Y0cmp, placing it in Y0weights
    and Y0weights, v4.16b, v6.16b
    ## apply UVcmp to Y1cmp, placing it in Y1weights
    and Y1weights, v5.16b, v6.16b
    ## write back to the image buffer, post-incrementing by 64
    st4 {Y0weights, v9.16b, Y1weights, v11.16b}, [bufferPtr], #64

    ## acc += Y0cmp && UVcmp + Y1cmp && UVcmp
    ## since we're using signed arithmetic, we need to handle this a bit differently than the non-mask
    ## version of NEONVision. if we were to just add Y0weights and Y1weights, we run the risk of
    ## overflowing and getting the wrong sum. so we add across the 2 vectors and add them to acc
    ## separately. we've already done the AND with UVcmp, so we can reuse that.
    ## add across Y0weights, placing the result in the lowest halfword of v7
    saddlv h7, Y0weights
    ## sign extend into x7
    smov x7, v7.H[0]
    ## add to the accumulator
    add acc, acc, x7
    ## add across Y1weights
    saddlv h7, Y1weights
    ## sign extend into x7
    smov x7, v7.H[0]
    ## add to the accumulator
    add acc, acc, x7

    ## continue the loop
    b processYUY2MaskForDisplay_loop

processYUY2MaskForDisplay_done:
    ## move the running accumulator to x0 for returning
    mov x0, acc

    ## restore x9 and x10
    ldp x9, x10, [sp], #16
    ## restore v10 and v11
    ldp q10, q11, [sp], #32
    ## restore v8 and v9
    ldp q8, q9, [sp], #32
    ## pop stack frame
    ldp x29, x30, [sp], #16

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
#undef Y0weights
#undef Y1weights
#undef maskPtr