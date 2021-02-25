#
# Assembly code for SIMD-accelerated vision processing.
# Since I don't have a copy of armclang, I can't write in proper ARM assembly. Thus, this code is
# in a wierd dialect of AArch64, modified to fit llvm's syntax.
# android studio doesn't understand A64 assembly, so it'll complain about some of this code.
# At the moment, this only works on the REV Control Hub.
#

    .text
    .arch armv8-a
    .cpu cortex-a53
    .section .text.neonvision, "ax", @progbits
    .p2align 2
    .globl testcall
    .type testcall, %function
    .globl accumThreshold
    .type accumThreshold, %function
    .globl Java_org_ftc9974_thorcore_vision_NEONVision_processYUY2
    .type Java_org_ftc9974_thorcore_vision_NEONVision_processYUY2, %function

    # test method to make sure the linking process worked
testcall:
    add x0, x0, x1
    add x0, x0, #10
    ret

    # iterate through a provided pixel buffer and counts the number of pixels that fit the specified
    # threshold. pixels must be in ARGB order.
    # args:
    # uint8_t* pixels -> x0
    # uint64_t pixelBufLen -> x1
    # (uint8_t highA, uint8_t highR, uint8_t highG, uint8_t highB)* -> x2
    # (uint8_t lowA, uint8_t lowR, uint8_t lowG, uint8_t lowB)* -> x3
    # uint64_t numPixels -> x4
accumThreshold:
    # store a pointer to the end of the buffer in x5
    add x5, x0, x1
    # prefetch the first block of 16 pixels. cache lines on the cortex A53 are 64 bytes wide.
    prfm PLDL1STRM, [x0]

    # fill comparison vectors. v16-v19 are high thresholds, v20-v23 are low thresholds
    # todo: this could probably be done with a dup instruction instead of passing in a pointer
    ld4r {v16.16b, v17.16b, v18.16b, v19.16b}, [x2]
    ld4r {v20.16b, v21.16b, v22.16b, v23.16b}, [x3]

    # clear accumulator register
    mov x9, #0
accumThreshold_loop:
    # break the loop once we've reached the end of the buffer
    # store the number of bytes left in x6
    subs x6, x5, x0
    # if the result is zero or negative, break. doing this in 2 instructions is a little hard to read,
    # but it's basically just splitting into separate "is zero" and "is negative" checks.
    b.eq accumThreshold_done
    b.mi accumThreshold_done

    # fetching is an expensive operation, so make sure that we'll actually need the next block before
    # we prefetch it. if there's less than 64 bytes (one block) remaining in the buffer, skip the
    # prefetch.
    cmp x6, #64
    b.lt accumThreshold_processBlock
    # prefetch the next block of 16 pixels while we process this block.
    prfm PLDL1STRM, [x0, #64]

accumThreshold_processBlock:
    # load v0, v1, v2, v3 with B, G, R, A, respectively, post-incrementing x0 to point at the next block.
    # the rev hub is little endian, but c++ and java use big endian. this doesn't actually affect anything
    # here, but it will come into play for YCbCr thresholding
    ld4 {v0.16b, v1.16b, v2.16b, v3.16b}, [x0], #64
    # compare each pixel to lows
    cmhs v24.16b, v0.16b, v20.16b
    cmhs v25.16b, v1.16b, v21.16b
    cmhs v26.16b, v2.16b, v22.16b
    cmhs v27.16b, v3.16b, v23.16b
    # compare each pixel to highs
    cmhs v4.16b, v16.16b, v0.16b
    cmhs v5.16b, v17.16b, v1.16b
    cmhs v6.16b, v18.16b, v2.16b
    cmhs v7.16b, v19.16b, v3.16b

    # AND the comparisons together
    and v4.16b, v4.16b, v24.16b
    and v5.16b, v5.16b, v25.16b
    and v6.16b, v6.16b, v26.16b
    and v7.16b, v7.16b, V27.16b

    # at this point, v4-v7 contains the result of the comparison per channel. collapse them
    # into a single vector (placed in v4) that is per pixel instead of per channel.
    and v4.16b, v4.16b, v5.16b
    and v4.16b, v4.16b, v6.16b
    and v4.16b, v4.16b, v7.16b

    # shift right to keep only a single bit, so we can just add across the vector and add it to
    # the existing match sum. (cmhs sets or clears all bits instead of just one)
    ushr v4.16b, v4.16b, #7

    # clear v5 for use as in intermediate accumulator (treating v5 as 2 64-bit integers)
    movi d5, #0
    # add across vector v4 into v5. despite the "b" prefix, this is in fact going into a vector
    # register. llvm just seems to want a size specifier, but apparently not in the typical vX.Q format.
    # maybe this is because the size specifier for general registers is in the instruction mnemonic,
    # not on the register itself (à la x86).
    addv b5, v4.16b
    # move into x10 so it can be added to x9. might be extraneous; i don't know Aarch64 ASM well
    # enough to know.
    umov x10, v5.D[0]
    add x9, x9, x10

    b accumThreshold_loop

accumThreshold_done:
    mov x0, x9

    # correct for padding of the buffer
    # calculate number of pixels in buffer (bytes / 4, equivalent to 2-bit right shift)
    lsr x1, x1, #2
    # calculate number of pixels that were inserted for padding
    sub x5, x1, x4
    # subtract that from x0
    sub x0, x0, x5

    ret

    # process pixels in YUY2 format. in YUY2, every 2 pixels is stored in a 32-bit int. byte order:
    # Y0 | U0 | Y1 | V0 (in big endian)
    # since the SDK stores these in little endian (and the cortex-a53 is little endian), we'll be
    # reading in the reverse order V0 | Y1 | U0 | Y0.
    #
    # in testing, this algorithm runs at ~1100fps with 640x480 images. In practice, this speed is
    # limited by the camera, but we can calculate theoretical speed by taking the reciprocal of
    # per-invocation time. intuitively, this calculation goes from seconds per iteration to iterations
    # per second. from this speed and image size, we can estimate the raw pixels/second the algorithm
    # can handle:
    # fps * image width * image height = 1100 * 640 * 480 = 33,792,000 pixels per second
    #
    # i've seen the algorithm hit 2000+fps, but only when the YUV range is set to encompass every
    # possible color. that range wouldn't be useful in practice, so the peak performance is kind of
    # a moot point.
    #
    # args:
    # JNIEnv* env -> x0
    # jobject obj -> x1
    # jlong bufferPtr -> x2 (interpreted as uint8_t*)
    # jlong bufferLen -> x3 (buffer length, in bytes)
    # jlong highColor -> x4 (YUV order)
    # jlong lowColor -> x5 (YUV order)
Java_org_ftc9974_thorcore_vision_NEONVision_processYUY2:
    # ensure the buffer has at least one block of pixels.
    cmp x3, #64
    b.ge processYUY2_start
    # indicate error code -2
    mov x0, #-2
    ret
processYUY2_start:
    # prefetch the first block. we do it here to maximize the time the cache controller has to read
    # from system memory before the data is needed
    prfm PLDL1STRM, [x2]

    # calculate a pointer to the end of the buffer, storing it in x7
    add x7, x2, x3

    # load high channels into v16-v19, interlacing from YUV to YUY2. this is done by shifting x4 right
    # then moving the low byte into an intermediate register, then duplicating that to a vector. x4
    # is in YUV order, so we copy in VUYY order (copying Y twice, to v17 and v19).
    # first clear x6.
    eor x6, x6, x6
    # V0 -> v16
    mov x6, x4
    and x6, x6, #255
    dup v16.16b, w6
    # U0 -> v18
    orr x6, x4, x4, LSR #8
    and x6, x6, #255
    dup v18.16b, w6
    # Y0 -> v19
    orr x6, x4, x4, LSR #16
    and x6, x6, #255
    dup v19.16b, w6
    # Y1 -> v17
    mov v17.16b, v19.16b

    # we then repeat the process with low channels, loading into v20-v23.
    eor x6, x6, x6
    # V0 -> v20
    mov x6, x5
    and x6, x6, #255
    dup v20.16b, w6
    # U0 -> v22
    orr x6, x5, x5, LSR #8
    and x6, x6, #255
    dup v22.16b, w6
    # Y0 -> v23
    orr x6, x5, x5, LSR #16
    and x6, x6, #255
    dup v23.16b, w6
    # Y1 -> V21
    mov v21.16b, v23.16b

    # clear v24.
    # v24.D[0] is used to accumulate over the entire image.
    # v24.D[1] is used as an intermediate accumulator.
    movi v24.2d, #0

processYUY2_loop:
    # calculate the number of remaining bytes in the buffer, storing it in x6
    subs x6, x7, x2

    # if there's less than one block of pixels remaining in the buffer, break the loop
    cmp x6, #64
    b.lt processYUY2_done

    # prefetch the next block of pixels
    prfm PLDL1STRM, [x2, #64]

# this label doesn't actually do anything, it just helps with organization
processYUY2_processBlock:
    # v0-v3 will store pixel channels.
    # V0 -> v0
    # Y1 -> v1
    # U0 -> v2
    # Y0 -> v3
    # this algorithm operates on "blocks" of the image. each block is 64 bytes and comprises 32 pixels.
    # padding is not needed if the image is in 16:9 and the width is divisible by 2.
    ld4 {v0.16b, v1.16b, v2.16b, v3.16b}, [x2], #64

    # the comparisons follow the inequality high > pixel > low. both high and low checks are thus
    # executed by cmhs.

    # compare to high (high > pixel), storing result in v4-v7.
    # channel: V0
    cmhs v4.16b, v16.16b, v0.16b
    # channel: Y1
    cmhs v5.16b, v17.16b, v1.16b
    # channel: U0
    cmhs v6.16b, v18.16b, v2.16b
    # channel: Y0
    cmhs v7.16b, v19.16b, v3.16b
    # compare to low (pixel > low), storing result in v0-v3. the pixel data isn't needed after this
    # point, which is why we can overwrite it.
    # channel: V0
    cmhs v0.16b, v0.16b, v20.16b
    # channel: Y1
    cmhs v1.16b, v1.16b, v21.16b
    # channel: U0
    cmhs v2.16b, v2.16b, v22.16b
    # channel: Y0
    cmhs v3.16b, v3.16b, v23.16b

    # AND the high and low comparisons together, effectively merging the two inequalities high > pixel
    # and pixel > low into the single inequality high > pixel > low (assuming high > low).
    # in YUY2, U and V channels are shared between 2 pixels, but Y being per-pixel. to do per-pixel
    # thresholding, we can split the YUY2 pixel into 2 separate YUV pixels:
    # YUV0 = {Y0, U0, V0}
    # YUV1 = {Y1, U0, V0}
    # since the U and V channels are the same between YUV0 and YUV1, we only have to do the comparison
    # for U and V once. the Y comparisons are still separate, though.
    # UVcmp = (highU > U0 > lowU) && (highV > V0 > lowV)
    # Y0cmp = (highY > Y0 > lowY)
    # Y1cmp = (highY > Y1 > lowY)
    # YUV0cmp = Y0cmp && UVcmp
    # YUV1cmp = Y1cmp && UVcmp

    # UVcmp -> v0
    # (V0 > lowV) is already in v0.
    # (V0 > lowV) && (highV > V0) -> v0
    and v0.16b, v0.16b, v4.16b
    # (V0 > lowV) && (highV > V0) && (U0 > lowU) -> v0
    and v0.16b, v0.16b, v2.16b
    # (V0 > lowV) && (highV > V0) && (U0 > lowU) && (highU > U0) -> v0
    and v0.16b, v0.16b, v6.16b

    # Y0cmp -> v3
    # (Y0 > lowY) is already in v3.
    # (Y0 > lowY) && (highY > Y0) -> v3
    and v3.16b, v3.16b, v7.16b

    # Y1cmp -> v1
    # (Y1 > lowY) is already in v1.
    # (Y1 > lowY) && (highY > Y1) -> v1
    and v1.16b, v1.16b, v5.16b

    # cmhs sets or clears all bits in a vector, so we right-shift to only keep the highest bit. this
    # also allows us to just ADDV across the vector and add that to the accumulator.
    ushr v3.16b, v3.16b, #7
    ushr v1.16b, v1.16b, #7

    # we can save an instruction by rearranging the intermediate accumulator equation:
    # acc = (Y0cmp && UVcmp) + (Y1cmp && UVcmp)
    # acc = (Y0cmp + Y1cmp) && UVcmp
    # intuitively, this operates on individual YUY2 pixels, instead of sets of 2 YUV pixels.
    # Y0cmp + Y1cmp -> v2
    add v2.16b, v3.16b, v1.16b
    # (Y0cmp + Y1cmp) && UVcmp -> v2
    and v2.16b, v2.16b, v0.16b
    # accumulate into low byte of v0.
    addv b0, v2.16b
    # copy intermediate sum into v24. this can then be interpreted as a 64-bit integer. since v24 is
    # cleared before the loop, we don't need to worry about zero-extending; nothing ever writes to
    # the rest of the 64 bits we're copying into.
    mov v24.B[8], v0.B[0]
    # add intermediate and overall accumulators, then store into overall accumulator
    addp d24, v24.2d

    b processYUY2_loop

processYUY2_done:
    # copy the accumulator into x0 for return
    umov x0, v24.D[0]

    ret