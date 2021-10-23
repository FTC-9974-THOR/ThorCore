package org.ftc9974.thorcore.vision;

import androidx.annotation.IntRange;

import org.ftc9974.thorcore.NativeCodeLoader;

/**
 * Hardware-accelerated vision system written in assembly. Uses NEON SIMD instructions to accelerate
 * vision to ludicrous speeds.
 *
 * During testing, the current implementation processed at about 300 million pixels per second. Testing
 * was done with a Logitech C270 webcam, streaming 640x480 images in YUY2 format. The average execution
 * time for the algorithm was 1ms, yielding a theoretical processing speed of
 * (640*480 pixels)/(0.001 seconds) = 307200000 pixels per second.
 *
 * Currently only supported on arm64-v8a ABIs, and only tested on the REV Control Hub.
 *
 * Assembly code is in src/main/asm/neon/NEONVision-arm64.s and src/main/asm/neon/NEONVisionMask-arm64.s
 */
public class NEONVision{

    /**
     * Logging tag
     */
    private static final String TAG = "NEONVision";

    static {
        // ensure the native library is loaded
        NativeCodeLoader.load();
    }

    // experimentally determined thresholds used in testing
    // thresholds for detecting everything *but* the rings:
    // 0-255 0-232 92-145
    // 0-179 0-232 92-145
    // thresholds for detecting the rings:
    // 0-178 0-155 139-162

    /**
     * Encodes a YUV color into a single number.
     *
     * Y is placed in the third-least significant byte.
     * U is placed in the second-least significant byte.
     * V is placed in the least significant byte.
     *
     * The format is similar to Android ColorInts or HTML color codes.
     *
     * @param y y value
     * @param u u value
     * @param v v value
     * @return encoded color
     */
    public static long yuvColorLong(@IntRange(from = 0, to = 255) int y, @IntRange(from = 0, to = 255) int u, @IntRange(from = 0, to = 255) int v) {
        return (y & 0xff) << 16 | (u & 0xff) << 8 | (v & 0xff);
    }

    // these methods do in fact resolve. it's just that android studio doesn't recognize what they're linked to.
    // these methods are actually implemented in assembly, in NEONCore.s, which itself dispatches to
    // NEONVision-arm64.s, NEONVision-arm32.s, and NEONVisionStubs.s, depending on the target architecture.

    /**
     * Checks if NEONVision is supported on this architecture/ABI.
     *
     * If this method returns false, none of the other NEON methods (processYUY2 and processYUY2ForDisplay) will work.
     *
     * @return true if supported, false otherwise.
     */
    @SuppressWarnings({"JavaJniMissingFunction"})
    public static native boolean supportsNeonAcceleration();

    /**
     * Iterates through an image buffer in YUY2 format, comparing each pixel to high and low thresholds
     * and returning the number of pixels that match the threshold.
     *
     * A pixel matches if all color components are within their corresponding range, inclusive.
     *
     * @param bufferPtr pointer to the start of the image buffer
     * @param bufferLen length of the buffer, in bytes
     * @param highColor high threshold color, encoded with yuvColorLong()
     * @param lowColor low threshold color, encoded with yuvColorLong()
     * @return the number of pixels in the image buffer that match the threshold
     */
    @SuppressWarnings("JavaJniMissingFunction")
    public static native long processYUY2(long bufferPtr, long bufferLen, long highColor, long lowColor);

    /**
     * Iterates through an image buffer in YUY2 format, comparing each pixel to high and low thresholds
     * and returning the number of pixels that match the threshold.
     *
     * The data in the image buffer is overwritten to show what the algorithm "sees". Any pixels that
     * do not match are set to black.
     *
     * A pixel matches if all color components are within their corresponding range, inclusive.
     *
     * @param bufferPtr pointer to the start of the image buffer
     * @param bufferLen length of the buffer, in bytes
     * @param highColor high threshold color, encoded with yuvColorLong()
     * @param lowColor low threshold color, encoded with yuvColorLong()
     * @return the number of pixels in the image buffer that match the threshold
     */
    @SuppressWarnings("JavaJniMissingFunction")
    public static native long processYUY2ForDisplay(long bufferPtr, long bufferLen, long highColor, long lowColor);

    @SuppressWarnings("JavaJniMissingFunction")
    public static native long processYUY2WithMask(long bufferPtr, long bufferLen, long highColor, long lowColor, long maskPtr);

    @SuppressWarnings("JavaJniMissingFunction")
    public static native long processYUY2WithMaskForDisplay(long bufferPtr, long bufferLen, long highColor, long lowColor, long maskPtr);
}
