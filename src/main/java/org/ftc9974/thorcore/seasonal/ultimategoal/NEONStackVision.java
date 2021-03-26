package org.ftc9974.thorcore.seasonal.ultimategoal;

import android.os.Build;

import androidx.annotation.FloatRange;
import androidx.annotation.IntRange;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.ftc9974.thorcore.internal.RealizableFactory;
import org.ftc9974.thorcore.robot.sensors.USBWebcamBase;
import org.ftc9974.thorcore.vision.NEONVision;

import java.io.IOException;

/**
 * Hardware-accelerated vision system for determining stack height in autonomous.
 *
 * This system uses NEONVision, which uses hand-tuned assembly and NEON SIMD instructions to accelerate
 * vision processing. In practice, the performance of this algorithm is limited only by the framerate
 * of the camera. The vision algorithm itself is very efficient, taking about 1ms on average to process
 * a frame.
 *
 * Note that this system only works with webcams, and not builtin phone cameras. This is because the
 * algorithm requires YUY2 formatted images, which USB cameras must support (according to the UVC spec).
 * While it is possible to use this kind of algorithm with RGB images, that would require converting
 * to YUV, which would incur a severe performance loss.
 *
 * The algorithm works as follows:
 * <ol>
 *     <li>Capture a frame from the camera. This frame must be in YUY2 format. Since the USB Camera
 *     class requires cameras to support this format, it will always be available.</li>
 *     <li>Apply NEONVision's processYUY2 to the frame. This iterates through every pixel in the
 *     image, and counts the number of pixels that are within a specified range of YUV values. In this
 *     use case, we give it thresholds such that it only matches the rings.</li>
 *     <li>The result of processYUY2 is (roughly) the number of pixels in the image that are part of
 *     the ring stack. By dividing that result by the number of pixels we expect a full stack to have
 *     (fullStackMatch), we get a "match factor". Intuitively, this match factor is a measure of how
 *     much "stack" is in frame.</li>
 *     <li>The match factor can be used to determine how many rings are in frame. If a match factor
 *     of 1 is 4 rings, then (theoretically) a match factor of 0.25 is 1 ring, and a match factor
 *     of 0 is 0 rings. To do this, the algorithm uses thresholds (fourStackMin and oneStackMin).</li>
 * </ol>
 *
 * @see NEONVision
 */
public class NEONStackVision extends USBWebcamBase {

    /**
     * Tag used for logging.
     */
    private static final String TAG = "NEONStackVision";

    /**
     * The number of pixels that match when there are four rings in the stack.
     */
    private double fullStackMatch;

    /**
     * Minimum match factor for a four stack.
     *
     */
    @FloatRange(from = 0, to = 1)
    private double fourStackMin;

    /**
     * Minimum match factor for a one stack.
     */
    @FloatRange(from = 0, to = 1)
    private double oneStackMin;

    /**
     * Low side of the YUV threshold, encoded in #YYUUVV order.
     *
     * @see NEONVision#yuvColorLong(int, int, int)
     */
    private long lowColor;

    /**
     * High side of the YUV threshold, encoded in #YYUUVV order.
     *
     * @see NEONVision#yuvColorLong(int, int, int)
     */
    private long highColor;

    /**
     * Timer used for calculating performance metrics.
     */
    private final ElapsedTime timer;

    /**
     * Last output of the algorithm.
     */
    // marked volatile because onNewFrame() is called from a background thread
    private volatile StackHeight stackHeight;

    /**
     * Raw result from NEONVision. This is the number of pixels in the image that match the threshold.
     */
    // marked volatile because onNewFrame() is called from a background thread
    private volatile long result;

    /**
     * Creates a new NEONStackVision with the specified camera name.
     *
     * @param name name of the USB camera to use
     * @param hw hardware map to look for the camera in
     * @throws IOException if there was a problem opening the camera
     */
    @RealizableFactory
    public NEONStackVision(String name, HardwareMap hw) throws IOException {
        this(hw.get(WebcamName.class, name));
    }

    /**
     * Creates a new NEONStackVision with the specified camera.
     *
     * @param webcamName name of the USB camera to use
     * @throws IOException if there was a problem opening the camera
     */
    public NEONStackVision(WebcamName webcamName) throws IOException {
        // initialize the webcam
        super(webcamName);

        // check if this robot controller supports NEONVision. currently, this has only been tested on
        // the REV Control Hub, but should work on anything using a 64-bit ARM processor. the code's
        // been optimized for a Cortex A53 specifically, but it will run on anything using the
        // arm64-v8a ABI.
        // I haven't been able to test this on the phones, unfortunately, but I believe most phones
        // in FTC should work.
        if (!NEONVision.supportsNeonAcceleration()) {
            throw new UnsupportedOperationException(String.format("NEONVision is currently not supported on CPU architecture \"%s\"", Build.SUPPORTED_ABIS[0]));
        }

        // default tunings. these are what i used in testing, but will probably need to be tuned for
        // your auto with the setFullStackMatch(), setFourStackMin(), and setOneStackMin() methods.
        fullStackMatch = 10000;
        fourStackMin = 0.5;
        oneStackMin = 0.35;

        // color thresholds. (hopefully) these should work in most cases. I've yet to pin down exactly
        // what color space the images are in, so these were determined by trial and error. if you do
        // have to change them, you can use the setThresholds() method.
        lowColor = NEONVision.yuvColorLong(0, 0, 139);
        highColor = NEONVision.yuvColorLong(178, 155, 162);

        // instantiate the performance timer
        timer = new ElapsedTime();
    }

    /**
     * Sets the number of pixels that a typical stack of 4 rings will match during thresholding.
     *
     * @param fullStackMatch matching pixel count
     */
    public void setFullStackMatch(double fullStackMatch) {
        this.fullStackMatch = fullStackMatch;
    }

    /**
     * Gets the number of pixels that a typical stack of 4 rings will match during thresholding.
     *
     * @return matching pixel count
     */
    public double getFullStackMatch() {
        return fullStackMatch;
    }

    /**
     * Sets the minimum match factor for a four stack.
     *
     * @param fourStackMin match factor, from 0 to 1.
     */
    public void setFourStackMin(@FloatRange(from = 0, to = 1) double fourStackMin) {
        this.fourStackMin = fourStackMin;
    }

    /**
     * Gets the minimum match factor for a four stack.
     *
     * @return match factor, from 0 to 1.
     */
    public double getFourStackMin() {
        return fourStackMin;
    }

    /**
     * Sets the minimum match factor for a one stack.
     *
     * @param oneStackMin match factor
     */
    public void setOneStackMin(@FloatRange(from = 0, to = 1) double oneStackMin) {
        this.oneStackMin = oneStackMin;
    }

    /**
     * Gets the minimum match factor for a one stack.
     *
     * @return match factor
     */
    public double getOneStackMin() {
        return oneStackMin;
    }

    /**
     * Sets the color thresholds.
     *
     * @param lowY lowest Y value to match
     * @param highY highest Y value to match
     * @param lowU lowest U value to match
     * @param highU highest U value to match
     * @param lowV lowest V value to match
     * @param highV highest V value to match
     */
    public void setThresholds(@IntRange(from = 0, to = 255) int lowY, @IntRange(from = 0, to = 255) int highY,
                              @IntRange(from = 0, to = 255) int lowU, @IntRange(from = 0, to = 255) int highU,
                              @IntRange(from = 0, to = 255) int lowV, @IntRange(from = 0, to = 255) int highV) {
        lowColor = NEONVision.yuvColorLong(lowY, lowU, lowV);
        highColor = NEONVision.yuvColorLong(highY, highU, highV);
    }

    /**
     * Runs the vision algorithm on a frame, storing the result in the stackHeight and result variables.
     *
     * @param frame camera frame
     */
    @Override
    protected void onNewFrame(CameraFrame frame) {
        // grab a timestamp so we can time the algorithm
        long start = System.nanoTime();
        // if we're streaming to the driver station, run a special version of the algorithm. this
        // version writes back into the image buffer to show what the algorithm's doing, but it's
        // much less efficient. thus, we run it only if necessary.
        // when we actually invoke the algorithm, we're making a call directly to assembly with the JNI.
        // we pass it a pointer to the image buffer, the size of said buffer, and the threshold colors.
        // the frame is actually just a wrapper around a native uvc_frame object, so we can't directly
        // access the buffer in Java. however, we can get a pointer to the buffer, which we can then
        // pass to our own native code. since JNI functions run in the same memory addressing space,
        // regardless of what Java class they come from, we can access the buffer without causing a
        // segfault.
        if (isStreaming()) {
            result = NEONVision.processYUY2ForDisplay(frame.getImageBuffer(), frame.getImageSize(), highColor, lowColor);
        } else {
            result = NEONVision.processYUY2(frame.getImageBuffer(), frame.getImageSize(), highColor, lowColor);
        }
        // calculate the time elapsed, in seconds.
        double elapsed = (System.nanoTime() - start) * 1e-9;
        // get the time since the last invocation of onNewFrame(). this gives an estimate of the
        // performance of the vision system overall. typically, the camera framerate is the bottleneck,
        // hence the name "cameraElapsed".
        double cameraElapsed = timer.seconds();
        // reset the timer to time the next invocation
        timer.reset();
        // log the performance metrics. fps is calculated by taking the reciprocal of elapsed time (seconds per frame to frames per second)
        RobotLog.dd(TAG, "Execution time: %f seconds Instantaneous FPS: %f Time since last frame: %f Camera FPS: %f",
                elapsed, 1 / elapsed, cameraElapsed, 1 / cameraElapsed);

        // calculate the match factor, as described in the class-level javadoc.
        double factor = result / fullStackMatch;
        // threshold logic
        if (factor > fourStackMin) {
            // if factor is at least fourStackMin, there are four rings
            stackHeight = StackHeight.FOUR;
        } else if (factor > oneStackMin) {
            // if factor is at least oneStackMin, there is one ring
            stackHeight = StackHeight.ONE;
        } else {
            // otherwise there's no rings
            stackHeight = StackHeight.ZERO;
        }
    }

    /**
     * Checks if the vision system has received at least one frame from the camera and completed
     * processing it.
     *
     * @return whether or not the vision system has determined stack height yet
     */
    public boolean hasCompletedProcessing() {
        return stackHeight != null;
    }

    /**
     * Gets the height of the stack determined by the last iteration of the algorithm, or null if the
     * vision system has not yet completed processing its first frame.
     *
     * @return stack height, or null if the vision system is still processing
     */
    public @Nullable StackHeight getStackHeight() {
        return stackHeight;
    }

    /**
     * Gets the raw result from the algorithm. This is the number of pixels in the last frame that
     * match the color threshold.
     *
     * @return matching pixel count
     */
    public long getRawResult() {
        return result;
    }
}
