package org.ftc9974.thorcore.seasonal.freightfrenzy;

import android.os.Build;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.ftc9974.thorcore.robot.sensors.USBWebcamBase;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.vision.NEONVision;
import org.ftc9974.thorcore.vision.NativeImageByteBuffer;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicLong;

public class NEONMarkerVision extends USBWebcamBase {

    public enum MarkerPosition {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }

    public final NativeImageByteBuffer leftMask, centerMask, rightMask;

    private long highColor, lowColor;

    private final AtomicLong leftResult, centerResult, rightResult;

    private volatile MarkerPosition position;
    private volatile MarkerPosition displayPosition;

    private volatile boolean invert;
    private volatile long leftInversionReference, centerInversionReference, rightInversionReference;

    private volatile double executionTime;

    public NEONMarkerVision(WebcamName webcam) throws IOException {
        super(webcam);

        if (!NEONVision.supportsNeonAcceleration()) {
            throw new UnsupportedOperationException(String.format("NEONVision is currently not supported on CPU architecture \"%s\"", Build.SUPPORTED_ABIS[0]));
        }

        Size size = getFrameSize();
        if (size == null) {
            throw new IOException("Internal Error: getFrameSize() returned null with a purportedly open webcam");
        }

        // a note about memory usage: these are *very* memory-hungry. with a 640x480 camera, these
        // three masks will eat up nearly a megabyte of memory.
        leftMask = new NativeImageByteBuffer(size.getWidth(), size.getHeight());
        centerMask = new NativeImageByteBuffer(size.getWidth(), size.getHeight());
        rightMask = new NativeImageByteBuffer(size.getWidth(), size.getHeight());

        leftMask.setAll((byte) 0);
        centerMask.setAll((byte) 0);
        rightMask.setAll((byte) 0);

        leftResult = new AtomicLong(0);
        centerResult = new AtomicLong(0);
        rightResult = new AtomicLong(0);

        position = displayPosition = MarkerPosition.UNKNOWN;

        leftInversionReference = centerInversionReference = rightInversionReference = size.getWidth() * size.getHeight();
    }

    @Override
    protected void onNewFrame(CameraFrame frame) {
        long localLeftResult, localCenterResult, localRightResult;
        localLeftResult = localCenterResult = localRightResult = 0;

        long start = System.nanoTime();
        if (isStreaming()) {
            // make sure displayPosition doesn't get changed while we're using it
            final MarkerPosition localDisplayPosition = displayPosition;
            if (localDisplayPosition != MarkerPosition.LEFT) {
                localLeftResult = NEONVision.processYUY2WithMask(
                        frame.getImageBuffer(), frame.getImageSize(),
                        highColor, lowColor,
                        leftMask.getPointer()
                );
            }
            if (localDisplayPosition != MarkerPosition.CENTER) {
                localCenterResult = NEONVision.processYUY2WithMask(
                        frame.getImageBuffer(), frame.getImageSize(),
                        highColor, lowColor,
                        centerMask.getPointer()
                );
            }
            if (localDisplayPosition != MarkerPosition.RIGHT) {
                localRightResult = NEONVision.processYUY2WithMask(
                        frame.getImageBuffer(), frame.getImageSize(),
                        highColor, lowColor,
                        rightMask.getPointer()
                );
            }

            // since this will overwrite the image, it must be done last
            switch (localDisplayPosition) {
                case LEFT:
                    localLeftResult = NEONVision.processYUY2WithMaskForDisplay(
                            frame.getImageBuffer(), frame.getImageSize(),
                            highColor, lowColor,
                            leftMask.getPointer()
                    );
                    break;
                case CENTER:
                    localCenterResult = NEONVision.processYUY2WithMaskForDisplay(
                            frame.getImageBuffer(), frame.getImageSize(),
                            highColor, lowColor,
                            centerMask.getPointer()
                    );
                    break;
                case RIGHT:
                    localRightResult = NEONVision.processYUY2WithMaskForDisplay(
                            frame.getImageBuffer(), frame.getImageSize(),
                            highColor, lowColor,
                            rightMask.getPointer()
                    );
                    break;
                case UNKNOWN:
                    break;
            }
        } else {
            localLeftResult = NEONVision.processYUY2WithMask(
                    frame.getImageBuffer(), frame.getImageSize(),
                    highColor, lowColor,
                    leftMask.getPointer()
            );
            localCenterResult = NEONVision.processYUY2WithMask(
                    frame.getImageBuffer(), frame.getImageSize(),
                    highColor, lowColor,
                    centerMask.getPointer()
            );
            localRightResult = NEONVision.processYUY2WithMask(
                    frame.getImageBuffer(), frame.getImageSize(),
                    highColor, lowColor,
                    rightMask.getPointer()
            );
        }
        executionTime = (System.nanoTime() - start) / (1e9);

        if (invert) {
            localLeftResult = leftInversionReference - localLeftResult;
            localCenterResult = centerInversionReference - localCenterResult;
            localRightResult = rightInversionReference - localRightResult;
        }

        long max = MathUtilities.max(localLeftResult, localCenterResult, localRightResult);
        if (max == localLeftResult) {
            position = MarkerPosition.LEFT;
        } else if (max == localCenterResult) {
            position = MarkerPosition.CENTER;
        } else {
            position = MarkerPosition.RIGHT;
        }

        leftResult.set(localLeftResult);
        centerResult.set(localCenterResult);
        rightResult.set(localRightResult);
    }

    public void setHighColor(int y, int u, int v) {
        highColor = NEONVision.yuvColorLong(y, u, v);
    }

    public void setLowColor(int y, int u, int v) {
        lowColor = NEONVision.yuvColorLong(y, u, v);
    }

    public long getLeftResult() {
        return leftResult.get();
    }

    public long getCenterResult() {
        return centerResult.get();
    }

    public long getRightResult() {
        return rightResult.get();
    }

    public MarkerPosition getPosition() {
        return position;
    }

    public void setDisplayPosition(MarkerPosition displayPosition) {
        this.displayPosition = displayPosition;
    }

    public MarkerPosition getDisplayPosition() {
        return displayPosition;
    }

    public void setInverted(boolean invert) {
        this.invert = invert;
    }

    public boolean isInverted() {
        return invert;
    }

    public void setLeftInversionReference(long leftInversionReference) {
        this.leftInversionReference = leftInversionReference;
    }

    public long getLeftInversionReference() {
        return leftInversionReference;
    }

    public void setCenterInversionReference(long centerInversionReference) {
        this.centerInversionReference = centerInversionReference;
    }

    public long getCenterInversionReference() {
        return centerInversionReference;
    }

    public void setRightInversionReference(long rightInversionReference) {
        this.rightInversionReference = rightInversionReference;
    }

    public long getRightInversionReference() {
        return rightInversionReference;
    }

    /**
     * Gets the execution time of the vision pipeline (not including comparison logic) in seconds.
     *
     * @return execution time
     */
    public double getExecutionTime() {
        return executionTime;
    }
}
