package org.ftc9974.thorcore.seasonal.ultimategoal;

import android.os.Build;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.ftc9974.thorcore.internal.RealizableFactory;
import org.ftc9974.thorcore.robot.sensors.USBWebcamBase;
import org.ftc9974.thorcore.vision.NEONVision;

import java.io.IOException;

public class NEONStackVision extends USBWebcamBase {

    private static final String TAG = "NEONStackVision";

    private double fullStackMatch;
    private double fourStackMin, oneStackMin;

    private long lowColor, highColor;

    private final ElapsedTime timer;

    private volatile StackHeight stackHeight;
    private volatile long result;

    @RealizableFactory
    public NEONStackVision(String name, HardwareMap hw) throws IOException {
        this(hw.get(WebcamName.class, name));
    }

    public NEONStackVision(WebcamName webcamName) throws IOException {
        super(webcamName);

        if (!NEONVision.supportsNeonAcceleration()) {
            throw new UnsupportedOperationException(String.format("NEONVision is currently not supported on CPU architecture \"%s\"", Build.SUPPORTED_ABIS[0]));
        }

        fullStackMatch = 10000;
        fourStackMin = 0.5;
        oneStackMin = 0.35;

        lowColor = NEONVision.yuvColorLong(0, 0, 139);
        highColor = NEONVision.yuvColorLong(178, 155, 162);

        timer = new ElapsedTime();
    }

    public void setFullStackMatch(double fullStackMatch) {
        this.fullStackMatch = fullStackMatch;
    }

    public double getFullStackMatch() {
        return fullStackMatch;
    }

    public void setFourStackMin(double fourStackMin) {
        this.fourStackMin = fourStackMin;
    }

    public double getFourStackMin() {
        return fourStackMin;
    }

    public void setOneStackMin(double oneStackMin) {
        this.oneStackMin = oneStackMin;
    }

    public double getOneStackMin() {
        return oneStackMin;
    }

    public void setThresholds(int lowY, int highY, int lowU, int highU, int lowV, int highV) {
        lowColor = NEONVision.yuvColorLong(lowY, lowU, lowV);
        highColor = NEONVision.yuvColorLong(highY, highU, highV);
    }

    @Override
    protected void onNewFrame(CameraFrame frame) {
        long start = System.nanoTime();
        if (isStreaming()) {
            result = NEONVision.processYUY2ForDisplay(frame.getImageBuffer(), frame.getImageSize(), highColor, lowColor);
        } else {
            result = NEONVision.processYUY2(frame.getImageBuffer(), frame.getImageSize(), highColor, lowColor);
        }
        double elapsed = (System.nanoTime() - start) * 1e-9;
        double cameraElapsed = timer.seconds();
        RobotLog.dd(TAG, "Execution time: %f seconds Instantaneous FPS: %f Time since last frame: %f Camera FPS: %f",
                elapsed, 1 / elapsed, cameraElapsed, 1 / cameraElapsed);

        double factor = result / fullStackMatch;
        if (factor > fourStackMin) {
            stackHeight = StackHeight.FOUR;
        } else if (factor > oneStackMin) {
            stackHeight = StackHeight.ONE;
        } else {
            stackHeight = StackHeight.ZERO;
        }
        timer.reset();
    }

    public boolean hasCompletedProcessing() {
        return stackHeight != null;
    }

    public StackHeight getStackHeight() {
        return stackHeight;
    }

    public long getRawResult() {
        return result;
    }
}
