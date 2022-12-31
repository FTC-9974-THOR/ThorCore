package org.ftc9974.thorcore.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.seasonal.ultimategoal.NEONStackVision;

import java.io.IOException;

/**
 * Sample for NEONStackVision. Shows how to use NEONStackVision to determine the number of rings in
 * the stack.
 *
 * Hardware Required:
 * A robot controller using an arm64-v8a ABI. The REV Control Hub will work, and I believe most phones
 * will too.
 * A USB webcam. In the configuration file, this must be named "Webcam 1".
 */
@TeleOp(name = "Sample - NEONStackVision", group = "ThorCore Samples")
public class SampleNEONStackVision extends OpMode {

    /**
     * Logging tag
     */
    private static final String TAG = "SampleNEONStackVision";

    /**
     * Instance of the NEONStackVision vision system. You may instantiate as many of these as you
     * like, but you'll typically only need one instance.
     */
    private NEONStackVision neon;

    @Override
    public void init() {
        try {
            // try to initialize the vision system.
            neon = new NEONStackVision("Webcam 1", hardwareMap);
            // start streaming to the driver station. you can see the output of the vision system on
            // the driver station in the 3-dot menu with the "Camera Stream" option. Note that the
            // camera stream is only available until you press the start button on the OpMode.
            neon.startStreaming();
        } catch (IOException e) {
            // A problem occurred opening the camera
            RobotLog.ee(TAG, e, "IOException while initializing NEONStackVision");
            telemetry.addLine(String.format("Init Failed: IOException: %s", e.getMessage()));
            telemetry.update();
        } catch (UnsupportedOperationException e) {
            // NEONVision is not supported on this ABI
            RobotLog.ee(TAG, e, "Unsupported ABI");
            telemetry.addLine(e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void init_loop() {
        // make sure we actually have a NEON instance first, because we don't force the OpMode to stop
        // in the event of init failure.
        if (neon != null) {
            if (!neon.hasCompletedProcessing()) {
                // we don't have a frame yet, so print a standby message
                telemetry.addLine("Waiting for frame...");
            } else {
                // print the output of the vision system
                telemetry.addData("Stack Height", neon.getStackHeight());
                telemetry.addData("Raw Result", neon.getRawResult());
            }
            telemetry.update();
        }
    }

    @Override
    public void start() {
        // unlike in init_loop(), there's nothing to do if we couldn't init NEONVision. in that case,
        // force stop the OpMode.
        if (neon == null) {
            requestOpModeStop();
            return;
        }
        // stop streaming. internally, this switches NEONStackVision back to a higher-performance
        // algorithm, so always do this when you stop streaming.
        neon.stopStreaming();
    }

    @Override
    public void loop() {
        // print the output of the vision system. we don't have to null-check neon anymore, since
        // we force OpMode stop in start() if it's null.
        telemetry.addData("Stack Height", neon.getStackHeight());
        telemetry.addData("Raw Result", neon.getRawResult());
    }

    @Override
    public void stop() {
        // shutdown the NEONStackVision instance. this releases the camera and stops streaming.
        neon.shutdown();
    }
}
