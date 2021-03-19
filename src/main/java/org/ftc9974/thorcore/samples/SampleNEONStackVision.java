package org.ftc9974.thorcore.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.seasonal.ultimategoal.NEONStackVision;

import java.io.IOException;

@TeleOp(name = "Sample - NEONStackVision", group = "ThorCore Samples")
public class SampleNEONStackVision extends OpMode {

    private static final String TAG = "SampleNEONStackVision";

    private NEONStackVision neon;

    @Override
    public void init() {
        try {
            neon = new NEONStackVision("Webcam 1", hardwareMap);
            neon.startStreaming();
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "IOException while initializing NEONStackVision");
            telemetry.addLine(String.format("Init Failed: IOException: %s", e.getMessage()));
            telemetry.update();
        } catch (UnsupportedOperationException e) {
            RobotLog.ee(TAG, e, "Unsupported ABI");
            telemetry.addLine(e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void init_loop() {
        if (neon != null) {
            if (!neon.hasCompletedProcessing()) {
                telemetry.addLine("Waiting for frame...");
            } else {
                telemetry.addData("Stack Height", neon.getStackHeight());
                telemetry.addData("Raw Result", neon.getRawResult());
            }
            telemetry.update();
        }
    }

    @Override
    public void start() {
        if (neon == null) {
            requestOpModeStop();
            return;
        }
        neon.stopStreaming();
    }

    @Override
    public void loop() {
        telemetry.addData("Stack Height", neon.getStackHeight());
        telemetry.addData("Raw Result", neon.getRawResult());
    }

    @Override
    public void stop() {
        neon.shutdown();
    }
}
