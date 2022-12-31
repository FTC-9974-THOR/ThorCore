package org.ftc9974.thorcore.samples.seasonal.powerplay;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.seasonal.powerplay.PowerPlaySeeker;
import org.ftc9974.thorcore.util.TimingUtilities;
import org.ftc9974.thorcore.vision.Seeker;

import java.io.IOException;

/**
 * This sample demonstrates the Power Play variant of the Seeker.
 */
@TeleOp(name = "Sample - Power Play Vision", group = "ThorCore Samples")
//@Disabled
public class SamplePowerPlayVision extends OpMode {

    private PowerPlaySeeker ppSeeker;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(20);
        telemetry.setItemSeparator("\n\t");

        telemetry.addLine("Initializing...");
        telemetry.update();

        try {
            // initialize the seeker
            ppSeeker = new PowerPlaySeeker("seeker", hardwareMap);

            // begin streaming camera frames to the Driver Station
            ppSeeker.startStreaming();
        } catch (IOException e) {
            e.printStackTrace();
            RobotLog.ee("Seeker", e, "IOException initializing seeker");
            throw new RuntimeException("IOException initializing seeker", e);
        }
    }

    @Override
    public void init_loop() {
        displaySignature("Blue Cone & Line", ppSeeker.blueConeAndLine);
        displaySignature("Red Cone & Line", ppSeeker.redConeAndLine);
        displaySignature("Pole", ppSeeker.pole);

        if (gamepad1.dpad_down) {
            ppSeeker.hideSignatures();
        } else if (gamepad1.dpad_left) {
            ppSeeker.showSignature(ppSeeker.blueConeAndLine);
        } else if (gamepad1.dpad_up) {
            ppSeeker.showSignature(ppSeeker.redConeAndLine);
        } else if (gamepad1.dpad_right) {
            ppSeeker.showSignature(ppSeeker.pole);
        }
    }

    @Override
    public void start() {
        ppSeeker.hideSignatures();
        ppSeeker.stopStreaming();
    }

    @Override
    public void loop() {
        displaySignature("Blue Cone & Line", ppSeeker.blueConeAndLine);
        displaySignature("Red Cone & Line", ppSeeker.redConeAndLine);
        displaySignature("Pole", ppSeeker.pole);
    }

    @Override
    public void stop() {
        // shutting down the camera can be a bit of a long-running operation, so we run it on a
        // separate thread to avoid blocking and freaking out the Robot Controller.
        TimingUtilities.runOnSeparateThread(ppSeeker::shutdown);
    }

    private void displaySignature(String name, Seeker.Signature signature) {
        telemetry.addLine(signature.hasLock() ? name + " [LOCKED]" : name)
                .addData("", "")
                .addData("X", signature.getX())
                .addData("Y", signature.getY())
                .addData("Lock Strength", signature.getLockStrength());
    }
}