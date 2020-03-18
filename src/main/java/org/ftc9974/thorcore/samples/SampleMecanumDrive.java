package org.ftc9974.thorcore.samples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.OpModeEnhanced;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

/**
 * A sample that shows how to use {@link MecanumDrive}.
 *
 * This looks for 4 motors: MD-frontLeft, MD-frontRight, MD-backLeft, and MD-backRight. MD-frontLeft
 * corresponds to the front left motor, MD-frontRight is the front right motor, MD-backLeft is the
 * back right motor, and MD-backRight is the back right motor.
 *
 * Why the MD in the names? It's part of the ThorCore naming convention, which is explained in the
 * javadocs for {@link org.ftc9974.thorcore.meta.Realizer#realize(Object, HardwareMap)}.
 */
@TeleOp(name = "Sample - Mecanum Drive", group = "ThorCore Samples")
@Disabled
public class SampleMecanumDrive extends OpModeEnhanced {

    // Declare a variable of type MecanumDrive
    private MecanumDrive rb;

    @Override
    public void init() {
        // Call super.init(). Required for OpModeEnhanced to work.
        super.init();
        // Construct a new instance of MecanumDrive
        rb = new MecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        // Call super.loop(). Required for OpModeEnhanced to work.
        super.loop();
        // Drive the robot. Right joystick is movement, left x-axis is turning.
        rb.drive(gamepad1.getRightX(), gamepad1.getRightY(), gamepad1.getLeftX());
    }
}
