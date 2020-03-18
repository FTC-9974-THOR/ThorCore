package org.ftc9974.thorcore.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftc9974.thorcore.robot.MotorType;
import org.ftc9974.thorcore.robot.drivetrains.TankDrive4Wheel;
import org.ftc9974.thorcore.util.MotorUtilities;

import java.util.Locale;

/**
 * Shows how to use encoders to drive a given distance.
 *
 * Note: setting a DcMotor's direction to reverse may not reverse the encoders. Thus, this
 * sample may make the robot turn in place instead of driving forward.
 */
@TeleOp(name = "Sample - Encoders", group = "ThorCore Samples")
@Disabled
public class SampleEncoders extends LinearOpMode {

    private TankDrive4Wheel rb;

    private final MotorType MOTOR_TYPE = MotorType.NEVEREST_40; // replace this with the motor type you use
    private final double WHEEL_DIAMETER = 4; // wheel diameter, in inches
    private final double GEAR_RATIO = 1; // ratio between wheels and motor shaft
    private final double DISTANCE = 12; // distance, in inches

    @Override
    public void runOpMode() throws InterruptedException {
        rb = new TankDrive4Wheel(hardwareMap);

        while (opModeIsActive() && !isStarted()) {
            telemetry.addLine("Waiting for start.");
            telemetry.update();
            idle();
        }

        // calculate ticks
        int ticks = MotorUtilities.ticksForDistance(DISTANCE, WHEEL_DIAMETER, GEAR_RATIO, MOTOR_TYPE);
        // reset the encoders
        rb.resetEncoders();
        // change motors to run to position mode
        rb.setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        // assign targets
        rb.setFrontLeftTarget(ticks);
        rb.setFrontRightTarget(ticks);
        rb.setBackLeftTarget(ticks);
        rb.setBackRightTarget(ticks);
        // set max speed of the motors
        rb.drive(1, 1);
        while (opModeIsActive() && rb.isBusy()) {
            telemetry.addLine(String.format(Locale.getDefault(), "Driving to %d", ticks));
            telemetry.addData("Front left position", rb.getFrontLeftPosition());
            telemetry.addData("Front right position", rb.getFrontRightPosition());
            telemetry.addData("Back left position", rb.getBackLeftPosition());
            telemetry.addData("Back right position", rb.getBackRightPosition());
            telemetry.update();
            idle();
        }
        rb.drive(0, 0);
    }
}
