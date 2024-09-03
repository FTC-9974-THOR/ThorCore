package org.ftc9974.thorcore.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.robot.drivetrains.TankDrive4Wheel;

/**
 * A sample showing how to use {@link TankDrive4Wheel}.
 *
 * This will look for 4 motors: TD4W-frontLeft, TD4W-frontRight, TD4W-backLeft, and TD4W-backRight.
 * TD4W-frontLeft corresponds to the front left motor, TD4W-frontRight corresponds to the front
 * right motor, TD4W-backLeft corresponds to the back left motor, and TD4W-backRight corresponds to
 * the back right motor.
 *
 * Why the TD4W- in the names? It's part of the ThorCore naming convention, which is explained in
 * the javadoc for {@link org.ftc9974.thorcore.meta.Realizer#realize(Object, HardwareMap)}.
 */
@TeleOp(name = "Sample - 4 Wheel Tank Drive", group = "_ ThorCore Samples")
@Disabled
public class Sample4WheelTankDrive extends OpMode {

    // Declare a variable of type TankDrive4Wheel
    private TankDrive4Wheel rb;

    @Override
    public void init() {
        // Construct a new instance of TankDrive4Wheel.
        rb = new TankDrive4Wheel(hardwareMap);
    }

    @Override
    public void loop() {
        // Drive using tank drive.
        rb.drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        // Or, if you prefer arcade drive, comment out the above line and uncomment out the line
        // below
        //rb.arcadeDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
    }
}
