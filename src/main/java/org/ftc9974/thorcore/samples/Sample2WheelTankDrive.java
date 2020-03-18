package org.ftc9974.thorcore.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.OpModeEnhanced;
import org.ftc9974.thorcore.robot.drivetrains.TankDrive2Wheel;

/**
 * A sample that shows how to use the {@link TankDrive2Wheel} class.
 *
 * This sample will look for 2 motors: TD2W-leftMotor and TD2W-rightMotor. TD2W-leftMotor should be
 * the robot's left motor, and TD2W-rightMotor should be the robot's right motor. Why the TD2W? It's
 * part of the ThorCore naming convention, which is explained in the javadocs for
 * {@link org.ftc9974.thorcore.meta.Realizer#realize(Object, HardwareMap)}.
 */
@TeleOp(name = "Sample - 2 Wheel Tank Drive", group = "ThorCore Samples")
@Disabled
public class Sample2WheelTankDrive extends OpModeEnhanced {

    // Declare a variable of type TankDrive2Wheel
    private TankDrive2Wheel rb;

    @Override
    public void init() {
        // Call super.init(). Required for OpModeEnhanced to work properly.
        super.init();

        // Construct a new TankDrive2Wheel object.
        rb = new TankDrive2Wheel(hardwareMap);
    }

    @Override
    public void loop() {
        // Call super.loop(). Required for OpModeEnhanced to work properly.
        super.loop();
        // Drive the robot. The first parameter is the left wheel power, and the second parameter
        // is the right wheel power. If you would prefer arcade drive, use rb.arcadeDrive instead.
        rb.drive(gamepad1.getLeftY(), gamepad1.getRightY());
    }
}
