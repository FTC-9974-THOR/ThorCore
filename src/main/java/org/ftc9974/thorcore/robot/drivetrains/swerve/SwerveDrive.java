package org.ftc9974.thorcore.robot.drivetrains.swerve;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public final class SwerveDrive implements HolonomicDrivetrain {

    @Hardware(name = "left")
    private SwerveModule leftModule;

    @Hardware(name = "right")
    private SwerveModule rightModule;

    private Vector2 leftPosition, rightPosition;

    public SwerveDrive(HardwareMap hw) {
        Realizer.realize(this, hw);

        leftPosition = new Vector2(-150, 0);
        rightPosition = new Vector2(150, 0);
    }

    @Override
    public void drive(double x, double y, double rot) {
        Vector2 inputVector = new Vector2(x, y);
        double inputHeading = inputVector.getHeading();
        double inputMag = Math.min(inputVector.getMagnitude(), 1);
        leftModule.setTargetDirection(inputHeading);
        rightModule.setTargetDirection(inputHeading);
        leftModule.setDrivePower(inputMag);
        rightModule.setDrivePower(inputMag);
    }

    @Override
    public void resetEncoders() {
        leftModule.resetEncoder();
        rightModule.resetEncoder();
    }
}
