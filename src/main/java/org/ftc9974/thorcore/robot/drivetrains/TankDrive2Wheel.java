package org.ftc9974.thorcore.robot.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

/**
 * Implements a 2 wheel tank drive.
 */
@SuppressWarnings("WeakerAccess")
public final class TankDrive2Wheel {

    @Hardware
    public DcMotor leftMotor, rightMotor;

    public TankDrive2Wheel(HardwareMap hardwareMap) {
        Realizer.realize(this, hardwareMap);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double left, double right) {
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    public void setTargets(int left, int right) {
        leftMotor.setTargetPosition(left);
        rightMotor.setTargetPosition(right);
    }

    public void setModes(DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    public boolean isLeftBusy() {
        return leftMotor.isBusy();
    }

    public boolean isRightBusy() {
        return rightMotor.isBusy();
    }

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour) {
        leftMotor.setZeroPowerBehavior(behaviour);
        rightMotor.setZeroPowerBehavior(behaviour);
    }

    public void arcadeDrive(double speed, double turn) {
        drive(speed + turn, speed - turn);
    }
}
