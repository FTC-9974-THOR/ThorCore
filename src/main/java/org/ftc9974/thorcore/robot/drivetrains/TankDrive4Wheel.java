package org.ftc9974.thorcore.robot.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

/**
 * Implements a 4 wheel tank drive.
 */
public final class TankDrive4Wheel {

    @Hardware
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public TankDrive4Wheel(HardwareMap hardwareMap) {
        Realizer.realize(this, hardwareMap);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double left, double right) {
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }

    public void arcadeDrive(double speed, double turn) {
        drive(speed + turn, speed - turn);
    }

    public void setMotorModes(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void resetEncoders() {
        DcMotor.RunMode previousMode = frontLeft.getMode();
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(previousMode);
    }

    public void setFrontLeftTarget(int target) {
        frontLeft.setTargetPosition(target);
    }

    public void setFrontRightTarget(int target) {
        frontRight.setTargetPosition(target);
    }

    public void setBackLeftTarget(int target) {
        backLeft.setTargetPosition(target);
    }

    public void setBackRightTarget(int target) {
        backRight.setTargetPosition(target);
    }

    public int getFrontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }

    public int getFrontRightPosition() {
        return frontRight.getCurrentPosition();
    }

    public int getBackLeftPosition() {
        return backLeft.getCurrentPosition();
    }

    public int getBackRightPosition() {
        return backRight.getCurrentPosition();
    }

    public boolean isBusy() {
        return frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
    }
}
