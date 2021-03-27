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

    /**
     * Creates a new TankDrive2Wheel object.
     *
     * @param hardwareMap hardware map
     */
    public TankDrive2Wheel(HardwareMap hardwareMap) {
        Realizer.realize(this, hardwareMap);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Drives with tank controls.
     *
     * @param left left power
     * @param right right power
     */
    public void drive(double left, double right) {
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    /**
     * Sets targets for the motors.
     *
     * @param left left motor target
     * @param right right motor target
     */
    public void setTargets(int left, int right) {
        leftMotor.setTargetPosition(left);
        rightMotor.setTargetPosition(right);
    }

    /**
     * Sets the mode of the motors.
     *
     * @param mode motor mode
     */
    public void setModes(DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    /**
     * Checks if the left motor is moving.
     *
     * @return true if the left motor is not at target.
     */
    public boolean isLeftBusy() {
        return leftMotor.isBusy();
    }

    /**
     * Checks if the right motor is moving.
     *
     * @return true if the right motor is not at target.
     */
    public boolean isRightBusy() {
        return rightMotor.isBusy();
    }

    /**
     * Sets the zero power behaviour of the motors.
     * @see DcMotor.ZeroPowerBehavior
     *
     * @param behaviour behaviour
     */
    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour) {
        leftMotor.setZeroPowerBehavior(behaviour);
        rightMotor.setZeroPowerBehavior(behaviour);
    }

    /**
     * Drives with arcade controls.
     *
     * @param speed forward/backward movement speed
     * @param turn turning speed
     */
    public void arcadeDrive(double speed, double turn) {
        drive(speed + turn, speed - turn);
    }
}
