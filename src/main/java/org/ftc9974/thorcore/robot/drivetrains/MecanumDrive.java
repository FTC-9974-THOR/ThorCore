package org.ftc9974.thorcore.robot.drivetrains;

import android.support.annotation.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.NavSource;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

/**
 * Implements the behaviour of a mecanum drive.
 */
public final class MecanumDrive implements HolonomicDrivetrain {

    @SuppressWarnings("WeakerAccess")
    @Hardware
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    private boolean xInv, yInv, tInv;
    private boolean flInv, frInv, blInv, brInv;

    /**
     * Creates a new MecanumDrive object.
     * @param hardwareMap hardwareMap.
     */
    public MecanumDrive(HardwareMap hardwareMap) {
        Realizer.realize(this, hardwareMap);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setAxisInversion(boolean x, boolean y, boolean rot) {
        xInv = x;
        yInv = y;
        tInv = rot;
    }

    public void setEncoderInversion(boolean fl, boolean fr, boolean bl, boolean br) {
        flInv = fl;
        frInv = fr;
        blInv = bl;
        brInv = br;
    }

    /**
     * Moves the robot. Expects values in the range [-1, 1] inclusive.
     * +y is forward, +x is right.
     * @param x speed in the x direction
     * @param y speed in the y direction
     * @param rot speed to rotate
     */
    @Override
    public void drive(double x, double y, double rot) {
        double _x = ((xInv) ? -1 : 1) * x;
        double _y = ((yInv) ? -1 : 1) * y;
        double _t = ((tInv) ? -1 : 1) * rot;
        double fl = _x + _y + _t;
        double fr = -_x + _y - _t;
        double bl= -_x + _y + _t;
        double br = _x + _y - _t;

        double max = MathUtilities.max(Math.abs(fl), Math.abs(fr), Math.abs(bl), Math.abs(br));
        if (max > 1) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(-fl);
        frontRight.setPower(-fr);
        backLeft.setPower(-bl);
        backRight.setPower(-br);
    }

    @Override
    public void resetEncoders() {
        DcMotor.RunMode previousMode = frontLeft.getMode();
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(previousMode);
    }

    @Override
    public void setEncoderTargets(@Size(4) int[] targets) {
        frontLeft.setTargetPosition(targets[0]);
        frontRight.setTargetPosition(targets[1]);
        backLeft.setTargetPosition(targets[2]);
        backRight.setTargetPosition(targets[3]);
    }

    @Override
    public @Size(4) int[] getEncoderPositions() {
        return new int[] {
                (flInv ? -1 : 1) * frontLeft.getCurrentPosition(),
                (frInv ? -1 : 1) * frontRight.getCurrentPosition(),
                (blInv ? -1 : 1) * backLeft.getCurrentPosition(),
                (brInv ? -1 : 1) * backRight.getCurrentPosition()
        };
    }

    @Override
    public void setMotorModes(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    @Override
    public boolean isMoving() {
        return frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
    }

    @Override
    public void setMotorPowers(@Size(4) double[] powers) {
        frontLeft.setPower(powers[0]);
        frontRight.setPower(powers[1]);
        backLeft.setPower(powers[2]);
        backRight.setPower(powers[3]);
    }
}
