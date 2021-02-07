package org.ftc9974.thorcore.robot.drivetrains;

import androidx.annotation.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
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
    private boolean flEncInv, frEncInv, blEncInv, brEncInv;
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

    /**
     * Sets inversion flags for each axis.
     * Passing true will invert the corresponding axis.
     *
     * @param x inversion flag for x
     * @param y inversion flag for y
     * @param rot inversion flag for turning
     */
    public void setAxisInversion(boolean x, boolean y, boolean rot) {
        xInv = x;
        yInv = y;
        tInv = rot;
    }

    /**
     * Sets inversion flags for each encoder.
     * Passing true will invert the corresponding encoder.
     *
     * @param fl inversion flag for the front left encoder
     * @param fr inversion flag for the front right encoder
     * @param bl inversion flag for the back left encoder
     * @param br inversion flag for the back right encoder
     */
    public void setEncoderInversion(boolean fl, boolean fr, boolean bl, boolean br) {
        flEncInv = fl;
        frEncInv = fr;
        blEncInv = bl;
        brEncInv = br;
    }

    /**
     * Sets inversion flags for each motor.
     * Passing true will invert the corresponding motor.
     *
     * @param fl inversion flag for the front left motor
     * @param fr inversion flag for the front right motor
     * @param bl inversion flag for the back left motor
     * @param br inversion flag for the back right motor
     */
    public void setMotorInversion(boolean fl, boolean fr, boolean bl, boolean br) {
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
        // apply inversions
        double _x = ((xInv) ? -1 : 1) * x;
        double _y = ((yInv) ? -1 : 1) * y;
        double _t = ((tInv) ? -1 : 1) * rot;

        // apply mecanum kinematics
        double fl = _x + _y + _t;
        double fr = -_x + _y - _t;
        double bl = -_x + _y + _t;
        double br = _x + _y - _t;

        // normalize if the speeds have saturated
        double max = MathUtilities.absMax(fl, fr, bl, br);
        if (max > 1) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        if (flInv) fl *= -1;
        if (frInv) fr *= -1;
        if (blInv) bl *= -1;
        if (brInv) br *= -1;

        /*double r = Math.hypot(x, y);
        fl = r * (fl / max);
        fr = r * (fr / max);
        bl = r * (bl / max);
        br = r * (br / max);*/

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    /**
     * Resets the encoders and returns them to the mode the front left motor was in before the call
     * to resetEncoders().
     */
    @Override
    public void resetEncoders() {
        DcMotor.RunMode previousMode = frontLeft.getMode();
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(previousMode);
    }

    /**
     * Sets the target position for each motor.
     *
     * @param targets array of targets, in the order:
     *                {
     *                    frontLeft,
     *                    frontRight,
     *                    backLeft,
     *                    backRight
     *                }
     */
    @Override
    public void setEncoderTargets(@Size(4) int[] targets) {
        frontLeft.setTargetPosition(targets[0]);
        frontRight.setTargetPosition(targets[1]);
        backLeft.setTargetPosition(targets[2]);
        backRight.setTargetPosition(targets[3]);
    }

    /**
     * Gets the current position of each encoder.
     *
     * @return array of encoder position, in the order:
     *         {
     *             frontLeft,
     *             frontRight,
     *             backLeft,
     *             backRight
     *         }
     */
    @Override
    public @Size(4) int[] getEncoderPositions() {
        return new int[] {
                (flEncInv ? -1 : 1) * frontLeft.getCurrentPosition(),
                (frEncInv ? -1 : 1) * frontRight.getCurrentPosition(),
                (blEncInv ? -1 : 1) * backLeft.getCurrentPosition(),
                (brEncInv ? -1 : 1) * backRight.getCurrentPosition()
        };
    }

    /**
     * Sets the mode of the motors.
     *
     * @param mode motor mode
     */
    @Override
    public void setMotorModes(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    /**
     * Checks if the robot is still moving to a target.
     *
     * @return true if all of the motors are not at target.
     */
    @Override
    public boolean isMoving() {
        return frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
    }

    /**
     * Direct access to motor powers. Used for advanced control systems.
     *
     * @param powers an array of powers, in the order:
     *               {
     *                  frontLeft,
     *                  frontRight,
     *                  backLeft,
     *                  backRight
     *               }
     */
    @Override
    public void setMotorPowers(@Size(4) double[] powers) {
        frontLeft.setPower((flInv) ? -powers[0] : powers[0]);
        frontRight.setPower((frInv) ? -powers[1] : powers[1]);
        backLeft.setPower((blInv) ? -powers[2] : powers[2]);
        backRight.setPower((brInv) ? -powers[3] : powers[3]);
    }
}
