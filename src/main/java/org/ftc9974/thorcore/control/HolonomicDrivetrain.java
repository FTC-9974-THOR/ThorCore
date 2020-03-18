package org.ftc9974.thorcore.control;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftc9974.thorcore.control.navigation.Navigator;

/**
 * Interface implemented by holonomic drivetrains. Used with {@link Navigator}.
 */
public interface HolonomicDrivetrain {

    void drive(double x, double y, double rot);

    default void setMotorPowers(double[] powers) {};

    default void setEncoderTargets(int[] targets) {}
    default int[] getEncoderPositions() {
        return new int[0];
    }
    default void setMotorModes(DcMotor.RunMode mode) {}
    default void resetEncoders() {}
    default boolean isMoving() {
        return false;
    }
}
