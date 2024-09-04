package org.ftc9974.thorcore.control.navigation.roadrunner;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.ftc9974.thorcore.robot.sensors.Encoder;

public interface Odometry {

    Encoder leftEncoder();
    Encoder rightEncoder();
    Encoder horizontalEncoder();

    /**
     * this is the distance between the center of the contact patches of the left and right odometer.
     * @return distance
     */
    double lateralDistance();

    /**
     * this is the distance from the center of the robot to the center of the contact patch of the
     * back odometer. a positive value means the odometer is towards the front, and a negative value
     * means the odometer is towards the back of the robot.
     * @return distance
     */
    double horizontalOffset();

    /**
     * conversion factor for converting ticks to millimeters
     * @return factor
     */
    double mmPerTick();

    static void resetEncoder(DcMotorEx motor) {
        DcMotor.RunMode prevMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(prevMode);
    }
}
