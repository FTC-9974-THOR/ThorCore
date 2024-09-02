package org.ftc9974.thorcore.control.navigation.roadrunner;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.ftc9974.thorcore.robot.sensors.Encoder;

public interface Odometry {

    Encoder leftEncoder();
    Encoder rightEncoder();
    Encoder horizontalEncoder();

    double lateralDistance();
    double horizontalOffset();
    double mmPerTick();

    static void resetEncoder(DcMotorEx motor) {
        DcMotor.RunMode prevMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(prevMode);
    }
}
