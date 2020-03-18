package org.ftc9974.thorcore.robot;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class StallDetector {

    private DcMotorEx motor;
    private double stallThreshold;

    public StallDetector(DcMotorEx motor, double threshold) {
        this.motor = motor;
        stallThreshold = threshold;
    }

    public void setStallThreshold(double threshold) {
        stallThreshold = threshold;
    }

    public double getStallThreshold() {
        return stallThreshold;
    }

    public boolean isStalled() {
        return motor.getPower() != 0 && Math.abs(motor.getVelocity(AngleUnit.RADIANS)) <= stallThreshold;
    }
}
