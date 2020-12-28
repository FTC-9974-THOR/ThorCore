package org.ftc9974.thorcore.control;

import android.os.SystemClock;

// todo still working on this
public class VIPD {

    private double kI, kP, kD;
    private double targetVelocity;

    private long lastTimestamp;
    private double lastVelocity;
    private double lastAccel;

    private double accumulator;

    public VIPD(double i, double p, double d) {
        kI = i;
        kP = p;
        kD = d;
    }

    public void setSetpoint(double setpoint) {
        targetVelocity = setpoint;
    }

    public double update(double velocity) {
        double deltaTime = (SystemClock.uptimeMillis() - lastTimestamp) / 1000.0;
        lastTimestamp = SystemClock.uptimeMillis();

        double error = targetVelocity - velocity;
        double deltaVelocity = velocity - lastVelocity;
        double accel = deltaVelocity / deltaTime;
        double deltaAccel = accel - lastAccel;
        double jerk = deltaAccel / deltaTime;

        accumulator += error * deltaTime;

        lastVelocity = velocity;
        lastAccel = accel;
        return kI * accumulator - kP * accel - kD * jerk;
    }
}
