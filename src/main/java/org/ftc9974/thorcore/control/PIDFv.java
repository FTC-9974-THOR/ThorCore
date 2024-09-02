package org.ftc9974.thorcore.control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFv {

    public double kP, kI, kD, kF;
    public double positionSetpoint = 0, velocitySetpoint = 0;
    public double lastPositionError, lastVelocityError;

    private double integrator = Double.NaN; // NaN is used as a sentinel value to indicate the first
                                            // run through update()
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private boolean continuous = false;
    private double contLow, contHigh, contDiff;

    public PIDFv(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double update(double currentPosition, double currentVelocity) {
        double positionError = positionSetpoint - currentPosition;
        double velocityError = velocitySetpoint - currentVelocity;

        if (continuous) {
            positionError = modulus(positionError);
        }

        if (Double.isNaN(integrator)) {
            integrator = 0;
            timer.reset();
        } else {
            integrator = positionError * timer.time();
            timer.reset();
        }

        lastPositionError = positionError;
        lastVelocityError = velocityError;
        return kP * positionError + kI * integrator + kD * velocityError + kF;
    }

    public void reset() {
        integrator = Double.NaN;
    }

    public void setContinuityRange(double low, double high) {
        contLow = low;
        contHigh = high;
        contDiff = contHigh - contLow;
    }

    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }

    private double modulus(double x) {
        while (x < contLow) x += contDiff;
        while (x > contHigh) x -= contDiff;
        return x;
    }
}
