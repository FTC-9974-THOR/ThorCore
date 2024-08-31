package org.ftc9974.thorcore.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftc9974.thorcore.util.MathUtilities;

public class SlewRateLimiter {

    public double slewRate;

    private final ElapsedTime timer = new ElapsedTime();
    private double output;

    private double contHigh, contLow, contDiff;
    private boolean continuous;

    public SlewRateLimiter(double slewRate) {
        this.slewRate = slewRate;
    }

    public double update(double setpoint) {
        double deltaT = timer.seconds();
        timer.reset();
        double adjustment = slewRate * deltaT;

        if (continuous) {
            setpoint = MathUtilities.wraparound(setpoint, contLow, contHigh);
        }

        double error = setpoint - output;
        if (continuous) {
            error = MathUtilities.wraparound(error, -0.5 * contDiff, 0.5 * contDiff);
        }
        if (Math.abs(error) > adjustment) {
            output += Math.copySign(adjustment, error);
        } else {
            output = setpoint;
        }
        if (continuous) {
            output = MathUtilities.wraparound(output, contLow, contHigh);
        }

        return output;
    }

    public void setContinuityRange(double low, double high) {
        if (low > high) {
            throw new IllegalArgumentException("High must be greater than low");
        }

        contLow = low;
        contHigh = high;
        contDiff = high - low;
    }

    public boolean isContinuous() {
        return continuous;
    }

    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }
}
