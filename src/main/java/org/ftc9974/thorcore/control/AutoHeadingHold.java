package org.ftc9974.thorcore.control;

import org.ftc9974.thorcore.control.navigation.IMUNavSource2;

public class AutoHeadingHold {

    public enum State {
        MANUAL_CONTROL,
        DECELERATION,
        AUTO_HEADING_HOLD
    }

    public State state;
    private double previousError;
    private final IMUNavSource2 navSource;

    public final PIDFv pidf;
    public boolean invertController = false;
    public double atTargetThreshold = 0;
    public double minInputThreshold = 0;

    public AutoHeadingHold(double p, double i, double d, double f, IMUNavSource2 navSource) {
        state = State.AUTO_HEADING_HOLD;
        pidf = new PIDFv(p, i, d, f);
        previousError = Double.NEGATIVE_INFINITY;
        this.navSource = navSource;

        pidf.velocitySetpoint = 0;
        pidf.setContinuityRange(-Math.PI, Math.PI);
        pidf.setContinuous(true);
        pidf.update(0, 0);
    }

    public double update(double driverTurningInput) {
        final double absInput = Math.abs(driverTurningInput);

        // state transition logic
        switch (state) {
            case MANUAL_CONTROL:
                // if the driver is no longer commanding a turn, begin deceleration
                if (absInput <= minInputThreshold) {
                    // set the current heading as the pidf setpoint to begin deceleration
                    pidf.positionSetpoint = navSource.getHeading();
                    pidf.reset();
                    state = State.DECELERATION;
                }
                break;
            case DECELERATION:
                // if the driver is commanding a turn, return to manual control.
                if (absInput > minInputThreshold) {
                    state = State.MANUAL_CONTROL;
                    break;
                }
                double absPIDFError = Math.abs(pidf.lastPositionError);
                // if the pidf error has stopped increasing and is now starting to decrease, end
                // deceleration.
                if (absPIDFError < previousError) {
                    // grab the current heading as the new heading setpoint now that we've stopped
                    // turning.
                    pidf.positionSetpoint = navSource.getHeading();
                    pidf.reset();
                    state = State.AUTO_HEADING_HOLD;
                }
                previousError = absPIDFError;
                break;
            case AUTO_HEADING_HOLD:
                // if the driver is commanding a turn, return to manual control.
                if (absInput > minInputThreshold) state = State.MANUAL_CONTROL;
                break;
            default:
                state = State.MANUAL_CONTROL;
                break;
        }

        // state behavior logic
        switch (state) {
            case MANUAL_CONTROL:
            default:
                return driverTurningInput;
            case DECELERATION:
            case AUTO_HEADING_HOLD:
                if (Math.abs(pidf.lastPositionError) < atTargetThreshold) return 0;
                else return (invertController ? -1 : 1) * pidf.update(
                        navSource.getHeading(),
                        navSource.getHeadingVelocity()
                );
        }
    }
}
