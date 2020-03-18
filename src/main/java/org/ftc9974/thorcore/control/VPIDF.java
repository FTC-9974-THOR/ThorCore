package org.ftc9974.thorcore.control;

/*
Velocity PID implementation

https://deltamotion.com/support/webhelp/rmctools/Controller_Features/Control_Modes/Velocity_PID.htm
 */
public final class VPIDF {

    private double velocityFeedForwardGain,
                   accelerationFeedForwardGain,
                   jerkFeedForwardGain,
                   proportionalGain,
                   differentialGain,
                   integralGain;

    private double targetVelocity,
                   targetAcceleration,
                   targetJerk;

    private double lastPosition,
                   lastVelocity;

    public double update(double input) {

        return 0;
    }
}
