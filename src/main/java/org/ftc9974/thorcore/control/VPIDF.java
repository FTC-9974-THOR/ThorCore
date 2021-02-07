package org.ftc9974.thorcore.control;

/*
Velocity PID implementation

https://deltamotion.com/support/webhelp/rmctools/Controller_Features/Control_Modes/Velocity_PID.htm
 */
public final class VPIDF {

    private double kP, kI, kD, kF;

    private double setpoint;

    private double runningIntegral;
    private double integralLow, integralHigh;

    private double contLow, contHigh;
    private boolean continuous;

    private double tolerance;

    private double peakOutputForward, peakOutputReverse;
    private double nominalOutputForward, nominalOutputReverse;


}
