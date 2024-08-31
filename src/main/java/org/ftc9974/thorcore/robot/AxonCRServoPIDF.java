package org.ftc9974.thorcore.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.internal.RealizableFactory;
import org.ftc9974.thorcore.util.MathUtilities;

public class AxonCRServoPIDF {
    public CRServoImplEx servo;
    public AnalogInput feedback;

    private final PIDF pidf;
    private double rotationsOffset, lastPosition;

    public AxonCRServoPIDF(CRServoImplEx servo, AnalogInput feedback) {
        this.servo = servo;
        this.feedback = feedback;

        servo.setPwmRange(new PwmControl.PwmRange(600, 2400));

        pidf = new PIDF(0.3, 0, 0, 0);
        pidf.setInputFunction(this::getPosition);
        pidf.setOutputFunction(this.servo::setPower);
        pidf.setContinuous(true);
        pidf.setContinuityRange(0, 2 * Math.PI);
        pidf.setPeriod(0.01);

        servo.setPower(0);

        lastPosition = getPosition();
    }

    @RealizableFactory
    public AxonCRServoPIDF(String name, HardwareMap hw) {
        this(
                hw.get(CRServoImplEx.class, String.format("%s-servo", name)),
                hw.analogInput.get(String.format("%s-feedback", name))
        );
    }

    public void setTunings(double p, double i, double d, double f) {
        pidf.setTunings(p, i, d, f);
    }

    public void setPosition(double setpoint) {
        pidf.setSetpoint(setpoint);
    }

    public double getPositionSetpoint() {
        return pidf.getSetpoint();
    }

    public double getPosition() {
        //return MathUtilities.map(feedback.getVoltage(), 3.3, 0, 0, 2 * Math.PI);
        return MathUtilities.map(feedback.getVoltage(), 0, 3.3, 0, 2 * Math.PI);
    }

    public double getIncrementalPosition() {
        return getPosition() + rotationsOffset;
    }

    public double getLastError() {
        return pidf.getLastError();
    }

    public void resetController() {
        pidf.resetControl();
    }

    public void update() {
        pidf.update();

        double currentPosition = getPosition();
        double posDelta = currentPosition - lastPosition;
        if (posDelta < -Math.PI) {
            // jumped from high to low - overflow
            rotationsOffset += 2 * Math.PI;
        } else if (posDelta > Math.PI) {
            // jumped from low to high - underflow
            rotationsOffset -= 2 * Math.PI;
        }
        lastPosition = currentPosition;
    }
}
