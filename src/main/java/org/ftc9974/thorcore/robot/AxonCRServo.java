package org.ftc9974.thorcore.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.internal.RealizableFactory;
import org.ftc9974.thorcore.util.MathUtilities;

public class AxonCRServo {
    public CRServoImplEx servo;
    public AnalogInput feedback;

    private double rotationsOffset, lastPosition;

    public AxonCRServo(CRServoImplEx servo, AnalogInput feedback) {
        this.servo = servo;
        this.feedback = feedback;

        servo.setPwmRange(new PwmControl.PwmRange(600, 2400));
        servo.setPower(0);

        lastPosition = getPosition();
    }

    @RealizableFactory
    public AxonCRServo(String name, HardwareMap hw) {
        this(
                hw.get(CRServoImplEx.class, String.format("%s-servo", name)),
                hw.analogInput.get(String.format("%s-feedback", name))
        );
    }

    public void setPower(double power) {
        servo.setPower(power);
    }

    public double getPosition() {
        return MathUtilities.map(feedback.getVoltage(), 3.3, 0, 0, 2 * Math.PI);
    }

    public double getIncrementalPosition() {
        return getPosition() + rotationsOffset;
    }

    public void update() {
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
