package org.ftc9974.thorcore.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.internal.RealizableFactory;
import org.ftc9974.thorcore.util.MathUtilities;

public class AxonServo {
    public ServoImplEx servo;
    public AnalogInput feedback;

    public AxonServo(ServoImplEx servo, AnalogInput feedback) {
        this.servo = servo;
        this.feedback = feedback;

        servo.setPwmRange(new PwmControl.PwmRange(600, 2400));
    }

    @RealizableFactory
    public AxonServo(String name, HardwareMap hw) {
        this(
                hw.get(ServoImplEx.class, String.format("%s-servo", name)),
                hw.analogInput.get(String.format("%s-feedback", name))
        );
    }

    public void setPosition(double setpoint) {
        servo.setPosition(MathUtilities.map(setpoint, 0, 2 * Math.PI, 0, 1));
    }

    public double getPositionSetpoint() {
        return MathUtilities.map(servo.getPosition(), 0, 1, 0, 2 * Math.PI);
    }

    public double getPosition() {
        return MathUtilities.map(feedback.getVoltage(), 3.3, 0, 0, 2 * Math.PI);
    }
}
