package org.ftc9974.thorcore.robot.drivetrains.swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.internal.RealizableFactory;
import org.ftc9974.thorcore.util.MathUtilities;

public final class SwerveModule {

    private static final double LOWEST_ENCODER_VALUE = 0,
                                HIGHEST_ENCODER_VALUE = 3.3,
                                AT_DIRECTION_THRESHOLD = Math.toRadians(5);

    private DcMotorEx driveMotor, slewMotor;
    private AnalogInput encoder;

    private PIDF slewPid;

    @RealizableFactory
    public SwerveModule(String name, HardwareMap hw) {
        driveMotor = hw.get(DcMotorEx.class, String.format("SM-%s-driveMotor", name));
        slewMotor = hw.get(DcMotorEx.class, String.format("SM-%s-slewMotor", name));
        encoder = hw.get(AnalogInput.class, String.format("SM-%s-encoder", name));

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slewMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slewPid = new PIDF(1, 0, 0, 0);
        slewPid.setContinuityRange(0, 2.0 * Math.PI);
        slewPid.setContinuous(true);
    }

    public double getCurrentDirection() {
        return MathUtilities.map(encoder.getVoltage(), LOWEST_ENCODER_VALUE, HIGHEST_ENCODER_VALUE, 0, 2.0 * Math.PI);
    }

    public void setTargetDirection(double direction) {
        slewPid.setSetpoint(direction);
    }

    public double getTargetDirection() {
        return slewPid.getSetpoint();
    }

    public boolean atTargetDirection() {
        return Math.abs(getTargetDirection() - getCurrentDirection()) < AT_DIRECTION_THRESHOLD;
    }

    public void setDrivePower(double power) {
        driveMotor.setPower(power);
    }

    public void resetEncoder() {
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        slewMotor.setPower(slewPid.update(getCurrentDirection()));
    }
}
