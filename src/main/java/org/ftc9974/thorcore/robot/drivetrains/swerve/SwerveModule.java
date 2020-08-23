package org.ftc9974.thorcore.robot.drivetrains.swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.internal.RealizableFactory;
import org.ftc9974.thorcore.robot.MotorType;
import org.ftc9974.thorcore.util.MathUtilities;

public final class SwerveModule {

    private static final double AT_DIRECTION_THRESHOLD = Math.toRadians(50);

    public DcMotorEx driveMotor, slewMotor;
    private AnalogInput encoder;

    private PIDF slewPid;

    private double encoderOffset;
    private boolean inverted;
    private double targetDirection;
    private boolean useQuadEncoder;
    private double lowestEncoderValue = 0;
    private double highestEncoderValue = 5;

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

    public void setUseQuadEncoder(boolean useQuadEncoder) {
        this.useQuadEncoder = useQuadEncoder;
    }

    public void setLowestEncoderValue(double lowestEncoderValue) {
        this.lowestEncoderValue = lowestEncoderValue;
    }

    public void setHighestEncoderValue(double highestEncoderValue) {
        this.highestEncoderValue = highestEncoderValue;
    }

    public void setDriveInversion(boolean inversion) {
        driveMotor.setDirection(inversion ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void setSlewInversion(boolean inversion) {
        slewMotor.setDirection(inversion ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void setPIDFTunings(double p, double i, double d, double f) {
        slewPid.setTunings(p, i, d, f);
    }

    public void setPIDFNominalOutput(double output) {
        slewPid.setNominalOutputForward(output);
    }

    public void setPIDFAtTargetThreshold(double threshold) {
        slewPid.setAtTargetThreshold(threshold);
    }

    public void setEncoderOffset(double offset) {
        encoderOffset = offset;
    }

    public double getCurrentDirection() {
        if (!useQuadEncoder) {
            double wrappedVoltage = encoder.getVoltage() - encoderOffset;
            if (wrappedVoltage < lowestEncoderValue) {
                wrappedVoltage += (highestEncoderValue - lowestEncoderValue);
            }
            return MathUtilities.map(wrappedVoltage, highestEncoderValue, lowestEncoderValue, 0, 2.0 * Math.PI);
        } else {
            double ticksPerRevolution = MotorType.YELLOWJACKET_71_2.ticksPerRevolution;
            return MathUtilities.map(
                    (slewMotor.getCurrentPosition() + encoderOffset) % ticksPerRevolution,
                    0,
                    ticksPerRevolution,
                    0,
                    2 * Math.PI
            );
        }
    }

    public void setTargetDirection(double direction) {
        targetDirection = direction;
    }

    public double getTargetDirection() {
        return targetDirection;
    }

    public boolean atTargetDirection() {
        return Math.abs(slewPid.getLastError()) < AT_DIRECTION_THRESHOLD;
    }

    public void setDrivePower(double power) {
        driveMotor.setPower((inverted) ? -power : power);
    }

    public void resetEncoder() {
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetSlewEncoder() {
        slewMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slewMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSlewPower(double power) {
        slewMotor.setPower(power);
    }

    public void update() {
        double error = targetDirection - getCurrentDirection();
        error %= 2 * Math.PI;
        if (Math.abs(error) > Math.PI) {
            if (error > 0) {
                error -= 2 * Math.PI;
            } else {
                error += 2 * Math.PI;
            }
        }
        if (Math.abs(error) > 0.5 * Math.PI) {
            inverted = true;
        } else {
            inverted = false;
        }

        if (inverted) {
            slewPid.setSetpoint(targetDirection + Math.PI);
        } else {
            slewPid.setSetpoint(targetDirection);
        }

        slewMotor.setPower(slewPid.update(getCurrentDirection()));
    }
}