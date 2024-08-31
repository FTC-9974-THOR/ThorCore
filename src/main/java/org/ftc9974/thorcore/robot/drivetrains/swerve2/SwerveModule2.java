package org.ftc9974.thorcore.robot.drivetrains.swerve2;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.ftc9974.thorcore.control.SlewRateLimiter;
import org.ftc9974.thorcore.control.VoltageRegulator;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.robot.AxonCRServoPIDF;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.MotorUtilities;

public class SwerveModule2 {

    @FunctionalInterface
    public interface GainSchedule {
        PIDFCoefficients calculateGains(double wheelAngle, Vector2 moduleVelocity, Vector2 robotLinearVelocity, double robotAngularVelocity);
    }

    private static final MotorUtilities.MotorConstants feedforward = MotorUtilities.calculateMotorConstants(
            MathUtilities.rpmToRadPerSec(1780.0), 0.2281, 0.2, 12
    );

    public DcMotorEx motor;
    public AxonCRServoPIDF servo;

    public final Vector2 position;

    private final double servoOffset;
    private final double conversionFactor;
    private final double distancePerTick;

    private double servoSetpoint;
    private double motorSetpoint;
    private final VoltageRegulator voltageRegulator;

    private final GainSchedule gainSchedule;
    private final SlewRateLimiter rateLimiter = new SlewRateLimiter(100);

    private SwerveModule2 scheduleReference = this;

    public SwerveModule2(String name, HardwareMap hardwareMap, Vector2 position, double servoOffset, double conversionFactor, double distancePerTick, GainSchedule gainSchedule) {
        this.position = position;
        this.servoOffset = servoOffset;
        this.conversionFactor = conversionFactor;
        this.distancePerTick = distancePerTick;
        this.gainSchedule = gainSchedule;

        motor = hardwareMap.get(DcMotorEx.class, String.format("%s-motor", name));
        servo = new AxonCRServoPIDF(name, hardwareMap);

        servoSetpoint = 0;
        motorSetpoint = 0;
        voltageRegulator = new VoltageRegulator(hardwareMap);

        rateLimiter.setContinuityRange(0, 2.0 * Math.PI);
        rateLimiter.setContinuous(true);
    }

    public void stop() {
        servo.servo.setPower(0);
        motor.setPower(0);
    }

    public void setScheduleReference(SwerveModule2 scheduleReference) {
        this.scheduleReference = scheduleReference;
    }

    public double getDirectionSetpoint() {
        return MathUtilities.wraparound(servo.getPositionSetpoint() - servoOffset, 0, 2 * Math.PI);
    }

    public void setDirectionSetpoint(double setpoint) {
        servoSetpoint = MathUtilities.wraparound(setpoint + servoOffset, 0, 2 * Math.PI);
    }

    public void setSpeed(double speed) {
        motorSetpoint = conversionFactor * speed;
    }

    public void setVelocity(Vector2 velocity) {
        double magnitude = velocity.getMagnitude();
        if (Math.abs(magnitude) > 1e-6) { // TODO: 1/6/24 Use an epsilon
            boolean inverted = velocity.dot(Vector2.unitHeadingVector(getCurrentDirection())) < 0;

            double heading = velocity.getHeading();
            if (inverted) {
                heading += Math.PI;
                magnitude = -magnitude;
            }

            setDirectionSetpoint(heading);
            motorSetpoint = conversionFactor * magnitude;
        } else {
            motorSetpoint = 0;
        }
    }

    public Vector2 getCurrentVelocity() {
        double speed = getCurrentSpeed();
        double angle = getCurrentDirection();
        return new Vector2(speed * cos(angle), speed * sin(angle));
    }

    public double getCurrentDirection() {
        return MathUtilities.wraparound(servo.getPosition() - servoOffset, 0, 2 * Math.PI);
    }

    public double getCurrentSpeed() {
        // todo ticks to mm conversion
        return distancePerTick * motor.getVelocity();
    }

    public void update(Vector2 currentRobotLinearVelocity, double currentRobotAngularVelocity) {
        servo.setPosition(rateLimiter.update(servoSetpoint));
        servo.update();

        // todo: gain scheduling
        // reduce the pid gains when traveling at speed
        PIDFCoefficients gains = gainSchedule.calculateGains(
                scheduleReference.getCurrentDirection(),
                SwerveDrive2.calculateModuleKinematics(this, currentRobotLinearVelocity, currentRobotAngularVelocity),
                currentRobotLinearVelocity,
                currentRobotAngularVelocity
        );
        servo.setTunings(gains.p, gains.i, gains.d, gains.f);

        final double FULL_POWER_THRESHOLD = Math.toRadians(8);
        final double CUTOFF_THRESHOLD = Math.toRadians(30);
        double absError = Math.abs(servo.getLastError());
        double motorCommand = 0;
        if (absError < CUTOFF_THRESHOLD) {
            motorCommand = cos(absError) * motorSetpoint;
        }
        /*double motorCommand = 0;
        if (absError < FULL_POWER_THRESHOLD) {
            motorCommand = motorSetpoint;
        } else if (absError < CUTOFF_THRESHOLD) {
            motorCommand = MathUtilities.map(absError,
                    CUTOFF_THRESHOLD, FULL_POWER_THRESHOLD,
                    0, motorSetpoint);
        } else {
            motorCommand = 0;
        }*/

        //voltageRegulator.setTargetVoltage(feedforward.computeVoltage(0, motorCommand));
        voltageRegulator.setTargetVoltage(12 * motorCommand);
        //RobotLog.dd("SwerveModule", String.format(Locale.US, "motorCommand: %f, voltage: %f", motorCommand, voltageRegulator.getTargetVoltage()));
        motor.setPower(voltageRegulator.getRegulatedOutput());
    }

    public double getCurrentDraw() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }
}
