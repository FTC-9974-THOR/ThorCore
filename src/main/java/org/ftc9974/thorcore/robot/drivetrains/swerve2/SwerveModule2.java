package org.ftc9974.thorcore.robot.drivetrains.swerve2;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.ftc9974.thorcore.control.SlewRateLimiter;
import org.ftc9974.thorcore.control.VoltageRegulator;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.robot.AxonCRServoPIDF;
import org.ftc9974.thorcore.util.MathUtilities;

public class SwerveModule2 {

    @FunctionalInterface
    public interface GainSchedule {
        PIDFCoefficients calculateGains(double wheelAngle, Vector2 moduleVelocity, Vector2 robotLinearVelocity, double robotAngularVelocity);
    }

    public static GainSchedule constantGains(PIDFCoefficients coefs) {
        return (wheelAngle, moduleVelocity, robotLinearVelocity, robotAngularVelocity) -> coefs;
    }

    public DcMotorEx motor;
    public AxonCRServoPIDF servo;

    public final Vector2 position;

    private final double servoOffset;
    private final double feedforwardCoef;

    private double servoSetpoint;
    private double motorSetpoint;
    private final VoltageRegulator voltageRegulator;

    private final GainSchedule gainSchedule;
    private final SlewRateLimiter rateLimiter = new SlewRateLimiter(100);

    public SwerveModule2(String name, HardwareMap hardwareMap, Vector2 position, double servoOffset, double wheelRadius, double gearRatio, double motorFreeSpeed, GainSchedule gainSchedule) {
        this.position = position;
        this.servoOffset = servoOffset;
        this.feedforwardCoef = gearRatio / (wheelRadius * motorFreeSpeed);
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

    public double getDirectionSetpoint() {
        return MathUtilities.wraparound(servo.getPositionSetpoint() - servoOffset, 0, 2 * Math.PI);
    }

    public void setDirectionSetpoint(double setpoint) {
        servoSetpoint = MathUtilities.wraparound(setpoint + servoOffset, 0, 2 * Math.PI);
    }

    public void setSpeed(double speed) {
        motorSetpoint = feedforwardCoef * speed;
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
            motorSetpoint = feedforwardCoef * magnitude;
        } else {
            motorSetpoint = 0;
        }
    }

    public double getCurrentDirection() {
        return MathUtilities.wraparound(servo.getPosition() - servoOffset, 0, 2 * Math.PI);
    }

    public void update() {
        update(Vector2.ZERO, 0);
    }

    public void update(Vector2 currentRobotLinearVelocity, double currentRobotAngularVelocity) {
        servo.setPosition(rateLimiter.update(servoSetpoint));
        servo.update();

        PIDFCoefficients gains = gainSchedule.calculateGains(
                getCurrentDirection(),
                SwerveDrive2.calculateModuleKinematics(this, currentRobotLinearVelocity, currentRobotAngularVelocity),
                currentRobotLinearVelocity,
                currentRobotAngularVelocity
        );
        servo.setTunings(gains.p, gains.i, gains.d, gains.f);

        final double CUTOFF_THRESHOLD = Math.toRadians(30);
        double absError = Math.abs(servo.getLastError());
        double motorCommand = 0;
        if (absError < CUTOFF_THRESHOLD) {
            motorCommand = cos(absError) * motorSetpoint;
        }

        voltageRegulator.setTargetVoltage(12 * motorCommand);
        motor.setPower(voltageRegulator.getRegulatedOutput());
    }

    public double getCurrentDraw() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public void setSlewRateLimit(double limit) {
        rateLimiter.slewRate = limit;
    }
}
