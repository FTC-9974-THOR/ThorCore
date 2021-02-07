package org.ftc9974.thorcore.robot.sensors;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.control.SlidingAverageFilter;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.util.MathUtilities;

public class OdometerWheel {

    private final DcMotorEx encoder;
    private final double mmPerTick;
    private final Vector2 location, direction;
    private final Vector2 cartesianLocation, cartesianDirection;

    private double calibrationFactor;

    private double origin;

    private double lastTimestamp;
    private double lastPosition;
    private final SlidingAverageFilter velocityEstimate;

    /**
     * Constructs a new OdometerWheel.
     *
     * locationOnRobot and direction must be in frame space. For more information on frame space,
     * see the docs for {@link org.ftc9974.thorcore.util.MathUtilities#frameToCartesian(Vector2)}.
     *
     * @param motor the DcMotorEx corresponding to motor port the odometer encoder is plugged into
     * @param wheelDiameter diameter of the wheel, in millimeters
     * @param gearRatio ratio between revolutions of the encoder and revolutions of the wheel. if
     *                  the wheel is connected directly to the encoder, gearRatio should be 1.
     *                  gearRatio can be calculated by the following equation:
     *                  gearRatio = encoder revolutions / wheel revolutions
     * @param ticksPerRevolution ticks per revolution of the encoder
     * @param locationOnRobot the physical location on the robot of the center of the odometer
     *                        wheel. this should be in frame space.
     * @param direction unit vector pointing in the same direction as the wheel. this vector should
     *                  be normal to the line going through the axle of the wheel and have a
     *                  magnitude of 1. when the robot moves in the same direction as this vector,
     *                  the encoder should count up.
     */
    public OdometerWheel(DcMotorEx motor, double wheelDiameter, double gearRatio, double ticksPerRevolution, Vector2 locationOnRobot, Vector2 direction) {
        encoder = motor;
        mmPerTick = (wheelDiameter * Math.PI) / (gearRatio * ticksPerRevolution);
        location = locationOnRobot;
        // out of paranoia, normalize the direction vector
        this.direction = direction.normalized();
        cartesianLocation = MathUtilities.frameToCartesian(locationOnRobot);
        cartesianDirection = MathUtilities.frameToCartesian(direction).normalized();

        calibrationFactor = 1;

        origin = 0;

        lastTimestamp = 0;
        velocityEstimate = new SlidingAverageFilter(3, 0);
        lastPosition = 0;
    }

    /**
     * Gets the current position of the odometer, in millimeters.
     *
     * Position refers to the distance the odometer wheel has travelled.
     *
     * This position should be relative to where the odometer was at the last call to reset(), or 0
     * if reset() has not yet been called.
     *
     * @return position
     */
    public double getPosition() {
        double position = encoder.getCurrentPosition();
        if (encoder.getDirection() == DcMotorSimple.Direction.REVERSE) {
            position = -position;
        }
        if (position != lastPosition) {
            long currentTimestamp = System.nanoTime();
            double deltaTime = (currentTimestamp - lastTimestamp) / 1e9;
            velocityEstimate.update((position - lastPosition) / deltaTime);
            lastTimestamp = currentTimestamp;
            lastPosition = position;
        }
        return position * mmPerTick - origin;
    }

    /**
     * Gets the first derivative of getPosition().
     *
     * @return velocity, in mm/s
     */
    public double getVelocity() {
        double rawVelocity = encoder.getVelocity();
        if (encoder.getDirection() == DcMotorSimple.Direction.REVERSE) {
            rawVelocity = -rawVelocity;
        }
        if (Double.isNaN(lastPosition)) {
            return rawVelocity * mmPerTick;
        } else {
            double fixed = inverseOverflow(rawVelocity, velocityEstimate.get());
            RobotLog.vv("OdometerWheel", "real: %f estimate: %f fixed: %f (%f mm/s)", rawVelocity, velocityEstimate.get(), fixed, fixed * mmPerTick);
            return fixed * mmPerTick;
        }
    }

    /**
     * Gets the location of the wheel on the robot in frame space.
     *
     * @return location
     */
    public Vector2 getLocationOnRobot() {
        return location;
    }

    /**
     * Gets the direction of the wheel in frame space.
     *
     * @return direction
     */
    public Vector2 getDirection() {
        return direction;
    }

    /**
     * Gets the location of the wheel on the robot in Cartesian space.
     *
     * @return location
     */
    public Vector2 getCartesianLocationOnRobot() {
        return cartesianLocation;
    }

    /**
     * Gets the direction of the wheel in Cartesian space.
     *
     * @return direction
     */
    public Vector2 getCartesianDirectionOnRobot() {
        return cartesianDirection;
    }

    public DcMotorEx getEncoderMotor() {
        return encoder;
    }

    public double getCalibrationFactor() {
        return calibrationFactor;
    }

    public void setCalibrationFactor(double calibrationFactor) {
        this.calibrationFactor = calibrationFactor;
    }

    public double getMmPerTick() {
        return mmPerTick;
    }

    public void reset() {
        origin = encoder.getCurrentPosition() * mmPerTick;
        if (encoder.getDirection() == DcMotorSimple.Direction.REVERSE) {
            origin = -origin;
        }
    }

    // many thanks to henopied
    // https://github.com/acmerobotics/road-runner-quickstart/pull/90
    private static double inverseOverflow(double velocity, double estimate) {
        while (Math.abs(estimate - velocity) > 0x8000) {
            velocity += Math.signum(estimate - velocity) * 0x10000;
        }
        return velocity;
    }
}
