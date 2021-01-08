package org.ftc9974.thorcore.robot.sensors;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.util.MathUtilities;

public class OdometerWheel {

    private final DcMotorEx encoder;
    private final double mmPerTick;
    private final Vector2 location, direction;
    private final Vector2 cartesianLocation, cartesianDirection;
    private final LynxModule connectedModule;

    private double calibrationFactor;

    private double origin;

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
    public OdometerWheel(DcMotorEx motor, double wheelDiameter, double gearRatio, double ticksPerRevolution, Vector2 locationOnRobot, Vector2 direction, LynxModule connectedRevHub) {
        encoder = motor;
        mmPerTick = (wheelDiameter * Math.PI) / (gearRatio * ticksPerRevolution);
        location = locationOnRobot;
        // out of paranoia, normalize the direction vector
        this.direction = direction.normalized();
        cartesianLocation = MathUtilities.frameToCartesian(locationOnRobot);
        cartesianDirection = MathUtilities.frameToCartesian(direction).normalized();
        connectedModule = connectedRevHub;

        calibrationFactor = 1;

        origin = 0;
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
        return encoder.getCurrentPosition() * mmPerTick - origin;
    }

    /**
     * Gets the first derivative of getPosition().
     *
     * @return velocity, in mm/s
     */
    public double getVelocity() {
        return encoder.getVelocity() * mmPerTick;
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

    public LynxModule getConnectedModule() {
        return connectedModule;
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
    }
}
