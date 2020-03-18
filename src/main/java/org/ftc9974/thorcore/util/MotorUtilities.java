package org.ftc9974.thorcore.util;

import org.ftc9974.thorcore.robot.MotorType;

/**
 * Utility class for working with motors.
 */
public final class MotorUtilities {

    /**
     * Calculates ticks to go the given distance.
     * @param distance distance, in inches
     * @param wheelDiameter wheel diameter, in inches
     * @param gearRatio ratio between wheels and motor shaft
     * @param motorType type of motor
     * @return ticks
     */
    public static int ticksForDistance(double distance, double wheelDiameter, double gearRatio, MotorType motorType) {
        return (int) (distance * (motorType.ticksPerRevolution / (Math.PI * wheelDiameter * gearRatio)));
    }
}
