package org.ftc9974.thorcore.util;

import org.ftc9974.thorcore.robot.MotorType;

/**
 * Utility class for working with motors.
 */
public final class MotorUtilities {

    /**
     * Calculates ticks to go the given distance.
     *
     * Works with whatever units you use, as long as distance and wheelDiameter are given in the
     * same units.
     *
     * @param distance distance
     * @param wheelDiameter wheel diameter
     * @param gearRatio ratio between wheels and motor shaft
     * @param motorType type of motor
     * @return ticks
     */
    public static int ticksForDistance(double distance, double wheelDiameter, double gearRatio, MotorType motorType) {
        //double circumference = wheelDiameter * Math.PI;
        //double wheelRevolutions = distance / circumference;
        //double ticksPerWheelRevolution = motorType.ticksPerRevolution * gearRatio;
        //return (int) (wheelRevolutions * ticksPerWheelRevolution);
        return (int) ((distance * motorType.ticksPerRevolution * gearRatio) / (wheelDiameter * Math.PI));
    }

    public static class MotorConstants {
        public double kV, kB, kStatic;
    }

    public static MotorConstants calculateMotorConstants(double freeSpeed, double stallTorque, double startupVoltage, double nominalVoltage) {
        /*
        % stallTorque = Kv * nominalVoltage - Kb * 0
        Kv = stallTorque / nominalVoltage; % motor constant
        % 0 = Kv * nominalVoltage - Kb * freeSpeed
        % 0 = (stallTorque / nominalVoltage) * nominalVoltage - Kb * freeSpeed
        % stallTorque = Kb * freeSpeed
        Kb = stallTorque / freeSpeed; % back-EMF and bearing drag
         */
        MotorConstants ret = new MotorConstants();
        ret.kV = stallTorque / nominalVoltage; // N m / V
        ret.kStatic = ret.kV * startupVoltage; // N m
        ret.kB = (stallTorque - ret.kStatic) / freeSpeed; // N m / (rad/s)
        return ret;
    }
}
