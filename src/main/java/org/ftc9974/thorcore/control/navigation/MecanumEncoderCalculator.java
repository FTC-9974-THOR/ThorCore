package org.ftc9974.thorcore.control.navigation;

import androidx.annotation.Size;

import org.ftc9974.thorcore.control.math.Vector2;

public class MecanumEncoderCalculator implements EncoderPositionCalculator {

    private static final double EFFICIENCY = Math.sin(0.25 * Math.PI);
    private static final Vector2 UP_RIGHT = new Vector2(1, 1).normalized().scalarDivide(EFFICIENCY),
                                 UP_LEFT = new Vector2(-1, 1).normalized().scalarDivide(EFFICIENCY);

    private final double ticksPerUnitDistance;

    public MecanumEncoderCalculator(double gearRatio, double wheelDiameter) {
        ticksPerUnitDistance = (7 * 4 * gearRatio) / (wheelDiameter * Math.PI);
    }

    @Override
    public @Size(4) int[] calculate(Vector2 point) {
        return new int[] {
                (int) (ticksPerUnitDistance * point.dot(UP_RIGHT)),
                (int) (ticksPerUnitDistance * point.dot(UP_LEFT)),
                (int) (ticksPerUnitDistance * point.dot(UP_LEFT)),
                (int) (ticksPerUnitDistance * point.dot(UP_RIGHT))
        };
    }
}
