package org.ftc9974.thorcore.control.navigation;

import org.ftc9974.thorcore.control.math.Vector2;

@FunctionalInterface
public interface EncoderPositionCalculator {

    int[] calculate(Vector2 point);

    default int getNumTargets() {
        return calculate(Vector2.ZERO).length;
    }
}
