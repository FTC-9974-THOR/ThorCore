package org.ftc9974.thorcore.control.navigation;

import org.ftc9974.thorcore.control.math.Vector2;

public interface NavSource {

    default void update() {}

    Vector2 getLocation();

    double getHeading();

    boolean trustworthy();

    default CoordinateSystem coordinateSystem() {
        return CoordinateSystem.FRAME;
    }
}
