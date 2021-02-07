package org.ftc9974.thorcore.control.navigation;

import org.ftc9974.thorcore.control.math.Vector2;

public interface NavSource {

    default void update() {}

    Vector2 getLocation();

    default Vector2 getLocation(CoordinateSystem system) {
        return coordinateSystem().convertTo(system, getLocation());
    }

    double getHeading();

    boolean trustworthy();

    default CoordinateSystem coordinateSystem() {
        return CoordinateSystem.FRAME;
    }
}
