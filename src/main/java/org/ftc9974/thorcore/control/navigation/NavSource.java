package org.ftc9974.thorcore.control.navigation;

import org.ftc9974.thorcore.control.math.Vector2;

public interface NavSource {

    Vector2 getLocation();

    double getHeading();

    boolean trustworthy();
}
