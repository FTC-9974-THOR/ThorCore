package org.ftc9974.thorcore.control.navigation;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.util.MathUtilities;

public enum CoordinateSystem {
    FRAME,
    CARTESIAN;

    public Vector2 convertTo(CoordinateSystem system, Vector2 point) {
        if (system == this) {
            return point;
        } else if (system == CARTESIAN) {
            return MathUtilities.frameToCartesian(point);
        } else {
            return MathUtilities.cartesianToFrame(point);
        }
    }

    // only use for atan2
    public double convertTo(CoordinateSystem system, double heading) {
        if (system == this) {
            return heading;
        } else if (system == CARTESIAN) {
            double convertedHeading = heading + MathUtilities.PIO2;
            if (convertedHeading > Math.PI) {
                convertedHeading -= 2 * Math.PI;
            }
            return convertedHeading;
        } else {
            double convertedHeading = heading - MathUtilities.PIO2;
            if (convertedHeading < -Math.PI) {
                convertedHeading += 2 * Math.PI;
            }
            return convertedHeading;
        }
    }
}
