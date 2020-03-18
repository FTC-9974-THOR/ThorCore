package org.ftc9974.thorcore.control.navigation;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.math.Vector2;

public interface MovementStrategy {

    double[] calculateMovement(HolonomicDrivetrain drivetrain, Vector2 currentPosition, double currentHeading, Vector2 targetPosition, double targetHeading);

    boolean atPositionalTarget();
    boolean atHeadingTarget();

    default void reset() {}
    default void onNewPositionalTarget(Vector2 target) {}
    default void onNewHeadingTarget(double target) {}
    default void onNewPositionalAndHeadingTarget(Vector2 targetPosition, double targetHeading) {}
}
