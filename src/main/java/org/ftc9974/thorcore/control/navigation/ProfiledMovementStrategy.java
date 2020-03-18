package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.robotcore.util.LastKnown;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.math.Vector2;

public final class ProfiledMovementStrategy implements MovementStrategy {

    private Vector2 startPoint, targetPoint;
    private Vector2 lastKnownPosition;

    private double totalDistance;

    private double startSpeed,
                   cruiseStart,
                   cruiseSpeed,
                   cruiseEnd,
                   crawlStart,
                   crawlSpeed;

    public ProfiledMovementStrategy() {

    }

    @Override
    public double[] calculateMovement(HolonomicDrivetrain drivetrain, Vector2 currentPosition, double currentHeading, Vector2 targetPosition, double targetHeading) {
        if (lastKnownPosition == null) {
            lastKnownPosition = currentPosition;
            startPoint = currentPosition;
        }
        Vector2 startToTarget = targetPoint.subtract(startPoint);
        double distanceRemaining = startToTarget.getMagnitude();

        // normalized movement vector
        Vector2 movementVector = startToTarget.scalarDivide(distanceRemaining);

        //double distanceTravel
        double speed = 1;

        return new double[] {0, 0, 0};
    }

    @Override
    public boolean atPositionalTarget() {
        return false;
    }

    @Override
    public boolean atHeadingTarget() {
        return false;
    }

    @Override
    public void onNewPositionalTarget(Vector2 target) {
        targetPoint = target;
        if (lastKnownPosition != null) {
            startPoint = lastKnownPosition;
            totalDistance = targetPoint.subtract(startPoint).getMagnitude();
        }
    }

    @Override
    public void onNewHeadingTarget(double target) {

    }

    @Override
    public void onNewPositionalAndHeadingTarget(Vector2 targetPosition, double targetHeading) {
        onNewPositionalTarget(targetPosition);
        onNewHeadingTarget(targetHeading);
    }
}
