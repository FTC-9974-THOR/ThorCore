package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.math.Vector2;

public final class SynchronousNavigator {

    private static final String TAG = "org.ftc9974.thorcore.control.navigation.SynchronousNavigator";

    private NavSource navSource;
    private HolonomicDrivetrain drivetrain;
    private MovementStrategy movementStrategy;

    private Vector2 targetPosition;
    private double targetHeading;

    private boolean enabled, hasPosition, hasHeading, allowMovement, allowTurning, targetDataOld, stopAtEnd;

    public SynchronousNavigator(NavSource navSource, HolonomicDrivetrain drivetrain, MovementStrategy movementStrategy) {
        this.navSource = navSource;
        this.drivetrain = drivetrain;
        this.movementStrategy = movementStrategy;
        enabled = false;
        hasPosition = false;
        hasHeading = false;
        allowMovement = true;
        allowTurning = true;

        targetPosition = new Vector2(0, 0);
        targetHeading = 0;

        drivetrain.resetEncoders();
        setStopAtEnd(true);
    }

    public void setTargetPosition(Vector2 position) {
        movementStrategy.reset();
        movementStrategy.onNewPositionalTarget(position);
        targetPosition = position;
        hasPosition = true;
    }

    public void setTargetHeading(double heading) {
        movementStrategy.reset();
        movementStrategy.onNewHeadingTarget(heading);
        targetHeading = heading;
        hasHeading = true;
    }

    public void setTarget(Vector2 position, double heading) {
        movementStrategy.reset();
        movementStrategy.onNewPositionalAndHeadingTarget(position, heading);
        targetPosition = position;
        targetHeading = heading;
        hasPosition = true;
        hasHeading = true;
    }

    public void setAllowMovement(boolean allow) {
            allowMovement = allow;
    }

    public void setAllowTurning(boolean allow) {
        allowTurning = allow;
    }

    public Vector2 getTargetPosition() {
        return targetPosition;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public boolean atTarget() {
        return !targetDataOld &&
                ((!hasPosition || movementStrategy.atPositionalTarget()) &&
                 (!hasHeading  || movementStrategy.atHeadingTarget()));
    }

    public void update() {
        if (enabled) {
            targetDataOld = false;
            Vector2 currentPosition = navSource.getLocation();
            double currentHeading = navSource.getHeading();
            double[] movement = movementStrategy.calculateMovement(drivetrain, currentPosition, currentHeading, targetPosition, targetHeading);
            if (!stopAtEnd) {
                Vector2 movementVector = new Vector2(movement[0], movement[1]);
                movementVector = movementVector.scalarDivide(movementVector.getMagnitude());
                movement[0] = movementVector.getX();
                movement[1] = movementVector.getY();
            }
            drivetrain.drive(
                    allowMovement? movement[0] : 0,
                    allowMovement ? movement[1] : 0,
                    allowTurning ? movement[2] : 0);
        }
    }

    public void setEnabled(boolean enabled) {
            movementStrategy.reset();
            this.enabled = enabled;
            if (!enabled) {
                // stop the robot if we're disabled
                drivetrain.drive(0, 0, 0);
            }
            targetDataOld = true;
            RobotLog.ii(TAG, "Enable state has changed. Marking target data old.");
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean willStopAtEnd() {
        return stopAtEnd;
    }

    public void setStopAtEnd(boolean stopAtEnd) {
        this.stopAtEnd = stopAtEnd;
    }
}
