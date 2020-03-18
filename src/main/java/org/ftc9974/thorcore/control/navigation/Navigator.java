package org.ftc9974.thorcore.control.navigation;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.MovementStrategy;
import org.ftc9974.thorcore.control.navigation.NavSource;
import org.ftc9974.thorcore.util.OpModeUtilities;

import java.util.concurrent.atomic.AtomicBoolean;

public final class Navigator implements OpModeManagerNotifier.Notifications {

    private static final String TAG = "org.ftc9974.thorcore.control.navigation.Navigator";

    private NavSource navSource;
    private HolonomicDrivetrain drivetrain;
    private MovementStrategy movementStrategy;

    private Vector2 targetPosition;
    private double targetHeading;

    private final Object movementLock = new Object();

    private AtomicBoolean enabled, inUse, hasPosition, hasHeading, allowMovement, allowTurning, raceBreaker;

    private Thread navigationThread;

    public Navigator(NavSource navSource, HolonomicDrivetrain drivetrain, MovementStrategy movementStrategy) {
        this.navSource = navSource;
        this.drivetrain = drivetrain;
        this.movementStrategy = movementStrategy;
        enabled = new AtomicBoolean(false);
        inUse = new AtomicBoolean(true);
        hasPosition = new AtomicBoolean(false);
        hasHeading = new AtomicBoolean(false);
        allowMovement = new AtomicBoolean(true);
        allowTurning = new AtomicBoolean(true);
        raceBreaker = new AtomicBoolean(false);

        navigationThread = new Thread(this::handleNavigation);

        targetPosition = new Vector2(0, 0);
        targetHeading = 0;

        OpModeUtilities.registerListener(this);

        drivetrain.resetEncoders();

        navigationThread.start();
    }

    public void setTargetPosition(Vector2 position) {
        synchronized (movementLock) {
            movementStrategy.reset();
            targetPosition = position;
            hasPosition.set(true);
        }
    }

    public void setTargetHeading(double heading) {
        synchronized (movementLock) {
            movementStrategy.reset();
            targetHeading = heading;
            hasHeading.set(true);
        }
    }

    public void setTarget(Vector2 position, double heading) {
        synchronized (movementLock) {
            movementStrategy.reset();
            targetPosition = position;
            targetHeading = heading;
            hasPosition.set(true);
            hasHeading.set(true);
        }
    }

    public void setAllowMovement(boolean allow) {
        synchronized (movementLock) {
            allowMovement.set(allow);
        }
    }

    public void setAllowTurning(boolean allow) {
        synchronized (movementLock) {
            allowTurning.set(allow);
        }
    }

    public Vector2 getTargetPosition() {
        return targetPosition;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public boolean atTarget() {
        synchronized (movementLock) {
            return (!hasPosition.get() || movementStrategy.atPositionalTarget()) &&
                    (!hasHeading.get() || movementStrategy.atHeadingTarget());
        }
    }

    private void handleNavigation() {
        while (inUse.get()) {
            if (enabled.get()) {
                Vector2 currentPosition = navSource.getLocation();
                double currentHeading = navSource.getHeading();
                synchronized (movementLock) {
                    double[] movement = movementStrategy.calculateMovement(drivetrain, currentPosition, currentHeading, targetPosition, targetHeading);
                    drivetrain.drive(
                            allowMovement.get() ? movement[0] : 0,
                            allowMovement.get() ? movement[1] : 0,
                            allowTurning.get() ? movement[2] : 0);
                }
            }
            raceBreaker.set(false);
        }
    }

    public void setEnabled(boolean enabled) {
        // technically, we shouldn't need thread-safe operations here,
        // since the navigation thread isn't running at this point.
        // but I'd rather be safe than sorry.
        synchronized (movementLock) {
            movementStrategy.reset();
            this.enabled.set(enabled);
            if (!enabled) {
                // stop the robot if we're disabled
                drivetrain.drive(0, 0, 0);
            }
        }
        // resolve the race condition by waiting for handleNavigation() to do an update, or timeout
        raceBreaker.set(true);
        final long startTime = SystemClock.uptimeMillis();
        while (raceBreaker.get() && SystemClock.uptimeMillis() - startTime < 100);
        if (!raceBreaker.get()) {
            RobotLog.ii(TAG, "Race Broken!");
        }
    }

    public boolean isEnabled() {
        return enabled.get();
    }

    public void shutdown() {
        inUse.set(false);
        try {
            navigationThread.join(1000);
        } catch (InterruptedException e) {
            RobotLog.ee(TAG, e, "Interrupted while joining navigation handler");
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        if (inUse.get()) {
            RobotLog.ww(TAG, "This Navigator instance was not shut down correctly! Shutting down due to OpMode stop");
            shutdown();
        }
        OpModeUtilities.unregisterListener(this);
    }
}
