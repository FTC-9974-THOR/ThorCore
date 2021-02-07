package org.ftc9974.thorcore.control.navigation;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.math.LineSegment;
import org.ftc9974.thorcore.control.math.ParametricCurve;
import org.ftc9974.thorcore.control.math.Vector2;

public class Fusion3 {

    public enum RelativeTo {
        ROBOT,
        FIELD
    }

    private final LinearOpMode root;
    private final HolonomicDrivetrain rb;
    private final NavSource navSource;
    private final PathPlanner pathPlanner;

    private RelativeTo relativeTo;
    private CoordinateSystem coordinateSystem;

    public Fusion3(LinearOpMode root, HolonomicDrivetrain drivetrain, NavSource navSource, PIDFCoefficients pidfCoefficients) {
        this(
                root, drivetrain, navSource, pidfCoefficients,
                1 / 200.0, 100.0, 300.0,
                1, 200, 0.22, 50,
                50, 0.01 * Math.PI,
                RelativeTo.ROBOT, CoordinateSystem.FRAME
        );
    }

    public Fusion3(LinearOpMode root, HolonomicDrivetrain drivetrain, NavSource navSource, PIDFCoefficients pidfCoefficients,
                   double kC, double kP, double convergenceDistance,
                   double cruiseSpeed, double rampDownDistance, double crawlSpeed, double crawlDistance,
                   double positionalTolerance, double headingTolerance,
                   RelativeTo relativeTo, CoordinateSystem coordinateSystem) {
        this.root = root;
        rb = drivetrain;
        this.navSource = navSource;
        pathPlanner = new PathPlanner(
                rb, this.navSource,
                kC, kP, convergenceDistance,
                cruiseSpeed, rampDownDistance, crawlSpeed, crawlDistance,
                positionalTolerance, headingTolerance,
                pidfCoefficients
        );
        this.relativeTo = relativeTo;
        this.coordinateSystem = coordinateSystem;
        pathPlanner.setCoordinateSystem(this.coordinateSystem);
    }

    public void setKC(double kC) {
        pathPlanner.setKC(kC);
    }

    public void setKP(double kP) {
        pathPlanner.setKP(kP);
    }

    public void setConvergenceDistance(double distance) {
        pathPlanner.setConvergenceDistance(distance);
    }

    public void setCruiseSpeed(double speed) {
        pathPlanner.setCruiseSpeed(speed);
    }

    public void setRampDownDistance(double distance) {
        pathPlanner.setRampDownDistance(distance);
    }

    public void setCrawlSpeed(double speed) {
        pathPlanner.setCrawlSpeed(speed);
    }

    public void setCrawlDistance(double distance) {
        pathPlanner.setCrawlDistance(distance);
    }

    public void setReference(RelativeTo reference) {
        relativeTo = reference;
    }

    public RelativeTo getReference() {
        return relativeTo;
    }

    public void setCoordinateSystem(CoordinateSystem coordinateSystem) {
        this.coordinateSystem = coordinateSystem;
    }

    public CoordinateSystem getCoordinateSystem() {
        return coordinateSystem;
    }

    public void driveToPoint(Vector2 point) {
        driveToPoint(point, relativeTo, null);
    }

    public void driveToPoint(Vector2 point, RelativeTo reference) {
        driveToPoint(point, reference, null);
    }

    public void driveToPoint(Vector2 point, @Nullable Runnable whileRunning) {
        driveToPoint(point, relativeTo, whileRunning);
    }

    public void driveToPoint(Vector2 point, RelativeTo reference, @Nullable Runnable whileRunning) {
        navSource.update();

        Vector2 startPoint = navSource.getLocation(coordinateSystem);
        Vector2 endPoint;
        if (reference == RelativeTo.ROBOT) {
            endPoint = startPoint.add(point.rotate(navSource.getHeading()));
        } else {
            endPoint = point;
        }

        pathPlanner.clear();
        pathPlanner.add(new LineSegment(startPoint, endPoint));

        pathPlanner.setHeadingMode(PathPlanner.HeadingMode.MANUAL);
        pathPlanner.setTargetHeading(navSource.getHeading());

        while (!root.isStopRequested()) {
            // pathPlanner.move() calls navSource.update()
            pathPlanner.move();

            if (whileRunning != null) {
                whileRunning.run();
            }

            if (pathPlanner.atPositionalTarget()) {
                break;
            }
        }

        rb.drive(0, 0, 0);
    }

    public void turnToHeading(double heading) {
        turnToHeading(heading, relativeTo, null);
    }

    public void turnToHeading(double heading, RelativeTo reference) {
        turnToHeading(heading, reference, null);
    }

    public void turnToHeading(double heading, @Nullable Runnable whileRunning) {
        turnToHeading(heading, relativeTo, whileRunning);
    }

    public void turnToHeading(double heading, RelativeTo reference, @Nullable Runnable whileRunning) {
        navSource.update();

        pathPlanner.clear();
        pathPlanner.setHeadingMode(PathPlanner.HeadingMode.MANUAL);

        if (reference == RelativeTo.ROBOT) {
            pathPlanner.setTargetHeading(heading + navSource.getHeading());
        } else {
            pathPlanner.setTargetHeading(heading);
        }

        while (!root.isStopRequested()) {
            pathPlanner.move();

            if (whileRunning != null) {
                whileRunning.run();
            }

            if (pathPlanner.atHeadingTarget()) {
                break;
            }
        }

        rb.drive(0, 0, 0);
    }

    // todo robot relative paths
    public void followPath(@Nullable Runnable whileRunning, ParametricCurve... segments) {
        navSource.update();

        pathPlanner.clear();
        pathPlanner.add(segments);

        while (!root.isStopRequested()) {
            pathPlanner.move();

            if (whileRunning != null) {
                whileRunning.run();
            }

            if (pathPlanner.atPositionalTarget()) {
                break;
            }
        }

        rb.drive(0, 0, 0);
    }
}
