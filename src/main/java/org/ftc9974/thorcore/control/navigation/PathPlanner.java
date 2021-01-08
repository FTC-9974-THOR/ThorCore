package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.TrapezoidalMotionProfile;
import org.ftc9974.thorcore.control.math.CompositeParametricCurve;
import org.ftc9974.thorcore.control.math.LineSegment;
import org.ftc9974.thorcore.control.math.ParametricCurve;
import org.ftc9974.thorcore.control.math.Vector2;

public class PathPlanner {

    public enum HeadingMode {
        MANUAL,
        FOLLOW_PATH,
        POINT_AT_TARGET
    }

    private HolonomicDrivetrain rb;
    private NavSource navSource;
    // temporary public
    public CompositeParametricCurve path;
    private GuidingVectorField gvf;

    private boolean hasPath;

    private CoordinateSystem coordinateSystem;

    private HeadingMode headingMode;
    private PIDF headingPid;
    private double targetHeading;
    private Vector2 headingTargetPoint;

    private TrapezoidalMotionProfile speedProfile;
    private double cruiseSpeed, rampDownDistance, crawlSpeed, crawlDistance;

    private double atPositionalTargetThreshold;
    private double lastDistanceAlongPath;

    public PathPlanner(HolonomicDrivetrain drivetrain, NavSource navSource,
                       double kC, double kP, double convergenceDistance,
                       double cruiseSpeed, double rampDownDistance,
                       double crawlSpeed, double crawlDistance,
                       double atPositionalTargetThreshold,
                       double atHeadingTargetThreshold,
                       PIDFCoefficients headingPidCoefs) {
        rb = drivetrain;
        this.navSource = navSource;
        path = new CompositeParametricCurve(new LineSegment(Vector2.ZERO, Vector2.ZERO));
        gvf = new GuidingVectorField(path, kC, kP);
        gvf.setConvergenceDistance(convergenceDistance);
        gvf.setConvergenceEnabled(true);

        coordinateSystem = CoordinateSystem.CARTESIAN;

        this.cruiseSpeed = cruiseSpeed;
        this.rampDownDistance = rampDownDistance;
        this.crawlSpeed = crawlSpeed;
        this.crawlDistance = crawlDistance;
        generateMotionProfile();

        this.atPositionalTargetThreshold = atPositionalTargetThreshold;
        lastDistanceAlongPath = -1;

        headingMode = HeadingMode.FOLLOW_PATH;
        headingPid = new PIDF(headingPidCoefs);
        headingPid.setAtTargetThreshold(atHeadingTargetThreshold);
        headingPid.setContinuityRange(-Math.PI, Math.PI);
        headingPid.setContinuous(true);
    }

    public void setCruiseSpeed(double speed) {
        cruiseSpeed = speed;
        generateMotionProfile();
    }

    public void setRampDownDistance(double distance) {
        rampDownDistance = distance;
        generateMotionProfile();
    }

    public void setCrawlSpeed(double speed) {
        crawlSpeed = speed;
        generateMotionProfile();
    }

    public void setCrawlDistance(double distance) {
        crawlDistance = distance;
    }

    private void generateMotionProfile() {
        speedProfile = new TrapezoidalMotionProfile(
                new TrapezoidalMotionProfile.Node(rampDownDistance, cruiseSpeed),
                new TrapezoidalMotionProfile.Node(crawlDistance, crawlSpeed)
        );
    }

    public void setCoordinateSystem(CoordinateSystem coordinateSystem) {
        this.coordinateSystem = coordinateSystem;
    }

    public void setConvergenceDistance(double distance) {
        gvf.setConvergenceDistance(distance);
    }

    public void setHeadingMode(HeadingMode mode) {
        headingMode = mode;
    }

    public void setTargetHeading(double heading) {
        targetHeading = heading;
    }

    public void setHeadingTargetPoint(final Vector2 point) {
        headingTargetPoint = point;
    }

    public void setHeadingPidInversion(boolean inverted) {
        headingPid.setPhase(inverted);
    }

    public void add(ParametricCurve... segments) {
        if (!hasPath) {
            path.removeSegment(0);
            hasPath = true;
        }
        path.addSegments(segments);
    }

    public void move() {
        navSource.update();
        if (!hasPath) {
            return;
        }

        Vector2 currentLocation = navSource.getLocation();
        double currentHeading = navSource.getHeading();

        // todo have the conversion done in NavSource by specifying coordinate system as a parameter
        currentLocation = navSource.coordinateSystem().convertTo(coordinateSystem, currentLocation);

        GuidingVectorField.Guidance guidance = gvf.guidance(currentLocation);

        Vector2 robotRelativeGuidanceVector = coordinateSystem.convertTo(CoordinateSystem.FRAME,
                Vector2.rotate(guidance.direction, -currentHeading));

        double distance = guidance.distanceToPath + guidance.distanceAlongPath;
        double speed = speedProfile.apply(distance);

        while (currentHeading > Math.PI) {
            currentHeading -= 2 * Math.PI;
        }
        while (currentHeading < -Math.PI) {
            currentHeading += 2 * Math.PI;
        }

        switch (headingMode) {
            case FOLLOW_PATH:
                headingTargetPoint = currentLocation.add(guidance.direction);
            case POINT_AT_TARGET:
                Vector2 toTarget = headingTargetPoint.subtract(currentLocation);
                targetHeading = CoordinateSystem.CARTESIAN.convertTo(coordinateSystem, toTarget.getHeading());
            case MANUAL:
                headingPid.setSetpoint(targetHeading);
                break;
        }

        if (atPositionalTarget() && !atHeadingTarget()) {
            headingPid.setNominalOutputForward(0.22);
            headingPid.setNominalOutputReverse(-0.22);
        } else {
            headingPid.setNominalOutputForward(0);
            headingPid.setNominalOutputReverse(0);
        }

        rb.drive(
                speed * robotRelativeGuidanceVector.getX(),
                speed * robotRelativeGuidanceVector.getY(),
                headingPid.update(currentHeading)
        );

        lastDistanceAlongPath = guidance.distanceAlongPath;
    }

    public boolean atPositionalTarget() {
        return lastDistanceAlongPath > 0 && lastDistanceAlongPath < atPositionalTargetThreshold;
    }

    public boolean atHeadingTarget() {
        return lastDistanceAlongPath > 0 && headingPid.atTarget();
    }
}
