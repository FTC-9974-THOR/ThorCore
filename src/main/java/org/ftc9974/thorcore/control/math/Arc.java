package org.ftc9974.thorcore.control.math;

import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.util.MathUtilities;

public class Arc extends ParametricCurve {

    public enum SweepDirection {
        COUNTERCLOCKWISE(1),
        CLOCKWISE(-1);

        private final double sign;

        SweepDirection(double sign) {
            this.sign = sign;
        }
    }

    private Vector2 startPoint, endPoint, center;
    private SweepDirection direction;
    private double angle, radius, startHeading;
    private Vector2 centerToStart, centerToEnd;

    public Arc(Vector2 from, Vector2 to, Vector2 center, SweepDirection direction) {
        startPoint = from;
        endPoint = to;
        this.center = center;
        this.direction = direction;

        centerToStart = startPoint.subtract(center);
        centerToEnd = endPoint.subtract(center);

        radius = centerToStart.getMagnitude();
        if (centerToEnd.getMagnitude() != radius) {
            throw new IllegalArgumentException("The radius at the start of the arc is different from the radius at the end of the arc");
        }

        double dot = centerToStart.dot(centerToEnd); // = r^2 cos(theta)
        // smallest angle between centerToStart and centerToEnd
        angle = Math.acos(dot / (radius * radius));

        RobotLog.vv("Arc", "start: %s end: %s center: %s radius: %f smallestAngle: %f",
                startPoint.toString(), endPoint.toString(), center.toString(), radius, angle);

        if (Math.signum(centerToStart.crossMag(centerToEnd)) != direction.sign) {
            // sweep direction goes the long way around
            angle = 2 * Math.PI - angle;
            RobotLog.vv("Arc", "reflex");
        }

        angle = Math.copySign(angle, direction.sign);
        RobotLog.vv("Arc", "angle: %f", angle);

        startHeading = centerToStart.getHeading();
    }

    @Override
    public Vector2 get(double t) {
        return new Vector2(
                radius * Math.cos(angle * t + startHeading) + center.getX(),
                radius * Math.sin(angle * t + startHeading) + center.getY()
        );
    }

    @Override
    public Vector2 derivative(double t) {
        return new Vector2(
                -angle * radius * Math.sin(angle * t + startHeading),
                angle * radius * Math.cos(angle * t + startHeading)
        );
    }

    @Override
    public Vector2 secondDerivative(double t) {
        return new Vector2(
                -angle * angle * radius * Math.cos(angle * t + startHeading),
                -angle * angle * radius * Math.sin(angle * t + startHeading)
        );
    }

    /*@Override
    public ClosestPoint findPointClosestTo(Vector2 p) {
        return findPointClosestTo(p, 0, 1);
    }

    @Override
    public ClosestPoint findPointClosestTo(Vector2 p, double start, double end) {
        Vector2 onArc = p.subtract(center).normalized().scalarMultiply(radius);
        double onArcHeading = onArc.getHeading();
        if (direction == SweepDirection.COUNTERCLOCKWISE) {
            while (onArcHeading < startHeading) {
                onArcHeading += 2 * Math.PI;
            }
        } else {
            while (onArcHeading > startHeading) {
                onArcHeading -= 2 * Math.PI;
            }
        }
        double startToOnArc = Math.abs(onArcHeading - (startHeading + angle * start));
        double endToOnArc = Math.abs(onArcHeading - (startHeading + angle * end));
        double t = startToOnArc / Math.abs(angle);
        if (startToOnArc > Math.abs(angle)) {
            if (startToOnArc < endToOnArc) {
                onArc = centerToStart;
                t = 0;
            } else {
                onArc = centerToEnd;
                t = 1;
            }
        }
        return new ClosestPoint(
                onArc.add(center),
                derivative(t),
                secondDerivative(t),
                onArc.add(center).subtract(p),
                t,
                0
        );
    }*/

    @Override
    public double length() {
        return radius * Math.abs(angle);
    }

    @Override
    public double segmentLength(double start, double end) {
        return (end - start) * length();
    }
}
