package org.ftc9974.thorcore.control.math;

import org.ftc9974.thorcore.util.MathUtilities;

public class LineSegment extends ParametricCurve {

    private Vector2 start, end, startToEnd;
    private double length;

    public LineSegment(Vector2 startPoint, Vector2 endPoint) {
        start = startPoint;
        end = endPoint;
        startToEnd = end.subtract(start);
        length = startToEnd.getMagnitude();
    }

    @Override
    public Vector2 get(double t) {
        return Vector2.lerp(start, end, t);
    }

    @Override
    public Vector2 derivative(double t) {
        return startToEnd;
    }

    @Override
    public Vector2 secondDerivative(double t) {
        return Vector2.ZERO;
    }

    @Override
    public ClosestPoint findPointClosestTo(Vector2 p) {
        return findPointClosestTo(p, 0, 1);
    }

    @Override
    public ClosestPoint findPointClosestTo(Vector2 p, double start, double end) {
        // <a, b> * <x - p_x, y - p_y> = 0
        // a(x - p_x) = b(y - p_y)
        // <x, y> = t<a, b>
        // x = t*a, y = t*b
        // t*a*a - a*p_x = b*y - b*p_y
        // y - p_y = (t*a*a - a*p_x)/b
        // y = (t*a*a - a_p_x)/b + p_y
        //
        double t = MathUtilities.constrain(
                startToEnd.dot(p.subtract(this.start)) / (length * length),
                start,
                end
        );
        Vector2 closestPoint = get(t), pToClosestPoint = closestPoint.subtract(p);
        return new ParametricCurve.ClosestPoint(
                closestPoint, derivative(t), secondDerivative(t),
                pToClosestPoint, t, 0
        );
    }

    @Override
    public double length() {
        return length;
    }

    @Override
    public double segmentLength(double start, double end) {
        return (end - start) * length;
    }
}
