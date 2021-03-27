package org.ftc9974.thorcore.control.math;

import androidx.annotation.FloatRange;

import org.ftc9974.thorcore.util.MathUtilities;

/**
 * Represents a line segment as a {@link ParametricCurve}.
 */
public class LineSegment extends ParametricCurve {

    /**
     * Starting point of this line segment.
     */
    private final Vector2 start;

    /**
     * End point of this line segment.
     */
    private final Vector2 end;

    /**
     * Cached vector from the start to end.
     */
    private final Vector2 startToEnd;

    /**
     * Cached length of this line segment.
     */
    private final double length;

    /**
     * Constructs a new LineSegment from startPoint to endPoint.
     *
     * @param startPoint start of the LineSegment
     * @param endPoint end of the LineSegment
     */
    public LineSegment(Vector2 startPoint, Vector2 endPoint) {
        start = startPoint;
        end = endPoint;
        startToEnd = end.subtract(start);
        length = startToEnd.getMagnitude();
    }

    /**
     * Gets the position on this line segment at the parameter t.
     *
     * @param t curve parameter
     * @return position
     */
    @Override
    public Vector2 get(@FloatRange(from = 0, to = 1) double t) {
        return Vector2.lerp(start, end, t);
    }

    /**
     * Gets the first derivative of this line segment at the parameter t.
     *
     * Since this is a line segment, the first derivative is constant.
     *
     * @param t curve parameter
     * @return first derivative
     */
    @Override
    public Vector2 derivative(@FloatRange(from = 0, to = 1) double t) {
        // as t changes from 0 to 1, the line goes from start to end. thus, the unit change of get()
        // for unit change of t is just startToEnd. we don't even have to multiply it by anything,
        // because t only ranges from 0 to 1.
        return startToEnd;
    }

    /**
     * Gets the second derivative of this line segment at the parameter t.
     *
     * Since this is a line segment, the second derivative is always zero.
     *
     * @param t curve parameter
     * @return second derivative
     */
    @Override
    public Vector2 secondDerivative(double t) {
        return Vector2.ZERO;
    }

    /**
     * Finds the point on this line segment closest to the specified point.
     *
     * This uses a closed-form solution, so it is very fast.
     *
     * @param p point to project onto the curve
     * @return closest point
     */
    @Override
    public ClosestPoint findPointClosestTo(Vector2 p) {
        return findPointClosestTo(p, 0, 1);
    }

    /**
     * Finds the point on this line segment on the specified range which is closest to the specified
     * point.
     *
     * This uses a closed-form solution, so it is very fast.
     *
     * @param p point to project onto the curve
     * @param start lower bound of the curve parameter
     * @param end upper bound of the curve parameter
     * @return closest point
     */
    @Override
    public ClosestPoint findPointClosestTo(Vector2 p, double start, double end) {
        // we can use the dot product to project the point p onto this line segment.
        // l = d * (p - a)
        // where l is the distance from a to the projection of p onto this line segment
        //       d is the normalized first derivative of the line, equivalent to by startToEnd / length
        //       p is the method argument p
        //       a is the start point of this line
        //       * is the dot product operator
        // we can divide l by the length of this line segment to get the corresponding t-value and
        // rewrite the dot product:
        // t = (d * (p - a)) / length
        //   = (|d||p - a|cos(theta)) / length
        //   = (|startToEnd / length||p - a|cos(theta)) / length
        //   = |startToEnd||p - a|cos(theta) / (length * length)
        //   = (startToEnd * (p - a)) / (length * length)
        double t = MathUtilities.constrain(
                startToEnd.dot(p.subtract(this.start)) / (length * length),
                start,
                end
        );
        Vector2 closestPoint = get(t), pToClosestPoint = closestPoint.subtract(p);
        return new ClosestPoint(
                closestPoint, derivative(t), secondDerivative(t),
                pToClosestPoint, t, 0
        );
    }

    /**
     * Gets the length of this line segment.
     *
     * @return length
     */
    @Override
    public double length() {
        return length;
    }

    /**
     * Gets the length of the specified section of this line segment.
     *
     * @param start parameter at the start of the segment
     * @param end parameter at the end of the segment
     * @return the length along the line segment from the start parameter to the end parameter
     */
    @Override
    public double segmentLength(double start, double end) {
        // the length of any segment is proportional to the range of the start and end t-values.
        // the section we're interested in here is basically ((end - start) * 100) percent of the
        // entire line, so we can multiply the entire line's length by (end - start) to get section
        // length.
        return (end - start) * length;
    }
}
