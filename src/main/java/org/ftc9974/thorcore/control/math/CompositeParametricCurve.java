package org.ftc9974.thorcore.control.math;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;

/**
 * Defines a piecewise {@link ParametricCurve} by stitching together multiple ParametricCurves.
 *
 * First and second derivatives take the chain rule into account.
 */
public class CompositeParametricCurve extends ParametricCurve {

    /**
     * All of the constituent curves that make up this CompositeParametricCurve.
     *
     * The get(), derivative(), and secondDerivative() methods will likely need random-access
     * to the segment list, so an ArrayList is used.
     */
    private final ArrayList<ParametricCurve> segments;

    /**
     * Constructs a new CompositeParametricCurve from the supplied constituent curves.
     *
     * The constituent curves are added in the order they were supplied.
     *
     * @param curveSegments segments of the new curve
     */
    public CompositeParametricCurve(ParametricCurve... curveSegments) {
        segments = new ArrayList<>(Arrays.asList(curveSegments));
    }

    /**
     * Gets the position of the curve at the specified parameter.
     *
     * The t-value range is split into equal segments for each segment in this curve. For example,
     * a CompositeParametricCurve with 2 segments will split the t-value range into 2 segments:
     * [0, 0.5) and [0.5, 1]. 3 segments will have the ranges [0, 0.33), [0.33, 0.66), [0.66, 1].
     * Note the difference between the bounds of the ranges. The start of a range is inclusive, and
     * the end is exclusive - with the exception of the last range, whose end is inclusive.
     *
     * The range that the supplied value of t falls in determines which curve segment is used. For
     * example, if the t-value falls in the first range the first segment is used. If it falls in
     * the third range, the third segment is used. The t-value is scaled back to the range [0, 1]
     * and passed to the segment. The returned position of the curve is then returned.
     *
     * @param t curve parameter
     * @return position
     */
    @Override
    public Vector2 get(double t) {
        // calculate which range the t-value falls in.
        int segmentIndex = (int) Math.floor(t * getNumSegments());
        // if t is exactly 1, then segmentIndex will be equal to getNumSegments(). this would cause
        // an index-out-of-bounds, so decrement segmentIndex in that case.
        if (segmentIndex == getNumSegments()) {
            segmentIndex--;
        }

        // u is the parameter to the curve at segmentIndex.
        // we multiply be the number of segments to put t in the range [0, numSegments]. we then
        // subtract our segment index. if our segmentIndex was determined properly, this yields a
        // value from 0 to 1. This effectively is mapping t from
        // [segmentIndex / numSegments, (segmentIndex + 1) / numSegments] to [0, 1].
        double u = t * getNumSegments() - segmentIndex;

        // evaluate the segment and return the result.
        return segments.get(segmentIndex).get(u);
    }

    /**
     * Gets the first derivative of the curve at the specified parameter.
     *
     * The t-value range is split into equal segments for each segment in this curve. For example,
     * a CompositeParametricCurve with 2 segments will split the t-value range into 2 segments:
     * [0, 0.5) and [0.5, 1]. 3 segments will have the ranges [0, 0.33), [0.33, 0.66), [0.66, 1].
     * Note the difference between the bounds of the ranges. The start of a range is inclusive, and
     * the end is exclusive - with the exception of the last range, whose end is inclusive.
     *
     * The range that the supplied value of t falls in determines which curve segment is used. For
     * example, if the t-value falls in the first range the first segment is used. If it falls in
     * the third range, the third segment is used. The t-value is scaled back to the range [0, 1]
     * and passed to the segment. The returned first derivative of the curve is then returned.
     *
     * @param t curve parameter
     * @return first derivative
     */
    @Override
    public Vector2 derivative(double t) {
        // calculate which range the t-value falls in.
        int segmentIndex = (int) Math.floor(t * getNumSegments());
        // if t is exactly 1, then segmentIndex will be equal to getNumSegments(). this would cause
        // an index-out-of-bounds, so decrement segmentIndex in that case.
        if (segmentIndex == getNumSegments()) {
            segmentIndex--;
        }

        // u is the parameter to the curve at segmentIndex.
        // we multiply be the number of segments to put t in the range [0, numSegments]. we then
        // subtract our segment index. if our segmentIndex was determined properly, this yields a
        // value from 0 to 1. This effectively is mapping t from
        // [segmentIndex / numSegments, (segmentIndex + 1) / numSegments] to [0, 1].
        double u = t * getNumSegments() - segmentIndex;

        // this method (CompositeParametricCurve.derivative()) must return the derivative of get()
        // with respect to t. Similarly, ParametricCurve.derivative() is the derivative of
        // ParametricCurve.get() with respect to its given parameter (in this case, u). However,
        // u != t. thus, the chain rule is required.
        // B(t) = B_i(u) = B_i(t * s)
        // dB/dt = dB_i/dt = dB_i/du * du/dt = s*B_i(t * s)
        return segments.get(segmentIndex).derivative(u).scalarMultiply(getNumSegments());
    }

    /**
     * Gets the second derivative of the curve at the specified parameter.
     *
     * The t-value range is split into equal segments for each segment in this curve. For example,
     * a CompositeParametricCurve with 2 segments will split the t-value range into 2 segments:
     * [0, 0.5) and [0.5, 1]. 3 segments will have the ranges [0, 0.33), [0.33, 0.66), [0.66, 1].
     * Note the difference between the bounds of the ranges. The start of a range is inclusive, and
     * the end is exclusive - with the exception of the last range, whose end is inclusive.
     *
     * The range that the supplied value of t falls in determines which curve segment is used. For
     * example, if the t-value falls in the first range the first segment is used. If it falls in
     * the third range, the third segment is used. The t-value is scaled back to the range [0, 1]
     * and passed to the segment. The returned second derivative of the curve is then returned.
     *
     * @param t curve parameter
     * @return second derivative
     */
    @Override
    public Vector2 secondDerivative(double t) {
        // calculate which range the t-value falls in.
        int segmentIndex = (int) Math.floor(t * getNumSegments());
        // if t is exactly 1, then segmentIndex will be equal to getNumSegments(). this would cause
        // an index-out-of-bounds, so decrement segmentIndex in that case.
        if (segmentIndex == getNumSegments()) {
            segmentIndex--;
        }

        // u is the parameter to the curve at segmentIndex.
        // we multiply be the number of segments to put t in the range [0, numSegments]. we then
        // subtract our segment index. if our segmentIndex was determined properly, this yields a
        // value from 0 to 1. This effectively is mapping t from
        // [segmentIndex / numSegments, (segmentIndex + 1) / numSegments] to [0, 1].
        double u = t * getNumSegments() - segmentIndex;

        // as with derivative(), the chain rule must be applied.
        return segments.get(segmentIndex).secondDerivative(u).scalarMultiply(getNumSegments() * getNumSegments());
    }

    /**
     * Finds the point on this curve closest to the supplied point.
     *
     * @param p point to project onto the curve
     * @return closest point
     */
    @Override
    public ClosestPoint findPointClosestTo(Vector2 p) {
        // iterate through the each segment, finding the closest point on each. the closest of those
        // points is the closest point overall.

        int closestSegmentIndex = 0;
        ClosestPoint closestPoint = segments.get(0).findPointClosestTo(p);
        for (int i = 1; i < getNumSegments(); i++) {
            ClosestPoint potentialClosestPoint = segments.get(i).findPointClosestTo(p);
            if (potentialClosestPoint.pToPoint.getMagnitude() < closestPoint.pToPoint.getMagnitude()) {
                closestPoint = potentialClosestPoint;
                closestSegmentIndex = i;
            }
        }

        return new ClosestPoint(
                closestPoint.point,
                closestPoint.firstDeriv.scalarMultiply(getNumSegments()),
                closestPoint.secondDeriv.scalarMultiply(getNumSegments() * getNumSegments()),
                closestPoint.pToPoint,
                (closestPoint.t + closestSegmentIndex) / getNumSegments(),
                closestPoint.accurateTo
        );
    }

    /**
     * Finds the point on this curve on the specified range closest to the specified point.
     *
     * @param p point to project onto the curve
     * @param start lower bound of the curve parameter
     * @param end upper bound of the curve parameter
     * @return closest point that lies on the curve at a t-value between the start and end parameters.
     */
    @Override
    public ClosestPoint findPointClosestTo(Vector2 p, double start, double end) {
        int startSegmentIndex = (int) Math.floor(start * getNumSegments());
        // if start is exactly 1, then startSegmentIndex will be equal to getNumSegments().
        if (startSegmentIndex == getNumSegments()) {
            startSegmentIndex--;
        }

        double startU = start * getNumSegments() - startSegmentIndex;

        int endSegmentIndex = (int) Math.floor(end * getNumSegments());
        if (endSegmentIndex == getNumSegments()) {
            endSegmentIndex--;
        }

        double endU = end * getNumSegments() - endSegmentIndex;

        int closestSegmentIndex = 0;
        ClosestPoint closestPoint = null;
        if (startSegmentIndex == endSegmentIndex) {
            closestPoint = segments.get(startSegmentIndex).findPointClosestTo(p, startU, endU);
            closestSegmentIndex = startSegmentIndex;
        } else {
            for (int i = startSegmentIndex; i <= endSegmentIndex; i++) {
                ClosestPoint potentialClosestPoint;
                if (i == startSegmentIndex) {
                    potentialClosestPoint = segments.get(i).findPointClosestTo(p, startU, 1);
                } else if (i == endSegmentIndex) {
                    potentialClosestPoint = segments.get(i).findPointClosestTo(p, 0, endU);
                } else {
                    potentialClosestPoint = segments.get(i).findPointClosestTo(p);
                }
                if (closestPoint == null || potentialClosestPoint.pToPoint.getMagnitude() < closestPoint.pToPoint.getMagnitude()) {
                    closestPoint = potentialClosestPoint;
                    closestSegmentIndex = i;
                }
            }
        }

        return new ClosestPoint(
                closestPoint.point,
                closestPoint.firstDeriv.scalarMultiply(getNumSegments()),
                closestPoint.secondDeriv.scalarMultiply(getNumSegments() * getNumSegments()),
                closestPoint.pToPoint,
                (closestPoint.t + closestSegmentIndex) / getNumSegments(),
                closestPoint.accurateTo
        );
    }

    @Override
    public double length() {
        double sum = 0;
        for (ParametricCurve segment : segments) {
            sum += segment.length();
        }
        return sum;
    }

    @Override
    public double segmentLength(double start, double end) {
        int startSegmentIndex = (int) Math.floor(start * getNumSegments());
        // if start is exactly 1, then startSegmentIndex will be equal to getNumSegments().
        if (startSegmentIndex == getNumSegments()) {
            startSegmentIndex--;
        }

        double startU = start * getNumSegments() - startSegmentIndex;

        int endSegmentIndex = (int) Math.floor(end * getNumSegments());
        if (endSegmentIndex == getNumSegments()) {
            endSegmentIndex--;
        }

        double endU = end * getNumSegments() - endSegmentIndex;

        if (startSegmentIndex == endSegmentIndex) {
            return segments.get(startSegmentIndex).segmentLength(startU, endU);
        } else {
            double sum = segments.get(startSegmentIndex).segmentLength(startU, 1) +
                    segments.get(endSegmentIndex).segmentLength(0, endU);
            for (int i = startSegmentIndex + 1; i < endSegmentIndex; i++) {
                sum += segments.get(i).length();
            }
            return sum;
        }
    }

    /**
     * Adds a segment to the end of this curve.
     *
     * @param segment segment to add
     */
    public void addSegment(ParametricCurve segment) {
        segments.add(segment);
    }

    public void addSegments(ParametricCurve... segments) {
        for (ParametricCurve segment : segments) {
            addSegment(segment);
        }
    }

    /**
     * Sets the segment at the specified index. The segment that was previously at the index is
     * replaced.
     *
     * @param index index to set at
     * @param newSegment segment to place at the index
     */
    public void setSegment(int index, ParametricCurve newSegment) {
        segments.set(index, newSegment);
    }

    /**
     * Inserts a segment into this curve at the specified index. The segment that was previously at
     * the index, as well as any segments after it, are shifted over (have their index incremented).
     *
     * @param index index to insert at
     * @param segment segment to insert
     */
    public void insertSegment(int index, ParametricCurve segment) {
        segments.add(index, segment);
    }

    /**
     * Removes the segment at the specified index.
     *
     * @param index index of the segment to remove
     */
    public void removeSegment(int index) {
        segments.remove(index);
    }

    /**
     * Removes segments within the specified index range.
     *
     * @param start start of the index range, inclusive
     * @param end end of the index range, exclusive
     */
    public void removeSegmentRange(int start, int end) {
        if (end < start) {
            throw new IllegalArgumentException("End must be greater than start.");
        }
        // subLists are backed by the original list
        segments.subList(start, end).clear();
    }

    public void clearSegments() {
        segments.clear();
    }

    /**
     * Gets the segment at the specified index.
     *
     * @param index index
     * @return segment
     */
    public ParametricCurve getSegment(int index) {
        return segments.get(index);
    }

    /**
     * Gets the number of segments in this curve.
     *
     * @return number of segments
     */
    public int getNumSegments() {
        return segments.size();
    }
}
