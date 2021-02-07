package org.ftc9974.thorcore.control.math;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;

public class CompositeParametricCurve extends ParametricCurve {

    private final ArrayList<ParametricCurve> segments;

    public CompositeParametricCurve(ParametricCurve... curveSegments) {
        segments = new ArrayList<>(Arrays.asList(curveSegments));

        // make sure that the curve is contiguous - that is, each curve ends at the same point that
        // the next one begins. note that this does not check for continuity - a curve can have
        // sharp turns and still be contiguous.
        /*Iterator<ParametricCurve> iterator = segments.iterator();
        ParametricCurve previousCurve = iterator.next();
        while (iterator.hasNext()) {
            ParametricCurve currentCurve = iterator.next();
            if (!previousCurve.get(1).equals(currentCurve.get(0))) {
                throw new IllegalArgumentException(String.format(
                        "The curve is not contiguous, jumping from %s to %s.",
                        previousCurve.get(1).toString(), currentCurve.get(0).toString()
                ));
            }
            previousCurve = currentCurve;
        }*/
    }

    @Override
    public Vector2 get(double t) {
        int segmentIndex = (int) Math.floor(t * getNumSegments());
        // if t is exactly 1, then segmentIndex will be equal to getNumSegments().
        if (segmentIndex == getNumSegments()) {
            segmentIndex--;
        }

        // u is the parameter to the curve at segmentIndex.
        double u = t * getNumSegments() - segmentIndex;

        return segments.get(segmentIndex).get(u);
    }

    @Override
    public Vector2 derivative(double t) {
        int segmentIndex = (int) Math.floor(t * getNumSegments());
        // if t is exactly 1, then segmentIndex will be equal to getNumSegments().
        if (segmentIndex == getNumSegments()) {
            segmentIndex--;
        }

        // u is the parameter to the curve at segmentIndex.
        double u = t * getNumSegments() - segmentIndex;

        // this method (CompositeParametricCurve.derivative()) must return the derivative of get()
        // with respect to t. Similarly, ParametricCurve.derivative() is the derivative of
        // ParametricCurve.get() with respect to its given parameter (in this case, u). However,
        // u != t. thus, the chain rule is required.
        // B(t) = B_i(u) = B_i(t * s)
        // dB/dt = dB_i/dt = dB_i/du * du/dt = s*B_i(t * s)
        return segments.get(segmentIndex).derivative(u).scalarMultiply(getNumSegments());
    }

    @Override
    public Vector2 secondDerivative(double t) {
        int segmentIndex = (int) Math.floor(t * getNumSegments());
        // if t is exactly 1, then segmentIndex will be equal to getNumSegments().
        if (segmentIndex == getNumSegments()) {
            segmentIndex--;
        }

        // u is the parameter to the curve at segmentIndex.
        double u = t * getNumSegments() - segmentIndex;

        // as with derivative(), the chain rule must be applied.
        return segments.get(segmentIndex).secondDerivative(u).scalarMultiply(getNumSegments() * getNumSegments());
    }

    @Override
    public ClosestPoint findPointClosestTo(Vector2 p) {
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
