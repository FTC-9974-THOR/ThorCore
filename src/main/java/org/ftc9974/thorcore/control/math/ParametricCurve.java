package org.ftc9974.thorcore.control.math;

import androidx.annotation.FloatRange;

import org.ftc9974.thorcore.util.MathUtilities;

/**
 * Abstract wrapper for parametric curves of a single parameter in 2D space. Since points are
 * represented as Vector2s, parametric curves are basically vector-valued functions.
 *
 * The curve parameter (t) ranges from 0 to 1, inclusive. Any implementers of ParametricCurve must
 * define get(), derivative(), and secondDerivative() on at least that range.
 *
 * Values returned by this class (and its implementers) have no defined units. However, as long as
 * you use a consistent unit system, it will work with whatever units you want.
 */
public abstract class ParametricCurve {

    /**
     * Class for holding data returned by a closest-point search. The default implementation calculates
     * most of these anyways, and these values will likely be useful for the caller.
     */
    public static class ClosestPoint {
        public final Vector2 point, firstDeriv, secondDeriv, pToPoint;
        public final double t, accurateTo;

        protected ClosestPoint(Vector2 point, Vector2 firstDeriv, Vector2 secondDeriv, Vector2 pToPoint, double t, double accurateTo) {
            this.point = point;
            this.firstDeriv = firstDeriv;
            this.secondDeriv = secondDeriv;
            this.pToPoint = pToPoint;
            this.t = t;
            this.accurateTo = accurateTo;
        }
    }

    /**
     * Gets the position on this curve at a specified value of t.
     *
     * @param t curve parameter
     * @return position
     */
    public abstract Vector2 get(@FloatRange(from = 0, to = 1) double t);

    /**
     * Gets the derivative of {@link #get(double)} with respect to t at a specified value of t.
     *
     * @param t curve parameter
     * @return derivative
     */
    public abstract Vector2 derivative(@FloatRange(from = 0, to = 1) double t);

    /**
     * Gets the second derivative of {@link #get(double)} with respect to t at a specified value of t.
     *
     * @param t curve parameter
     * @return second derivative
     */
    public abstract Vector2 secondDerivative(@FloatRange(from = 0, to = 1) double t);

    /**
     * Finds the point on the curve closest to the specified point.
     *
     * This method should be overridden if a closed-form solution exists for the curve being
     * implemented.
     *
     * @param p point to project onto the curve
     * @return an object containing information about the point on the curve closest to the
     *         specified point
     */
    public ClosestPoint findPointClosestTo(Vector2 p) {
        return findPointClosestTo(p, 0, 1);
    }

    /**
     * Finds the point on the curve closest to the specified point, limiting the range of the curve
     * parameter.
     *
     * This method should be overridden if a closed-form solution exists for the curve being
     * implemented.
     *
     * @param p point to project onto the curve
     * @param start lower bound of the curve parameter
     * @param end upper bound of the curve parameter
     * @return an object containing information about the point on the curve closest to the
     *         specified point
     */
    public ClosestPoint findPointClosestTo(Vector2 p, @FloatRange(from = 0, to = 1) double start, @FloatRange(from = 0, to = 1) double end) {
        return findPointClosestTo(p, start, end, 5, 10);
    }

    /**
     * Finds the point on the curve closest to the specified point.
     *
     * This algorithm performs multiple scans over the curve, with each scan being more
     * range-limited, but more precise.
     *
     * The precision of the algorithm may be calculated as follows.
     * Let s represent the samples parameter, and i represent the iterations parameter.
     * Let c = (2s^(-i)) / s
     *       = 2 / (s * s^i)
     *       = 2 / s^(i + 1)
     * The returned t value (field t of the returned object) will be within c/2 of the t value of
     * the actual closest point. Finding the precision in real space is difficult, however, because
     * change in t is not proportional to distance travelled. Mathematically stated, no constant
     * value of v exists such that ∫|dB/dt|dt = vΔt across the domain t ∈ [0, 1], where B is this
     * curve (expressed as a vector-valued function of t). Regardless, real space precision may be
     * approximated by multiplying |dB/dt| by Δt:
     * p = |dB/dt| Δt = |dB/dt| / (2c)
     * The returned point will be at most, approximately p units from the actual closest point.
     *
     * In general, the precision of the final result scales linearly with samples, and exponentially
     * with iterations. On top of that, time complexity is O(n) with respect to both number of
     * samples *and* number of iterations. this is great, because it means the algorithm can gain
     * lots of precision at the cost of a little time.
     * for example, say the algorithm is taking 5 samples for 10 iterations. by increasing the
     * number of iterations from 10 to 15, the algorithm will be 50% slower, but it will be 3125
     * times as precise.
     *
     * @param p point to project onto the curve
     * @param start lowest t value to consider
     * @param end highest t value to consider
     * @param samples number of samples per scan. this can be fairly low, as each iteration of
     *                sampling is exponentially more precise, which makes up for the low sampling
     *                density.
     * @param iterations number of times to scan. each scan is exponentially more precise than the
     *                   last.
     * @return an object containing information about the point on the curve closest to the
     *         specified point
     */
    public ClosestPoint findPointClosestTo(Vector2 p, @FloatRange(from = 0, to = 1) double start, @FloatRange(from = 0, to = 1) double end, int samples, int iterations) {
        // t stores the closest point that we have found as of yet
        double t = 0;
        // margin is the range plus or minus t that we will search for a new closest point
        double margin = 1;
        final double initialStart = start, initialEnd = end;
        for (int i = 1; i <= iterations; i++) {
            // scan the range between start and end, sampling a given number of points within the
            // range.
            t = minimizeDistOnRange(p, start, end, samples);
            if (t == initialStart) {
                // p lies beyond the start of the curve, so the closest point is the start
                double dot = derivative(initialStart).dot(p.subtract(get(initialStart)));
                if (dot < 0) break;
            }
            if (t == initialEnd) {
                // p lies beyond the end of the curve, so the closest point is the end
                double dot = derivative(initialEnd).dot(p.subtract(get(initialEnd)));
                if (dot > 0) break;
            }
            // decrease the margin so the next scanned area is the 2 adjacent areas that were scanned
            // in the previous iteration. each successive iteration needs to look at 1/samples of the
            // range of the previous iteration. we can write a closed-form solution of it by taking
            // samples to the negative i-th power, where i is the current iteration number.
            margin = Math.pow(samples, -i);
            // calculate a new range, created by placing a margin around the (current) closest point.
            start = MathUtilities.constrain(t - margin, initialStart, initialEnd);
            end = MathUtilities.constrain(t + margin, initialStart, initialEnd);
        }
        Vector2 curvePoint = get(t);
        Vector2 firstDeriv = derivative(t);
        double accurateTo = 0;
        // if start == end, there's only one point, with perfect accuracy.
        if (t != start && t != end) {
            // calculate the distance to which this point is accurate to
            accurateTo = (margin / samples) * firstDeriv.getMagnitude();
        }
        return new ClosestPoint(curvePoint, firstDeriv, secondDerivative(t), curvePoint.subtract(p), t, accurateTo);
    }

    /**
     * Looks for the closest point on this curve to the specified point on the specified range.
     *
     * @param p a point in space, to which this method finds the closest point on this curve
     * @param start the minimum t-value (curve parameter) to be considered
     * @param end the maximum t-value (curve parameter) to be considered
     * @param samples the number of points to check on the range
     * @return t-value of the closest point
     */
    protected double minimizeDistOnRange(Vector2 p, double start, double end, int samples) {
        // if the range is degenerate, there's only one point that is valid.
        if (start == end) return start;

        // minDistanceSq and closestT store the as-of-yet closest distance and t-value.
        double minDistanceSq = Double.POSITIVE_INFINITY;
        double closestT = 0;
        // iterate through the range of t-values from start to end
        for (double t = start, dt = (end - start) / (double) samples; t < end + dt; t += dt) {
            // currently considered t-value
            double consT = MathUtilities.constrain(t, start, end);
            // calculate the squared distance. since squaring is a one-to-one monotonic increasing
            // function, we don't have to take the square root to compare distance. on embedded systems,
            // taking the square root can be a tad expensive, so we avoid it for the sake of performance.
            double distSq = p.subtract(get(consT)).getMagnitudeSq();
            // if this distance is less than the closest distance we've seen yet, we've found a new
            // closest point. record this new distance and t-value.
            if (distSq < minDistanceSq) {
                minDistanceSq = distSq;
                closestT = consT;
            }
        }

        return closestT;
    }

    /**
     * Gets the length of this curve.
     *
     * @return length, measured in the same units as the value returned by get().
     */
    public double length() {
        return segmentLength(0, 1);
    }

    /**
     * Gets the length of a segment of this curve.
     *
     * Note that the default implementation can be expensive. Subclasses may implement closed-form
     * solutions with significantly better performance, but the default implementation isn't very fast.
     *
     * @param start parameter at the start of the segment
     * @param end parameter at the end of the segment
     * @return length of the segment, measured in the same units as the value returned by get().
     */
    public double segmentLength(@FloatRange(from = 0, to = 1) double start, @FloatRange(from = 0, to = 1) double end) {
        // naive first-order implementation
        // todo better implementation that uses curvature circles instead of lines
        double length = 0;
        for (double t = start, dt = (end - start) / 100; t < end; t += dt) {
            length += dt * derivative(t).getMagnitude();
        }
        return length;
    }
}
