package org.ftc9974.thorcore.control.math;

import org.ftc9974.thorcore.util.MathUtilities;

/**
 * Implements Cubic Bézier Curves in 2D space.
 *
 * Bézier curves (named after Pierre Bézier)
 */
public class CubicBezierCurve {

    public static class ClosestPoint {
        // all of these vectors were calculated anyways, so pass them along in case they come in
        // handy
        public final Vector2 point, firstDeriv, secondDeriv, pToPoint;
        public final double t, accurateTo;

        private ClosestPoint(Vector2 point, Vector2 firstDeriv, Vector2 secondDeriv, Vector2 pToPoint, double t, double accurateTo) {
            this.point = point;
            this.firstDeriv = firstDeriv;
            this.secondDeriv = secondDeriv;
            this.pToPoint = pToPoint;
            this.t = t;
            this.accurateTo = accurateTo;
        }
    }

    private Vector2 p0, p1, p2, p3;

    // cached values of vector terms. these only change when the control points change, so they
    // don't need to be recalculated in every call to get(), derivative(), and secondDerivative().
    // they're also premultiplied by the coefficients of their term, so it's a few less
    // multiplications per call.
    // this may seem like excessive optimization, but every millisecond counts when it comes to
    // control loops.
    private Vector2 p10, p21, p32;
    private Vector2 p210, p321;
    private boolean hasLoop;

    public CubicBezierCurve(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        cacheConstants();
    }

    private void cacheConstants() {
        p10 = p1.subtract(p0);
        p21 = p2.subtract(p1);
        p32 = p3.subtract(p2);

        p210 = p2.subtract(p1.scalarMultiply(2)).add(p0);
        p321 = p3.subtract(p2.scalarMultiply(2)).add(p1);

        // calculate which side of p10 p2 and p3 lie on
        double crossP2 = Math.signum(Vector2.crossMag(p10, p2.subtract(p0)));
        double crossP3 = Math.signum(Vector2.crossMag(p10, p3.subtract(p0)));
        // calculate which side of p32 p0 and p1 lie on
        double crossP0 = Math.signum(Vector2.crossMag(p32, p0.subtract(p2)));
        double crossP1 = Math.signum(Vector2.crossMag(p32, p1.subtract(p2)));

        // if p2 and p3 lie on different sides of p10, the line segment between p2 and p3 intersects
        // the line through p0 and p1.
        // if p0 and p1 lie on different sides of p32, the line segment between p0 and p1 intersects
        // the line through p2 and p3.
        // alternatively, a point may lie ON a line, in which case it's corresponding cross product
        // will be zero.
        // if both intersection conditions are met, then the line segment through p0 and p1
        // intersects the line segment through p2 and p3.
        hasLoop = (crossP2 != crossP3 || Math.abs(crossP2) == 0 || Math.abs(crossP3) == 0) &&
                  (crossP0 != crossP1 || Math.abs(crossP0) == 0 || Math.abs(crossP1) == 0);

        // premultiply
        p10 = p10.scalarMultiply(3);
        p21 = p21.scalarMultiply(6);
        p32 = p32.scalarMultiply(3);
        p210 = p210.scalarMultiply(6);
        p321 = p321.scalarMultiply(6);
    }

    public Vector2 get(double t) {
        // time-saving special cases
        if (t == 0) return p0;
        if (t == 1) return p3;
        double a = 1 - t;
        return Vector2.add(p0.scalarMultiply(a * a * a), p1.scalarMultiply(3 * a * a * t), p2.scalarMultiply(3 * a * t * t), p3.scalarMultiply(t * t * t));
    }

    public Vector2 derivative(double t) {
        // time-saving special cases
        if (t == 0) return p10;
        if (t == 1) return p32;
        double a = 1 - t;
        return Vector2.add(p10.scalarMultiply(a * a), p21.scalarMultiply(a * t), p32.scalarMultiply(t * t));
    }

    public Vector2 secondDerivative(double t) {
        // time-saving special cases
        if (t == 0) return p210;
        if (t == 1) return p321;
        return Vector2.add(p210.scalarMultiply((1 - t)), p321.scalarMultiply(t));
    }

    public boolean hasLoop() {
        return hasLoop;
    }

    public ClosestPoint findPointClosestTo(Vector2 p) {
        return findPointClosestTo(p, 0, 1);
    }

    public ClosestPoint findPointClosestTo(Vector2 p, double start, double end) {
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
    public ClosestPoint findPointClosestTo(Vector2 p, double start, double end, int samples, int iterations) {
        double t = 0;
        double margin = 1;
        final double initialStart = start, initialEnd = end;
        for (int i = 1; i <= iterations; i++) {
            t = minimizeDistOnRange(p, start, end, samples);
            if (t == initialStart) {
                double dot = derivative(initialStart).dot(p.subtract(get(initialStart)));
                if (dot < 0) break;
            }
            if (t == initialEnd) {
                double dot = derivative(initialEnd).dot(p.subtract(get(initialEnd)));
                if (dot > 0) break;
            }
            margin = Math.pow(samples, -i);
            start = MathUtilities.constrain(t - margin, initialStart, initialEnd);
            end = MathUtilities.constrain(t + margin, initialStart, initialEnd);
        }
        Vector2 curvePoint = get(t);
        Vector2 firstDeriv = derivative(t);
        double accurateTo = 0;
        if (t != start && t != end) {
            accurateTo = (margin / samples) * firstDeriv.getMagnitude();
        }
        return new ClosestPoint(curvePoint, firstDeriv, secondDerivative(t), curvePoint.subtract(p), t, accurateTo);
    }

    private double minimizeDistOnRange(Vector2 p, double start, double end, int samples) {
        if (start == end) return start;
        double minDistanceSq = Double.POSITIVE_INFINITY;
        double closestT = 0;
        for (double t = start, dt = (end - start) / (double) samples; t < end + dt; t += dt) {
            double consT = MathUtilities.constrain(t, start, end);
            double distSq = p.subtract(get(consT)).getMagnitudeSq();
            if (distSq < minDistanceSq) {
                minDistanceSq = distSq;
                closestT = consT;
            }
        }
        return closestT;
    }

    public double length() {
        return segmentLength(0, 1);
    }

    public double segmentLength(double start, double end) {
        // naive first-order implementation
        double length = 0;
        for (double t = start, dt = (end - start) / 100; t < end; t += dt) {
            length += dt * derivative(t).getMagnitude();
        }
        return length;
    }

    public Vector2 getP0() {
        return p0;
    }

    public void setP0(Vector2 p0) {
        this.p0 = p0;
        cacheConstants();
    }

    public Vector2 getP1() {
        return p1;
    }

    public void setP1(Vector2 p1) {
        this.p1 = p1;
        cacheConstants();
    }

    public Vector2 getP2() {
        return p2;
    }

    public void setP2(Vector2 p2) {
        this.p2 = p2;
        cacheConstants();
    }

    public Vector2 getP3() {
        return p3;
    }

    public void setP3(Vector2 p3) {
        this.p3 = p3;
        cacheConstants();
    }
}
