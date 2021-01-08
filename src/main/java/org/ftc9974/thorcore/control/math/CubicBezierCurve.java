package org.ftc9974.thorcore.control.math;

import org.ftc9974.thorcore.util.MathUtilities;

/**
 * Implements Cubic Bézier Curves in 2D space.
 */
public class CubicBezierCurve extends ParametricCurve {

    private Vector2 p0, p1, p2, p3;

    // cached values of vector terms. these only change when the control points change, so they
    // don't need to be recalculated in every call to get(), derivative(), and secondDerivative().
    // they're also premultiplied by the coefficients of their term, so it's a few less
    // multiplications per call.
    // this may seem like excessive optimization, but every millisecond counts when it comes to
    // control loops.
    private Vector2 p10, p21, p32;
    private Vector2 p210, p321;

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

        // premultiply
        p10 = p10.scalarMultiply(3);
        p21 = p21.scalarMultiply(6);
        p32 = p32.scalarMultiply(3);
        p210 = p210.scalarMultiply(6);
        p321 = p321.scalarMultiply(6);
    }

    @Override
    public Vector2 get(double t) {
        // time-saving special cases
        if (t == 0) return p0;
        if (t == 1) return p3;
        double a = 1 - t;
        return Vector2.add(p0.scalarMultiply(a * a * a), p1.scalarMultiply(3 * a * a * t), p2.scalarMultiply(3 * a * t * t), p3.scalarMultiply(t * t * t));
    }

    @Override
    public Vector2 derivative(double t) {
        // time-saving special cases
        if (t == 0) return p10;
        if (t == 1) return p32;
        double a = 1 - t;
        return Vector2.add(p10.scalarMultiply(a * a), p21.scalarMultiply(a * t), p32.scalarMultiply(t * t));
    }

    @Override
    public Vector2 secondDerivative(double t) {
        // time-saving special cases
        if (t == 0) return p210;
        if (t == 1) return p321;
        return Vector2.add(p210.scalarMultiply((1 - t)), p321.scalarMultiply(t));
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
