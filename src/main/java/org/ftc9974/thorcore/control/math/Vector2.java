package org.ftc9974.thorcore.control.math;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.ftc9974.thorcore.util.MathUtilities;

import java.util.Locale;

/**
 * Utility class for doing 2D vector math.
 * Implements Euclidean vector operations in a regular Cartesian coordinate system
 * (+x -> right, +y -> up, heading of 0 -> right, heading increases counterclockwise)
 */
@SuppressWarnings({"WeakerAccess", "unused"})
public final class Vector2 {

    public static final Vector2 ZERO = new Vector2(0, 0);

    private double x, y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Deprecated
    public double get(int axis) {
        if (axis == 0) {
            return x;
        } else if (axis == 1) {
            return y;
        } else {
            throw new ArrayIndexOutOfBoundsException();
        }
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    @Deprecated
    public void set(int axis, double value) {
        if (axis == 0) {
            x = value;
        } else if (axis == 1) {
            y = value;
        } else {
            throw new ArrayIndexOutOfBoundsException();
        }
    }

    public void setX(double value) {
        x = value;
    }

    public void setY(double value) {
        y = value;
    }

    public double getHeading() {
        return Math.atan2(getY(), getX());
    }

    public double getMagnitude() {
        return Math.hypot(getX(), getY());
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Vector2) {
            Vector2 other = (Vector2) obj;
            return other.getX() == getX() && other.getY() == getY();
        }
        return false;
    }

    @Override
    public String toString() {
        return String.format(Locale.getDefault(), "(%f, %f)", getX(), getY());
    }

    // IMPORTANT:
    // operations are non-mutating!
    // that is, these methods will not change the vector object
    // that calls them!

    public Vector2 add(Vector2 other) {
        return add(this, other);
    }

    public Vector2 subtract(Vector2 other) {
        return subtract(this, other);
    }

    public Vector2 scalarMultiply(double scalar) {
        return scalarMultiply(this, scalar);
    }

    public Vector2 scalarDivide(double scalar) {
        return scalarDivide(this, scalar);
    }

    public Vector2 rotate(double theta) {
        return rotate(this, theta);
    }

    public static Vector2 add(Vector2 vec1, Vector2 vec2) {
        return new Vector2(vec1.getX() + vec2.getX(), vec1.getY() + vec2.getY());
    }

    public static Vector2 subtract(Vector2 vec1, Vector2 vec2) {
        return new Vector2(vec1.getX() - vec2.getX(), vec1.getY() - vec2.getY());
    }

    public static Vector2 scalarMultiply(Vector2 vec, double scalar) {
        return new Vector2(vec.getX() * scalar, vec.getY() * scalar);
    }

    public static Vector2 scalarDivide(Vector2 vec, double scalar) {
        return new Vector2(vec.getX() / scalar, vec.getY() / scalar);
    }

    public static Vector2 rotate(Vector2 vec1, double theta) {
        double[] rotated = MathUtilities.rotate2D(new double[] {vec1.getX(), vec1.getY()}, theta);
        return new Vector2(rotated[0], rotated[1]);
    }

    public static Vector2 lerp(Vector2 a, Vector2 b, double t) {
        return new Vector2(
                MathUtilities.lerp(a.getX(), b.getX(), t),
                MathUtilities.lerp(a.getY(), b.getY(), t)
        );
    }
}
