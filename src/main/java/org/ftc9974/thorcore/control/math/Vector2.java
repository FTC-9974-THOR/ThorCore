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

    /**
     * Constructs a new Vector2 object.
     *
     * @param x x coordinate
     * @param y y coordinate
     */
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Gets the value of the specified axis.
     *
     * @param axis 0 -> x, 1 -> y
     * @return the value
     */
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

    /**
     * Gets the x coordinate of the vector.
     *
     * @return x coord
     */
    public double getX() {
        return x;
    }

    /**
     * Gets the y coordinate of the vector.
     *
     * @return y coord
     */
    public double getY() {
        return y;
    }

    /**
     * Sets the value of the specified axis.
     *
     * @param axis 0 -> x, 1 -> y
     * @param value value
     */
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

    /**
     * Sets the x coordinate of the vector.
     * @param value value
     */
    public void setX(double value) {
        x = value;
    }

    /**
     * Sets the y coordinate of the vector.
     *
     * @param value value
     */
    public void setY(double value) {
        y = value;
    }

    /**
     * Gets the heading of the vector.
     *
     * @return the heading, in radians. 0 is along the +x axis.
     */
    public double getHeading() {
        return Math.atan2(getY(), getX());
    }

    /**
     * Gets the magnitude of the vector.
     *
     * @return magnitude
     */
    public double getMagnitude() {
        return Math.hypot(getX(), getY());
    }

    /**
     * Gets the square of the magnitude of the vector.
     *
     * @return magnitude squared
     */
    public double getMagnitudeSq() {
        return dot(this, this);
    }

    /**
     * Checks if this vector has the same x and y values as the supplied vector.
     *
     * @param obj object to compare to
     * @return true if obj is a Vector2 with equal x and y values
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Vector2) {
            Vector2 other = (Vector2) obj;
            return other.getX() == getX() && other.getY() == getY();
        }
        return false;
    }

    /**
     * Formats the vector to a human-readable String.
     *
     * @return String representation
     */
    @Override
    public String toString() {
        return String.format(Locale.getDefault(), "(%f, %f)", getX(), getY());
    }

    // IMPORTANT:
    // operations are non-mutating!
    // that is, these methods will not change the vector object
    // that calls them!

    /**
     * Returns the vector sum of this vector and the supplied vector.
     *
     * @param other other vector
     * @return vector sum
     */
    public Vector2 add(Vector2 other) {
        return add(this, other);
    }

    /**
     * Returns the vector difference of this vector and the supplied vector.
     *
     * @param other other vector
     * @return vector difference
     */
    public Vector2 subtract(Vector2 other) {
        return subtract(this, other);
    }

    /**
     * Returns the scalar product of this vector and a scalar.
     *
     * @param scalar scalar
     * @return scalar product
     */
    public Vector2 scalarMultiply(double scalar) {
        return scalarMultiply(this, scalar);
    }

    /**
     * Returns the scalar quotient of this vector and a scalar.
     *
     * @param scalar scalar
     * @return scalar quotient
     */
    public Vector2 scalarDivide(double scalar) {
        return scalarDivide(this, scalar);
    }

    /**
     * Returns a rotated copy of this vector.
     *
     * @param theta angle to rotate by, in radians. Positive angles are counterclockwise.
     * @return rotated copy
     */
    public Vector2 rotate(double theta) {
        return rotate(this, theta);
    }

    /**
     * Returns the vector dot product of this vector and the supplied vector.
     *
     * The dot product is defined as
     * a.x * b.x + a.y * b.y
     * And is equal to
     * |a||b|cos(t)
     * Where |a| is the magnitude of a
     *       |b| is the magnitude of b
     *       t is the angle between a and b
     *
     * @param other other vector
     * @return dot product
     */
    public double dot(Vector2 other) {
        return dot(this, other);
    }

    /**
     * Returns the signed magnitude of the cross product of this vector and the supplied vector,
     * which is equivalent to the determinant of the matrix constructed from the 2 vectors:
     *               [ a.x b.y ]
     * |a X b| = det [ b.x b.y ] = a.x * b.y - a.y * b.x = |a||b|sin(t)
     * Where t is the angle between a and b.
     *
     * The magnitude of the cross product is equal to the area of the parallelogram defined by the
     * 2 vectors.
     *
     * @param other other vector
     * @return cross product magnitude
     */
    public double crossMag(Vector2 other) {
        return crossMag(this, other);
    }

    /**
     * Returns a normalized copy of this vector (that is, a vector with a magnitude of 1).
     *
     * @return normalized copy
     */
    public Vector2 normalized() {
        return normalized(this);
    }

    /**
     * Adds vectors.
     *
     * @param vec1 addend
     * @param vec2 addend
     * @param vecs any additional addends
     * @return sum
     */
    // having 2 explicit arguments keeps the signature different from that of the instance version
    // of add()
    public static Vector2 add(Vector2 vec1, Vector2 vec2, Vector2... vecs) {
        double x = vec1.getX() + vec2.getX(),
               y = vec1.getY() + vec2.getY();
        for (Vector2 vec : vecs) {
            x += vec.getX();
            y += vec.getY();
        }
        return new Vector2(x, y);
    }

    /**
     * Subtracts vectors.
     *
     * @param vec1 minuend
     * @param vecs subtrahends
     * @return difference
     */
    public static Vector2 subtract(Vector2 vec1, Vector2... vecs) {
        double x = vec1.getX(), y = vec1.getY();
        for (Vector2 vec : vecs) {
            x -= vec.getX();
            y -= vec.getY();
        }
        return new Vector2(x, y);
    }

    /**
     * Multiplies a vector and a scalar.
     *
     * @param vec vector
     * @param scalar scalar
     * @return product
     */
    public static Vector2 scalarMultiply(Vector2 vec, double scalar) {
        return new Vector2(vec.getX() * scalar, vec.getY() * scalar);
    }

    /**
     * Divides a vector and a scalar.
     *
     * @param vec vector
     * @param scalar scalar
     * @return quotient
     */
    public static Vector2 scalarDivide(Vector2 vec, double scalar) {
        return new Vector2(vec.getX() / scalar, vec.getY() / scalar);
    }

    /**
     * Rotates a vector.
     *
     * @param vec1 vector
     * @param theta angle to rotate by, in radians. 0 is along the +x axis, increasing counterclockwise.
     * @return rotated vector
     */
    public static Vector2 rotate(Vector2 vec1, double theta) {
        double[] rotated = MathUtilities.rotate2D(new double[] {vec1.getX(), vec1.getY()}, theta);
        return new Vector2(rotated[0], rotated[1]);
    }

    /**
     * Calculates the dot product of 2 vectors.
     *
     * @param vec1 vector
     * @param vec2 vector
     * @return dot product
     */
    public static double dot(Vector2 vec1, Vector2 vec2) {
        return vec1.x * vec2.x + vec1.y * vec2.y;
    }

    /**
     * Calculates the magnitude of the cross product of 2 vectors.
     *
     * @param vec1 vector
     * @param vec2 vector
     * @return cross product magnitude
     */
    public static double crossMag(Vector2 vec1, Vector2 vec2) {
        return vec1.getX() * vec2.getY() - vec1.getY() * vec2.getX();
    }

    /**
     * Normalizes a vector. Normalizing retains the heading, but scales the vector so it has a
     * magnitude of 1.
     *
     * @param vec vector
     * @return normalized vector
     */
    public static Vector2 normalized(Vector2 vec) {
        return scalarDivide(vec, vec.getMagnitude());
    }

    /**
     * Linearly (element-wise) interpolates between 2 vectors.
     *
     * Performs lerp on the x and y coordinates (hence it being called element-wise).
     *
     * @param a "from" vector
     * @param b "to" vector
     * @param t variable of interpolation
     * @return interpolated vector
     */
    public static Vector2 lerp(Vector2 a, Vector2 b, double t) {
        return new Vector2(
                MathUtilities.lerp(a.getX(), b.getX(), t),
                MathUtilities.lerp(a.getY(), b.getY(), t)
        );
    }
}
