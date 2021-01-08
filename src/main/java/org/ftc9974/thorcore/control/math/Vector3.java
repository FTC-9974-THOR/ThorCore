package org.ftc9974.thorcore.control.math;

import org.ftc9974.thorcore.util.MathUtilities;

import java.util.Locale;

/**
 * Utility class for doing 3D vector math.
 * Implements Euclidean vector operations in Cartesian coordinates.
 * Additionally, some methods support spherical coordinates.
 */
public final class Vector3 {

    // zero and unit vectors i, j, k
    public static final Vector3 ZERO = new Vector3(0, 0, 0),
                                I = new Vector3(1, 0, 0),
                                J = new Vector3(0, 1, 0),
                                K = new Vector3(0, 0, 1);

    private double x, y, z;

    // these values are a tad expensive to calculate, so they are cached.
    // the cache is lazy-initialized, and is cleared whenever x, y, or z
    // is changed.
    private double magnitude, phi, theta, r;

    /**
     * Constructs a new Vector3 object.
     *
     * @param x x coordinate
     * @param y y coordinate
     * @param z z coordinate
     */
    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;

        magnitude = phi = theta = r = Double.NaN;
    }

    /**
     * Gets the x coordinate of this vector.
     *
     * @return x coord
     */
    public double getX() {
        return x;
    }

    /**
     * Gets the y coordinate of this vector.
     *
     * @return y coord
     */
    public double getY() {
        return y;
    }

    /**
     * Gets the z coordinate of this vector.
     *
     * @return z coord
     */
    public double getZ() {
        return z;
    }

    /**
     * Sets the x coordinate of this vector.
     *
     * @param value new x coord
     */
    public void setX(double value) {
        x = value;
        magnitude = theta = phi = r = Double.NaN;
    }

    /**
     * Sets the y coordinate of this vector.
     *
     * @param value new y coord
     */
    public void setY(double value) {
        y = value;
        magnitude = theta = phi = r = Double.NaN;
    }

    /**
     * Sets the z coordinate of this vector.
     *
     * @param value new z coord
     */
    public void setZ(double value) {
        z = value;
        magnitude = theta = phi = r = Double.NaN;
    }

    /**
     * Gets the r component of this vector projected onto the XY plane. This is equal to the
     * distance from the end of this vector to the closest point on the Z axis.
     *
     * If you were to project this vector onto the XY plane, this would be the magnitude of the
     * projection. This method is called getR() because the projection's magnitude is the r
     * component of said projection in polar coordinates.
     *
     * @return r component (distance from the z axis)
     */
    public double getR() {
        if (Double.isNaN(r)) {
            r = Math.hypot(getX(), getY());
        }
        return r;
    }

    /**
     * Gets the rho component of this vector in spherical coordinates.
     *
     * This method is an overload of {@link #getMagnitude()}. It is provided in case someone prefers
     * to use the formal names of spherical coordinates.
     *
     * @return rho component
     */
    public double getRho() {
        return getMagnitude();
    }

    /**
     * Gets the square of the rho component of this vector in spherical coordinates.
     *
     * This method is an overload of {@link #getMagnitudeSq()}. It is provided in case someone
     * prefers to use the formal names of spherical coordinates.
     *
     * @return rho component squared
     */
    public double getRhoSq() {
        return getMagnitudeSq();
    }

    /**
     * Gets the magnitude of this vector.
     *
     * @return magnitude
     */
    public double getMagnitude() {
        if (Double.isNaN(magnitude)) {
            magnitude = Math.sqrt(x * x + y * y + z * z);
        }
        return magnitude;
    }

    /**
     * Gets the square of the magnitude of this vector.
     *
     * @return magnitude squared
     */
    public double getMagnitudeSq() {
        if (!Double.isNaN(magnitude)) {
            return magnitude * magnitude;
        }
        return dot(this);
    }

    /**
     * Gets the theta component of this vector in spherical coordinates.
     *
     * If you were to project this vector onto the XY plane, theta is the angle between the
     * projection and the +x axis, increasing counterclockwise.
     *
     * @return theta, in radians, within the interval [-pi, pi].
     */
    public double getTheta() {
        if (Double.isNaN(theta)) {
            theta = Math.atan2(getY(), getX());
        }
        return theta;
    }

    /**
     * Gets the phi component of this vector in spherical coordinates.
     *
     * This is basically the angle between this vector and the +z axis.
     *
     * @return phi, in radians, within the interval [0, pi].
     */
    public double getPhi() {
        if (Double.isNaN(phi)) {
            phi = Math.atan2(getR(), getZ());
        }
        return phi;
    }

    /**
     * Checks if this vector has the same component values as the supplied vector.
     *
     * @param obj object to compare to
     * @return true if obj is a Vector3 with equal x, y, and z values
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Vector3) {
            Vector3 other = (Vector3) obj;
            return other.getX() == getX() && other.getY() == getY() && other.getZ() == getZ();
        }
        return false;
    }

    /**
     * Formats this vector to a human-readable String.
     *
     * @return String representation
     */
    @Override
    public String toString() {
        return String.format(Locale.getDefault(), "(%f, %f, %f)", getX(), getY(), getZ());
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
    public Vector3 add(Vector3 other) {
        return add(this, other);
    }

    /**
     * Returns the vector difference of this vector and the supplied vector.
     *
     * @param other other vector
     * @return vector difference
     */
    public Vector3 subtract(Vector3 other) {
        return subtract(this, other);
    }

    /**
     * Returns the scalar product of this vector and a scalar.
     *
     * @param scalar scalar
     * @return scalar product
     */
    public Vector3 scalarMultiply(double scalar) {
        return scalarMultiply(this, scalar);
    }

    /**
     * Returns the scalar quotient of this vector and a scalar.
     *
     * @param scalar scalar
     * @return scalar quotient
     */
    public Vector3 scalarDivide(double scalar) {
        return scalarDivide(this, scalar);
    }

    /**
     * Returns a copy of this vector that has been rotated around the specified axis.
     *
     * This uses Rodrigues' rotation formula.
     * https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
     *
     * The rotation is applied following the right hand rule. If you imagine pointing the thumb of
     * your right hand along the rotation axis, then a positive value of theta will cause a rotation
     * in the same direction that your fingers curl around the rotation axis.
     *
     * @param axis vector to rotate around. it does *not* need to be normalized.
     * @param theta angle to rotate by, in radians
     * @return rotated copy
     */
    public Vector3 rotate(Vector3 axis, double theta) {
        return rotate(this, axis, theta);
    }

    /**
     * Returns a copy of this vector that has been rotated around the x axis.
     *
     * @param theta angle, in radians, to rotate by
     * @return rotated copy
     */
    public Vector3 rotateAroundX(double theta) {
        return rotate(this, I, theta);
    }

    /**
     * Returns a copy of this vector that has been rotated around the y axis.
     *
     * @param theta angle, in radians, to rotate by
     * @return rotated copy
     */
    public Vector3 rotateAroundY(double theta) {
        return rotate(this, J, theta);
    }

    /**
     * Returns a copy of this vector that has been rotated around the z axis.
     *
     * @param theta angle, in radians, to rotate by
     * @return rotated copy
     */
    public Vector3 rotateAroundZ(double theta) {
        return rotate(this, K, theta);
    }

    /**
     * Returns the vector dot product of this vector and the supplied vector.
     *
     * The dot product is defined as
     * a.x * b.x + a.y * b.y + a.z * b.z
     * And is equal to
     * |a||b|cos(t)
     * Where |a| is the magnitude of a
     *       |b| is the magnitude of b
     *       t is the angle between a and b
     *
     * @param other other vector
     * @return dot product
     */
    public double dot(Vector3 other) {
        return dot(this, other);
    }

    /**
     * Returns the cross product of this vector and the supplied vector.
     *
     * @param other other vector
     * @return cross product
     */
    public Vector3 cross(Vector3 other) {
        return cross(this, other);
    }

    /**
     * Returns a normalized copy of this vector - that is, a vector with the same direction but a
     * magnitude of 1.
     *
     * @return normalized copy
     */
    public Vector3 normalized() {
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
    public static Vector3 add(Vector3 vec1, Vector3 vec2, Vector3... vecs) {
        double x = vec1.getX() + vec2.getX(),
               y = vec1.getY() + vec2.getY(),
               z = vec1.getZ() + vec2.getZ();
        for (Vector3 vec : vecs) {
            x += vec.getX();
            y += vec.getY();
            z += vec.getZ();
        }
        return new Vector3(x, y, z);
    }

    /**
     * Subtracts vectors.
     *
     * Every subtrahend is subtracted from the minuend. The returned vector is equal to
     * vec1 - (vecs[0] + vecs[1] + vecs[2] + ... + vecs[n - 1])
     * where n is the length of vecs.
     *
     * @param vec1 minuend
     * @param vecs subtrahends
     * @return difference
     */
    public static Vector3 subtract(Vector3 vec1, Vector3... vecs) {
        double x = vec1.getX(), y = vec1.getY(), z = vec1.getZ();
        for (Vector3 vec : vecs) {
            x -= vec.getX();
            y -= vec.getY();
            z -= vec.getZ();
        }
        return new Vector3(x, y, z);
    }

    /**
     * Multiplies a vector by a scalar.
     *
     * @param vec vector
     * @param scalar scalar
     * @return product
     */
    public static Vector3 scalarMultiply(Vector3 vec, double scalar) {
        return new Vector3(vec.getX() * scalar, vec.getY() * scalar, vec.getZ() * scalar);
    }

    /**
     * Divides a vector by a scalar.
     *
     * @param vec vector
     * @param scalar scalar
     * @return quotient
     */
    public static Vector3 scalarDivide(Vector3 vec, double scalar) {
        return new Vector3(vec.getX() / scalar, vec.getY() / scalar, vec.getZ() / scalar);
    }

    /**
     * Rotates a vector around a specified axis.
     *
     * This uses Rodrigues' rotation formula.
     * https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
     *
     * The rotation is applied following the right hand rule. If you imagine pointing the thumb of
     * your right hand along the rotation axis, then a positive value of theta will cause a rotation
     * in the same direction that your fingers curl around the rotation axis.
     *
     * @param vec vector to rotate
     * @param axis axis around which to rotate vec. this does *not* need to be normalized.
     * @param theta angle to rotate by, in radians
     * @return rotated vector
     */
    public static Vector3 rotate(Vector3 vec, Vector3 axis, double theta) {
        Vector3 normalizedAxis;
        if (axis.getMagnitude() != 1) {
            normalizedAxis = axis.normalized();
        } else {
            normalizedAxis = axis;
        }
        return Vector3.add(
                vec.scalarMultiply(Math.cos(theta)),
                normalizedAxis.cross(vec).scalarMultiply(Math.sin(theta)),
                normalizedAxis.scalarMultiply(normalizedAxis.dot(vec) * (1 - Math.cos(theta)))
        );
    }

    /**
     * Calculates the dot product of 2 vectors.
     *
     * @param vec1 vector
     * @param vec2 vector
     * @return dot product
     */
    public static double dot(Vector3 vec1, Vector3 vec2) {
        return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
    }

    /**
     * Calculates the cross product of 2 vectors.
     *
     * @param vec1 left-hand argument
     * @param vec2 right-hand argument
     * @return cross product
     */
    public static Vector3 cross(Vector3 vec1, Vector3 vec2) {
        return new Vector3(
                vec1.y * vec2.z - vec1.z * vec2.y,
                -(vec1.x * vec2.z - vec1.z * vec2.x),
                vec1.x * vec2.y - vec1.y * vec2.x
        );
    }

    /**
     * Normalizes a vector. Normalizing retains the heading, but scales the vector so it has a
     * magnitude of 1.
     *
     * @param vec vector
     * @return normalized vector
     */
    public static Vector3 normalized(Vector3 vec) {
        return scalarDivide(vec, vec.getMagnitude());
    }

    /**
     * Linearly (element-wise) interpolates between 2 vectors. The end of the vector returned by
     * this method will always lie on the line between said 2 vectors.
     *
     * Performs lerp on the x, y, and z coordinates (hence it being called element-wise).
     *
     * @param a "from" vector
     * @param b "to" vector
     * @param t variable of interpolation
     * @return interpolated vector
     */
    public static Vector3 lerp(Vector3 a, Vector3 b, double t) {
        return new Vector3(
                MathUtilities.lerp(a.getX(), b.getX(), t),
                MathUtilities.lerp(a.getY(), b.getY(), t),
                MathUtilities.lerp(a.getZ(), b.getZ(), t)
        );
    }
}
