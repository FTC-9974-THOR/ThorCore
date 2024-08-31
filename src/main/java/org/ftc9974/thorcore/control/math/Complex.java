package org.ftc9974.thorcore.control.math;

public class Complex {
    public double real, imag;

    public Complex(double real, double imag) {
        this.real = real;
        this.imag = imag;
    }

    public double magnitude() {
        return Math.hypot(real, imag);
    }

    public double angle() {
        return Math.atan2(imag, real);
    }

    public double arg() {
        return angle();
    }

    public static Complex eulerIdentity(double theta) {
        return new Complex(Math.cos(theta), Math.sin(theta));
    }
}
