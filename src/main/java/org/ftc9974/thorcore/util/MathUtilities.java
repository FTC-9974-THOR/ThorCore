package org.ftc9974.thorcore.util;

import android.annotation.TargetApi;
import android.os.Build;
import androidx.annotation.NonNull;

import org.ftc9974.thorcore.control.math.Vector2;

import java.util.Arrays;
import java.util.function.BiFunction;
import java.util.function.BinaryOperator;
import java.util.function.Function;

/**
 * Various math functions and utilities.
 */
public final class MathUtilities {

    private static final String TAG = "org.ftc9974.thorcore.utils.MathUtilites";

    private MathUtilities() {}

    public static byte min(byte... bytes) {
        byte min = Byte.MAX_VALUE;
        for (byte b : bytes) {
            if (b < min) {
                min = b;
            }
        }
        return min;
    }

    public static char min(char... chars) {
        // I actually can't think of a time this would be useful, but you never know
        char min = 65535;
        for (char c : chars) {
            if (c < min) {
                min = c;
            }
        }
        return min;
    }

    public static short min(short... shorts) {
        short min = Short.MAX_VALUE;
        for (short s : shorts) {
            if (s < min) {
                min = s;
            }
        }
        return min;
    }

    public static int min(int... ints) {
        int min = Integer.MAX_VALUE;
        for (int i : ints) {
            if (i < min) {
                min = i;
            }
        }
        return min;
    }

    public static long min(long... longs) {
        long min = Long.MAX_VALUE;
        for (long l : longs) {
            if (l < min) {
                min = l;
            }
        }
        return min;
    }

    public static float min(float... floats) {
        float min = Float.MAX_VALUE;
        for (float f : floats) {
            if (f < min) {
                min = f;
            }
        }
        return min;
    }

    public static double min(double... doubles) {
        double min = Double.MAX_VALUE;
        for (double d : doubles) {
            if (d < min) {
                min = d;
            }
        }
        return min;
    }

    public static byte max(byte... bytes) {
        byte max = Byte.MIN_VALUE;
        for (byte b : bytes) {
            if (b > max) {
                max = b;
            }
        }
        return max;
    }

    public static char max(char... chars) {
        // I actually can't think of a time this would be useful, but you never know
        char max = 0;
        for (char c : chars) {
            if (c > max) {
                max = c;
            }
        }
        return max;
    }

    public static short max(short... shorts) {
        short max = Short.MIN_VALUE;
        for (short s : shorts) {
            if (s > max) {
                max = s;
            }
        }
        return max;
    }

    public static int max(int... ints) {
        int max = Integer.MIN_VALUE;
        for (int i : ints) {
            if (i > max) {
                max = i;
            }
        }
        return max;
    }

    public static long max(long... longs) {
        long max = Long.MIN_VALUE;
        for (long l : longs) {
            if (l > max) {
                max = l;
            }
        }
        return max;
    }

    public static float max(float... floats) {
        float max = -Float.MAX_VALUE;
        for (float f : floats) {
            if (f > max) {
                max = f;
            }
        }
        return max;
    }

    public static double max(double... doubles) {
        double max = -Double.MAX_VALUE;
        for (double d : doubles) {
            if (d > max) {
                max = d;
            }
        }
        return max;
    }

    public static double sum(double... doubles) {
        double sum = 0;
        for (double aDouble : doubles) {
            sum += aDouble;
        }
        return sum;
    }

    public static int absMin(int... ints) {
        int ret = Integer.MAX_VALUE;
        for (int possibleMin : ints) {
            int absPossibleMin = Math.abs(possibleMin);
            if (absPossibleMin < ret) {
                ret = absPossibleMin;
            }
        }
        return ret;
    }

    public static double absMin(double... doubles) {
        double ret = Double.POSITIVE_INFINITY;
        for (double possibleMin : doubles) {
            double absPossibleMin = Math.abs(possibleMin);
            if (absPossibleMin < ret) {
                ret = absPossibleMin;
            }
        }
        return ret;
    }

    public static int absMax(int... ints) {
        int ret = -1;
        for (int possibleMax : ints) {
            int absPossibleMax = Math.abs(possibleMax);
            if (absPossibleMax > ret) {
                ret = absPossibleMax;
            }
        }
        return ret;
    }

    public static double absMax(double... doubles) {
        double ret = -1;
        for (double possibleMax : doubles) {
            double absPossibleMax = Math.abs(possibleMax);
            if (absPossibleMax > ret) {
                ret = absPossibleMax;
            }
        }
        return ret;
    }

    /**
     * Approximates the definite integral of a function. Works best with doubles.
     * It may be a rather expensive operation, so use it sparingly.
     * @param function function to integrate
     * @param lowBound low bound of the integral
     * @param highBound high bound of the integral
     * @param step number to increment argument by
     * @param <T> type of function's argument
     * @param <R> type of function's return
     * @return integral of function
     */
    @TargetApi(Build.VERSION_CODES.N)
    public static <T extends Comparable<T>, R extends Comparable<R>> R integrate(@NonNull Function<T, R> function,
                                                                                 @NonNull T lowBound,
                                                                                 @NonNull T highBound,
                                                                                 @NonNull T step,
                                                                                 @NonNull BinaryOperator<T> tAdder,
                                                                                 @NonNull BinaryOperator<R> rAdder,
                                                                                 @NonNull BiFunction<T, R, R> multiplier,
                                                                                 @NonNull R sumInitialiser) {

        R sum = sumInitialiser;
        for (T arg = lowBound; arg.compareTo(highBound) < 0; arg = tAdder.apply(arg, step)) {
            sum = rAdder.apply(sum, multiplier.apply(step, function.apply(arg)));
        }
        return sum;
    }

    public static double map(double x, double inputMin, double inputMax, double outputMin, double outputMax) {
        return ((outputMax - outputMin) / (inputMax - inputMin)) * (x - inputMin) + outputMin;
    }

    public static double lerp(double from, double to, double t) {
        return (1 - t) * from + t * to;
    }

    public static Vector2 lerp(Vector2 from, Vector2 to, double t) {
        return new Vector2(lerp(from.getX(), to.getX(), t), lerp(from.getY(), to.getY(), t));
    }

    @Deprecated
    public static double[] rotate2D(double[] vector, double theta) {
        // x = x cos t - y sin t
        // y = x sin t + y cos t
        return new double[] {vector[0] * Math.cos(theta) - vector[1] * Math.sin(theta), vector[0] * Math.sin(theta) + vector[1] * Math.cos(theta)};
    }

    public static double applyDeadband(double x, double deadband) {
        return applyDeadband(x, deadband, 0);
    }

    public static double applyDeadband(double x, double deadband, double centeredAround) {
        if (Math.abs(x - centeredAround) <= deadband) {
            return centeredAround;
        } else {
            return x;
        }
    }

    public static double average(int... values) {
        int sum = 0;
        for (int value : values) {
            sum += value;
        }
        return sum / (double) values.length;
    }

    public static double average(double... values) {
        double sum = 0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.length;
    }

    /**
     * Converts rgb to hsv.
     * Arguments must be in the range [0, 1].
     * Saturation and Value are in [0, 1], and Hue is in radians.
     * @param r r value
     * @param g g value
     * @param b b value
     * @return {h, s, v}
     */
    public static double[] RGBtoHSV(double r, double g, double b) {
        double minimum = min(r, g, b);
        double maximum = max(r, g, b);

        double hue;
        if (minimum == maximum) {
            hue = 0;
        } else if (maximum == r) {
            hue = 60 * ((g - b) / (maximum - minimum));
        } else if (maximum == g) {
            hue = 60 * (2 + ((b - r) / (maximum - minimum)));
        } else {
            hue = 60 * (4 + ((r - g) / (maximum - minimum)));
        }

        if (hue < 0) {
            hue += 360;
        }

        double saturation;
        if (maximum == 0) {
            saturation = 0;
        } else {
            saturation = (maximum - minimum) / maximum;
        }

        return new double[] {Math.toRadians(hue), saturation, maximum};
    }

    public static double[] RGBtoHSL(double r, double g, double b) {
        double minimum = min(r, g, b);
        double maximum = max(r, g, b);

        double hue;
        if (minimum == maximum) {
            hue = 0;
        } else if (maximum == r) {
            hue = 60 * ((g - b) / (maximum - minimum));
        } else if (maximum == g) {
            hue = 60 * (2 + ((b - r) / (maximum - minimum)));
        } else {
            hue = 60 * (4 + ((r - g) / (maximum - minimum)));
        }

        if (hue < 0) {
            hue += 360;
        }

        double saturation;
        if (maximum == 0 || minimum == 1) {
            saturation = 0;
        } else {
            saturation = (maximum - minimum) / (1 - Math.abs(maximum + minimum - 1));
        }

        double lightness = average(maximum, minimum);

        return new double[] {Math.toRadians(hue), saturation, lightness};
    }

    public static boolean withinRange(double x, double lowInclusive, double highInclusive) {
        return x >= lowInclusive && x <= highInclusive;
    }

    public static double inchesToMM(double inches) {
        return inches * 25.4;
    }

    public static double mmToInches(double mm) {
        return mm / 25.4;
    }

    /**
     * *sergal noises*
     *
     * Multi-dimensional scalar interpolation
     *
     * (weight_0 * value_0 + weight_1 + value_1 + ... + weight_n * value_n) / (weight_0 + ... + weight_n)
     * @param values an array of values
     * @param weights an array of weights corresponding to each value, in the range [0, 1].
     * @return the interpolated number
     */
    public static double merp(double[] values, double[] weights) {
        if (values.length != weights.length) {
            throw new IllegalArgumentException("Inequal number of values and weights");
        }

        double numerator = 0, denominator = 0;
        for (int i = 0; i < values.length; i++) {
            numerator += weights[i] * values[i];
            denominator += weights[i];
        }

        return numerator / denominator;
    }

    public static int constrain(int x, int low, int high) {
        if (x < low) {
            return low;
        } else if (x > high) {
            return high;
        }
        return x;
    }

    public static double constrain(double x, double low, double high) {
        if (x < low) {
            return low;
        } else if (x > high) {
            return high;
        }
        return x;
    }

    /**
     * Converts from frame coordinates to Cartesian coordinates.
     *
     * Frame space is relative to the robot's frame. The origin is at the robot's center of
     * rotation, the +y axis points towards the robot's front, and the +x axis points towards the
     * right side of the robot. Headings are measured relative to the +y axis, increasing
     * counterclockwise. This is the coordinate system that much of ThorCore uses.
     *
     * Cartesian space is the coordinate system most of us are familiar with: the standard XY plane.
     * In Cartesian coordinates, the origin is at the robot's center of rotation, the +x axis points
     * towards the front of the robot, and the +y axis points towards the left side of the robot.
     * Headings are measured relative to the +x axis, increasing counterclockwise.
     *
     * Headings in frame space are the same as the corresponding heading in Cartesian space, as both
     * are measured relative to the same direction.
     *
     * The conversion equation can be expressed as a rotation matrix:
     * [x']   [ 0 1][x]   [ y]
     * [y'] = [-1 0][y] = [-x]
     * where (x, y) is a point in frame space and (x', y') is said point converted to Cartesian
     * space.
     *
     * @param point point in frame coordinates
     * @return the same point expressed in Cartesian space
     */
    public static Vector2 frameToCartesian(Vector2 point) {
        return new Vector2(point.getY(), -point.getX());
    }

    // inverse operation of frameToCartesian()
    public static Vector2 cartesianToFrame(Vector2 point) {
        return new Vector2(-point.getY(), point.getX());
    }

    // ============================================================================================
    // Note: fresnelIntegral(), evaluatePolynomial(), and evaluatePolynomial() are Java ports of C
    // functions from the Cephes Mathematical Library. Cephes was written by and is a copyright of
    // Stephen L. Moshier. According to the readme of Cephes, "[Cephes] may be used freely but it
    // comes with no support or guarantee."
    //
    // Cephes may be found on NIST's Guide to Available Mathematical Software here:
    // https://gams.nist.gov/cgi-bin/serve.cgi/Package/CEPHES
    // Or on Stephen Moshier's website:
    // http://www.moshier.net/#Cephes
    // ============================================================================================

    // These constants store the coefficients of polynomials used in fresnelIntegral().
    private static final double[] FRESNEL_SN = {
            -2.99181919401019853726E3,
            7.08840045257738576863E5,
            -6.29741486205862506537E7,
            2.54890880573376359104E9,
            -4.42979518059697779103E10,
            3.18016297876567817986E11,
    };

    private static final double[] FRESNEL_SD = {
            /* 1.00000000000000000000E0,*/
            2.81376268889994315696E2,
            4.55847810806532581675E4,
            5.17343888770096400730E6,
            4.19320245898111231129E8,
            2.24411795645340920940E10,
            6.07366389490084639049E11,
    };

    private static final double[] FRESNEL_CN = {
            -4.98843114573573548651E-8,
            9.50428062829859605134E-6,
            -6.45191435683965050962E-4,
            1.88843319396703850064E-2,
            -2.05525900955013891793E-1,
            9.99999999999999998822E-1,
    };

    private static final double[] FRESNEL_CD = {
            3.99982968972495980367E-12,
            9.15439215774657478799E-10,
            1.25001862479598821474E-7,
            1.22262789024179030997E-5,
            8.68029542941784300606E-4,
            4.12142090722199792936E-2,
            1.00000000000000000118E0,
    };

    private static final double[] FRESNEL_FN = {
            4.21543555043677546506E-1,
            1.43407919780758885261E-1,
            1.15220955073585758835E-2,
            3.45017939782574027900E-4,
            4.63613749287867322088E-6,
            3.05568983790257605827E-8,
            1.02304514164907233465E-10,
            1.72010743268161828879E-13,
            1.34283276233062758925E-16,
            3.76329711269987889006E-20,
    };
    private static final double[] FRESNEL_FD = {
            /*  1.00000000000000000000E0,*/
            7.51586398353378947175E-1,
            1.16888925859191382142E-1,
            6.44051526508858611005E-3,
            1.55934409164153020873E-4,
            1.84627567348930545870E-6,
            1.12699224763999035261E-8,
            3.60140029589371370404E-11,
            5.88754533621578410010E-14,
            4.52001434074129701496E-17,
            1.25443237090011264384E-20,
    };

    private static final double[] FRESNEL_GN = {
            5.04442073643383265887E-1,
            1.97102833525523411709E-1,
            1.87648584092575249293E-2,
            6.84079380915393090172E-4,
            1.15138826111884280931E-5,
            9.82852443688422223854E-8,
            4.45344415861750144738E-10,
            1.08268041139020870318E-12,
            1.37555460633261799868E-15,
            8.36354435630677421531E-19,
            1.86958710162783235106E-22,
    };

    private static final double[] FRESNEL_GD = {
            /*  1.00000000000000000000E0,*/
            1.47495759925128324529E0,
            3.37748989120019970451E-1,
            2.53603741420338795122E-2,
            8.14679107184306179049E-4,
            1.27545075667729118702E-5,
            1.04314589657571990585E-7,
            4.60680728146520428211E-10,
            1.10273215066240270757E-12,
            1.38796531259578871258E-15,
            8.39158816283118707363E-19,
            1.86958710162783236342E-22,
    };

    // is it redundant to make this a constant? possibly.
    // it is, however, a useful value when working with angles in radians,
    // so I'll keep it in and make it public.
    public static final double PIO2 = Math.PI / 2.0;

    /**
     * Evaluates the normalized Fresnel sine and cosine integrals.
     *
     * The Fresnel integrals are defined as:
     *
     *        x                        x
     * S(x) = ∫ sin(t^2) dt     C(x) = ∫ cos(t^2) dt
     *        0                        0
     *
     * This method evaluates *normalized* Fresnel integrals. Normalized Fresnel integrals are
     * defined as:
     *
     *        x                              x
     * S(x) = ∫ sin((π/2) t^2) dt     C(x) = ∫ cos((π/2) t^2) dt
     *        0                              0
     *
     * For more information on Fresnel integrals, see https://en.wikipedia.org/wiki/Fresnel_integral.
     *
     * This implementation comes from the Cephes Mathematical Library written by Stephen Moshier.
     * Specifically, this is the IEEE 754 double precision version found in cephes/misc/fresnl.c,
     * as the function fresnl().
     *
     * Cephes can be found on NIST's Guide to Available Mathematical Software here:
     * https://gams.nist.gov/cgi-bin/serve.cgi/Package/CEPHES
     *
     * All credit goes to Stephen Moshier.
     *
     * Implementation Notes:
     * Java stores numbers in big endian format, and (since Java 1.2) allows the JVM to use
     * additional precision as allowed by the hardware it's running on. Thus, I'm using the MIEEE
     * flag in Cephes to recognize Java's big endianness and the strictfp keyword to force Java to
     * use IEEE 754 floating point math, just to make sure math behaves the way Cephes expects it to.
     *
     * @param xxa argument to the sine and cosine Fresnel integrals
     * @return a 2-element array storing the result. the first element is the result of the cosine
     *         integral C(x), and the second element is the result of the sine integral S(x).
     */
    /*
    Cephes Math Library Release 2.8:  June, 2000
    Copyright 1984, 1987, 1989, 2000 by Stephen L. Moshier
    */
    public static double[] fresnelIntegral(double xxa) {
        // original C source:
        // int fresnl( xxa, ssa, cca )
        // double xxa, *ssa, *cca;
        // {
        //     double f, g, cc, ss, c, s, t, u;
        //     double x, x2;
        //
        //     x = fabs(xxa);
        //     x2 = x * x;
        //     if( x2 < 2.5625 ) {
        //         t = x2 * x2;
        //         ss = x * x2 * polevl( t, sn, 5)/p1evl( t, sd, 6 );
        //         cc = x * polevl( t, cn, 5)/polevl(t, cd, 6 );
        //         goto done;
        //     }
        //
        //     if( x > 36974.0 ) {
        //         cc = 0.5;
        //         ss = 0.5;
        //         goto done;
        //     }
        //
        //     /*		Asymptotic power series auxiliary functions
        //      *		for large argument
        //      */
        //     x2 = x * x;
        //     t = PI * x2;
        //     u = 1.0/(t * t);
        //     t = 1.0/t;
        //     f = 1.0 - u * polevl( u, fn, 9)/p1evl(u, fd, 10);
        //     g = t * polevl( u, gn, 10)/p1evl(u, gd, 11);
        //
        //     t = PIO2 * x2;
        //     c = cos(t);
        //     s = sin(t);
        //     t = PI * x;
        //     cc = 0.5  +  (f * s  -  g * c)/t;
        //     ss = 0.5  -  (f * c  +  g * s)/t;
        //
        //     done:
        //     if( xxa < 0.0 ) {
        //         cc = -cc;
        //         ss = -ss;
        //     }
        //
        //     *cca = cc;
        //     *ssa = ss;
        //
        //     return(0);
        // }

        double f, g, cc, ss, c, s, t, u;
        double x, x2;

        x = Math.abs(xxa);
        x2 = x * x;
        if (x2 < 2.5625) {
            t = x2 * x2;
            ss = x * x2 * evaluatePolynomial(t, FRESNEL_SN, 5) / evaluatePolynomialWithLC1(t, FRESNEL_SD, 6);
            cc = x * evaluatePolynomial(t, FRESNEL_CN, 5) / evaluatePolynomial(t, FRESNEL_CD, 6);
        } else if (x > 36974.0) {
            cc = 0.5;
            ss = 0.5;
        } else {
            // this appears to be a redundant assignment to x2?
            // maybe i'm not reading the control flow right
            x2 = x * x;
            t = Math.PI * x2;
            u = 1.0 / (t * t);
            t = 1.0 / t;
            f = 1.0 - u * evaluatePolynomial(u, FRESNEL_FN, 9) / evaluatePolynomialWithLC1(u, FRESNEL_FD, 10);
            g = t * evaluatePolynomial(u, FRESNEL_GN, 10) / evaluatePolynomialWithLC1(u, FRESNEL_GD, 11);

            t = PIO2 * x2;
            c = Math.cos(t);
            s = Math.sin(t);
            t = Math.PI * x;
            cc = 0.5 + (f * s - g * c) / t;
            ss = 0.5 - (f * x + g * s) / t;
        }
        if (xxa < 0.0) {
            cc = -cc;
            ss = -ss;
        }
        return new double[] {cc, ss};
    }

    /**
     * Evaluates a polynomial of arbitrary degree at the given value of x.
     *
     * Coefficients are stored in reverse order. The first element of coefficients is the coefficient
     * of the highest-degree term of the polynomial, and the last element is the coefficient of the
     * lowest-degree term - that degree being zero.
     *
     * For example, a 3rd degree polynomial would be evaluated as:
     * y = coefficients[3] + coefficients[2] * x + coefficients[1] * x^2 + coefficients[0] * x^3
     *
     * The length of coefficients must be 1 greater than the degree of the polynomial.
     *
     * This implementation comes from the Cephes Mathematical Library written by Stephen Moshier.
     * Specifically, this is the IEEE 754 double precision version found in cephes/misc/polevl.c,
     * as the function polevl().
     *
     * Cephes can be found on NIST's Guide to Available Mathematical Software here:
     * https://gams.nist.gov/cgi-bin/serve.cgi/Package/CEPHES
     *
     * All credit goes to Stephen Moshier.
     *
     * @param x value to evaluate the polynomial at
     * @param coefficients coefficients of the terms of the polynomial.
     * @param degree degree of the polynomial
     * @return value of the polynomial
     */
    /*
    Cephes Math Library Release 2.1:  December, 1988
    Copyright 1984, 1987, 1988 by Stephen L. Moshier
    Direct inquiries to 30 Frost Street, Cambridge, MA 02140
    */
    public static double evaluatePolynomial(double x, double[] coefficients, int degree) {
        // this could probably be rewritten as a for loop, but I don't want to risk breaking anything
        // in Cephes.
        int p = 0;
        int i = degree;
        // the original C implementation uses pointers, so i've emulated them with array indexing
        double ans = coefficients[p++];
        do {
            ans = ans * x + coefficients[p++];
        } while (--i > 0);
        return ans;
    }

    /**
     * Evaluates a polynomial of arbitrary degree at the given value of x, assuming the coefficient
     * of the highest-degree term is 1.
     *
     * This method is like {@link #evaluatePolynomial(double, double[], int)}, except that the leading
     * coefficient of the polynomial is assumed to be one and is omitted from the coefficients array.
     *
     * Coefficients are stored in reverse order. The first element of coefficients is the coefficient
     * of the second-highest-degree term of the polynomial, and the last element is the coefficient
     * of the lowest-degree term - that degree being zero.
     *
     * For example, a 3rd degree polynomial would be evaluated as:
     * y = coefficients[2] + coefficients[1] * x + coefficients[0] * x^2 + x^3
     *
     * The length of coefficients must be equal to the degree of the polynomial.
     *
     * This implementation comes from the Cephes Mathematical Library written by Stephen Moshier.
     * Specifically, this is the IEEE 754 double precision version found in cephes/misc/polevl.c,
     * as the function p1evl().
     *
     * Cephes can be found on NIST's Guide to Available Mathematical Software here:
     * https://gams.nist.gov/cgi-bin/serve.cgi/Package/CEPHES
     *
     * All credit goes to Stephen Moshier.
     *
     * @param x value to evaluate the polynomial at
     * @param coefficients coefficients of each term of the polynomial
     * @param degree degree of the polynomial
     * @return value of the polynomial
     */
    // WithLC1 stands for "with leading coefficient 1"
    /*
    Cephes Math Library Release 2.1:  December, 1988
    Copyright 1984, 1987, 1988 by Stephen L. Moshier
    Direct inquiries to 30 Frost Street, Cambridge, MA 02140
    */
    public static double evaluatePolynomialWithLC1(double x, double[] coefficients, int degree) {
        // like evaluatePolynomial(), this could probably be rewritten as a for loop.
        int p = 0;
        int i = degree - 1;
        // the original C implementation uses pointers, so i've emulated them with array indexing
        double ans = x + coefficients[p++];
        do {
            ans = ans * x + coefficients[p++];
        } while (--i > 0);
        return ans;
    }
}