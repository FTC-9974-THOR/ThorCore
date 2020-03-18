package org.ftc9974.thorcore.util;

import android.annotation.TargetApi;
import android.os.Build;
import android.support.annotation.NonNull;

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
     * Calculates the Riemann sum of a function. Works best with doubles.
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
    public static <T extends Comparable, R extends Comparable> R integrate(@NonNull Function<T, R> function,
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
        return map(t, 0, 1, from, to);
    }

    public static Vector2 lerp(Vector2 from, Vector2 to, double t) {
        return new Vector2(map(t, 0, 1, from.getX(), to.getX()), map(t, 0, 1, from.getY(), to.getY()));
    }

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
        return sum / values.length;
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

    public static double constrain(double x, double low, double high) {
        if (x < low) {
            return low;
        } else if (x > high) {
            return high;
        }
        return x;
    }
}