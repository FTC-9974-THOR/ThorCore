package org.ftc9974.thorcore;

import org.ftc9974.thorcore.util.MathUtilities;
import org.junit.Assert;
import org.junit.Test;

import java.util.function.BiFunction;
import java.util.function.BinaryOperator;
import java.util.function.Function;

public class MathUtilitiesTests {

    @Test
    public void integrateTest() {
        double lowBound = 0;
        double highBound = 5;
        Function<Double, Double> function = (d1) -> 4.0 * d1;
        BinaryOperator<Double> tAdder = (d1, d2) -> d1 + d2;
        BinaryOperator<Double> rAdder = (d1, d2) -> d1 + d2;
        BiFunction<Double, Double, Double> multiplier = (d1, d2) -> d1 * d2;
        System.out.println(String.format("Low: %f High: %f", lowBound, highBound));
        double result = MathUtilities.integrate(function, lowBound, highBound, 0.001, tAdder, rAdder, multiplier, 0.0);
        Assert.assertEquals(50, result, 0.1);
    }

    @Test
    public void mapTest() {
        Assert.assertEquals(180, MathUtilities.map(0.5, 0, 1, 0, 360), 0.1);
    }
}
