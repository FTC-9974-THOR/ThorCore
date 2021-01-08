package org.ftc9974.thorcore;

import org.ftc9974.thorcore.util.MathUtilities;
import org.junit.Assert;
import org.junit.Test;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.util.Random;
import java.util.Scanner;
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

    @Test
    public void testFresnelIntegral() throws IOException {
        ClassLoader classLoader = getClass().getClassLoader();
        Assert.assertNotNull(classLoader);
        Scanner scanner = new Scanner(classLoader.getResourceAsStream("fresnelTestData.csv"));
        scanner.useDelimiter(",");
        while (scanner.hasNextDouble()) {
            double x = scanner.nextDouble();
            double expectedS = scanner.nextDouble();
            double expectedC = scanner.nextDouble();
            double[] actual = MathUtilities.fresnelIntegral(x);
            Assert.assertEquals(expectedC, actual[0], 0.001);
            Assert.assertEquals(expectedS, actual[1], 0.001);
        }
    }

    public void testEvaluatePolynomial() {
        Random rng = new Random();
        for (int i = 0; i < 10000; i++) {
            int degree = rng.nextInt(20) + 1;
        }
    }
}
