package org.ftc9974.thorcore.control.navigation;

import org.ftc9974.thorcore.control.math.Vector2;
import org.junit.Assert;
import org.junit.Test;

import java.util.Random;

public class MecanumEncoderCalculatorTests {

    @Test
    public void test() {
        MecanumEncoderCalculator vector = new MecanumEncoderCalculator(13.7 * 2, 96);
        ScalarMecanumEncoderCalculator regular = new ScalarMecanumEncoderCalculator(13.7 * 2, 96);

        Vector2 target = new Vector2(0, 100);
        System.out.println("Vector");
        printArray(vector.calculate(target));
        System.out.println("Regular");
        printArray(regular.calculate(target));
        Assert.assertArrayEquals(regular.calculate(target), vector.calculate(target));

        target = new Vector2(100, 0);
        System.out.println("Vector");
        printArray(vector.calculate(target));
        System.out.println("Regular");
        printArray(regular.calculate(target));
        Assert.assertArrayEquals(regular.calculate(target), vector.calculate(target));

        target = new Vector2(100, 100);
        System.out.println("Vector");
        printArray(vector.calculate(target));
        System.out.println("Regular");
        printArray(regular.calculate(target));
        Assert.assertArrayEquals(regular.calculate(target), vector.calculate(target));

        Random random = new Random();
        for (int i = 0; i < 100; i++) {
            target = new Vector2(random.nextInt(), random.nextInt());
            Assert.assertArrayEquals(regular.calculate(target), vector.calculate(target));
        }
    }

    private void printArray(int[] arr) {
        System.out.print("[ ");
        for (int i : arr) {
            System.out.print(i);
            System.out.print(" ");
        }
        System.out.println("]");
    }
}
