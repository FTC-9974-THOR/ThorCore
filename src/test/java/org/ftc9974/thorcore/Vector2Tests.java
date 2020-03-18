package org.ftc9974.thorcore;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.util.MathUtilities;
import org.junit.Assert;
import org.junit.Test;

public class Vector2Tests {

    private boolean ulpEq(Vector2 a, Vector2 b) {
        double lowX = a.getX() - Math.ulp(a.getX());
        double highX = a.getX() + Math.ulp(a.getX());
        double lowY = a.getY() - Math.ulp(a.getY());
        double highY = a.getY() + Math.ulp(a.getY());
        return MathUtilities.withinRange(b.getX(), lowX, highX) && MathUtilities.withinRange(b.getY(), lowY, highY);
    }

    @Test
    public void test_someThings() {
        Vector2 targetPosition = new Vector2(1219, -1219);
        Vector2 currentPosition = new Vector2(431, -431);
        double currentHeading = -0.79;
        Vector2 toTarget = targetPosition.subtract(currentPosition);
        System.out.println(toTarget.toString());
        toTarget = toTarget.rotate(-currentHeading + 0.5 * Math.PI);
        System.out.println(toTarget.toString());
    }

    @Test
    public void test_equals() {
        Vector2 a = new Vector2(1, 1);
        Vector2 b = new Vector2(1, 1);
        Assert.assertEquals(a, b);
        Assert.assertEquals(b, a);
    }

    @Test
    public void test_addition() {
        Vector2 a = new Vector2(0, 1);
        Vector2 b = new Vector2(1, 0);
        Vector2 expected = new Vector2(1, 1);
        Vector2 result = a.add(b);
        Assert.assertEquals(expected, result);
        result = b.add(a);
        Assert.assertEquals(expected, result);
        result = Vector2.add(a, b);
        Assert.assertEquals(expected, result);
        result = Vector2.add(b, a);
        Assert.assertEquals(expected, result);
    }

    @Test
    public void test_subtraction() {
        Vector2 a = new Vector2(0, 1);
        Vector2 b = new Vector2(1, 0);

        // a - b
        Vector2 expected = new Vector2(-1, 1);
        Vector2 result = a.subtract(b);
        Assert.assertEquals(expected, result);
        result = Vector2.subtract(a, b);
        Assert.assertEquals(expected, result);

        // b - a
        expected = new Vector2(1, -1);
        result = b.subtract(a);
        Assert.assertEquals(expected, result);
        result = Vector2.subtract(b, a);
        Assert.assertEquals(expected, result);

        // a - a
        expected = new Vector2(0, 0);
        result = a.subtract(a);
        Assert.assertEquals(expected, result);
        result = Vector2.subtract(a, a);
        Assert.assertEquals(expected, result);

        // b - b
        result = b.subtract(b);
        Assert.assertEquals(expected, result);
        result = Vector2.subtract(b, b);
        Assert.assertEquals(expected, result);
    }

    @Test
    public void test_rotate() {
        Vector2 a = new Vector2(1, 0);
        Vector2 expected = new Vector2(0, 1);
        Vector2 result = a.rotate(0.5 * Math.PI);
        Assert.assertEquals(expected.getX(), result.getX(), 0.001);
        Assert.assertEquals(expected.getY(), result.getY(), 0.001);
        result = Vector2.rotate(a, 0.5 * Math.PI);
        Assert.assertEquals(expected.getX(), result.getX(), 0.001);
        Assert.assertEquals(expected.getY(), result.getY(), 0.001);
    }

    @Test
    public void test_getMagnitude() {
        Vector2 a = new Vector2(1, 0);
        double expected = 1;
        Assert.assertEquals(expected, a.getMagnitude(), 0.001);
    }

    @Test
    public void test_getHeading() {
        Vector2 a = new Vector2(1, 0);
        double expected = 0;
        Assert.assertEquals(expected, a.getHeading(), 0.001);
        a = new Vector2(0, 1);
        expected = 0.5 * Math.PI;
        Assert.assertEquals(expected, a.getHeading(), 0.001);

        a = new Vector2(0, 100);
        Vector2 b = new Vector2(0, 0);
        Vector2 tecTarget = a.subtract(b);
        double tecHeading = tecTarget.getHeading();
        Assert.assertEquals(0.5 * Math.PI, tecHeading, 0.001);
    }
}
