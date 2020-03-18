package org.ftc9974.thorcore;

import junit.framework.Assert;

import org.ftc9974.thorcore.control.math.Matrix;
import org.junit.Test;

public class MatrixTests {

    private void printWithLabel(String label, String msg) {
        StringBuilder builder = new StringBuilder();
        builder.append(label);
        builder.append(": ");
        boolean firstLine = true;
        for (String s : msg.split("\n")) {
            if (!firstLine) {
                for (int i = 0; i < label.length() + 2; i++) {
                    builder.append(" ");
                }
            } else {
                firstLine = false;
            }
            builder.append(s);
            builder.append("\n");
        }
        System.out.println(builder.toString());
    }

    @Test
    public void test_equals() {
        Matrix mat1 = new Matrix(new double[][]{
                {1, 2, 3},
                {4, 5, 6}
        });
        Matrix mat2 = new Matrix(new double[][]{
                {1, 2, 3},
                {5, 5, 6}
        });
        Assert.assertFalse(mat1.equals(mat2));
    }

    @Test
    public void test_addition() {
        Matrix mat1 = new Matrix(new double[][]{
                {1, 2, 3},
                {1, 2, 3}
        });
        Matrix mat2 = new Matrix(new double[][]{
                {2, 3, 4},
                {2, 3, 4}
        });

        Matrix result = Matrix.add(mat1, mat2);
        Matrix expected = new Matrix(new double[][]{
                {3, 5, 7},
                {3, 5, 7}
        });
        System.out.println(result.toString());
        System.out.println(expected.toString());
        Assert.assertEquals(result, expected);
    }

    @Test
    public void test_scalarMultiplication() {
        double scalar = 2;
        Matrix mat = new Matrix(new double[][]{
                {1, 2, 3},
                {4, 5, 6}
        });
        Matrix result = Matrix.multiply(scalar, mat);
        Matrix expected = new Matrix(new double[][]{
                {2, 4, 6},
                {8, 10, 12}
        });
        Assert.assertEquals(expected, result);
    }

    @Test
    public void test_navigation() {
        final double ax = 1, az = 1, ar = 1, Rw = 2, Lw = 6, Ww = 6;
        double m = 1.0 / (Rw * ax);
        Matrix fk = new Matrix(new double[][] {
                {1, 1 / ar, -(Lw + Ww) / az},
                {1, -1 / ar, (Lw + Ww) / az},
                {1, -1 / ar, (Lw + Ww) / az},
                {1, 1 / ar, -(Lw + Ww) / az}
        });
        Matrix input = new Matrix(new double[][] {
                {1},
                {0},
                {0}
        });
        printWithLabel("FK", fk.toString());
        System.out.println();
        printWithLabel("Input", input.toString());
        System.out.println();
        Matrix intermediate = Matrix.multiply(m, fk);
        printWithLabel("INT", intermediate.toString());
        System.out.println();
        Matrix result = Matrix.multiply(intermediate, input);
        printWithLabel("RES", result.toString());
    }
}
