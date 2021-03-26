package org.ftc9974.thorcore.control.math;

// wip rewrite of OldMatrix
public class Matrix {

    // indexed as row, col
    private double[][] elements;
    private int rows, cols;

    public double get(int row, int col) {
        return elements[row][col];
    }

    public void set(double value, int row, int col) {
        elements[row][col] = value;
    }


}
