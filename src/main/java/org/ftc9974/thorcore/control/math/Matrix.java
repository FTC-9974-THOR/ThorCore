package org.ftc9974.thorcore.control.math;

import java.util.Arrays;

/**
 * Work-in-progress: Implements matrix math.
 */
public final class Matrix {

    private double[][] values;

    private int rows, cols;

    /**
     * Creates a matrix filled with zeros.
     * @param rows number of rows
     * @param cols number of columns
     */
    public Matrix(int rows, int cols) {
        this.rows = rows;
        this.cols = cols;
        values = new double[rows][cols];
    }

    /**
     * Creates a matrix with the specified values.
     * @param values values to initialize to
     */
    public Matrix(double[][] values) {
        this.values = values;
        rows = values.length;
        cols = values[0].length;
    }

    public int rows() {
        return rows;
    }

    public int cols() {
        return cols;
    }

    public double get(int row, int col) {
        return values[row][col];
    }

    public void set(double value, int row, int col) {
        values[row][col] = value;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Matrix) {
            Matrix other = (Matrix) obj;
            if (other.rows != rows || other.cols != cols) {
                return false;
            }
            for (int i = 0; i < rows; i++) {
                if (!Arrays.equals(other.values[i], values[i])) {
                    return false;
                }
            }
            return true;
        } else {
            return false;
        }
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        /*if (rows == 1) {
            builder.append("[ ");
            for (double v : values[0]) {
                builder.append(v);
                builder.append(" ");
            }
            builder.append("]");
        } else if (rows == 2) {
            builder.append("⎡ "); // u23a1
            for (double v : values[0]) {
                builder.append(v);
                builder.append(" ");
            }
            builder.append("\u23a4");
            builder.append("\n");
            builder.append("\u23a3 ");
            for (double v : values[1]) {
                builder.append(v);
                builder.append(" ");
            }
            builder.append("\u23a6");
        } else {
            for (int r = 0; r < rows; r++) {
                if (r == 0) {
                    builder.append("\u23a1 ");
                } else if (r == rows - 1) {
                    builder.append("\u23a3 ");
                } else {
                    builder.append("\u23a2 ");
                }
                for (double v : values[r]) {
                    builder.append(v);
                    builder.append(" ");
                }
                if (r == 0) {
                    builder.append("\u23a4\n");
                } else if (r == rows - 1) {
                    builder.append("\u23a5\n");
                } else {
                    builder.append("\u23a6");
                }
            }
        }*/
        for (double[] value : values) {
            for (double v : value) {
                builder.append(v);
                builder.append(" ");
            }
            builder.append("\n");
        }
        return builder.toString();
    }

    @Override
    public Matrix clone() {
        Matrix retMat = new Matrix(rows, cols);
        retMat.values = values;
        return retMat;
    }

    public static Matrix add(Matrix mat1, Matrix mat2) {
        if (mat1.rows != mat2.rows || mat1.cols != mat2.cols) {
            throw new IllegalArgumentException("Size mismatch");
        }

        Matrix retMat = new Matrix(mat1.rows, mat1.cols);
        for (int i = 0; i < mat1.rows; i++) {
            for (int j = 0; j < mat1.cols; j++) {
                retMat.set(mat1.get(i, j) + mat2.get(i, j), i, j);
            }
        }
        return retMat;
    }

    public static Matrix subtract(Matrix mat1, Matrix mat2) {
        if (mat1.rows != mat2.rows || mat1.cols != mat2.cols) {
            throw new IllegalArgumentException("Size mismatch");
        }

        Matrix retMat = new Matrix(mat1.rows, mat1.cols);
        for (int i = 0; i < mat1.rows; i++) {
            for (int j = 0; j < mat1.cols; j++) {
                retMat.set(mat1.get(i, j) - mat2.get(i, j), i, j);
            }
        }
        return retMat;
    }

    public static Matrix elementWiseMultiply(Matrix mat1, Matrix mat2) {
        if (mat1.rows != mat2.rows || mat1.cols != mat2.cols) {
            throw new IllegalArgumentException("Size mismatch");
        }

        Matrix retMat = new Matrix(mat1.rows, mat1.cols);
        for (int i = 0; i < mat1.rows; i++) {
            for (int j = 0; j < mat1.cols; j++) {
                retMat.set(mat1.get(i, j) * mat2.get(i, j), i, j);
            }
        }
        return retMat;
    }

    public static Matrix elementWiseDivide(Matrix mat1, Matrix mat2) {
        if (mat1.rows != mat2.rows || mat1.cols != mat2.cols) {
            throw new IllegalArgumentException("Size mismatch");
        }

        Matrix retMat = new Matrix(mat1.rows, mat1.cols);
        for (int i = 0; i < mat1.rows; i++) {
            for (int j = 0; j < mat1.cols; j++) {
                retMat.set(mat1.get(i, j) / mat2.get(i, j), i, j);
            }
        }
        return retMat;
    }

    public static Matrix multiply(Matrix mat1, Matrix mat2) {
        if (mat1.rows != mat2.cols) {
            throw new IllegalArgumentException("Size mismatch");
        }

        Matrix retMat = new Matrix(mat1.rows, mat2.cols);
        for (int i = 0; i < retMat.rows; i++) {
            for (int j = 0; j < retMat.cols; j++) {
                double c = 0;
                for (int m = 0; m < retMat.cols; m++) {
                    c += mat1.get(i, m) * mat2.get(m, j);
                }
                retMat.set(c, i, j);
            }
        }
        return retMat;
    }

    public static Matrix multiply(double scalar, Matrix mat) {
        Matrix retMat = mat.clone();
        for (int i = 0; i < mat.rows; i++) {
            for (int j = 0; j < mat.cols; j++) {
                retMat.set(scalar * mat.get(i, j), i, j);
            }
        }
        return retMat;
    }
}

