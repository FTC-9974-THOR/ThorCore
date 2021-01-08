package org.ftc9974.thorcore.control.math;

// work in progress
public final class Tensor {

    private double[] values;

    private int[] dimensions;

    public Tensor(int... dimensions) {
        this.dimensions = dimensions;
        int length = 1;
        for (int dimension : dimensions) {
            length *= dimension;
        }
        values = new double[length];
    }

    public Tensor(double[] values, int... dimensions) {
        this.dimensions = dimensions;
        this.values = values;
    }

    public double get(int... index) {
        return values[flattenIndex(index)];
    }

    public void set(double value, int... index) {
        values[flattenIndex(index)] = value;
    }

    public static Tensor add(Tensor mat1, Tensor mat2) {
        if (mat1.dimensions != mat2.dimensions) {
            throw new IllegalArgumentException("Size mismatch while adding tensors");
        }
        Tensor retMat = new Tensor(mat1.dimensions);
        for (int i = 0; i < mat1.values.length; i++) {
            retMat.values[i] = mat1.values[i] + mat2.values[i];
        }
        return retMat;
    }

    public static Tensor subtract(Tensor mat1, Tensor mat2) {
        if (mat1.dimensions != mat2.dimensions) {
            throw new IllegalArgumentException("Size mismatch while subtracting tensors");
        }
        Tensor retMat = new Tensor(mat1.dimensions);
        for (int i = 0; i < mat1.values.length; i++) {
            retMat.values[i] = mat1.values[i] - mat2.values[i];
        }
        return retMat;
    }

    public static Tensor elementWiseMultiply(Tensor mat1, Tensor mat2) {
        if (mat1.dimensions != mat2.dimensions) {
            throw new IllegalArgumentException("Size mismatch while multiplying tensors");
        }
        Tensor retMat = new Tensor(mat1.dimensions);
        for (int i = 0; i < mat1.values.length; i++) {
            retMat.values[i] = mat1.values[i] * mat2.values[i];
        }
        return retMat;
    }

    public static Tensor elementWiseDivide(Tensor mat1, Tensor mat2) {
        if (mat1.dimensions != mat2.dimensions) {
            throw new IllegalArgumentException("Size mismatch while dividing tensors");
        }
        Tensor retMat = new Tensor(mat1.dimensions);
        for (int i = 0; i < mat1.values.length; i++) {
            retMat.values[i] = mat1.values[i] / mat2.values[i];
        }
        return retMat;
    }

    private int flattenIndex(int... index) {
        if (index.length != dimensions.length) {
            throw new IllegalArgumentException("Illegal index");
        }
        int flattenedIndex = 0;
        for (int i = 0; i < dimensions.length; i++) {
            if (index[i] >= dimensions[i]) {
                throw new ArrayIndexOutOfBoundsException(index[i]);
            }
            flattenedIndex += index[i] * dimensions[i];
        }
        return flattenedIndex;
    }
}
