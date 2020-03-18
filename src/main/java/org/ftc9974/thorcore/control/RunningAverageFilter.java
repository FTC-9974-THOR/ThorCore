package org.ftc9974.thorcore.control;

public final class RunningAverageFilter {

    private double[] samples;
    private int order;
    private int numSamples;

    public RunningAverageFilter(int order) {
        this.order = order;
        samples = new double[order];
    }

    public double update(double input) {
        if (numSamples < order) {
            samples[numSamples] = input;
            numSamples++;
        } else {
            System.arraycopy(samples, 1, samples, 0, order - 1);
            samples[order - 1] = input;
        }

        double sum = 0;
        for (int i = 0; i < numSamples; i++) {
            sum += samples[i];
        }

        return sum / numSamples;
    }
}
