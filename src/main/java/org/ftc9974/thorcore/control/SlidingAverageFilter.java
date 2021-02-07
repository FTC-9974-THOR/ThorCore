package org.ftc9974.thorcore.control;

import java.util.Arrays;

public final class SlidingAverageFilter {

    // samples are stored in a circular buffer, because it doesn't matter
    // what order they're stored in, the average will be the same
    private final double[] samples;
    private int index;

    private final int order;
    private int numSamples;

    public SlidingAverageFilter(int order) {
        this.order = order;
        samples = new double[order];
    }

    public SlidingAverageFilter(int order, double initialValue) {
        this(order);
        Arrays.fill(samples, initialValue);
    }

    /**
     * Accepts new data and returns the new value of the filter.
     *
     * @param input new data
     * @return new value
     */
    public double update(double input) {
        if (numSamples < order) {
            numSamples++;
        }
        samples[index++] = input;
        if (index == order) {
            index = 0;
        }

        return get();
    }

    /**
     * Gets the current value (average) of the filter.
     *
     * @return value
     */
    public double get() {
        double sum = 0;
        for (int i = 0; i < numSamples; i++) {
            sum += samples[i];
        }

        return sum / numSamples;
    }
}
