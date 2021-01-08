package org.ftc9974.thorcore.control;

import com.qualcomm.robotcore.util.RobotLog;

public class RunningAverage {

    private double sum, samples;

    public RunningAverage() {
        reset();
    }

    public double update(double input) {
        sum += input;
        samples += 1;
        return get();
    }

    public double get() {
        return sum / samples;
    }

    public void reset() {
        sum = 0;
        samples = 0;
    }

    public int numSamples() {
        return (int) samples;
    }
}
