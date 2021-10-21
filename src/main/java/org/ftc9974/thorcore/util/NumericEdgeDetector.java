package org.ftc9974.thorcore.util;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * Numeric edge detector with configurable hysteresis. Basically a Schmidt trigger.
 */
public class NumericEdgeDetector {

    private final double highThreshold, lowThreshold;

    private double last;

    private boolean rising;
    private boolean falling;
    private boolean changing;

    private final ReadWriteLock lock;

    /**
     * Constructs a new NumericEdgeDetector.
     *
     * @param threshold edge threshold
     * @param startingValue the initial value to compare to on the first call to update()
     */
    public NumericEdgeDetector(double threshold, double startingValue) {
        highThreshold = threshold;
        lowThreshold = threshold;
        last = startingValue;
        rising = falling = changing = false;
        lock = new ReentrantReadWriteLock(true);
    }

    /**
     * Constructs a new NumericEdgeDetector.
     *
     * @param threshold edge threshold
     * @param hysteresis hysteresis
     * @param startingValue the initial value to compare to on the first call to update()
     */
    public NumericEdgeDetector(double threshold, double hysteresis, double startingValue) {
        highThreshold = threshold + hysteresis;
        lowThreshold = threshold - hysteresis;
        last = startingValue;
        rising = falling = changing = false;
        lock = new ReentrantReadWriteLock(true);
    }

    /**
     * Checks for edges triggered by a new value.
     *
     * @param value new value to compare to
     * @return the provided value
     */
    public double update(double value) {
        lock.writeLock().lock();
        try {
            rising = last <= highThreshold && value > highThreshold;
            falling = last >= lowThreshold && value < lowThreshold;
            changing = rising || falling;
            last = value;
        } finally {
            lock.writeLock().unlock();
        }
        return value;
    }

    /**
     * Gets the last value passed to update().
     *
     * @return last value
     */
    public double getLast() {
        lock.readLock().lock();
        double ret;
        try {
            ret = last;
        } finally {
            lock.readLock().unlock();
        }
        return ret;
    }

    /**
     * Indicates if the last call to update() triggered a rising edge.
     *
     * @return true if a rising edge is detected, false otherwise
     */
    public boolean isRising() {
        lock.readLock().lock();
        boolean ret;
        try {
            ret = rising;
        } finally {
            lock.readLock().unlock();
        }
        return ret;
    }

    /**
     * Indicates if the last call to update() triggered a falling edge.
     *
     * @return true if a falling edge is detected, false otherwise
     */
    public boolean isFalling() {
        lock.readLock().lock();
        boolean ret;
        try {
            ret = falling;
        } finally {
            lock.readLock().unlock();
        }
        return ret;
    }

    /**
     * Indicates if the last call to update() triggered a rising or falling edge.
     *
     * @return true if a change of value is detected, false otherwise
     */
    public boolean isChanging() {
        lock.readLock().lock();
        boolean ret;
        try {
            ret = changing;
        } finally {
            lock.readLock().unlock();
        }
        return ret;
    }
}
