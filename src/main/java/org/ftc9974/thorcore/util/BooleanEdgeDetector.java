package org.ftc9974.thorcore.util;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * Detects edges in a boolean signal.
 *
 * This class is thread-safe: any call to getLast(), isRising(), isFalling(), and isChanging() will
 * complete before any new calls to update() are completed.
 */
public class BooleanEdgeDetector {

    private boolean last;

    private boolean rising;
    private boolean falling;
    private boolean changing;

    private final ReadWriteLock lock;

    /**
     * Constructs a new BooleanEdgeDetector.
     *
     * @param startingValue the initial value to compare to on the first call to update()
     */
    public BooleanEdgeDetector(boolean startingValue) {
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
    public boolean update(boolean value) {
        lock.writeLock().lock();
        try {
            rising = !last && value;
            falling = last && !value;
            changing = last ^ value;
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
    public boolean getLast() {
        lock.readLock().lock();
        boolean ret;
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
