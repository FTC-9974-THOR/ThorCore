package org.ftc9974.thorcore;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

// double buffers an object to pass it between threads. for example, used in USBWebcamBase to buffer
// camera frames. does not need synchronization.
public class TransceiverDoubleBuffer<T> {

    private volatile T a, b;
    private final AtomicBoolean usingBufferB, locked;

    private Consumer<T> cleanupHandler;

    public TransceiverDoubleBuffer() {
        usingBufferB = new AtomicBoolean(false);
        locked = new AtomicBoolean(false);
    }

    // in case you need to, for example, manually free memory.
    // try to keep the callback short.
    public void setCleanupHandler(Consumer<T> handler) {
        cleanupHandler = handler;
    }

    public void forceFlip() {
        usingBufferB.set(!usingBufferB.get());
    }

    public void set(T value) {
        // if we're locked, discard new data
        if (locked.get()) {
            if (cleanupHandler != null) {
                cleanupHandler.accept(value);
            }
            return;
        }
        if (usingBufferB.get()) {
            if (cleanupHandler != null) {
                cleanupHandler.accept(a);
            }
            a = value;
            usingBufferB.set(false);
        } else {
            if (cleanupHandler != null) {
                cleanupHandler.accept(b);
            }
            b = value;
            usingBufferB.set(true);
        }
    }

    public T get() {
        return usingBufferB.get() ? b : a;
    }

    // reject new data. ideally would be solved by triple buffering
    public void lockForRead() {
        locked.set(true);
    }

    public void releaseLock() {
        locked.set(false);
    }
}
