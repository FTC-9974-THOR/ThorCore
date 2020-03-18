package org.ftc9974.thorcore.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.concurrent.atomic.AtomicBoolean;

public final class UpdateLoopHandler implements OpModeManagerNotifier.Notifications {

    private static final String TAG = "org.ftc9974.thorcore.util.UpdateLoopHandler";

    private AtomicBoolean stopRequested, enabled;
    private Runnable handler;
    private boolean startOnOpModeStart;

    private Thread looper;
    private final Object lock = new Object();

    public UpdateLoopHandler(Runnable handler) {
        this.handler = handler;
        stopRequested = new AtomicBoolean(false);
        enabled = new AtomicBoolean(false);

        looper = new Thread(() -> {
            while (!stopRequested.get()) {
                if (enabled.get()) {
                    this.handler.run();
                } else {
                    synchronized (lock) {
                        try {
                            lock.wait();
                        } catch (InterruptedException e) {
                            RobotLog.ee(TAG, e, "Interrupted while waiting for re-enable");
                        }
                    }
                }
            }
        });

        OpModeUtilities.registerListener(this);
    }

    public void startOnOpModeStart() {
        startOnOpModeStart = true;
    }

    public void start() {
        if (!looper.isAlive()) {
            looper.start();
        }
    }

    public void stop() {
        stopRequested.set(true);
        synchronized (lock) {
            lock.notify();
        }
    }

    public void setEnabled(boolean enabled) {
        this.enabled.set(enabled);
        synchronized (lock) {
            lock.notify();
        }
    }

    public boolean isEnabled() {
        return enabled.get();
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        if (startOnOpModeStart) {
            start();
        }
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stop();
        OpModeUtilities.unregisterListener(this);
    }
}
