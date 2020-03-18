package org.ftc9974.thorcore.util;

import android.annotation.TargetApi;
import android.os.Build;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@TargetApi(Build.VERSION_CODES.N)
public final class TimingUtilities {

    public static void nothing() {}

    public static void blockWhile(LinearOpMode opMode, Supplier<Boolean> condition, @Nullable Runnable whileWaiting, @Nullable Runnable stopRequested) {
        while (!opMode.isStopRequested() && condition.get()) {
            if (whileWaiting != null) {
                whileWaiting.run();
            }
        }
        if (opMode.isStopRequested() && stopRequested != null) {
            stopRequested.run();
        }
    }

    public static void blockUntil(LinearOpMode opMode, Supplier<Boolean> condition, @Nullable Runnable whileWaiting, @Nullable Runnable stopRequested) {
        blockWhile(opMode, () -> !condition.get(), whileWaiting, stopRequested);
    }

    public static void sleep(LinearOpMode opMode, double time, @Nullable Runnable whileWaiting, @Nullable Runnable stopRequested) {
        blockUntil(opMode, CompositeFunction.afterTimeElapses(time), whileWaiting, stopRequested);
    }

    public static void runOnSeparateThreadConditionally(@NonNull Runnable func, @NonNull BooleanSupplier shouldTerminate, @NonNull Runnable onTermination) {
        Thread thread = new Thread(func);
        thread.start();
        runOnSeparateThread(() -> {
            while (thread.isAlive()) {
                if (!thread.isInterrupted() && shouldTerminate.getAsBoolean()) {
                    thread.interrupt();
                    onTermination.run();
                    break;
                }
            }
        });
    }

    public static void runOnSeparateThreadConditionally(@NonNull Runnable func, @NonNull BooleanSupplier shouldTerminate) {
        runOnSeparateThreadConditionally(func, shouldTerminate, TimingUtilities::nothing);
    }

    public static void runOnSeparateThread(@NonNull Runnable func, long terminateAfterMillis, @NonNull Runnable onTermination) {
        Thread thread = new Thread(func);
        thread.start();
        runAfterDelay(() -> {
            if (thread.isAlive() && !thread.isInterrupted()) {
                thread.interrupt();
            }
            onTermination.run();
        }, terminateAfterMillis);
    }

    public static void runOnSeparateThread(@NonNull Runnable func, long terminateAfterMillis) {
        runOnSeparateThread(func, terminateAfterMillis, TimingUtilities::nothing);
    }

    public static void runOnSeparateThread(@NonNull Runnable func) {
        new Thread(func).start();
    }

    public static void runAfterDelay(@NonNull Runnable func, long delayMillis) {
        runOnSeparateThread(() -> {
            try {
                Thread.sleep(delayMillis);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
            func.run();
        });
    }
}
