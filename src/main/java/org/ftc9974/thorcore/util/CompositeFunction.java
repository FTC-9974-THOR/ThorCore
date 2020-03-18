package org.ftc9974.thorcore.util;

import android.annotation.TargetApi;
import android.os.Build;
import android.os.SystemClock;

import java.util.function.Supplier;

/**
 * Uses Java 8 features to perform function composition.
 */
@TargetApi(Build.VERSION_CODES.N)
@FunctionalInterface
public interface CompositeFunction extends Supplier<Boolean> {

    @Override
    Boolean get();

    default CompositeFunction and(Supplier<Boolean> other) {
        return () -> this.get() && other.get();
    }

    default CompositeFunction or(Supplier<Boolean> other) {
        return () -> this.get() || other.get();
    }

    default CompositeFunction xor(Supplier<Boolean> other) {
        return () -> this.get() ^ other.get();
    }

    default CompositeFunction negated() {
        return () -> !this.get();
    }

    default CompositeFunction withTimeout(double time) {
        return or(afterTimeElapses(time));
    }

    default CompositeFunction withMinimumTime(double time) {
        return and(afterTimeElapses(time));
    }

    static CompositeFunction afterTimeElapses(double time) {
        final long startTime = SystemClock.uptimeMillis();
        return () -> SystemClock.uptimeMillis() - startTime > 1000 * time;
    }

    static CompositeFunction not(Supplier<Boolean> other) {
        return ((CompositeFunction) other::get).negated();
    }
}