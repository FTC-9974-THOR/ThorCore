package org.ftc9974.thorcore;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * Handles loading of libThorCore.so, which holds all native code for ThorCore in one big shared
 * library. Different native modules can be easily added via the native build chain, and will be
 * linked into libThorCore.so at compile time.
 */
public class NativeCodeLoader {

    private static boolean loaded = false;

    static {
        load();
    }

    public static void load() {
        if (!loaded) {
            RobotLog.ii("org.ftc9974.thorcore.NativeCodeLoader", "Loading native library...");
            System.loadLibrary("ThorCore");
            logLoaded();
            loaded = true;
        }
    }

    private static native void logLoaded();
}
