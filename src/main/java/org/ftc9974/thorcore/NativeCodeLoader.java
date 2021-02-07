package org.ftc9974.thorcore;

/**
 * Handles loading of libThorCore.so, which holds all native code for ThorCore in one big shared
 * library. Different native modules can be easily added via the native build chain, and will be
 * linked into libThorCore.so at compile time.
 */
public class NativeCodeLoader {

    static {
        System.loadLibrary("ThorCore");
    }
}
