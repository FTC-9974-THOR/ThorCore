package org.ftc9974.thorcore.util;

import java.io.FileInputStream;
import java.io.IOException;

/**
 * Internal utilities for file IO.
 */
public final class FileUtilities {

    public static String getFileContents(FileInputStream fis) throws IOException {
        byte[] b = new byte[fis.available()];
        return new String(b);
    }
}
