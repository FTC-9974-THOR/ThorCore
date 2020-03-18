package org.ftc9974.thorcore.util;

public final class StringUtilities {

    public static String join(String delimiter, byte... items) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < items.length; i++) {
            builder.append(items[i]);
            if (i != items.length - 1) {
                builder.append(delimiter);
            }
        }
        return builder.toString();
    }

    public static String join(String delimiter, short... items) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < items.length; i++) {
            builder.append(items[i]);
            if (i != items.length - 1) {
                builder.append(delimiter);
            }
        }
        return builder.toString();
    }

    public static String join(String delimiter, int... items) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < items.length; i++) {
            builder.append(items[i]);
            if (i != items.length - 1) {
                builder.append(delimiter);
            }
        }
        return builder.toString();
    }

    public static String join(String delimiter, long... items) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < items.length; i++) {
            builder.append(items[i]);
            if (i != items.length - 1) {
                builder.append(delimiter);
            }
        }
        return builder.toString();
    }

    public static String join(String delimiter, float... items) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < items.length; i++) {
            builder.append(items[i]);
            if (i != items.length - 1) {
                builder.append(delimiter);
            }
        }
        return builder.toString();
    }

    public static String join(String delimiter, double... items) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < items.length; i++) {
            builder.append(items[i]);
            if (i != items.length - 1) {
                builder.append(delimiter);
            }
        }
        return builder.toString();
    }

    public static String join(String delimiter, boolean... items) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < items.length; i++) {
            builder.append(items[i]);
            if (i != items.length - 1) {
                builder.append(delimiter);
            }
        }
        return builder.toString();
    }

    public static String join(String delimiter, String... items) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < items.length; i++) {
            builder.append(items[i]);
            if (i != items.length - 1) {
                builder.append(delimiter);
            }
        }
        return builder.toString();
    }
}
