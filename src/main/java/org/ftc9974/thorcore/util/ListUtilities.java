package org.ftc9974.thorcore.util;

import java.util.Arrays;
import java.util.List;

public final class ListUtilities {

    public static String join(String delimiter, String... strings) {
        return join(delimiter, Arrays.asList(strings));
    }

    public static String join(String delimiter, List<String> strings) {
        if (strings.size() == 0) {
            return "";
        }
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < strings.size() - 1; i++) {
            builder.append(strings.get(i));
            builder.append(delimiter);
        }
        builder.append(strings.get(strings.size() - 1));
        return builder.toString();
    }
}
