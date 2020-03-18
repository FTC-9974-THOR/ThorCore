package org.ftc9974.thorcore.util;

import android.annotation.TargetApi;
import android.os.Build;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

@TargetApi(Build.VERSION_CODES.N)
public final class LambdaUtilities {

    public static <T, R, L extends List<R>> void transformList(List<T> sourceList, Function<T, R> function, L destList) {
        sourceList.forEach((item) -> {
            destList.add(function.apply(item));
        });
    }

    public static <T, R> ArrayList<R> transformList(List<T> list, Function<T, R> function) {
        ArrayList<R> ret = new ArrayList<>();
        transformList(list, function, ret);
        return ret;
    }
}
