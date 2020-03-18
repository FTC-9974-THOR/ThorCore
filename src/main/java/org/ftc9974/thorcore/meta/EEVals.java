package org.ftc9974.thorcore.meta;

import android.content.Context;

import com.google.gson.Gson;
import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.util.FileUtilities;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Map;

/**
 * API for EEVals. Most of these methods are for internal use, so it is rare you will ever need to
 * directly interact with this class.
 */
public final class EEVals {

    private static final String TAG = "org.ftc9974.thorcore.meta.EEVals";

    private static Map<String, String> cache;
    private static Gson gson;

    private EEVals() {}

    public static void init(Context context) {
        if (gson == null) {
            gson = new Gson();
        }
        buildCache(context);
    }

    private static void buildCache(Context context) {
        FileInputStream inputStream;
        try {
            inputStream = context.openFileInput("eevals.json");
        } catch (FileNotFoundException e) {
            RobotLog.ee(TAG, e, "EEVal file not found");
            e.printStackTrace();
            return;
        }
        try {
            cache = gson.fromJson(FileUtilities.getFileContents(inputStream), cache.getClass());
            inputStream.close();
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "IOException while reading/closing file");
        }
    }

    public static <T> T get(String key, Class<T> type) {
        if (!cache.containsKey(key)) {
            throw new IllegalArgumentException(String.format("\"%s\" is not in the EEVal registry", key));
        }
        return gson.fromJson(cache.get(key), type);
    }

    public static <T> void set(String key, T value) {
        cache.put(key, gson.toJson(value));
    }

    public static boolean contains(String key) {
        return cache.containsKey(key);
    }

    public static void store(Context context) {
        String jsonCache = gson.toJson(cache);
        try {
            FileOutputStream outputStream = context.openFileOutput("eevals.json", Context.MODE_PRIVATE);
            outputStream.write(jsonCache.getBytes());
            outputStream.close();
        } catch (FileNotFoundException e) {
            RobotLog.ee(TAG, e, "EEVal file not found");
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "IOException while writing to storage");
        }
    }
}
