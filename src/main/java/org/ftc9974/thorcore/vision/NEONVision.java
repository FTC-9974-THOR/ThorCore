package org.ftc9974.thorcore.vision;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Build;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.ftc9974.thorcore.NativeCodeLoader;
import org.ftc9974.thorcore.robot.sensors.USBWebcam;
import org.ftc9974.thorcore.robot.sensors.USBWebcamBase;
import org.ftc9974.thorcore.util.MathUtilities;

import java.io.IOException;
import java.nio.ByteBuffer;


/**
 * Vision system written in assembly. Uses NEON SIMD instructions to accelerate vision to ludicrous
 * speeds.
 *
 * Current implementation runs at ~1000fps. (processYUY2)
 *
 * Assembly code is in src/main/asm/NEONVision-arm64.s.
 */
public class NEONVision{

    private static final String TAG = "NEONVision";

    static {
        NativeCodeLoader.load();
    }

    // thresholds for not-ring:
    // 0-255 0-232 92-145
    // 0-179 0-232 92-145
    // for ring:
    // 0-178 0-155 139-162

    public static long yuvColorLong(int y, int u, int v) {
        return (y & 0xff) << 16 | (u & 0xff) << 8 | (v & 0xff);
    }

    // these do in fact resolve. it's just that android studio doesn't recognize what they're linked to.
    // these functions are actually implemented in assembly, in NEONCore.s, which itself dispatches to
    // NEONVision-arm64.s, NEONVision-arm32.s, and NEONVisionStubs.s, depending on the target architecture.

    @SuppressWarnings({"JavaJniMissingFunction"})
    public static native boolean supportsNeonAcceleration();

    @SuppressWarnings("JavaJniMissingFunction")
    public static native long processYUY2(long bufferPtr, long bufferLen, long highColor, long lowColor);

    @SuppressWarnings("JavaJniMissingFunction")
    public static native long processYUY2ForDisplay(long bufferPtr, long bufferLen, long highColor, long lowColor);
}
