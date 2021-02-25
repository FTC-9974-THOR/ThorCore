package org.ftc9974.thorcore.vision;

import android.graphics.Bitmap;
import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.ftc9974.thorcore.NativeCodeLoader;
import org.ftc9974.thorcore.util.MathUtilities;

import java.nio.ByteBuffer;


/**
 * Vision system written in assembly. Uses NEON SIMD instructions to accelerate vision to ludicrous
 * speeds.
 *
 * Current work-in-progress implementation runs at 1100fps. (processYUY2)
 *
 * Assembly code is in src/main/cpp/NEONVision.s.
 */
// work-in-progress
public class NEONVision {

    private static String TAG = "NEONVision";

    static {
        NativeCodeLoader.load();
    }

    private ByteBuffer imageBuffer;
    private final Object lock = new Object();

    public NEONVision() { }

    public NEONVision(int imageWidth, int imageHeight) {
        // in ARGB8888, each pixel is 4 bytes.
        // size of image = width * height * 4
        ensureBufferSize(imageWidth * imageHeight * 4);
    }

    private void ensureBufferSize(int size) {
        int bufferSize = size;
        // the NEON code operates on 16 pixels (64 bytes) at a time. if the buffer size isn't a multiple
        // of 64, the NEON code would have to process the last dozen or so pixels in a much less
        // efficient manner. any camera that uses a 16:9 aspect ratio will be fine, as 16 must be a
        // factor of the number of pixels.
        int padAmount = bufferSize % 64;
        if (padAmount != 0) {
            bufferSize += 64 - padAmount;
        }
        // only reallocate if:
        //   a) imageBuffer is null
        //   b) the optimal size isn't in the same 64-byte interval as imageBuffer's size. in theory,
        //      this is if the buffer is smaller than the min size (the size parameter) or larger
        //      than the padded size (bufferSize). in practice, it's just if the padded size != actual size,
        //      since the padding logic will pad to the same size anyways in the case of alignment.
        // tl;dr this won't reallocate unless you change the image size, and even then it sometimes
        // doesn't have to reallocate.
        if (imageBuffer == null || imageBuffer.capacity() != bufferSize) {
            imageBuffer = ByteBuffer.allocateDirect(bufferSize);
        }
    }

    /*public long processBitmapARGB(Bitmap bitmap, int lowA, int highA, int lowR, int highR, int lowG, int highG, int lowB, int highB) {
        Bitmap workingBitmap;
        if (bitmap.getConfig() != Bitmap.Config.ARGB_8888) {
            workingBitmap = bitmap.copy(Bitmap.Config.ARGB_8888, false);
        } else {
            workingBitmap = bitmap;
        }

        synchronized (lock) {
            ensureBufferSize(workingBitmap.getByteCount());

            imageBuffer.position(0);
            workingBitmap.copyPixelsToBuffer(imageBuffer);
            // fill the alignment padding with false positives. the native assembly code will
            // account for this in its calculations.
            while (imageBuffer.hasRemaining()) {
                imageBuffer.putInt(Color.argb(lowA, lowR, lowG, lowB));
            }

            int pixel = imageBuffer.getInt(0);
            RobotLog.ii(TAG, "java pixel: ARGB(%d, %d, %d, %d) -> %x",
                    Color.alpha(pixel),
                    Color.red(pixel),
                    Color.green(pixel),
                    Color.blue(pixel),
                    pixel);

            long start = System.nanoTime();
            long ret = processARGB(imageBuffer, imageBuffer.capacity(), workingBitmap.getWidth() * workingBitmap.getHeight(), lowA, highA, lowR, highR, lowG, highG, lowB, highB);
            double timeElapsed = (System.nanoTime() - start) / 1e9;
            RobotLog.ii(TAG, "elapsed time: %f theoretical performance: %f fps", timeElapsed, 1 / timeElapsed);
            return ret;
        }
    }*/

    public long processCameraFrame(CameraFrame frame, int lowY, int highY, int lowU, int highU, int lowV, int highV) {
        long highColor = highY << 16 | highU << 8 | highV;
        long lowColor = lowY << 16 | lowU << 8 | lowV;
        return processYUY2(frame.getImageBuffer(), frame.getImageSize(), highColor, lowColor);
    }

    public static native boolean canUseNeonAcceleration();

    //public native long test(long a, long b);

    //public native long processARGB(ByteBuffer imageBuffer, long bufferLen, long numPixels, int lowA, int highA, int lowR, int highR, int lowG, int highG, int lowB, int highB);

    //public native long processPointerARGB(long bufferPtr, long bufferLen, @ColorInt int highColor, @ColorInt int lowColor);

    // this does in fact resolve. it's just that android studio doesn't recognize what it's linked to.
    // there's an extern symbol declared in JNINEONVision.cpp, but that doesn't actually do anything.
    // processYUY2 is actually implemented in assembly, in NEONVision.s.
    @SuppressWarnings("JavaJniMissingFunction")
    public native long processYUY2(long bufferPtr, long bufferLen, long highColor, long lowColor);
}
