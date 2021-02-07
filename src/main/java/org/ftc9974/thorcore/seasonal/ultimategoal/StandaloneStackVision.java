package org.ftc9974.thorcore.seasonal.ultimategoal;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.os.Environment;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Frame;

import org.apache.commons.lang3.ArrayUtils;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;

public class StandaloneStackVision {

    private static final String TAG = "StandaloneStackVision";

    public enum StackHeight {
        ZERO_RINGS,
        ONE_RING,
        FOUR_RINGS,

        UNKNOWN
    }

    private final VuforiaLocalizer vuforia;
    private final Rect stackRegion, scanRegion;

    private double oneRingThreshold, fourRingThreshold;

    public StandaloneStackVision(HardwareMap hw, Rect stackRegion, Rect scanRegion, String webcamName, String vuforiaKey) {
        this(hw, stackRegion, scanRegion, hw.get(WebcamName.class, webcamName), vuforiaKey);
    }

    public StandaloneStackVision(HardwareMap hw, Rect stackRegion, Rect scanRegion, VuforiaLocalizer.CameraDirection cameraDirection, String vuforiaKey) {
        this(hw, stackRegion, scanRegion, ClassFactory.getInstance().getCameraManager().nameFromCameraDirection(cameraDirection), vuforiaKey);
    }

    private StandaloneStackVision(HardwareMap hw, Rect stackRegion, Rect scanRegion, CameraName cameraName, String vuforiaKey) {
        this.stackRegion = stackRegion;
        this.scanRegion = scanRegion;

        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraName = cameraName;
        parameters.vuforiaLicenseKey = vuforiaKey;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        oneRingThreshold = 0.12;
        fourRingThreshold = 0.65;
    }

    public void setOneRingThreshold(double threshold) {
        oneRingThreshold = threshold;
    }

    public void setFourRingThreshold(double threshold) {
        fourRingThreshold = threshold;
    }

    // known problem: calling vuforia.getFrameOnce() before the continuation is dispatched will
    // overwrite the previous continuation.
    // todo add some kind of reentrant lock

    public StackHeight analyze() {
        final ContinuationSynchronizer<StackHeight> synchronizer = new ContinuationSynchronizer<>();
        vuforia.getFrameOnce(Continuation.createTrivial(new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap == null) {
                    synchronizer.finish(StackHeight.UNKNOWN);
                } else {
                    synchronizer.finish(processBitmap(bitmap));
                }
            }
        }));
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        return synchronizer.getValue();
    }

    // warning: the supplied callback may not be called on the opmode thread
    public void analyzeAsync(Consumer<StackHeight> callback) {
        vuforia.getFrameOnce(Continuation.create(CallbackLooper.getDefault().getHandler(), new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap == null) {
                    callback.accept(StackHeight.UNKNOWN);
                } else {
                    callback.accept(processBitmap(bitmap));
                }
            }
        }));
    }

    private StackHeight processBitmap(Bitmap bitmap) {
        ensureRectIsWithinFrame(stackRegion, bitmap.getWidth(), bitmap.getHeight());
        ensureRectIsWithinFrame(scanRegion, bitmap.getWidth(), bitmap.getHeight());

        // by default, Vuforia converts to RGB_565. however, getPixels() converts to ARGB_8888.
        int[] pixels = new int[scanRegion.width() * scanRegion.height()];
        bitmap.getPixels(
                pixels,
                0, scanRegion.width(),
                scanRegion.left, scanRegion.top,
                scanRegion.width(), scanRegion.height()
        );

        //int[] matchImagePixels = new int[pixels.length];

        double matchFactor = 0;
        float[] hsv = new float[3];
        double pixelValue;
        for (int i = 0; i < pixels.length && !Thread.currentThread().isInterrupted(); i++) {
            int pixel = pixels[i];
            Color.colorToHSV(pixel, hsv);
            if (hsv[0] >= 20 && hsv[0] <= 40 && hsv[1] > 0.6 && hsv[2] > 0.2) {
                pixelValue = 1;
            } else {
                pixelValue = 0;
            }
            matchFactor += pixelValue;
            //pixelValue *= 255;
            //matchImagePixels[i] = Color.argb(255, (int) pixelValue, (int) pixelValue, (int) pixelValue);
        }
        matchFactor /= stackRegion.width() * stackRegion.height();

        //Bitmap matchImage = Bitmap.createBitmap(matchImagePixels, scanRegion.width(), scanRegion.height(), Bitmap.Config.ARGB_8888);
        //saveBitmap(matchImage, "matchImage");
        //matchImage.recycle();

        RobotLog.ii(TAG, "matchFactor: %f", matchFactor);

        if (matchFactor > fourRingThreshold) {
            return StackHeight.FOUR_RINGS;
        } else if (matchFactor > oneRingThreshold) {
            return StackHeight.ONE_RING;
        } else {
            return StackHeight.ZERO_RINGS;
        }
    }

    private void ensureRectIsWithinFrame(Rect r, int width, int height) {
        if (r.top < 0) {
            r.top = 0;
        }
        if (r.left < 0) {
            r.left = 0;
        }
        if (r.right > width) {
            r.right = width;
        }
        if (r.bottom > height) {
            r.bottom = height;
        }
    }

    public boolean saveFrame() {
        RobotLog.ii(TAG, "saveFrame()");
        final ContinuationSynchronizer<Boolean> synchronizer = new ContinuationSynchronizer<>();
        vuforia.getFrameOnce(Continuation.createTrivial(new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap == null) {
                    RobotLog.ee(TAG, "Got a null bitmap from convertFrameToBitmap(), aborting");
                    synchronizer.finish(false);
                    return;
                }

                synchronizer.finish(saveBitmap(bitmap, "frame"));
            }
        }));

        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        return synchronizer.getValue();
    }

    public boolean saveFrameCroppedToStackRegion() {
        RobotLog.ii(TAG, "saveFrameCroppedToStackRegion()");
        final ContinuationSynchronizer<Boolean> synchronizer = new ContinuationSynchronizer<>();
        vuforia.getFrameOnce(Continuation.createTrivial(new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap == null) {
                    RobotLog.ee(TAG, "Got a null bitmap from convertFrameToBitmap(), aborting");
                    synchronizer.finish(false);
                    return;
                }

                ensureRectIsWithinFrame(stackRegion, bitmap.getWidth(), bitmap.getHeight());

                int[] pixels = new int[stackRegion.width() * stackRegion.height()];
                bitmap.getPixels(
                        pixels,
                        0, stackRegion.width(),
                        stackRegion.left, stackRegion.top,
                        stackRegion.width(), stackRegion.height()
                );
                Bitmap cropped = Bitmap.createBitmap(pixels, stackRegion.width(), stackRegion.height(), Bitmap.Config.ARGB_8888);

                synchronizer.finish(saveBitmap(cropped, "stackRegion"));
                cropped.recycle();
            }
        }));

        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        return synchronizer.getValue();
    }

    public boolean saveFrameCroppedToScanRegion() {
        RobotLog.ii(TAG, "saveFrameCroppedToScanRegion()");
        final ContinuationSynchronizer<Boolean> synchronizer = new ContinuationSynchronizer<>();
        vuforia.getFrameOnce(Continuation.createTrivial(new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap == null) {
                    RobotLog.ee(TAG, "Got a null bitmap from convertFrameToBitmap(), aborting");
                    synchronizer.finish(false);
                    return;
                }

                ensureRectIsWithinFrame(scanRegion, bitmap.getWidth(), bitmap.getHeight());

                int[] pixels = new int[scanRegion.width() * scanRegion.height()];
                bitmap.getPixels(
                        pixels,
                        0, scanRegion.width(),
                        scanRegion.left, scanRegion.top,
                        scanRegion.width(), scanRegion.height()
                );
                Bitmap cropped = Bitmap.createBitmap(pixels, scanRegion.width(), scanRegion.height(), Bitmap.Config.ARGB_8888);

                synchronizer.finish(saveBitmap(cropped, "scanRegion"));
                cropped.recycle();
            }
        }));

        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        return synchronizer.getValue();
    }

    private boolean saveBitmap(Bitmap bitmap, String name) {
        File path = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM), name + ".png");

        try (FileOutputStream fos = new FileOutputStream(path)) {
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, fos);
            return true;
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "IOException while saving bitmap");
            return false;
        }
    }
}
