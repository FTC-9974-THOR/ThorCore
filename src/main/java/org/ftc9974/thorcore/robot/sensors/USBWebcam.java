package org.ftc9974.thorcore.robot.sensors;

import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.drawable.BitmapDrawable;
import android.graphics.drawable.Drawable;
import android.os.Handler;

import androidx.annotation.DrawableRes;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.LastKnown;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.lang3.RandomUtils;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.ftc9974.thorcore.R;
import org.ftc9974.thorcore.internal.RealizableFactory;
import org.ftc9974.thorcore.util.TimingUtilities;

import java.io.IOException;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Locale;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

public class USBWebcam extends USBWebcamBase {

    private static final String TAG = "USBWebcam";

    // frame listeners are serviced as fast as possible. if a listener is still running with the
    // previous frame by the time a new one arrives, the new frame is not sent to the listener.
    private final LinkedList<Consumer<CameraFrame>> frameListeners;
    private final LinkedList<Consumer<CameraFrame>> pendingFrameRequests;
    private final Object listenerLock = new Object();

    private final ThreadPoolExecutor callbackExecutor;

    @RealizableFactory
    public USBWebcam(String name, HardwareMap hw) throws IOException {
        this(hw.get(WebcamName.class, name));
    }

    public USBWebcam(WebcamName webcamName) throws IOException {
        super(webcamName);

        frameListeners = new LinkedList<>();
        pendingFrameRequests = new LinkedList<>();

        callbackExecutor = (ThreadPoolExecutor) Executors.newCachedThreadPool();
        callbackExecutor.setRejectedExecutionHandler(new ThreadPoolExecutor.AbortPolicy());

    }

    @Override
    protected void onNewFrame(CameraFrame frame) {
        // due to how the super constructor is called in our constructor, there's a (small) possibility
        // we received a frame before the rest of the constructor executed.
        if (frameListeners == null) return;
        // if a listener was to be removed after the countdown latch was instantiated and the actual dispatch,
        // we could deadlock - hence the synchronized block.
        synchronized (listenerLock) {
            int numListeners = frameListeners.size() + pendingFrameRequests.size();
            if (numListeners > 0) {
                // limit the number of worker threads that can be spawned.
                // this keeps us from spawning a boatload of long-running threads
                // if a callback takes a long time to execute.
                // we set the limit to the number of callbacks we expect to execute, plus one to
                // release the copy of the frame
                callbackExecutor.setMaximumPoolSize(numListeners + 1);

                // make sure the frame isn't GC'ed while the listeners are using it. all callbacks
                // operate on a copy of the frame.
                final CameraFrame frameCopy = frame.copy();

                final CountDownLatch releaseLatch = new CountDownLatch(numListeners);
                for (Consumer<CameraFrame> frameListener : frameListeners) {
                    try {
                        callbackExecutor.execute(() -> {
                            try {
                                frameListener.accept(frameCopy);
                            } catch (Throwable t) {
                                RobotLog.ee(TAG, t, "Frame listener threw an error: %s", t.getMessage());
                            } finally {
                                releaseLatch.countDown();
                            }
                        });
                    } catch (RejectedExecutionException e) {
                        // if a task is rejected, pretend it's finished so
                        // the camera frame eventually gets released
                        releaseLatch.countDown();
                    }
                }
                final LinkedList<Consumer<CameraFrame>> extantFrameRequests = new LinkedList<>();
                for (Consumer<CameraFrame> pendingFrameRequest : pendingFrameRequests) {
                    try {
                        callbackExecutor.execute(() -> {
                            try {
                                pendingFrameRequest.accept(frameCopy);
                            } catch (Throwable t) {
                                RobotLog.ee(TAG, t, "Frame request threw an error: %s", t.getMessage());
                            } finally {
                                releaseLatch.countDown();
                            }
                        });
                    } catch (RejectedExecutionException e) {
                        // if a task is rejected, pretend it's finished so
                        // the camera frame eventually gets released
                        releaseLatch.countDown();

                        // frame requests have to be serviced at some point,
                        // so mark it as extant and try again with the next
                        // frame
                        extantFrameRequests.add(pendingFrameRequest);
                    }
                }
                pendingFrameRequests.clear();
                pendingFrameRequests.addAll(extantFrameRequests);

                Runnable releaseCallback = () -> {
                    try {
                        // this is a very long timeout, but if something was
                        // to break, 45 minutes is the longest i'd expect the
                        // robot to need to hold onto the frame before using it.
                        // pretty much the only way this could happen is if
                        // the robot grabs a frame on OpMode init() and then
                        // the match gets delayed.
                        // either way, an extra 2MB lying around unused isn't
                        // going to be too much of a problem.
                        releaseLatch.await(45, TimeUnit.MINUTES);
                    } catch (InterruptedException e) {
                        RobotLog.ee(TAG, e, "Interrupted waiting for frame callbacks to finish");
                    } finally {
                        if (releaseLatch.getCount() != 0) {
                            RobotLog.ww(TAG, "Camera Frame released after timeout. This is most likely an internal error.");
                        }
                        RobotLog.dd(TAG, "Releasing camera frame");
                        frameCopy.releaseRef();
                    }
                };
                try {
                    callbackExecutor.execute(releaseCallback);
                } catch (RejectedExecutionException e) {
                    // this shouldn't happen, but we have to account for it
                    // anyways. the frame has to be released at some point,
                    // after all.
                    RobotLog.ee(TAG, e, "Internal Error: Release callback was rejected: %s", e.getMessage());
                    TimingUtilities.runOnSeparateThread(releaseCallback);
                }
            }
        }
    }

    public void addNewFrameListener(Consumer<CameraFrame> callback) {
        synchronized (listenerLock) {
            frameListeners.add(callback);
        }
    }

    public void addNewFrameListeners(Collection<Consumer<CameraFrame>> callbacks) {
        synchronized (listenerLock) {
            frameListeners.addAll(callbacks);
        }
    }

    public void removeNewFrameListener(Consumer<CameraFrame> callback) {
        synchronized (listenerLock) {
            frameListeners.remove(callback);
        }
    }
}
