package org.ftc9974.thorcore.robot.sensors;

import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.LastKnown;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.lang3.RandomUtils;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
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
import org.ftc9974.thorcore.TransceiverDoubleBuffer;
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

public class USBWebcamBase implements CameraStreamSource {

    private static final String TAG = "USBWebcamBase";

    protected static final int FRAME_FORMAT = ImageFormat.YUY2;

    private static final LinkedList<Bitmap> STANDBY_IMAGES;

    static {
        Resources res = AppUtil.getInstance().getActivity().getResources();
        int[] imageIds = {
                R.drawable.camera_stream_standby0,
                R.drawable.camera_stream_standby1,
                R.drawable.camera_stream_standby2,
                R.drawable.camera_stream_standby3,
                R.drawable.camera_stream_standby4
        };
        STANDBY_IMAGES = new LinkedList<>();
        for (int imageId : imageIds) {
            Bitmap bitmap = BitmapFactory.decodeResource(res, imageId);
            if (bitmap != null) {
                STANDBY_IMAGES.add(bitmap);
            }
        }
        RobotLog.vv(TAG, "Loaded %d standby images", STANDBY_IMAGES.size());
    }

    protected final WebcamName cameraName;
    protected Camera camera;
    protected CameraCaptureSession session;
    private Size frameSize;
    private int fps;

    private final Handler handler;

    private boolean closed, streaming;

    private final TransceiverDoubleBuffer<CameraFrame> doubleBuffer;

    // constructors aren't inherited, so this has to be re-declared in every subclass
    @RealizableFactory
    public USBWebcamBase(String name, HardwareMap hw) throws IOException {
        this(hw.get(WebcamName.class, name));
    }

    public USBWebcamBase(WebcamName webcamName) throws IOException {
        CameraManager cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = webcamName;
        handler = CallbackLooper.getDefault().getHandler();

        doubleBuffer = new TransceiverDoubleBuffer<>();
        doubleBuffer.setCleanupHandler((frame) -> {
            if (frame != null) {
                frame.releaseRef();
            }
        });

        try {
            RobotLog.vv(TAG, "Attempting to open camera \"%s\"", cameraName.getDeviceName());
            camera = cameraManager.requestPermissionAndOpenCamera(new Deadline(10, TimeUnit.SECONDS), cameraName, null);
            if (camera == null) {
                throw new IOException(String.format("Unable to open camera \"%s\". This may be due to permission errors or a faulty USB connection.", cameraName.getDeviceName()));
            }

            final CameraCharacteristics characteristics = cameraName.getCameraCharacteristics();
            frameSize = characteristics.getDefaultSize(FRAME_FORMAT);
            fps = characteristics.getMaxFramesPerSecond(FRAME_FORMAT, frameSize);
            RobotLog.vv(TAG, "Request configuration: format: 0x%x frameSize: %s fps: %d", FRAME_FORMAT, frameSize.toString(), fps);

            // this is kind of a janky way of synchronizing and handling errors, but it works
            final ContinuationSynchronizer<CameraException> synchronizer = new ContinuationSynchronizer<>();
            camera.createCaptureSession(Continuation.create(handler, new CameraCaptureSession.StateCallback() {
                @Override
                public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        final CameraCaptureRequest request = camera.createCaptureRequest(FRAME_FORMAT, frameSize, fps);
                        session.startCapture(request, (captureSession, captureRequest, cameraFrame) -> {
                            // the double buffer is set to automatically call releaseRef() when it overwrites an old frame
                            onNewFrame(cameraFrame);
                            doubleBuffer.set(cameraFrame.copy());
                        }, Continuation.create(handler, (captureSession, cameraCaptureSequenceId, lastFrameNumber) -> {
                            RobotLog.dd(TAG, "Capture sequence completed: id: %s frame: %d", cameraCaptureSequenceId.toString(), lastFrameNumber);
                        }));
                        USBWebcamBase.this.session = session;
                        synchronizer.finish(null);
                        RobotLog.ii(TAG, "Successfully opened camera \"%s\"", cameraName.getDeviceName());
                    } catch (CameraException e) {
                        synchronizer.finish(e);
                    }
                }

                @Override
                public void onClosed(@NonNull CameraCaptureSession session) {
                    RobotLog.dd(TAG, "Camera capture session closed");
                }
            }));

            synchronizer.await();
            if (synchronizer.getValue() != null) {
                throw synchronizer.getValue();
            }
        } catch (CameraException e) {
            String message = String.format(Locale.getDefault(), "Unable to initialize camera \"%s\": %s", cameraName.getDeviceName(), e.getMessage());
            RobotLog.ee(TAG, e, message);
            shutdown();
            throw new IOException(message, e);
        } catch (InterruptedException e) {
            RobotLog.ee(TAG, e, "Interrupted waiting for camera to open");
            shutdown();
        }
    }

    // note: either make this run fast or grab a copy of the frame before doing heavy number-crunching.
    // it's called directly from the frame receiver thread, so the frame might get freed if 2 more frames
    // arrive before you're done processing.
    protected void onNewFrame(CameraFrame frame) {

    }

    public void startStreaming() {
        streaming = true;
        CameraStreamServer.getInstance().setSource(this);
    }

    public boolean isStreaming() {
        return streaming;
    }

    public boolean isConnected() {
        return session != null;
    }

    public boolean isClosed() {
        return closed;
    }

    // always make sure you releaseRef() when done.
    public @Nullable CameraFrame getLastFrame() {
        CameraFrame ret = doubleBuffer.get();
        if (ret == null) {
            return null;
        }
        return ret.copy();
    }

    public @Nullable CameraFrame getLastFrameDirect() {
        return doubleBuffer.get();
    }

    // easiest to use API method, but slowest
    public @Nullable Bitmap getLastFrameAsBitmap() {
        doubleBuffer.lockForRead();
        CameraFrame frame = getLastFrameDirect();
        if (frame == null) return null;
        Bitmap ret = frameToBitmap(frame);
        doubleBuffer.releaseLock();
        return ret;
    }

    public WebcamName getWebcamName() {
        return cameraName;
    }

    public int getFormat() {
        return FRAME_FORMAT;
    }

    public @Nullable Size getFrameSize() {
        return frameSize;
    }

    public int getFps() {
        return fps;
    }

    public void shutdown() {
        // this can be called multiple times if needed

        if (session != null) {
            session.close();
            session = null;
        }
        if (camera != null) {
            camera.close();
            camera = null;
        }
        if (streaming) {
            CameraStreamServer.getInstance().setSource(null);
            streaming = false;
        }
        closed = true;
    }

    public void stopStreaming() {
        if (streaming) {
            CameraStreamServer.getInstance().setSource(null);
            streaming = false;
        }
    }

    @Override
    public void finalize() {
        shutdown();
    }

    @Override
    public void getFrameBitmap(Continuation<? extends org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap>> continuation) {
        RobotLog.dd(TAG, "Sending frame to camera stream");
        if (doubleBuffer.get() != null) {
            // we have an image. we may send a different image when the continuation is actually ran,
            // but that doesn't really matter too much.
            continuation.dispatch((consumer) -> consumer.accept(getLastFrameAsBitmap()));
        } else if (!STANDBY_IMAGES.isEmpty()) {
            // out of paranoia, make sure we actually *have* a standby image before sending it
            RobotLog.dd(TAG, "Sending standby image");
            continuation.dispatch((consumer) -> consumer.accept(STANDBY_IMAGES.get(RandomUtils.nextInt(0, STANDBY_IMAGES.size()))));
        } else {
            // failsafe: return a blank image
            RobotLog.dd(TAG, "Failsafe: sending blank image");
            continuation.dispatch((consumer) -> consumer.accept(Bitmap.createBitmap(128, 128, Bitmap.Config.ARGB_8888)));
        }
    }

    public static Bitmap frameToBitmap(CameraFrame frame) {
        Bitmap ret = Bitmap.createBitmap(frame.getSize().getWidth(), frame.getSize().getHeight(), Bitmap.Config.ARGB_8888);
        frame.copyToBitmap(ret);
        return ret;
    }
}
