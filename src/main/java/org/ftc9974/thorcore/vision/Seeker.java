package org.ftc9974.thorcore.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.ftc9974.thorcore.control.SlidingAverageFilter;
import org.ftc9974.thorcore.internal.RealizableFactory;
import org.ftc9974.thorcore.robot.sensors.USBWebcamBase;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

// this class is (probably) thread-safe, but i haven't tested it thoroughly.
public class Seeker extends USBWebcamBase {
    private static final String TAG = "Seeker";

    public static class Signature {
        private final long HIGH_THRESHOLD, LOW_THRESHOLD;
        private final double MIN_MATCH_FOR_LOCK;

        private double lastX, lastY;
        private double lastMatch;
        private final SlidingAverageFilter xFilter, yFilter;
        private boolean active;

        private final Object lock = new Object();

        public Signature(long highThresh, long lowThresh, double minMatch, int filterOrder) {
            HIGH_THRESHOLD = highThresh;
            LOW_THRESHOLD = lowThresh;
            MIN_MATCH_FOR_LOCK = minMatch;
            if (minMatch < 0) {
                throw new IllegalArgumentException("minMatch must be at least 0");
            }
            lastX = lastY = 0;
            lastMatch = -1;
            xFilter = new SlidingAverageFilter(filterOrder, lastX);
            yFilter = new SlidingAverageFilter(filterOrder, lastY);
            active = true;
        }

        public boolean hasLock() {
            synchronized (lock) {
                return lastMatch >= MIN_MATCH_FOR_LOCK;
            }
        }

        public OptionalDouble getUnfilteredX() {
            synchronized (lock) {
                if (hasLock()) return OptionalDouble.of(lastX);
                else return OptionalDouble.empty();
            }
        }

        public OptionalDouble getX() {
            synchronized (lock) {
                if (hasLock()) return OptionalDouble.of(xFilter.get());
                else return OptionalDouble.empty();
            }
        }

        public OptionalDouble getUnfilteredY() {
            synchronized (lock) {
                if (hasLock()) return OptionalDouble.of(lastY);
                else return OptionalDouble.empty();
            }
        }

        public OptionalDouble getY() {
            synchronized (lock) {
                if (hasLock()) return OptionalDouble.of(yFilter.get());
                else return OptionalDouble.empty();
            }
        }

        public double getLockStrength() {
            synchronized (lock) {
                return lastMatch / MIN_MATCH_FOR_LOCK;
            }
        }

        public void setActive(boolean isActive) {
            synchronized (lock) {
                active = isActive;
            }
        }

        public boolean isActive() {
            synchronized (lock) {
                return active;
            }
        }

        private void update(double newX, double newY, double newMatch) {
            synchronized (lock) {
                lastMatch = newMatch;
                if (hasLock()) {
                    xFilter.update(newX);
                    yFilter.update(newY);
                    lastX = newX;
                    lastY = newY;
                }
            }
        }
    }

    // todo replace these with atomics
    private final Set<Signature> signatures;
    private Optional<Signature> shownSignature;
    private final Object lock = new Object();

    private final float numPixels;
    private final NativeImageByteBuffer xMask, yMask;

    private final AtomicBoolean initialized = new AtomicBoolean(false);

    @RealizableFactory
    public Seeker(String name, HardwareMap hw) throws IOException {
        super(name, hw);

        if (!NEONVision.supportsNeonAcceleration()) {
            shutdown();
            throw new UnsupportedOperationException("NEON is not supported on this robot's processor!");
        }

        signatures = new HashSet<>();

        shownSignature = Optional.empty();

        Size size = getFrameSize();
        if (size == null) {
            shutdown();
            throw new IOException("Unable to get frame size");
        }
        numPixels = size.getHeight() * size.getWidth();
        xMask = new NativeImageByteBuffer(size.getWidth(), size.getHeight());
        yMask = new NativeImageByteBuffer(size.getWidth(), size.getHeight());

        int center = size.getWidth() / 2;
        double xScale = 0xff / (double) size.getWidth();
        double yScale = 0xff / (double) size.getHeight();
        for (int y = 0; y < size.getHeight(); y++) {
            for (int x = 0; x < size.getWidth(); x++) {
                byte value = (byte) MathUtilities.constrain((x - center) * xScale, Byte.MIN_VALUE, Byte.MAX_VALUE);
                xMask.buffer.put(xMask.coordToIndex(x, y), value);
                value = (byte) MathUtilities.constrain((y - center) * yScale * -1, Byte.MIN_VALUE, Byte.MAX_VALUE);
                yMask.buffer.put(yMask.coordToIndex(x, y), value);
            }
        }

        RobotLog.dd(TAG, "Seeker initialized with following parameters: resolution=(%d, %d), fps=%d", size.getWidth(), size.getHeight(), getFps());
        initialized.set(true);
    }

    public void registerSignatures(Signature... signatures) {
        synchronized (lock) {
            this.signatures.addAll(Arrays.asList(signatures));
        }
    }

    public boolean registerSignature(Signature signature) {
        synchronized (lock) {
            return signatures.add(signature);
        }
    }

    public boolean unregisterSignature(Signature signature) {
        synchronized (lock) {
            return signatures.remove(signature);
        }
    }

    public void showSignature(Signature signature) {
        synchronized (lock) {
            shownSignature = Optional.of(signature);
        }
    }

    public void hideSignatures() {
        synchronized (lock) {
            shownSignature = Optional.empty();
        }
    }

    private void updateSignature(Signature signature, CameraFrame frame) {
        if (!signature.isActive()) return;
        float xSum = (float) NEONVision.processYUY2WithMask(
                frame.getImageBuffer(), frame.getImageSize(),
                signature.HIGH_THRESHOLD, signature.LOW_THRESHOLD,
                xMask.getPointer()
        );
        float ySum = (float) NEONVision.processYUY2WithMask(
                frame.getImageBuffer(), frame.getImageSize(),
                signature.HIGH_THRESHOLD, signature.LOW_THRESHOLD,
                yMask.getPointer()
        );
        float numMatch = (float) NEONVision.processYUY2(
                frame.getImageBuffer(), frame.getImageSize(),
                signature.HIGH_THRESHOLD, signature.LOW_THRESHOLD
        );

        // here we make use of Java's reference semantics. since all non-primitive objects in Java
        // are references, we can update the object without "owning" the variable it lives in.
        signature.update(
                xSum / numMatch,
                ySum / numMatch,
                numMatch / numPixels
        );
    }

    @Override
    protected void onNewFrame(CameraFrame frame) {
        if (!initialized.get()) return;
        Optional<Signature> signature;
        synchronized (lock) {
            signature = shownSignature;
        }

        if (signature.isPresent()) {
            // ideally, this buffer would be allocated once then reused.
            // however, there doesn't seem to be a way to copy the frame into an existing buffer,
            // and java doesn't give us low-enough-level access to just copy between the buffers in memory.
            final CameraFrame copy = frame.copy();
            TimingUtilities.runOnSeparateThread(() -> {
                synchronized (lock) {
                    signatures.forEach(s -> updateSignature(s, copy));
                }
                copy.releaseRef();
            });
            NEONVision.processYUY2ForDisplay(
                    frame.getImageBuffer(), frame.getImageSize(),
                    signature.get().HIGH_THRESHOLD, signature.get().LOW_THRESHOLD
            );
        } else {
            synchronized (lock) {
                signatures.forEach(s -> updateSignature(s, frame));
            }
        }
    }
}
