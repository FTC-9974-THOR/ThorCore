package org.ftc9974.thorcore.robot.sensors;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.hardware.usb.UsbDeviceConnection;
import android.util.Size;

import androidx.annotation.ColorInt;
import androidx.annotation.IntRange;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.ftc9974.thorcore.TransceiverDoubleBuffer;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.util.OpModeUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;
import org.jetbrains.annotations.Nullable;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Class for interfacing with a Pixy2 over USB.
 *
 * To use this, plug the Pixy into a USB port on the Control Hub and query {@link Pixy2USBManager} for
 * a list of connected Pixies.
 *
 * Per FTC Game Manual Part 1, Rule RE12.a: "Compatible sensors from any manufacturer may be
 * connected to the REV Expansion Hub or REV Control Hub."
 * The rules don't specify *how* sensors are connected, so USB is fair game. However, this is only
 * valid on Control Hubs, so phones are unfortunately out of luck (for now; I2C is still in the works).
 *
 * Per FTC Game Manual Part 1, Rule RE14.a: "Self-contained video recording devices (GoPro or similar)
 * are allowed providing they are used only for non-functional post-Match viewing and the wireless
 * capability is turned off. Approved self-contained video cameras must be powered by an internal
 * battery (as supplied by the manufacturer)."
 * The Pixy is not a recording device.
 *
 * Per FTC Game Manual Part 1, Rule RE14.b: "UVC Compatible Cameras are allowed for computer
 * vision-related tasks. It is recommended that UVC Compatible Cameras be connected directly to a
 * REV Control Hub, or through a powered USB hub that is in turn connected to an Android Device
 * Robot Controller through an OTG adapter."
 * The Pixy is not a USB Camera, nor is it UVC compliant. The Pixy's USB class id field identifies
 * it as a unique class, so it is quite literally not a USB camera.
 *
 * Per FTC Game Manual Part 1, Rule RE13.d.ii: Other than internal batteries, the only allowed power
 * sources for lights are "REV Expansion Hub or REV Control Hub Motor-control ports, spare XT30 ports,
 * 5V auxiliary power ports, and I2C sensor ports."
 * Don't turn on the lamps in competition.
 *
 * Internally, this uses the JNI to interface with libpixyusb2, the library that comes with the Pixy.
 * USB permissions and connections are handled here in Javaland, but actually communicating with the
 * Pixy is handled in C++.
 *
 * This could definitely be done entirely in Java with Android's USB API, but that would require
 * reverse engineering the Pixy's Chirp protocol. Besides, using native code allows for updates to
 * libpixyusb2 to be easily integrated.
 *
 * @see Pixy2USBManager
 * @see org.ftc9974.thorcore.samples.SamplePixy2USB
 */
public class Pixy2USB implements CameraStreamSource, OpModeManagerNotifier.Notifications {

    private static final String TAG = "Pixy2USB";

    /**
     * Class for holding version information of a Pixy.
     */
    public static class Version {
        /**
         * Hardware revision
         */
        public final int hardware;

        /**
         * Major firmware revision
         */
        public final short firmwareMajor;

        /**
         * Minor firmware version
         */
        public final short firmwareMinor;

        /**
         * Firmware build code
         */
        public final int firmwareBuild;

        /**
         * Firmware type
         */
        public final String firmwareType;

        private Version(int hardware, short firmwareMajor, short firmwareMinor, int firmwareBuild, String firmwareType) {
            this.hardware = hardware;
            this.firmwareMajor = firmwareMajor;
            this.firmwareMinor = firmwareMinor;
            this.firmwareBuild = firmwareBuild;
            this.firmwareType = firmwareType;
        }

        /**
         * Constructs a human-readable representation of this version.
         *
         * @return String representation
         */
        @Override
        public @NonNull String toString() {
            return String.format(Locale.getDefault(), "%d.%d.%d hw %d \"%s\"", firmwareMajor, firmwareMinor, firmwareBuild, hardware, firmwareType);
        }
    }

    /**
     * Class for representing color connected blocks detected by Pixies.
     */
    public static class Block {
        /**
         * Index of the color signature associated with this block. Ranges from 0 to 7.
         */
        @IntRange(from = 0, to = 7)
        public final int signature;

        /**
         * Position of this block in frame.
         *
         * I believe this is from the center of the block, with the origin being at the top-left
         * corner of the frame, x increasing right and y increasing down. However, I have not yet
         * verified this.
         */
        // todo check coordinate system
        public final Vector2 position;

        /**
         * Size of this block's bounding box, in pixels
         */
        public final Size size;

        /**
         * Angle of this block
         */
        public final double angle;

        /**
         * Tracking index of this block. This index is a number from 0 to 255, assigned by the Pixy
         * on the first frame this block is detected. This index persists until the block is no longer
         * detected.
         *
         * Use this field for tracking specific blocks.
         */
        @IntRange(from = 0, to = 255)
        public final int trackingIndex;

        /**
         * Number of consecutive frames this block has been tracked. Ranges from 0 to 255. After 255
         * frames, this field stops increasing.
         */
        @IntRange(from = 0, to = 255)
        public final int age;

        private Block(int signature, int x, int y, int width, int height, short angle, short trackingIndex, short age) {
            this.signature = signature;
            position = new Vector2(x, y);
            size = new Size(width, height);
            this.angle = angle;
            this.trackingIndex = trackingIndex;
            this.age = age;
        }
    }

    /**
     * Represents programs to run on the Pixy.
     */
    public enum Program {
        /**
         * Default operation mode of the Pixy. Tracks blocks of similarly-colored pixels.
         */
        COLOR_CONNECTED_COMPONENTS,
        /**
         * Used for line following.
         */
        LINE_TRACKING,
        /**
         * Disables vision processing, making only raw video available.
         *
         * (Note: raw video is not yet supported.)
         */
        VIDEO
    }

    private final UsbDeviceConnection connection;
    private final int fileDescriptor;
    private boolean connected, streaming;

    private final TransceiverDoubleBuffer<Bitmap> bitmapBuffer;
    private final AtomicBoolean alreadyProcessingBitmap;

    Pixy2USB(UsbDeviceConnection connection) {
        this.connection = connection;
        fileDescriptor = connection.getFileDescriptor();
        bitmapBuffer = new TransceiverDoubleBuffer<>();
        alreadyProcessingBitmap = new AtomicBoolean(false);
        OpModeUtilities.registerListener(this);
    }

    void open() throws OutOfMemoryError, IllegalAccessError, IOException {
        nativeOpen(fileDescriptor);
        // make sure the lamps are turned off
        setLamps(false, false);
        // blink the LED green to indicate successful connection
        setLED(Color.GREEN);
        TimingUtilities.runAfterDelay(() -> setLED(Color.BLACK), 200);

        Size resolution = getResolution();
        bitmapBuffer.set(Bitmap.createBitmap(resolution.getWidth(), resolution.getHeight(), Bitmap.Config.ARGB_8888));
        bitmapBuffer.set(Bitmap.createBitmap(resolution.getWidth(), resolution.getHeight(), Bitmap.Config.ARGB_8888));

        connected = true;
    }

    void close() {
        connected = false;
        stopCameraStream();
        nativeClose(fileDescriptor);
        connection.close();
        OpModeUtilities.unregisterListener(this);
    }

    /**
     * Gets the serial string of this Pixy.
     *
     * I believe this is just "DEMO 0.0" on all Pixies.
     *
     * @return serial string
     */
    public @Nullable String getSerial() {
        return connection.getSerial();
    }

    /**
     * Gets the version information of this Pixy.
     *
     * Returns null on I/O failure.
     *
     * @return version, or null on failure.
     */
    public @Nullable Version getVersion() {
        try {
            return nativeGetVersion(fileDescriptor);
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "IOException getting version");
        }
        return null;
    }

    /**
     * Sets the active program of this Pixy.
     *
     * Currently this wrapper only supports color connected components.
     *
     * @param program program to run
     * @return true on success, false on failure
     */
    public boolean setProgram(Program program) {
        try {
            nativeSetProgram(fileDescriptor, program.ordinal());
            return true;
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "IOException setting program");
        }
        return false;
    }

    /**
     * Sets the state of this Pixy's lamps.
     *
     * Remember to turn these off once you're done with them, as they will stay on until commanded to
     * turn off - regardless of if there's an OpMode running or not.
     *
     * @param upper whether or not the upper lamps should be illuminated
     * @param lower whether or not the lower lamp should be illuminated
     * @return true on success, false on failure
     */
    public boolean setLamps(boolean upper, boolean lower) {
        try {
            nativeSetLamps(fileDescriptor, upper, lower);
            return true;
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "IOException setting lamp states");
        }
        return false;
    }

    /**
     * Sets the color of the LED on this Pixy.
     *
     * @param r red component, 0-255 inclusive.
     * @param g green component, 0-255 inclusive.
     * @param b blue component, 0-255 inclusive.
     * @return true on success, false on failure
     */
    public boolean setLED(@IntRange(from = 0, to = 255) int r, @IntRange(from = 0, to = 255) int g, @IntRange(from = 0, to = 255) int b) {
        try {
            nativeSetLED(fileDescriptor, r, g, b);
            return true;
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "IOException setting LED state");
        }
        return false;
    }

    /**
     * Sets the color of the LED on this Pixy.
     *
     * @param color color to set the LED to, in Android ColorInt format
     * @return true on success, false on failure
     */
    public boolean setLED(@ColorInt int color) {
        return setLED(Color.red(color), Color.green(color), Color.blue(color));
    }

    /**
     * Gets the resolution of this Pixy's camera, in pixels.
     *
     * @return resolution, represented as a Size object.
     */
    public Size getResolution() {
        return nativeGetResolution(fileDescriptor);
    }

    private @Nullable Block[] internalGetBlocks() {
        Block[] blockArr = null;
        try {
            blockArr = nativeGetCCCBlocks(fileDescriptor);
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "IOException getting blocks");
        }

        return blockArr;
    }

    /**
     * Gets a list of the color connected blocks detected by this Pixy.
     *
     * Returns null on I/O failure.
     *
     * @return blocks, or null on failure
     */
    public @Nullable List<Block> getBlocks() {
        Block[] blocks = internalGetBlocks();
        if (blocks == null) {
            return null;
        }
        return Arrays.asList(blocks);
    }

    /**
     * Gets a map of color connected blocks detected by this Pixy, keyed by tracking index.
     *
     * Returns null on I/O failure.
     *
     * @return block map, or null on failure. the key of each block is the block's tracking index.
     */
    public @Nullable HashMap<Integer, Block> getBlockMap() {
        Block[] blocks = internalGetBlocks();

        if (blocks == null) {
            return null;
        }

        HashMap<Integer, Block> ret = new HashMap<>(blocks.length);
        for (Block b : blocks) {
            if (b != null) {
                ret.put((int) b.trackingIndex, b);
            }
        }

        return ret;
    }

    /**
     * Gets the color connected block with the specified tracking index.
     *
     * Returns null on I/O failure or if no such block exists.
     *
     * @param index tracking index, from 0 to 255.
     * @return block, or null on failure
     */
    public @Nullable Block getBlockByTrackingIndex(@IntRange(from = 0, to = 255) int index) {
        Block[] blocks = internalGetBlocks();

        if (blocks == null) {
            return null;
        }

        for (Block block : blocks) {
            if (block.trackingIndex == index) {
                return block;
            }
        }
        return null;
    }

    /**
     * Gets a list of color connected blocks detected by this Pixy that have a specified signature.
     *
     * Returns null on I/O failure.
     *
     * @param signature index of the signature, from 0 to 7.
     * @return blocks, or null on failure
     */
    public @Nullable List<Block> getBlocksBySignature(@IntRange(from = 0, to = 7) int signature) {
        Block[] blocks = internalGetBlocks();

        if (blocks == null) {
            return null;
        }

        List<Block> ret = new LinkedList<>();
        for (Block block : blocks) {
            if (block.signature == signature) {
                ret.add(block);
            }
        }

        return ret;
    }

    // double buffers so only one frame is ever being processed
    public @Nullable Bitmap getRawFrame() {
        if (!alreadyProcessingBitmap.get()) {
            alreadyProcessingBitmap.set(true);
            // this has to be in little endian, because the native code copies a 32-bit int into the buffer.
            // the Control Hub is a little-endian machine, so the byte order must match.
            ByteBuffer outputBuffer = ByteBuffer.allocateDirect(bitmapBuffer.get().getByteCount()).order(ByteOrder.LITTLE_ENDIAN);
            try {
                nativeGetFrame(fileDescriptor, outputBuffer);
            } catch (IOException e) {
                RobotLog.ee(TAG, e, "IOException while getting raw frame: %s", e.getMessage());
                return null;
            } catch (IndexOutOfBoundsException e) {
                // buffer size mismatch
                RobotLog.ee(TAG, e, "Internal error while getting raw frame: %s", e.getMessage());
                return null;
            }

            Bitmap freshBitmap;
            // make sure we don't flip the buffers while someone else is using them
            synchronized (bitmapBuffer) {
                bitmapBuffer.forceFlip();
                freshBitmap = bitmapBuffer.get();
                bitmapBuffer.forceFlip();
            }

            // copy into the inactive side of the buffer
            freshBitmap.copyPixelsFromBuffer(outputBuffer);
            // and then make that the active side
            bitmapBuffer.forceFlip();
            alreadyProcessingBitmap.set(false);

            outputBuffer = null;
            System.gc();

            return bitmapBuffer.get();
        } else {
            synchronized (bitmapBuffer) {
                return bitmapBuffer.get();
            }
        }
    }

    public void startCameraStream() {
        CameraStreamServer.getInstance().setSource(this);
        streaming = true;
    }

    public void stopCameraStream() {
        if (streaming) {
            CameraStreamServer.getInstance().setSource(null);
            streaming = false;
        }
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch((consumer) -> {
            Bitmap bitmap = getRawFrame();
            // draw hud
            List<Block> blocks = getBlocks();
            synchronized (bitmapBuffer) {
                if (blocks != null) {
                    Canvas canvas = new Canvas(bitmap);
                    Paint paint = new Paint();
                    paint.setColor(Color.WHITE);
                    paint.setStrokeWidth(4);
                    paint.setStyle(Paint.Style.STROKE);
                    for (Block block : blocks) {
                        int left = (int) (block.position.getX() - 0.5 * block.size.getWidth());
                        int bottom = (int) (block.position.getY() + 0.5 * block.size.getHeight());
                        canvas.drawRect(new Rect(
                                left,
                                (int) (block.position.getY() - 0.5 * block.size.getHeight()),
                                (int) (block.position.getX() + 0.5 * block.size.getWidth()),
                                bottom
                        ), paint);
                        canvas.drawText(String.format(Locale.getDefault(), "Sig: %d", block.signature), left, bottom + 10, paint);
                    }
                }
                consumer.accept(bitmap);
            }
        });
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stopCameraStream();
    }

    // each Java instance of Pixy2USB has a native counterpart, and they identify themselves by
    // their device ID when going back and forth across the JNI boundary. this is a somewhat inelegant
    // solution, as it requires having 2 registries (one in Java, one in C++). however, doing it this
    // way lets libpixyusb2 do most of the teardown when a Pixy is disconnected.

    private static native void nativeOpen(int fileDescriptor) throws OutOfMemoryError, IllegalAccessError, IOException;
    private static native void nativeClose(int fileDescriptor);

    private static native Version nativeGetVersion(int fileDescriptor) throws IOException;
    private static native void nativeSetProgram(int fileDescriptor, int programIndex) throws IOException;
    private static native void nativeSetLamps(int fileDescriptor, boolean upper, boolean lower) throws IOException;
    private static native void nativeSetLED(int fileDescriptor, int r, int g, int b) throws IOException;
    private static native Size nativeGetResolution(int fileDescriptor);

    private static native Block[] nativeGetCCCBlocks(int fileDescriptor) throws IOException;

    private static native void nativeGetFrame(int fileDescriptor, ByteBuffer outputBuffer) throws IOException;
}
