package org.ftc9974.thorcore.vision;

import android.graphics.Rect;

import org.ftc9974.thorcore.NativeCodeLoader;

import java.nio.ByteBuffer;

// due to the use of direct ByteBuffers, these are expensive to create and destroy. Instantiate these
// once and save them as constants, if possible.
public class NativeImageByteBuffer {

    public static final byte BT601_BLACK = 16;
    public static final byte BT601_WHITE = -21; // 0xeb, equal to 235 as an unsigned byte

    static {
        // ensure the native library is loaded
        NativeCodeLoader.load();
    }

    public final ByteBuffer buffer;
    private final int width, height;

    public NativeImageByteBuffer(int width, int height) {
        this.width = width;
        this.height = height;
        buffer = ByteBuffer.allocateDirect(width * height);
        setAll((byte) 0);
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public long getPointer() {
        // pass the buffer directly to JNI so the C++ code doesn't have to go through the song and
        // dance (and performance hit) of accessing the field through calls to the JNIEnv pointer.
        return nativeGetPointer(buffer);
    }

    private native long nativeGetPointer(ByteBuffer buf);

    public int coordToIndex(int x, int y) {
        return y * width + x;
    }

    public void setAll(byte value) {
        for (int i = 0; i < width * height; i++) {
            buffer.put(i, value);
        }
    }

    public void drawFilledRectangle(Rect rect, byte value) {
        for (int y = rect.top; y < rect.bottom; y++) {
            for (int x = rect.left; x < rect.right; x++) {
                buffer.put(coordToIndex(x, y), value);
            }
        }
    }
}
