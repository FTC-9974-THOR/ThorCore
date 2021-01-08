package org.ftc9974.thorcore;

import android.os.SystemClock;

import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.lang3.ArrayUtils;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicBoolean;

public final class Telepathy {

    // Packet format:
    // Byte 0:     Header
    // Byte 1:     Start of key string
    // Byte N:     End of key string
    // Byte N + 1: Value type key
    // Byte N + 2: Start of value
    // Byte K:     End of value

    private static final String TAG = "Telepathy";

    private static ServerSocket socket;
    private static Socket client;
    private static InputStream receiveStream;
    private static OutputStream transmitStream;

    private static Thread workerThread;
    // arrays (even of primitive types) are not themselves primitive types. thus, we can use them
    // as type parameters.
    private static ConcurrentLinkedQueue<byte[]> messageQueue;
    private static AtomicBoolean workerThreadShouldStop;

    private static final byte[] HEADER = {0};
    private static final long ACK_TIMEOUT = 100;

    private static boolean initialized;
    private static AtomicBoolean connected;

    private static long lastKeepAliveTime;

    enum Type {
        STRING(0, String.class),
        BYTE(1, byte.class),
        CHAR(2, char.class),
        SHORT(3, short.class),
        INT(4, int.class),
        FLOAT(5, float.class),
        LONG(6, long.class),
        DOUBLE(7, double.class);

        private byte typeKey;
        private Class clazz;

        Type(int typeKey, Class clazz) {
            this.typeKey = (byte) typeKey;
            this.clazz = clazz;
        }

        Class classForType() {
            return clazz;
        }

        static Type forByte(byte theByte) {
            switch (theByte) {
                case 0:
                    return STRING;
                case 1:
                    return BYTE;
                case 2:
                    return CHAR;
                case 3:
                    return SHORT;
                case 4:
                    return INT;
                case 5:
                    return FLOAT;
                case 6:
                    return LONG;
                case 7:
                default:
                    return DOUBLE;
            }
        }

        static Type forClass(Class<?> clazz) {
            if (clazz.isAssignableFrom(STRING.clazz)) {
                return STRING;
            } else if (clazz.isAssignableFrom(BYTE.clazz)) {
                return BYTE;
            } else if (clazz.isAssignableFrom(CHAR.clazz)) {
                return CHAR;
            } else if (clazz.isAssignableFrom(SHORT.clazz)) {
                return SHORT;
            } else if (clazz.isAssignableFrom(INT.clazz)) {
                return INT;
            } else if (clazz.isAssignableFrom(FLOAT.clazz)) {
                return FLOAT;
            } else if (clazz.isAssignableFrom(LONG.clazz)) {
                return LONG;
            } else {
                return DOUBLE;
            }
        }
    }

    static class Message<T> {
        String key;
        Type type;
        T message;

        Message(String key, T message) {
            this.key = key;
            type = Type.forClass(message.getClass());
            this.message = message;
        }
    }

    /**
     * Initializes the Telepathy server. This should never need to be called by the user.
     * @return true if successful, false otherwise
     */
    public static boolean initialize() {
        if (socket != null) {
            RobotLog.ii(TAG, "A socket already exists, closing it");
            try {
                socket.close();
            } catch (IOException e) {
                RobotLog.ee(TAG, e, "Error closing existing socket");
            }
        }

        try {
            socket = new ServerSocket(6387);
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "Error creating ServerSocket");
            initialized = false;
            return false;
        }

        lastKeepAliveTime = -1;

        messageQueue = new ConcurrentLinkedQueue<>();
        workerThreadShouldStop = new AtomicBoolean(false);

        connected = new AtomicBoolean(false);

        workerThread = new Thread(Telepathy::doLifecycle, "TelepathyWorker");
        workerThread.start();

        initialized = true;
        return true;
    }

    public static boolean isInitialized() {
        return initialized;
    }

    private static void doLifecycle() {
        while (!workerThreadShouldStop.get()) {
            try {
                client = socket.accept();
                receiveStream = client.getInputStream();
                transmitStream = client.getOutputStream();
                connected.set(true);
            } catch (IOException e) {
                RobotLog.ee(TAG, e, "Error accepting client");
            }
            if (connected.get()) {
                RobotLog.ii(TAG, "Client connected");
                lastKeepAliveTime = -1;
                while (!workerThreadShouldStop.get()) {
                    try {
                        if (receiveStream.available() > 0) {
                            while (receiveStream.available() > 0) {
                                receiveStream.read();
                            }
                            lastKeepAliveTime = SystemClock.uptimeMillis();
                        }
                        if (lastKeepAliveTime > 0 && SystemClock.uptimeMillis() - lastKeepAliveTime > 1000) {
                            // keep-alive not detected, connection lost
                            connected.set(false);
                            RobotLog.ww(TAG, "Keep-alive timed out");
                            break;
                        }
                    } catch (IOException e) {
                        RobotLog.ee(TAG, e, "Error looking for keep-alive");
                    }
                    if (messageQueue.size() > 0) {
                        if (client != null) {
                            try {
                                byte[] message = messageQueue.poll();
                                if (message == null) {
                                    RobotLog.ee(TAG, "Somehow a null message was returned from a non-empty queue");
                                    continue;
                                }
                                transmitStream.write(message);
                            } catch (IOException e) {
                                RobotLog.ee(TAG, e, "Error writing message to client, assuming connection lost");
                                connected.set(false);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * Shuts downs the Telepathy server. Like {@link #initialize()}, the user should never need to
     * call this.
     */
    public static void shutdown() {
        if (initialized) {
            workerThreadShouldStop.set(true);
            if (workerThread.isAlive()) {
                try {
                    workerThread.join(1000);
                } catch (InterruptedException e) {
                    RobotLog.ee(TAG, e, "Interrupted during shutdown");
                    if (workerThread.isAlive() && !workerThread.isInterrupted()) {
                        workerThread.interrupt();
                    }
                } finally {
                    workerThread = null;
                    workerThreadShouldStop = null;
                }
            }
            messageQueue = null;

            if (client != null) {
                try {
                    transmitStream.flush();
                } catch (IOException e) {
                    RobotLog.ee(TAG, e, "Error flushing transmit stream for shutdown");
                } finally {
                    transmitStream = null;
                }

                try {
                    client.close();
                } catch (IOException e) {
                    RobotLog.ee(TAG, e, "Error closing client connection");
                } finally {
                    client = null;
                }
            }

            receiveStream = null;

            try {
                socket.close();
            } catch (IOException e) {
                RobotLog.ee(TAG, e, "Error closing server socket");
            } finally {
                socket = null;
            }
        }
    }

    public static boolean isConnected() {
        return connected.get();
    }

    public static void send(String key, String value) {
        sendBytes(HEADER);
        /*try {
            waitForAck();
        } catch (TimeoutException e) {
            RobotLog.ee(TAG, e, "Timeout waiting for ACK");
            return;
        }*/
        sendBytes(prepareForTransmit(key, Type.STRING, value.getBytes()));
    }

    // TODO: 11/18/18 Instead of using ByteBuffers, use bit shifts instead

    public static void send(String key, byte value) {
        sendBytes(HEADER);
        /*try {
            waitForAck();
        } catch (TimeoutException e) {
            RobotLog.ee(TAG, e, "Timeout waiting for ACK");
            return;
        }*/
        sendBytes(prepareForTransmit(key, Type.BYTE, new byte[] {value}));
    }

    public static void send(String key, char value) {
        sendBytes(HEADER);
        /*try {
            waitForAck();
        } catch (TimeoutException e) {
            RobotLog.ee(TAG, e, "Timeout waiting for ACK");
            return;
        }*/
        sendBytes(prepareForTransmit(key, Type.CHAR, new byte[] {(byte) value}));
    }

    public static void send(String key, short value) {
        sendBytes(HEADER);
        /*try {
            waitForAck();
        } catch (TimeoutException e) {
            RobotLog.ee(TAG, e, "Timeout waiting for ACK");
            return;
        }*/
        sendBytes(prepareForTransmit(key, Type.SHORT, ByteBuffer.allocate(2).putShort(value).array()));
    }

    public static void send(String key, int value) {
        sendBytes(HEADER);
        /*try {
            waitForAck();
        } catch (TimeoutException e) {
            RobotLog.ee(TAG, e, "Timeout waiting for ACK");
            return;
        }*/
        sendBytes(prepareForTransmit(key, Type.INT, ByteBuffer.allocate(4).putInt(value).array()));
    }

    public static void send(String key, float value) {
        sendBytes(HEADER);
        /*try {
            waitForAck();
        } catch (TimeoutException e) {
            RobotLog.ee(TAG, e, "Timeout waiting for ACK");
            return;
        }*/
        sendBytes(prepareForTransmit(key, Type.FLOAT, ByteBuffer.allocate(4).putFloat(value).array()));
    }

    public static void send(String key, long value) {
        sendBytes(HEADER);
        /*try {
            waitForAck();
        } catch (TimeoutException e) {
            RobotLog.ee(TAG, e, "Timeout waiting for ACK");
            return;
        }*/
        sendBytes(prepareForTransmit(key, Type.LONG, ByteBuffer.allocate(8).putLong(value).array()));
    }

    public static void send(String key, double value) {
        sendBytes(HEADER);
        /*try {
            waitForAck();
        } catch (TimeoutException e) {
            RobotLog.ee(TAG, e, "Timeout waiting for ACK");
            return;
        }*/
        sendBytes(prepareForTransmit(key, Type.DOUBLE, ByteBuffer.allocate(8).putDouble(value).array()));
    }

    // this is package private for the purpose of tests
    @SuppressWarnings({"WeakerAccess"})
    static byte[] prepareForTransmit(String key, Type type, byte[] value) {
        byte[] keyBytes = key.getBytes();
        ByteBuffer messageBuffer = ByteBuffer.allocate(keyBytes.length + 1 + 4 + value.length);
        messageBuffer.put(keyBytes);
        //messageBuffer.put((byte) 0);
        messageBuffer.put(type.typeKey);
        messageBuffer.putInt(value.length);
        messageBuffer.put(value);
        return messageBuffer.array();
    }

    @SuppressWarnings("WeakerAccess")
    static Message<?> deserializeMessage(byte[] bytes) {
        List<Byte> keyBytes = new ArrayList<>();
        for (byte aByte : bytes) {
            if (aByte < 32) {
                break;
            }
            keyBytes.add(aByte);
        }
        String key = new String(ArrayUtils.toPrimitive(keyBytes.toArray(new Byte[0])));
        ByteBuffer buffer = ByteBuffer.wrap(ArrayUtils.subarray(bytes, keyBytes.size(), bytes.length));
        switch (Type.forByte(buffer.get())) {
            case STRING:
                return new Message<>(key, new String(ArrayUtils.subarray(bytes, keyBytes.size() + 5, bytes.length)));
            case BYTE:
                buffer.getInt();
                return new Message<>(key, buffer.get());
            case CHAR:
                buffer.getInt();
                return new Message<>(key, buffer.getChar());
            case SHORT:
                buffer.getInt();
                return new Message<>(key, buffer.getShort());
            case INT:
                buffer.getInt();
                return new Message<>(key, buffer.getInt());
            case FLOAT:
                buffer.getInt();
                return new Message<>(key, buffer.getFloat());
            case LONG:
                buffer.getInt();
                return new Message<>(key, buffer.getLong());
            case DOUBLE:
                buffer.getInt();
                return new Message<>(key, buffer.getDouble());
            default:
                throw new RuntimeException("Invalid type received (this error should never happen)");
        }
    }

    private static void waitForAck() throws TimeoutException {
        long startTime = SystemClock.uptimeMillis();
        int available = 0;
        do {
            try {
                available = receiveStream.available();
            } catch (IOException e) {
                RobotLog.ee(TAG, e, "IOException while waiting for ACK");
            }
            if (SystemClock.uptimeMillis() - startTime > Telepathy.ACK_TIMEOUT) {
                throw new TimeoutException("Timeout exceeded while waiting for ACK");
            }
        } while (available < 1);
    }

    private static void sendBytes(byte[] value) {
        messageQueue.add(value);
    }
}
