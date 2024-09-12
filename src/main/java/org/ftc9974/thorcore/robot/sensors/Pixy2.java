package org.ftc9974.thorcore.robot.sensors;

import android.util.Size;

import androidx.annotation.IntRange;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteMultipleBytesCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.internal.CommunicationException;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

@I2cDeviceType
@DeviceProperties(xmlTag = "Pixy2", name = "Pixy2", description = "Pixy2 CMUcam5 Vision Camera", compatibleControlSystems = {ControlSystem.REV_HUB})
public class Pixy2 extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, Optional<LynxModule>> {

    private static final String TAG = "Pixy2";

    public enum PacketType {
        GET_VERSION(14),
        GET_RESOLUTION(12),
        SET_CAMERA_BRIGHTNESS(16),
        SET_SERVOS(18),
        SET_RGB_LED(20),
        SET_LAMP(22),
        GET_FPS(24),

        // Color Connected Components
        GET_BLOCKS(32),

        // Line Tracking
        GET_MAIN_FEATURES(48),
        SET_MODE(54),
        SET_NEXT_TURN(58),
        SET_DEFAULT_TURN(60),
        SET_VECTOR(56),
        REVERSE_VECTOR(62),

        // Video
        GET_RGB(112);

        private final byte bVal;

        PacketType(int bVal) {
            this.bVal = uint2Byte(bVal);
        }
    }

    public static class Version {
        public int hardwareVersion, firmwareBuild;
        public int majorFirmware, minorFirmware;
        public String firmwareType;

        private Version(byte[] payload) {
            hardwareVersion = byte2Uint(payload[0]);
            hardwareVersion |= byte2Uint(payload[1]) << 8;
            majorFirmware = byte2Uint(payload[2]);
            minorFirmware = byte2Uint(payload[3]);
            firmwareBuild = byte2Uint(payload[4]);
            firmwareBuild |= byte2Uint(payload[5]) << 8;
            firmwareType = new String(payload, 6, payload.length - 6);
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(Locale.getDefault(),
                    "HW: %d FW: %d.%d.%d/%s",
                    hardwareVersion, majorFirmware, minorFirmware, firmwareBuild, firmwareType);
        }
    }

    public static class Block {
        public @IntRange(from = 0, to = 255) int signature;
        public @IntRange(from = 0, to = 315) int x;
        public @IntRange(from = 0, to = 207) int y;
        public @IntRange(from = 0, to = 316) int width;
        public @IntRange(from = 0, to = 208) int height;
        public @IntRange(from = -180, to = 180) int angle; // only works with color codes
        public @IntRange(from = 0, to = 255) int trackingIndex;
        // unfortunately, due to a firmware bug, we're not able to get age of the block.

        private Block(byte[] payload) {
            ByteBuffer buf = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN);
            signature = buf.getShort();
            x = buf.getShort();
            y = buf.getShort();
            width = buf.getShort();
            height = buf.getShort();
            angle = buf.getShort();
            trackingIndex = byte2Uint(buf.get());
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(Locale.getDefault(),
                    "sig: %d x: %d y: %d w: %d h: %d angle: %d idx: %d",
                    signature, x, y, width, height, angle, trackingIndex);
        }
    }

    private LynxModule parent;
    private int bus;

    public Pixy2(I2cDeviceSynch device) {
        super(device, true, Optional.empty());

        deviceClient.setI2cAddress(I2cAddr.create7bit(0x54));

        super.registerArmingStateCallback(false);
        deviceClient.engage();
    }

    public boolean initialize(LynxModule module, int bus) {
        this.bus = bus;
        return initialize(Optional.of(module));
    }

    @Override
    protected boolean internalInitialize(@NonNull Optional<LynxModule> optionalModule) {
        if (!optionalModule.isPresent()) {
            // apparently internalInitialize() gets called at least twice: once when the object is
            // created, and once when the user calls initialize() in the OpMode.
            return false;
        }
        parent = optionalModule.get();
        parameters = optionalModule;
        return true;
    }

    public void setI2cAddress(int address) {
        deviceClient.setI2cAddress(I2cAddr.create7bit(address));
    }

    // API methods

    public @Nullable Version getPixyVersion() {
        writePacket(PacketType.GET_VERSION);
        byte[] payload = readPacket();
        if (payload == null) return null;
        return new Version(payload);
    }

    public @Nullable Size getResolution() {
        writePacket(PacketType.GET_RESOLUTION);
        byte[] payload = readPacket();
        if (payload == null) return null;
        int x = byte2Uint(payload[0]);
        x |= byte2Uint(payload[1]) << 8;
        int y = byte2Uint(payload[2]);
        y |= byte2Uint(payload[3]) << 8;
        return new Size(x, y);
    }

    public void setCameraBrightness(@IntRange(from = 0, to = 255) int brightness) {
        writePacket(PacketType.SET_CAMERA_BRIGHTNESS, new byte[] {uint2Byte(brightness)});
        // eat ACK packet
        readPacket();
    }

    public void setServos(@IntRange(from = 0, to = 511) int servo0,
                          @IntRange(from = 0, to = 511) int servo1) {
        byte[] payload = new byte[4];
        payload[0] = uint2Byte(servo0 & 0xff);
        payload[1] = uint2Byte(servo0 >> 8);
        payload[2] = uint2Byte(servo1 & 0xff);
        payload[3] = uint2Byte(servo1 >> 8);
        writePacket(PacketType.SET_SERVOS, payload);
        // eat ACK packet
        readPacket();
    }

    public void setLED(@IntRange(from = 0, to = 255) int r,
                       @IntRange(from = 0, to = 255) int g,
                       @IntRange(from = 0, to = 255) int b) {
        byte[] data = {
                uint2Byte(r),
                uint2Byte(g),
                uint2Byte(b)
        };
        writePacket(PacketType.SET_RGB_LED, data);
        // eat the ACK packet
        readPacket();
    }

    public void setLamp(boolean upper, boolean lower) {
        byte[] data = {
                (byte) ((upper) ? 1 : 0),
                (byte) ((lower) ? 1 : 0)
        };
        writePacket(PacketType.SET_LAMP, data);
        // eat the ACK packet
        readPacket();
    }

    public int getFPS() {
        writePacket(PacketType.GET_FPS);
        byte[] payload = readPacket();
        if (payload == null) return -1;
        return payload[0] + (payload[1] << 8) + (payload[2] << 16) + (payload[3] << 24);
    }

    public List<Block> getBlocks(@IntRange(from = 0, to = 255) int sigmap,
                                           @IntRange(from = 0, to = 18) int maxBlocks) {
        byte[] payload = new byte[] {
                uint2Byte(sigmap),
                uint2Byte(maxBlocks)
        };
        writePacket(PacketType.GET_BLOCKS, payload);

        // the payload length of this response can exceed the 100-byte maximum read size of the rev
        // hub I2C stack. for that reason, we need to be a bit more hands-on here than the other API
        // calls. first, we read the header.
        byte[] packetHeader = readMultipleBytes(5);
        int dataLength = byte2Uint(packetHeader[3]);
        // unfortunately, dataLength maxes out at 255. that's only enough for 18 blocks. blocks are
        // 14 bytes each, so to find the number of blocks we'll be processing, divide dataLength by
        // 14, using integer division to round down.
        int numBlocks = dataLength / 14;
        List<Block> blocks = new ArrayList<>(numBlocks);
        for (int i = 0; i < numBlocks; i++) {
            // ask for 13 bytes. 14 will actually be read, with the 14th lost to the firmware bug.
            payload = readMultipleBytes(13);
            blocks.add(new Block(payload));
        }

        // make sure we clean up the rest of the payload, if any. if everything is working properly,
        // this shouldn't ever happen. however, if it was to happen and this check wasn't here, it
        // would break *everything*.
        int remaining = dataLength % 14;
        if (remaining > 0) {
            readMultipleBytes(remaining);
        }

        return blocks;
    }

    // Internal methods

    private void writePacket(PacketType packetType) {
        writePacket(packetType, null);
    }

    private void writePacket(PacketType packetType, @Nullable byte[] data) {
        byte dataLength = data == null ? 0 : uint2Byte(data.length);
        byte[] packet = new byte[4 + dataLength];
        // todo this was working with just normal casts to byte. make sure this still works
        packet[0] = uint2Byte(0xae);
        packet[1] = uint2Byte(0xc1);
        packet[2] = packetType.bVal;
        packet[3] = dataLength;
        if (dataLength > 0) {
            System.arraycopy(data, 0, packet, 4, dataLength);
        }
        writeMultipleBytes(packet);
    }

    private @Nullable byte[] readPacket() {
        // the actual header is 6 bytes long. however, due to a rev firmware bug, the rev hub will
        // actually read one more byte than it's supposed to and discards the byte. luckily, the
        // last byte in the header is the high 8 bits of the checksum. as long as we have the lower
        // 8 bits of the checksum, we can still do a pretty good job of error checking. since the
        // checksum is just the sum of all the payload bytes, the upper bits don't really change
        // much. we can just check the lower byte of the checksum. the only way that can fail is if
        // the errors in the payload add up to exactly 256. it's not 100% perfect, but it's better
        // than nothing.
        byte[] packetHeader = readMultipleBytes(5);
        int dataLength = byte2Uint(packetHeader[3]);
        if (dataLength > 0) {
            int checksum = byte2Uint(packetHeader[4]);
            byte[] data = readMultipleBytes(dataLength);
            int sum = 0;
            for (int i = 0; i < dataLength; i++) {
                sum += byte2Uint(data[i]);
            }
            // as previously mentioned, we only have the lower 8 bits of the checksum. thus, that's
            // the only bits we care about in the sum.
            sum &= 0xff;
            if (sum != checksum) {
                RobotLog.ee(TAG, "Checksum failure: expected 0x%04x, got 0x%04x", checksum, sum);
                //throw new RuntimeException(String.format("Checksum failure: expected %d, got %d", checksum, sum));
            }
            return data;
        }
        return null;
    }

    private byte[] readMultipleBytes(int numBytes) {
        final LynxI2cReadMultipleBytesCommand cmd = new LynxI2cReadMultipleBytesCommand(parent, bus, deviceClient.getI2cAddress(), numBytes);
        try {
            return parent.acquireI2cLockWhile(() -> {
                cmd.send();

                return pollForReadResult(numBytes);
            });
        } catch (InterruptedException e) {
            throw new CommunicationException("Pixy2", "Interrupted while reading multiple bytes", e);
        } catch (RobotCoreException e) {
            throw new CommunicationException("Pixy2", "RobotCoreException while reading multiple bytes", e);
        } catch (LynxNackException e) {
            throw new CommunicationException("Pixy2", String.format("LynxNackException while reading multiple bytes (reason: %s", e.getNack().getNackReasonCode().toString()), e);
        }
    }

    private void writeMultipleBytes(byte[] data) {
        final LynxI2cWriteMultipleBytesCommand cmd = new LynxI2cWriteMultipleBytesCommand(parent, bus, deviceClient.getI2cAddress(), data);
        try {
            parent.acquireI2cLockWhile(() -> {
                sendCommand(cmd);
                return null;
            });
        } catch (InterruptedException e) {
            RobotLog.ee(TAG, e, "Interrupted while writing multiple bytes");
        } catch (RobotCoreException e) {
            throw new CommunicationException("Pixy2", "RobotCoreException while writing multiple bytes", e);
        } catch (LynxNackException e) {
            throw new CommunicationException("Pixy2", String.format("LynxNackException while writing multiple bytes (reason: %s", e.getNack().getNackReasonCode().toString()), e);
        }
    }

    // sendCommand and pollForReadResult borrow lots of code from LynxI2cDeviceSynch and
    // LynxI2cDeviceSynchV2. i've changed the error handling, but that's about it. i'm not sure what
    // all of the NACK codes are caused by.
    private void sendCommand(LynxCommand<?> cmd) throws InterruptedException, LynxNackException {
        while (true) {
            try {
                RobotLog.vv(TAG, "Attempting to send command");
                cmd.send();
                break;
            } catch (LynxNackException e) {
                switch (e.getNack().getNackReasonCodeAsEnum()) {
                    case I2C_MASTER_BUSY:
                    case I2C_OPERATION_IN_PROGRESS:
                        Thread.sleep(3);
                        break;
                    default:
                        throw e;
                }
            }
        }
    }

    private byte[] pollForReadResult(int numBytes) {
        boolean keepTrying = true;

        while (keepTrying) {
            LynxI2cReadStatusQueryCommand readStatus = new LynxI2cReadStatusQueryCommand(parent, bus, numBytes);
            try {
                LynxI2cReadStatusQueryResponse response = readStatus.sendReceive();
                byte[] data = response.getBytes();

                StringBuilder builder = new StringBuilder();
                builder.append("Recieved data: ");
                for (byte datum : data) {
                    builder.append(String.format("0x%02x (%s)", datum, Integer.toBinaryString(byte2Uint(datum))));
                    builder.append(" ");
                }
                RobotLog.vv(TAG, builder.toString());
                if (data.length == numBytes) {
                    return data;
                }
                RobotLog.ee(TAG, "received an incorrect number of bytes (expected %d, got %d)", numBytes, data.length);
                keepTrying = false;
            } catch (LynxNackException e) {
                switch (e.getNack().getNackReasonCodeAsEnum()) {
                    case I2C_MASTER_BUSY:               // TODO: REVIEW: is this ever actually returned in this situation?
                    case I2C_OPERATION_IN_PROGRESS:
                        // We used to sleep for 3ms while waiting for the result to avoid a "busy loop", but that
                        // caused a serious performance hit over what we could get otherwise, at least on the CH.
                        // Besides, we're not *truly* busy looping, we still end up waiting for the module's response
                        // and what not.

                        //try { Thread.sleep(msBusyWait); } catch (InterruptedException ignored) { Thread.currentThread().interrupt(); }
                        continue;
                    case I2C_NO_RESULTS_PENDING:
                        // This is an internal error of some sort
                        throw new CommunicationException("Pixy2", "I2cReadStatusQuery NACK'd due to I2C_NO_RESULTS_PENDING", e);
                    default:
                        throw new CommunicationException("Pixy2", String.format("I2cReadStatusQuery NACK'd due to %s", e.getNack().getNackReasonCode()), e);
                }
            } catch (InterruptedException | RuntimeException e) {
                throw new CommunicationException("Pixy2", "Received an error while polling for read result", e);
            }
        }
        // this happens about once every 20 calls to pollForReadResult(), and i have no clue why.
        // according to the logs, it only happens when the ReadStatusQuery command returns an empty
        // byte array.
        throw new CommunicationException("Pixy2", "Somehow broke out of polling loop without receiving data");
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Pixy2 CMUCam5";
    }

    private static int byte2Uint(byte b) {
        // java uses signed integers for everything. this means that if you read an unsigned byte
        // into a java byte and the unsigned byte is greater than 0x7f, java will interpret it as a
        // negative number. this code corrects for that.
        if (b < 0) return b + 256;
        else return b;
    }

    private static byte uint2Byte(int i) {
        if (i > Byte.MAX_VALUE) return (byte) (i - 256);
        else return (byte) i;
    }
}
