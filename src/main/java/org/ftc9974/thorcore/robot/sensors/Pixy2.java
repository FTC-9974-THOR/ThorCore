package org.ftc9974.thorcore.robot.sensors;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadSingleByteCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cReadStatusQueryResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteMultipleBytesCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteSingleByteCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.internal.CommunicationException;
import org.ftc9974.thorcore.internal.LynxI2cDeviceSynchEx;
import org.ftc9974.thorcore.util.StringUtilities;

import java.util.Optional;
import java.util.Set;

// todo work in progress
@I2cDeviceType
@DeviceProperties(xmlTag = "Pixy2", name = "Pixy2", description = "Pixy2 CMUcam5 Vision Camera", compatibleControlSystems = {ControlSystem.REV_HUB})
public final class Pixy2 extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, Optional<LynxModule>> {

    private static final String TAG = "Pixy2";

    private static final byte[] EMPTY_DATA = new byte[0];

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

        private byte bVal;

        PacketType(int bVal) {
            this.bVal = (byte) bVal;
        }
    }

    public class Version {
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
            byte[] firmwareString = new byte[payload.length - 6];
            System.arraycopy(payload, 6, firmwareString, 0, firmwareString.length);
            firmwareType = new String(firmwareString);
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

    // as far as I can tell, everything but the last 1 or 2 bytes from readData() are correct. this
    // causes the first letter of the firmwareType string to be corrupted.
    public Version getPixyVersion() {
        writePacket(PacketType.GET_VERSION, EMPTY_DATA);
        byte[] payload = readData();
        if (payload == null) {
            throw new CommunicationException(String.format("Pixy2 [%s]", deviceClient.getI2cAddr().toString()), "I2C Failure");
        }
        return new Version(payload);
    }

    // not working in the slightest; returns garbage data
    // maybe caused by byte being stored as two's complement?
    public Vector2 getResolution() {
        byte[] data = {0};
        writePacket(PacketType.GET_RESOLUTION, data);
        byte[] payload = readData();
        int x = byte2Uint(payload[0]);
        x |= byte2Uint(payload[1]) << 8;
        int y = byte2Uint(payload[2]);
        y |= byte2Uint(payload[3]) << 8;
        return new Vector2(x, y);
    }

    // seems to work correctly
    public void setLED(int r, int g, int b) {
        byte[] data = {
                (byte) r,
                (byte) g,
                (byte) b
        };
        writePacket(PacketType.SET_RGB_LED, data);
        readPacket();
    }

    // seems to work correctly
    public void setLamp(boolean upper, boolean lower) {
        byte[] data = {
                (byte) ((upper) ? 1 : 0),
                (byte) ((lower) ? 1 : 0)
        };
        writePacket(PacketType.SET_LAMP, data);
        readPacket();
    }

    // Internal methods

    private void writePacket(PacketType packetType, byte[] data) {
        byte dataLength = (byte) data.length;
        byte[] packet = new byte[4 + dataLength];
        packet[0] = (byte) 0xae;
        packet[1] = (byte) 0xc1;
        packet[2] = packetType.bVal;
        packet[3] = dataLength;
        if (dataLength > 0) {
            System.arraycopy(data, 0, packet, 4, dataLength);
        }
        StringBuilder builder = new StringBuilder();
        builder.append("Writing data: ");
        for (byte datum : packet) {
            builder.append(String.format("0x%02x (%s)", datum, Integer.toBinaryString(byte2Uint(datum))));
            builder.append(" ");
        }
        RobotLog.vv(TAG, builder.toString());
        writeMultipleBytes(packet);
    }

    private byte[] readPacket() {
        // packet header always seems to be read correctly
        byte[] packetHeader = readMultipleBytes(6);
        int dataLength = byte2Uint(packetHeader[3]);
        if (dataLength > 0) {
            int checksum = byte2Uint(packetHeader[4]);
            checksum |= byte2Uint(packetHeader[5]) << 8;

            byte[] data = readMultipleBytes(dataLength);
            // the last 2 bytes of the packet data are always 0x00 0x80. why?

            int sum = 0;
            for (int i = 0; i < dataLength; i++) {
                sum += byte2Uint(data[i]);
            }
            // always fails due to the aforementioned 0x00 0x80 in the packet data.
            if (sum != checksum) {
                RobotLog.ee(TAG, "Checksum failure: expected 0x%04x, got 0x%04x", checksum, sum);
                //throw new RuntimeException(String.format("Checksum failure: expected %d, got %d", checksum, sum));
            }
            byte[] returnBuffer = new byte[6 + dataLength];
            System.arraycopy(packetHeader, 0, returnBuffer, 0, 6);
            System.arraycopy(data, 0, returnBuffer, 6, dataLength);
            return returnBuffer;
        }
        return null;
    }

    private byte[] readData() {
        byte[] packetHeader = readMultipleBytes(6);
        int dataLength = byte2Uint(packetHeader[3]);
        if (dataLength > 0) {
            int checksum = byte2Uint(packetHeader[4]);
            checksum |= byte2Uint(packetHeader[5]) << 8;
            byte[] data = readMultipleBytes(dataLength);
            int sum = 0;
            for (int i = 0; i < dataLength; i++) {
                sum += byte2Uint(data[i]);
            }
            if (sum != checksum) {
                RobotLog.ee(TAG, "Checksum failure: expected 0x%04x, got 0x%04x", checksum, sum);
                //throw new RuntimeException(String.format("Checksum failure: expected %d, got %d", checksum, sum));
            }
            return data;
        }
        return null;
    }

    private byte readSingleByte() {
        final LynxI2cReadSingleByteCommand cmd = new LynxI2cReadSingleByteCommand(parent, bus, deviceClient.getI2cAddress());
        try {
            return parent.acquireI2cLockWhile(() -> {
                cmd.send();

                return pollForReadResult(1)[0];
            });
        } catch (InterruptedException e) {
            throw new CommunicationException("Pixy2", "Interrupted while reading a single byte", e);
        } catch (RobotCoreException e) {
            throw new CommunicationException("Pixy2", "RobotCoreException while reading a single byte", e);
        } catch (LynxNackException e) {
            throw new CommunicationException("Pixy2", String.format("LynxNackException while reading a single byte (reason: %s", e.getNack().getNackReasonCode().toString()), e);
        }
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

    private void writeSingleByte(byte b) {
        final LynxI2cWriteSingleByteCommand cmd = new LynxI2cWriteSingleByteCommand(parent, bus, deviceClient.getI2cAddress(), b);
        try {
            parent.acquireI2cLockWhile(() -> {
                sendCommand(cmd);
                return null;
            });
        } catch (InterruptedException e) {
            RobotLog.ee(TAG, e, "Interrupted while writing a single byte");
        } catch (RobotCoreException e) {
            throw new CommunicationException("Pixy2", "RobotCoreException while writing a single byte", e);
        } catch (LynxNackException e) {
            throw new CommunicationException("Pixy2", String.format("LynxNackException while writing a single byte (reason: %s", e.getNack().getNackReasonCode().toString()), e);
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

    // the difficulties of converting between C-style unsigned ints and java's two's complement ints
    // is making me seriously consider writing this class in c++ and using JNI so i don't have to
    // deal with the conversions
    private int byte2Uint(byte b) {
        return b & 0xff;
    }
}
