package org.ftc9974.thorcore.robot.sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.ftc9974.thorcore.internal.CommunicationException;
import org.ftc9974.thorcore.internal.LynxI2cDeviceSynchEx;

//@I2cDeviceType
//@DeviceProperties(xmlTag = "Pixy2", name = "Pixy2", description = "Pixy2 Vision Camera")
public final class Pixy2 extends I2cDeviceSynchDevice<LynxI2cDeviceSynchEx> {

    private static final byte[] EMPTY_DATA = new byte[0];

    public enum PacketType {
        GET_VERSION(14),
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
        public short majorFirmware, minorFirmware;
        public String firmwareType;

        private Version(byte[] payload) {
            hardwareVersion = payload[0];
            hardwareVersion |= payload[1] << 8;
            majorFirmware = payload[2];
            minorFirmware = payload[3];
            firmwareBuild = payload[4];
            firmwareBuild |= payload[5] << 8;
            byte[] firmwareString = new byte[payload.length - 6];
            System.arraycopy(payload, 6, firmwareString, 0, firmwareString.length);
            firmwareType = new String(firmwareString);
        }
    }

    public Pixy2(LynxI2cDeviceSynchEx device) {
        super(device, true);

        deviceClient.setI2cAddress(I2cAddr.create7bit(0x54));

        super.registerArmingStateCallback(false);
        deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        return false;
    }

    public void setI2cAddress(int address) {
        deviceClient.setI2cAddress(I2cAddr.create7bit(address));
    }

    // API methods

    public Version getPixyVersion() {
        writePacket(PacketType.GET_VERSION, EMPTY_DATA);
        byte[] payload = readData();
        if (payload == null) {
            throw new CommunicationException(String.format("Pixy2 [%s]", deviceClient.getI2cAddr().toString()), "I2C Failure");
        }
        return new Version(payload);
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
        deviceClient.writeMultipleBytes(packet);
    }

    private byte[] readPacket() {
        byte[] packetHeader = deviceClient.readMultipleBytes(6);
        byte dataLength = packetHeader[3];
        if (dataLength > 0) {
            short checksum = packetHeader[4];
            checksum |= ((short) packetHeader[5]) << 8;
            byte[] data = deviceClient.readMultipleBytes(dataLength);
            short sum = 0;
            for (short i = 0; i < dataLength; i++) {
                sum += data[i];
            }
            if (sum != checksum) {
                throw new RuntimeException(String.format("Checksum failure: expected %d, got %d", checksum, sum));
            }
            byte[] returnBuffer = new byte[6 + dataLength];
            System.arraycopy(packetHeader, 0, returnBuffer, 0, 6);
            System.arraycopy(data, 0, returnBuffer, 6, dataLength);
            return returnBuffer;
        }
        return null;
    }

    private byte[] readData() {
        byte[] packetHeader = deviceClient.readMultipleBytes(6);
        byte dataLength = packetHeader[3];
        if (dataLength > 0) {
            short checksum = packetHeader[4];
            checksum |= ((short) packetHeader[5]) << 8;
            byte[] data = deviceClient.readMultipleBytes(dataLength);
            short sum = 0;
            for (short i = 0; i < dataLength; i++) {
                sum += data[i];
            }
            if (sum != checksum) {
                throw new RuntimeException(String.format("Checksum failure: expected %d, got %d", checksum, sum));
            }
            return data;
        }
        return null;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Pixy2 CMUCam5";
    }
}
