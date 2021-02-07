package org.ftc9974.thorcore.robot.sensors;

public class Pixy2USB {

    public static class Version {
        public final int hardware;
        public final short firmwareMajor, firmwareMinor;
        public final int firmwareBuild;
        public final String firmwareType;

        public Version(int hardware, short firmwareMajor, short firmwareMinor, int firmwareBuild, String firmwareType) {
            this.hardware = hardware;
            this.firmwareMajor = firmwareMajor;
            this.firmwareMinor = firmwareMinor;
            this.firmwareBuild = firmwareBuild;
            this.firmwareType = firmwareType;
        }
    }

    private final int fileDescriptor;

    public Pixy2USB(int fd) {
        fileDescriptor = fd;
        initNativeLayer();
    }

    public void open() {
        nativeOpen(fileDescriptor);
    }

    public void close() {
        nativeClose(fileDescriptor);
    }

    public Version getVersion() {
        return nativeGetVersion(fileDescriptor);
    }

    private static native void initNativeLayer();

    private static native void nativeOpen(int fileDescriptor);
    private static native void nativeClose(int fileDescriptor);

    private static native Version nativeGetVersion(int fileDescriptor);
}
