package org.ftc9974.thorcore.robot.sensors;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;

import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.NativeCodeLoader;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Class for managing {@link Pixy2USB}s that are connected to the robot.
 *
 * Do not instantiate this class. The constructor and onReceive() are only public to allow the Android
 * OS to use this class as a broadcast receiver. (although I'm not sure said methods being public is
 * strictly required)
 *
 * Known Problem:
 * If a Pixy is plugged in before the app starts (ie, it was never unplugged in the first place),
 * it won't be detected by the manager. If this happens, unplug the Pixy and plug it back in.
 *
 * @see Pixy2USB
 * @see org.ftc9974.thorcore.samples.SamplePixy2USB
 */
// todo scan for Pixies that were plugged in on boot
public class Pixy2USBManager extends BroadcastReceiver {

    private static final String TAG = "Pixy2USBManager";

    private static final ConcurrentHashMap<Integer, Pixy2USB> connectedPixies;

    static {
        NativeCodeLoader.load();

        connectedPixies = new ConcurrentHashMap<>(2);
    }

    @Override
    public void onReceive(Context context, Intent intent) {
        String action = intent.getAction();
        RobotLog.ii(TAG, "Intent received: action: %s", action);

        UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);

        if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action)) {
            UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            RobotLog.dd(TAG, "USB Device Attached: name: %s devid: %d s/n: %s vid: 0x%04x pid: 0x%04x hasPerms: %s",
                    device.getDeviceName(), device.getDeviceId(), device.getSerialNumber(), device.getVendorId(), device.getProductId(),
                    usbManager.hasPermission(device) ? "yes" : "no");

            if (isPixy(device) && !connectedPixies.containsKey(device.getDeviceId())) {
                registerPixy(usbManager, device);
            }
        } else if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action)) {
            UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
            RobotLog.dd(TAG, "USB Device Detached: vid: 0x%04x pid: 0x%04x", device.getVendorId(), device.getProductId());

            if (isPixy(device) && connectedPixies.containsKey(device.getDeviceId())) {
                deregisterPixy(device);
            }
        }
    }

    // there's an intent filter already, but it isn't working for some reason.
    private static boolean isPixy(UsbDevice device) {
        return device.getVendorId() == 0xb1ac && device.getProductId() == 0xf000;
    }

    private static void registerPixy(UsbManager manager, UsbDevice device) {
        UsbDeviceConnection connection = manager.openDevice(device);
        Pixy2USB pixy = new Pixy2USB(connection);
        try {
            pixy.open();
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "IOException opening pixy: %s", e.getMessage());
            pixy.close();
        }
        connectedPixies.put(device.getDeviceId(), pixy);
    }

    private static void deregisterPixy(UsbDevice device) {
        Pixy2USB pixy = connectedPixies.get(device.getDeviceId());
        if (pixy == null) {
            RobotLog.ww(TAG, "Somehow an entry disappeared from the registry. This should only happen if deregisterPixy gets called twice on the same device very quickly.");
            return;
        }
        pixy.close();
        connectedPixies.remove(device.getDeviceId());
    }

    /**
     * Gets a list of Pixies connected to the robot over USB.
     *
     * The returned list is unmodifiable and concurrent (as it's a view of a concurrent collection).
     *
     * @return pixy list
     */
    public static List<Pixy2USB> getConnectedPixies() {
        return Collections.unmodifiableList(new ArrayList<>(connectedPixies.values()));
    }
}
