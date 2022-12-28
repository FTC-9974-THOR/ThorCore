package org.ftc9974.thorcore.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.ftc9974.thorcore.internal.RealizableFactory;

import java.util.concurrent.atomic.AtomicBoolean;

public final class QuadratureEncoder {

    private DigitalChannel a, b;
    private byte state;
    private int accumulator;

    private class Notifications implements OpModeManagerNotifier.Notifications {

        @Override
        public void onOpModePreInit(OpMode opMode) {

        }

        @Override
        public void onOpModePreStart(OpMode opMode) {

        }

        @Override
        public void onOpModePostStop(OpMode opMode) {
            shutdownRequested.set(true);
            OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getRootActivity()).unregisterListener(this);
        }
    }

    @SuppressWarnings("FieldCanBeLocal")
    private Thread updateThread;
    private AtomicBoolean shutdownRequested;
    private final Object lock = new Object();

    @RealizableFactory
    public QuadratureEncoder(HardwareMap hardwareMap, String name) {
        a = hardwareMap.get(DigitalChannel.class, name + "-a");
        b = hardwareMap.get(DigitalChannel.class, name + "-b");
        state = 0;
        accumulator = 0;
        if (a.getState()) {
            state |= 0b0101;
        }
        if (b.getState()) {
            state |= 0b1010;
        }
        OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getRootActivity()).registerListener(new Notifications());
        shutdownRequested = new AtomicBoolean(false);
        updateThread = new Thread(this::update);
        updateThread.start();
    }

    // going forwards, b rises first
    private void update() {
        while (!shutdownRequested.get()) {
            synchronized (lock) {
                byte workingState = (byte) (state & 0b0011);
                if (a.getState()) {
                    workingState |= 0b0100;
                }
                if (b.getState()) {
                    workingState |= 0b1000;
                }
                switch (workingState) {
                    case 0b0000:
                    case 0b0101:
                    case 0b1010:
                    case 0b1111:
                        // no movement
                        break;
                    case 0b0001:
                    case 0b0111:
                    case 0b1000:
                    case 0b1110:
                        // +1
                        accumulator++;
                        break;
                    case 0b0010:
                    case 0b0100:
                    case 0b1011:
                    case 0b1101:
                        // -1
                        accumulator--;
                        break;
                    case 0b0011:
                    case 0b1100:
                        // +2
                        accumulator += 2;
                        break;
                    default:
                        // -2
                        accumulator -= 2;
                        break;
                }
                state >>= 2;
            }
        }
    }

    public int getPosition() {
        synchronized (lock) {
            return accumulator;
        }
    }

    public void reset() {
        synchronized (lock) {
            accumulator = 0;
        }
    }
}
