package org.ftc9974.thorcore.control;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

public final class PhoneAccelerometer extends PhoneSensor {

    public PhoneAccelerometer(Context appContext) {
        super(appContext, Sensor.TYPE_ACCELEROMETER, 2, SensorManager.SENSOR_DELAY_GAME);
    }

    @Override
    protected void onNewData(SensorEvent event, double[] data) {
        double x = -event.values[1];
        double y = event.values[0];
        double z = -event.values[2];
        data[0] = Math.atan2(y, z) + Math.PI;
        if (data[0] > Math.PI) {
            data[0] = data[0] - 2 * Math.PI;
        } else if (data[0] < -Math.PI) {
            data[0] = data[0] + 2 * Math.PI;
        }
        data[1] = Math.atan2(-x, Math.sqrt(Math.pow(y, 2) + Math.pow(z, 2)));
    }

    public double getRoll() {
        return getData()[0];
    }

    public double getPitch() {
        return getData()[1];
    }
}
