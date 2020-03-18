package org.ftc9974.thorcore.control;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.internal.RealizableFactory;

public final class PhoneGyroscope extends PhoneSensor {

    @RealizableFactory
    public static PhoneGyroscope create(String name, HardwareMap hardwareMap) {
        return new PhoneGyroscope(hardwareMap.appContext);
    }

    public PhoneGyroscope(Context appContext) {
        super(appContext, Sensor.TYPE_ROTATION_VECTOR, 3, SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    protected void onNewData(SensorEvent event, double[] data) {
        float[] R = new float[4];
        float[] values = new float[3];
        SensorManager.getRotationMatrixFromVector(R, event.values);
        SensorManager.getOrientation(R, values);
        for (int i = 0; i < values.length; i++) {
            data[i] = values[i];
        }
    }

    public double getRoll() {
        return getData()[2];
    }

    public double getPitch() {
        return getData()[1];
    }

    public double getHeading() {
        return getData()[0];
    }
}
