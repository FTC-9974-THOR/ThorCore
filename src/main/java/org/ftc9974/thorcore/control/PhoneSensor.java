package org.ftc9974.thorcore.control;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;

import java.util.Locale;

abstract class PhoneSensor implements SensorEventListener {

    protected SensorManager sensorManager;
    protected Sensor sensor;
    private double[] data;
    private final Object dataLock = new Object();
    private long timestamp;

    protected int pollingInterval;

    protected PhoneSensor(Context appContext, int sensorType, int dataSize) {
        this(appContext, sensorType, dataSize, 100);
    }

    protected PhoneSensor(Context appContext, int sensorType, int dataSize, int pollingInterval) {
        sensorManager = (SensorManager) appContext.getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager == null) {
            throw new RuntimeException("Could not get sensor manager");
        }
        sensor = sensorManager.getDefaultSensor(sensorType);
        if (sensor == null) {
            String sensorName;
            // TODO: 11/11/18 Finish this switch statement
            switch (sensorType) {
                case Sensor.TYPE_ACCELEROMETER:
                case Sensor.TYPE_ACCELEROMETER_UNCALIBRATED:
                    sensorName = "ACCELEROMETER";
                    break;
                case Sensor.TYPE_MAGNETIC_FIELD:
                case Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED:
                    sensorName = "MAGNETIC_FIELD";
                    break;
                case Sensor.TYPE_ORIENTATION:
                    sensorName = "ORIENTATION (which is deprecated anyways)";
                    break;
                case Sensor.TYPE_GYROSCOPE:
                case Sensor.TYPE_GYROSCOPE_UNCALIBRATED:
                    sensorName = "GYROSCOPE";
                    break;
                case Sensor.TYPE_LIGHT:
                    sensorName = "LIGHT";
                    break;
                case Sensor.TYPE_PRESSURE:
                    sensorName = "PRESSURE";
                    break;
                case Sensor.TYPE_TEMPERATURE:
                    sensorName = "TEMPERATURE (which is deprecated anyways)";
                    break;
                case Sensor.TYPE_PROXIMITY:
                    sensorName = "PROXIMITY";
                    break;
                case Sensor.TYPE_GRAVITY:
                    sensorName = "GRAVITY";
                    break;
                case Sensor.TYPE_LINEAR_ACCELERATION:
                    sensorName = "LINEAR_ACCELERATION";
                    break;
                case Sensor.TYPE_ROTATION_VECTOR:
                    sensorName = "ROTATION_VECTOR";
                    break;
                case Sensor.TYPE_RELATIVE_HUMIDITY:
                    sensorName = "RELATIVE_HUMIDITY";
                    break;
                default:
                    sensorName = Integer.toString(sensorType);
                    break;
            }
            throw new RuntimeException(String.format(Locale.getDefault(), "This %s %s %s does not have a sensor of type %s", Build.MANUFACTURER, Build.MODEL, Build.VERSION.RELEASE, sensorName));
        }
        this.pollingInterval = pollingInterval;
        sensorManager.registerListener(this, sensor, pollingInterval);
        data = new double[dataSize];
    }

    protected double[] getData() {
        synchronized (dataLock) {
            return data;
        }
    }

    public long getTimestamp() {
        synchronized (dataLock) {
            return timestamp;
        }
    }

    protected abstract void onNewData(SensorEvent event, double[] data);

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == sensor.getType()) {
            synchronized (dataLock) {
                onNewData(event, data);
                timestamp = event.timestamp;
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public void shutdown() {
        sensorManager.unregisterListener(this);
    }
}
