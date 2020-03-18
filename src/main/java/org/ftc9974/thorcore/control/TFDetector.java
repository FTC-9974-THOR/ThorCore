package org.ftc9974.thorcore.control;

import android.content.Context;
import android.os.Build;
import android.support.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public final class TFDetector {

    private VuforiaLocalizer vuforiaLocalizer;
    private TFObjectDetector objectDetector;

    public TFDetector(@NonNull String tfliteFile, @NonNull String vuforiaKey, @NonNull Context appContext, @NonNull String... labels) {
        this(tfliteFile, vuforiaKey, appContext, ClassFactory.getInstance().getCameraManager().nameForUnknownCamera(), false, labels);
    }

    public TFDetector(@NonNull String tfliteFile, @NonNull String vuforiaKey, @NonNull Context appContext, boolean showPreview, @NonNull String... labels) {
        this(tfliteFile, vuforiaKey, appContext, ClassFactory.getInstance().getCameraManager().nameForUnknownCamera(), showPreview, labels);
    }

    public TFDetector(@NonNull String tfliteFile, @NonNull String vuforiaKey, @NonNull Context appContext, @NonNull CameraName camera, @NonNull String... labels) {
        this(tfliteFile, vuforiaKey, appContext, camera, false, labels);
    }

    public TFDetector(@NonNull String tfliteFile, @NonNull String vuforiaKey, @NonNull Context appContext, @NonNull CameraName camera, boolean showPreview, @NonNull String... labels) {
        if (!ClassFactory.getInstance().canCreateTFObjectDetector()) {
            throw new RuntimeException(String.format("Android SDK version too low! Current: %d Required: 23", Build.VERSION.SDK_INT));
        }
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();
        vuforiaParameters.vuforiaLicenseKey = vuforiaKey;
        vuforiaParameters.cameraName = camera;
        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(vuforiaParameters);
        TFObjectDetector.Parameters tfParameters;
        if (showPreview) {
            int tfMonitorId = appContext.getResources().getIdentifier("tfodMonitorViewId", "id", appContext.getPackageName());
            tfParameters = new TFObjectDetector.Parameters(tfMonitorId);
        } else {
            tfParameters = new TFObjectDetector.Parameters();
        }
        objectDetector = ClassFactory.getInstance().createTFObjectDetector(tfParameters, vuforiaLocalizer);
        objectDetector.loadModelFromAsset(tfliteFile, labels);
    }

    public void activate() {
        objectDetector.activate();
    }

    public void deactivate() {
        objectDetector.deactivate();
    }

    public List<Recognition> getRecognitions() {
        return objectDetector.getRecognitions();
    }

    public void shutdown() {
        objectDetector.shutdown();
        //vuforiaLocalizer.close();
    }

    public VuforiaLocalizer getVuforiaLocalizer() {
        return vuforiaLocalizer;
    }
}
