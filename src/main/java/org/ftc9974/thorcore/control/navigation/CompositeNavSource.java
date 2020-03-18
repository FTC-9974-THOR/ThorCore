package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.ftc9974.thorcore.control.math.Vector2;

public final class CompositeNavSource implements NavSource {

    private VuMarkNavSource vuMarkNavSource;
    private IMUNavSource imuNavSource;

    private boolean hasOffset;
    private double imuOffset;

    public CompositeNavSource(VuforiaLocalizer localizer, HardwareMap hardwareMap, String vuforiaKey, VuforiaLocalizer.CameraDirection direction, OpenGLMatrix phoneLocation) {
        this(localizer, hardwareMap, vuforiaKey, direction, ClassFactory.getInstance().getCameraManager().nameForUnknownCamera(), phoneLocation, false);
    }

    public CompositeNavSource(VuforiaLocalizer localizer, HardwareMap hardwareMap, String vuforiaKey, CameraName cameraName, OpenGLMatrix cameraLocation) {
        this(localizer, hardwareMap, vuforiaKey, VuforiaLocalizer.CameraDirection.BACK, cameraName, cameraLocation, true);
    }

    public CompositeNavSource(HardwareMap hardwareMap, String vuforiaKey, VuforiaLocalizer.CameraDirection direction, OpenGLMatrix phoneLocation) {
        this(null, hardwareMap, vuforiaKey, direction, ClassFactory.getInstance().getCameraManager().nameForUnknownCamera(), phoneLocation, false);
    }

    public CompositeNavSource(HardwareMap hardwareMap, String vuforiaKey, CameraName cameraName, OpenGLMatrix cameraLocation) {
        this(null, hardwareMap, vuforiaKey, VuforiaLocalizer.CameraDirection.BACK, cameraName, cameraLocation, true);
    }

    private CompositeNavSource(VuforiaLocalizer localizer, HardwareMap hardwareMap, String vuforiaKey, VuforiaLocalizer.CameraDirection direction, CameraName name, OpenGLMatrix location, boolean isWebcam) {
        vuMarkNavSource = new VuMarkNavSource(localizer, hardwareMap.appContext, vuforiaKey, name, direction, location, isWebcam);
        imuNavSource = new IMUNavSource(hardwareMap);
    }

    @Override
    public Vector2 getLocation() {
        return vuMarkNavSource.getLocation();
    }

    @Override
    public double getHeading() {
        return imuNavSource.getHeading();
    }

    @Override
    public boolean trustworthy() {
        // this may be somewhat naive, but it's most likely true.
        return true;
    }

    public BNO055IMU.CalibrationStatus getCalibrationStatus() {
        return imuNavSource.getCalibrationStatus();
    }
}
