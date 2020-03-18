package org.ftc9974.thorcore.control.navigation;

import android.content.Context;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.ftc9974.thorcore.control.math.Vector2;
import org.jetbrains.annotations.Nullable;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public final class VuMarkNavSource implements NavSource {

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (float) (12 * 6 * 25.4),
            mmTargetHeight = (float) (6 * 25.4);

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private VuforiaLocalizer vuforia;
    private VuforiaTrackables trackables;
    private OpenGLMatrix lastKnownLocation;

    private VuforiaTrackable stoneTarget;
    private OpenGLMatrix lastKnownStoneLocation;

    public VuMarkNavSource(VuforiaLocalizer localizer, Context context, String vuforiaKey, VuforiaLocalizer.CameraDirection direction, OpenGLMatrix phoneLocation) {
        this(localizer, context, vuforiaKey, ClassFactory.getInstance().getCameraManager().nameForUnknownCamera(), direction, phoneLocation, false);
    }

    public VuMarkNavSource(VuforiaLocalizer localizer, Context context, String vuforiaKey, CameraName cameraName, OpenGLMatrix cameraLocation) {
        this(localizer, context, vuforiaKey, cameraName, VuforiaLocalizer.CameraDirection.BACK, cameraLocation, true);
    }

    public VuMarkNavSource(Context context, String vuforiaKey, VuforiaLocalizer.CameraDirection direction, OpenGLMatrix phoneLocation) {
        this(null, context, vuforiaKey, ClassFactory.getInstance().getCameraManager().nameForUnknownCamera(), direction, phoneLocation, false);
    }

    public VuMarkNavSource(Context context, String vuforiaKey, CameraName cameraName, OpenGLMatrix cameraLocation) {
        this(null, context, vuforiaKey, cameraName, VuforiaLocalizer.CameraDirection.BACK, cameraLocation, true);
    }

    VuMarkNavSource(VuforiaLocalizer localizer, Context context, String vuforiaKey, CameraName cameraName, VuforiaLocalizer.CameraDirection direction, OpenGLMatrix cameraLocation, boolean isWebcam) {
        int cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());
        if (localizer == null) {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.cameraDirection = direction;
            parameters.cameraName = cameraName;
            parameters.vuforiaLicenseKey = vuforiaKey;
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        } else {
            vuforia = localizer;
        }

        trackables = vuforia.loadTrackablesFromAsset("Skystone");
        stoneTarget = trackables.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = trackables.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = trackables.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = trackables.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = trackables.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = trackables.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = trackables.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = trackables.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = trackables.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = trackables.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = trackables.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = trackables.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = trackables.get(12);
        rear2.setName("Rear Perimeter 2");

        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        for (VuforiaTrackable trackable : trackables) {
            if (isWebcam) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(cameraName, cameraLocation);
            } else {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(cameraLocation, direction);
            }
        }

        trackables.activate();
        lastKnownLocation = new OpenGLMatrix();
        lastKnownStoneLocation = new OpenGLMatrix();
    }

    @Override
    public Vector2 getLocation() {
        for (VuforiaTrackable trackable : trackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                OpenGLMatrix location = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (location != null) {
                    VectorF translation = location.getTranslation();
                    if (trackable.getName().equals("Stone Target")) {
                        lastKnownStoneLocation = location;
                    } else {
                        lastKnownLocation = location;
                        return new Vector2(-translation.get(1), translation.get(0));
                    }
                }
            }
        }
        VectorF translation = lastKnownLocation.getTranslation();
        return new Vector2(-translation.get(1), translation.get(0));
    }

    @Override
    public double getHeading() {
        Orientation rotation = Orientation.getOrientation(lastKnownLocation, EXTRINSIC, XYZ, AngleUnit.RADIANS);
        return rotation.thirdAngle;
    }

    @Override
    public boolean trustworthy() {
        for (VuforiaTrackable trackable : trackables) {
            if (!trackable.getName().equals("Stone Target") && ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                return true;
            }
        }
        return false;
    }

    public int getNumVuMarksDetected() {
        int detected = 0;
        for (VuforiaTrackable trackable : trackables) {
            if (!trackable.getName().equals("Stone Target") && ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                detected++;
            }
        }
        return detected;
    }

    public boolean canSeeStone() {
        return ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible();
    }

    /**
     * Gets the location of the stone relative to the robot origin
     * @return location
     */
    public @Nullable OpenGLMatrix getRelativeStoneLocation() {
        OpenGLMatrix relativeStoneLocation = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
        if (relativeStoneLocation != null) {
            lastKnownStoneLocation = relativeStoneLocation;
        }

        return lastKnownStoneLocation;
    }

    /*
     * Gets the location of the stone relative to field origin
     * @return location
     */
    //todo
    /*public Vector2 getAbsoluteStoneLocation() {
        return getLocation().add(getRelativeStoneLocation());
    }*/

    public VuforiaLocalizer getVuforiaLocalizer() {
        return vuforia;
    }
}
