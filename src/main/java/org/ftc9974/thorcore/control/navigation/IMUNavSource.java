package org.ftc9974.thorcore.control.navigation;

import androidx.annotation.IntRange;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.ftc9974.thorcore.control.math.Vector2;

// reading data from the IMU is very slow, with each read taking about 30ms. If speed is important,
// the IMU might not be a good choice.
public final class IMUNavSource implements NavSource, BNO055IMU.AccelerationIntegrator {

    private BNO055IMU imu;
    private Position position;
    private Velocity velocity;
    private Acceleration acceleration;

    private int axis;

    private static final boolean rotationRequired = false;

    private boolean isFallback;

    public IMUNavSource(HardwareMap hardwareMap) {
        this(hardwareMap, 3, null);
    }

    public IMUNavSource(HardwareMap hardwareMap, @IntRange(from = 1, to = 3) int axis) {
        this(hardwareMap, axis, null);
    }

    public IMUNavSource(HardwareMap hardwareMap, @IntRange(from = 1, to = 3) int axis, @Nullable String calibrationFile) {
        this.axis = axis;
        if (axis < 1 || axis > 3) {
            throw new IllegalArgumentException("Axis argument must be in the range [1-3]");
        }
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.accelerationIntegrationAlgorithm = this;
        parameters.calibrationDataFile = calibrationFile;
        if (!imu.initialize(parameters)) {
            RobotLog.e("Error initializing IMU! Error: " + imu.getSystemError().toString());
            imu = hardwareMap.get(BNO055IMU.class, "imu 0");
            isFallback = true;
            if (!imu.initialize(parameters)) {
                throw new RuntimeException("Both IMUs have failed! Error message: " + imu.getSystemError().toString());
            }
        }
        imu.startAccelerationIntegration(null, null, 10);
    }

    public BNO055IMU.CalibrationStatus getCalibrationStatus() {
        return imu.getCalibrationStatus();
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public boolean isFallback() {
        return isFallback;
    }

    @Override
    public Vector2 getLocation() {
        return new Vector2(position.x, position.y);
    }

    // reading data from the IMU is very slow, with each read taking about 30ms. If speed is
    // important, the IMU might not be a good choice.
    @Override
    public double getHeading() {
        Orientation orientation = getOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.XYZ);
        switch (axis) {
            case 1:
                return orientation.firstAngle;
            case 2:
                return orientation.secondAngle;
            case 3:
            default:
                return orientation.thirdAngle;
        }
    }

    @Override
    public boolean trustworthy() {
        return imu.isGyroCalibrated() && imu.isAccelerometerCalibrated() && imu.isSystemCalibrated();
    }

    public Orientation getOrientation() {
        return imu.getAngularOrientation();
    }

    @Override
    public void initialize(@NonNull BNO055IMU.Parameters parameters, @Nullable Position initialPosition, @Nullable Velocity initialVelocity) {
        position = initialPosition != null ? initialPosition : position;
        velocity = initialVelocity != null ? initialVelocity : velocity;
        acceleration = null;
    }

    @Override
    public Position getPosition() {
        return position;
    }

    @Override
    public Velocity getVelocity() {
        return velocity;
    }

    @Override
    public Acceleration getAcceleration() {
        return acceleration;
    }

    @Override
    public void update(Acceleration linearAcceleration) {
        if (linearAcceleration.acquisitionTime != 0) {
            if (acceleration != null) {
                Acceleration prevAccel = acceleration;
                Velocity prevVelocity = velocity;

                if (rotationRequired) {
                    OpenGLMatrix matrix = new OpenGLMatrix();
                    Orientation orientation = imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).toAngleUnit(AngleUnit.RADIANS).toAxesReference(AxesReference.EXTRINSIC);
                    matrix.rotate(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS, orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle);
                    matrix.translate((float) linearAcceleration.xAccel, (float) linearAcceleration.yAccel, (float) linearAcceleration.zAccel);
                    VectorF translation = matrix.getTranslation();
                    acceleration = new Acceleration(linearAcceleration.unit, translation.get(0), translation.get(1), translation.get(0), linearAcceleration.acquisitionTime);
                } else {
                    acceleration = linearAcceleration;
                }

                if (prevAccel.acquisitionTime != 0) {
                    Velocity dV = NavUtil.meanIntegrate(acceleration, prevAccel);
                    velocity = NavUtil.plus(velocity, dV);
                }

                if (prevVelocity.acquisitionTime != 0) {
                    Position dP = NavUtil.meanIntegrate(velocity, prevVelocity);
                    position = NavUtil.plus(position, dP);
                }
            }
        }
    }
}
