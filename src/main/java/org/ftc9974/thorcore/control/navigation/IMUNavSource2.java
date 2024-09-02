package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.ftc9974.thorcore.control.math.Vector2;

import java.util.List;

public class IMUNavSource2 implements NavSource {

    private IMU imu = null;
    private boolean inFailover = false;

    public IMUNavSource2(HardwareMap hw, ImuOrientationOnRobot orientation) {
        IMU.Parameters params = new IMU.Parameters(orientation);

        List<IMU> imus = hw.getAll(IMU.class);
        for (IMU imu : imus) {
            if (tryInit(imu, params)) break;
            else inFailover = true;
        }

        if (imu == null) throw new RuntimeException("All IMUs have failed to initialize!");
    }

    public IMUNavSource2(HardwareMap hw,
                         String primaryName, ImuOrientationOnRobot primaryOrientation,
                         String secondaryName, ImuOrientationOnRobot secondaryOrientation) {
        if (tryInit(hw.get(IMU.class, primaryName), new IMU.Parameters(primaryOrientation))) return;
        inFailover = true;
        if (tryInit(hw.get(IMU.class, secondaryName), new IMU.Parameters(secondaryOrientation))) return;
        throw new RuntimeException("Both IMUs have failed to initialize!");
    }

    private boolean tryInit(IMU imu, IMU.Parameters params) {
        if (imu.initialize(params)) {
            this.imu = imu;
            imu.resetYaw();
            return true;
        } else return false;
    }

    @Override
    public Vector2 getLocation() {
        return null;
    }

    @Override
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public boolean trustworthy() {
        return false;
    }

    public AngularVelocity getAngularVelocity() {
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS);
    }

    public double getHeadingVelocity() {
        return getAngularVelocity().zRotationRate;
    }

    public boolean isInFailover() {
        return inFailover;
    }

    public IMU getImu() {
        return imu;
    }
}
