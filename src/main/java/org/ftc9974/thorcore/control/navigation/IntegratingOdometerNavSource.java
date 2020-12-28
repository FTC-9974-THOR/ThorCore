package org.ftc9974.thorcore.control.navigation;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.control.math.Vector2;

public class IntegratingOdometerNavSource implements NavSource {

    private DcMotorEx xEnc, yEnc;
    private IMUNavSource imu;

    private Vector2 location;
    private Vector2 lastEncoderPosition;
    private long lastTimestamp;
    private double mmPerTick;
    private double xRadius;
    private double yRadius;

    public IntegratingOdometerNavSource(DcMotorEx xEnc, DcMotorEx yEnc, double xRadius, double yRadius, HardwareMap hw, double wheelDiameter, double ticksPerRevolution) {
        this.xEnc = xEnc;
        this.yEnc = yEnc;
        imu = new IMUNavSource(hw);
        mmPerTick = (wheelDiameter * Math.PI) / ticksPerRevolution;

        location = new Vector2(0, 0);
        lastEncoderPosition = new Vector2(0, 0);
        lastTimestamp = -1;
    }

    public void update() {
        if (lastTimestamp == -1) {
            location = null;
        }
        double deltaTime = (SystemClock.uptimeMillis() - lastTimestamp) / 1000.0;
        lastTimestamp = SystemClock.uptimeMillis();

        Vector2 encoderPosition = new Vector2(xEnc.getCurrentPosition(), yEnc.getCurrentPosition());
        Vector2 delta = encoderPosition.subtract(lastEncoderPosition).scalarMultiply(mmPerTick);

        location = location.add(delta.rotate(imu.getHeading()).scalarMultiply(deltaTime));
        lastEncoderPosition = encoderPosition;
    }

    @Override
    public Vector2 getLocation() {
        update();
        return location;
    }

    @Override
    public double getHeading() {
        return 0;
    }

    @Override
    public boolean trustworthy() {
        return false;
    }
}
