package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.control.math.Vector2;

public final class OdometerNavSource implements NavSource {

    private DcMotorEx xEncoder, yEncoder;
    private IMUNavSource imu;

    private Vector2 currentPosition;
    private double lastHeading;

    private int lastXPosition, lastYPosition;

    private double wheelDiameter, ticksPerRev;

    private double xRho, xR, yRho, yR;

    private int headingWraparound;

    private int xNegation, yNegation;

    private double headingOffset;
    private boolean totalizing;

    public OdometerNavSource(HardwareMap hw, DcMotorEx xEncoder, DcMotorEx yEncoder, Vector2 xPosition, Vector2 yPosition, double wheelDiameter, double ticksPerRev) {
        this.xEncoder = xEncoder;
        this.yEncoder = yEncoder;
        imu = new IMUNavSource(hw);
        this.wheelDiameter = wheelDiameter;
        this.ticksPerRev = ticksPerRev;

        currentPosition = new Vector2(0, 0);

        xR = xPosition.getMagnitude();
        xRho = Math.atan(Math.abs(xPosition.getY() / xPosition.getX()));
        yR = yPosition.getMagnitude();
        yRho = Math.atan(Math.abs(yPosition.getY() / yPosition.getX()));
        xNegation = 1;
        yNegation = 1;

    }

    public void resetOdometers() {
        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        headingOffset = getHeading();
    }

    public void setXInversion(boolean inverted) {
        xNegation = inverted ? -1 : 1;
    }

    public void setYInversion(boolean inverted) {
        yNegation = inverted ? -1 : 1;
    }

    public void setTotalizing(boolean totalizing) {
        this.totalizing = totalizing;
    }

    @Override
    public Vector2 getLocation() {
        int xPosition = xNegation * xEncoder.getCurrentPosition();
        int yPosition = yNegation * yEncoder.getCurrentPosition();

        /*// remove counts from turning
        double currentHeading = getHeading();
        if (lastHeading < -0.5 * Math.PI && currentHeading > 0.5 * Math.PI) {
            // clockwise wraparound
            headingWraparound--;
        } else if (lastHeading > 0.5 * Math.PI && currentHeading < -0.5 * Math.PI) {
            // counterclockwise wraparound
            headingWraparound++;
        }
        double continuousHeading = currentHeading + 2 * Math.PI * headingWraparound;
        xPosition += continuousHeading * xR * Math.sin(xRho);
        yPosition += continuousHeading * yR * Math.cos(yRho);
        */
        if (totalizing) {
            int xDelta = xPosition - lastXPosition;
            int yDelta = yPosition - lastYPosition;
            Vector2 delta = new Vector2(
                    wheelDiameter * Math.PI * (xDelta / ticksPerRev),
                    wheelDiameter * Math.PI * (yDelta / ticksPerRev)
            );
            delta = delta.rotate((getHeading() - headingOffset));
            currentPosition = currentPosition.add(delta);
            lastHeading = getHeading();
            //lastHeading = currentHeading;
            lastXPosition = xPosition;
            lastYPosition = yPosition;
            return currentPosition;
        } else {
            currentPosition = new Vector2(
                    wheelDiameter * Math.PI * (xPosition / ticksPerRev),
                    wheelDiameter * Math.PI * (yPosition / ticksPerRev)
            );
            return currentPosition;
        }

    }

    @Override
    public double getHeading() {
        return imu.getHeading();
    }

    @Override
    public boolean trustworthy() {
        return true;
    }

    public double calculateContinuousHeading() {
        double currentHeading = getHeading();
        if (lastHeading < -0.5 * Math.PI && currentHeading > 0.5 * Math.PI) {
            // clockwise wraparound
            headingWraparound--;
        } else if (lastHeading > 0.5 * Math.PI && currentHeading < -0.5 * Math.PI) {
            // counterclockwise wraparound
            headingWraparound++;
        }
        lastHeading = currentHeading;
        return currentHeading + 2 * Math.PI * headingWraparound;
    }

    public int getHeadingWraparound() {
        return headingWraparound;
    }

    public int getXEncoderReading() {
        return xEncoder.getCurrentPosition();
    }

    public int getYEncoderReading() {
        return yEncoder.getCurrentPosition();
    }
}
