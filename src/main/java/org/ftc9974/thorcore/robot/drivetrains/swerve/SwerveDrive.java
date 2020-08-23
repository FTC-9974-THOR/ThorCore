package org.ftc9974.thorcore.robot.drivetrains.swerve;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.MotorType;
import org.ftc9974.thorcore.util.MathUtilities;

public final class SwerveDrive implements HolonomicDrivetrain {

    public SwerveModule leftModule,
                        rightModule;

    private Vector2 leftPosition, rightPosition;
    private double trackwidth;

    private Telemetry telemetry;

    public SwerveDrive(HardwareMap hw) {
        leftModule = new SwerveModule("left", hw);
        rightModule = new SwerveModule("right", hw);

        leftModule.setPIDFTunings(
                1.3,
                0,
                0.1,
                0
        );
        rightModule.setPIDFTunings(
                1.3,
                0,
                0.1,
                0
        );
        //left.setPIDFNominalOutput(0.03);
        //right.setPIDFNominalOutput(0.03);
        leftModule.setPIDFAtTargetThreshold(0.005);
        rightModule.setPIDFAtTargetThreshold(0.005);

        leftModule.setUseQuadEncoder(false);
        leftModule.setEncoderOffset(0);
        rightModule.setUseQuadEncoder(false);
        rightModule.setEncoderOffset(0);

        leftModule.setDriveInversion(true);
        rightModule.setDriveInversion(true);

        leftPosition = new Vector2(-0.5 * 432 + 60, 0);
        rightPosition = new Vector2(0.5 * 432 - 60, 0);

        trackwidth = Math.abs(leftPosition.subtract(rightPosition).getX());

        leftModule.setTargetDirection(0.5 * Math.PI);
        rightModule.setTargetDirection(0.5 * Math.PI);
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setLeftEncoderOffset(double offset) {
        leftModule.setEncoderOffset(offset);
    }

    public void setRightEncoderOffset(double offset) {
        rightModule.setEncoderOffset(offset);
    }

    @Override
    public void drive(double x, double y, double rot) {
        telemetry.addData("Left At Target", leftModule.atTargetDirection());
        telemetry.addData("Right At Target", rightModule.atTargetDirection());

        Vector2 inputVector = new Vector2(x, y);
        double inputHeading = inputVector.getHeading();
        double inputMag = Math.min(inputVector.getMagnitude(), 1);

        telemetry.addData("Input Mag", inputMag);
        double movementVectorMag = 0.5 * (trackwidth + 10) * inputMag;
        double leftAngle, rightAngle;
        leftAngle = rightAngle = inputHeading;
        double leftSpeed, rightSpeed;
        leftSpeed = rightSpeed = inputMag;

        double inputRot = 0.25 * Math.PI * MathUtilities.constrain(rot, -1, 1);

        double chord = movementVectorMag / Math.sin(0.5 * Math.PI - inputRot);
        double arcAngle = 2.0 * inputRot;
        double arcRadius = chord / (2.0 * Math.sin(arcAngle / 2.0));

        telemetry.addData("Mag", movementVectorMag);
        telemetry.addData("iRot", inputRot);
        telemetry.addData("ArcRadius", arcRadius);

        /*Vector2 leftWorkingPos = leftPosition.rotate(inputHeading - 0.5 * Math.PI);
        Vector2 rightWorkingPos = rightPosition.rotate(inputHeading - 0.5 * Math.PI);

        telemetry.addData("Input Heading", inputHeading);
        telemetry.addData("Left", leftWorkingPos.toString());
        telemetry.addData("Left Heading", leftWorkingPos.getHeading());*/

        Vector2 arcCenter = new Vector2(arcRadius, 0).rotate(inputHeading - 0.5 * Math.PI);
        Vector2 arcToLeft = leftPosition.subtract(arcCenter);
        Vector2 arcToRight = rightPosition.subtract(arcCenter);

        telemetry.addData("AtL M", arcToLeft.getMagnitude());
        telemetry.addData("AtR M", arcToRight.getMagnitude());
        telemetry.addData("AtL H", arcToLeft.getHeading());
        telemetry.addData("AtR H", arcToRight.getHeading());

        if (inputRot != 0) {
            leftAngle = arcToLeft.getHeading();
            rightAngle = arcToRight.getHeading();
            if (inputRot > 0) {
                leftAngle -= 0.5 * Math.PI;
                rightAngle -= 0.5 * Math.PI;
            } else {
                leftAngle += 0.5 * Math.PI;
                rightAngle += 0.5 * Math.PI;
            }

            if (inputMag > 0.05) {
                double leftArcRadius = arcToLeft.getMagnitude();
                double rightArcRadius = arcToRight.getMagnitude();

                double greaterRadius = Math.max(leftArcRadius, rightArcRadius);

                leftSpeed = leftArcRadius / greaterRadius;
                rightSpeed = rightArcRadius / greaterRadius;

                telemetry.addData("Greater Radius", greaterRadius);
                telemetry.addData("Left Speed PM", leftSpeed);
                telemetry.addData("Right Speed PM", rightSpeed);

                // speed mixing
                // controls how low speed turns and strafes interact
                double speedFactor = MathUtilities.constrain(inputMag + Math.abs(rot), 0, 1);
                leftSpeed *= speedFactor;
                rightSpeed *= speedFactor;
            } else {
                leftSpeed = rightSpeed = Math.abs(rot);
            }
        }

        telemetry.addData("Left Angle", leftAngle);
        telemetry.addData("Right Angle", rightAngle);
        telemetry.addData("Left Speed", leftSpeed);
        telemetry.addData("Right Speed", rightSpeed);

        if (inputMag > 0.05 || Math.abs(rot) > 0.05) {
            leftModule.setTargetDirection(leftAngle);
            rightModule.setTargetDirection(rightAngle);
        }
        if (leftModule.atTargetDirection()) {
            leftModule.setDrivePower(leftSpeed);
        } else {
            leftModule.setDrivePower(0);
        }

        if (rightModule.atTargetDirection()) {
            rightModule.setDrivePower(rightSpeed);
        } else {
            rightModule.setDrivePower(0);
        }
        update();
    }

    @Override
    public void resetEncoders() {
        leftModule.resetEncoder();
        rightModule.resetEncoder();
    }

    public void resetSlewEncoders() {
        leftModule.resetSlewEncoder();
        rightModule.resetSlewEncoder();
    }

    public void update() {
        leftModule.update();
        rightModule.update();
    }
}
