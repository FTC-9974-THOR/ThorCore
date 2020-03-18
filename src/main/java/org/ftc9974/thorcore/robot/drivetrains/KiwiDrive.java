package org.ftc9974.thorcore.robot.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.navigation.NavSource;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.Motor;
import org.ftc9974.thorcore.util.MathUtilities;

public final class KiwiDrive implements HolonomicDrivetrain {

    @SuppressWarnings("WeakerAccess")
    @Hardware
    public DcMotorEx wheel1, wheel2, wheel3;

    private NavSource navSource;
    private PIDF turningPidf;
    private double fieldRelativeOffset;
    private boolean isFieldRelative;
    private boolean turningPidfActive;
    private double lastRot;
    private boolean lastUpdateWasZeroControl;
    private double lastError, lastErrorDerivative;
    private boolean calculusActive;

    private static final double W1_ANGLE = Math.toRadians(-60),
                                W2_ANGLE = Math.toRadians(60),
                                W3_ANGLE = Math.toRadians(180),
                                TURNING_EDGE_DEADBAND = 0.1;

    public KiwiDrive(HardwareMap hardwareMap, PIDFCoefficients turningCoefficients, NavSource navSource) {
        this(hardwareMap, turningCoefficients, navSource, (byte) 0b000);
    }

    public KiwiDrive(HardwareMap hardwareMap, PIDFCoefficients turningCoefficients, NavSource navSource, byte inversion) {
        Realizer.realize(this, hardwareMap);
        turningPidf = new PIDF(turningCoefficients);
        turningPidf.setContinuous(true);
        turningPidf.setContinuityRange(-Math.PI, Math.PI);
        if (navSource != null) {
            this.navSource = navSource;
        }
        if ((inversion & 0b100) == 0b100) {
            wheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if ((inversion & 0b010) == 0b010) {
            wheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if ((inversion & 0b001) == 0b001) {
            wheel3.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void setFieldRelative(boolean isFieldRelative) {
        this.isFieldRelative = isFieldRelative;
        if (isFieldRelative) {
            setTargetHeading(getTargetHeading() - fieldRelativeOffset);
        }
    }

    public boolean isFieldRelative() {
        return isFieldRelative;
    }

    public void setTargetHeading(double targetHeading) {
        turningPidf.setSetpoint(targetHeading);
    }

    public double getTargetHeading() {
        return turningPidf.getSetpoint();
    }

    public void setHeadingDeadband(double deadband) {
        turningPidf.setAtTargetThreshold(deadband);
    }

    public double getHeadingDeadband() {
        return turningPidf.getAtTargetThreshold();
    }

    public void resetFieldForward() {
        fieldRelativeOffset = navSource.getHeading();
    }

    public void setTurningPidfActive(boolean active) {
        turningPidfActive = active;
        if (active) {
            turningPidf.resetControl();
        }
    }

    public boolean isTurningPidfActive() {
        return turningPidfActive;
    }

    @Override
    public void drive(double x, double y, double rot) {
        double _y = -y;
        double currentHeading;
        if (isFieldRelative) {
            currentHeading = navSource.getHeading() - fieldRelativeOffset;
        } else {
            currentHeading = navSource.getHeading();
        }

        double _rot = 0;
        if (turningPidfActive) {
            if (Math.abs(rot) > TURNING_EDGE_DEADBAND) {
                // rising edge (turn starts)
                _rot = rot;
            } else if (Math.abs(rot) < TURNING_EDGE_DEADBAND) {
                // not actively turning
                if (Math.abs(lastRot) > TURNING_EDGE_DEADBAND) {
                    // falling edge (turn stops)
                    setTargetHeading(currentHeading);
                    turningPidf.resetControl();
                }
                _rot = turningPidf.update(currentHeading);
            }
        } else {
            _rot = rot;
        }

        if (MathUtilities.applyDeadband(_y, 0.1) == 0 &&
                MathUtilities.applyDeadband(x, 0.1) == 0 &&
                MathUtilities.applyDeadband(rot, 0.1) == 0) {
            if (!lastUpdateWasZeroControl) {
                lastUpdateWasZeroControl = true;
                setMotorZeroPowerBehaviours(DcMotor.ZeroPowerBehavior.BRAKE);
                calculusActive = true;
                lastError = 0;
                lastErrorDerivative = 0;
            }
        } else {
            if (lastUpdateWasZeroControl) {
                setMotorZeroPowerBehaviours(DcMotor.ZeroPowerBehavior.FLOAT);
                calculusActive = false;
            }
            lastUpdateWasZeroControl = false;
        }

        if (calculusActive) {
            // We don't need the true derivative, we really only need the delta.
            // However, the concept is the same.
            double dE = Math.abs(turningPidf.getLastError() - lastError);
            if (dE < lastErrorDerivative) {
                setTargetHeading(currentHeading);
                calculusActive = false;
            }
            lastError = turningPidf.getLastError();
            lastErrorDerivative = dE;
        }

        if (!isFieldRelative) {
            currentHeading = 0;
        }

        double w1Vector = Math.cos(W1_ANGLE + currentHeading) * x +
                Math.sin(W1_ANGLE + currentHeading) * _y +
                _rot;
        double w2Vector = Math.cos(W2_ANGLE + currentHeading) * x +
                Math.sin(W2_ANGLE + currentHeading) * _y +
                _rot;
        double w3Vector = Math.cos(W3_ANGLE + currentHeading) * x +
                Math.sin(W3_ANGLE + currentHeading) * _y +
                _rot;

        double maxVector = MathUtilities.max(Math.abs(w1Vector), Math.abs(w2Vector), Math.abs(w3Vector));
        w1Vector /= maxVector;
        w2Vector /= maxVector;
        w3Vector /= maxVector;

        double maxInput = MathUtilities.max(Math.hypot(Math.abs(x), Math.abs(_y)), Math.abs(_rot));
        w1Vector *= maxInput;
        w2Vector *= maxInput;
        w3Vector *= maxInput;

        wheel1.setPower(w1Vector);
        wheel2.setPower(w2Vector);
        wheel3.setPower(w3Vector);

        lastRot = rot;
    }

    private void setMotorZeroPowerBehaviours(DcMotor.ZeroPowerBehavior behaviour) {
        wheel1.setZeroPowerBehavior(behaviour);
        wheel2.setZeroPowerBehavior(behaviour);
        wheel3.setZeroPowerBehavior(behaviour);
    }

    public double getFieldRelativeOffset() {
        return fieldRelativeOffset;
    }

    public boolean isCalculusActive() {
        return calculusActive;
    }
}
