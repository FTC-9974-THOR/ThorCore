package org.ftc9974.thorcore.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.ftc9974.thorcore.internal.RealizableFactory;

import java.util.Locale;

/**
 * Class that can interact with both SparkMinis and actual motors. Behaves like a regular
 * {@link DcMotor}.
 */
public final class Motor implements DcMotorEx, OpModeManagerNotifier.Notifications {

    private DcMotorEx dcMotor;
    private CRServo sparkMini;

    private StallDetector stallDetector;

    private enum Mode {
        DC_MOTOR,
        SPARK_MINI
    }

    private Mode mode;

    /**
     * Creates a new Motor instance, looking for a device with the given name.
     * @param name name of device to look for
     * @param hardwareMap hardwareMap to look in
     */
    @RealizableFactory
    public Motor(String name, HardwareMap hardwareMap) {
        if (hardwareMap.dcMotor.contains(name)) {
            mode = Mode.DC_MOTOR;
            dcMotor = (DcMotorEx) hardwareMap.dcMotor.get(name);
            dcMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
            stallDetector = new StallDetector(this, 0.25 * Math.PI);
        } else if (hardwareMap.crservo.contains(name)) {
            mode = Mode.SPARK_MINI;
            sparkMini = hardwareMap.crservo.get(name);
        } else {
            throw new IllegalArgumentException(String.format(Locale.getDefault(),
                    "No device with name \"%s\" found.", name));
        }
    }

    public boolean isMotor() {
        return mode == Mode.DC_MOTOR;
    }

    public boolean isSparkMini() {
        return mode == Mode.SPARK_MINI;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getMotorType();
        } else {
            return MotorConfigurationType.getUnspecifiedMotorType();
        }
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        if (mode == Mode.DC_MOTOR) {
            dcMotor.setMotorType(motorType);
        }
    }

    @Override
    public DcMotorController getController() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getController();
        } else {
            return null;
        }
    }

    @Override
    public int getPortNumber() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getPortNumber();
        } else {
            return sparkMini.getPortNumber();
        }
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        // while the spark minis can do this, it is a physical switch on the controller
        if (mode == Mode.DC_MOTOR) {
            dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getZeroPowerBehavior();
        } else {
            return ZeroPowerBehavior.UNKNOWN;
        }
    }

    @Deprecated
    @Override
    public void setPowerFloat() {
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0);
    }

    @Override
    public boolean getPowerFloat() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getPowerFloat();
        } else {
            return false;
        }
    }

    @Override
    public void setTargetPosition(int position) {
        if (mode == Mode.DC_MOTOR) {
            dcMotor.setTargetPosition(position);
        }
    }

    @Override
    public int getTargetPosition() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getTargetPosition();
        } else {
            return 0;
        }
    }

    @Override
    public boolean isBusy() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.isBusy();
        } else {
            return false;
        }
    }

    @Override
    public int getCurrentPosition() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getCurrentPosition();
        } else {
            return 0;
        }
    }

    @Override
    public void setMode(RunMode mode) {
        if (this.mode == Mode.DC_MOTOR) {
            dcMotor.setMode(mode);
        }
    }

    @Override
    public RunMode getMode() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getMode();
        } else {
            return RunMode.RUN_WITHOUT_ENCODER;
        }
    }

    @Override
    public void setDirection(Direction direction) {
        if (mode == Mode.DC_MOTOR) {
            dcMotor.setDirection(direction);
        } else {
            sparkMini.setDirection(direction);
        }
    }

    @Override
    public Direction getDirection() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getDirection();
        } else {
            return sparkMini.getDirection();
        }
    }

    @Override
    public void setPower(double power) {
        if (mode == Mode.DC_MOTOR) {
            dcMotor.setPower(power);
        } else {
            sparkMini.setPower(power);
        }
    }

    @Override
    public double getPower() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getPower();
        } else {
            return sparkMini.getPower();
        }
    }

    @Override
    public Manufacturer getManufacturer() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getManufacturer();
        } else {
            // IMO, there should be a REV option
            return Manufacturer.Other;
        }
    }

    @Override
    public String getDeviceName() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getDeviceName();
        } else {
            return "REV Spark Mini";
        }
    }

    @Override
    public String getConnectionInfo() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getConnectionInfo();
        } else {
            return "REV Spark Mini - " + sparkMini.getConnectionInfo();
        }
    }

    @Override
    public int getVersion() {
        if (mode == Mode.DC_MOTOR) {
            return dcMotor.getVersion();
        } else {
            return 1;
        }
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        if (mode == Mode.DC_MOTOR) {
            dcMotor.resetDeviceConfigurationForOpMode();
        } else {
            sparkMini.resetDeviceConfigurationForOpMode();
        }
    }

    @Override
    public void close() {
        if (mode == Mode.DC_MOTOR) {
            dcMotor.close();
        } else {
            sparkMini.close();
        }
    }

    @Override
    public void setMotorEnable() {
        if (isMotor()) {
            dcMotor.setMotorEnable();
        }
    }

    @Override
    public void setMotorDisable() {
        if (isMotor()) {
            dcMotor.setMotorDisable();
        }
    }

    @Override
    public boolean isMotorEnabled() {
        return isMotor() && dcMotor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        if (isMotor()) {
            dcMotor.setVelocity(angularRate);
        }
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        if (isMotor()) {
            dcMotor.setVelocity(angularRate, unit);
        }
    }

    @Override
    public double getVelocity() {
        return (isMotor()) ? dcMotor.getVelocity() : 0;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return (isMotor()) ? dcMotor.getVelocity(unit) : 0;
    }

    @Deprecated
    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        if (isMotor()) {
            dcMotor.setPIDCoefficients(mode, pidCoefficients);
        }
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        if (isMotor()) {
            dcMotor.setPIDFCoefficients(mode, pidfCoefficients);
        }
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        if (isMotor()) {
            dcMotor.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        if (isMotor()) {
            dcMotor.setPositionPIDFCoefficients(p);
        }
    }

    @Deprecated
    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return (isMotor()) ? dcMotor.getPIDCoefficients(mode) : null;
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return (isMotor()) ? dcMotor.getPIDFCoefficients(mode) : null;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        if (isMotor()) {
            dcMotor.setTargetPositionTolerance(tolerance);
        }
    }

    @Override
    public int getTargetPositionTolerance() {
        return (isMotor()) ? dcMotor.getTargetPositionTolerance() : 0;
    }

    public boolean isOnRevHub() {
        return dcMotor.getController().getManufacturer().equals(Manufacturer.Lynx);
    }

    public void setStallThreshold(double threshold) {
        if (isMotor()) {
            stallDetector.setStallThreshold(threshold);
        }
    }

    public double getStallThreshold() {
        if (isMotor()) {
            return stallDetector.getStallThreshold();
        }
        return 0;
    }

    public boolean isStalled() {
        // I know I could use a short-circuited boolean AND,
        // but this is clearer
        if (isMotor()) {
            return stallDetector.isStalled();
        }
        return false;
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        if (isMotor()) {
            return dcMotor.getCurrent(unit);
        }
        return -1;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        if (isMotor()) {
            return dcMotor.getCurrentAlert(unit);
        }
        return -1;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        if (isMotor()) {
            dcMotor.setCurrentAlert(current, unit);
        }
    }

    @Override
    public boolean isOverCurrent() {
        if (isMotor()) {
            return dcMotor.isOverCurrent();
        }
        return false;
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        setMode(RunMode.RUN_WITHOUT_ENCODER);
    }
}
