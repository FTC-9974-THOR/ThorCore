package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.util.MathUtilities;

import java.util.Arrays;
import java.util.Locale;

/**
 * A navigation strategy that uses encoders, the IMU, and Vuforia.
 *
 * It is the end-all, be-all navigation system.
 *
 * 12/30/18 12:26 AM: THE CODE GODS HAVE TESTED ME AND I HAVE BESTED THEM!
 */
public class SensorFusionNavStrategy implements NavSource, MovementStrategy {

    public class DiagnosticData {
        public State state;
        public Vector2 lastKnownLocation, targetPosition, toTarget;
        public double encProgress;
        public int[] encTargets, encOffsets;
        public double[] progress;
        public int[] errors;

        private DiagnosticData(State state, Vector2 lastKnownLocation, Vector2 targetPosition, Vector2 toTarget, double encProgress, int[] encTargets, int[] encOffsets, double[] progress, int[] errors) {
            this.state = state;
            this.lastKnownLocation = lastKnownLocation;
            this.targetPosition = targetPosition;
            this.toTarget = toTarget;
            this.encProgress = encProgress;
            this.encTargets = encTargets;
            this.encOffsets = encOffsets;
            this.progress = progress;
            this.errors = errors;
        }
    }

    public enum State {
        ENCODER_DRIVE,
        PIDF_DRIVE,
        TURN_AFTER_DRIVE
    }

    private final double startRampDistance = 0.1, endRampDistance = 0.5;

    private VuMarkNavSource vuMarkNavSource;
    private IMUNavSource imuNavSource;
    private PIDFMovementStrategy pidfMovementStrategy;

    private Vector2 lastKnownLocation;
    private double encProgress;
    private Vector2 encMotionStart, targetPosition, toTarget;
    private double startHeading;
    private int[] encTargets;
    private int[] encOffsets;
    private HardwareMap.DeviceMapping<VoltageSensor> voltageSensors;
    private EncoderPositionCalculator calculator;
    private double speedLimit;
    private double[] progress;
    private int[] errors;
    private boolean vuforiaEnabled;

    private State state;

    private final Object lock = new Object();

    public SensorFusionNavStrategy(VuforiaLocalizer localizer, HardwareMap hardwareMap, String vuforiaKey, VuforiaLocalizer.CameraDirection direction, OpenGLMatrix phoneLocation, double xKp, double xKi, double xKd, double xKf, double yKp, double yKi, double yKd, double yKf, double tKp, double tKi, double tKd, double tKf, double xDb, double yDb, double tDb, double xTh, double yTh, double tTh, Vector2 startPos, double startHeading, EncoderPositionCalculator calculator) {
        this(localizer, hardwareMap, vuforiaKey, ClassFactory.getInstance().getCameraManager().nameForUnknownCamera(), direction, phoneLocation, false, xKp, xKi, xKd, xKf, yKp, yKi, yKd, yKf, tKp, tKi, tKd, tKf, xDb, yDb, tDb, xTh, yTh, tTh, startPos, startHeading, calculator);
    }

    public SensorFusionNavStrategy(VuforiaLocalizer localizer, HardwareMap hardwareMap, String vuforiaKey, CameraName cameraName, OpenGLMatrix phoneLocation, double xKp, double xKi, double xKd, double xKf, double yKp, double yKi, double yKd, double yKf, double tKp, double tKi, double tKd, double tKf, double xDb, double yDb, double tDb, double xTh, double yTh, double tTh, Vector2 startPos, double startHeading, EncoderPositionCalculator calculator) {
        this(localizer, hardwareMap, vuforiaKey, cameraName, VuforiaLocalizer.CameraDirection.BACK, phoneLocation, true, xKp, xKi, xKd, xKf, yKp, yKi, yKd, yKf, tKp, tKi, tKd, tKf, xDb, yDb, tDb, xTh, yTh, tTh, startPos, startHeading, calculator);
    }

    public SensorFusionNavStrategy(HardwareMap hardwareMap, String vuforiaKey, VuforiaLocalizer.CameraDirection direction, OpenGLMatrix phoneLocation, double xKp, double xKi, double xKd, double xKf, double yKp, double yKi, double yKd, double yKf, double tKp, double tKi, double tKd, double tKf, double xDb, double yDb, double tDb, double xTh, double yTh, double tTh, Vector2 startPos, double startHeading, EncoderPositionCalculator calculator) {
        this(null, hardwareMap, vuforiaKey, ClassFactory.getInstance().getCameraManager().nameForUnknownCamera(), direction, phoneLocation, false, xKp, xKi, xKd, xKf, yKp, yKi, yKd, yKf, tKp, tKi, tKd, tKf, xDb, yDb, tDb, xTh, yTh, tTh, startPos, startHeading, calculator);
    }

    public SensorFusionNavStrategy(HardwareMap hardwareMap, String vuforiaKey, CameraName cameraName, OpenGLMatrix phoneLocation, double xKp, double xKi, double xKd, double xKf, double yKp, double yKi, double yKd, double yKf, double tKp, double tKi, double tKd, double tKf, double xDb, double yDb, double tDb, double xTh, double yTh, double tTh, Vector2 startPos, double startHeading, EncoderPositionCalculator calculator) {
        this(null, hardwareMap, vuforiaKey, cameraName, VuforiaLocalizer.CameraDirection.BACK, phoneLocation, true, xKp, xKi, xKd, xKf, yKp, yKi, yKd, yKf, tKp, tKi, tKd, tKf, xDb, yDb, tDb, xTh, yTh, tTh, startPos, startHeading, calculator);
    }

    public SensorFusionNavStrategy(VuforiaLocalizer localizer, HardwareMap hardwareMap, String vuforiaKey, VuforiaLocalizer.CameraDirection direction, OpenGLMatrix phoneLocation, PIDFCoefficients xCoefficients, PIDFCoefficients yCoefficients, PIDFCoefficients thetaCoefficients, double xDb, double yDb, double tDb, double xTh, double yTh, double tTh, Vector2 startPos, double startHeading, EncoderPositionCalculator calculator) {
        this(localizer, hardwareMap, vuforiaKey, ClassFactory.getInstance().getCameraManager().nameForUnknownCamera(), direction, phoneLocation, false, xCoefficients.p, xCoefficients.i, xCoefficients.d, xCoefficients.f, yCoefficients.p, yCoefficients.i, yCoefficients.d, yCoefficients.f, thetaCoefficients.p, thetaCoefficients.i, thetaCoefficients.d, thetaCoefficients.f, xDb, yDb, tDb, xTh, yTh, tTh, startPos, startHeading, calculator);
    }

    public SensorFusionNavStrategy(VuforiaLocalizer localizer, HardwareMap hardwareMap, String vuforiaKey, CameraName cameraName, OpenGLMatrix phoneLocation, PIDFCoefficients xCoefficients, PIDFCoefficients yCoefficients, PIDFCoefficients thetaCoefficients, double xDb, double yDb, double tDb, double xTh, double yTh, double tTh, Vector2 startPos, double startHeading, EncoderPositionCalculator calculator) {
        this(localizer, hardwareMap, vuforiaKey, cameraName, VuforiaLocalizer.CameraDirection.BACK, phoneLocation, true, xCoefficients.p, xCoefficients.i, xCoefficients.d, xCoefficients.f, yCoefficients.p, yCoefficients.i, yCoefficients.d, yCoefficients.f, thetaCoefficients.p, thetaCoefficients.i, thetaCoefficients.d, thetaCoefficients.f, xDb, yDb, tDb, xTh, yTh, tTh, startPos, startHeading, calculator);
    }

    public SensorFusionNavStrategy(HardwareMap hardwareMap, String vuforiaKey, VuforiaLocalizer.CameraDirection direction, OpenGLMatrix phoneLocation, PIDFCoefficients xCoefficients, PIDFCoefficients yCoefficients, PIDFCoefficients thetaCoefficients, double xDb, double yDb, double tDb, double xTh, double yTh, double tTh, Vector2 startPos, double startHeading, EncoderPositionCalculator calculator) {
        this(null, hardwareMap, vuforiaKey, ClassFactory.getInstance().getCameraManager().nameForUnknownCamera(), direction, phoneLocation, false, xCoefficients.p, xCoefficients.i, xCoefficients.d, xCoefficients.f, yCoefficients.p, yCoefficients.i, yCoefficients.d, yCoefficients.f, thetaCoefficients.p, thetaCoefficients.i, thetaCoefficients.d, thetaCoefficients.f, xDb, yDb, tDb, xTh, yTh, tTh, startPos, startHeading, calculator);
    }

    public SensorFusionNavStrategy(HardwareMap hardwareMap, String vuforiaKey, CameraName cameraName, OpenGLMatrix phoneLocation, PIDFCoefficients xCoefficients, PIDFCoefficients yCoefficients, PIDFCoefficients thetaCoefficients, double xDb, double yDb, double tDb, double xTh, double yTh, double tTh, Vector2 startPos, double startHeading, EncoderPositionCalculator calculator) {
        this(null, hardwareMap, vuforiaKey, cameraName, VuforiaLocalizer.CameraDirection.BACK, phoneLocation, true, xCoefficients.p, xCoefficients.i, xCoefficients.d, xCoefficients.f, yCoefficients.p, yCoefficients.i, yCoefficients.d, yCoefficients.f, thetaCoefficients.p, thetaCoefficients.i, thetaCoefficients.d, thetaCoefficients.f, xDb, yDb, tDb, xTh, yTh, tTh, startPos, startHeading, calculator);
    }

    // now that's a lot of arguments
    public SensorFusionNavStrategy(VuforiaLocalizer localizer, HardwareMap hardwareMap, String vuforiaKey, CameraName cameraName, VuforiaLocalizer.CameraDirection direction, OpenGLMatrix cameraLocation, boolean isWebcam, double xKp, double xKi, double xKd, double xKf, double yKp, double yKi, double yKd, double yKf, double tKp, double tKi, double tKd, double tKf, double xDb, double yDb, double tDb, double xTh, double yTh, double tTh, Vector2 startPos, double startHeading, EncoderPositionCalculator calculator) {
        vuMarkNavSource = new VuMarkNavSource(localizer, hardwareMap.appContext, vuforiaKey, cameraName, direction, cameraLocation, isWebcam);
        imuNavSource = new IMUNavSource(hardwareMap, 3);
        pidfMovementStrategy = new PIDFMovementStrategy(xKp, xKi, xKd, xKf, yKp, yKi, yKd, yKf, tKp, tKi, tKd, tKf, xDb, yDb, tDb, xTh, yTh, tTh);
        pidfMovementStrategy.setContinuity(startHeading - Math.PI, startHeading + Math.PI);
        state = State.PIDF_DRIVE;
        lastKnownLocation = startPos;
        encTargets = new int[calculator.calculate(Vector2.ZERO).length];
        encOffsets = encTargets;
        this.startHeading = startHeading;
        this.calculator = calculator;
        voltageSensors = hardwareMap.voltageSensor;
        speedLimit = 1;
        progress = new double[encTargets.length];
        errors = new int[encTargets.length];
    }

    public void updatePosition(Vector2 position) {
        synchronized (lock) {
            lastKnownLocation = position;
            reset();
        }
    }

    @Override
    public double[] calculateMovement(HolonomicDrivetrain drivetrain, Vector2 currentPosition, double currentHeading, Vector2 targetPosition, double targetHeading) {
        synchronized (lock) {
            if (drivetrain.getEncoderPositions().length != calculator.calculate(Vector2.ZERO).length) {
                throw new RuntimeException(String.format(Locale.getDefault(), "The drivetrain (%s) and the encoder position calculator (%s) do not expect the same amount of encoders (drivetrain expected %d, encoder position calculator expected %d)", drivetrain.getClass().getSimpleName(), calculator.getClass().getSimpleName(), drivetrain.getEncoderPositions().length, calculator.calculate(Vector2.ZERO).length));
            }
            this.targetPosition = targetPosition;
            if (vuforiaEnabled && vuMarkNavSource.getNumVuMarksDetected() > 0) {
                if (state != State.PIDF_DRIVE) {
                    pidfMovementStrategy.reset();
                }
                state = State.PIDF_DRIVE;
                return pidfMovementStrategy.calculateMovement(drivetrain, currentPosition, currentHeading, targetPosition, targetHeading);
            } else if (state == State.PIDF_DRIVE) {
                // lost sight of VuMarks
                state = State.ENCODER_DRIVE;
                encOffsets = drivetrain.getEncoderPositions();
                toTarget = targetPosition.subtract(currentPosition).rotate(-currentHeading + 0.5 * Math.PI);
                int[] targets = calculator.calculate(toTarget);
                int[] offsetTargets = new int[encOffsets.length];
                for (int i = 0; i < targets.length; i++) {
                    offsetTargets[i] = encOffsets[i] + targets[i];
                }
                encTargets = offsetTargets;
                encMotionStart = currentPosition;
                return new double[] {0, 0, 0};
            } else if (state == State.ENCODER_DRIVE) {
                calculateEncoderProgress(drivetrain);
                if (atPositionalTarget()) {
                    state = State.TURN_AFTER_DRIVE;
                    return pidfMovementStrategy.calculateMovement(drivetrain, currentPosition, currentHeading, currentPosition, targetHeading);
                }
                double minVoltage = Double.POSITIVE_INFINITY;
                for (VoltageSensor voltageSensor : voltageSensors) {
                    if (voltageSensor.getVoltage() > 0) {
                        minVoltage = Math.min(voltageSensor.getVoltage(), minVoltage);
                    }
                }
                double effectiveSpeedLimit = speedLimit;
                if (toTarget.getMagnitude() < 250) {
                    effectiveSpeedLimit = MathUtilities.map(toTarget.getMagnitude(), 0, 250, 0.6 * speedLimit, speedLimit);
                }
                if (minVoltage > 0) {
                    if (encProgress < startRampDistance) {
                        double rampCoef = MathUtilities.map(encProgress, 0, startRampDistance, Math.max(0.6, 0.6 * effectiveSpeedLimit), effectiveSpeedLimit);
                        return new double[]{rampCoef * Math.cos(toTarget.getHeading()), rampCoef * Math.sin(toTarget.getHeading()), 0};
                    }
                    if (encProgress > 1 - endRampDistance) {
                        double rampCoef = MathUtilities.map(encProgress, 1 - endRampDistance, 1, effectiveSpeedLimit, Math.max(0.4, 0.4 * effectiveSpeedLimit));
                        return new double[]{rampCoef * Math.cos(toTarget.getHeading()), rampCoef * Math.sin(toTarget.getHeading()), 0};
                    }
                    return new double[]{effectiveSpeedLimit * Math.cos(toTarget.getHeading()), effectiveSpeedLimit * Math.sin(toTarget.getHeading()), 0};
                } else {
                    // this else branch could be changed, but it is left in for the sake of readability.
                    return new double[]{effectiveSpeedLimit * Math.cos(toTarget.getHeading()), effectiveSpeedLimit * Math.sin(toTarget.getHeading()), 0};
                }
            } else if (state == State.TURN_AFTER_DRIVE) {
                return new double[] {0, 0, pidfMovementStrategy.calculateMovement(drivetrain, currentPosition, currentHeading, currentPosition, targetHeading)[2]};
            }
            return new double[]{0, 0, 0};
        }
    }

    private void calculateEncoderProgress(HolonomicDrivetrain drivetrain) {
        progress = new double[encTargets.length];
        int[] current = drivetrain.getEncoderPositions();
        for (int i = 0; i < encTargets.length; i++) {
            errors[i] = Math.abs(encTargets[i] - current[i]);
            double target = (((double) encTargets[i]) - ((double) encOffsets[i]));
            if (MathUtilities.applyDeadband(errors[i], 20) == 0) {
                progress[i] = 1;
            } else {
                progress[i] = Math.abs(current[i] - encOffsets[i] / target);
            }
        }
        encProgress = MathUtilities.average(progress);
    }

    public void setSpeedLimit(double limit) {
        synchronized (lock) {
            speedLimit = limit;
        }
    }

    @Override
    public boolean atPositionalTarget() {
        if (state == State.PIDF_DRIVE) {
            return pidfMovementStrategy.atPositionalTarget();
        } else {
            return MathUtilities.average(errors) < 100 || encProgress > 0.95;
        }
    }

    @Override
    public boolean atHeadingTarget() {
        if (state == State.PIDF_DRIVE || state == State.TURN_AFTER_DRIVE) {
            return pidfMovementStrategy.atHeadingTarget();
        }
        return false;
    }

    @Override
    public void reset() {
        synchronized (lock) {
            pidfMovementStrategy.reset();
            Arrays.fill(progress, 0);
            Arrays.fill(errors, 0);
            encProgress = 0;
            state = State.PIDF_DRIVE;
        }
    }

    @Override
    public Vector2 getLocation() {
        if (vuforiaEnabled && vuMarkNavSource.getNumVuMarksDetected() > 0) {
            lastKnownLocation = vuMarkNavSource.getLocation();
            return lastKnownLocation;
        } else if (state == State.ENCODER_DRIVE || state == State.TURN_AFTER_DRIVE) {
            lastKnownLocation = MathUtilities.lerp(encMotionStart, targetPosition, encProgress);
        }
        return lastKnownLocation;
    }

    @Override
    public double getHeading() {
        return imuNavSource.getHeading() + startHeading;
    }

    @Override
    public boolean trustworthy() {
        return true;
    }

    public DiagnosticData getDiagnosticData() {
        return new DiagnosticData(state, lastKnownLocation, targetPosition, toTarget, encProgress, encTargets, encOffsets, progress, errors);
    }

    public void setVuforiaEnabled(boolean enabled) {
        synchronized (lock) {
            vuforiaEnabled = enabled;
        }
    }

    public boolean isVuforiaEnabled() {
        return vuforiaEnabled;
    }
}
