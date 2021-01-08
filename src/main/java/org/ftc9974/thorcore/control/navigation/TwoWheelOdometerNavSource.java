package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.ftc9974.thorcore.control.RunningAverage;
import org.ftc9974.thorcore.control.SlidingAverageFilter;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.math.Vector3;
import org.ftc9974.thorcore.robot.sensors.OdometerWheel;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

// todo work in progress
public final class TwoWheelOdometerNavSource implements NavSource {

    private static final String TAG = "OdometerNavSource";

    public enum Algorithm {
        ZEROTH_ORDER,
        FIRST_ORDER,
        SECOND_ORDER,
        AUTO
    }

    // the variable names in these classes are derived from the symbols I used to denote them when
    // working out the kinematics, so some of them are cryptic.
    private static class RobotState {
        // heading of the robot
        double theta;
        // angular velocity of the robot
        double omega;
        // angular acceleration of the robot
        double alpha;
        // time
        double t;
    }

    private static class OdometerState {
        // position of the wheel
        double lambda;
        // velocity of the wheel (derivative of lambda)
        double dLambda;
        // circumference coefficient of the wheel
        double circCoef;
        // heading of the wheel direction vector
        double mu;
    }

    private final OdometerWheel wheelA, wheelB;
    // circumference coefficients for A and B wheels.
    private final double circCoefA, circCoefB;
    // heading of the direction vector for A and B wheels.
    private final double headingA, headingB;
    private final BNO055IMU imu;

    private RobotState lastRobotState;
    private OdometerState lastAState, lastBState;
    private double lastHeading;
    private int headingWrapIndex;

    private SlidingAverageFilter thetaFilter, omegaFilter;

    private Vector2 location;

    private Algorithm selectedAlgorithm;

    private boolean inCalibrationMode;
    private RunningAverage calibrationFactorAFilter, calibrationFactorBFilter;

    private double calibrationFactorA, calibrationFactorB;

    public TwoWheelOdometerNavSource(HardwareMap hw, OdometerWheel a, OdometerWheel b) {
        wheelA = a;
        wheelB = b;
        if (wheelA.getDirection().dot(wheelB.getDirection()) != 0) {
            // theoretically, it's possible to use any pair of wheels as long as they aren't
            // pointing the same direction. however, I've yet to work out the math necessary for
            // that. until then, wheels will have to be normal to each other.
            throw new RuntimeException("Odometer wheels need to be pointing at right angles to each other");
        }

        circCoefA = wheelA.getCartesianLocationOnRobot().crossMag(wheelA.getCartesianDirectionOnRobot());
        circCoefB = wheelB.getCartesianLocationOnRobot().crossMag(wheelB.getCartesianDirectionOnRobot());

        headingA = wheelA.getCartesianDirectionOnRobot().getHeading();
        headingB = wheelB.getCartesianDirectionOnRobot().getHeading();

        imu = hw.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(params);

        thetaFilter = new SlidingAverageFilter(3);
        omegaFilter = new SlidingAverageFilter(3);

        headingWrapIndex = 0;

        location = Vector2.ZERO;
        selectedAlgorithm = Algorithm.AUTO;

        calibrationFactorA = 1;
        calibrationFactorB = 1;
    }

    private void init() {
        lastRobotState = new RobotState();
        lastRobotState.theta = imu.getAngularOrientation(
                AxesReference.EXTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        ).thirdAngle;
        // z orientation is around the ground's normal vector
        // but angular velocity is straight from the gyro, relative to the rev hub itself
        // i thought, if you can use dot products to project positions, why not velocities?
        // and it worked
        Acceleration gravity = imu.getGravity();
        // the gravity vector points down, so up is in the opposite direction. find up by
        // multiplying gravity by -1. however, the imu's z axis points into the hub. thus, don't
        // negate zAccel, since positive is supposed to be up.
        Vector3 up = new Vector3(-gravity.xAccel, -gravity.yAccel, gravity.zAccel).normalized();
        AngularVelocity angVec = imu.getAngularVelocity();
        lastRobotState.omega = up.dot(new Vector3(
                angVec.xRotationRate,
                angVec.yRotationRate,
                angVec.zRotationRate
        ));
        lastRobotState.alpha = 0;
        lastRobotState.t = System.nanoTime();

        lastAState = new OdometerState();
        lastAState.lambda = wheelA.getPosition();
        lastAState.dLambda = wheelA.getVelocity();
        lastAState.circCoef = circCoefA;
        lastAState.mu = headingA;

        lastBState = new OdometerState();
        lastBState.lambda = wheelB.getPosition();
        lastBState.dLambda = wheelB.getVelocity();
        lastBState.circCoef = circCoefB;
        lastBState.mu = headingB;

        lastHeading = lastRobotState.theta;
    }

    @Override
    public void update() {
        RobotLog.vv(TAG, ">>> Starting update");
        if (lastRobotState == null) {
            init();
            return;
        }
        RobotState currentRobotState = new RobotState();
        currentRobotState.t = System.nanoTime();
        double deltaTime = (currentRobotState.t - lastRobotState.t) * (1e-9);
        RobotLog.vv(TAG, "Δtime: %.6f", deltaTime);
        fastReadSensorState();
        double currentHeading = imu.getAngularOrientation(
                AxesReference.EXTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        ).thirdAngle;
        double deltaHeading = currentHeading - lastHeading;
        RobotLog.vv(TAG, "deltaHeading: %f lastOmega: %f", deltaHeading, lastRobotState.omega);
        if (deltaHeading > 1.5 * Math.PI && lastRobotState.omega < 0) {
            // heading increased but angular velocity was negative
            // wrap clockwise -> negative
            headingWrapIndex--;
        } else if (deltaHeading < -1.5 * Math.PI && lastRobotState.omega > 0) {
            // heading decreased but angular velocity was positive
            // wrap counterclockwise -> positive
            headingWrapIndex++;
        }
        currentRobotState.theta = currentHeading + 2 * Math.PI * headingWrapIndex;
        // z orientation is around the ground's normal vector
        // but angular velocity is straight from the gyro, relative to the rev hub itself
        // i thought, if you can use dot products to project positions, why not velocities?
        // and it worked
        Acceleration gravity = imu.getGravity();
        // the gravity vector points down, so up is in the opposite direction. find up by
        // multiplying gravity by -1. however, the imu's z axis points into the hub. thus, don't
        // negate zAccel, since positive is supposed to be up.
        // todo calculate up vector with euler angles or quaternion
        Vector3 up = new Vector3(-gravity.xAccel, -gravity.yAccel, gravity.zAccel).normalized();
        AngularVelocity angVec = imu.getAngularVelocity();
        currentRobotState.omega = up.dot(new Vector3(
                angVec.xRotationRate,
                angVec.yRotationRate,
                angVec.zRotationRate
        ));
        currentRobotState.alpha = (lastRobotState.omega - currentRobotState.omega) / deltaTime;
        RobotLog.vv(TAG, "lastHeading: %f currentHeading: %f headingWrapIndex: %d ~omega: %f", lastHeading, currentHeading, headingWrapIndex, (currentRobotState.theta - lastRobotState.theta) / deltaTime);

        OdometerState currentAState = new OdometerState();
        currentAState.lambda = wheelA.getPosition();
        currentAState.dLambda = wheelA.getVelocity();
        currentAState.circCoef = calibrationFactorA * circCoefA;
        currentAState.mu = headingA;

        OdometerState currentBState = new OdometerState();
        currentBState.lambda = wheelB.getPosition();
        currentBState.dLambda = wheelB.getVelocity();
        currentBState.circCoef = calibrationFactorB * circCoefB;
        currentBState.mu = headingB;

        RobotLog.vv(
                TAG,
                "Robot State\n" +
                        "theta0: %f thetaF: %f Δtheta: %f\n" +
                        "omega0: %f omegaF: %f Δomega: %f\n" +
                        "alpha0: %f alphaF: %f Δalpha: %f",
                lastRobotState.theta, currentRobotState.theta, currentRobotState.theta - lastRobotState.theta,
                lastRobotState.omega, currentRobotState.omega, currentRobotState.omega - lastRobotState.omega,
                lastRobotState.alpha, currentRobotState.alpha, currentRobotState.alpha - lastRobotState.alpha
        );
        RobotLog.vv(
                TAG,
                "Odometer A\n" +
                        "frameLocation: %s frameDirection: %s\n" +
                        "cartesianLocation: %s cartesianDirection: %s\n" +
                        "circCoef: %.3f\n" +
                        "lambda0: %.3f lambdaF: %.3f Δlambda: %.3f\n" +
                        "dLambda0: %.3f dLambdaF: %.3f ΔdLambda: %.3f\n",
                wheelA.getLocationOnRobot().toString(), wheelA.getDirection().toString(),
                wheelA.getCartesianLocationOnRobot().toString(), wheelA.getCartesianDirectionOnRobot().toString(),
                currentAState.circCoef,
                lastAState.lambda, currentAState.lambda, currentAState.lambda - lastAState.lambda,
                lastAState.dLambda, currentAState.dLambda, currentAState.dLambda - lastAState.dLambda
        );
        RobotLog.vv(
                TAG,
                "Odometer B\n" +
                        "frameLocation: %s frameDirection: %s\n" +
                        "cartesianLocation: %s cartesianDirection: %s\n" +
                        "circCoef: %.3f\n" +
                        "lambda0: %.3f lambdaF: %.3f Δlambda: %.3f\n" +
                        "dLambda0: %.3f dLambdaF: %.3f ΔdLambda: %.3f\n",
                wheelB.getLocationOnRobot().toString(), wheelB.getDirection().toString(),
                wheelB.getCartesianLocationOnRobot().toString(), wheelB.getCartesianDirectionOnRobot().toString(),
                currentBState.circCoef,
                lastBState.lambda, currentBState.lambda, currentBState.lambda - lastBState.lambda,
                lastBState.dLambda, currentBState.dLambda, currentBState.dLambda - lastBState.dLambda
        );

        //StringBuilder message = new StringBuilder();
        //message.append(String.format("theta0: %f thetaF: %f\n", )

        if (inCalibrationMode && currentRobotState.theta != lastRobotState.theta) {
            // interesting. circCoef is equal to the theta derivative of lambda.
            // or, approximately deltaLambda / deltaTheta.
            double deltaTheta = currentRobotState.theta - lastRobotState.theta;
            double rotationCorrection = deltaTheta * circCoefA;
            double deltaLambda = currentAState.lambda - lastAState.lambda;
            calibrationFactorAFilter.update(deltaLambda / rotationCorrection);
            rotationCorrection = deltaTheta * circCoefB;
            deltaLambda = currentBState.lambda - lastBState.lambda;
            calibrationFactorBFilter.update(deltaLambda / rotationCorrection);
            RobotLog.vv(TAG, "calibratedCircCoefA: %f calibratedCircCoefB: %f", calibrationFactorAFilter.get(), calibrationFactorBFilter.get());
        }

        Vector2 offsetA, offsetB;
        if (selectedAlgorithm == Algorithm.ZEROTH_ORDER) {
            RobotLog.vv(TAG, "Calculating offsetA");
            offsetA = zerothOrder(lastRobotState, currentRobotState, lastAState, currentAState);
            RobotLog.vv(TAG, "Calculating offsetB");
            offsetB = zerothOrder(lastRobotState, currentRobotState, lastBState, currentBState);
        } else if (selectedAlgorithm == Algorithm.FIRST_ORDER) {
            // per the fundamental theorem of calculus, use the antiderivative to calculate the
            // definite integral
            offsetA = firstOrderAntiderivative(
                    deltaTime,
                    lastRobotState, currentRobotState,
                    lastAState, currentAState
            ).subtract(firstOrderAntiderivative(
                    0,
                    lastRobotState, currentRobotState,
                    lastAState, currentAState
            ));
            offsetB = firstOrderAntiderivative(
                    deltaTime,
                    lastRobotState, currentRobotState,
                    lastBState, currentBState
            ).subtract(firstOrderAntiderivative(
                    0,
                    lastRobotState, currentRobotState,
                    lastBState, currentBState
            ));
        } else if (selectedAlgorithm == Algorithm.SECOND_ORDER) {
            offsetA = Vector2.ZERO;
            offsetB = Vector2.ZERO;
        } else {
            if ((lastRobotState.theta == currentRobotState.theta && lastRobotState.omega == currentRobotState.omega) || lastRobotState.omega == 0) {
                RobotLog.vv(TAG, "selected zeroth order algorithm");
                offsetA = zerothOrder(lastRobotState, currentRobotState, lastAState, currentAState);
                offsetB = zerothOrder(lastRobotState, currentRobotState, lastBState, currentBState);
            } else {
                RobotLog.vv(TAG, "selected first order algorithm");
                // per the fundamental theorem of calculus, use the antiderivative to calculate the
                // definite integral
                offsetA = firstOrderAntiderivative(
                        deltaTime,
                        lastRobotState, currentRobotState,
                        lastAState, currentAState
                ).subtract(firstOrderAntiderivative(
                        0,
                        lastRobotState, currentRobotState,
                        lastAState, currentAState
                ));
                offsetB = firstOrderAntiderivative(
                        deltaTime,
                        lastRobotState, currentRobotState,
                        lastBState, currentBState
                ).subtract(firstOrderAntiderivative(
                        0,
                        lastRobotState, currentRobotState,
                        lastBState, currentBState
                ));
            }
        }
        RobotLog.vv(TAG, "selectedAlgorithm: %s offsetA: %s offsetB: %s",
                selectedAlgorithm.toString(), offsetA.toString(), offsetB.toString());
        Vector2 deltaLocation = offsetA.add(offsetB);
        location = location.add(deltaLocation);

        lastHeading = currentHeading;
        lastRobotState = currentRobotState;
        lastAState = currentAState;
        lastBState = currentBState;
    }

    private Vector2 zerothOrder(RobotState rbState0, RobotState rbStateF, OdometerState odState0, OdometerState odStateF) {
        // change in wheel position
        double deltaLambda = odStateF.lambda - odState0.lambda;
        // change in wheel position due to rotation
        double rotationCorrection = (rbStateF.theta - rbState0.theta) * odState0.circCoef;
        RobotLog.vv(TAG, "calib: %f", deltaLambda / rotationCorrection);
        // change in wheel position due to translation
        double beta = deltaLambda - rotationCorrection;
        RobotLog.vv(TAG, "deltaLambda: %f rotationCorrection: %.3f beta: %.3f", deltaLambda, rotationCorrection, beta);
        // heading of wheel
        double tau = rbState0.theta + odState0.mu;
        // beta * direction of wheel
        return new Vector2(beta * Math.cos(tau), beta * Math.sin(tau));
    }

    // treats angular velocity and wheel velocity as constants
    private Vector2 firstOrderAntiderivative(double t, RobotState rbState0, RobotState rbStateF, OdometerState odState0, OdometerState odStateF) {
        // heading of the robot
        double tau = rbState0.omega * t + rbState0.theta;
        // derivative of change in wheel position due to translation
        double dBeta = odState0.dLambda - rbState0.omega * odState0.circCoef;
        RobotLog.vv(TAG, "tau: %f dLambda: %f rotationCorrection: %f dBeta: %f ~omega: %f", tau, odState0.dLambda, rbState0.omega * odState0.circCoef, dBeta, (rbStateF.theta - rbState0.theta) / (rbStateF.t - rbState0.t));
        // dBeta * integral of direction vector of the wheel / angular velocity
        return new Vector2(dBeta * Math.sin(tau + odState0.mu) / rbState0.omega, -dBeta * Math.cos(tau + odState0.mu) / rbState0.omega);
    }

    // max loop time: 0.3s
    // max omega: 2 rad/s
    // max alpha: 53 rad/s^2
    // max fresnel param: 1.37
    // i'll do this as soon as i figure out how to approximate the fresnel integrals
    // treats angular acceleration and wheel acceleration as constants
    private Vector2 secondOrderAntiderivative(double t, RobotState rbState0, RobotState rbStateF, OdometerState odState0, OdometerState odStateF) {
        // todo
        return null;
    }

    // invokes deep magic to synchronize encoder and imu reads as much as possible
    private double[] fastReadSensorState() {
        double lambdaA, dLambdaA, lambdaB, dLambdaB;
        ByteBuffer imuData = ByteBuffer.wrap(imu.read(BNO055IMU.Register.GYR_DATA_X_LSB, 12)).order(ByteOrder.LITTLE_ENDIAN);
        if (wheelA.getConnectedModule().equals(wheelB.getConnectedModule())) {
            LynxModule.BulkData bulkData = wheelA.getConnectedModule().getBulkData();
            lambdaA = bulkData.getMotorCurrentPosition(wheelA.getEncoderMotor().getPortNumber());
            dLambdaA = bulkData.getMotorVelocity(wheelA.getEncoderMotor().getPortNumber());
            lambdaB = bulkData.getMotorCurrentPosition(wheelB.getEncoderMotor().getPortNumber());
            dLambdaB = bulkData.getMotorVelocity(wheelB.getEncoderMotor().getPortNumber());
        } else {
            // if the wheels are on different rev hubs, 2 bulk data reads must be issued
            // this is about 2ms slower
            LynxModule.BulkData bulkDataA = wheelA.getConnectedModule().getBulkData();
            LynxModule.BulkData bulkDataB = wheelB.getConnectedModule().getBulkData();
            lambdaA = bulkDataA.getMotorCurrentPosition(wheelA.getEncoderMotor().getPortNumber());
            dLambdaA = bulkDataA.getMotorVelocity(wheelA.getEncoderMotor().getPortNumber());
            lambdaB = bulkDataB.getMotorCurrentPosition(wheelB.getEncoderMotor().getPortNumber());
            dLambdaB = bulkDataB.getMotorVelocity(wheelB.getEncoderMotor().getPortNumber());
        }
        lambdaA *= wheelA.getMmPerTick();
        dLambdaA *= wheelA.getMmPerTick();
        lambdaB *= wheelB.getMmPerTick();
        dLambdaB *= wheelB.getMmPerTick();
        // dividing by 900 because we're using radians
        // see BNO055IMUImpl#getAngularVelocity() and BNO055IMUImpl#getAngularScale()

        double omegaZ = -imuData.getShort() / 900.0;
        double omegaY = imuData.getShort() / 900.0;
        double omegaX = imuData.getShort() / 900.0;
        double thetaZ = AngleUnit.normalizeRadians(-imuData.getShort() / 900.0);
        double thetaY = AngleUnit.normalizeRadians(imuData.getShort() / 900.0);
        double thetaX = AngleUnit.normalizeRadians(imuData.getShort() / 900.0);

        RobotLog.vv(TAG, "deep magic -> lambdaA: %f dLambdaA: %f lambdaB: %f dLambdaB: %f " +
                        "thetaX: %f thetaY: %f thetaZ: %f omegaX: %f omegaY: %f omegaZ: %f",
                lambdaA, dLambdaA, lambdaB, dLambdaB,
                thetaX, thetaY, thetaZ,
                omegaX, omegaY, omegaZ);
        Orientation orientation = imu.getAngularOrientation(
                AxesReference.EXTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        );
        AngularVelocity angularVelocity = imu.getAngularVelocity();
        thetaZ = orientation.thirdAngle;
        thetaY = orientation.secondAngle;
        thetaX = orientation.firstAngle;
        omegaZ = angularVelocity.zRotationRate;
        omegaY = angularVelocity.yRotationRate;
        omegaX = angularVelocity.xRotationRate;
        RobotLog.vv(TAG, "              lambdaA: %f dLambdaA: %f lambdaB: %f dLambdaB: %f " +
                "thetaX: %f thetaY: %f thetaZ: %f omegaX: %f omegaY: %f omegaZ: %f",
                lambdaA, dLambdaA, lambdaB, dLambdaB,
                thetaX, thetaY, thetaZ,
                omegaX, omegaY, omegaZ);
        return new double[] {
                lambdaA, dLambdaA,
                lambdaB, dLambdaB,
                thetaX, thetaY, thetaZ,
                omegaX, omegaY, omegaZ
        };
    }

    public void setLocation(Vector2 location) {
        this.location = location;
    }

    @Override
    public Vector2 getLocation() {
        // the imu updates at 100Hz, so only run update() every 10ms at most
        if (lastRobotState == null || System.nanoTime() - lastRobotState.t > 1e7) update();
        return location;
    }

    @Override
    public double getHeading() {
        if (lastRobotState == null || System.nanoTime() - lastRobotState.t > 1e7) update();
        return lastHeading;
    }

    public void useAlgorithm(Algorithm algorithm) {
        selectedAlgorithm = algorithm;
    }

    public double getTheta() {
        return lastRobotState.theta;
    }

    public int getHeadingWrapIndex() {
        return headingWrapIndex;
    }

    public void activateCalibrationMode() {
        inCalibrationMode = true;
        calibrationFactorAFilter = new RunningAverage();
        calibrationFactorBFilter = new RunningAverage();
        calibrationFactorA = 1;
        calibrationFactorB = 1;
    }

    public void setCalibrationFactorA(double factor) {
        calibrationFactorA = factor;
    }

    public double getCalibrationFactorA() {
        if (inCalibrationMode && calibrationFactorAFilter.numSamples() > 0) {
            return calibrationFactorAFilter.get();
        }
        return calibrationFactorA;
    }

    public void setCalibrationFactorB(double factor) {
        calibrationFactorB = factor;
    }

    public double getCalibrationFactorB() {
        if (inCalibrationMode && calibrationFactorBFilter.numSamples() > 0) {
            return calibrationFactorBFilter.get();
        }
        return calibrationFactorB;
    }

    public void resetCalibration() {
        if (inCalibrationMode) {
            calibrationFactorAFilter.reset();
            calibrationFactorBFilter.reset();
        }
    }

    public BNO055IMU.CalibrationStatus getIMUCalibrationStatus() {
        return imu.getCalibrationStatus();
    }

    @Override
    public boolean trustworthy() {
        return true;
    }
}
