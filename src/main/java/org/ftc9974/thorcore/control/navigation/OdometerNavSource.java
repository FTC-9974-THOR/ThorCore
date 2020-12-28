package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.robot.sensors.OdometerWheel;

// todo work in progress
public final class OdometerNavSource implements NavSource {

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

    private Vector2 location;

    public OdometerNavSource(HardwareMap hw, OdometerWheel a, OdometerWheel b) {
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
    }

    private void init() {
        // todo
    }

    @Override
    public void update() {
        if (lastRobotState == null) {
            init();
            return;
        }
        RobotState currentRobotState = new RobotState();
        currentRobotState.t = System.nanoTime();
        double deltaTime = (currentRobotState.t - lastRobotState.t) * (1e-9);
        double currentHeading = imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        ).thirdAngle;
        double deltaHeading = currentHeading - lastHeading;
        if (deltaHeading > 0 && lastRobotState.omega < 0) {
            // heading increased but angular velocity was negative
            // wrap clockwise -> negative
            headingWrapIndex--;
        } else if (deltaHeading < 0 && lastRobotState.omega > 0) {
            // heading decreased but angular velocity was positive
            // wrap counterclockwise -> positive
            headingWrapIndex++;
        }
        currentRobotState.theta = currentHeading + 2 * Math.PI * headingWrapIndex;
        currentRobotState.omega = imu.getAngularVelocity().zRotationRate;
        currentRobotState.alpha = (lastRobotState.omega - currentRobotState.omega) / deltaTime;

        OdometerState currentAState = new OdometerState();
        currentAState.lambda = wheelA.getPosition();
        currentAState.dLambda = wheelA.getVelocity();
        currentAState.circCoef = circCoefA;
        currentAState.mu = headingA;

        OdometerState currentBState = new OdometerState();
        currentBState.lambda = wheelB.getPosition();
        currentBState.dLambda = wheelB.getVelocity();
        currentBState.circCoef = circCoefB;
        currentBState.mu = headingB;

        Vector2 offsetA, offsetB;
        if (lastRobotState.theta == currentRobotState.theta || lastRobotState.omega == 0) {
            offsetA = zerothOrder(lastRobotState, currentRobotState, lastAState, currentAState);
            offsetB = zerothOrder(lastRobotState, currentRobotState, lastBState, currentBState);
        } else {
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
        // change in wheel position due to translation
        double beta = deltaLambda - rotationCorrection;
        // heading of wheel
        double tau = rbState0.theta + odState0.mu;
        // beta * direction of wheel
        return new Vector2(beta * Math.cos(tau), beta * Math.sin(tau));
    }

    private Vector2 firstOrderAntiderivative(double t, RobotState rbState0, RobotState rbStateF, OdometerState odState0, OdometerState odStateF) {
        // heading of the robot
        double tau = rbState0.omega * t + rbState0.theta;
        // derivative of change in wheel position due to translation
        double dBeta = odState0.dLambda - rbState0.omega * odState0.circCoef;
        // dBeta * integral of direction vector of the wheel / angular velocity
        return new Vector2(dBeta * Math.sin(tau + odState0.mu) / rbState0.omega, -dBeta * Math.cos(tau + odState0.mu) / rbState0.omega);
    }

    // i'll do this as soon as i figure out how to approximate the fresnel integrals
    private Vector2 secondOrderAntiderivative(double t, RobotState rbState0, RobotState rbStateF, OdometerState odState0, OdometerState odStateF) {
        // todo
        return null;
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

    @Override
    public boolean trustworthy() {
        return true;
    }
}
