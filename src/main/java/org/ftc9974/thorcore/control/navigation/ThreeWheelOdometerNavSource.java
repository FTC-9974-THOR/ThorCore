package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.robot.sensors.OdometerWheel;
import org.ftc9974.thorcore.util.MathUtilities;

public class ThreeWheelOdometerNavSource implements NavSource {

    private static final String TAG = "ThreeWheelOdometerNavSource";

    private static final double EPSILON = 0.0001;

    private final OdometerWheel parallelA, parallelB, perpendicular;
    private final boolean aAndBAreAntiParallel;
    private final double circCoefA, circCoefB, circCoefP;
    private final double parallelAHeading, parallelBHeading, perpendicularHeading;

    // robot heading is relative to the initial heading of the robot
    private OdometryKinematics.RobotState previousRobotState;
    private OdometryKinematics.OdometerState previousAState, previousBState, previousPState;

    // location of the robot in *field-relative* coordinates
    private Vector2 location;
    // measure heading relative to
    private double headingReference;
    // offset between initial robot pose and field coordinates
    private double headingOffset;

    public ThreeWheelOdometerNavSource(OdometerWheel parallelA, OdometerWheel parallelB, OdometerWheel perpendicular) {
        if (Math.abs(parallelA.getDirection().crossMag(parallelB.getDirection())) > EPSILON) {
            throw new IllegalArgumentException("The provided parallel odometers are, in fact, not parallel");
        }
        if (Math.abs(parallelA.getDirection().dot(perpendicular.getDirection())) > EPSILON) {
            throw new IllegalArgumentException("The provided perpendicular odometer is, in fact, not perpendicular to the other 2 odometers");
        }
        this.parallelA = parallelA;
        this.parallelB = parallelB;
        this.perpendicular = perpendicular;
        circCoefA = parallelA.getCartesianLocationOnRobot().crossMag(parallelA.getCartesianDirectionOnRobot()) * parallelA.getCalibrationFactor();
        circCoefB = parallelB.getCartesianLocationOnRobot().crossMag(parallelB.getCartesianDirectionOnRobot()) * parallelB.getCalibrationFactor();
        circCoefP = perpendicular.getCartesianLocationOnRobot().crossMag(perpendicular.getCartesianDirectionOnRobot()) * perpendicular.getCalibrationFactor();

        // the theta and omega calculations change depending on if parallelA and parallelB count up
        // in the same direction, so set a flag indicating which version is needed.
        aAndBAreAntiParallel = parallelA.getCartesianDirectionOnRobot().dot(parallelB.getCartesianDirectionOnRobot()) < 0;

        parallelAHeading = parallelA.getCartesianDirectionOnRobot().getHeading();
        parallelBHeading = parallelB.getCartesianDirectionOnRobot().getHeading();
        perpendicularHeading = perpendicular.getCartesianDirectionOnRobot().getHeading();

        RobotLog.vv(TAG, "circCoefA: %f circCoefB: %f circCoefP: %f antiParallel: %s parallelHeading: %f perpendicularHeading: %f",
                circCoefA, circCoefB, circCoefP, aAndBAreAntiParallel ? "true" : "false", parallelAHeading, perpendicularHeading);

        //parallelA.getConnectedModule().setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //parallelB.getConnectedModule().setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //perpendicular.getConnectedModule().setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        location = Vector2.ZERO;
        headingOffset = 0;
        headingReference = 0;
    }

    // the odometer kinematics work in a coordinate system relative to the initial location and
    // direction of the robot (the state of the robot when init() is called). the robot pose within
    // this coordinate system is than transformed into the field-relative coordinate system.

    public void setLocation(final Vector2 location) {
        // location parameter is final to prevent outside code from changing this class's location
        // variable. most Vector2 operations are non-mutating, but setX() and setY() are mutating.
        this.location = location;
    }

    public void setHeading(double heading) {
        headingOffset = heading;
        if (previousRobotState != null) {
            headingReference = previousRobotState.theta;
        }
        // headingOffset - headingReference = heading of kinematic system x axis in field relative system
    }

    private void init() {
        RobotLog.vv(TAG, "init()");
        parallelA.reset();
        parallelB.reset();
        perpendicular.reset();
        double timestamp = System.nanoTime() / (1e9);
        double parallelAPosition = parallelA.getPosition(), parallelAVelocity = parallelA.getVelocity();
        double parallelBPosition = parallelB.getPosition(), parallelBVelocity = parallelB.getVelocity();
        double perpendicularPosition = perpendicular.getPosition(), perpendicularVelocity = perpendicular.getVelocity();

        double theta, omega;
        if (aAndBAreAntiParallel) {
            theta = (parallelAPosition + parallelBPosition) / (circCoefA + circCoefB);
            omega = (parallelAVelocity + parallelBVelocity) / (circCoefA + circCoefB);
        } else {
            theta = (parallelAPosition - parallelBPosition) / (circCoefA - circCoefB);
            omega = (parallelAVelocity - parallelBVelocity) / (circCoefA - circCoefB);
        }

        previousRobotState = new OdometryKinematics.RobotState();
        previousRobotState.theta = theta;
        previousRobotState.omega = omega;
        previousRobotState.t = timestamp;

        previousAState = new OdometryKinematics.OdometerState();
        previousAState.lambda = parallelAPosition;
        previousAState.dLambda = parallelAVelocity;
        previousAState.circCoef = circCoefA;
        previousAState.mu = parallelAHeading;
        previousAState.direction = parallelA.getCartesianDirectionOnRobot();

        previousBState = new OdometryKinematics.OdometerState();
        previousBState.lambda = parallelBPosition;
        previousBState.dLambda = parallelBVelocity;
        previousBState.circCoef = circCoefB;
        previousBState.mu = parallelBHeading;
        previousBState.direction = parallelB.getCartesianDirectionOnRobot();

        previousPState = new OdometryKinematics.OdometerState();
        previousPState.lambda = perpendicularPosition;
        previousPState.dLambda = perpendicularVelocity;
        previousPState.circCoef = circCoefP;
        previousPState.mu = perpendicularHeading;
        previousPState.direction = perpendicular.getCartesianDirectionOnRobot();

        //RobotLog.vv(TAG, "previousRobotState: %s", previousRobotState.toString());
        //RobotLog.vv(TAG, "previousAState: %s", previousAState.toString());
        //RobotLog.vv(TAG, "previousBState: %s", previousBState.toString());
        //RobotLog.vv(TAG, "previousPState: %s", previousPState.toString());
    }

    @Override
    public void update() {
        RobotLog.vv(TAG, "update()");
        if (previousRobotState == null) {
            init();
            return;
        }
        double timestamp = System.nanoTime() / (1e9);
        double parallelAPosition = parallelA.getPosition(), parallelAVelocity = parallelA.getVelocity();
        double parallelBPosition = parallelB.getPosition(), parallelBVelocity = parallelB.getVelocity();
        double perpendicularPosition = perpendicular.getPosition(), perpendicularVelocity = perpendicular.getVelocity();

        // since a and b are parallel, betaA = betaB, assuming parallelA and parallelB point in the
        // same direction - at least, in theory. real life will have some wheel slip.
        // parallelAPosition - theta * circCoefA = parallelBPosition - theta * circCoefB
        // parallelAPosition - parallelBPosition = theta * circCoefA - theta * circCeofB
        // parallelAPosition - parallelBPosition = theta * (circCoefA - circCoefB)
        // theta = (parallelAPosition - parallelBPosition) / (circCoefA - circCoefB)
        // if parallelA points in exactly the opposite direction as parallelB, betaA = -betaB
        // parallelAPosition - theta * circCoefA = theta * circCoefB - parallelBPosition
        // parallelAPosition + parallelBPosition = theta * circCoefB + theta * circCoefA
        // parallelAPosition + parallelBPosition = theta * (circCoefA + circCoefB)
        // theta = (parallelAPosition + parallelBPosition) / (circCoefA + circCoefB)
        // this bears a striking resemblance to the typical equation for the parallel pair,
        // beta = (parallelAPosition - parallelBPosition) / trackWidth
        // where trackWidth is the distance between parallelA and parallelB.
        double theta;
        if (aAndBAreAntiParallel) {
            theta = (parallelAPosition + parallelBPosition) / (circCoefA + circCoefB);
        } else {
            theta = (parallelAPosition - parallelBPosition) / (circCoefA - circCoefB);
        }
        //betaA = parallelAPosition - theta * circCoefA;
        //betaB = parallelBPosition - theta * circCoefB;
        //betaP = perpendicularPosition - theta * circCoefP;

        // calibration mode?
        // dTheta/dCircCoefA = (parallelAPosition - parallelBPosition)/((circCoefA - circCoefB)^2)
        //                   = theta / (circCoefA - circCoefB)
        // betaA = parallelAPosition - circCoefA * (parallelAPosition - parallelBPosition) / (circCoefA - circCoefB)
        // dBetaA/dCircCoefA = -(theta + circCoefA * dTheta/dCircCoefA)
        //                   = -theta - circCoefA * theta / (circCoefA - circCoefB)
        //                   = -theta * (1 - circCoefA / (circCoefA - circCoefB))
        //                   = circCoefB * (parallelAPosition - parallelBPosition) / (circCoefA - circCoefB)^2
        // betaA = 0 = parallelAPosition - theta * circCoefA;
        // theta * circCoefA = parallelAPosition
        // circCoefA = parallelAPosition / theta

        // theta = (parallelAPosition - parallelBPosition) / (circCoefA - circCoefB)
        // theta = (1/(circCoefA - circCoefB))(parallelAPosition - parallelBPosition)
        // omega = d/dt[theta] = (1/(circCoefA - circCoefB))(parallelAVelocity - parallelBVelocity)
        // omega = (parallelAVelocity - parallelBVelocity0 / (circCoefA - circCoefB)
        double omega;
        if (aAndBAreAntiParallel) {
            omega = (parallelAVelocity + parallelBVelocity) / (circCoefA + circCoefB);
        } else {
            omega = (parallelAVelocity - parallelBVelocity) / (circCoefA - circCoefB);
        }

        // betaA = parallelAPosition - theta * circCoefA
        // dBetaA = d/dt[betaA] = parallelAVelocity - omega * circCoefA
        // dBetaB = d/dt[betaB] = parallelBVelocity - omega * circCoefB
        // betaAB = (betaA + betaB) / 2
        // dBetaAB = (dBetaA + dBetaB) / 2
        //dBetaA = parallelAVelocity - omega * circCoefA;
        //dBetaB = parallelBVelocity - omega * circCoefB;
        //double betaAB = (betaA + betaB) / 2;
        //double dBetaAB = (dBetaA + dBetaB) / 2;
        // betaP = perpendicularPosition - theta * circCoefP
        // dBetaP = d/dt[betaP] = perpendicularVelocity - omega * circCoefP
        //dBetaP = perpendicularVelocity - omega * circCoefP;

        OdometryKinematics.RobotState currentRobotState = new OdometryKinematics.RobotState();
        currentRobotState.theta = theta;
        currentRobotState.omega = omega;
        currentRobotState.t = timestamp;

        OdometryKinematics.OdometerState currentAState = new OdometryKinematics.OdometerState();
        currentAState.lambda = parallelAPosition;
        currentAState.dLambda = parallelAVelocity;
        currentAState.circCoef = circCoefA;
        currentAState.mu = parallelAHeading;
        currentAState.direction = parallelA.getCartesianDirectionOnRobot();

        OdometryKinematics.OdometerState currentBState = new OdometryKinematics.OdometerState();
        currentBState.lambda = parallelBPosition;
        currentBState.dLambda = parallelBVelocity;
        currentBState.circCoef = circCoefB;
        currentBState.mu = parallelBHeading;
        currentBState.direction = parallelB.getCartesianDirectionOnRobot();

        OdometryKinematics.OdometerState currentPState = new OdometryKinematics.OdometerState();
        currentPState.lambda = perpendicularPosition;
        currentPState.dLambda = perpendicularVelocity;
        currentPState.circCoef = circCoefP;
        currentPState.mu = perpendicularHeading;
        currentPState.direction = perpendicular.getCartesianDirectionOnRobot();

        /*
        RobotLog.vv(TAG, "previousRobotState: %s", previousRobotState.toString());
        RobotLog.vv(TAG, "currentRobotState: %s", currentRobotState.toString());
        RobotLog.vv(TAG, "previousAState: %s", previousAState.toString());
        RobotLog.vv(TAG, "currentAState: %s", currentAState.toString());
        RobotLog.vv(TAG, "previousBState: %s", previousBState.toString());
        RobotLog.vv(TAG, "currentBState: %s", currentBState.toString());
        RobotLog.vv(TAG, "previousPState: %s", previousPState.toString());
        RobotLog.vv(TAG, "currentPState: %s", currentPState.toString());
         */

        //RobotLog.vv(TAG, "Calculating positionDelta with parallelA and perpendicular");
        Vector2 positionDeltaA = OdometryKinematics.calculatePositionDelta(
                previousRobotState, currentRobotState,
                previousAState, currentAState,
                previousPState, currentPState
        );
        //RobotLog.vv(TAG, "Calculating positionDelta with parallelB and perpendicular");
        Vector2 positionDeltaB = OdometryKinematics.calculatePositionDelta(
                previousRobotState, currentRobotState,
                previousBState, currentBState,
                previousPState, currentPState
        );
        // headingOffset - headingReference = heading of kinematic system x axis in field relative system
        location = location.add(Vector2.lerp(positionDeltaA, positionDeltaB, 0.5).rotate(headingOffset - headingReference));

        previousRobotState = currentRobotState;
        previousAState = currentAState;
        previousBState = currentBState;
        previousPState = currentPState;
    }

    @Override
    public Vector2 getLocation() {
        if (previousRobotState == null) update();
        return location;
    }

    @Override
    public double getHeading() {
        if (previousRobotState == null) update();
        return MathUtilities.wraparound(
                (previousRobotState.theta - headingReference) + headingOffset,
                -Math.PI, Math.PI
        );
    }

    @Override
    public boolean trustworthy() {
        return true;
    }

    @Override
    public CoordinateSystem coordinateSystem() {
        return CoordinateSystem.CARTESIAN;
    }
}
