package org.ftc9974.thorcore.control.navigation;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.RobotLog;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.util.MathUtilities;

import java.util.Locale;

public class OdometryKinematics {

    private static final String TAG = "OdometryKinematics";

    public static class RobotState {
        // heading of the robot
        public double theta;
        // angular velocity of the robot
        public double omega;
        // timestamp, in seconds
        public double t;

        @NonNull
        @Override
        public String toString() {
            return String.format(
                    Locale.getDefault(),
                    "RobotState: theta: %f omega: %f t: %f",
                    theta, omega, t
            );
        }
    }

    public static class OdometerState {
        // position of the wheel
        public double lambda;
        // velocity of the wheel (derivative of lambda)
        public double dLambda;
        // circumference coefficient of the wheel
        public double circCoef;
        // direction unit vector
        public Vector2 direction;
        // heading of the wheel direction vector
        public double mu;

        @NonNull
        @Override
        public String toString() {
            return String.format(
                    Locale.getDefault(),
                    "OdometerState: lambda: %f dLambda: %f circCoef: %f direction: %s mu: %f",
                    lambda, dLambda, circCoef, direction.toString(), mu
            );
        }
    }

    private OdometryKinematics() { }

    public static Vector2 calculatePositionDelta(RobotState initialRobotState, RobotState finalRobotState,
                                                 OdometerState initialAState, OdometerState finalAState,
                                                 OdometerState initialBState, OdometerState finalBState) {
        RobotLog.vv(TAG, "calculatePositionDelta()");
        // there are some cases where the first and second order kinematics aren't any more accurate
        // then the zeroth order kinematics. this if/else chain tests for these cases and selects
        // the kinematics appropriate for the situation.
        if (initialRobotState.theta == finalRobotState.theta || initialRobotState.omega == 0/*&& initialRobotState.omega == 0 && finalRobotState.omega == 0*/) {
            // this is a degenerate case of the first order kinematics - that is, it's a case where
            // the result of the first order kinematics is the same as the simpler zeroth order
            // kinematics. in fact, the first order kinematics actually won't work if the robot
            // isn't rotating due to a divide-by-zero.
            //RobotLog.vv(TAG, "zeroth order");
            return zerothOrder(
                    initialRobotState, finalRobotState,
                    initialAState, finalAState,
                    initialBState, finalBState
            );
        } else if (initialRobotState.omega == finalRobotState.omega || initialRobotState.t == finalRobotState.t) {
            // this is a degenerate case of the second order kinematics. since angular velocity is
            // constant, the result of the first order kinematics is the same as that of the second
            // order kinematics.
            //RobotLog.vv(TAG, "first order");
            return firstOrder(
                    initialRobotState, finalRobotState,
                    initialAState, finalAState,
                    initialBState, finalBState
            );
        } else {
            /*RobotLog.vv(TAG, "second order");
            secondOrder(
                    initialRobotState, finalRobotState,
                    initialAState, finalAState,
                    initialBState, finalBState
            );*/
            //RobotLog.vv(TAG, "\"second\" order");
            // temporary
            return firstOrder(
                    initialRobotState, finalRobotState,
                    initialAState, finalAState,
                    initialBState, finalBState
            );
        }
    }

    // assumes constant heading.
    public static Vector2 zerothOrder(RobotState initialRobotState, RobotState finalRobotState,
                                      OdometerState initialAState, OdometerState finalAState,
                                      OdometerState initialBState, OdometerState finalBState) {
        double deltaTheta = finalRobotState.theta - initialRobotState.theta;
        double deltaPositionA = finalAState.lambda - initialAState.lambda;
        double deltaPositionB = finalBState.lambda - initialBState.lambda;
        double translationalComponentA = deltaPositionA - deltaTheta * initialAState.circCoef;
        double translationalComponentB = deltaPositionB - deltaTheta * initialBState.circCoef;

        /*double orthogonalizationCoefficient = initialAState.direction.crossMag(initialBState.direction);
        // define A's direction as the x axis
        double componentAlongA = translationalComponentA;
        // and calculate the movement normal to A, allowing it to be along the y axis
        double componentNormalToA = translationalComponentB * orthogonalizationCoefficient;*/

        Vector2 offsetA = Vector2.unitHeadingVector(initialRobotState.theta + initialAState.mu).scalarMultiply(translationalComponentA);
        Vector2 offsetB = Vector2.unitHeadingVector(initialRobotState.theta + initialBState.mu).scalarMultiply(translationalComponentB);
        //RobotLog.vv(TAG, "offsetA: %s offsetB: %s", offsetA.toString(), offsetB.toString());
        return offsetA.add(offsetB);
    }

    // assumes constant linear and angular velocity.
    public static Vector2 firstOrder(RobotState initialRobotState, RobotState finalRobotState,
                                     OdometerState initialAState, OdometerState finalAState,
                                     OdometerState initialBState, OdometerState finalBState) {
        double deltaT = finalRobotState.t - initialRobotState.t;
        Vector2 offsetA = Vector2.subtract(
                firstOrderAntiderivative(deltaT, initialRobotState, initialAState),
                firstOrderAntiderivative(0, initialRobotState, initialAState)
        );
        Vector2 offsetB = Vector2.subtract(
                firstOrderAntiderivative(deltaT, initialRobotState, initialBState),
                firstOrderAntiderivative(0, initialRobotState, initialBState)
        );
        //RobotLog.vv(TAG, "offsetA: %s offsetB: %s", offsetA.toString(), offsetB.toString());
        return offsetA.add(offsetB);
    }

    private static Vector2 firstOrderAntiderivative(double t, RobotState initialRobotState, OdometerState initialOdometerState) {
        double wheelHeading = initialRobotState.omega * t + initialRobotState.theta + initialOdometerState.mu;
        // translationalPosition = odometerPosition - deltaWheelHeading * circCoef
        // translationalPosition = (odometerVelocity * t + initialOdometerPosition) - robotAngularVelocity * t * circCoef
        // translationalVelocity = d/dt[translationalPosition] = odometerVelocity - robotAngularVelocity * circCoef
        double translationalVelocity = initialOdometerState.dLambda - initialRobotState.omega * initialOdometerState.circCoef;
        // integrate (initialOdometerState.dLambda - initialRobotState.omega * initialOdometerState.circCoef) * Math.cos(initialRobotState.omega * t + initialRobotState.theta) dt
        // (initialOdometerState.dLambda - initialRobotState.omega * initialOdometerState.circCoef) * Math.sin(initialRobotState.omega * t + initialRobotState.theta) / initialRobotState.omega
        // translationalVelocity * Math.sin(wheelHeading) / initialRobotState.omega
        // there is a degenerate case when initialRobotState.omega is 0. in this case, these
        // kinematics will return NaNs, but since the robot isn't spinning, the zeroth-order
        // kinematics may be used instead.
        //RobotLog.vv(TAG, "wheelHeading: %f initialOmega: %f initialTheta: %f", wheelHeading, initialRobotState.omega, initialRobotState.theta);
        //RobotLog.vv(TAG, "dBeta: %f initialDLambda: %f rotationCorr: %f", translationalVelocity, initialOdometerState.dLambda, initialRobotState.omega * initialOdometerState.circCoef);
        //RobotLog.vv(TAG, "x: %f y: %f", translationalVelocity * Math.sin(wheelHeading) / initialRobotState.omega, -translationalVelocity * Math.cos(wheelHeading) / initialRobotState.omega);
        return new Vector2(
                translationalVelocity * Math.sin(wheelHeading) / initialRobotState.omega,
                -translationalVelocity * Math.cos(wheelHeading) / initialRobotState.omega
        );
    }

    // assumes constant linear and angular acceleration.
    public static Vector2 secondOrder(RobotState initialRobotState, RobotState finalRobotState,
                                      OdometerState initialAState, OdometerState finalAState,
                                      OdometerState initialBState, OdometerState finalBState) {
        double deltaT = finalRobotState.t - initialRobotState.t;
        Vector2 offsetA;
        //if (initialAState.dLambda < finalAState.dLambda)
        offsetA = Vector2.subtract(
                secondOrderAntiderivative(
                        deltaT,
                        initialRobotState, finalRobotState,
                        initialAState, finalAState
                ),
                secondOrderAntiderivative(
                        0,
                        initialRobotState, finalRobotState,
                        initialAState, finalAState
                )
        );
        Vector2 offsetB = Vector2.subtract(
                secondOrderAntiderivative(
                        deltaT,
                        initialRobotState, finalRobotState,
                        initialBState, finalBState
                ),
                secondOrderAntiderivative(
                        0,
                        initialRobotState, finalRobotState,
                        initialBState, finalBState
                )
        );
        return offsetA.add(offsetB);
    }

    private static Vector2 secondOrderAntiderivative(double t,
                                                     RobotState initialRobotState, RobotState finalRobotState,
                                                     OdometerState initialOdometerState, OdometerState finalOdometerState) {
        // translationalPosition = odometerPosition - deltaWheelHeading * circCoef
        //                       = (odometerVelocity * t + initialOdometerPosition) - robotAngularVelocity * t * circCoef
        // translationalVelocity = odometerVelocity - robotAngularVelocity * circCoef
        //                       = (odometerAcceleration * t + initialOdometerVelocity) - (robotAngularAcceleration * t + initialRobotAngularVelocity) * circCoef
        // assume constant acceleration, approximating by deltas
        double deltaTime = finalRobotState.t - initialRobotState.t;
        double odometerAcceleration = (finalOdometerState.dLambda - initialOdometerState.dLambda) / deltaTime;
        double angularAcceleration = (finalRobotState.omega - initialRobotState.omega) / deltaTime;
        // wheelHeading = ∫∫α dt = 0.5αt^2 + ωt + θ + μ
        //double wheelHeading = 0.5 * angularAcceleration * t * t + initialRobotState.omega * t + initialRobotState.theta + initialOdometerState.mu;
        //double translationalVelocity = (odometerAcceleration * t + initialOdometerState.dLambda) - (angularAcceleration * t + initialRobotState.omega) * initialOdometerState.circCoef;
        // the full integral is a pain to solve (and write out), so here's Wolfram|Alpha solving just the x component:
        // https://www.wolframalpha.com/input/?i=integrate+%28a*t+%2B+v+-+%28alpha*t+%2B+omega%29c%29%28cos%281%2F2+*+alpha+*+t%5E2+%2B+omega+*+t+%2B+theta+%2B+mu%29%29+dt
        // where a = odometerAcceleration
        //       v = initialOdometerVelocity
        //       α = robotAngularAcceleration
        //       ω = initialRobotAngularVelocity
        //       θ = initialRobotHeading
        //       μ = odometerHeading
        //       c = circCoef
        // when written out in plaintext, the x and y components of the antiderivative are:
        // x = (sqrt(α) (a - α c) sin(θ + μ + (α t^2)/2 + t ω) + sqrt(π) (α v - a ω) C((t α + ω)/(sqrt(π) sqrt(α))) cos(-ω^2/(2 α) + θ + μ) - sqrt(π) (α v - a ω) S((t α + ω)/(sqrt(π) sqrt(α))) sin(-ω^2/(2 α) + θ + μ))/α^(3/2)
        // y = (sqrt(α) (α c - a) cos(θ + μ + (α t^2)/2 + t ω) + sqrt(π) (α v - a ω) C((t α + ω)/(sqrt(π) sqrt(α))) sin(-ω^2/(2 α) + θ + μ) + sqrt(π) (α v - a ω) S((t α + ω)/(sqrt(π) sqrt(α))) cos(-ω^2/(2 α) + θ + μ))/α^(3/2)
        // where C(x) and S(x) are the cosine and sine Fresnel integrals, respectively.
        double sqrtAlpha = Math.sqrt(angularAcceleration);
        double fresnelParam = (t * angularAcceleration + initialRobotState.omega) / (MathUtilities.SQRT_PI * sqrtAlpha);
        // θ + μ + (α t^2)/2 + t ω = 0.5 α t^2 + ω t + θ + μ
        double wheelHeading = 0.5 * angularAcceleration * t * t + initialRobotState.omega * t + initialRobotState.theta + initialOdometerState.mu;
        // -ω^2/(2 α) + θ + μ
        double trigParam = -(initialRobotState.omega * initialRobotState.omega) / (2 * angularAcceleration) + initialRobotState.theta + initialOdometerState.mu;
        // sqrt(π) (α v - a ω)
        double fresnelCoef = MathUtilities.SQRT_PI * (angularAcceleration * initialOdometerState.dLambda - odometerAcceleration * initialRobotState.omega);
        // sqrt(α) (a - α c)
        double xTrigCoef = sqrtAlpha * (odometerAcceleration - angularAcceleration * initialOdometerState.circCoef);
        // sqrt(α) (α c - a) = sqrt(α) (-a + α c) = -sqrt(α) (a - α c)
        double yTrigCoef = -xTrigCoef;
        // 1 / (α^(3/2)) = 1 / (sqrt(α) α) = sqrt(α) / (α^2)
        double reciprocalAlphaCbSqrt = sqrtAlpha / (angularAcceleration * angularAcceleration);

        double[] fresnel = MathUtilities.fresnelIntegral(fresnelParam);
        double fresnelC = fresnel[0];
        double fresnelS = fresnel[1];

        double x = (xTrigCoef * Math.sin(wheelHeading) + fresnelCoef * (fresnelC * Math.cos(trigParam) - fresnelS * Math.sin(trigParam))) * reciprocalAlphaCbSqrt;
        double y = (yTrigCoef * Math.cos(wheelHeading) + fresnelCoef * (fresnelC * Math.sin(trigParam) + fresnelS * Math.cos(trigParam))) * reciprocalAlphaCbSqrt;

        //RobotLog.vv(TAG, "α: %f a: %f √α: %f fresnelParam: %f wheelHeading: %f trigParam: %f fresnelCoef: %f xTrigCoef: %f yTrigCoef: %f 1/(α^(3/2)): %f",
        //        angularAcceleration, odometerAcceleration, sqrtAlpha, fresnelParam, wheelHeading, trigParam, fresnelCoef, xTrigCoef, yTrigCoef, reciprocalAlphaCbSqrt);

        //RobotLog.vv(TAG, "fresnelC: %f fresnelS: %f x: %f y: %f", fresnelC, fresnelS, x, y);

        return new Vector2(x, y);
        /*return new Vector2(
                (xTrigCoef * Math.sin(wheelHeading) + fresnelCoef * (fresnelC * Math.cos(trigParam) - fresnelS * Math.sin(trigParam))) * reciprocalAlphaCbSqrt,
                (yTrigCoef * Math.cos(wheelHeading) + fresnelCoef * (fresnelC * Math.sin(trigParam) + fresnelS * Math.cos(trigParam))) * reciprocalAlphaCbSqrt
        );*/
    }
}
