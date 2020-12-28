package org.ftc9974.thorcore.control.navigation;

import org.ftc9974.thorcore.control.math.CubicBezierCurve;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.util.MathUtilities;

public class GuidingVectorField {

    private CubicBezierCurve path;
    private double kC, kP;
    private double convergenceDistance;
    private boolean convergenceEnabled;

    public GuidingVectorField(CubicBezierCurve path, double kC, double kP) {
        this.path = path;
        this.kC = kC;
        this.kP = kP;
    }

    public void setKC(double kC) {
        this.kC = kC;
    }

    public double getKC() {
        return kC;
    }

    public void setKP(double kP) {
        this.kP = kP;
    }

    public double getKP() {
        return kP;
    }

    public void setConvergenceDistance(double distance) {
        convergenceDistance = Math.max(0, distance);
    }

    public double getConvergenceDistance() {
        return convergenceDistance;
    }

    public void setConvergenceEnabled(boolean enabled) {
        convergenceEnabled = enabled;
    }

    public boolean isConvergenceEnabled() {
        return convergenceEnabled;
    }

    public Vector2 guidance(Vector2 robotPosition) {
        CubicBezierCurve.ClosestPoint closestPoint = path.findPointClosestTo(robotPosition);
        Vector2 toCurve = closestPoint.pToPoint;

        // guidance responsible for moving the robot onto the path
        Vector2 correctiveGuidance = toCurve.scalarMultiply(kC * toCurve.getMagnitude());

        // guidance responsible for moving the robot along the path
        Vector2 primaryGuidance = closestPoint.firstDeriv.normalized().scalarMultiply(kP);

        // convergence logic
        if (convergenceEnabled && convergenceDistance > 0) {
            // as the robot approaches the end of the path, decrease primaryGuidance. this causes
            // correctiveGuidance to start becoming the dominant guidance. this makes the vector
            // field smoothly converge to the end point of the path, even correcting for overshoot.
            double kTParam = 1 - (path.segmentLength(closestPoint.t, 1) / convergenceDistance);
            // check if convergence should start taking effect before actually doing it
            if (kTParam >= 0) {
                double kT;
                // piecewise function used for ramping
                if (kTParam <= 0.5) {
                    kT = 1 - 2 * (kTParam * kTParam);
                } else {
                    kT = 2 * (kTParam - 1) * (kTParam - 1);
                }
                // account for the robot's minimum speed
                kT = MathUtilities.map(kT, 0, 1, 0.1, 1);
                primaryGuidance = primaryGuidance.scalarMultiply(kT);
            }
        }

        return correctiveGuidance.add(primaryGuidance);
    }
}
