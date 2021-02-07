package org.ftc9974.thorcore.control.navigation;

import org.ftc9974.thorcore.control.math.ParametricCurve;
import org.ftc9974.thorcore.control.math.Vector2;

public class GuidingVectorField {

    public static class Guidance {
        public Vector2 direction, closestPoint, deriv;
        public double distanceToPath, distanceAlongPath;

        private Guidance(Vector2 direction, Vector2 closestPoint, Vector2 deriv, double distanceToPath, double distanceAlongPath) {
            this.direction = direction;
            this.closestPoint = closestPoint;
            this.deriv = deriv;
            this.distanceToPath = distanceToPath;
            this.distanceAlongPath = distanceAlongPath;
        }
    }

    private ParametricCurve path;
    private double kC, kP;
    private double convergenceDistance;
    private boolean convergenceEnabled;

    public GuidingVectorField(ParametricCurve path, double kC, double kP) {
        this.path = path;
        this.kC = kC;
        this.kP = kP;
    }

    public void setPath(ParametricCurve path) {
        this.path = path;
    }

    public ParametricCurve getPath() {
        return path;
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

    public Guidance guidance(Vector2 robotPosition) {
        ParametricCurve.ClosestPoint closestPoint = path.findPointClosestTo(robotPosition);
        Vector2 toCurve = closestPoint.pToPoint;

        // guidance responsible for moving the robot onto the path
        Vector2 correctiveGuidance = toCurve.scalarMultiply(kC/* * toCurve.getMagnitude()*/);

        // guidance responsible for moving the robot along the path
        Vector2 primaryGuidance = closestPoint.firstDeriv.normalized().scalarMultiply(kP);

        double distanceAlongCurve = -1;

        // convergence logic
        if (convergenceEnabled && convergenceDistance > 0) {
            distanceAlongCurve = path.segmentLength(closestPoint.t, 1);
            // as the robot approaches the end of the path, decrease primaryGuidance. this causes
            // correctiveGuidance to start becoming the dominant guidance, which makes the vector
            // field smoothly converge to the end point of the path, even correcting for overshoot.
            double kTParam = 1 - (distanceAlongCurve / convergenceDistance);
            // check if convergence should start taking effect before actually doing it
            if (kTParam >= 0) {
                double kT;
                // piecewise function used for ramping
                if (kTParam <= 0.5) {
                    // kT = 1 - 2x^2
                    kT = 1 - 2 * (kTParam * kTParam);
                } else {
                    // kT = 2(x - 1)^2
                    kT = 2 * (kTParam - 1) * (kTParam - 1);
                }
                primaryGuidance = primaryGuidance.scalarMultiply(kT);
            }
        }

        return new Guidance(
                correctiveGuidance.add(primaryGuidance).normalized(),
                closestPoint.point,
                closestPoint.firstDeriv,
                toCurve.getMagnitude(),
                distanceAlongCurve
        );
    }
}
