package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.TrapezoidalMotionProfile;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.util.MathUtilities;

public class Fusion2 {

    private LinearOpMode root;
    private HolonomicDrivetrain rb;
    private EncoderPositionCalculator calculator;
    private IMUNavSource imu;
    private PIDF headingPid;

    private double startSpeed,
                   rampUpDistance,
                   cruiseSpeed,
                   rampDownDistance,
                   crawlSpeed,
                   crawlDistance;
    private double minTurningSpeed;

    private int numTargets;

    private boolean ownsHeadingPid;

    public Fusion2(LinearOpMode root, HolonomicDrivetrain drivetrain, EncoderPositionCalculator calculator, IMUNavSource imu, PIDF headingPid) {
        this.root = root;
        rb = drivetrain;
        this.calculator = calculator;
        this.imu = imu;
        this.headingPid = headingPid;

        numTargets = calculator.getNumTargets();

        ownsHeadingPid = false;
        minTurningSpeed = 0.15;
    }

    public Fusion2(LinearOpMode root, HolonomicDrivetrain drivetrain, EncoderPositionCalculator calculator, IMUNavSource imu, PIDFCoefficients pidfCoefficients) {
        this.root = root;
        rb = drivetrain;
        this.calculator = calculator;
        this.imu = imu;
        headingPid = new PIDF(pidfCoefficients);
        headingPid.setPhase(true);
        headingPid.setPeakOutputForward(1);
        headingPid.setPeakOutputReverse(-1);
        headingPid.setContinuityRange(-Math.PI, Math.PI);
        headingPid.setContinuous(true);
        headingPid.setAtTargetThreshold(Math.toRadians(1));

        numTargets = calculator.getNumTargets();

        ownsHeadingPid = true;
        minTurningSpeed = 0.15;
    }

    public void driveToPoint(Vector2 point) {
        driveToPoint(point, null);
    }

    public void driveToPoint(Vector2 point, Runnable whileRunning) {
        // reset the encoders
        rb.resetEncoders();

        // generate the motion profile used for speed control, based on the control parameters.
        double totalDistance = point.getMagnitude();
        TrapezoidalMotionProfile motionProfile;
        // there are 2 special cases for if the total distance of the move is not long enough to
        // reach cruise speed.
        if (totalDistance < crawlDistance) {
            motionProfile = new TrapezoidalMotionProfile(
                    new TrapezoidalMotionProfile.Node(0, crawlSpeed),
                    new TrapezoidalMotionProfile.Node(totalDistance, crawlSpeed)
            );
        } else if (totalDistance < rampUpDistance + rampDownDistance + crawlDistance) {
            // rampUpSpeed = (cruiseSpeed - startSpeed) / rampUpDistance
            // rampDownSpeed = (crawlSpeed - cruiseSpeed) / rampDownDistance
            // cruisePatch = totalDistance - crawlDistance
            // rampUpSpeed * intersectionPoint + startSpeed = rampDownSpeed * (intersectionPoint - cruisePatch) + crawlSpeed
            // rampUpSpeed * intersectionPoint - rampDownSpeed * (intersectionPoint - cruisePatch) = crawlSpeed - startSpeed
            // rampUpSpeed * intersectionPoint - rampDownSpeed * intersectionPoint + rampDownSpeed * cruisePatch = crawlSpeed - startSpeed
            // (rampUpSpeed - rampDownSpeed) * intersectionPoint = crawlSpeed - startSpeed - rampDownSpeed * cruisePatch
            // intersectionPoint = (crawlSpeed - startSpeed - rampDownSpeed * cruisePatch) / (rampUpSpeed - rampDownSpeed)
            // speedAtIntersectionPoint = rampUpSpeed * intersectionPoint + startSpeed
            double rampUpSpeed = (cruiseSpeed - startSpeed) / rampDownDistance;
            double rampDownSpeed = (crawlSpeed - cruiseSpeed) / rampDownDistance;
            double cruisePatch = totalDistance - crawlDistance;
            double intersectionPoint = (crawlSpeed - startSpeed - rampDownSpeed * cruisePatch) / (rampUpSpeed - rampDownSpeed);
            double speedAtIntersectionPoint = rampUpSpeed * intersectionPoint + startSpeed;
            motionProfile = new TrapezoidalMotionProfile(
                    new TrapezoidalMotionProfile.Node(0, startSpeed),
                    new TrapezoidalMotionProfile.Node(intersectionPoint, speedAtIntersectionPoint),
                    new TrapezoidalMotionProfile.Node(cruisePatch, crawlSpeed),
                    new TrapezoidalMotionProfile.Node(totalDistance, crawlSpeed)
            );
        } else {
            motionProfile = new TrapezoidalMotionProfile(
                    new TrapezoidalMotionProfile.Node(0, startSpeed),
                    new TrapezoidalMotionProfile.Node(rampUpDistance, cruiseSpeed),
                    new TrapezoidalMotionProfile.Node(totalDistance - rampDownDistance, cruiseSpeed),
                    new TrapezoidalMotionProfile.Node(totalDistance - crawlDistance, crawlSpeed),
                    new TrapezoidalMotionProfile.Node(totalDistance, crawlSpeed)
            );
        }

        // calculate the encoder targets
        int[] targets = calculator.calculate(point);

        // calculate weights for the progress calculation. these weights are simply the absolute
        // value of each encoder target.
        double[] weights = new double[numTargets];
        for (int i = 0; i < numTargets; i++) {
            weights[i] = Math.abs(targets[i]);
        }

        // calculate the direction vector. this is used to generate the driving commands later.
        // this vector points towards the target point, but has a magnitude of one.
        Vector2 direction = point.normalized();

        while (!root.isStopRequested()) {
            // get the current encoder positions
            int[] positions = rb.getEncoderPositions();

            // calculate the progress of each encoder. this is a value from 0-1 representing how far
            // the encoder has travelled. 0 corresponds to the starting position, and 1 corresponds
            // to the target position.
            double[] progresses = new double[numTargets];
            for (int i = 0; i < numTargets; i++) {
                int target = targets[i];
                // avoid divide-by-zero.
                if (target == 0) {
                    // since target is zero, this progress value will have a weight of zero in the
                    // final progress calculation, so it doesn't matter what you use for this
                    // progress value.
                    progresses[i] = 0;
                } else {
                    progresses[i] = positions[i] / (double) target;
                }
            }
            // calculate overall progress. merp is Multiple intERPolation, which is effectively a
            // weighted average.
            double progress = MathUtilities.merp(progresses, weights);
            root.telemetry.addData("Progress", progress);

            // if the robot has reached (or went past) the target, break out of the loop
            if (progress >= 1) {
                break;
            }

            // apply the motion profile based on the estimated distance the robot has travelled
            double estimatedDistanceTravelled = totalDistance * progress;
            double speed = motionProfile.apply(estimatedDistanceTravelled);

            root.telemetry.addData("Distance", estimatedDistanceTravelled);
            root.telemetry.addData("Speed", speed);
            root.telemetry.addData("NumTargets", numTargets);
            for (int i = 0; i < numTargets; i++) {
                root.telemetry.addData(String.format("P %d", i), positions[i]);
                root.telemetry.addData(String.format("T %d", i), targets[i]);
                root.telemetry.addData(String.format("Pr %d", i), progresses[i]);
            }
            root.telemetry.update();

            // calculate a movement vector in the direction of the target and magnitude equal to the
            // speed from the motion profile
            Vector2 movement = direction.scalarMultiply(speed);
            rb.drive(movement.getX(), movement.getY(), headingPid.update(imu.getHeading()));

            if (whileRunning != null) {
                whileRunning.run();
            }
        }

        rb.drive(0, 0, 0);
    }

    public void turnToHeading(double heading) {
        turnToHeading(heading, null);
    }

    public void turnToHeading(double heading, Runnable whileRunning) {
        headingPid.setSetpoint(heading);
        if (ownsHeadingPid) {
            headingPid.setNominalOutputForward(minTurningSpeed);
            headingPid.setNominalOutputReverse(-minTurningSpeed);
        }

        while (!root.isStopRequested()) {
            rb.drive(0, 0, headingPid.update(imu.getHeading()));

            if (headingPid.atTarget()) {
                break;
            }
            if (whileRunning != null) {
                whileRunning.run();
            }
        }

        rb.drive(0, 0, 0);

        if (ownsHeadingPid) {
            headingPid.setNominalOutputForward(0);
            headingPid.setNominalOutputReverse(0);
        }
    }

    /**
     * Gets the starting speed of the motion profile.
     * @return starting speed
     */
    public double getStartSpeed() {
        return startSpeed;
    }

    /**
     * Sets the starting speed of the motion profile. This is the speed at which the robot will
     * initially move at when driving to a point.
     * @param startSpeed
     */
    public void setStartSpeed(double startSpeed) {
        this.startSpeed = startSpeed;
    }

    public double getRampUpDistance() {
        return rampUpDistance;
    }

    public void setRampUpDistance(double rampUpDistance) {
        this.rampUpDistance = rampUpDistance;
    }

    public double getCruiseSpeed() {
        return cruiseSpeed;
    }

    public void setCruiseSpeed(double cruiseSpeed) {
        this.cruiseSpeed = cruiseSpeed;
    }

    public double getRampDownDistance() {
        return rampDownDistance;
    }

    public void setRampDownDistance(double rampDownDistance) {
        this.rampDownDistance = rampDownDistance;
    }

    public double getCrawlSpeed() {
        return crawlSpeed;
    }

    public void setCrawlSpeed(double crawlSpeed) {
        this.crawlSpeed = crawlSpeed;
    }

    public double getCrawlDistance() {
        return crawlDistance;
    }

    public void setCrawlDistance(double crawlDistance) {
        this.crawlDistance = crawlDistance;
    }

    public double getMinTurningSpeed() {
        return minTurningSpeed;
    }

    public void setMinTurningSpeed(double minTurningSpeed) {
        this.minTurningSpeed = minTurningSpeed;
    }
}
