package org.ftc9974.thorcore.control.navigation.roadrunner;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;
import org.ejml.dense.fixed.CommonOps_DDF3;

public class OdometryLocalizer implements Localizer {

    private final double LATERAL_DISTANCE;
    private final double FRONT_ENCODER_OFFSET;
    private final double MM_PER_TICK;

    //private final DMatrix3x3 rotation = new DMatrix3x3();
    private final DMatrix3x3 correction = new DMatrix3x3();
    private final DMatrix3 delta = new DMatrix3();

    private final Odometry odometry;

    private double lastLeftPos, lastRightPos, lastFrontPos;

    public Pose2d pose;
    public Pose2d velocity;
    public Pose2d relativeVelocity;
    public Pose2d relativePoseDelta;

    public OdometryLocalizer(Odometry odometry, Pose2d startingPose) {
        this.odometry = odometry;
        pose = startingPose;
        velocity = new Pose2d();
        relativeVelocity = new Pose2d();
        relativePoseDelta = new Pose2d();

        LATERAL_DISTANCE = odometry.lateralDistance();
        FRONT_ENCODER_OFFSET = odometry.horizontalOffset();
        MM_PER_TICK = odometry.mmPerTick();
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public Pose2d getVelocity() {
        return velocity;
    }

    @Override
    public void update() {
        double x_l = odometry.leftEncoder().getCurrentPosition() * MM_PER_TICK;
        double x_r = odometry.rightEncoder().getCurrentPosition() * MM_PER_TICK;
        double x_h = odometry.horizontalEncoder().getCurrentPosition() * MM_PER_TICK;
        double v_l = odometry.leftEncoder().getCorrectedVelocity() * MM_PER_TICK;
        double v_r = odometry.rightEncoder().getCorrectedVelocity() * MM_PER_TICK;
        double v_h = odometry.horizontalEncoder().getCorrectedVelocity() * MM_PER_TICK;

        relativePoseDelta = computeRelativePoseDelta(x_l, x_r, x_h);
        pose = new Pose2d(
                pose.vec().plus(relativePoseDelta.vec().rotated(pose.getHeading())),
                pose.getHeading() + relativePoseDelta.getHeading()
        );

        relativeVelocity = computeRelativeVelocity(v_l, v_r, v_h);
        velocity = new Pose2d(
                relativeVelocity.vec().rotated(pose.getHeading()),
                relativeVelocity.getHeading()
        );
    }

    private Pose2d computeRelativePoseDelta(double x_l, double x_r, double x_h) {
        double deltaX_l = x_l - lastLeftPos;
        double deltaX_r = x_r - lastRightPos;
        double deltaX_h = x_h - lastFrontPos;
        // a positive (counterclockwise) rotation will make the right odometer count up and the left
        // odometer count down.
        double phi = (deltaX_r - deltaX_l) / LATERAL_DISTANCE;
        double deltaX_c = (deltaX_r + deltaX_l) / 2.0;
        // the front odometer is ahead of the robot center, so a positive rotation causes is to
        // count up. subtract a correction to account for this.
        double deltaX_perp = deltaX_h - FRONT_ENCODER_OFFSET * phi;
        //double theta_0 = currentPose.heading.toDouble();

        /*double cosTheta = cos(theta_0), sinTheta = sin(theta_0);
        rotation.setTo(
                cosTheta, -sinTheta, 0,
                sinTheta, cosTheta, 0,
                0, 0, 1
        );*/
        delta.setTo(
                deltaX_c,
                deltaX_perp,
                phi
        );
        if (phi != 0) {
            double cosPhi = cos(phi), sinPhi = sin(phi);
            correction.setTo(
                    sinPhi / phi, (cosPhi - 1) / phi, 0,
                    (1 - cosPhi) / phi, sinPhi / phi, 0,
                    0, 0, 1
            );
            CommonOps_DDF3.mult(correction, delta, delta);
        }
        //CommonOps_DDF3.mult(rotation, delta, delta);

        lastLeftPos = x_l;
        lastRightPos = x_r;
        lastFrontPos = x_h;

        return new Pose2d(delta.a1, delta.a2, delta.a3);
    }

    private Pose2d computeRelativeVelocity(double v_l, double v_r, double v_h) {
        double angularVelocity = (v_r - v_l) / LATERAL_DISTANCE;
        double xVelocity = (v_r + v_l) / 2.0;
        double yVelocity = v_h - FRONT_ENCODER_OFFSET * angularVelocity;

        return new Pose2d(xVelocity, yVelocity, angularVelocity);
    }
}
