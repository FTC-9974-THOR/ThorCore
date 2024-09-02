package org.ftc9974.thorcore.control.navigation.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.vision.VisionPortal;
import org.ftc9974.thorcore.control.navigation.IMUNavSource2;
import org.ftc9974.thorcore.util.MathUtilities;

import java.util.List;

public class SensorFusionLocalizer implements Localizer {

    private final VisionPortal visionPortal;
    private final AprilTagLocalizer aprilTagLocalizer;
    private final OdometryLocalizer odometryLocalizer;
    private final IMUNavSource2 imuNavSource;

    public Pose2d poseEstimate;

    private Pose2d lastOdometryPose;
    private double lastImuHeading;
    private final double imuHeadingOffset;

    public SensorFusionLocalizer(VisionPortal visionPortal, AprilTagLocalizer aprilTagLocalizer,
                                 OdometryLocalizer odometryLocalizer, IMUNavSource2 imuNavSource,
                                 Pose2d startPose) {
        this.visionPortal = visionPortal;
        this.aprilTagLocalizer = aprilTagLocalizer;
        this.odometryLocalizer = odometryLocalizer;
        this.imuNavSource = imuNavSource;

        poseEstimate = startPose;
        lastOdometryPose = odometryLocalizer.pose;
        lastImuHeading = imuNavSource.getHeading();
        imuHeadingOffset = -imuNavSource.getHeading() + poseEstimate.getHeading();
    }

    @Override
    public Pose2d getPose() {
        return poseEstimate;
    }

    @Override
    public Pose2d getVelocity() {
        return odometryLocalizer.velocity;
    }

    @Override
    public void update() {
        odometryLocalizer.update();

        double x = poseEstimate.getX();
        double y = poseEstimate.getY();
        double t = poseEstimate.getHeading();

        // apply correction to pose estimate
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            // get pose fixes from AprilTags
            List<Pose2d> poses = aprilTagLocalizer.update(true);
            if (!poses.isEmpty()) {
                // we have location fixes from the tags, so use them to correct our pose estimate.
                // there's a lot of fancy ways to do this (ie a Kalman filter), but I've elected to
                // just average the pose estimates.
                poses.add(poseEstimate);
                poses.add(poseEstimate);
                poses.add(poseEstimate);
                poses.add(poseEstimate);

                x = poses.stream().mapToDouble(Pose2d::getX).sum() / poses.size();
                y = poses.stream().mapToDouble(Pose2d::getY).sum() / poses.size();

                t = MathUtilities.circularMean(poses.stream().mapToDouble(Pose2d::getHeading).toArray());
            }
        }

        // compute pose delta measured by the odometry
        //Pose2d odometryPoseDelta = odometryLocalizer.pose.minus(lastOdometryPose);
        //lastOdometryPose = odometryLocalizer.pose;
        Pose2d odometryRelativePoseDelta = odometryLocalizer.relativePoseDelta;
        Pose2d odometryPoseDelta = new Pose2d(
                odometryRelativePoseDelta.vec().rotated(t),
                odometryRelativePoseDelta.getHeading()
        );

        // compute heading delta measured by the IMU
        double imuHeading = imuNavSource.getHeading() + imuHeadingOffset;
        double imuHeadingDelta = imuHeading - lastImuHeading;
        lastImuHeading = imuHeading;

        // apply delta to pose estimate
        poseEstimate = new Pose2d(
                x + odometryPoseDelta.getX(),
                y + odometryPoseDelta.getY(),
                headingWrap(t + MathUtilities.circularMean(odometryPoseDelta.getHeading(), imuHeadingDelta))
        );
    }

    private static double headingWrap(double t) {
        return MathUtilities.wraparound(t, -Math.PI, Math.PI);
    }
}
