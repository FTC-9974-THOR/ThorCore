package org.ftc9974.thorcore.control.navigation.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.ftc9974.thorcore.control.math.Vector2;

import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;

public class AprilTagLocalizer {

    private final AprilTagProcessor processor;
    private final Vector2 robotOriginToCamera = new Vector2(0, 0);
    private double cameraHeading = 0;

    public AprilTagLocalizer(AprilTagProcessor processor) {
        this.processor = processor;
    }

    public List<Pose2d> update(boolean freshOnly) {
        List<AprilTagDetection> detections = freshOnly ? processor.getFreshDetections() : processor.getDetections();
        if (detections == null) {
            return new LinkedList<>();
        }
        return detections.stream()
                // i'm not sure what caused it, but i encountered an error where this field would be
                // null. thus, i'm checking it just to be safe.
                .filter(detection -> detection.ftcPose != null)
                // the pose estimate starts to get quite noisy at long range, so filter those out
                .filter(detection -> detection.ftcPose.range < 1500)
                .map(this::calculateRobotPose)
                .collect(Collectors.toList());
    }

    public void setCameraPose(Pose2d cameraPose) {
        robotOriginToCamera.setX(cameraPose.getX());
        robotOriginToCamera.setY(cameraPose.getY());
        cameraHeading = cameraPose.getHeading();
    }

    private Pose2d calculateRobotPose(AprilTagDetection detection) {
        //noinspection SuspiciousNameCombination
        Vector2 cameraToTag = new Vector2(detection.ftcPose.y, -detection.ftcPose.x); // relative to camera
        Vector2 robotToTagRelative = cameraToTag.rotate(cameraHeading).add(robotOriginToCamera); // relative to robot
        VectorF tagNormalVector = detection.metadata.fieldOrientation.applyToVector(new VectorF(0, 0, 1)); // this points *into* the tag
        double tagHeading = Math.atan2(tagNormalVector.get(1), tagNormalVector.get(0));
        double heading = tagHeading - detection.ftcPose.yaw - cameraHeading;
        Vector2 robotToTagAbsolute = robotToTagRelative.rotate(heading); // relative to tag
        Vector2 tagToRobot = robotToTagAbsolute.scalarMultiply(-1); // relative to tag
        double unitConversionFactor = detection.metadata.distanceUnit.toMm(1);
        Vector2 tagPositionOnField = new Vector2(
                unitConversionFactor * detection.metadata.fieldPosition.get(0),
                unitConversionFactor * detection.metadata.fieldPosition.get(1)
        );
        Vector2 robotPositionOnField = tagPositionOnField.add(tagToRobot);
        return new Pose2d(robotPositionOnField.getX(), robotPositionOnField.getY(), heading);
    }
}
