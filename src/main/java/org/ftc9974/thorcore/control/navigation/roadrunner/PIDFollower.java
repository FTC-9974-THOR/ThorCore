package org.ftc9974.thorcore.control.navigation.roadrunner;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ftc9974.thorcore.control.PIDFv;

public class PIDFollower extends TrajectoryFollower {

    public final PIDFv xController, yController, tController;

    public PIDFollower() {
        xController = new PIDFv(0, 0, 0, 0);
        yController = new PIDFv(0, 0, 0, 0);
        tController = new PIDFv(0, 0, 0, 0);
        tController.setContinuityRange(-Math.PI, Math.PI);
        tController.setContinuous(true);
    }

    @NonNull
    @Override
    protected DriveSignal internalUpdate(@NonNull Pose2d currentPose, @Nullable Pose2d currentVelocity) {
        if (currentVelocity == null) currentVelocity = new Pose2d();

        double t = elapsedTime();

        Pose2d desiredPose = trajectory.get(t);
        Pose2d desiredVelocity = trajectory.velocity(t);
        Pose2d desiredAcceleration = trajectory.acceleration(t);

        xController.positionSetpoint = desiredPose.getX();
        yController.positionSetpoint = desiredPose.getY();
        tController.positionSetpoint = desiredPose.getHeading();

        xController.velocitySetpoint = desiredVelocity.getX();
        yController.velocitySetpoint = desiredVelocity.getY();
        tController.velocitySetpoint = desiredVelocity.getHeading();

        double xCorrection = xController.update(currentPose.getX(), currentVelocity.getX());
        double yCorrection = yController.update(currentPose.getY(), currentVelocity.getY());
        double tCorrection = tController.update(currentPose.getHeading(), currentVelocity.getHeading());

        Pose2d accelerationCommand = new Pose2d(
                xCorrection + desiredAcceleration.getX(),
                yCorrection + desiredAcceleration.getY(),
                tCorrection + desiredAcceleration.getHeading()
        );

        return new DriveSignal(desiredVelocity, accelerationCommand);
    }


    // the following is an artifact of Java not supporting some of Kotlin's features, namely access
    // modifiers for gets/sets on variables. lastError is defined as follows in TrajectoryFollower.kt:
    //     abstract var lastError: Pose2d
    //            protected set
    // Java doesn't support that, so it has to be done with getters and setters.

    private Pose2d lastError;

    @NonNull
    @Override
    public Pose2d getLastError() {
        return lastError;
    }

    @Override
    protected void setLastError(@NonNull Pose2d lastError) {
        this.lastError = lastError;
    }
}
