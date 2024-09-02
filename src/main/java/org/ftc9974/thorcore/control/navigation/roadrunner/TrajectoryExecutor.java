package org.ftc9974.thorcore.control.navigation.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TrajectoryExecutor {

    private final HolonomicDrivetrainRR drivetrain;
    private final TrajectoryFollower follower;
    private final Localizer localizer;

    public TrajectoryExecutor(@NonNull HolonomicDrivetrainRR drivetrain, @NonNull TrajectoryFollower follower, @NonNull Localizer localizer) {
        this.drivetrain = drivetrain;
        this.follower = follower;
        this.localizer = localizer;
    }

    public void startPath(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public void update(Telemetry telemetry) {
        DriveSignal driveSignal = follower.update(localizer.getPose(), localizer.getVelocity());
        //telemetry.addData("Drive Signal", driveSignal);
        //RobotLog.dd("Drivetrain", "Follower Error: %s", follower.getLastError().toString());
        drivetrain.drive(driveSignal.getVel(), driveSignal.getAccel());
    }

    public boolean isFollowing() {
        return follower.isFollowing();
    }
}
