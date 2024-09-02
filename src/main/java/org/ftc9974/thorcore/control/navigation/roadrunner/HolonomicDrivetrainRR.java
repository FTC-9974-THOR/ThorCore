package org.ftc9974.thorcore.control.navigation.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public interface HolonomicDrivetrainRR {

    default void drive(Pose2d velocity) {
        drive(velocity, new Pose2d(0, 0, 0));
    }

    void drive(Pose2d velocity, Pose2d acceleration);
}
