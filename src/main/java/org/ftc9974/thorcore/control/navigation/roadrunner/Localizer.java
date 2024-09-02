package org.ftc9974.thorcore.control.navigation.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public interface Localizer {
    Pose2d getPose();
    Pose2d getVelocity();
    default void update() { }
}
