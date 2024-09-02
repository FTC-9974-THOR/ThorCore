package org.ftc9974.thorcore.robot.drivetrains.swerve2;

import org.ftc9974.thorcore.control.math.Vector2;

import java.util.Arrays;
import java.util.List;

public class SwerveDrive2 {

    private final List<SwerveModule2> modules;

    public SwerveDrive2(SwerveModule2... modules) {
        this(Arrays.asList(modules));
    }

    public SwerveDrive2(List<SwerveModule2> modules) {
        this.modules = modules;
    }

    public void drive(Vector2 linearVelocity, double angularVelocity) {
        for (SwerveModule2 module : modules) {
            updateModule(module, linearVelocity, angularVelocity);
        }
    }

    private void updateModule(SwerveModule2 module, Vector2 linearVelocity, double angularVelocity) {
        module.setVelocity(calculateModuleKinematics(module, linearVelocity, angularVelocity));
    }

    static Vector2 calculateModuleKinematics(SwerveModule2 module, Vector2 linearVelocity, double angularVelocity) {
        double xVel = -angularVelocity * module.position.getY() + linearVelocity.getX();
        double yVel = angularVelocity * module.position.getX() + linearVelocity.getY();
        return new Vector2(xVel, yVel);
    }

    public void update(Vector2 currentRobotLinearVelocity, double currentRobotAngularVelocity) {
        for (SwerveModule2 module : modules) {
            module.update(currentRobotLinearVelocity, currentRobotAngularVelocity);
        }
    }

    public void update() {
        update(Vector2.ZERO, 0);
    }

    public void preAlignWheels(Vector2 linearVelocity, double angularVelocity) {
        for (SwerveModule2 module : modules) {
            Vector2 moduleVelocity = calculateModuleKinematics(module, linearVelocity, angularVelocity);
            module.setDirectionSetpoint(moduleVelocity.getHeading());
        }
    }

    public void stop() {
        modules.forEach(SwerveModule2::stop);
    }
}
