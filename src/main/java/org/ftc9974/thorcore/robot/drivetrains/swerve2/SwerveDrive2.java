package org.ftc9974.thorcore.robot.drivetrains.swerve2;

import com.qualcomm.robotcore.util.RobotLog;

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
        //double[] crossProduct = cross(0, 0, angularVelocity, moduleX, moduleY, 0);
        // vel = angularVelocity x modulePosition + linearVelocity
        //double xVel = -angularVelocity * module.position.getY() + linearVelocity.getX();
        //double yVel = angularVelocity * module.position.getX() + linearVelocity.getY();

        module.setVelocity(calculateModuleKinematics(module, linearVelocity, angularVelocity));
    }

    public static Vector2 calculateModuleKinematics(SwerveModule2 module, Vector2 linearVelocity, double angularVelocity) {
        double xVel = -angularVelocity * module.position.getY() + linearVelocity.getX();
        double yVel = angularVelocity * module.position.getX() + linearVelocity.getY();
        return new Vector2(xVel, yVel);
    }

    private double[] cross(double ax, double ay, double az, double bx, double by, double bz) {
        return new double[] {
                ay * bz - az * by,
                az * bx - ax * bz,
                ax * by - ay * bx
        };
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
