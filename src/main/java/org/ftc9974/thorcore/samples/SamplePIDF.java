package org.ftc9974.thorcore.samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.ftc9974.thorcore.OpModeEnhanced;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.drivetrains.TankDrive2Wheel;

/**
 * This sample uses a PIDF controller to turn the robot to face an angle.
 *
 * It expects a 2-wheel tank drive, using a REV hub.
 */
@TeleOp(name = "Sample - PIDF", group = "ThorCore Samples")
@Disabled
public class SamplePIDF extends OpModeEnhanced {

    // This will look for an imu called simply "imu". If you are using REV hubs, the imu is configured
    // this way by default.
    @Hardware(name = "imu")
    BNO055IMU imu;

    private TankDrive2Wheel rb;

    private PIDF pidf;

    // Tune these values to make your PIDF work better. Unfortunately, this is a process of trial
    // and error - there is no easy way to do it. The tunings you use are unique to your robot.
    // For advice on tuning a PIDF, see
    // https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
    private final double kP = 1; // P tuning of the PIDF
    private final double kI = 0; // I tuning of the PIDF
    private final double kD = 0; // D tuning of the PIDF
    private final double kF = 0; // F tuning of the PIDF

    @Override
    public void init() {
        // Call super.init(). Required for OpModeEnhanced to work.
        super.init();

        // Use the Realizer to realize this class
        Realizer.realize(this, hardwareMap);

        // Initialize the drivetrain
        rb = new TankDrive2Wheel(hardwareMap);

        // Initialise the IMU
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.NDOF;
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        if (!imu.initialize(params)) {
            RobotLog.setGlobalErrorMsg("Unable to initialise IMU. :(");
            requestOpModeStop();
        }

        // Construct a new PIDF.
        pidf = new PIDF(kP, kI, kD, kF, -1000, 1000);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset the integral of the PIDF. Why this is done is explained in the javadoc of the
        // resetControl() method.
        pidf.resetControl();
        // TODO: 11/1/18 Find out if this is the right thing from the imu
        pidf.setSetpoint(imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle);
    }

    @Override
    public void loop() {
        // Call super.loop(). Required for OpModeEnhanced to work.
        super.loop();

        double currentHeading = imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle;

        telemetry.addData("Target heading", pidf.getSetpoint());
        telemetry.addData("Current heading", currentHeading);

        // Update the PIDF with our input
        double pidfOutput = pidf.update(currentHeading);

        // Use the output. If the robot spins crazily instead of tracking a heading, comment out the
        // following line and uncomment out the line after it.
        rb.drive(-pidfOutput, pidfOutput);
        //rb.drive(pidfOutput, -pidfOutput);
    }
}
