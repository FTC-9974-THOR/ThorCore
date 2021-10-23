package org.ftc9974.thorcore.samples;

import android.graphics.Rect;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.ftc9974.thorcore.seasonal.freightfrenzy.NEONMarkerVision;
import org.ftc9974.thorcore.util.TimingUtilities;

import java.io.IOException;
import java.util.Locale;

@Autonomous(name = "Sample - NEONVision Mask", group = "ThorCore Samples")
public class SampleNEONVisionMask extends LinearOpMode {

    private static final String TAG = "SampleNEONVisionMask";

    // regions of interest - these are the areas of the frame that the left, center, and right positions
    // will be looked for in
    private static final Rect LEFT_RECT = new Rect(0, 0, 200, 480);
    private static final Rect CENTER_RECT = new Rect(220, 0, 420, 480);
    private static final Rect RIGHT_RECT = new Rect(440, 0, 640, 480);

    private NEONMarkerVision markerVision;
    private boolean ready;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            markerVision = new NEONMarkerVision(hardwareMap.get(WebcamName.class, "Webcam 1"));
            markerVision.startStreaming();

            // these actually match everything *but* bright green
            markerVision.setLowColor(16, 128 - 20, 128 - 20);
            markerVision.setHighColor(215, 128 + 20, 128 + 20);

            // inversion logic: if you manage to find tunings that match everything but what you want
            // to match, you can invert them. if inversion is active, each result is recalculated as
            // result = inversionReference - result. this basically goes from "how much of the image
            // doesn't match the target" to "how much of the image matches the target". inversionReference
            // should be set to what the maximum value that the result could be if there was no target
            // visible at all. since the masks will be filled with the value 127 in the areas we care
            // about, we multiply the size of the area by 127 to get the inversion reference.
            markerVision.setLeftInversionReference(127 * LEFT_RECT.width() * LEFT_RECT.height());
            markerVision.setCenterInversionReference(127 * CENTER_RECT.width() * CENTER_RECT.height());
            markerVision.setRightInversionReference(127 * RIGHT_RECT.width() * RIGHT_RECT.height());

            // enable inversion logic
            markerVision.setInverted(true);

            // mark the areas we care about in the masks
            markerVision.leftMask.drawFilledRectangle(LEFT_RECT, (byte) 127);
            markerVision.centerMask.drawFilledRectangle(CENTER_RECT, (byte) 127);
            markerVision.rightMask.drawFilledRectangle(RIGHT_RECT, (byte) 127);

            ready = true;
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "Error initializing NEONMarkerVision");
        }

        telemetry.addData("Ready", ready);
        telemetry.update();

        if (!ready) {
            waitForStart();
        } else {
            while (!isStarted() && !isStopRequested()) {
                // gamepads are disabled during camera stream, so make sure you exit the stream
                // before switching display position
                if (gamepad1.dpad_left) {
                    markerVision.setDisplayPosition(NEONMarkerVision.MarkerPosition.LEFT);
                }
                if (gamepad1.dpad_up) {
                    markerVision.setDisplayPosition(NEONMarkerVision.MarkerPosition.CENTER);
                }
                if (gamepad1.dpad_right) {
                    markerVision.setDisplayPosition(NEONMarkerVision.MarkerPosition.RIGHT);
                }
                if (gamepad1.dpad_down) {
                    markerVision.setDisplayPosition(NEONMarkerVision.MarkerPosition.UNKNOWN);
                }

                telemetry.addData("Ready", ready);

                telemetry.addData("Position", markerVision.getPosition());
                telemetry.addData("Left Result", markerVision.getLeftResult());
                telemetry.addData("Center Result", markerVision.getCenterResult());
                telemetry.addData("Right Result", markerVision.getRightResult());
                telemetry.addData("Execution Time", "%f ms", 1000 * markerVision.getExecutionTime());
                telemetry.addLine(String.format(Locale.getDefault(), "If the camera was fast enough, the pipeline could run at %.2f fps.", 1.0 / markerVision.getExecutionTime()));
                telemetry.addData("Active Display Region", markerVision.getDisplayPosition());

                telemetry.update();
            }
            if (isStopRequested()) {
                shutdown();
                return;
            }
            markerVision.stopStreaming();

            while (!isStopRequested()) {
                telemetry.addData("Position", markerVision.getPosition());
                telemetry.addData("Left Result", markerVision.getLeftResult());
                telemetry.addData("Center Result", markerVision.getCenterResult());
                telemetry.addData("Right Result", markerVision.getRightResult());
                telemetry.addData("Execution Time", "%f ms", 1000 * markerVision.getExecutionTime());
                telemetry.addLine(String.format(Locale.getDefault(), "If the camera was fast enough, the pipeline could run at %.2f fps.", 1.0 / markerVision.getExecutionTime()));

                telemetry.update();
            }

            shutdown();
        }
    }

    private void shutdown() {
        markerVision.stopStreaming();
        // closing the USB camera is slow enough it needs to be run on a separate thread. the default
        // time given to an opmode during stop() is too short to completely shutdown cleanly if we
        // ran this on the opmode thread.
        TimingUtilities.runOnSeparateThread(() -> markerVision.shutdown());
    }
}
