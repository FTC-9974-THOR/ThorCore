package org.ftc9974.thorcore.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.ftc9974.thorcore.robot.sensors.USBWebcam;

import java.io.IOException;

@TeleOp(name = "Sample - USBWebcam", group = "ThorCore Samples")
public class SampleUSBWebcam extends OpMode {

    private USBWebcam webcam;

    @Override
    public void init() {
        try {
            webcam = new USBWebcam("Webcam 1", hardwareMap);
            // alternatively, you can pass a WebcamName:
            //webcam = new USBWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

            webcam.startStreaming();
        } catch (IOException e) {
            RobotLog.ee("SampleUSBWebcam", e, "IOException initializing webcam: %s", e.getMessage());
            telemetry.addLine("Init failed.");
        }
    }

    @Override
    public void loop() {
        CameraFrame frame = webcam.getLastFrame();
        // alternatively, you can get a Bitmap. however, this comes at the cost of performance, as
        // converting a CameraFrame to a Bitmap isn't cheap (about 15-60ms of processing time in my tests)
        //Bitmap bitmap = webcam.getLastFrameAsBitmap();

        // make sure you release the frame when you're done with it. this isn't *required*, per se,
        // but it is highly encouraged.
        frame.releaseRef();
        // note that getLastFrameDirect() and getLastFrameAsBitmap() do not need to be released.
        // however, the frame returned by getLastFrameDirect() is a direct reference to the frame buffer,
        // and will be invalidated very quickly. getLastFrameDirect() is intended for internal use,
        // but is exposed in case you want to do low-level operations directly on the frame buffer.
        // getLastFrameAsBitmap(), on the other hand, is just slow.
    }

    @Override
    public void stop() {
        webcam.shutdown();
    }
}
