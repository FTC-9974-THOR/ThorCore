package org.ftc9974.thorcore.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.ftc9974.thorcore.robot.sensors.Pixy2USB;
import org.ftc9974.thorcore.robot.sensors.Pixy2USBManager;

import java.util.List;

/**
 * Sample OpMode for interfacing with Pixies over USB.
 *
 * This OpMode will list all connected Pixies during the init loop. If any are connected, it will
 * use the first one found for the loop() part of the OpMode (henceforth referred to as the active
 * Pixy).
 *
 * On start(), the OpMode turns on the upper lamps of the active Pixy.
 *
 * During loop(), the OpMode lists all color connected blocks detected by the active Pixy.
 *
 * On stop(), the OpMode turns off the lamps of the active Pixy.
 * 
 * @see Pixy2USB
 */
@TeleOp(name = "Sample - Pixy2USB", group = "ThorCore Samples")
//@Disabled
public class SamplePixy2USB extends OpMode {

    private Pixy2USB pixy;

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator("<br/>");
    }

    @Override
    public void init_loop() {
        if (Pixy2USBManager.getConnectedPixies().isEmpty()) {
            telemetry.addLine("No Pixies found.");
            return;
        }

        telemetry.addLine("Pixies connected:");
        for (Pixy2USB p : Pixy2USBManager.getConnectedPixies()) {
            if (pixy == null) {
                pixy = p;
                pixy.setProgram(Pixy2USB.Program.VIDEO);
                pixy.startCameraStream();
            }
            telemetry.addLine("Pixy:<br/>")
                    .addData("Version", p.getVersion())
                    .addData("Resolution", p.getResolution())
                    .addData("Serial ID", p.getSerial());
        }

        if (pixy != null) {
            telemetry.addLine("Blocks:");
            List<Pixy2USB.Block> blocks = pixy.getBlocks();
            if (blocks == null) return;
            for (Pixy2USB.Block block : blocks) {
                telemetry.addLine("Block:<br/>")
                        .addData("Position", block.position)
                        .addData("Angle", block.angle)
                        .addData("Size", block.size)
                        .addData("Tracking ID", block.trackingIndex)
                        .addData("Signature ID", block.signature)
                        .addData("Age", block.age);
            }
        }
    }

    @Override
    public void start() {
        telemetry.setMsTransmissionInterval(50);
        if (pixy != null) {
            pixy.setProgram(Pixy2USB.Program.COLOR_CONNECTED_COMPONENTS);
            pixy.setLamps(true, false);
        }
    }

    @Override
    public void loop() {
        if (pixy == null) {
            telemetry.addLine("Please connect a Pixy and restart the OpMode.");
            return;
        }

        telemetry.addLine("Blocks:");
        List<Pixy2USB.Block> blocks = pixy.getBlocks();
        if (blocks == null) return;
        for (Pixy2USB.Block block : blocks) {
            telemetry.addLine("Block:<br/>")
                    .addData("Position", block.position)
                    .addData("Angle", block.angle)
                    .addData("Size", block.size)
                    .addData("Tracking ID", block.trackingIndex)
                    .addData("Signature ID", block.signature)
                    .addData("Age", block.age);
        }
    }

    @Override
    public void stop() {
        if (pixy != null) {
            pixy.setLamps(false, false);
        }
    }
}
