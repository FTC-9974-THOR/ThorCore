package org.ftc9974.thorcore.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

/**
 * This sample showcases one of the most powerful features of ThorCore - the Realizer.
 * The Realizer loops through every field declared in a class, looking for any fields annotated
 * with {@code @Hardware}. Any field with that annotation is then loaded from the hardware map.
 *
 * For the sake of being an example, this sample controls a pushbot.
 *
 * <b>Note: Fields marked with {@code @Hardware} CANNOT be private!</b>
 */
@TeleOp(name = "Sample - Realized Hardware", group = "_ ThorCore Samples")
@Disabled
public class SampleRealizedHardware extends OpMode {

    @Hardware
    DcMotor leftMotor, rightMotor, armMotor;

    @Hardware
    Servo leftClaw, rightClaw;

    @Override
    public void init() {
        // Call the Realizer.
        Realizer.realize(this, hardwareMap);
        // The above line is equivalent to this:
        // leftMotor = hardwareMap.dcMotor.get("SRH-leftMotor");
        // rightMotor = hardwareMap.dcMotor.get("SRH-rightMotor");
        // armMotor = hardwareMap.dcMotor.get("SRH-armMotor");
        // leftClaw = hardwareMap.servo.get("SRH-leftClaw");
        // rightClaw = hardwareMap.servo.get("SRH-rightClaw");

        // Reverse the left motor
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftMotor.setPower(-gamepad1.left_stick_y);
        rightMotor.setPower(-gamepad1.right_stick_y);

        if (gamepad1.a) {
            armMotor.setPower(1);
        } else if (gamepad1.b) {
            armMotor.setPower(-1);
        } else {
            armMotor.setPower(0);
        }

        if (gamepad1.left_bumper) {
            leftClaw.setPosition(1);
            rightClaw.setPosition(0);
        } else if (gamepad1.right_bumper) {
            leftClaw.setPosition(0);
            rightClaw.setPosition(1);
        }
    }
}
