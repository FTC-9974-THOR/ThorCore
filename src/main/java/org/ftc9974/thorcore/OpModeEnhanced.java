package org.ftc9974.thorcore;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * {@link OpMode}, but better.
 *
 * Make sure you call {@code super.init()} in {@code init()} and {@code super.loop()} in
 * {@code loop()}.
 */
public abstract class OpModeEnhanced extends OpMode {

    protected GamepadEnhanced gamepad1, gamepad2;

    @Override
    public void init() {
        this.gamepad1 = new GamepadEnhanced(super.gamepad1);
        this.gamepad2 = new GamepadEnhanced(super.gamepad2);
    }

    @Override
    public void loop() {
        this.gamepad1.update();
        this.gamepad2.update();
    }
}
