package org.ftc9974.thorcore;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedList;
import java.util.List;

/**
 * {@link Gamepad}, but better.
 */
public final class GamepadEnhanced {

    public enum Button {
        A, B, X, Y,
        LEFT_BUMPER, RIGHT_BUMPER,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        BACK, GUIDE
    }

    public enum EdgeType {
        RISING,
        FALLING,
        RISING_OR_FALLING
    }

    private static class GamepadState {
        boolean a, b, x, y, leftBumper, rightBumper, dpadUp, dpadDown, dpadLeft, dpadRight, back, guide;
        float leftTrigger, rightTrigger;
        double leftX, leftY, rightX, rightY;

        private GamepadState() {
            a = b = x = y = leftBumper = rightBumper = dpadUp = dpadDown = dpadLeft = dpadRight = back = guide = false;
            leftTrigger = rightTrigger = 0;
            leftX = leftY = rightX = rightY = 0;
        }

        private GamepadState(Gamepad other) {
            Gamepad gamepad;
            try {
                // try to create a local copy that can't be affected by other threads. Gamepads
                // receive updates asynchronously, and their state variables are not thread safe.
                gamepad = new Gamepad();
                gamepad.copy(other);
            } catch (RobotCoreException e) {
                // if we can't make a copy, use the non-thread-safe one and hope nothing gets
                // changed while we store its state
                gamepad = other;
            }
            a = gamepad.a;
            b = gamepad.b;
            x = gamepad.x;
            y = gamepad.y;
            leftBumper = gamepad.left_bumper;
            rightBumper = gamepad.right_bumper;
            dpadUp = gamepad.dpad_up;
            dpadDown = gamepad.dpad_down;
            dpadLeft = gamepad.dpad_left;
            dpadRight = gamepad.dpad_right;
            back = gamepad.back;
            guide = gamepad.guide;
            leftTrigger = gamepad.left_trigger;
            rightTrigger = gamepad.right_trigger;
            leftX = gamepad.left_stick_x;
            leftY = -gamepad.left_stick_y;
            rightX = gamepad.right_stick_x;
            rightY = -gamepad.right_stick_y;
        }

        private boolean getButtonState(Button button) {
            switch (button) {
                case A:
                    return a;
                case B:
                    return b;
                case X:
                    return x;
                case Y:
                    return y;
                case LEFT_BUMPER:
                    return leftBumper;
                case RIGHT_BUMPER:
                    return rightBumper;
                case DPAD_UP:
                    return dpadUp;
                case DPAD_DOWN:
                    return dpadDown;
                case DPAD_LEFT:
                    return dpadLeft;
                case DPAD_RIGHT:
                    return dpadRight;
                case BACK:
                    return back;
                case GUIDE:
                    return guide;
                default:
                    throw new IllegalArgumentException(String.format("Button %s does not exist", button.name()));
            }
        }
    }

    private final Gamepad gamepad;
    // storing two states is necessary for edge detection
    // gamepads have an annoying tendency to update their state variables while you're using them,
    // so storing two states avoids any race conditions
    private GamepadState lastState, currentState;

    // the variable type might look crazy but it's *really* fast
    // uses enum ordinal as index into array
    // index by button then edge type
    private final EnumMap<Button, EnumMap<EdgeType, LinkedList<Runnable>>> bindings;

    public GamepadEnhanced(Gamepad gamepad) {
        this.gamepad = gamepad;
        bindings = new EnumMap<>(Button.class);
        for (Button button : Button.values()) {
            EnumMap<EdgeType, LinkedList<Runnable>> edgeMap = new EnumMap<>(EdgeType.class);
            for (EdgeType edgeType : EdgeType.values()) {
                edgeMap.put(edgeType, new LinkedList<>());
            }
            bindings.put(button, edgeMap);
        }
        advanceState();
    }

    // try to keep execution time short
    // think of it as an interrupt service routine
    public void bind(Button button, EdgeType edgeType, Runnable callback) {
        bindings.get(button).get(edgeType).add(callback);
    }

    private void advanceState() {
        if (currentState == null) {
            lastState = new GamepadState();
        } else {
            lastState = currentState;
        }
        currentState = new GamepadState(gamepad);
    }

    public void update() {
        advanceState();
        for (Button button : bindings.keySet()) {
            boolean lastButtonState = lastState.getButtonState(button);
            boolean currentButtonState = currentState.getButtonState(button);
            boolean rising = !lastButtonState && currentButtonState;
            boolean falling = lastButtonState && !currentButtonState;
            if (rising) {
                for (Runnable callback : bindings.get(button).get(EdgeType.RISING)) {
                    callback.run();
                }
            }
            if (falling) {
                for (Runnable callback : bindings.get(button).get(EdgeType.FALLING)) {
                    callback.run();
                }
            }
            if (rising || falling) {
                for (Runnable callback: bindings.get(button).get(EdgeType.RISING_OR_FALLING)) {
                    callback.run();
                }
            }
        }
    }

    public double getLeftX() {
        return currentState.leftX;
    }

    public double getLeftY() {
        return currentState.leftY;
    }

    public double getRightX() {
        return currentState.rightX;
    }

    public double getRightY() {
        return currentState.rightY;
    }

    public boolean getA() {
        return currentState.a;
    }

    public boolean getB() {
        return currentState.b;
    }

    public boolean getX() {
        return currentState.x;
    }

    public boolean getY() {
        return currentState.y;
    }

    public boolean getLeftBumper() {
        return currentState.leftBumper;
    }

    public boolean getRightBumper() {
        return currentState.rightBumper;
    }

    public boolean getDpadUp() {
        return currentState.dpadUp;
    }

    public boolean getDpadDown() {
        return currentState.dpadDown;
    }

    public boolean getDpadLeft() {
        return currentState.dpadLeft;
    }

    public boolean getDpadRight() {
        return currentState.dpadRight;
    }

    public boolean getBack() {
        return currentState.back;
    }

    public boolean getGuide() {
        return currentState.guide;
    }

    public float getLeftTrigger() {
        return currentState.leftTrigger;
    }

    public float getRightTrigger() {
        return currentState.rightTrigger;
    }

    public boolean getADown() {
        return !lastState.a && currentState.a;
    }

    public boolean getBDown() {
        return !lastState.b && currentState.b;
    }

    public boolean getXDown() {
        return !lastState.x && currentState.x;
    }

    public boolean getYDown() {
        return !lastState.x && currentState.x;
    }

    // todo add the rest of the edges
}
