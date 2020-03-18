package org.ftc9974.thorcore;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
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

    private Gamepad gamepad;
    private boolean lastA, lastB, lastX, lastY, lastLeftBumper, lastRightBumper, lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight, lastBack, lastGuide;
    private float lastLeftTrigger, lastRightTrigger;
    private double lastLeftX, lastLeftY, lastRightX, lastRightY;

    private List<Runnable> aEvents, bEvents, xEvents, yEvents, leftBumperEvents, rightBumperEvents, dpadDownEvents, dpadUpEvents, dpadLeftEvents, dpadRightEvents, backEvents, guideEvents;

    public GamepadEnhanced(Gamepad gamepad) {
        this.gamepad = gamepad;
        aEvents = new ArrayList<>();
        bEvents = new ArrayList<>();
        xEvents = new ArrayList<>();
        yEvents = new ArrayList<>();
        leftBumperEvents = new ArrayList<>();
        rightBumperEvents = new ArrayList<>();
        dpadDownEvents = new ArrayList<>();
        dpadUpEvents = new ArrayList<>();
        dpadLeftEvents = new ArrayList<>();
        dpadRightEvents = new ArrayList<>();
        backEvents = new ArrayList<>();
        guideEvents = new ArrayList<>();
    }

    public void bind(Button button, Runnable func) {
        switch (button) {
            case A:
                aEvents.add(func);
                break;
            case B:
                bEvents.add(func);
                break;
            case X:
                xEvents.add(func);
                break;
            case Y:
                yEvents.add(func);
                break;
            case LEFT_BUMPER:
                leftBumperEvents.add(func);
                break;
            case RIGHT_BUMPER:
                rightBumperEvents.add(func);
                break;
            case DPAD_UP:
                dpadUpEvents.add(func);
                break;
            case DPAD_DOWN:
                dpadDownEvents.add(func);
                break;
            case DPAD_LEFT:
                dpadLeftEvents.add(func);
                break;
            case DPAD_RIGHT:
                dpadRightEvents.add(func);
                break;
            case BACK:
                backEvents.add(func);
                break;
            case GUIDE:
                guideEvents.add(func);
                break;
            default: // utter, complete paranoia
                break;
        }
    }

    public void update() {
        if (gamepad.a && !lastA) {
            for (Runnable func : aEvents) {
                func.run();
            }
        }
        if (gamepad.b && !lastB) {
            for (Runnable func : bEvents) {
                func.run();
            }
        }
        if (gamepad.x && !lastX) {
            for (Runnable func : xEvents) {
                func.run();
            }
        }
        if (gamepad.y && !lastY) {
            for (Runnable func : yEvents) {
                func.run();
            }
        }

        if (gamepad.left_bumper && !lastLeftBumper) {
            for (Runnable func : leftBumperEvents) {
                func.run();
            }
        }
        if (gamepad.right_bumper && !lastRightBumper) {
            for (Runnable func : rightBumperEvents) {
                func.run();
            }
        }

        if (gamepad.dpad_up && !lastDpadUp) {
            for (Runnable func : dpadUpEvents) {
                func.run();
            }
        }
        if (gamepad.dpad_down && !lastDpadDown) {
            for (Runnable func : dpadDownEvents) {
                func.run();
            }
        }
        if (gamepad.dpad_left && !lastDpadLeft) {
            for (Runnable func : dpadLeftEvents) {
                func.run();
            }
        }
        if (gamepad.dpad_right && !lastDpadRight) {
            for (Runnable func : dpadRightEvents) {
                func.run();
            }
        }

        lastA = gamepad.a;
        lastB = gamepad.b;
        lastX = gamepad.x;
        lastY = gamepad.y;
        lastLeftBumper = gamepad.left_bumper;
        lastRightBumper = gamepad.right_bumper;
        lastDpadUp = gamepad.dpad_up;
        lastDpadDown = gamepad.dpad_down;
        lastDpadLeft = gamepad.dpad_left;
        lastDpadRight = gamepad.dpad_right;
        lastBack = gamepad.back;
        lastGuide = gamepad.guide;

        lastLeftTrigger = gamepad.left_trigger;
        lastRightTrigger = gamepad.right_trigger;

        lastLeftX = -gamepad.left_stick_x;
        lastLeftY = -gamepad.left_stick_y;
        lastRightX = -gamepad.right_stick_x;
        lastRightY = -gamepad.right_stick_y;
    }

    public boolean atRest() {
        return gamepad.atRest();
    }

    public double getLeftX() {
        return -gamepad.left_stick_x;
    }

    public double getLeftY() {
        return -gamepad.left_stick_y;
    }

    public double getRightX() {
        return -gamepad.right_stick_x;
    }

    public double getRightY() {
        return -gamepad.right_stick_y;
    }

    public boolean getA() {
        return gamepad.a;
    }

    public boolean getB() {
        return gamepad.b;
    }

    public boolean getX() {
        return gamepad.x;
    }

    public boolean getY() {
        return gamepad.y;
    }

    public boolean getLeftBumper() {
        return gamepad.left_bumper;
    }

    public boolean getRightBumper() {
        return gamepad.right_bumper;
    }

    public boolean getDpadUp() {
        return gamepad.dpad_up;
    }

    public boolean getDpadDown() {
        return gamepad.dpad_down;
    }

    public boolean getDpadLeft() {
        return gamepad.dpad_left;
    }

    public boolean getDpadRight() {
        return gamepad.dpad_right;
    }

    public boolean getBack() {
        return gamepad.back;
    }

    public boolean getGuide() {
        return gamepad.guide;
    }

    public float getLeftTrigger() {
        return gamepad.left_trigger;
    }

    public float getRightTrigger() {
        return gamepad.right_trigger;
    }

    public boolean getADown() {
        return !lastA && gamepad.a;
    }

    public boolean getBDown() {
        return !lastB && gamepad.b;
    }

    public boolean getXDown() {
        return !lastX && gamepad.x;
    }

    public boolean getYDown() {
        return !lastY && gamepad.y;
    }
}
