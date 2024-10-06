package org.firstinspires.ftc.teamcode.util;



public class Button {
    public enum States {
        TAP,   // moment press down
        DOUBLE_TAP, // pressed down in quick succession
        HELD,   // continued press down
        UP,     // moment of release
        OFF,    // continued release
        NOT_INITIALIZED
    }

    private static final int doubleTapIntervalMs = 500;
    private States state;
    private long lastTapped = -1;

    public Button() {
        state = States.NOT_INITIALIZED;
    }

    public States getState() {
        return state;
    }

    private boolean doubleTapIntervalNotSet() {
        return doubleTapIntervalMs == -1;
    }

    //## Safety - this method assumes that the buttonPressed parameter always
    // refers to the same button on the gamepad. There is no checking. The
    // update method is designed to be called once for each cycle of the main
    // loop in a TeleOp OpMode.
    //?? For a true toggle you could use a double-tap to turn an operation on,
    // for example displaying a telemetry message on the Driver Station, and
    // a second double-tap to turn the operation off. Or, if you want a single-
    // tap toggle you could supply an enum to the constructor for SINGLE_TAP_TOGGLE
    // or DOUBLE_TAP_TOGGLE.
    public States update(boolean buttonPressed) {
        if (buttonPressed) {
            if (state == States.OFF || state == States.UP || state == States.NOT_INITIALIZED) {
                if (System.currentTimeMillis() - lastTapped < doubleTapIntervalMs) state = States.DOUBLE_TAP;
                else {
                    lastTapped = System.currentTimeMillis();
                    state = States.TAP;
                }
            }
            else {
                state = States.HELD;
            }
        }
        else {
            if (state == States.HELD || state == States.TAP || state == States.DOUBLE_TAP) {
                state = States.UP;
            }
            else {
                state = States.OFF;
            }
        }
        return state;
    }

    public boolean is(States state) {
        return this.state == state;
    }

}