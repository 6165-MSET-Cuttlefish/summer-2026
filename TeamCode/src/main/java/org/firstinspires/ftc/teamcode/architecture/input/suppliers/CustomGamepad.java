package org.firstinspires.ftc.teamcode.architecture.input.suppliers;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Wraps the SDK's {@link Gamepad} with a per-control {@link EnhancedBooleanSupplier} /
 * {@link EnhancedDoubleSupplier}. While {@link #atRest} is true, every supplier reads as
 * neutral (false / 0.0) so a layered system can have inactive layers physically present
 * but logically silent.
 */
public final class CustomGamepad {
    private final Gamepad gamepad;
    private boolean atRest = false;

    private final List<EnhancedBooleanSupplier> boolSuppliers = new ArrayList<>();
    private final List<EnhancedDoubleSupplier> doubleSuppliers = new ArrayList<>();

    public final EnhancedDoubleSupplier leftStickX, leftStickY;
    public final EnhancedDoubleSupplier rightStickX, rightStickY;
    public final EnhancedDoubleSupplier leftTrigger, rightTrigger;

    public final EnhancedBooleanSupplier a, b, x, y;
    public final EnhancedBooleanSupplier dpadUp, dpadDown, dpadLeft, dpadRight;
    public final EnhancedBooleanSupplier leftBumper, rightBumper;
    public final EnhancedBooleanSupplier leftStickButton, rightStickButton;
    public final EnhancedBooleanSupplier guide, start, back;

    public final EnhancedBooleanSupplier touchpad, touchpadFinger1, touchpadFinger2;
    public final EnhancedDoubleSupplier touchpadFinger1X, touchpadFinger1Y;
    public final EnhancedDoubleSupplier touchpadFinger2X, touchpadFinger2Y;

    public CustomGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;

        leftStickX  = doubleSupplier(() -> atRest ? 0.0 : gamepad.left_stick_x);
        leftStickY  = doubleSupplier(() -> atRest ? 0.0 : gamepad.left_stick_y);
        rightStickX = doubleSupplier(() -> atRest ? 0.0 : gamepad.right_stick_x);
        rightStickY = doubleSupplier(() -> atRest ? 0.0 : gamepad.right_stick_y);
        leftTrigger  = doubleSupplier(() -> atRest ? 0.0 : gamepad.left_trigger);
        rightTrigger = doubleSupplier(() -> atRest ? 0.0 : gamepad.right_trigger);

        a = boolSupplier(() -> !atRest && gamepad.a);
        b = boolSupplier(() -> !atRest && gamepad.b);
        x = boolSupplier(() -> !atRest && gamepad.x);
        y = boolSupplier(() -> !atRest && gamepad.y);

        dpadUp    = boolSupplier(() -> !atRest && gamepad.dpad_up);
        dpadDown  = boolSupplier(() -> !atRest && gamepad.dpad_down);
        dpadLeft  = boolSupplier(() -> !atRest && gamepad.dpad_left);
        dpadRight = boolSupplier(() -> !atRest && gamepad.dpad_right);

        leftBumper  = boolSupplier(() -> !atRest && gamepad.left_bumper);
        rightBumper = boolSupplier(() -> !atRest && gamepad.right_bumper);

        leftStickButton  = boolSupplier(() -> !atRest && gamepad.left_stick_button);
        rightStickButton = boolSupplier(() -> !atRest && gamepad.right_stick_button);

        guide = boolSupplier(() -> !atRest && gamepad.guide);
        start = boolSupplier(() -> !atRest && gamepad.start);
        back  = boolSupplier(() -> !atRest && gamepad.back);

        touchpad         = boolSupplier(() -> !atRest && gamepad.touchpad);
        touchpadFinger1  = boolSupplier(() -> !atRest && gamepad.touchpad_finger_1);
        touchpadFinger2  = boolSupplier(() -> !atRest && gamepad.touchpad_finger_2);
        touchpadFinger1X = doubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_1_x);
        touchpadFinger1Y = doubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_1_y);
        touchpadFinger2X = doubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_2_x);
        touchpadFinger2Y = doubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_2_y);
    }

    private EnhancedBooleanSupplier boolSupplier(BooleanSupplier source) {
        EnhancedBooleanSupplier s = new EnhancedBooleanSupplier(source);
        boolSuppliers.add(s);
        return s;
    }

    private EnhancedDoubleSupplier doubleSupplier(DoubleSupplier source) {
        EnhancedDoubleSupplier s = new EnhancedDoubleSupplier(source);
        doubleSuppliers.add(s);
        return s;
    }

    public void setAtRest(boolean atRest) {
        boolean wasAtRest = this.atRest;
        this.atRest = atRest;
        // Rest→active transition: prime suppliers to "this is the new baseline" so a button
        // already held at the time of activation doesn't fire wasJustPressed.
        if (wasAtRest && !atRest) primeAllSuppliers();
    }

    public boolean isAtRest() {
        return atRest;
    }

    public void invalidateAll() {
        for (int i = 0; i < boolSuppliers.size(); i++) boolSuppliers.get(i).invalidate();
        for (int i = 0; i < doubleSuppliers.size(); i++) doubleSuppliers.get(i).invalidate();
    }

    private void primeAllSuppliers() {
        for (int i = 0; i < boolSuppliers.size(); i++) boolSuppliers.get(i).primeToCurrentState();
        for (int i = 0; i < doubleSuppliers.size(); i++) doubleSuppliers.get(i).primeToCurrentState();
    }

    /** Escape hatch to the underlying SDK gamepad — useful for fields not wrapped here. */
    public Gamepad getRawGamepad() {
        return gamepad;
    }
}
