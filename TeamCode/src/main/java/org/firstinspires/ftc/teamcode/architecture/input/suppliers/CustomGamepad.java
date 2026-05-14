package org.firstinspires.ftc.teamcode.architecture.input.suppliers;

import com.qualcomm.robotcore.hardware.Gamepad;

public class CustomGamepad {
    private final Gamepad gamepad;
    private boolean atRest = false;

    private EnhancedDoubleSupplier leftStickX;
    private EnhancedDoubleSupplier leftStickY;
    private EnhancedDoubleSupplier rightStickX;
    private EnhancedDoubleSupplier rightStickY;
    private EnhancedDoubleSupplier leftTrigger;
    private EnhancedDoubleSupplier rightTrigger;

    private EnhancedBooleanSupplier dpadUp;
    private EnhancedBooleanSupplier dpadDown;
    private EnhancedBooleanSupplier dpadLeft;
    private EnhancedBooleanSupplier dpadRight;

    private EnhancedBooleanSupplier a;
    private EnhancedBooleanSupplier b;
    private EnhancedBooleanSupplier x;
    private EnhancedBooleanSupplier y;

    private EnhancedBooleanSupplier guide;
    private EnhancedBooleanSupplier start;
    private EnhancedBooleanSupplier back;

    private EnhancedBooleanSupplier leftBumper;
    private EnhancedBooleanSupplier rightBumper;

    private EnhancedBooleanSupplier leftStickButton;
    private EnhancedBooleanSupplier rightStickButton;

    private EnhancedBooleanSupplier touchpad;
    private EnhancedBooleanSupplier touchpadFinger1;
    private EnhancedBooleanSupplier touchpadFinger2;
    private EnhancedDoubleSupplier touchpadFinger1X;
    private EnhancedDoubleSupplier touchpadFinger1Y;
    private EnhancedDoubleSupplier touchpadFinger2X;
    private EnhancedDoubleSupplier touchpadFinger2Y;

    public CustomGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
        initializeSuppliers();
    }

    private void initializeSuppliers() {
        leftStickX = new EnhancedDoubleSupplier(() -> atRest ? 0.0 : gamepad.left_stick_x);
        leftStickY = new EnhancedDoubleSupplier(() -> atRest ? 0.0 : gamepad.left_stick_y);
        rightStickX = new EnhancedDoubleSupplier(() -> atRest ? 0.0 : gamepad.right_stick_x);
        rightStickY = new EnhancedDoubleSupplier(() -> atRest ? 0.0 : gamepad.right_stick_y);
        leftTrigger = new EnhancedDoubleSupplier(() -> atRest ? 0.0 : gamepad.left_trigger);
        rightTrigger = new EnhancedDoubleSupplier(() -> atRest ? 0.0 : gamepad.right_trigger);

        dpadUp = new EnhancedBooleanSupplier(() -> !atRest && gamepad.dpad_up);
        dpadDown = new EnhancedBooleanSupplier(() -> !atRest && gamepad.dpad_down);
        dpadLeft = new EnhancedBooleanSupplier(() -> !atRest && gamepad.dpad_left);
        dpadRight = new EnhancedBooleanSupplier(() -> !atRest && gamepad.dpad_right);

        a = new EnhancedBooleanSupplier(() -> !atRest && gamepad.a);
        b = new EnhancedBooleanSupplier(() -> !atRest && gamepad.b);
        x = new EnhancedBooleanSupplier(() -> !atRest && gamepad.x);
        y = new EnhancedBooleanSupplier(() -> !atRest && gamepad.y);

        guide = new EnhancedBooleanSupplier(() -> !atRest && gamepad.guide);
        start = new EnhancedBooleanSupplier(() -> !atRest && gamepad.start);
        back = new EnhancedBooleanSupplier(() -> !atRest && gamepad.back);

        leftBumper = new EnhancedBooleanSupplier(() -> !atRest && gamepad.left_bumper);
        rightBumper = new EnhancedBooleanSupplier(() -> !atRest && gamepad.right_bumper);

        leftStickButton = new EnhancedBooleanSupplier(() -> !atRest && gamepad.left_stick_button);
        rightStickButton = new EnhancedBooleanSupplier(() -> !atRest && gamepad.right_stick_button);

        touchpad = new EnhancedBooleanSupplier(() -> !atRest && gamepad.touchpad);
        touchpadFinger1 = new EnhancedBooleanSupplier(() -> !atRest && gamepad.touchpad_finger_1);
        touchpadFinger2 = new EnhancedBooleanSupplier(() -> !atRest && gamepad.touchpad_finger_2);
        touchpadFinger1X =
                new EnhancedDoubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_1_x);
        touchpadFinger1Y =
                new EnhancedDoubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_1_y);
        touchpadFinger2X =
                new EnhancedDoubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_2_x);
        touchpadFinger2Y =
                new EnhancedDoubleSupplier(() -> atRest ? 0.0 : gamepad.touchpad_finger_2_y);
    }

    public EnhancedDoubleSupplier getLeftStickX() {
        return leftStickX;
    }

    public void setLeftStickX(EnhancedDoubleSupplier leftStickX) {
        this.leftStickX = leftStickX;
    }

    public EnhancedDoubleSupplier getLeftStickY() {
        return leftStickY;
    }

    public void setLeftStickY(EnhancedDoubleSupplier leftStickY) {
        this.leftStickY = leftStickY;
    }

    public EnhancedDoubleSupplier getRightStickX() {
        return rightStickX;
    }

    public void setRightStickX(EnhancedDoubleSupplier rightStickX) {
        this.rightStickX = rightStickX;
    }

    public EnhancedDoubleSupplier getRightStickY() {
        return rightStickY;
    }

    public void setRightStickY(EnhancedDoubleSupplier rightStickY) {
        this.rightStickY = rightStickY;
    }

    public EnhancedDoubleSupplier getLeftTrigger() {
        return leftTrigger;
    }

    public void setLeftTrigger(EnhancedDoubleSupplier leftTrigger) {
        this.leftTrigger = leftTrigger;
    }

    public EnhancedDoubleSupplier getRightTrigger() {
        return rightTrigger;
    }

    public void setRightTrigger(EnhancedDoubleSupplier rightTrigger) {
        this.rightTrigger = rightTrigger;
    }

    public EnhancedBooleanSupplier getDpadUp() {
        return dpadUp;
    }

    public void setDpadUp(EnhancedBooleanSupplier dpadUp) {
        this.dpadUp = dpadUp;
    }

    public EnhancedBooleanSupplier getDpadDown() {
        return dpadDown;
    }

    public void setDpadDown(EnhancedBooleanSupplier dpadDown) {
        this.dpadDown = dpadDown;
    }

    public EnhancedBooleanSupplier getDpadLeft() {
        return dpadLeft;
    }

    public void setDpadLeft(EnhancedBooleanSupplier dpadLeft) {
        this.dpadLeft = dpadLeft;
    }

    public EnhancedBooleanSupplier getDpadRight() {
        return dpadRight;
    }

    public void setDpadRight(EnhancedBooleanSupplier dpadRight) {
        this.dpadRight = dpadRight;
    }

    public EnhancedBooleanSupplier getA() {
        return a;
    }

    public void setA(EnhancedBooleanSupplier a) {
        this.a = a;
    }

    public EnhancedBooleanSupplier getB() {
        return b;
    }

    public void setB(EnhancedBooleanSupplier b) {
        this.b = b;
    }

    public EnhancedBooleanSupplier getX() {
        return x;
    }

    public void setX(EnhancedBooleanSupplier x) {
        this.x = x;
    }

    public EnhancedBooleanSupplier getY() {
        return y;
    }

    public void setY(EnhancedBooleanSupplier y) {
        this.y = y;
    }

    public EnhancedBooleanSupplier getGuide() {
        return guide;
    }

    public void setGuide(EnhancedBooleanSupplier guide) {
        this.guide = guide;
    }

    public EnhancedBooleanSupplier getStart() {
        return start;
    }

    public void setStart(EnhancedBooleanSupplier start) {
        this.start = start;
    }

    public EnhancedBooleanSupplier getBack() {
        return back;
    }

    public void setBack(EnhancedBooleanSupplier back) {
        this.back = back;
    }

    public EnhancedBooleanSupplier getLeftBumper() {
        return leftBumper;
    }

    public void setLeftBumper(EnhancedBooleanSupplier leftBumper) {
        this.leftBumper = leftBumper;
    }

    public EnhancedBooleanSupplier getRightBumper() {
        return rightBumper;
    }

    public void setRightBumper(EnhancedBooleanSupplier rightBumper) {
        this.rightBumper = rightBumper;
    }

    public EnhancedBooleanSupplier getLeftStickButton() {
        return leftStickButton;
    }

    public void setLeftStickButton(EnhancedBooleanSupplier leftStickButton) {
        this.leftStickButton = leftStickButton;
    }

    public EnhancedBooleanSupplier getRightStickButton() {
        return rightStickButton;
    }

    public void setRightStickButton(EnhancedBooleanSupplier rightStickButton) {
        this.rightStickButton = rightStickButton;
    }

    public EnhancedBooleanSupplier getTouchpad() {
        return touchpad;
    }

    public void setTouchpad(EnhancedBooleanSupplier touchpad) {
        this.touchpad = touchpad;
    }

    public EnhancedBooleanSupplier getTouchpadFinger1() {
        return touchpadFinger1;
    }

    public void setTouchpadFinger1(EnhancedBooleanSupplier touchpadFinger1) {
        this.touchpadFinger1 = touchpadFinger1;
    }

    public EnhancedBooleanSupplier getTouchpadFinger2() {
        return touchpadFinger2;
    }

    public void setTouchpadFinger2(EnhancedBooleanSupplier touchpadFinger2) {
        this.touchpadFinger2 = touchpadFinger2;
    }

    public EnhancedDoubleSupplier getTouchpadFinger1X() {
        return touchpadFinger1X;
    }

    public void setTouchpadFinger1X(EnhancedDoubleSupplier touchpadFinger1X) {
        this.touchpadFinger1X = touchpadFinger1X;
    }

    public EnhancedDoubleSupplier getTouchpadFinger1Y() {
        return touchpadFinger1Y;
    }

    public void setTouchpadFinger1Y(EnhancedDoubleSupplier touchpadFinger1Y) {
        this.touchpadFinger1Y = touchpadFinger1Y;
    }

    public EnhancedDoubleSupplier getTouchpadFinger2X() {
        return touchpadFinger2X;
    }

    public void setTouchpadFinger2X(EnhancedDoubleSupplier touchpadFinger2X) {
        this.touchpadFinger2X = touchpadFinger2X;
    }

    public EnhancedDoubleSupplier getTouchpadFinger2Y() {
        return touchpadFinger2Y;
    }

    public void setTouchpadFinger2Y(EnhancedDoubleSupplier touchpadFinger2Y) {
        this.touchpadFinger2Y = touchpadFinger2Y;
    }

    public EnhancedBooleanSupplier getCircle() {
        return getB();
    }

    public void setCircle(EnhancedBooleanSupplier circle) {
        setB(circle);
    }

    public EnhancedBooleanSupplier getCross() {
        return getA();
    }

    public void setCross(EnhancedBooleanSupplier cross) {
        setA(cross);
    }

    public EnhancedBooleanSupplier getTriangle() {
        return getY();
    }

    public void setTriangle(EnhancedBooleanSupplier triangle) {
        setY(triangle);
    }

    public EnhancedBooleanSupplier getSquare() {
        return getX();
    }

    public void setSquare(EnhancedBooleanSupplier square) {
        setX(square);
    }

    public EnhancedBooleanSupplier getOptions() {
        return getStart();
    }

    public void setOptions(EnhancedBooleanSupplier options) {
        setStart(options);
    }

    public void setAtRest(boolean atRest) {
        boolean wasAtRest = this.atRest;
        this.atRest = atRest;

        if (wasAtRest && !atRest) {
            primeAllSuppliersToCurrentState();
        }
    }

    public boolean isAtRest() {
        return atRest;
    }

    private void primeAllSuppliersToCurrentState() {
        leftStickX.primeToCurrentState();
        leftStickY.primeToCurrentState();
        rightStickX.primeToCurrentState();
        rightStickY.primeToCurrentState();
        leftTrigger.primeToCurrentState();
        rightTrigger.primeToCurrentState();
        touchpadFinger1X.primeToCurrentState();
        touchpadFinger1Y.primeToCurrentState();
        touchpadFinger2X.primeToCurrentState();
        touchpadFinger2Y.primeToCurrentState();

        dpadUp.primeToCurrentState();
        dpadDown.primeToCurrentState();
        dpadLeft.primeToCurrentState();
        dpadRight.primeToCurrentState();
        a.primeToCurrentState();
        b.primeToCurrentState();
        x.primeToCurrentState();
        y.primeToCurrentState();
        guide.primeToCurrentState();
        start.primeToCurrentState();
        back.primeToCurrentState();
        leftBumper.primeToCurrentState();
        rightBumper.primeToCurrentState();
        leftStickButton.primeToCurrentState();
        rightStickButton.primeToCurrentState();
        touchpad.primeToCurrentState();
        touchpadFinger1.primeToCurrentState();
        touchpadFinger2.primeToCurrentState();
    }

    public void invalidateAll() {
        leftStickX.invalidate();
        leftStickY.invalidate();
        rightStickX.invalidate();
        rightStickY.invalidate();
        leftTrigger.invalidate();
        rightTrigger.invalidate();
        touchpadFinger1X.invalidate();
        touchpadFinger1Y.invalidate();
        touchpadFinger2X.invalidate();
        touchpadFinger2Y.invalidate();
        dpadUp.invalidate();
        dpadDown.invalidate();
        dpadLeft.invalidate();
        dpadRight.invalidate();
        a.invalidate();
        b.invalidate();
        x.invalidate();
        y.invalidate();
        guide.invalidate();
        start.invalidate();
        back.invalidate();
        leftBumper.invalidate();
        rightBumper.invalidate();
        leftStickButton.invalidate();
        rightStickButton.invalidate();
        touchpad.invalidate();
        touchpadFinger1.invalidate();
        touchpadFinger2.invalidate();
    }

    public boolean isLayerActive() {
        return !atRest;
    }

    public Gamepad getRawGamepad() {
        return gamepad;
    }
}
