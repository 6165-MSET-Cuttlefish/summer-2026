package org.firstinspires.ftc.teamcode.architecture.input;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.firstinspires.ftc.teamcode.architecture.input.suppliers.CustomGamepad;
import org.firstinspires.ftc.teamcode.architecture.input.suppliers.EnhancedBooleanSupplier;
import org.firstinspires.ftc.teamcode.architecture.input.suppliers.EnhancedDoubleSupplier;

/**
 * A gamepad façade that delegates to whichever {@link CustomGamepad} is currently active in
 * the underlying {@link LayerStack}. Each accessor returns a stable supplier whose value is
 * sourced from the active layer; if no layer is active or a layer switch is in progress,
 * suppliers return neutral (false / 0.0).
 */
public class LayeredGamepad<T> {
    private final LayerStack<T> layerStack;
    private boolean suppressInputsUntilNextInvalidate = false;

    private final EnhancedDoubleSupplier leftStickX, leftStickY;
    private final EnhancedDoubleSupplier rightStickX, rightStickY;
    private final EnhancedDoubleSupplier leftTrigger, rightTrigger;
    private final EnhancedBooleanSupplier a, b, x, y;
    private final EnhancedBooleanSupplier dpadUp, dpadDown, dpadLeft, dpadRight;
    private final EnhancedBooleanSupplier leftBumper, rightBumper;
    private final EnhancedBooleanSupplier start, back, guide;
    private final EnhancedBooleanSupplier leftStickButton, rightStickButton;
    private final EnhancedBooleanSupplier touchpad;
    private final EnhancedDoubleSupplier touchpadFinger1X, touchpadFinger1Y;

    public LayeredGamepad(LayerStack<T> layerStack) {
        if (layerStack == null) throw new IllegalArgumentException("layerStack cannot be null");
        this.layerStack = layerStack;

        leftStickX  = mappedDouble(cg -> cg.leftStickX);
        leftStickY  = mappedDouble(cg -> cg.leftStickY);
        rightStickX = mappedDouble(cg -> cg.rightStickX);
        rightStickY = mappedDouble(cg -> cg.rightStickY);
        leftTrigger  = mappedDouble(cg -> cg.leftTrigger);
        rightTrigger = mappedDouble(cg -> cg.rightTrigger);

        a = mappedBool(cg -> cg.a);
        b = mappedBool(cg -> cg.b);
        x = mappedBool(cg -> cg.x);
        y = mappedBool(cg -> cg.y);

        dpadUp    = mappedBool(cg -> cg.dpadUp);
        dpadDown  = mappedBool(cg -> cg.dpadDown);
        dpadLeft  = mappedBool(cg -> cg.dpadLeft);
        dpadRight = mappedBool(cg -> cg.dpadRight);

        leftBumper  = mappedBool(cg -> cg.leftBumper);
        rightBumper = mappedBool(cg -> cg.rightBumper);

        start = mappedBool(cg -> cg.start);
        back  = mappedBool(cg -> cg.back);
        guide = mappedBool(cg -> cg.guide);

        leftStickButton  = mappedBool(cg -> cg.leftStickButton);
        rightStickButton = mappedBool(cg -> cg.rightStickButton);

        touchpad         = mappedBool(cg -> cg.touchpad);
        touchpadFinger1X = mappedDouble(cg -> cg.touchpadFinger1X);
        touchpadFinger1Y = mappedDouble(cg -> cg.touchpadFinger1Y);
    }

    private EnhancedBooleanSupplier mappedBool(Function<CustomGamepad, EnhancedBooleanSupplier> getter) {
        BooleanSupplier source = () -> {
            CustomGamepad active = getActiveGamepad();
            return active != null && getter.apply(active).getState();
        };
        return new EnhancedBooleanSupplier(source);
    }

    private EnhancedDoubleSupplier mappedDouble(Function<CustomGamepad, EnhancedDoubleSupplier> getter) {
        DoubleSupplier source = () -> {
            CustomGamepad active = getActiveGamepad();
            return active != null ? getter.apply(active).getState() : 0.0;
        };
        return new EnhancedDoubleSupplier(source);
    }

    public LayerStack<T> getLayerStack() { return layerStack; }

    public CustomGamepad getActiveGamepad() {
        if (suppressInputsUntilNextInvalidate) return null;
        return layerStack.getGamepad();
    }

    public T getCurrentLayer() { return layerStack.getLayer(); }

    public void setLayer(T layer) {
        if (layer == null) throw new IllegalArgumentException("layer cannot be null");
        if (layer.equals(layerStack.getLayer())) return;

        layerStack.setLayer(layer);
        // Push the atRest change physically before priming so suppliers see new layer state.
        layerStack.invalidateAll();
        primeAllSuppliers();
        suppressInputsUntilNextInvalidate = true;
    }

    public boolean isActive(CustomGamepad gamepad) {
        return layerStack.isActive(gamepad);
    }

    public void update() {
        layerStack.update();
    }

    /**
     * Refresh every supplier from the underlying gamepads — call once per loop. After a layer
     * switch this frame, suppliers are primed (no false edges) and inputs resume next frame.
     */
    public void invalidateAll() {
        // Update underlying gamepads first so wrappers don't stay one frame behind.
        layerStack.invalidateAll();

        if (suppressInputsUntilNextInvalidate) {
            suppressInputsUntilNextInvalidate = false;
            primeAllSuppliers();
            return;
        }

        forEachSupplier(EnhancedBooleanSupplier::invalidate, EnhancedDoubleSupplier::invalidate);
    }

    public void invalidateActive() {
        layerStack.invalidateActive();

        if (suppressInputsUntilNextInvalidate) {
            suppressInputsUntilNextInvalidate = false;
            primeAllSuppliers();
            return;
        }

        forEachSupplier(EnhancedBooleanSupplier::invalidate, EnhancedDoubleSupplier::invalidate);
    }

    private void primeAllSuppliers() {
        forEachSupplier(EnhancedBooleanSupplier::primeToCurrentState,
                EnhancedDoubleSupplier::primeToCurrentState);
    }

    private void forEachSupplier(java.util.function.Consumer<EnhancedBooleanSupplier> bool,
                                 java.util.function.Consumer<EnhancedDoubleSupplier> dbl) {
        bool.accept(a); bool.accept(b); bool.accept(x); bool.accept(y);
        bool.accept(dpadUp); bool.accept(dpadDown); bool.accept(dpadLeft); bool.accept(dpadRight);
        bool.accept(leftBumper); bool.accept(rightBumper);
        bool.accept(start); bool.accept(back); bool.accept(guide);
        bool.accept(leftStickButton); bool.accept(rightStickButton);
        bool.accept(touchpad);

        dbl.accept(leftStickX); dbl.accept(leftStickY);
        dbl.accept(rightStickX); dbl.accept(rightStickY);
        dbl.accept(leftTrigger); dbl.accept(rightTrigger);
        dbl.accept(touchpadFinger1X); dbl.accept(touchpadFinger1Y);
    }

    public EnhancedDoubleSupplier getLeftStickX()  { return leftStickX; }
    public EnhancedDoubleSupplier getLeftStickY()  { return leftStickY; }
    public EnhancedDoubleSupplier getRightStickX() { return rightStickX; }
    public EnhancedDoubleSupplier getRightStickY() { return rightStickY; }
    public EnhancedDoubleSupplier LT() { return leftTrigger; }
    public EnhancedDoubleSupplier RT() { return rightTrigger; }

    public EnhancedBooleanSupplier A() { return a; }
    public EnhancedBooleanSupplier B() { return b; }
    public EnhancedBooleanSupplier X() { return x; }
    public EnhancedBooleanSupplier Y() { return y; }

    public EnhancedBooleanSupplier DPAD_UP()    { return dpadUp; }
    public EnhancedBooleanSupplier DPAD_DOWN()  { return dpadDown; }
    public EnhancedBooleanSupplier DPAD_LEFT()  { return dpadLeft; }
    public EnhancedBooleanSupplier DPAD_RIGHT() { return dpadRight; }

    public EnhancedBooleanSupplier LB() { return leftBumper; }
    public EnhancedBooleanSupplier RB() { return rightBumper; }

    public EnhancedBooleanSupplier getStart() { return start; }
    public EnhancedBooleanSupplier getBack()  { return back; }
    public EnhancedBooleanSupplier getGuide() { return guide; }

    public EnhancedBooleanSupplier LSB() { return leftStickButton; }
    public EnhancedBooleanSupplier RSB() { return rightStickButton; }

    public EnhancedBooleanSupplier getTouchpad() { return touchpad; }
    public EnhancedDoubleSupplier  TX() { return touchpadFinger1X; }
    public EnhancedDoubleSupplier  TY() { return touchpadFinger1Y; }

    public EnhancedBooleanSupplier C() { return A(); }
    public EnhancedBooleanSupplier O() { return B(); }
    public EnhancedBooleanSupplier Q() { return X(); }
    public EnhancedBooleanSupplier T() { return Y(); }
    public EnhancedBooleanSupplier getOptions() { return getStart(); }

    @Override
    public String toString() {
        return "LayeredGamepad{currentLayer=" + getCurrentLayer() + "}";
    }
}
