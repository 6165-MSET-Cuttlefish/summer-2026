package org.firstinspires.ftc.teamcode.architecture.input;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import org.firstinspires.ftc.teamcode.architecture.input.suppliers.CustomGamepad;

/**
 * A keyed stack of gamepads, of which exactly one is "active" at a time. The active
 * gamepad is the only one whose physical inputs reach the OpMode; the rest are forced
 * to {@code atRest} so suppliers reading them see neutral values.
 *
 * <p>Layer keys can be any type — typically a small enum (e.g. {@code TELE/SORT/ENDGAME}).
 * Map iteration is insertion-order to make {@link #getAvailableLayers()} predictable.
 */
public class LayerStack<T> {

    private final Map<T, CustomGamepad> layers;
    private T currentLayer;

    public LayerStack(T initialLayer, Map<T, CustomGamepad> layers) {
        if (layers == null || layers.isEmpty()) {
            throw new IllegalArgumentException("layers cannot be null or empty");
        }
        if (initialLayer != null && !layers.containsKey(initialLayer)) {
            throw new IllegalArgumentException("initial layer " + initialLayer + " is not in the map");
        }
        // Defensive copy keeps callers from mutating us through a shared reference.
        this.layers = new LinkedHashMap<>(layers);
        this.currentLayer = initialLayer;
        update();
    }

    // ─── current layer ───────────────────────────────────────────────────────

    public T getLayer() { return currentLayer; }

    public void setLayer(T layer) {
        if (layer == null) throw new IllegalArgumentException("layer cannot be null");
        if (!layers.containsKey(layer)) {
            throw new IllegalArgumentException("layer " + layer + " is not in the map");
        }
        this.currentLayer = layer;
        update();
    }

    public CustomGamepad getGamepad() {
        return currentLayer == null ? null : layers.get(currentLayer);
    }

    public boolean isActive(CustomGamepad gamepad) {
        if (gamepad == null || currentLayer == null) return false;
        return gamepad.equals(layers.get(currentLayer));
    }

    // ─── layer management ────────────────────────────────────────────────────

    public Set<T> getAvailableLayers() { return layers.keySet(); }
    public boolean hasLayer(T layer) { return layers.containsKey(layer); }
    public int getLayerCount() { return layers.size(); }
    public boolean isEmpty() { return layers.isEmpty(); }

    public void putLayer(T layer, CustomGamepad gamepad) {
        if (layer == null) throw new IllegalArgumentException("layer cannot be null");
        if (gamepad == null) throw new IllegalArgumentException("gamepad cannot be null");
        layers.put(layer, gamepad);
        if (currentLayer == null) currentLayer = layer;
        update();
    }

    public CustomGamepad removeLayer(T layer) {
        if (layer == null) return null;
        CustomGamepad removed = layers.remove(layer);
        if (layer.equals(currentLayer)) {
            currentLayer = layers.isEmpty() ? null : layers.keySet().iterator().next();
            update();
        }
        return removed;
    }

    // ─── physical state propagation ──────────────────────────────────────────

    /** Push the active/atRest state into every underlying gamepad. */
    public void update() {
        for (Map.Entry<T, CustomGamepad> entry : layers.entrySet()) {
            CustomGamepad gp = entry.getValue();
            if (gp != null) gp.setAtRest(!entry.getKey().equals(currentLayer));
        }
    }

    /** Drop cached state on every underlying gamepad so the next read sees fresh hardware. */
    public void invalidateAll() {
        for (CustomGamepad gp : layers.values()) {
            if (gp != null) gp.invalidateAll();
        }
    }

    /** Drop cached state only on the currently active gamepad. */
    public void invalidateActive() {
        CustomGamepad active = getGamepad();
        if (active != null) {
            active.invalidateAll();
        }
    }

    @Override
    public String toString() {
        return "LayerStack{currentLayer=" + currentLayer + ", layerCount=" + layers.size() + "}";
    }
}
