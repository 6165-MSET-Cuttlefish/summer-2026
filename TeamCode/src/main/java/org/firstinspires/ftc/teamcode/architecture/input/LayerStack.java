package org.firstinspires.ftc.teamcode.architecture.input;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * Keyed stack of gamepads — exactly one is active at a time; the rest are forced to
 * {@code atRest} so their suppliers read neutral. Layer keys are typically a small enum.
 */
public class LayerStack<T> {

    private final Map<T, LayerGamepad> layers;
    private T currentLayer;
    private Runnable onLayerChange;

    public LayerStack(T initialLayer, Map<T, LayerGamepad> layers) {
        if (layers == null || layers.isEmpty()) {
            throw new IllegalArgumentException("layers cannot be null or empty");
        }
        if (initialLayer != null && !layers.containsKey(initialLayer)) {
            throw new IllegalArgumentException("initial layer " + initialLayer + " is not in the map");
        }
        this.layers = new LinkedHashMap<>(layers);
        this.currentLayer = initialLayer;
        update();
    }

    public T getLayer() { return currentLayer; }

    /**
     * Notified after any actual {@code currentLayer} change. {@link LayeredGamepad} subscribes so its
     * facade suppliers get primed+suppressed even when a caller changes layers on the stack directly
     * (bypassing {@code LayeredGamepad.setLayer}); otherwise a held button fires a spurious edge on
     * the new layer. Not fired by {@link #update()} (no layer change there).
     */
    void setOnLayerChange(Runnable onLayerChange) { this.onLayerChange = onLayerChange; }

    private void fireLayerChanged() {
        if (onLayerChange != null) onLayerChange.run();
    }

    public void setLayer(T layer) {
        if (layer == null) throw new IllegalArgumentException("layer cannot be null");
        if (!layers.containsKey(layer)) {
            throw new IllegalArgumentException("layer " + layer + " is not in the map");
        }
        boolean changed = !layer.equals(currentLayer);
        this.currentLayer = layer;
        update();
        if (changed) fireLayerChanged();
    }

    public LayerGamepad getGamepad() {
        return currentLayer == null ? null : layers.get(currentLayer);
    }

    public boolean isActive(LayerGamepad gamepad) {
        if (gamepad == null || currentLayer == null) return false;
        return gamepad.equals(layers.get(currentLayer));
    }

    public Set<T> getAvailableLayers() { return layers.keySet(); }
    public boolean hasLayer(T layer) { return layers.containsKey(layer); }
    public int getLayerCount() { return layers.size(); }
    public boolean isEmpty() { return layers.isEmpty(); }

    public void putLayer(T layer, LayerGamepad gamepad) {
        if (layer == null) throw new IllegalArgumentException("layer cannot be null");
        if (gamepad == null) throw new IllegalArgumentException("gamepad cannot be null");
        layers.put(layer, gamepad);
        boolean changed = false;
        if (currentLayer == null) {
            currentLayer = layer;
            changed = true;
        }
        update();
        if (changed) fireLayerChanged();
    }

    public LayerGamepad removeLayer(T layer) {
        if (layer == null) return null;
        LayerGamepad removed = layers.remove(layer);
        if (layer.equals(currentLayer)) {
            currentLayer = layers.isEmpty() ? null : layers.keySet().iterator().next();
            update();
            fireLayerChanged();
        }
        return removed;
    }

    /** Push the active/atRest state into every underlying gamepad. */
    public void update() {
        for (Map.Entry<T, LayerGamepad> entry : layers.entrySet()) {
            LayerGamepad gp = entry.getValue();
            if (gp != null) gp.setAtRest(!entry.getKey().equals(currentLayer));
        }
    }

    public void invalidateAll() {
        for (LayerGamepad gp : layers.values()) {
            if (gp != null) gp.invalidateAll();
        }
    }

    public void invalidateActive() {
        LayerGamepad active = getGamepad();
        if (active != null) {
            active.invalidateAll();
        }
    }

    @Override
    public String toString() {
        return "LayerStack{currentLayer=" + currentLayer + ", layerCount=" + layers.size() + "}";
    }
}
