package org.firstinspires.ftc.teamcode.architecture.telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class LoopProfiler {

    private static final int WINDOW = 20;

    private final LinkedHashMap<String, Bucket> sections = new LinkedHashMap<>();
    private long markAnchorNs;

    public boolean enabled = true;

    public void start() {
        if (!enabled) return;
        markAnchorNs = System.nanoTime();
    }

    public void mark(String section) {
        if (!enabled) return;
        long now = System.nanoTime();
        accumulate(section, (now - markAnchorNs) / 1_000_000.0);
        markAnchorNs = now;
    }

    /** Tokens are single-use — leaving twice with the same token double-counts the elapsed time. */
    public long enterSection() {
        return enabled ? System.nanoTime() : 0L;
    }

    public void leaveSection(String section, long startTokenNs) {
        if (!enabled || startTokenNs == 0L) return;
        accumulate(section, (System.nanoTime() - startTokenNs) / 1_000_000.0);
    }

    public List<Map.Entry<String, Double>> snapshotSortedDesc() {
        List<Map.Entry<String, Double>> out = new ArrayList<>(sections.size());
        for (Map.Entry<String, Bucket> e : sections.entrySet()) {
            out.add(new java.util.AbstractMap.SimpleEntry<>(e.getKey(), e.getValue().avg()));
        }
        Collections.sort(out, (a, b) -> Double.compare(b.getValue(), a.getValue()));
        return out;
    }

    public void reset() {
        sections.clear();
        markAnchorNs = 0L;
    }

    private void accumulate(String section, double ms) {
        Bucket b = sections.get(section);
        if (b == null) {
            b = new Bucket();
            sections.put(section, b);
        }
        b.add(ms);
    }

    private static final class Bucket {
        final double[] window = new double[WINDOW];
        int idx = 0;
        int filled = 0;
        double sum = 0;

        void add(double v) {
            sum -= window[idx];
            window[idx] = v;
            sum += v;
            idx = (idx + 1) % WINDOW;
            if (filled < WINDOW) filled++;
        }

        double avg() {
            return filled == 0 ? 0 : sum / filled;
        }
    }
}
