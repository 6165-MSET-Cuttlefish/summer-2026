package org.firstinspires.ftc.teamcode.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Per-section timing for the main OpMode loop. Each named section keeps a fixed-size ring
 * buffer of recent samples (in ms) and exposes its rolling average.
 *
 * <p>Usage from {@link EnhancedOpMode}: call {@link #start()} at the top of the loop, then
 * {@link #mark(String)} after each phase. The first {@code mark} after {@code start} measures
 * the time since {@code start}; subsequent marks measure the time since the previous mark.
 * For ad-hoc timing of an arbitrary scope (e.g. a single module's read), use
 * {@link #beginScope(String)} + {@link #endScope()} which are independent of the mark cursor.
 *
 * <p>Profiler overhead is a handful of {@code System.nanoTime()} reads and one HashMap probe
 * per mark. Allocation only happens the first time a section name is seen.
 */
public final class LoopProfiler {

    private static final int WINDOW = 2;

    private final LinkedHashMap<String, Bucket> sections = new LinkedHashMap<>();
    private long markAnchorNs = 0;
    private long scopeAnchorNs = 0;

    /** Reset the mark cursor to "now". Call once at the top of each loop. */
    public void start() {
        markAnchorNs = System.nanoTime();
    }

    /** Record the time since the previous {@link #start()} or {@link #mark(String)}. */
    public void mark(String section) {
        long now = System.nanoTime();
        accumulate(section, (now - markAnchorNs) / 1_000_000.0);
        markAnchorNs = now;
    }

    /** Begin an independent scope; pair with {@link #endScope(String)}. Scopes do not nest. */
    public void beginScope(String unused) {
        scopeAnchorNs = System.nanoTime();
    }

    /** End the most recent {@link #beginScope(String)} and accumulate under {@code section}. */
    public void endScope(String section) {
        accumulate(section, (System.nanoTime() - scopeAnchorNs) / 1_000_000.0);
    }

    /**
     * Reentrant timer: capture {@code stamp()} before the work, then call
     * {@link #stop(String, long)} after. Multiple stamps may be in flight at once and may
     * nest freely.
     */
    public long stamp() {
        return System.nanoTime();
    }

    /** Accumulate the elapsed time since {@code stamp} under {@code section}. */
    public void stop(String section, long stamp) {
        accumulate(section, (System.nanoTime() - stamp) / 1_000_000.0);
    }

    /** Add a manually measured duration (in ms) to a section. */
    public void accumulate(String section, double ms) {
        Bucket b = sections.get(section);
        if (b == null) {
            b = new Bucket();
            sections.put(section, b);
        }
        b.add(ms);
    }

    /**
     * Snapshot of (section, avg ms) entries, sorted by descending avg. Allocates — call only
     * when rendering telemetry, not in the hot loop.
     */
    public List<Map.Entry<String, Double>> snapshotSortedDesc() {
        List<Map.Entry<String, Double>> out = new ArrayList<>(sections.size());
        for (Map.Entry<String, Bucket> e : sections.entrySet()) {
            out.add(new java.util.AbstractMap.SimpleEntry<>(e.getKey(), e.getValue().avg()));
        }
        Collections.sort(out, (a, b) -> Double.compare(b.getValue(), a.getValue()));
        return out;
    }

    /** Clear all accumulated samples. */
    public void reset() {
        sections.clear();
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
