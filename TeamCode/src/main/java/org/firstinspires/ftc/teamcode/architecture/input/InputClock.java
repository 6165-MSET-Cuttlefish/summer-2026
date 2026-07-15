package org.firstinspires.ftc.teamcode.architecture.input;

/**
 * Monotonic per-loop counter. {@code EnhancedOpMode} calls {@link #advance()} once at the top of
 * each init_loop/loop tick; edge suppliers key their per-loop refresh off it (see
 * {@link EdgeBooleanSupplier}) so they update exactly once per loop whether or not they're pumped
 * by a {@link LayeredGamepad}. Static so it survives Sloth hot-reload; a monotonic counter has no
 * stale-state hazard — suppliers created after a reload simply refresh on the next differing frame.
 */
public final class InputClock {
    private static long frame = 0;

    private InputClock() {}

    public static long current() { return frame; }

    public static void advance() { frame++; }
}
