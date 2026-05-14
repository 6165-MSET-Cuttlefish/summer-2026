package org.firstinspires.ftc.teamcode.architecture.diagnostics;

import com.acmerobotics.dashboard.config.Config;

/**
 * Live-tunable perf knobs surfaced on FTC Dashboard. Every field here is consumed somewhere
 * in the framework — verify before adding a new one. Defaults favor visibility (data shows
 * up) over throughput; tighten for competition.
 */
@Config
public final class OptimizationToggles {
    private OptimizationToggles() {}

    // Dashboard packet send cadence. 1 = every loop. Bump for cheaper loops at the cost of
    // jerkier dashboard rendering.
    public static int dashboardEveryNLoops = 2;

    public static boolean dashboardSkipFieldImage = true;
    public static boolean dashboardSkipGrid = true;
    public static boolean dashboardSkipPoseHistory = true;

    // When pose history is drawn, throttle further (it's the heaviest single draw).
    public static int dashboardPoseHistoryEveryNLoops = 3;

    // Driver Station telemetry transmit cadence. 1 = every loop.
    public static int telemetryEveryNLoops = 1;

    // Skip the String.format + Item allocation on telemetry calls when both backends are off.
    public static boolean telemetryLazyFormat = true;

    // Sort modules into telemetry order once at init. Safe iff Module.telemetryOrder() values
    // don't change at runtime — the default is constant 0 and overrides are typically static.
    public static boolean telemetrySortModulesOnce = true;

    // Master enable for LoopProfiler. False = every profiler call short-circuits, no
    // System.nanoTime reads, no map probes.
    public static boolean profilerEnabled = true;

    // Default value of Robot.telemetryToggles.loopProfile at class load. The profile dump is
    // verbose; opt in from FtcDashboard when actively tuning.
    public static boolean loopProfileTelemetryByDefault = false;

    // Throttle Lynx-hub current reads (one I2C round-trip per hub). 1 = every loop.
    public static int currentReadEveryNLoops = 1;
}
