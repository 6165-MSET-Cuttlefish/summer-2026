package org.firstinspires.ftc.teamcode.architecture;

import com.acmerobotics.dashboard.config.Config;

/** Live-tunable perf knobs on FTC Dashboard. Defaults favor visibility; tighten for competition. */
@Config
public final class OptimizationToggles {
    private OptimizationToggles() {}

    public static int dashboardEveryNLoops = 2;

    public static boolean dashboardSkipFieldImage = true;
    public static boolean dashboardSkipGrid = true;
    public static boolean dashboardSkipPoseHistory = true;

    public static int dashboardPoseHistoryEveryNLoops = 3;

    public static int telemetryEveryNLoops = 1;

    /** Skip format + Item allocation on telemetry calls when both backends are off. */
    public static boolean telemetryLazyFormat = true;

    /** Sort modules once at init. Only safe when Module.telemetryOrder() values don't change at runtime. */
    public static boolean telemetrySortModulesOnce = true;

    public static boolean profilerEnabled = true;

    /** loopProfile default, read at class load — not live like the rest of this class. */
    public static boolean loopProfileTelemetryByDefault = true;

    /** Lynx current-read cadence; one I2C round-trip per hub when fired. */
    public static int currentReadEveryNLoops = 1;

    /** Drivetrain current-limiter: compute the floodgate multiplier every N loops instead of every setTargets. */
    public static boolean optimizeCurrentLimiterComputation = false;
    public static int optimizeCurrentLimiterEveryNLoops = 2;
    public static boolean optimizeCurrentLimiterTelemetry = false;

    /** Drivetrain.getMotorPowers() returns a cached double[4] instead of allocating each call. */
    public static boolean optimizeMotorPowersCaching = true;
}
