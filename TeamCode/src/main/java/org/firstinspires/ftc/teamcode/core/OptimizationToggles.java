package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OptimizationToggles {
    private OptimizationToggles() {}

    public static boolean optimizeDashboardRendering = true;
    public static int optimizeDashboardEveryNLoops = 2;
    public static boolean optimizeDashboardSkipFieldImage = true;
    public static boolean optimizeDashboardSkipGrid = true;
    public static boolean optimizeDashboardSkipPoseHistory = true;
    public static int optimizeDashboardPoseHistoryEveryNLoops = 3;

    public static boolean optimizeTelemetryCadence = false;
    public static int optimizeTelemetryEveryNLoops = 1;

    public static boolean optimizeBuildAutonomousSequenceOnceInInit = false;
    public static boolean optimizeRenderVisualizationCadence = true;
    public static int optimizeRenderVisualizationEveryNLoops = 2;

    public static boolean optimizeCurrentLimiterComputation = false;
    public static int optimizeCurrentLimiterEveryNLoops = 2;
    public static boolean optimizeCurrentLimiterTelemetry = false;

    public static boolean optimizeInputInvalidation = true;
    public static boolean optimizeControlsTelemetryCadence = false;
    public static int optimizeControlsTelemetryEveryNLoops = 1;
    public static boolean optimizeRumbleCooldown = true;
    public static long optimizeRumbleCooldownMs = 250;

    public static boolean optimizeSensorCadence = false;
    public static int optimizeDistanceSensorEveryNLoops = 2;
    public static int optimizeColorSensorEveryNLoops = 2;

    public static boolean optimizeLimelightDrawCadence = true;
    public static int optimizeLimelightDrawEveryNLoops = 2;
    public static boolean optimizeSuppressLimelightPoseLogs = true;
    public static boolean optimizeObeliskLogSpam = true;

    public static boolean optimizeObeliskLoopDelay = false;
    public static long optimizeObeliskLoopDelayMs = 20;

    // ── New toggles added in the loop-time optimization pass ─────────────────
    // Each gates a specific change so it can be reverted independently from FtcDashboard.

    /** Default Robot.telemetryToggles.loopProfile to false at class-load (still toggleable at runtime). */
    public static boolean optimizeDisableLoopProfileTelemetryByDefault = false;

    /** Apply caching tolerances to Magazine + Turret servos so identical positions skip the I2C write. */
    public static boolean optimizeServoCachingTolerances = true;

    /** Drivetrain.getMotorPowers() returns a cached double[4] instead of allocating each call. */
    public static boolean optimizeMotorPowersCaching = true;

    /** Module.get(stateClass) uses an O(1) Map lookup instead of an O(n) list scan. */
    public static boolean optimizeStateLookupMap = true;

    /** Throttle limelight.updateRobotOrientation to every N loops (each is a USB serial transaction). */
    public static boolean optimizeLimelightHeadingUpdateCadence = true;
    public static int optimizeLimelightHeadingUpdateEveryNLoops = 3;

    /** Throttle Lynx hub current reads (each is an I2C round-trip per hub). */
    public static boolean optimizeCurrentReadCadence = false;
    public static int optimizeCurrentReadEveryNLoops = 3;

    /** Skip String.format and ArrayList allocation in EnhancedTelemetry when both backends disabled. */
    public static boolean optimizeTelemetryLazyFormat = false;

    /** Sort the telemetry-ordered module list once at init instead of every loop. */
    public static boolean optimizeTelemetryModuleSortOnce = false;

    /** Use precomputed "read.<name>"/"write.<name>" profile keys instead of String concat each loop. */
    public static boolean optimizeProfilerScopeKeys = true;
}
