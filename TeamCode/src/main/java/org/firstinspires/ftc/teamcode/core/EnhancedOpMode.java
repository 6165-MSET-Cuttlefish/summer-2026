package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedCRServo;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;

import static org.firstinspires.ftc.teamcode.core.Robot.telemetryToggles;
import static org.firstinspires.ftc.teamcode.core.HtmlFormatter.*;
import static org.firstinspires.ftc.teamcode.core.OptimizationToggles.*;

/**
 * Base OpMode with automatic module management, voltage compensation,
 * Robot integration, and FTC Dashboard support.
 *
 * Subclasses implement:
 *   - initialize()      — called once after Robot is created
 *   - initializeLoop()  — called each init_loop cycle
 *   - primaryLoop()     — called each main loop cycle
 *   - onEnd()           — called when the OpMode stops
 *   - telemetry()       — add custom telemetry each loop
 */
public abstract class EnhancedOpMode extends OpMode {
    protected Robot robot;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime gameTimer = new ElapsedTime();
    private final ElapsedTime readWriteTimer = new ElapsedTime();
    private final double[] loopTimes = new double[20];
    private final LoopProfiler profiler = new LoopProfiler();
    private List<LynxModule> lynxHubs;
    private int loopIndex = 0;
    private int voltageLoopCounter = 0;
    private int dashboardLoopCounter = 0;
    private int dashboardPoseHistoryLoopCounter = 0;
    private int telemetryLoopCounter = 0;

    private final List<Module> modules = new ArrayList<>();
    /** Cached telemetry-ordered modules built once after init when {@code optimizeTelemetryModuleSortOnce}
     *  is on; null otherwise. */
    private List<Module> sortedTelemetryModules;
    private int currentReadLoopCounter = 0;
    private double cachedTotalCurrent = 0.0;
    private VoltageSensor voltageSensor;
    private double voltage = 12.0;
    private boolean running = false;
    private boolean stopRequested = false;
    private boolean isInit = false;

    private BrailleRenderer field;
    /** Cached braille-field HTML; only re-rendered when the robot pose changes meaningfully or
     *  every {@link #fieldRenderInterval} loops, whichever comes first. */
    private String cachedFieldHtml = "";
    private double lastFieldRenderX = Double.NaN;
    private double lastFieldRenderY = Double.NaN;
    private double lastFieldRenderHeading = Double.NaN;
    private int loopsSinceFieldRender = Integer.MAX_VALUE;
    /** Force a render at most every N loops even if the pose hasn't moved (keeps the DS warm). */
    protected int fieldRenderInterval = 10;

    /** Minimum loop time in milliseconds. Set to stabilize control loop timing. */
    protected int minLoopMs = 0;
    /**
     * How many loops between voltage sensor reads. The battery voltage changes extremely slowly;
     * reading it every loop wastes ~2-4 ms per cycle on an uncached round-trip.
     * 50 loops at ~5 ms each ≈ once per 250 ms, which is more than sufficient.
     */
    protected int voltageReadLoopInterval = 50;
    /** Enable automatic voltage compensation for motors and CR servos. */
    protected boolean voltageCompensationEnabled = true;

    // ── Subclass lifecycle hooks ──────────────────────────────────────

    /** Called once after Robot is created and modules are initialized. */
    protected void initialize() {}

    /** Called repeatedly during the init phase. */
    protected void initializeLoop() {}

    /** Called repeatedly during the main loop. */
    protected void primaryLoop() {}

    /** Called once when the OpMode transitions from init to running. */
    protected void onStart() {}

    /** Called when the OpMode stops. */
    protected void onEnd() {}

    /** Override to add custom telemetry each loop. */
    protected void telemetry() {}

    /** Override to return true if hardware writes should occur during init_loop. */
    protected boolean shouldWriteDuringInit() {
        return false;
    }

    /** Override to return false to skip module reads during init_loop. */
    protected boolean shouldReadDuringInit() {
        return true;
    }

    /** Override to preserve robot position across OpMode transitions. */
    protected boolean shouldPreservePosition() {
        return true;
    }

    /** Override to control SRS Hub initialization. */
    protected boolean shouldInitializeSRSHub() {
        return true;
    }

    /**
     * Subclasses produce the game-specific {@link Robot} instance here. Called once during
     * {@link #init()} before any module discovery or {@link #initialize()}.
     */
    protected abstract Robot createRobot() throws InterruptedException;

    /**
     * Game hook fired at the top of every init_loop and loop cycle, after voltage publish but
     * before module reads. Default: no-op. Game-specific subclasses override to refresh shared
     * derived state (e.g. shot-on-the-move pose math).
     */
    protected void onLoopStart() {}

    // ── OpMode lifecycle (final) ─────────────────────────────────────

    @Override
    public final void init() {
        // Sloth hot-reloads and back-to-back opmode runs leave stale framework state behind:
        // the State→Module map and the Actions active list both live in static fields. Rebuild
        // both before this opmode's modules instantiate.
        StateRegistry.clearModuleBindings();
        Actions.cancelAll();

        configureBulkCaching();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Robot is constructed by a game-specific subclass via createRobot(); EnhancedOpMode
        // doesn't know about specific mechanisms, only the framework Robot type.
        try {
            robot = createRobot();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        telemetry = robot.telemetry;
        robot.packet = new TelemetryPacket(false);

        // Modules are picked up via reflection on the OpMode + Robot field graph.
        autoDiscoverModules();
        initModules();
        if (optimizeTelemetryModuleSortOnce) buildSortedTelemetryModules();
        initialize();

        loopTimer.reset();
        gameTimer.reset();
        isInit = true;

        field = new BrailleRenderer(73, 74);
        field.drawFieldLayout();
        field.snapshot();
    }

    @Override
    public final void init_loop() {
        if (!isInit) return;
        if (stopRequested) {
            requestOpModeStop();
            return;
        }

        profiler.start();

        clearBulkCaches();
        profiler.mark("clearBulkCaches");

        if (voltageCompensationEnabled) {
            updateVoltageThrottled();
            EnhancedMotor.updateVoltage(voltage);
            EnhancedCRServo.updateVoltage(voltage);
        }
        profiler.mark("voltage");

        onLoopStart();
        profiler.mark("onLoopStart");

        if (shouldReadDuringInit())
            readModules();
        profiler.mark("readModules");

        Pose currentPose = robot.follower.getPose();
        addStatusTelemetry(currentPose);
        profiler.mark("statusTelemetry");

        robot.follower.update();
        profiler.mark("follower.update");

        initializeLoop();
        profiler.mark("initializeLoop");

        if (shouldWriteDuringInit() && gameTimer.milliseconds() > 500) {
            writeModules();
        }
        profiler.mark("writeModules");

        updateTelemetry();
        profiler.mark("updateTelemetry");

        updateDashboard(currentPose);
        profiler.mark("updateDashboard");

        loopTimer.reset();
    }

    @Override
    public final void start() {
        running = true;
        gameTimer.reset();
        clearBulkCaches();
        Actions.reset();
        scheduleDefaultActions();
        onStart();
        loopTimer.reset();
    }

    @Override
    public final void loop() {
        if (stopRequested) {
            running = false;
            requestOpModeStop();
            return;
        }

        profiler.start();

        clearBulkCaches();
        profiler.mark("clearBulkCaches");

        if (voltageCompensationEnabled) {
            updateVoltageThrottled();
            EnhancedMotor.updateVoltage(voltage);
            EnhancedCRServo.updateVoltage(voltage);
        }
        profiler.mark("voltage");

        onLoopStart();
        profiler.mark("onLoopStart");

        readModules();
        profiler.mark("readModules");

        Pose currentPose = robot.follower.getPose();
        addStatusTelemetry(currentPose);
        profiler.mark("statusTelemetry");

        robot.updateWriteToggles();
        robot.follower.update();
        profiler.mark("follower.update");

        primaryLoop();
        profiler.mark("primaryLoop");

        writeModules();
        profiler.mark("writeModules");

        updateTelemetry();
        profiler.mark("updateTelemetry");

        updateDashboard(currentPose);
        profiler.mark("updateDashboard");

        long remaining = minLoopMs - (long) loopTimer.milliseconds();
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        recordLoopTime();
        loopTimer.reset();
    }

    @Override
    public final void stop() {
        running = false;
        Actions.shutdown();
        // Reset any pending auto sequence so the next opmode starts clean.
        if (robot != null && robot.pathActionScheduler != null) {
            robot.pathActionScheduler.cancelAll();
        }
        onEnd();
    }

    // ── Module management ────────────────────────────────────────────

    /**
     * Register modules manually. Modules are also auto-discovered from fields.
     */
    protected void register(Module... mods) {
        for (Module m : mods) {
            if (!modules.contains(m)) {
                modules.add(m);
            }
        }
    }

    private void autoDiscoverModules() {
        Set<Object> visited = new HashSet<>();
        try {
            discover(this, getClass(), visited);
        } catch (IllegalAccessException e) {
            throw new RuntimeException("Module auto-discovery failed", e);
        }
    }

    private void discover(Object obj, Class<?> clazz, Set<Object> visited)
            throws IllegalAccessException {
        if (obj == null || !visited.add(obj))
            return;

        while (clazz != EnhancedOpMode.class && clazz != Object.class && clazz != null) {
            for (Field f : clazz.getDeclaredFields()) {
                f.setAccessible(true);
                Object val = f.get(obj);
                if (val == null)
                    continue;

                if (val instanceof Module) {
                    register((Module) val);
                } else if (shouldRecurse(val.getClass())) {
                    discover(val, val.getClass(), visited);
                }
            }
            clazz = clazz.getSuperclass();
        }
    }

    private boolean shouldRecurse(Class<?> c) {
        String n = c.getName();
        return !n.startsWith("java.") && !n.startsWith("android.") && !n.startsWith("com.qualcomm.")
                && !n.startsWith("kotlin.");
    }

    private void initModules() {
        for (Module m : modules) {
            m.setTelemetry(telemetry);
            m.init();
        }
    }

    /** Modules sorted by {@link Module#telemetryOrder()} (stable). Used only for telemetry. */
    private List<Module> telemetryOrderedModules() {
        if (optimizeTelemetryModuleSortOnce) {
            if (sortedTelemetryModules == null) buildSortedTelemetryModules();
            return sortedTelemetryModules;
        }
        List<Module> ordered = new ArrayList<>(modules);
        Collections.sort(ordered, (a, b) -> Integer.compare(a.telemetryOrder(), b.telemetryOrder()));
        return ordered;
    }

    private void buildSortedTelemetryModules() {
        List<Module> ordered = new ArrayList<>(modules);
        Collections.sort(ordered, (a, b) -> Integer.compare(a.telemetryOrder(), b.telemetryOrder()));
        sortedTelemetryModules = ordered;
    }

    private void readModules() {
        for (int i = 0; i < modules.size(); i++) {
            Module m = modules.get(i);
            profiler.beginScope("read");
            m.read();
            profiler.endScope(optimizeProfilerScopeKeys ? m.getReadScopeKey() : "read." + m.getName());
        }
    }

    protected void writeModules() {
        for (int i = 0; i < modules.size(); i++) {
            Module m = modules.get(i);
            if (m.isWriteEnabled()) {
                profiler.beginScope("write");
                m.write();
                profiler.endScope(optimizeProfilerScopeKeys ? m.getWriteScopeKey() : "write." + m.getName());
            }
        }
    }

    private void scheduleDefaultActions() {
        for (Module m : modules) {
            Action def = m.getDefaultAction();
            if (def != null && !Actions.isModuleActive(m)) {
                def.run();
            }
        }
    }

    // ── Hardware utilities ───────────────────────────────────────────

    private void configureBulkCaching() {
        lynxHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : lynxHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void clearBulkCaches() {
        for (LynxModule hub : lynxHubs) {
            hub.clearBulkCache();
        }
    }

    private void updateVoltageThrottled() {
        if (voltageLoopCounter == 0) {
            voltage = voltageSensor.getVoltage();
        }
        voltageLoopCounter = (voltageLoopCounter + 1) % voltageReadLoopInterval;
    }

    private void recordLoopTime() {
        loopTimes[loopIndex] = loopTimer.milliseconds();
        loopIndex = (loopIndex + 1) % loopTimes.length;
    }

    // ── Telemetry ────────────────────────────────────────────────────

    private void updateTelemetry() {
        if (optimizeTelemetryCadence) {
            int every = Math.max(1, optimizeTelemetryEveryNLoops);
            if ((telemetryLoopCounter++ % every) != 0) {
                telemetry.update();
                return;
            }
        }

        if (telemetry instanceof EnhancedTelemetry) {
            EnhancedTelemetry et = (EnhancedTelemetry) telemetry;

            et.addDashboardData("Game Time", "%.1fs", gameTimer.seconds());
            et.addData("Loop Time", "%.1fms (avg %.1fms)", loopTimer.milliseconds(), avgLoopMs());

            if (!modules.isEmpty()) {
                et.addSeparator();
                et.addGroupHeader("MODULES", HtmlFormatter.COLOR_MODULE);
                for (Module m : telemetryOrderedModules()) {
                    m.telemetry();
                }
            }

            if (telemetryToggles.loopProfile) {
                et.addSeparator();
                et.addGroupHeader("LOOP PROFILE (avg ms)", HtmlFormatter.COLOR_BLUE);
                for (Map.Entry<String, Double> entry : profiler.snapshotSortedDesc()) {
                    et.addDashboardData(entry.getKey(), "%.2fms", entry.getValue());
                }
            }

            Pose fieldPose = robot.follower.getPose();
            double fx = fieldPose.getX();
            double fy = fieldPose.getY();
            double fh = robot.follower.getHeading();
            boolean poseChanged =
                    Math.abs(fx - lastFieldRenderX) > 0.5
                    || Math.abs(fy - lastFieldRenderY) > 0.5
                    || Math.abs(fh - lastFieldRenderHeading) > Math.toRadians(2);
            if (poseChanged || loopsSinceFieldRender >= fieldRenderInterval) {
                field.restore();
                field.drawRobot(fx, fy, fh,
                        Context.allianceColor.equals(AllianceColor.RED) ? COLOR_RED : COLOR_BLUE);
                cachedFieldHtml = HtmlFormatter.htmlSize(FONT_MINI_FIELD, field.renderHtml());
                lastFieldRenderX = fx;
                lastFieldRenderY = fy;
                lastFieldRenderHeading = fh;
                loopsSinceFieldRender = 0;
            } else {
                loopsSinceFieldRender++;
            }
            robot.telemetry.addDSLine(cachedFieldHtml);
        } else {
            telemetry.addData("Game Time", "%.1fs", gameTimer.seconds());
            telemetry.addData("Loop Time", "%.1fms (avg %.1fms)", loopTimer.milliseconds(), avgLoopMs());

            for (Module m : telemetryOrderedModules()) {
                m.telemetry();
            }
        }

        telemetry();
        telemetry.update();
    }

    protected void updateDashboard(Pose currentPose) {
        if (!optimizeDashboardRendering) {
            Canvas overlay = robot.packet.fieldOverlay();
            overlay.setAlpha(0.4);
            overlay.drawImage("/images/fieldcoordinates-pedro.png", 0, 0, 144, 144);
            overlay.setAlpha(1);
            overlay.drawGrid(0, 0, 144, 144, 7, 7);

            FieldVisualization.drawRobot(currentPose);
            FieldVisualization.drawPoseHistory(robot.follower.getPoseHistory());

            FtcDashboard.getInstance().sendTelemetryPacket(robot.packet);
            robot.packet = new TelemetryPacket(false);
            return;
        }

        int every = Math.max(1, optimizeDashboardEveryNLoops);
        if ((dashboardLoopCounter++ % every) != 0) {
            robot.packet = new TelemetryPacket(false);
            return;
        }

        Canvas overlay = robot.packet.fieldOverlay();

        if (!optimizeDashboardSkipFieldImage) {
            overlay.setAlpha(0.4);
            overlay.drawImage("/images/fieldcoordinates-pedro.png", 0, 0, 144, 144);
            overlay.setAlpha(1);
        }

        if (!optimizeDashboardSkipGrid) {
            overlay.drawGrid(0, 0, 144, 144, 7, 7);
        }

        FieldVisualization.drawRobot(currentPose);

        if (!optimizeDashboardSkipPoseHistory) {
            int poseEvery = Math.max(1, optimizeDashboardPoseHistoryEveryNLoops);
            if ((dashboardPoseHistoryLoopCounter++ % poseEvery) == 0) {
                FieldVisualization.drawPoseHistory(robot.follower.getPoseHistory());
            }
        }

        FtcDashboard.getInstance().sendTelemetryPacket(robot.packet);
        robot.packet = new TelemetryPacket(false);
    }

    private void addStatusTelemetry(Pose currentPose) {
        robot.telemetry.addGroupHeader("ROBOT STATUS");
        addAllianceTelemetry();
        robot.telemetry.addData("Robot Position", "X: %.1f, Y: %.1f, Heading: %.1f°",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        addVoltageCurrentTelemetry();
    }

    private void addAllianceTelemetry() {
        String colorHex = Context.allianceColor == AllianceColor.RED
                ? HtmlFormatter.COLOR_RED
                : HtmlFormatter.COLOR_BLUE;
        String htmlValue = HtmlFormatter.htmlColor(colorHex,
                HtmlFormatter.htmlBold(String.valueOf(Context.allianceColor)));
        robot.telemetry.addDSRawHtml("Alliance", htmlValue);
        robot.telemetry.addDashboardData("Alliance Color", Context.allianceColor);
    }

    private void addVoltageCurrentTelemetry() {
        if (telemetryToggles.voltage) {
            robot.telemetry.addDashboardData("Voltage", "%.2fV", voltage);
        }
        if (telemetryToggles.current) {
            // Sum across all Lynx hubs — game-agnostic, no need to reach into a specific module.
            robot.telemetry.addDashboardData("Current", "%.2fA", getTotalCurrent());
        }
    }

    // ── Public accessors ─────────────────────────────────────────────

    /** Request a graceful stop of the OpMode. */
    public final void requestStop() {
        stopRequested = true;
    }

    /** Check if the OpMode is currently in the main loop phase. */
    public final boolean isRunning() {
        return running;
    }

    /** Get the current battery voltage. */
    public final double getVoltage() {
        return voltage;
    }

    /** Get the timer tracking total game time. */
    public final ElapsedTime getGameTimer() {
        return gameTimer;
    }

    /** Get the average loop time in milliseconds. */
    public final double avgLoopMs() {
        double sum = 0;
        int count = 0;
        for (double t : loopTimes) {
            if (t > 0) {
                sum += t;
                count++;
            }
        }
        return count > 0 ? sum / count : 0;
    }

    /** Get the total current draw from all Lynx hubs in amps. */
    public final double getTotalCurrent() {
        if (!optimizeCurrentReadCadence) {
            double total = 0;
            for (LynxModule hub : lynxHubs) {
                total += hub.getCurrent(CurrentUnit.AMPS);
            }
            return total;
        }
        int every = Math.max(1, optimizeCurrentReadEveryNLoops);
        if (currentReadLoopCounter == 0) {
            double total = 0;
            for (LynxModule hub : lynxHubs) {
                total += hub.getCurrent(CurrentUnit.AMPS);
            }
            cachedTotalCurrent = total;
        }
        currentReadLoopCounter = (currentReadLoopCounter + 1) % every;
        return cachedTotalCurrent;
    }

    /** Get an unmodifiable list of all registered modules. */
    public final List<Module> getModules() {
        return Collections.unmodifiableList(modules);
    }

    /**
     * Per-loop section profiler. Subclasses may add their own scopes (e.g. wrapping a
     * specific block of {@link #primaryLoop()}) — the breakdown is rendered in dashboard
     * telemetry when {@code Robot.telemetryToggles.loopProfile} is enabled.
     */
    public final LoopProfiler getProfiler() {
        return profiler;
    }
}
