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
 * Base OpMode with auto-discovered modules, voltage compensation, dual telemetry, and dashboard
 * field rendering. Subclasses provide a Robot via {@link #createRobot()} and override
 * {@link #initialize()} / {@link #initializeLoop()} / {@link #primaryLoop()} as needed.
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
    private List<Module> sortedTelemetryModules;
    private int currentReadLoopCounter = 0;
    private double cachedTotalCurrent = 0.0;
    private VoltageSensor voltageSensor;
    private double voltage = 12.0;
    private boolean running = false;
    private boolean stopRequested = false;
    private boolean isInit = false;

    private BrailleRenderer field;
    private String cachedFieldHtml = "";
    private double lastFieldRenderX = Double.NaN;
    private double lastFieldRenderY = Double.NaN;
    private double lastFieldRenderHeading = Double.NaN;
    private int loopsSinceFieldRender = Integer.MAX_VALUE;
    /** Re-render the DS field map at most every N loops even when the pose is still. */
    protected int fieldRenderInterval = 10;

    /** Hold the loop to at least this many ms (0 = no holding). Stabilizes control loop timing. */
    protected int minLoopMs = 0;
    /**
     * Loops between battery-voltage reads. The bus call costs a few ms; voltage moves slowly,
     * so 50 loops (~250 ms at 5 ms/loop) is plenty.
     */
    protected int voltageReadLoopInterval = 50;
    protected boolean voltageCompensationEnabled = true;

    protected void initialize() {}
    protected void initializeLoop() {}
    protected void primaryLoop() {}
    protected void onStart() {}
    protected void onEnd() {}
    protected void telemetry() {}

    protected boolean shouldWriteDuringInit() { return false; }
    protected boolean shouldReadDuringInit() { return true; }
    protected boolean shouldPreservePosition() { return true; }
    protected boolean shouldInitializeSRSHub() { return true; }

    /** Game subclass produces its typed {@link Robot} here. Called once before module discovery. */
    protected abstract Robot createRobot() throws InterruptedException;

    /** Called at the top of every init_loop and loop, before module reads. Default: no-op. */
    protected void onLoopStart() {}

    @Override
    public final void init() {
        // Sloth hot-reloads and back-to-back opmode runs leave stale framework state behind:
        // the State→Module map and the Actions active list both live in static fields. Rebuild
        // both before this opmode's modules instantiate.
        StateRegistry.clearModuleBindings();
        Actions.cancelAll();

        configureBulkCaching();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        try {
            robot = createRobot();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        telemetry = robot.telemetry;
        robot.packet = new TelemetryPacket(false);

        autoDiscoverModules();
        initModules();
        if (telemetrySortModulesOnce) buildSortedTelemetryModules();
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

        profiler.enabled = profilerEnabled;
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

        if (shouldReadDuringInit()) readModules();
        profiler.mark("readModules");

        Pose currentPose = robot.follower.getPose();
        addStatusTelemetry(currentPose);
        profiler.mark("statusTelemetry");

        robot.follower.update();
        profiler.mark("follower.update");

        initializeLoop();
        profiler.mark("initializeLoop");

        // Wait 500 ms after init_loop starts before writing — gives the SDK a chance to settle
        // its hardware mode/direction calls so the very first writes don't fight them.
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

        profiler.enabled = profilerEnabled;
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
        if (robot != null && robot.pathActionScheduler != null) {
            robot.pathActionScheduler.cancelAll();
        }
        onEnd();
    }

    /** Manually register modules; reflection auto-discovery picks up the rest. */
    protected void register(Module... mods) {
        for (Module m : mods) {
            if (!modules.contains(m)) modules.add(m);
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
        if (obj == null || !visited.add(obj)) return;

        while (clazz != EnhancedOpMode.class && clazz != Object.class && clazz != null) {
            for (Field f : clazz.getDeclaredFields()) {
                f.setAccessible(true);
                Object val = f.get(obj);
                if (val == null) continue;

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
        return !n.startsWith("java.") && !n.startsWith("android.")
                && !n.startsWith("com.qualcomm.") && !n.startsWith("kotlin.");
    }

    private void initModules() {
        for (Module m : modules) {
            m.setTelemetry(telemetry);
            m.init();
        }
    }

    private List<Module> telemetryOrderedModules() {
        if (telemetrySortModulesOnce) {
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
            long t = profiler.startScope();
            m.read();
            profiler.endScope(m.getReadScopeKey(), t);
        }
    }

    protected void writeModules() {
        for (int i = 0; i < modules.size(); i++) {
            Module m = modules.get(i);
            if (m.isWriteEnabled()) {
                long t = profiler.startScope();
                m.write();
                profiler.endScope(m.getWriteScopeKey(), t);
            }
        }
    }

    private void scheduleDefaultActions() {
        for (Module m : modules) {
            Action def = m.getDefaultAction();
            if (def != null && !Actions.isModuleActive(m)) def.run();
        }
    }

    private void configureBulkCaching() {
        lynxHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : lynxHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void clearBulkCaches() {
        for (LynxModule hub : lynxHubs) hub.clearBulkCache();
    }

    private void updateVoltageThrottled() {
        if (voltageLoopCounter == 0) voltage = voltageSensor.getVoltage();
        voltageLoopCounter = (voltageLoopCounter + 1) % voltageReadLoopInterval;
    }

    private void recordLoopTime() {
        loopTimes[loopIndex] = loopTimer.milliseconds();
        loopIndex = (loopIndex + 1) % loopTimes.length;
    }

    private void updateTelemetry() {
        int every = Math.max(1, telemetryEveryNLoops);
        if (every > 1 && (telemetryLoopCounter++ % every) != 0) {
            telemetry.update();
            return;
        }

        if (telemetry instanceof EnhancedTelemetry) {
            EnhancedTelemetry et = (EnhancedTelemetry) telemetry;

            et.addDashboardData("Game Time", "%.1fs", gameTimer.seconds());
            et.addData("Loop Time", "%.1fms (avg %.1fms)", loopTimer.milliseconds(), avgLoopMs());

            if (!modules.isEmpty()) {
                et.addSeparator();
                et.addGroupHeader("MODULES", HtmlFormatter.COLOR_MODULE);
                for (Module m : telemetryOrderedModules()) m.telemetry();
            }

            if (telemetryToggles.loopProfile) {
                et.addSeparator();
                et.addGroupHeader("LOOP PROFILE (avg ms)", HtmlFormatter.COLOR_BLUE);
                for (Map.Entry<String, Double> entry : profiler.snapshotSortedDesc()) {
                    et.addDashboardData(entry.getKey(), "%.2fms", entry.getValue());
                }
            }

            renderFieldMap();
        } else {
            telemetry.addData("Game Time", "%.1fs", gameTimer.seconds());
            telemetry.addData("Loop Time", "%.1fms (avg %.1fms)", loopTimer.milliseconds(), avgLoopMs());
            for (Module m : telemetryOrderedModules()) m.telemetry();
        }

        telemetry();
        telemetry.update();
    }

    private void renderFieldMap() {
        Pose fieldPose = robot.follower.getPose();
        double fx = fieldPose.getX();
        double fy = fieldPose.getY();
        double fh = robot.follower.getHeading();
        // Re-render only when the robot moved meaningfully or we've gone N loops without a refresh.
        boolean poseChanged = Math.abs(fx - lastFieldRenderX) > 0.5
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
    }

    protected void updateDashboard(Pose currentPose) {
        int every = Math.max(1, dashboardEveryNLoops);
        if (every > 1 && (dashboardLoopCounter++ % every) != 0) {
            // Drop accumulated draws on skipped loops by replacing the packet wholesale.
            robot.packet = new TelemetryPacket(false);
            return;
        }

        Canvas overlay = robot.packet.fieldOverlay();

        if (!dashboardSkipFieldImage) {
            overlay.setAlpha(0.4);
            overlay.drawImage("/images/fieldcoordinates-pedro.png", 0, 0, 144, 144);
            overlay.setAlpha(1);
        }

        if (!dashboardSkipGrid) overlay.drawGrid(0, 0, 144, 144, 7, 7);

        FieldVisualization.drawRobot(currentPose);

        if (!dashboardSkipPoseHistory) {
            int poseEvery = Math.max(1, dashboardPoseHistoryEveryNLoops);
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
        if (telemetryToggles.voltage) robot.telemetry.addDashboardData("Voltage", "%.2fV", voltage);
        if (telemetryToggles.current) robot.telemetry.addDashboardData("Current", "%.2fA", getTotalCurrent());
    }

    public final void requestStop() { stopRequested = true; }
    public final boolean isRunning() { return running; }
    public final double getVoltage() { return voltage; }
    public final ElapsedTime getGameTimer() { return gameTimer; }

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

    public final double getTotalCurrent() {
        int every = Math.max(1, currentReadEveryNLoops);
        if (every == 1 || currentReadLoopCounter == 0) {
            double total = 0;
            for (LynxModule hub : lynxHubs) total += hub.getCurrent(CurrentUnit.AMPS);
            cachedTotalCurrent = total;
        }
        currentReadLoopCounter = (currentReadLoopCounter + 1) % every;
        return cachedTotalCurrent;
    }

    public final List<Module> getModules() {
        return Collections.unmodifiableList(modules);
    }

    public final LoopProfiler getProfiler() { return profiler; }
}
