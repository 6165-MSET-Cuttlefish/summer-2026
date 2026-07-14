package org.firstinspires.ftc.teamcode.architecture.core;

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
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.architecture.auto.FieldVisualization;
import org.firstinspires.ftc.teamcode.architecture.telemetry.LoopProfiler;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedCRServo;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.input.InputClock;
import org.firstinspires.ftc.teamcode.architecture.telemetry.FieldMapRenderer;
import org.firstinspires.ftc.teamcode.architecture.telemetry.DualTelemetry;
import org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter;
import org.firstinspires.ftc.teamcode.architecture.action.Action;
import org.firstinspires.ftc.teamcode.architecture.action.Actions;

import static org.firstinspires.ftc.teamcode.architecture.core.Robot.telemetryToggles;
import static org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter.*;
import static org.firstinspires.ftc.teamcode.architecture.OptimizationToggles.*;

/**
 * Base OpMode with auto-discovered modules, voltage compensation, dual telemetry, and dashboard
 * field rendering. Subclasses provide a {@link Robot} via {@link #createRobot()}.
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
    private long monotonicLoopCount = 0;
    private int voltageLoopCounter = 0;
    private int dashboardLoopCounter = 0;
    private int dashboardPoseHistoryLoopCounter = 0;
    private int telemetryLoopCounter = 0;

    private final List<Module> modules = new ArrayList<>();
    private List<Module> sortedTelemetryModules;
    private long lastCurrentReadLoop = Long.MIN_VALUE;
    private int initializedModuleCount = 0;
    private double cachedTotalCurrent = 0.0;
    private VoltageSensor voltageSensor;
    private double voltage = 12.0;
    private boolean running = false;
    private boolean stopRequested = false;
    private boolean isInit = false;

    private FieldMapRenderer field;
    private String cachedFieldHtml = "";
    private double lastFieldRenderX = Double.NaN;
    private double lastFieldRenderY = Double.NaN;
    private double lastFieldRenderHeading = Double.NaN;
    private int loopsSinceFieldRender = Integer.MAX_VALUE;
    private AllianceColor cachedAllianceColor;
    private String cachedAllianceHtml;
    /** Re-render the DS field map at most every N loops even when the pose hasn't moved. */
    protected int fieldRenderInterval = 10;

    /** Hold each loop to at least this many ms (0 = no holding). Stabilizes control timing. */
    protected int minLoopMs = 0;
    /** Loops between voltage reads. Bus call costs a few ms; voltage moves slowly. */
    protected int voltageReadLoopInterval = 50;
    protected boolean voltageCompensationEnabled = true;

    protected void initialize() {}
    protected void initializeLoop() {}
    protected void gameLoop() {}
    protected void onStart() {}
    protected void onEnd() {}
    protected void telemetry() {}

    protected boolean shouldWriteDuringInit() { return false; }
    protected boolean shouldReadDuringInit() { return true; }

    protected abstract Robot createRobot() throws InterruptedException;

    /** Called at the top of every init_loop and loop, before module reads. */
    protected void onLoopStart() {}

    @Override
    public final void init() {
        // State→Module map and Actions active list live in statics — drop stale entries from
        // prior runs (Sloth hot-reload, back-to-back opmodes) before modules instantiate.
        State.clearModuleBindings();
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
        initialize();
        // Pick up (and initialize) any modules registered/created during initialize(), then build
        // the sorted telemetry list once all modules — including late ones — are known.
        autoDiscoverModules();
        initModules();
        if (telemetrySortModulesOnce) buildSortedTelemetryModules();

        loopTimer.reset();
        gameTimer.reset();
        isInit = true;

        field = new FieldMapRenderer(73, 74);
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

        // New loop tick: edge suppliers (EdgeBooleanSupplier) refresh once against this frame.
        InputClock.advance();

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

        robot.follower.update();
        profiler.mark("follower.update");

        initializeLoop();
        profiler.mark("initializeLoop");

        // Init-scoped: start() calls Actions.reset(), so anything in flight at match start
        // gets cancelled.
        Actions.update();
        profiler.mark("actions");

        // 500 ms grace before the first write so SDK mode/direction calls settle.
        if (shouldWriteDuringInit() && gameTimer.milliseconds() > 500) {
            writeModules();
        }
        profiler.mark("writeModules");

        updateTelemetry();
        profiler.mark("updateTelemetry");

        updateDashboard();
        profiler.mark("updateDashboard");

        loopTimer.reset();
    }

    @Override
    public final void start() {
        running = true;
        gameTimer.reset();
        clearBulkCaches();
        // Reset throttle counters so the first match loop samples voltage/current/telemetry/field
        // fresh instead of inheriting a mid-cycle phase from the init loop.
        voltageLoopCounter = 0;
        telemetryLoopCounter = 0;
        dashboardLoopCounter = 0;
        dashboardPoseHistoryLoopCounter = 0;
        loopsSinceFieldRender = Integer.MAX_VALUE;
        lastCurrentReadLoop = Long.MIN_VALUE;
        // Drop init-phase timings so the first match loops don't average them into loop time.
        for (int i = 0; i < loopTimes.length; i++) loopTimes[i] = 0.0;
        loopIndex = 0;
        profiler.reset();
        Actions.reset();
        scheduleStartupActions();
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

        // New loop tick: edge suppliers (EdgeBooleanSupplier) refresh once against this frame.
        InputClock.advance();

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

        robot.follower.update();
        profiler.mark("follower.update");

        gameLoop();
        profiler.mark("gameLoop");

        // Run between user code and writes so action-applied state lands in the same write pass.
        Actions.update();
        profiler.mark("actions");

        writeModules();
        profiler.mark("writeModules");

        updateTelemetry();
        profiler.mark("updateTelemetry");

        updateDashboard();
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
        // stop() is the framework's safe-state pass: every module must get a chance to zero its
        // hardware even if an earlier module's stop() throws (the SDK does NOT auto-zero motors on
        // OpMode stop, and a normal STOP keeps the RC keepalive alive so the Lynx failsafe never
        // trips). We still fail-fast overall — the first Throwable is captured and rethrown after
        // every module and onEnd() have run, so the failure still surfaces, but nothing is left
        // energized because an unrelated module threw first.
        Throwable first = null;
        for (int i = 0; i < modules.size(); i++) {
            try {
                modules.get(i).stop();
            } catch (Throwable t) {
                if (first == null) first = t;
            }
        }
        try {
            onEnd();
        } catch (Throwable t) {
            if (first == null) first = t;
        }
        if (first != null) {
            if (first instanceof RuntimeException) throw (RuntimeException) first;
            throw (Error) first;
        }
    }

    /** Manually register modules; reflection auto-discovery picks up the rest. */
    protected void register(Module... mods) {
        for (Module m : mods) {
            if (!modules.contains(m)) modules.add(m);
        }
    }

    private void autoDiscoverModules() {
        // Identity-based so user classes that override equals/hashCode don't collide.
        Set<Object> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        try {
            discover(this, getClass(), visited);
            // discover() walks the subclass field hierarchy only down to (excluding) EnhancedOpMode,
            // so the inherited `robot` field is NOT a discovery root. An OpMode that returns its Robot
            // from createRobot() and only ever reads the inherited (base-typed) `robot` — with no
            // redundant typed field — would otherwise register zero modules silently. Seed from it
            // directly; idempotent via the visited set when a typed field already reached the robot.
            discoverValue(robot, visited);
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
                discoverValue(f.get(obj), visited);
            }
            clazz = clazz.getSuperclass();
        }
    }

    /** Register a Module, or descend through arrays/collections/maps and object fields to find them. */
    private void discoverValue(Object val, Set<Object> visited) throws IllegalAccessException {
        if (val == null) return;

        if (val instanceof Module) {
            register((Module) val);
        } else if (val instanceof Object[]) {
            for (Object e : (Object[]) val) discoverValue(e, visited);
        } else if (val instanceof Iterable<?>) {
            for (Object e : (Iterable<?>) val) discoverValue(e, visited);
        } else if (val instanceof Map<?, ?>) {
            for (Map.Entry<?, ?> e : ((Map<?, ?>) val).entrySet()) {
                discoverValue(e.getKey(), visited);
                discoverValue(e.getValue(), visited);
            }
        } else if (shouldRecurse(val.getClass())) {
            discover(val, val.getClass(), visited);
        }
    }

    private boolean shouldRecurse(Class<?> c) {
        String n = c.getName();
        return !n.startsWith("java.") && !n.startsWith("android.")
                && !n.startsWith("com.qualcomm.") && !n.startsWith("kotlin.");
    }

    private void initModules() {
        // Idempotent: only initialize modules not yet initialized, so calling this twice (before
        // and after initialize()) doesn't re-run init() on an already-initialized module.
        for (int i = initializedModuleCount; i < modules.size(); i++) {
            Module m = modules.get(i);
            m.setTelemetry(telemetry);
            m.initStates();
            m.init();
        }
        initializedModuleCount = modules.size();
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
            m.refreshTunables();
            long t = profiler.enterSection();
            m.read();
            profiler.leaveSection(m.getReadSectionName(), t);
        }
    }

    protected void writeModules() {
        for (int i = 0; i < modules.size(); i++) {
            Module m = modules.get(i);
            if (m.isWriteEnabled()) {
                long t = profiler.enterSection();
                m.write();
                profiler.leaveSection(m.getWriteSectionName(), t);
            }
        }
    }

    private void scheduleStartupActions() {
        for (Module m : modules) {
            Action startup = m.getStartupAction();
            if (startup != null && !Actions.isModuleActive(m)) startup.schedule();
        }
    }

    private void configureBulkCaching() {
        lynxHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : lynxHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void clearBulkCaches() {
        for (int i = 0; i < lynxHubs.size(); i++) lynxHubs.get(i).clearBulkCache();
    }

    private void updateVoltageThrottled() {
        int interval = Math.max(1, voltageReadLoopInterval);
        if (voltageLoopCounter == 0) voltage = voltageSensor.getVoltage();
        voltageLoopCounter = (voltageLoopCounter + 1) % interval;
    }

    private void recordLoopTime() {
        loopTimes[loopIndex] = loopTimer.milliseconds();
        loopIndex = (loopIndex + 1) % loopTimes.length;
        monotonicLoopCount++;
    }

    private void updateTelemetry() {
        int every = Math.max(1, telemetryEveryNLoops);
        if (every > 1 && (telemetryLoopCounter++ % every) != 0) {
            // Pending items carry over; calling update() here would cause partial-packet flicker.
            return;
        }

        // Wire the documented Robot.telemetryToggles DS/dashboard switches to the actual backend
        // gates (Decode did this in updateWriteToggles(); the summer refactor dropped the wiring,
        // leaving both fields dead). Cheap two-boolean write on render loops only.
        DualTelemetry.enableDSTelemetry = telemetryToggles.dsTelemetry;
        DualTelemetry.enableDashboardTelemetry = telemetryToggles.dashboardTelemetry;

        addStatusTelemetry();

        if (telemetry instanceof DualTelemetry) {
            DualTelemetry et = (DualTelemetry) telemetry;

            et.addDashboardData("Game Time", "%.1fs", gameTimer.seconds());
            et.addData("Loop Time", "%.1fms (avg %.1fms)", loopTimer.milliseconds(), avgLoopMs());

            if (!modules.isEmpty()) {
                et.addSeparator();
                et.addGroupHeader("MODULES", HtmlFormatter.COLOR_MODULE);
                List<Module> ordered = telemetryOrderedModules();
                for (int i = 0; i < ordered.size(); i++) ordered.get(i).telemetry();
            }

            if (telemetryToggles.loopProfile) {
                et.addSeparator();
                et.addGroupHeader("LOOP PROFILE (avg ms)", HtmlFormatter.COLOR_BLUE);
                List<Map.Entry<String, Double>> snapshot = profiler.snapshotSortedDesc();
                for (int i = 0; i < snapshot.size(); i++) {
                    Map.Entry<String, Double> entry = snapshot.get(i);
                    et.addDashboardData(entry.getKey(), "%.2fms", entry.getValue());
                }
            }

            renderFieldMap();
        } else {
            telemetry.addData("Game Time", "%.1fs", gameTimer.seconds());
            telemetry.addData("Loop Time", "%.1fms (avg %.1fms)", loopTimer.milliseconds(), avgLoopMs());
            List<Module> ordered = telemetryOrderedModules();
            for (int i = 0; i < ordered.size(); i++) ordered.get(i).telemetry();
        }

        telemetry();
        telemetry.update();
    }

    private void renderFieldMap() {
        Pose fieldPose = robot.follower.getPose();
        double fx = fieldPose.getX();
        double fy = fieldPose.getY();
        double fh = robot.follower.getHeading();
        boolean poseChanged = Math.abs(fx - lastFieldRenderX) > 0.5
                || Math.abs(fy - lastFieldRenderY) > 0.5
                || Math.abs(fh - lastFieldRenderHeading) > Math.toRadians(2);
        if (poseChanged || loopsSinceFieldRender >= fieldRenderInterval) {
            field.restore();
            field.drawRobot(fx, fy, fh,
                    Context.allianceColor.equals(AllianceColor.RED) ? COLOR_RED : COLOR_BLUE);
            cachedFieldHtml = HtmlFormatter.htmlSize(FONT_SMALL, field.renderHtml());
            lastFieldRenderX = fx;
            lastFieldRenderY = fy;
            lastFieldRenderHeading = fh;
            loopsSinceFieldRender = 0;
        } else {
            loopsSinceFieldRender++;
        }
        robot.telemetry.addDSLine(cachedFieldHtml);
    }

    protected void updateDashboard() {
        int every = Math.max(1, dashboardEveryNLoops);
        if (every > 1 && (dashboardLoopCounter++ % every) != 0) {
            // Discard any draws accumulated this loop by swapping in a fresh packet.
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

        FieldVisualization.drawRobot(overlay, robot.follower.getPose());

        if (!dashboardSkipPoseHistory) {
            int poseEvery = Math.max(1, dashboardPoseHistoryEveryNLoops);
            if ((dashboardPoseHistoryLoopCounter++ % poseEvery) == 0) {
                FieldVisualization.drawPoseHistory(overlay, robot.follower.getPoseHistory());
            }
        }

        FtcDashboard.getInstance().sendTelemetryPacket(robot.packet);
        robot.packet = new TelemetryPacket(false);
    }

    private void addStatusTelemetry() {
        Pose currentPose = robot.follower.getPose();
        robot.telemetry.addGroupHeader("ROBOT STATUS");
        addAllianceTelemetry();
        robot.telemetry.addData("Robot Position", "X: %.1f, Y: %.1f, Heading: %.1f°",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        addVoltageCurrentTelemetry();
    }

    private void addAllianceTelemetry() {
        if (Context.allianceColor != cachedAllianceColor) {
            cachedAllianceColor = Context.allianceColor;
            String colorHex = Context.allianceColor == AllianceColor.RED
                    ? HtmlFormatter.COLOR_RED
                    : HtmlFormatter.COLOR_BLUE;
            cachedAllianceHtml = HtmlFormatter.htmlColor(colorHex,
                    HtmlFormatter.htmlBold(String.valueOf(Context.allianceColor)));
        }
        robot.telemetry.addDSRawHtml("Alliance", cachedAllianceHtml);
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
        // Throttle against the real loop counter, not call count: getTotalCurrent() is only invoked
        // when the current telemetry toggle is on AND telemetry renders this loop, so a per-call
        // counter would read far less often than currentReadEveryNLoops promises.
        if (lastCurrentReadLoop == Long.MIN_VALUE || monotonicLoopCount - lastCurrentReadLoop >= every) {
            double total = 0;
            for (LynxModule hub : lynxHubs) total += hub.getCurrent(CurrentUnit.AMPS);
            cachedTotalCurrent = total;
            lastCurrentReadLoop = monotonicLoopCount;
        }
        return cachedTotalCurrent;
    }

    public final List<Module> getModules() {
        return Collections.unmodifiableList(modules);
    }

    public final LoopProfiler getProfiler() { return profiler; }
}
