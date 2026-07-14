# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project context

This is the **pre-season-only** summer 2026 repository for **FIRST Tech Challenge team 6165 MSET Cuttlefish** (Saratoga, CA — NorCal region, competing since 2012). It is intentionally a learning + testing scaffold ahead of the **BIOBUZZ™** season; there is no real robot code in here yet. Once BIOBUZZ kicks off, formal-season competition code moves to a separate repo — work here stops then.

**Season chronology:**

- **2025–2026 — DECODE™** (just-finished season; FIRST AGE™ program). Archaeology-themed: collect/shoot/align "artifacts," manage gates, overflow scoring. This repo is not for DECODE.
- **2026–2027 — BIOBUZZ™** (upcoming season; FIRST CANOPY™ program; kickoff **September 12, 2026**). Nature/pollination theme. Game element is **Pollen** — ~2.8" yellow plastic balls. Full game manual and robot rules drop at kickoff; pre-kickoff work is constrained to drivetrain + intake experimentation against the published *StarterBot Base* designs and the pre-season pollen challenges (intaking from open field, from corners/walls, autonomous navigation between known points).

**What this means for code written here:** assume the field, the scoring rules, and the legal robot config are not yet known. Don't hard-code field-element counts, scoring tables, alliance-station coordinates, or auto path constants that imply a specific BIOBUZZ field layout — those will all be invalidated at kickoff, at which point this repo's job is done. Drivetrain code, intake prototypes, sensor wrappers, vision pipelines, dashboard/telemetry plumbing, and Sloth hot-reload workflows are all fair game and may be ported into the in-season repo. When in doubt, build for swappability over a guess at the spec.

**Upstream / sibling repos worth knowing about:**

- `github.com/6165-MSET-Cuttlefish/Sloth` — team's audit-fixed fork of Sloth (consumed here via `ext.slothRef`).
- `github.com/6165-MSET-Cuttlefish/slothboard` — team's fork of `Dairy-Foundation/ftc-dashboard` (consumed here via `ext.slothboardRef`).

## Keep this file fresh

Claude maintains this file, not the user. After any change that alters how the project is built, deployed, structured, or what its conventions are — update this file in the same commit. Examples that warrant an update: bumping the FTC SDK version, adding/removing a Gradle module, changing the ABI / target SDK, swapping dependency hosting (e.g. moving Sloth off JitPack), introducing a new Gradle task that's part of the dev loop, or codifying a new convention. Updates that are just "I edited file X" do not warrant a CLAUDE.md change.

## Build & deploy

```bash
./gradlew :TeamCode:assembleDebug      # debug APK at TeamCode/build/outputs/apk/debug/
./gradlew :TeamCode:installDebug       # full ADB install — required for the first deploy onto a Control Hub
./gradlew :TeamCode:deploySloth        # hot-reload OpMode classes only (subsequent iterations)
./gradlew clean :TeamCode:assembleDebug
```

`deploySloth` only works after the Sloth runtime is already on the device, so do `installDebug` once after wiping the Control Hub or after a Sloth-runtime upgrade. The hot-reload protocol uses `/storage/emulated/0/FIRST/dairy/sloth/sloth.lock` to coordinate; if a deploy hangs waiting for that file, the previous load probably crashed mid-swap.

There is no test suite in this repo — it's a robot controller APK, not a library. `./gradlew :TeamCode:lint` exists via AGP but no team lint baseline is checked in.

**Build JDK: Java 17.** The Gradle daemon is pinned to Java 17 via `gradle/gradle-daemon-jvm.properties` (`toolchainVersion=17`) — AGP 8.7's required JDK. Building on a newer JDK (e.g. 25) only produced deprecation warnings, but JDK 17 is the supported baseline, so a JDK 17 install must be discoverable (Gradle 8.9 can't auto-download one — that landed in 8.10). The `./gradlew` launcher JVM is separate and can be anything on `PATH`; only the daemon JVM (which runs AGP/javac/dex) is pinned. Don't bump this without checking AGP/Gradle JDK compatibility.

The framework and OpModes are pure Java, but **Kotlin is wired in** for the `purepursuit/` experiment tree (its `*.kt` sources). The root `build.gradle` pins `ext.kotlin_version = '2.4.0'` as the single source of truth for the `kotlin-gradle-plugin` classpath; `TeamCode/build.gradle` applies `org.jetbrains.kotlin.android` with `kotlinOptions { jvmTarget '1.8' }`; `androidx.core:core-ktx` is on the classpath. Kotlin 2.4.0 fully supports the pinned Gradle 8.9 (its supported range is 7.6.3–9.5.0), so no Gradle bump is needed. If the `purepursuit/` tree is removed, dropping the Kotlin plugin + `core-ktx` reverts this to pure Java.

## Module / Gradle structure

**Single-module Android application.** The traditional FTC SDK template ships two modules (`FtcRobotController` and `TeamCode`); they've been collapsed into one. The `FtcRobotControllerActivity`, `PermissionValidatorWrapper`, and `FtcOpModeRegister` live under `TeamCode/src/main/java/org/firstinspires/ftc/robotcontroller/internal/`. Framework + user OpModes live under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. The Gradle namespace is `com.qualcomm.ftcrobotcontroller` (matches `applicationId`) so the bundled activity's `import com.qualcomm.ftcrobotcontroller.BuildConfig` resolves; user OpMode Java packages are unaffected.

**No `FtcRobotController/` module.** Don't re-introduce one. The SDK is consumed entirely as Maven Central artifacts (`org.firstinspires.ftc:*:11.1.0`). If a new SDK version is needed, bump the version strings in `TeamCode/build.gradle` — there's no in-tree SDK clone to keep in sync.

**Target hardware: REV Control Hub only.** `abiFilters` is `arm64-v8a` only. Don't add `armeabi-v7a` back unless legacy RC phones come back into the picture; the Control Hub is RK3328 (arm64) and RC phones aren't competition-legal as of the 2024-2025 season.

**Sloth integration.** The Sloth runtime AAR and its Gradle Load plugin come from the team's audit-fixed fork at `https://github.com/6165-MSET-Cuttlefish/Sloth`, pinned by `ext.slothRef` in the root `build.gradle`. The Load plugin (`apply plugin: 'dev.frozenmilk.sinister.sloth.load'` in `TeamCode/build.gradle`) registers `assembleSloth` / `dexSloth` / `deploySloth` / `removeSlothRemote`. Sloth pulls `dev.frozenmilk:Sinister:2.2.0` transitively from `https://repo.dairy.foundation/releases` — both that repo and JitPack are declared in the root `build.gradle`'s `allprojects` block. To bump Sloth, edit `ext.slothRef` to a new commit SHA on the 6165 fork.

**Dashboard.** Telemetry/control comes via slothboard (`com.github.6165-MSET-Cuttlefish.slothboard:dashboard`), the team's fork of `Dairy-Foundation/ftc-dashboard`. Pinned by `ext.slothboardRef` in the root `build.gradle`. Both Sloth and slothboard pin by commit SHA rather than release tag — it's a single-team chain with no external consumers expecting semver. Slothboard's POM transitively pulls the 6165 Sloth fork, so the chain is `summer-2026 → slothboard → 6165/Sloth`. To bump slothboard, edit `slothboardRef`; don't introduce tags.

**Pedro Pathing.** Path follower comes from `com.pedropathing:ftc:2.1.2` (Maven Central). The team's wrapper layer assumes Pedro 2.x semantics — `Pose` is immutable, `Pose.mirror(fieldWidth)` exists, `FTCCoordinates.INSTANCE` is the field-centered coordinate system, `PoseHistory` is a 30-sample fixed ring, `Follower.getTotalDistanceRemaining()` is the chain-aware remaining distance. Don't downgrade without checking the wrapper for callers of those.

**Roadrunner (purepursuit experiment only).** The `purepursuit/` tree pulls `com.acmerobotics.roadrunner:{ftc:0.1.25,core:1.0.1,actions:1.0.1}` plus `org.ftclib.ftclib:core`, which require the `https://maven.brott.dev/` repo declared in `TeamCode/build.gradle`. FTC Dashboard is excluded from all configurations (`configurations.configureEach { exclude group: 'com.acmerobotics.dashboard' }`) so slothboard is the only dashboard on the classpath. These deps ship in every APK regardless of which OpMode runs — drop them (and the Kotlin plugin) if the pure-pursuit testbed is removed.

## Framework layout

The team's framework lives under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`, **entirely within the `architecture/` package**. (The old top-level `core/` package was merged into `architecture/core/` + `architecture/action/`; nothing lives in `core/` anymore.)

- **`architecture/core/`** — central abstractions every game uses.
  - `EnhancedOpMode.java` — base OpMode with auto-discovered modules, voltage compensation, dual telemetry, dashboard field rendering, action scheduler pump.
  - `Robot.java` — game-agnostic robot base. Game subclass instantiates mechanism modules in `initializeGameModules()`. Builds the Pedro `Follower` (always from the configured start pose — no pose-carry across Sloth reloads).
  - `Module.java` — base for a hardware subsystem. Override `initStates()`, `read()`, `write()`, optional `init()` and `stop()`.
  - `State.java` — state-machine state interface; backing maps live as statics on the interface itself, cleared per-OpMode via `State.clearModuleBindings()`.
  - `AllianceColor.java`, `Context.java` — the only cross-game shared state is alliance side.
- **`architecture/action/`** — `Action`, `ActionBuilder`, `Actions` (cooperative single-thread scheduler).
- **`architecture/`** — other supporting wrappers and infrastructure.
  - `OptimizationToggles.java` — framework-wide `@Config` toggles for telemetry cadence, profiler enable, etc.
  - `auto/` — Pedro Pathing setup + path-action scheduler. `PedroSetup` is a **per-robot selector** (`PedroSetup.activeRobot` ∈ `{BETTA, DECODE}`, `@Config`) whose `createFollower(...)` dispatches to `BettaPedroSetup`+`BettaHardwareConfig` or `DecodePedroSetup`+`DecodeHardwareConfig` (tuning + hardware wiring split per robot). DECODE OpModes pin `activeRobot=DECODE` in `DecodeOpMode.createRobot()` before the follower is built. `scheduler/*` is the autonomous DSL on top of Pedro (its builders/scheduler take an injected `Follower` + game-time supplier — there is no `Robot.robot` service locator).
  - `control/` — `PidController` (PID + feed-forward + static-friction kick).
  - `hardware/` — `EnhancedMotor`, `EnhancedServo`, `EnhancedCRServo`, `WriteCache`, `BatteryVoltage`, `AbsoluteAnalogEncoder`, `LaserRangefinder`.
  - `input/` — gamepad layering and edge-detecting suppliers: `LayerStack`, `LayeredGamepad`, `LayerGamepad`, `EdgeBooleanSupplier`, `CachedDoubleSupplier`, `InputClock` (per-loop frame counter advanced by `EnhancedOpMode` so edge suppliers refresh exactly once per loop).
  - `prism/` — vendored goBilda Prism RGB LED driver (MIT-licensed Base 10 Assets code; don't refactor, track upstream).
  - `telemetry/` — `DualTelemetry` (fan-out DS + dashboard), `HtmlFormatter`, `FieldMapRenderer` (DS field map), `LoopProfiler` (per-section timing).
  - `testing/` — `HardwareTest` (single-device bench-test OpMode templates).

When adding a new file, prefer placing it in an existing subpackage. New top-level groupings should be rare.

## OpMode lifecycle

`EnhancedOpMode.init()` runs once. Order:

1. `State.clearModuleBindings()` and `Actions.cancelAll()` — drop stale framework state from prior runs (Sloth hot-reloads, back-to-back opmode runs).
2. Configure Lynx hubs to manual bulk caching, capture voltage sensor.
3. `createRobot()` — game subclass hook. Instantiates Robot (and indirectly its Modules).
4. `autoDiscoverModules()` — reflection walk of OpMode + Robot fields, registers everything `instanceof Module`.
5. `initModules()` — for each Module: set its telemetry handle, call `initStates()` (binds State→Module and sets initial values), then call the optional `init()` hook.
6. Build the sorted telemetry-modules list once (if `telemetrySortModulesOnce`).
7. `initialize()` — user hook.
8. Construct the `FieldMapRenderer` field-map and snapshot it.

`init_loop()` and `loop()` share the same per-tick pipeline; `loop()` calls `gameLoop()` where `init_loop()` calls `initializeLoop()`, and `loop()` writes unconditionally where `init_loop()` gates writes on `shouldWriteDuringInit()` + a 500 ms post-init grace. Both pump `Actions.update()` (init-scope actions are cancelled by `Actions.reset()` at `start()`):

```
profiler.start
clearBulkCaches             // every loop, manual mode
updateVoltageThrottled      // every voltageReadLoopInterval (default 50)
onLoopStart                 // game subclass hook
readModules                 // refreshTunables() then m.read() per module
robot.follower.update       // Pedro odometry + path follow tick
gameLoop                    // user code (init_loop calls initializeLoop instead)
Actions.update              // pump cooperative action scheduler
writeModules                // m.write() per module if isWriteEnabled
updateTelemetry             // gated by telemetryEveryNLoops; addStatusTelemetry
                            // (alliance, pose, voltage, optional current) +
                            // framework data + per-module telemetry +
                            // braille field map
updateDashboard             // build + send TelemetryPacket via FtcDashboard
optional Thread.sleep       // if minLoopMs forces a floor
```

`Actions.update()` runs **between** `gameLoop()` and `writeModules()` so action-applied state lands in the same write pass as user code. Don't move it.

`stop()` cancels all actions, cancels the path-action scheduler, calls `module.stop()` on every module then `onEnd()` — this is the framework's safe-state pass, so it is best-effort: each `module.stop()` and `onEnd()` runs even if an earlier one throws (the SDK does not auto-zero motors on stop), and the **first** captured `Throwable` is rethrown after all of them have run, preserving fail-fast visibility without leaving hardware energized because an unrelated module threw first.

## Module pattern

```java
public class Shooter extends Module {
    public enum FlywheelState implements State {
        OFF(0), SHOOT(5400);
        FlywheelState(double rpm) { setValue(rpm); }
    }

    @Config public static class Tuning {
        public static double shootRpm = 5400;
    }

    private final EnhancedMotor flywheel;

    public Shooter(HardwareMap hw) {
        flywheel = new EnhancedMotor(hw, "flywheel").withVoltageCompensation(12.0);
    }

    @Override protected void initStates() {
        setStates(FlywheelState.OFF);
        bindTunable(FlywheelState.SHOOT, () -> Tuning.shootRpm);
    }

    @Override protected void read() {
        // Read sensors. Tunables are already refreshed by the time we get here.
    }

    @Override protected void write() {
        flywheel.setVelocity(getState(FlywheelState.class).getValue());
    }

    @Override public void stop() {
        flywheel.setPower(0);  // safe-state on opmode end
    }
}
```

Conventions:
- Mechanism state is an enum implementing `State`. Each variant carries its setpoint via `setValue` in the ctor.
- `setStates(...)` once with the initial state per state-class. The framework binds every enum constant to this Module and re-applies its initial value, so a re-run starts clean.
- `bindTunable(state, supplier)` after `setStates` for any setpoint that should be live-tunable. The framework re-applies the supplier value before each `read()`.
- Pair tunables with a `@Config static class Tuning { public static double ... }` so FtcDashboard sees them as sliders.
- `read()` reads sensors; `write()` commands hardware. They run every loop on the OpMode thread.
- `init()` is optional — override for one-shot setup that runs once after `initStates()` has bound states. Both `init()` and `initStates()` run from `EnhancedOpMode.initModules()`, after the subclass constructor completes, so both can freely reference instance fields set up by the constructor.
- `stop()` is optional — override to put hardware in a safe state on opmode end.
- `state.activate()` from anywhere calls `module.setState(state)`. Returns true on success or no-op same-state; false only when the state class isn't registered or a guard rejected the transition.

## Action system

`Actions` is a single-threaded cooperative scheduler. Composable from primitives; runs on the OpMode thread between `gameLoop()` and `writeModules()`. No coroutines, no background threads, no concurrency.

```java
Actions.builder()
    .set(IntakeState.IDLE, MagazineState.READY)
    .delay(200)
    .waitUntil(() -> robot.intake.isFull())
    .run(() -> robot.lights.flash())
    .build()
    .schedule();
```

Composers: `sequence`, `parallel`, `race`, `timeout`, `repeat`, `loop`, `ifThen`, `ifElse`, `retry`. All return new `Action`s; can be nested.

Conflict model: when an action is scheduled, any running action whose `targets` set intersects gets cancelled. `targets` is auto-populated by `set(State...)` (each State's owning Module) and by the inner actions of composers. Manual `targets(module)` exists if needed.

Steps execute one per scheduler tick (one per OpMode loop). Instantaneous steps (`set`, `run`, `stopIf`) chain within a single tick — only blocking steps (`delay`, `waitUntil`, in-flight sub-actions) yield. Compose `delay(ms)` not `Thread.sleep(ms)`; `waitUntil(cond)` not `while (!cond) {}`.

`Action.runBlocking()` and `Action.runSuspending()` no longer exist (they were coroutine-only foot-guns).

## Path-action scheduler

`PathActionBuilder` (`architecture/auto/scheduler/`) composes autonomous sequences of Pedro paths and Actions. Each tick of `PathActionScheduler.update()` advances the current segment by exactly one transition; the scheduler never blocks the loop.

```java
robot.pathActionScheduler = new PathActionBuilder()
    .setStartPose(robot.follower.getPose())
    .setState(IntakeState.PICKUP)
    .buildPath(p -> p.addLine(scorePose).setLinearHeading(0, Math.PI/2))
    .action(robot.actions.shootAll())
    .build();
```

Queued `setState`/`run`/`actionDuring` calls accumulate and flush into the prelude/during of the next real segment, so they cost zero extra ticks. `setOverride(condition, handler)` (or `setTimeOverride(ms, handler)`) cancels the sequence and runs cleanup. `skipCurrentSegment()` and segment timeouts both honor after-actions before advancing — autos that "drive then shoot" still shoot when the drive doesn't finish in time.

The scheduler is pumped from your auto's `gameLoop()`:

```java
@Override protected void gameLoop() {
    robot.pathActionScheduler.update();
    // ... gamepad-driven overrides etc. ...
}
```

## Tuning

There are three layers of "knob," in order of where to look:

1. **Per-mechanism `@Config static class Tuning`** — setpoints, gains, offsets. Pair with `Module.bindTunable(state, () -> Tuning.x)` so dashboard edits land in the next `read()`.
2. **`{Betta,Decode}PedroSetup`** (`architecture/auto/`) — per-robot Pedro `FollowerConstants` / `MecanumConstants` / `PinpointConstants` / `PathConstraints`. Edit the robot's file when retuning path follow. `PedroSetup.activeRobot` selects which one `createFollower` uses.
3. **`{Betta,Decode}HardwareConfig`** (`architecture/auto/`) — per-robot motor names, motor directions, Pinpoint pod offsets. Fields are `final`; edit the robot's file when wiring that robot. (Betta and DECODE differ in the strafe-pod encoder direction.)

Plus framework-wide knobs:

- **`OptimizationToggles`** (`architecture/`) — runtime perf tunables (dashboard / telemetry cadence, profiler enable, current-read cadence, etc.). All are live; default values favor visibility, tighten for competition.
- **`Robot.telemetryToggles`** — `dsTelemetry`, `dashboardTelemetry`, `voltage`, `current`, `loopProfile`. `loopProfile` is the master switch for the per-section breakdown rendered when tuning loop time.

Avoid scattering ad-hoc `public static double` fields. If something is tunable, it lives in one of the buckets above.

## Testing

`architecture/testing/HardwareTest` provides nested abstract OpMode templates for single-device bench tests:

```java
@TeleOp(name = "Test leftClaw")
public class LeftClawTest extends HardwareTest.ServoPosition {
    @Override protected String hardwareName() { return "leftClaw"; }
}
```

Three flavors: `ServoPosition` (D-pad nudges position), `MotorPower` (left stick drives power, logs encoder), `CRServoPower` (left stick drives power). Subclass, override `hardwareName()`, slap on `@TeleOp`. Gamepad-driven so they work without a dashboard.

For "test one module in isolation" patterns, write a custom OpMode extending `OpMode` directly (not `EnhancedOpMode`) — the framework wants a `Robot` with discoverable modules, which is too much ceremony for a single-mechanism test.

## Conventions

- Source/target `JavaVersion.VERSION_1_8`, `targetSdkVersion 28` — pinned by the FTC SDK. Don't bump without verifying with the FTC SDK release notes.
- The `versionCode` (61) and `versionName` ("11.1") in `TeamCode/build.gradle`'s `defaultConfig` track the FTC SDK version they were derived from. Bump in lockstep with `org.firstinspires.ftc:*` deps when upgrading.
- Debug keystore at `libs/ftc.debug.keystore` is the FTC standard one (matches RC firmware installer's expectations); don't replace it.
- `TeamCode/lib/OpModeAnnotationProcessor.jar` is the FTC annotation processor — leave it where it is.
- Comments: only when the WHY is non-obvious. No method-name-restating Javadoc, no section-header banners (`// ─── label ──`). Short docs on user-facing public API where the name alone doesn't carry intent are fine.
- **Fail-fast — don't swallow exceptions.** Let exceptions propagate so failures are visible; a crashed OpMode with a stack trace beats one limping on in a bad state. Don't add try/catch to "protect" a loop or hide a bad-state exception, and prefer removing existing exception-swallowing (empty/`ignored` catches, broad catches that only log). Numeric guards (divide-by-zero, NaN) and resource cleanup via try/**finally** (no catch — the exception still propagates) are fine. When bridging a checked exception (e.g. `Callable` → `BooleanSupplier`/`Runnable`), rethrow it as unchecked rather than returning a default.
- Test/sample OpModes go under `opmodes/test/` (already present: `MockAuto`/`MockMechanism`/`MockRobot` framework smoke test, camera + distance-sensor tests).

## Repo conventions discovered from prior conversations

- Commit messages are short subject + a one-line "why" body. Examples: `Drop armeabi-v7a, target arm64-v8a only`, `Merge FtcRobotController module into TeamCode`. Avoid multi-paragraph commit bodies unless the change is genuinely complex.
- Pushes directly to `main` are gated by a hook; ask before pushing if it's not obvious the user wants it.
