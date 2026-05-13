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

## Architecture

**Single-module Android application.** The traditional FTC SDK template ships two modules (`FtcRobotController` and `TeamCode`); they've been collapsed into one. The `FtcRobotControllerActivity`, `PermissionValidatorWrapper`, and `FtcOpModeRegister` live under `TeamCode/src/main/java/org/firstinspires/ftc/robotcontroller/internal/`. User OpModes live under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. The Gradle namespace is `com.qualcomm.ftcrobotcontroller` (matches `applicationId`) so the bundled activity's `import com.qualcomm.ftcrobotcontroller.BuildConfig` resolves; user OpMode Java packages are unaffected.

**No `FtcRobotController/` module.** Don't re-introduce one. The SDK is consumed entirely as Maven Central artifacts (`org.firstinspires.ftc:*:11.1.0`). If a new SDK version is needed, bump the version strings in `TeamCode/build.gradle` — there's no in-tree SDK clone to keep in sync.

**Target hardware: REV Control Hub only.** `abiFilters` is `arm64-v8a` only. Don't add `armeabi-v7a` back unless legacy RC phones come back into the picture; the Control Hub is RK3328 (arm64) and RC phones aren't competition-legal as of the 2024-2025 season.

**Sloth integration.** The Sloth runtime AAR and its Gradle Load plugin come from the team's audit-fixed fork at `https://github.com/6165-MSET-Cuttlefish/Sloth`, pinned by `ext.slothRef` in the root `build.gradle`. The Load plugin (`apply plugin: 'dev.frozenmilk.sinister.sloth.load'` in `TeamCode/build.gradle`) registers `assembleSloth` / `dexSloth` / `deploySloth` / `removeSlothRemote`. Sloth pulls `dev.frozenmilk:Sinister:2.2.0` transitively from `https://repo.dairy.foundation/releases` — both that repo and JitPack are declared in the root `build.gradle`'s `allprojects` block. To bump Sloth, edit `ext.slothRef` to a new commit SHA on the 6165 fork.

**Dashboard.** Telemetry/control comes via slothboard (`com.github.6165-MSET-Cuttlefish.slothboard:dashboard`), the team's fork of `Dairy-Foundation/ftc-dashboard`. Pinned by `ext.slothboardRef` in the root `build.gradle` (currently `edd4aca`). Both Sloth and slothboard pin by commit SHA rather than release tag — it's a single-team chain with no external consumers expecting semver. Slothboard's POM transitively pulls the 6165 Sloth fork, so the chain is `summer-2026 → slothboard → 6165/Sloth`. To bump slothboard, edit `slothboardRef`; don't introduce tags.

**Kotlin coroutines are wired up** (`kotlinx-coroutines-android:1.11.0`) but no OpMode uses them yet. Useful patterns are non-obvious: OpModes have no built-in `CoroutineScope` (unlike Android `Activity`/`ViewModel`); managed scope must be cancelled in `finally` after `runOpMode()` returns or in `OpMode.stop()`. Never `runBlocking` from the OpMode main thread — it blocks the control loop. Hardware reads are safe to call from background coroutines because the SDK synchronizes bus access, but logical races over your own state are your problem.

## Conventions

- Source/target `JavaVersion.VERSION_1_8`, `targetSdkVersion 28` — these are pinned by the FTC SDK's expectations. Don't bump without verifying with the FTC SDK release notes.
- The `versionCode` (61) and `versionName` ("11.1") in `TeamCode/build.gradle`'s `defaultConfig` track the FTC SDK version they were derived from. Bump in lockstep with `org.firstinspires.ftc:*` deps when upgrading.
- Debug keystore at `libs/ftc.debug.keystore` is the FTC standard one (matches RC firmware installer's expectations); don't replace it.
- `TeamCode/lib/OpModeAnnotationProcessor.jar` is the FTC annotation processor — leave it where it is.

## Repo conventions discovered from prior conversations

- Commit messages are short subject + a one-line "why" body. Examples: `Drop armeabi-v7a, target arm64-v8a only`, `Merge FtcRobotController module into TeamCode`. Avoid multi-paragraph commit bodies unless the change is genuinely complex.
- Pushes directly to `main` are gated by a hook; ask before pushing if it's not obvious the user wants it.
