package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;
import org.firstinspires.ftc.teamcode.architecture.core.State;

/**
 * Robot for the ported DECODE "Close" auto, running on the summer-2026 scaffold.
 *
 * <p>The real DECODE mechanisms (Shooter / Turret / Magazine / Drivetrain / Endgame) do not exist
 * in this pre-season repo, so every mechanism the auto touches is a <strong>hardware-free STUB</strong>
 * {@link Module}: it carries the exact state enums the Close sequence references, but its
 * {@code read()}/{@code write()} are no-ops (it commands no hardware) and it only flips the
 * State machine + the few public flags/fields the auto reads. The single real output is the Pedro
 * follower driving the (real) drivetrain — that is expected for an auto and is owned by the framework.
 *
 * <p>Stubs are nested static classes for tidiness; each is exposed as a public field so module
 * auto-discovery (reflection over Robot + OpMode fields, after {@code createRobot()} returns) picks
 * it up. The {@code RobotActions}-style helpers and the lock / getCurrentVelocityRPM /
 * closeVelocityOffset surface the Close lambdas call live on the stubs, so the ported sequence reads
 * line-for-line like the original while actuating nothing.
 */
public class CloseRobot extends Robot {

    // Stand-ins for Context.redTargetPose / blueTargetPose (DECODE put these statics in Context and
    // wired them per-alliance in DecodeRobot.setTargetPosesForAlliance(); summer-2026's Context only
    // holds allianceColor, so they live here on the robot instead).
    public static final Pose redTargetPose = new Pose(124.5, 144, 0);
    public static final Pose blueTargetPose = new Pose(17, 144, 0);

    // Mirrors DecodeRobot.targetPose — the live aim point the auto re-points the turret at.
    public Pose targetPose = redTargetPose;

    public Drivetrain drivetrain;
    public Shooter shooter;
    public Turret turret;
    public Magazine magazine;
    public Endgame endgame;

    // Mirrors DecodeRobot.actions (the RobotActions factory). Hardware-free here: every factory
    // builds a mock Actions sequence that drives the stub Magazine's states + preserves the delays.
    public RobotActions actions;

    public CloseRobot(EnhancedOpMode opMode) throws InterruptedException {
        // The auto seeds the follower from the setup pose in initialize().
        super(opMode);
    }

    @Override
    protected void initializeGameModules() {
        // Assigned here (not via field initializers): this runs inside the Robot super-ctor, before
        // subclass field initializers, which would otherwise null these out.
        HardwareMap hw = opMode.hardwareMap;
        drivetrain = new Drivetrain(hw);
        shooter = new Shooter(hw);
        turret = new Turret(hw);
        magazine = new Magazine(hw);
        endgame = new Endgame(hw);
        actions = new RobotActions(this);
    }

    // ---------------------------------------------------------------------------------------------
    // Stub modules. Each carries ONLY the states the Close auto references, no hardware.
    // Setpoint values are copied verbatim from the DECODE module enums so dashboard/telemetry match.
    // ---------------------------------------------------------------------------------------------

    /** 4-motor mecanum base. In auto it's driven by Pedro; the stub only models DriveState + bonk. */
    public static class Drivetrain extends Module {
        public enum DriveState implements State {
            MANUAL(0), EXTERNAL(0);
            DriveState(double v) { setValue(v); }
        }

        public Drivetrain(HardwareMap hw) { /* no hardware */ }

        @Override protected void initStates() { setStates(DriveState.EXTERNAL); }
        @Override protected void read() {}
        @Override protected void write() {}

        /** DECODE used isBonk()/bonkTimer for stall recovery; never true in the hardware-free stub. */
        public boolean isBonk() { return false; }
    }

    /** Flywheel + adjustable hood. Stub only flips FlywheelState/HoodState and no-ops the lock path. */
    public static class Shooter extends Module {
        public enum FlywheelState implements State {
            IDLE(1800), FAR(3150), FAR_AUTO(3150), CLOSE(2750),
            CLOSE_AUTO(2420), CLOSE_AUTO_PRELOAD(2410), CLOSE_AUTO_PARK(2200),
            LOW(1500), OFF(0), MANUAL(0.2), COAST_TO_TARGET(0), PID(0);
            FlywheelState(double v) { setValue(v); }
        }

        public enum HoodState implements State {
            RESET(0.585), BOTTOM(0), SUPER_CLOSE(0.31), KINDA_CLOSE(0.28),
            FAR(0.15), FAR_AUTO(0.15), CLOSE(0.19), CLOSE_AUTO(0.115),
            CLOSE_AUTO_PRELOAD(0.12), CLOSE_AUTO_PARK(0.04), TOP(0.32),
            MANUAL(-1), PID(0);
            HoodState(double v) { setValue(v); }
        }

        /** DECODE Shooter.closeVelocityOffset — per-alliance RPM trim the auto writes each cycle. */
        public double closeVelocityOffset = 0;

        public Shooter(HardwareMap hw) { /* no hardware */ }

        @Override protected void initStates() { setStates(FlywheelState.OFF, HoodState.BOTTOM); }
        @Override protected void read() {}
        @Override protected void write() {}

        @Override public void stop() { setState(FlywheelState.OFF); }

        // --- lock / LUT surface the Close lambdas call. No-ops on the stub. ---
        public void lockFlywheel(Pose target) { /* no hardware */ }
        public void lockHood(Pose target) { /* no hardware */ }
        public void unlock() { /* no hardware */ }

        /** DECODE gates the preload shot on this climbing past 1000; the stub reports "spun up". */
        public double getCurrentVelocityRPM() { return getState(FlywheelState.class).getValue(); }
    }

    /** Two-servo aiming turret + Limelight. Stub only models the lock/unlock no-ops the auto calls. */
    public static class Turret extends Module {
        public enum TurretState implements State {
            CENTER(0.5), RIGHT(0.995), LEFT(0.005),
            AUTOAIM(-1), HOLD(-1), OFF(-1), MANUAL(-1);
            TurretState(double v) { setValue(v); }
        }

        public Turret(HardwareMap hw) { /* no hardware */ }

        @Override protected void initStates() { setStates(TurretState.AUTOAIM); }
        @Override protected void read() {}
        @Override protected void write() {}

        public void lock(Pose target) { /* no hardware */ }
        public void unlock() { /* no hardware */ }
    }

    /**
     * Ball handling hub (intake / vertical feeder / two horizontal pushers). The Close auto drives
     * IntakeState + VerticalState and reads {@code shotSorted}; the other lanes/LEDs are modeled so
     * the RobotActions mock sequences have somewhere to write.
     */
    public static class Magazine extends Module {
        public enum IntakeState implements State {
            FORWARD(1), SHOOTING(1), HALF(0.5), IDLE(0.3),
            REVERSE(-1), OFF(0), ANALOG_EXTAKE(0), AUTO_EXTAKE(-0.5);
            IntakeState(double v) { setValue(v); }
        }

        public enum VerticalState implements State {
            ON(1), HALF_DOWN(-0.5), OFF(0);
            VerticalState(double v) { setValue(v); }
        }

        public enum HorizontalFrontState implements State {
            OPEN(0.6), OPEN_SHOOT(0.475), STORED(0.06), MANUAL(-1);
            HorizontalFrontState(double v) { setValue(v); }
        }

        public enum HorizontalBackState implements State {
            OPEN(0.39), OPEN_SHOOT(0.37), STORED(0.68), STORED_SHOOT(0.66);
            HorizontalBackState(double v) { setValue(v); }
        }

        /** DECODE: set true when a sorted/burst shot finishes; the auto awaits it between cycles. */
        public boolean shotSorted = true;
        /** DECODE: ball-detection "magazine full" flag (LED feedback in Auto.primaryLoop). */
        public boolean intakeIsFull = false;

        public Magazine(HardwareMap hw) { /* no hardware */ }

        @Override protected void initStates() {
            setStates(IntakeState.OFF, VerticalState.OFF,
                    HorizontalFrontState.OPEN, HorizontalBackState.OPEN);
        }

        @Override protected void read() {}
        @Override protected void write() {}

        @Override public void stop() { setState(IntakeState.OFF, VerticalState.OFF); }

        private void setState(State... states) { for (State s : states) s.activate(); }
    }

    /** Hang/climb. DECODE-specific; unused by Close's run path but kept so the module set matches. */
    public static class Endgame extends Module {
        public enum FullLiftState implements State {
            INIT(0), OFF(0), FULL_LIFT(-14000);
            FullLiftState(double v) { setValue(v); }
        }

        public enum InitialState implements State {
            LIFT(700), DISABLED(700), ZERO(0), HOLD_BELLYPAN(-150);
            InitialState(double v) { setValue(v); }
        }

        public Endgame(HardwareMap hw) { /* no hardware */ }

        @Override protected void initStates() {
            setStates(FullLiftState.INIT, InitialState.HOLD_BELLYPAN);
        }

        @Override protected void read() {}
        @Override protected void write() {}
    }
}
