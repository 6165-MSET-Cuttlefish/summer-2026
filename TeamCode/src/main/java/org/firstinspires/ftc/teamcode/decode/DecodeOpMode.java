package org.firstinspires.ftc.teamcode.decode;

import org.firstinspires.ftc.teamcode.architecture.auto.PedroSetup;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

/**
 * Game-specific intermediate base for any OpMode that drives the {@link DecodeRobot}. Bridges
 * the framework {@link EnhancedOpMode} (which only knows about the abstract {@link Robot}) and
 * the typed {@link DecodeRobot} reference that game code wants to read.
 *
 * <p>Concrete OpModes (Tele, Auto subclasses, test opmodes) extend this class so {@code robot.shooter}
 * etc. resolve without casting.
 */
public abstract class DecodeOpMode extends EnhancedOpMode {

    /**
     * Game-typed reference to the same instance as {@link EnhancedOpMode#robot}. Set inside
     * {@link #createRobot()} so this field is populated before lifecycle hooks run.
     */
    protected DecodeRobot robot;

    @Override
    protected final Robot createRobot() throws InterruptedException {
        // Pin the DECODE robot's follower tuning + hardware wiring before the Robot ctor builds the
        // follower, so every DECODE OpMode uses DECODE config regardless of the dashboard default.
        PedroSetup.activeRobot = PedroSetup.RobotType.DECODE;
        DecodeRobot r = new DecodeRobot(this);
        this.robot = r; // shadow EnhancedOpMode.robot with a typed reference to the same object
        return r;
    }

    @Override
    protected void onLoopStart() {
        if (robot != null) {
            robot.updateWriteToggles();
            // Decode-specific shared pose math (turret aim, shot-on-the-move corrections).
            DecodeContext.updateSharedPose(robot);
        }
    }
}
