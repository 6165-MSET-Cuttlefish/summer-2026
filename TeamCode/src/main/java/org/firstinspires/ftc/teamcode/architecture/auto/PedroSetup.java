package org.firstinspires.ftc.teamcode.architecture.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Per-robot follower selector. The framework {@link org.firstinspires.ftc.teamcode.architecture.core.Robot}
 * builds its {@link Follower} through {@link #createFollower(HardwareMap)}, which dispatches to the
 * active robot's tuning + wiring:
 *
 * <ul>
 *   <li><b>betta bot</b> (summer testbed) — {@link BettaPedroSetup} + {@link BettaHardwareConfig}</li>
 *   <li><b>DECODE robot</b> — {@link DecodePedroSetup} + {@link DecodeHardwareConfig}</li>
 * </ul>
 *
 * <p>{@link #activeRobot} is {@code @Config}-tunable (dashboard dropdown) and defaults to
 * {@link RobotType#BETTA}. DECODE OpModes pin it to {@link RobotType#DECODE} in
 * {@code DecodeOpMode.createRobot()} before the follower is built, so running a DECODE OpMode always
 * uses DECODE tuning regardless of the dashboard value.
 */
@Config
public final class PedroSetup {
    private PedroSetup() {}

    public enum RobotType { BETTA, DECODE }

    public static RobotType activeRobot = RobotType.DECODE;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return activeRobot == RobotType.DECODE
                ? DecodePedroSetup.createFollower(hardwareMap)
                : BettaPedroSetup.createFollower(hardwareMap);
    }
}
