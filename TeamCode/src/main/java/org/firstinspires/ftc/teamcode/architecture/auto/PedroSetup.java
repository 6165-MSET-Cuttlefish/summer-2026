package org.firstinspires.ftc.teamcode.architecture.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Per-robot follower selector; {@link #activeRobot} must be set before the follower is built. */
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
