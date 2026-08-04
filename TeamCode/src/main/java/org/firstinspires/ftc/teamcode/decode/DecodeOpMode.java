package org.firstinspires.ftc.teamcode.decode;

import org.firstinspires.ftc.teamcode.architecture.auto.PedroSetup;
import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

/** Base for OpModes driving the {@link DecodeRobot}; adds a typed {@code robot} so game code needn't cast. */
public abstract class DecodeOpMode extends EnhancedOpMode {

    protected DecodeRobot robot;

    @Override
    protected final Robot createRobot() throws InterruptedException {
        // Must precede the Robot ctor, which builds the follower from PedroSetup.activeRobot.
        PedroSetup.activeRobot = PedroSetup.RobotType.DECODE;
        DecodeRobot r = new DecodeRobot(this);
        this.robot = r; // shadows EnhancedOpMode.robot with a typed reference to the same object
        return r;
    }

    @Override
    protected void onLoopStart() {
        if (robot != null) {
            robot.updateWriteToggles();
            DecodeContext.updateSharedPose(robot);
        }
    }
}
