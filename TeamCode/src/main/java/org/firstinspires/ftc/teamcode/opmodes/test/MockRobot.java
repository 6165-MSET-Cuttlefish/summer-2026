package org.firstinspires.ftc.teamcode.opmodes.test;

import org.firstinspires.ftc.teamcode.architecture.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.architecture.core.Robot;

public class MockRobot extends Robot {
    public MockMechanism mech;

    public MockRobot(EnhancedOpMode opMode) throws InterruptedException {
        super(opMode);
    }

    @Override
    protected void initializeGameModules() {
        // Not a field initializer: this runs inside the Robot super-ctor, so one would overwrite it.
        mech = new MockMechanism(opMode.hardwareMap);
    }
}
