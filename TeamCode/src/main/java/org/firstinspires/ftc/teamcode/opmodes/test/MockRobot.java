package org.firstinspires.ftc.teamcode.opmodes.test;

import org.firstinspires.ftc.teamcode.core.EnhancedOpMode;
import org.firstinspires.ftc.teamcode.core.Robot;

/** Minimal Robot for the architecture smoke test: one hardware-free module, plus the Pedro follower. */
public class MockRobot extends Robot {
    public MockMechanism mech;

    public MockRobot(EnhancedOpMode opMode) throws InterruptedException {
        // false = don't carry pose from a prior run; start from PedroSetup's default pose so the
        // test is deterministic.
        super(opMode, false);
    }

    @Override
    protected void initializeGameModules() {
        // Assigned here (not via a field initializer): this runs inside the Robot super-ctor, and a
        // subclass field initializer would overwrite the assignment afterwards. Module auto-discovery
        // picks `mech` up after createRobot() returns.
        mech = new MockMechanism(opMode.hardwareMap);
    }
}
