package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.architecture.core.AllianceColor;

/**
 * RED entry point for the ported "Close" auto. DECODE split alliance into separate ContextRed /
 * ContextBlue selector opmodes run before the auto; the summer-2026 fold folds the selection into a
 * thin {@link Close} subclass that pins {@link AllianceColor#RED} (via {@code alliance()}, read by
 * Close's instance initializer before its mirrored poses and {@code createRobot()} build) and runs
 * the full sequence. (Plain {@link Close} already defaults to RED — this is the explicit twin of
 * {@link CloseBlue}.)
 */
@Autonomous(name = "Close RED", group = "A")
public class CloseRed extends Close {
    @Override
    protected AllianceColor alliance() {
        return AllianceColor.RED;
    }
}
