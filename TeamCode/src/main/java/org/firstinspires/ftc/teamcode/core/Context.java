package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;

/**
 * Game-agnostic cross-opmode scratch. Only alliance side lives here; game-specific shared
 * state goes in a game-specific class.
 */
@Config
public final class Context {
    private Context() {}

    public static AllianceColor allianceColor = AllianceColor.RED;
}
