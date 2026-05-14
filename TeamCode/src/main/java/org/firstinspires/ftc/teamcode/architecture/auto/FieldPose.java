package org.firstinspires.ftc.teamcode.architecture.auto;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.core.AllianceColor;
import org.firstinspires.ftc.teamcode.core.Context;

/**
 * Pose helpers tied to {@link Context#allianceColor}. Path geometry can be authored once for
 * the RED side and these helpers will mirror to BLUE when the alliance switches.
 */
public final class FieldPose {
    private FieldPose() {}

    /**
     * Mirror a RED-side pose to BLUE if {@link Context#allianceColor} is BLUE. Uses Pedro's
     * built-in {@link Pose#mirror(double)} (line 313) so the math stays in sync with Pedro's
     * coordinate-system semantics.
     */
    public static Pose colorPose(double x, double y, double heading) {
        return colorPose(x, y, heading, FieldConfig.fieldWidthInches);
    }

    /** Same as {@link #colorPose(double, double, double)} with an explicit field width. */
    public static Pose colorPose(double x, double y, double heading, double fieldWidth) {
        Pose red = new Pose(x, y, heading);
        return Context.allianceColor == AllianceColor.BLUE ? red.mirror(fieldWidth) : red;
    }
}
