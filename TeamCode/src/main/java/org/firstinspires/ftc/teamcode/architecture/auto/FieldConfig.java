package org.firstinspires.ftc.teamcode.architecture.auto;

/**
 * Field-level constants that are independent of the path follower / Pedro setup. Pedro-specific
 * tuning (drive constants, follower constants, localizer constants, path constraints) lives in
 * {@link Constants}; physical-field dimensions and similar season-portable values belong here.
 */
public final class FieldConfig {
    private FieldConfig() {}

    /** Playable field width in inches. FTC fields are 141.5" (~12 ft) on a side; override per season if needed. */
    public static double fieldWidthInches = 141.5;
}
