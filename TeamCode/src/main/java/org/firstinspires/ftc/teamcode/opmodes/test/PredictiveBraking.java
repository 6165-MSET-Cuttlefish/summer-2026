package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;

/**
 * Scales a requested drive power down as the robot nears an obstacle, so holding full speed into
 * a wall decelerates smoothly and settles at {@link Tuning#stopDistanceCm} instead of colliding.
 *
 * Sensor-agnostic on purpose — which distance sensor we end up on isn't decided yet, so this takes
 * a {@link DistanceReader} lambda instead of a concrete sensor type. See
 * {@link PredictiveBrakingUltrasonicTest} and {@link PredictiveBrakingLaserTest} for the two
 * sensors currently on the bench.
 *
 * The braking law between the two thresholds is
 * <pre>
 *   t       = (distance - stopDistanceCm) / (brakeStartCm - stopDistanceCm)   // 1 far, 0 at stop
 *   ceiling = sqrt(t) * brakingMultiplier                                     // clamped to [minCreepPower, 1]
 *   power   = min(requestedPower, ceiling)
 * </pre>
 * The square root is what constant deceleration looks like: stopping distance goes with the square
 * of speed, so the safe speed goes with the square root of the distance left. A linear ramp instead
 * brakes far too gently at the start and far too hard at the end.
 *
 * This only ever pulls power down. It never adds power, and it never touches a negative (backing
 * away) request, so the driver can always reverse out of a corner.
 */
public class PredictiveBraking {

    public interface DistanceReader {
        /** Latest distance reading, in centimeters. Return {@link Double#NaN} for an invalid read. */
        double getDistanceCm();
    }

    @Config
    public static class Tuning {
        /** Standoff the robot should settle at. Power is hard-zeroed at or inside this. */
        public static double stopDistanceCm = 20;
        /** Threshold: nothing happens until the robot is this close. Farther out, full power. */
        public static double brakeStartCm = 120;
        /**
         * Aggressiveness. 1.0 means "reach zero right at stopDistanceCm". Below 1.0 brakes harder
         * and earlier (stops short, wastes approach time); above 1.0 stays fast longer and brakes
         * later (faster cycle, overshoots if pushed too far). Tune this until the robot settles on
         * stopDistanceCm without a bounce.
         */
        public static double brakingMultiplier = 1.0;
        /** Power floor inside the braking zone — below this the drivetrain stalls instead of creeping in. */
        public static double minCreepPower = 0.12;
        /** Readings below this are sensor noise, not "on top of the wall." */
        public static double minValidCm = 2;
        /** Readings at or beyond this are out of the sensor's useful range — treated as no obstacle. */
        public static double maxValidCm = 400;
        public static boolean enabled = true;
    }

    private final DistanceReader reader;
    private double lastDistanceCm = Tuning.maxValidCm;

    public PredictiveBraking(DistanceReader reader) {
        this.reader = reader;
    }

    /** Poll the sensor. Call once per loop, before {@link #clampApproachPower}. */
    public void read() {
        double d = reader.getDistanceCm();
        if (!Double.isNaN(d) && d >= Tuning.minValidCm) {
            lastDistanceCm = d;
        }
        // A bad single sample (NaN or under minValidCm) keeps the last good distance rather than
        // snapping to 0, so one flaky read can't slam the brakes on for a loop.
    }

    public double getDistanceCm() {
        return lastDistanceCm;
    }

    /** True if the robot is inside the braking threshold and this is actively limiting power. */
    public boolean isBraking() {
        return Tuning.enabled && lastDistanceCm < Tuning.brakeStartCm;
    }

    /** The power ceiling this would impose right now, ignoring what the driver asked for. */
    public double getPowerCeiling() {
        return clampApproachPower(1.0);
    }

    /**
     * Clamps a requested power toward this sensor's facing direction (positive = approaching).
     * Negative power (driving away) passes through unchanged.
     */
    public double clampApproachPower(double requestedPower) {
        if (!Tuning.enabled || requestedPower <= 0 || lastDistanceCm >= Tuning.maxValidCm) {
            return requestedPower;
        }
        if (lastDistanceCm <= Tuning.stopDistanceCm) {
            return 0;
        }
        if (lastDistanceCm >= Tuning.brakeStartCm) {
            return requestedPower;
        }
        // Both guards above have passed, so stopDistanceCm < lastDistanceCm < brakeStartCm and the
        // denominator is strictly positive — no divide-by-zero even if the thresholds get crossed
        // on the dashboard.
        double t = (lastDistanceCm - Tuning.stopDistanceCm) / (Tuning.brakeStartCm - Tuning.stopDistanceCm);
        double ceiling = Math.sqrt(t) * Tuning.brakingMultiplier;
        ceiling = Math.max(Tuning.minCreepPower, Math.min(1.0, ceiling));

        return Math.min(requestedPower, ceiling);
    }
}
