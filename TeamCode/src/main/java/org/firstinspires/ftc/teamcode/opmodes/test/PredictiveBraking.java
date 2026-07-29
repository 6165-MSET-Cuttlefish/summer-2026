package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;

/**
 * Scales a requested drive power down as the robot nears an obstacle, so holding full speed at a
 * wall decelerates smoothly and settles on {@link Tuning#stopDistanceCm} instead of colliding.
 *
 * Sensor-agnostic on purpose — which distance sensor we end up on isn't decided yet, so this takes
 * a {@link DistanceReader} lambda instead of a concrete sensor type. See
 * {@link PredictiveBrakingUltrasonicTest} and {@link PredictiveBrakingLaserTest}.
 *
 * <h2>Why "predictive"</h2>
 * The distance reading is always stale — the MB1242 only produces a new sample every ~100 ms, and
 * at 150 cm/s the robot covers 15 cm in that time. Braking on the raw reading therefore always
 * brakes late by roughly one sample period, which no amount of curve tuning can fix. So this
 * estimates closing speed by differentiating successive readings and brakes on where the robot
 * <em>will</em> be {@link Tuning#lookaheadSeconds} from now, not where it was when the sensor last
 * spoke. That is what lets it stop on the target distance rather than past it.
 *
 * <h2>The braking curve</h2>
 * Full requested power all the way in to {@link Tuning#fullPowerAboveCm}. Inside that, power decays
 * exponentially from {@link Tuning#powerAtThreshold} down to zero at {@link Tuning#stopDistanceCm}:
 * <pre>
 *   t       = (predicted - stopDistanceCm) / (fullPowerAboveCm - stopDistanceCm)  // 1 at threshold, 0 at stop
 *   shape   = (exp(decelRate * t) - 1) / (exp(decelRate) - 1)                     // 1 at threshold, 0 at stop
 *   ceiling = minCreepPower + (powerAtThreshold - minCreepPower) * shape
 * </pre>
 * {@link Tuning#decelRate} is the only shape knob:
 * <ul>
 *   <li>→0 — straight linear ramp from powerAtThreshold to zero.</li>
 *   <li>2 — power falls off quickly just inside the threshold, then eases into a slower approach.</li>
 *   <li>5+ — power collapses almost immediately at the threshold, then a long crawl in.</li>
 * </ul>
 * So raise decelRate to shed speed harder and sooner after the threshold, lower it toward zero for
 * an even ramp. Where braking <em>begins</em> is fullPowerAboveCm; decelRate only controls the
 * shape between there and the stop.
 *
 * This only ever pulls power down. It never adds power and never touches a negative (backing away)
 * request, so the driver can always reverse out of a corner.
 */
public class PredictiveBraking {

    public interface DistanceReader {
        /** Latest distance reading, in centimeters. Return {@link Double#NaN} for an invalid read. */
        double getDistanceCm();
    }

    // Slothboard registers @Config classes by getSimpleName(), which for a nested class is just
    // "Tuning" — every nested Tuning in the project would collide on one key and all but one would
    // vanish from the dashboard. Nested @Config classes must carry an explicit unique name.
    @Config("Braking Law")
    public static class Tuning {
        /** Standoff the robot should settle on. Power is hard-zeroed at or inside this. */
        public static double stopDistanceCm = 25;
        /** Full requested power at any distance beyond this. Braking only happens inside it. */
        public static double fullPowerAboveCm = 80;
        /** Power the instant the robot crosses fullPowerAboveCm — the top of the decay curve. */
        public static double powerAtThreshold = 0.9;
        /** Exponential steepness inside the threshold. Higher sheds speed sooner. See class javadoc. */
        public static double decelRate = 2.0;
        /**
         * How far ahead to project the robot's motion, covering sensor sample period plus the
         * loop's own reaction time. Too low and it stops long; too high and it stops short.
         */
        public static double lookaheadSeconds = 0.15;
        /** Low-pass on the closing-speed estimate, 0-1. Lower is smoother but laggier. */
        public static double velocityFilter = 0.4;
        /** Power floor inside the braking zone — below this the drivetrain stalls instead of creeping in. */
        public static double minCreepPower = 0.10;
        /** Once stopped, stay stopped, instead of creeping and re-braking around the target. */
        public static boolean latchOnStop = true;
        /** Readings below this are sensor noise, not "on top of the wall." */
        public static double minValidCm = 2;
        /** Readings at or beyond this are out of the sensor's useful range — treated as no obstacle. */
        public static double maxValidCm = 400;
        public static boolean enabled = true;
    }

    private final DistanceReader reader;

    private double lastDistanceCm = Tuning.maxValidCm;
    private double closingVelocityCmPerSec = 0;
    private boolean stopped = false;

    private double lastSampleCm = Double.NaN;
    private long lastSampleNanos = 0;

    public PredictiveBraking(DistanceReader reader) {
        this.reader = reader;
    }

    /** Poll the sensor. Call once per loop, before {@link #clampApproachPower}. */
    public void read() {
        double d = reader.getDistanceCm();
        if (Double.isNaN(d) || d < Tuning.minValidCm) {
            // A bad single sample keeps the last good distance rather than snapping to 0, so one
            // flaky read can't slam the brakes on for a loop.
            return;
        }

        // Only differentiate when the value actually moved. A ping/wait/read sensor republishes the
        // same number every loop between pings, so differentiating per-loop would read as "velocity
        // zero" most loops and then spike on the loop the sample lands.
        if (!Double.isNaN(lastSampleCm) && d != lastSampleCm) {
            double dtSec = (System.nanoTime() - lastSampleNanos) / 1e9;
            if (dtSec > 0) {
                double instantaneous = (lastSampleCm - d) / dtSec;
                closingVelocityCmPerSec += Tuning.velocityFilter * (instantaneous - closingVelocityCmPerSec);
            }
        }
        if (Double.isNaN(lastSampleCm) || d != lastSampleCm) {
            lastSampleCm = d;
            lastSampleNanos = System.nanoTime();
        }

        lastDistanceCm = d;
    }

    /** Raw latest distance, in cm. */
    public double getDistanceCm() {
        return lastDistanceCm;
    }

    /** Estimated closing speed in cm/s. Positive means approaching the obstacle. */
    public double getClosingVelocityCmPerSec() {
        return closingVelocityCmPerSec;
    }

    /** Where the robot is projected to be {@link Tuning#lookaheadSeconds} from now, in cm. */
    public double getPredictedDistanceCm() {
        return Math.max(0, lastDistanceCm - closingVelocityCmPerSec * Tuning.lookaheadSeconds);
    }

    public boolean isBraking() {
        return Tuning.enabled && getPredictedDistanceCm() < Tuning.fullPowerAboveCm;
    }

    /** True once the stop distance has been reached and {@link Tuning#latchOnStop} is holding it there. */
    public boolean isStopped() {
        return stopped;
    }

    /** Release the stop latch so the robot may approach again. */
    public void resetStop() {
        stopped = false;
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

        double predicted = getPredictedDistanceCm();

        if (predicted <= Tuning.stopDistanceCm) {
            stopped = Tuning.latchOnStop;
            return 0;
        }
        if (stopped) {
            return 0;
        }
        if (predicted >= Tuning.fullPowerAboveCm) {
            return requestedPower;
        }

        // Both guards above have passed, so stopDistanceCm < predicted < fullPowerAboveCm and the
        // denominator is strictly positive — no divide-by-zero even if the thresholds get crossed
        // on the dashboard.
        double t = (predicted - Tuning.stopDistanceCm) / (Tuning.fullPowerAboveCm - Tuning.stopDistanceCm);
        double ceiling = Tuning.minCreepPower + (Tuning.powerAtThreshold - Tuning.minCreepPower) * decayShape(t);
        ceiling = Math.max(Tuning.minCreepPower, Math.min(1.0, ceiling));

        return Math.min(requestedPower, ceiling);
    }

    /** Maps t (1 at the braking threshold, 0 at the stop distance) onto the same 1..0 range. */
    private static double decayShape(double t) {
        double k = Tuning.decelRate;
        if (Math.abs(k) < 1e-6) {
            return t;
        }
        return (Math.exp(k * t) - 1) / (Math.exp(k) - 1);
    }
}
