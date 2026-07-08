/*
 * Copyright (c) 2026 Vikram Janakiraman. All rights reserved.
 */
package org.firstinspires.ftc.teamcode.purepursuit.math

import android.graphics.Color
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.purepursuit.Robot
import org.firstinspires.ftc.teamcode.offseason.math.ParameterizedCircle
import org.firstinspires.ftc.teamcode.purepursuit.Drawing
import org.firstinspires.ftc.teamcode.purepursuit.MecanumDrivePurePursuit
import org.firstinspires.ftc.teamcode.purepursuit.pid.SquIDController
import java.lang.annotation.Inherited
import java.util.Collections
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

class PurePursuit(drive: MecanumDrivePurePursuit) {
    fun Pose2d.toPose(): Pose {
        return Pose(this.position.x, this.position.y, this.heading.toDouble())
    }

    companion object {
        @JvmField var builder: BezierBuilder = BezierBuilder()
        @JvmField var pathBuilder: BezierPathBuilder = BezierPathBuilder()

        @JvmField var drawPathsInBuild: Boolean = true
    }

    @JvmField var mecDrive = drive
    @JvmField var localizer = mecDrive.localizer!!

    @JvmField var pose = localizer.pose.toPose()

    @JvmField var targetPose = Pose()

    @JvmField var searchRad: Double = 8.0
    @JvmField var searchRadius = ParameterizedCircle(pose.vec(), searchRad)

    @JvmField var lastT = 0.0

    @JvmField var telemetryPacket = TelemetryPacket()

    @JvmField var maxPower = 0.65

    @JvmField var atEnd = false

    @JvmField var currentPath: Bezier? = null


    private var epsilon = 6e-5
    private var stepSize = 1e-5
    private var checkEpsilon = 1e-4
    private var sleepTime: Long = 50

    @JvmField var hasReachedDestination = false
    @JvmField var currentBezierPath: BezierPath? = null

    fun updateSearchRadius(radius: Double) {
        searchRad = radius
        localizer = mecDrive.localizer!!

        pose = localizer.pose.toPose()
        searchRadius = ParameterizedCircle(pose.vec(), radius)
    }
    fun updatePaths() {
        telemetryPacket.fieldOverlay().setStroke("#faaccd")
        Drawing.drawRobot(telemetryPacket.fieldOverlay(), pose.toPose2d(), this)
//        telemetryPacket.fieldOverlay().setStroke("#159470")
//        Drawing.drawRobot(telemetryPacket.fieldOverlay(), targetPose.toPose2d(), this)
    }

    object Defaults {
        val posTolerance = 1.0
        val hTolerance = 5.0.toRadians()
    }

    @JvmField var kSQx = 0.05
    @JvmField var kSQy = 0.05
    @JvmField var kF = 0.1

    var xSquid: SquIDController = SquIDController(kSQx, kF)
    var ySquid: SquIDController = SquIDController(kSQy, kF)
    @JvmField var hPID: PIDCoefficients = PIDCoefficients(1.5, 0.05,0.05)

    var hController = PIDController(hPID.p, hPID.i, hPID.d)

    @JvmField var targetList = Collections.emptyList<ParametricPose>()

    @JvmField var rotX = 0.0
    @JvmField var rotY = 0.0
    @JvmField var powH = 0.0
    @JvmField var tuning = false


    fun updatePose() {
        localizer = mecDrive.localizer!!
        pose = localizer.pose.toPose()
    }
    fun clip (value: Double, lowerBound: Double, upperBound: Double): Double {
        if (value > upperBound) return upperBound
        else if (value < lowerBound) return lowerBound
        return value
    }

    fun normalizeAngleError(target: Double, current: Double): Double {
        val error = (target - current + Math.PI) % (2 * Math.PI) - Math.PI
        return if (error < -Math.PI) error + (2 * Math.PI) else error
    }

    fun singlePIDtoPoint(pose: Pose) {
        xSquid.setSquID(kSQx, kF)
        ySquid.setSquID(kSQy, kF)
        hController.setPID(hPID.p, hPID.i, hPID.d)

        this.targetPose = pose

        val heading: Double = this.pose.h

        val x = xSquid.compute(pose.x - this.pose.x)
        val y = ySquid.compute(pose.y - this.pose.y)

        var rotX = x * cos(-heading) - y * sin(-heading)
        val rotY = x * sin(-heading) + y * cos(-heading)

        rotX *= 1.1

        val hError = AngleUnit.normalizeRadians(pose.h - heading)

        val h: Double = hController.calculate(0.0, hError)

        this.rotX = rotX
        this.rotY = rotY
        this.powH = h

        val magnitude = sqrt(rotX.pow(2) + rotY.pow(2))
        val scale = if (magnitude > maxPower) maxPower / magnitude else 1.0

        mecDrive.setDrivePowers(PoseVelocity2d(Vector2d(rotX * scale, rotY * scale), h))
    }

    fun within(currentPos: Double, target: Double, tolerance: Double) = (abs(target - currentPos) <= tolerance)
    fun withinHeading(current: Double, target: Double, tolerance: Double): Boolean {
        val error = AngleUnit.normalizeRadians(target - current)
        return abs(error) <= tolerance
    }
    fun notWithinTolerance(pose: Pose, target: Pose, xyTol: Double, hTol: Double): Boolean {
        return !within(pose.x, target.x, xyTol) || !within(pose.y, target.y, xyTol) || !withinHeading(pose.h, target.h, hTol)
    }


    //TODO: Pure-Pursuit Functions and Algorithms

    fun calcIntersections(curve: Bezier, searchRad: ParameterizedCircle): MutableList<ParametricPose> {

        fun tEquation(t: Double): Double {
            val pos = curve.solveParametric(t) ?: throw IllegalArgumentException(Bezier.WARNING + " Also make sure t is in range.")
            return (pos.x - searchRad.center.x).pow(2) + (pos.y - searchRad.center.y).pow(2) - searchRad.radius.pow(2)
        }

        fun tEquationDerivative(t: Double): Double {
            val pos = curve.solveParametric(t) ?: throw IllegalArgumentException(Bezier.WARNING + " Also make sure t is in range.")
            val derivative = curve.bezierCurveDerivative(t) ?: return 0.0
            return 2 * ((pos.x - searchRad.center.x) * derivative.x + (pos.y - searchRad.center.y) * derivative.y)
        }

        fun findSignChanges(): List<Double> {
            val step = 0.001
            val signChangePoints = mutableListOf<Double>()

            var prevT = 0.0
            var prevValue = tEquation(prevT)

            var t = step
            while (t <= 1.0) {
                val value = tEquation(t)

                if (prevValue * value <= 0) {
                    signChangePoints.add((prevT + t) / 2)
                }

                prevT = t
                prevValue = value
                t += step
            }
            return signChangePoints
        }

        fun newtonRaphson(guess: Double, maxIter: Int = 30): Double {
            var t = guess
            repeat(maxIter) {
                val derivative = tEquationDerivative(t)
                if (abs(derivative) < 1E-7) return t
                t -= tEquation(t) / derivative
                t = Maths.clamp(t, 0.0, 1.0)
            }
            return t
        }

        fun iteration(): List<ParametricPose> {
            val tS = findSignChanges().map { newtonRaphson(it) }
            return tS.mapNotNull { curve.solveParametric(it) }
        }

        val test = iteration().toMutableList()
        val result = mergeClosePoints(test, 0.02).toMutableList()
        return result
    }


    fun mergeClosePoints(points: List<ParametricPose>, threshold: Double): List<ParametricPose> {
        val merged = mutableListOf<ParametricPose>()

        for (point in points) {
            val close = merged.find { Maths.dist(it.vec(), point.vec()) < threshold }
            if (close == null) {
                merged.add(point)
            }
        }
        return merged
    }

    fun calculateTargetPose(path: Bezier): ParametricPose {
        if (currentPath !== path) {
            lastT = 0.0
            atEnd = false
            currentPath = path
            telemetryPacket.put("PATH_CHANGED", "New path detected")
            hController.reset()
        }

        targetList = calcIntersections(path, searchRadius)

        telemetryPacket.put("lastTAtStart", lastT)
        telemetryPacket.put("targetListSize", targetList.size)

        telemetryPacket.put("Intersection0T", targetList.getOrNull(0)?.t ?: -1.0)
        telemetryPacket.put("Intersection0X", targetList.getOrNull(0)?.x ?: -1.0)
        telemetryPacket.put("Intersection1T", targetList.getOrNull(1)?.t ?: -1.0)
        telemetryPacket.put("Intersection1X", targetList.getOrNull(1)?.x ?: -1.0)

        val robotPose = this.pose
        val distToEnd = Maths.dist(robotPose, path.end().toPose())

        // Candidate: nearest intersection ahead of lastT
        val candidate = targetList
            .filter { it.t >= lastT - 1e-3 }
            .maxByOrNull { it.t }

        val targetPose = when {
            targetList.isEmpty() -> {
                telemetryPacket.put("TARGET_FALLBACK", "No intersections found. Going to closest point.")
                val closestT = (0..100).map { it / 100.0 }
                    .minByOrNull { t ->
                        path.solveParametric(t)?.let { p ->
                            Maths.dist(p.vec(), robotPose.vec())
                        } ?: Double.MAX_VALUE
                    } ?: 0.0
                // Look ahead from closest point instead of targeting it directly
                val lookaheadT = minOf(closestT + 0.1, 1.0)  // 0.1 in t-space, tune this
                path.solveParametric(lookaheadT) ?: path.end()
            }
            distToEnd <= 2.0 -> {
                telemetryPacket.put("TARGET_FALLBACK", "Within 2.0 of end")
                path.end()
            }
            candidate != null -> {
                telemetryPacket.put("TARGET_FALLBACK", "None - valid candidate")
                candidate
            }
            else -> {
                telemetryPacket.put("TARGET_FALLBACK", "No valid candidate ahead of lastT")
                val closestT = (0..100).map { it / 100.0 }
                    .minByOrNull { t ->
                        path.solveParametric(t)?.let { p ->
                            Maths.dist(p.vec(), robotPose.vec())
                        } ?: Double.MAX_VALUE
                    } ?: 0.0
                val lookaheadT = minOf(closestT + 0.1, 1.0)
                path.solveParametric(lookaheadT) ?: path.end()
            }
        }

        // Adjust heading
        val adjustedPose = ParametricPose(
            targetPose.x,
            targetPose.y,
            targetPose.h,
            targetPose.t
        )

        // Update lastT only if we advanced along intersections
        if (targetList.isNotEmpty()) {
            lastT = maxOf(lastT, adjustedPose.t)
        }

        return adjustedPose
    }

    fun calculateTargetPose(path: BezierPath): ParametricPose {
        if (currentBezierPath !== path) {
            lastT = 0.0
            atEnd = false
            currentBezierPath = path
            hController.reset()
            telemetryPacket.put("PATH_CHANGED", "New BezierPath detected")
        }

        val targetList = path.beziers.flatMapIndexed { index, bezier ->
            calcIntersections(bezier, searchRadius).map { pose ->
                ParametricPose(pose.x, pose.y, pose.h, pose.t + index)  // shift t to global space
            }
        }.toMutableList()

        val robotPose = this.pose
        val distToEnd = Maths.dist(robotPose, path.end().toPose())

        val candidate = targetList
            .filter { it.t >= lastT - 1e-3 }
            .maxByOrNull { it.t }

        val targetPose = when {
            targetList.isEmpty() -> {
                telemetryPacket.put("TARGET_FALLBACK", "No intersections found. Going to closest point.")
                val closestGlobalT = (0..100 * path.beziers.size).map { it / 100.0 }
                    .minByOrNull { t ->
                        path.solve(t)?.let { p ->
                            Maths.dist(p.vec(), robotPose.vec())
                        } ?: Double.MAX_VALUE
                    } ?: 0.0
                val lookaheadT = minOf(closestGlobalT + 0.1, path.maxT())
                val curveIndex = minOf(floor(lookaheadT).toInt(), path.beziers.size - 1)
                val localT = lookaheadT - curveIndex
                path.beziers[curveIndex].solveParametric(localT)
                    ?.let { ParametricPose(it.x, it.y, it.h, lookaheadT) }
                    ?: path.end()
            }
            distToEnd <= 2.0 -> {
                telemetryPacket.put("TARGET_FALLBACK", "Within 2.0 of end")
                path.end()
            }
            candidate != null -> {
                telemetryPacket.put("TARGET_FALLBACK", "None - valid candidate")
                candidate
            }
            else -> {
                telemetryPacket.put("TARGET_FALLBACK", "No valid candidate ahead of lastT")
                val closestGlobalT = (0..100 * path.beziers.size).map { it / 100.0 }
                    .minByOrNull { t ->
                        path.solve(t)?.let { p ->
                            Maths.dist(p.vec(), robotPose.vec())
                        } ?: Double.MAX_VALUE
                    } ?: 0.0
                val lookaheadT = minOf(closestGlobalT + 0.1, path.maxT())
                val curveIndex = minOf(floor(lookaheadT).toInt(), path.beziers.size - 1)
                val localT = lookaheadT - curveIndex
                path.beziers[curveIndex].solveParametric(localT)
                    ?.let { ParametricPose(it.x, it.y, it.h, lookaheadT) }
                    ?: path.end()
            }
        }

        val adjustedPose = ParametricPose(targetPose.x, targetPose.y, targetPose.h, targetPose.t)

        if (targetList.isNotEmpty()) {
            lastT = maxOf(lastT, adjustedPose.t)
        }

        return adjustedPose
    }

    @JvmOverloads
    fun followPathSingle(path: Bezier, packet: TelemetryPacket, heading: Double? = null, reversed: Boolean? = null) {
        val vel = mecDrive.updatePoseEstimate().linearVel
        val speed = sqrt(vel.x.pow(2) + vel.y.pow(2))
        telemetryPacket = packet
        updateSearchRadius(searchRad)
//        val baseSearchRad = 8.0
//
//        val dynamicSearchRad = when {
//            speed > 35.0 -> baseSearchRad * 1.6   // Very fast: 12.8"
//            speed > 25.0 -> baseSearchRad * 1.4   // Fast: 11.2"
//            speed > 15.0 -> baseSearchRad * 1.2   // Medium: 9.6"
//            speed > 8.0 -> baseSearchRad           // Normal: 8.0"
//            speed > 3.0 -> baseSearchRad * 0.85   // Slow: 6.8"
//            else -> baseSearchRad * 0.7           // Very slow/settling: 5.6"
//        }
//
//        searchRad = dynamicSearchRad
//        searchRadius = ParameterizedCircle(pose.vec(), dynamicSearchRad)

        var targetPose = calculateTargetPose(path)
        path.draw(packet.fieldOverlay())
        drawRobot(packet.fieldOverlay(), pose.toPose2d())


        telemetryPacket.put("TargetX", targetPose.x)
        telemetryPacket.put("TargetY", targetPose.y)
        telemetryPacket.put("TargetH", targetPose.h)
        telemetryPacket.put("TargetT", targetPose.t)

        telemetryPacket.put("NumIntersections", targetList.size)
        telemetryPacket.put("LastT", lastT)
        telemetryPacket.put("CandidateT", targetPose.t)

        if (heading != null) {
            targetPose = ParametricPose(targetPose.x, targetPose.y, heading, targetPose.t)
        }
        if (reversed == true) {
            targetPose = ParametricPose(targetPose.x, targetPose.y, AngleUnit.normalizeRadians(targetPose.h + Math.PI), targetPose.t)
        }

        this.targetPose = targetPose.toPose()

        telemetryPacket.put("CurrentX", pose.x)
        telemetryPacket.put("CurrentY", pose.y)
        telemetryPacket.put("CurrentH", pose.h)

        telemetryPacket.put("RotX", rotX)
        telemetryPacket.put("RotY", rotY)
        telemetryPacket.put("H", powH)

        telemetryPacket.put("DistanceToTarget", Maths.dist(pose, targetPose.toPose()))
        telemetryPacket.put("SearchRadius", searchRad)

        singlePIDtoPoint(targetPose.toPose())
        updatePaths()
        // FIX: no send here. Actions.runBlocking() sends this packet after run() returns.

    }
    @JvmOverloads
    fun followPathSingle(path: BezierPath, packet: TelemetryPacket, heading: Double? = null, reversed: Boolean? = null) {
        if (!path.isFollowable()) return
        val vel = mecDrive.updatePoseEstimate().linearVel
        val speed = sqrt(vel.x.pow(2) + vel.y.pow(2))
        telemetryPacket = packet
        updateSearchRadius(searchRad)

        var targetPose = calculateTargetPose(path)
        path.beziers.forEach { it.draw(packet.fieldOverlay()) }
        drawRobot(packet.fieldOverlay(), pose.toPose2d())

        telemetryPacket.put("TargetX", targetPose.x)
        telemetryPacket.put("TargetY", targetPose.y)
        telemetryPacket.put("TargetH", targetPose.h)
        telemetryPacket.put("TargetT", targetPose.t)
        telemetryPacket.put("LastT", lastT)
        telemetryPacket.put("Speed", speed)

        if (heading != null) {
            targetPose = ParametricPose(targetPose.x, targetPose.y, heading, targetPose.t)
        }
        if (reversed == true) {
            targetPose = ParametricPose(targetPose.x, targetPose.y, AngleUnit.normalizeRadians(targetPose.h + Math.PI), targetPose.t)
        }

        this.targetPose = targetPose.toPose()

        telemetryPacket.put("CurrentX", pose.x)
        telemetryPacket.put("CurrentY", pose.y)
        telemetryPacket.put("CurrentH", pose.h)
        telemetryPacket.put("DistanceToTarget", Maths.dist(pose, targetPose.toPose()))
        telemetryPacket.put("SearchRadius", searchRad)

        singlePIDtoPoint(targetPose.toPose())
        updatePaths()
        // FIX: no send here. Actions.runBlocking() sends this packet after run() returns.
    }
    fun followPath(path: Bezier, xyTol: Double, hTol: Double, packet: TelemetryPacket) {
        // Reset state for a new path
        lastT = 0.0
        hasReachedDestination = false

        telemetryPacket = packet
        mecDrive.updatePoseEstimate() // FIX: refresh localizer before first target calc
        updateSearchRadius(searchRad)

        var targetPose = calculateTargetPose(path)

        packet.put("TargetX", targetPose.x)
        packet.put("TargetY", targetPose.y)
        packet.put("TargetH", targetPose.h)
        packet.put("TargetT", targetPose.t)

        while (!hasReachedDestination && notWithinTolerance(pose, targetPose.toPose(), xyTol, hTol)) {
            mecDrive.updatePoseEstimate() // FIX: without this, localizer.pose never changes and the loop is frozen
            updateSearchRadius(searchRad)

            telemetryPacket = TelemetryPacket() // FIX: one NEW packet per iteration = one dashboard frame

            targetPose = calculateTargetPose(path)

            this.targetPose = targetPose.toPose()

            singlePIDtoPoint(targetPose.toPose())

            path.draw(telemetryPacket.fieldOverlay()) // FIX: draw the path into every frame
            updatePaths() // FIX: draw robot (pink) + target (green) into every frame

            telemetryPacket.put("CurrentX", pose.x)
            telemetryPacket.put("CurrentY", pose.y)
            telemetryPacket.put("CurrentH", pose.h)

            telemetryPacket.put("DistanceToTarget", Maths.dist(pose, targetPose.toPose()))
            telemetryPacket.put("SearchRadius", searchRad)
            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket) // the ONLY send per iteration
        }


        hasReachedDestination = true
    }

    fun followPath(path: Bezier, packet: TelemetryPacket) = followPath(path, Defaults.posTolerance, Defaults.hTolerance, packet)

    fun followPath(path: BezierPath, xyTol: Double, hTol: Double, packet: TelemetryPacket) {
        if (!path.isFollowable()) return

        telemetryPacket = packet
        mecDrive.updatePoseEstimate() // FIX: refresh localizer before first target calc
        updateSearchRadius(searchRad)

        var targetPose = calculateTargetPose(path)

        while (!hasReachedDestination && notWithinTolerance(pose, targetPose.toPose(), xyTol, hTol)) {
            mecDrive.updatePoseEstimate() // FIX: without this, localizer.pose never changes and the loop is frozen
            updateSearchRadius(searchRad)

            telemetryPacket = TelemetryPacket() // FIX: one NEW packet per iteration = one dashboard frame

            targetPose = calculateTargetPose(path)

            val targetHeading = if (path.headingTargetsExist()) {
                path.headingTargets.filter { it.t > targetPose.t }
                    .minByOrNull { it.t }
                    ?.heading ?: targetPose.h
            } else {
                targetPose.h
            }

            targetPose = ParametricPose(targetPose, targetHeading)

            singlePIDtoPoint(targetPose.toPose())

            path.beziers.forEach { it.draw(telemetryPacket.fieldOverlay()) } // FIX: draw the path into every frame
            updatePaths() // FIX: draw robot (pink) + target (green) into every frame

            telemetryPacket.put("TargetX", targetPose.x)
            telemetryPacket.put("TargetY", targetPose.y)
            telemetryPacket.put("TargetH", targetPose.h)
            telemetryPacket.put("TargetT", targetPose.t)

            telemetryPacket.put("CurrentX", pose.x)
            telemetryPacket.put("CurrentY", pose.y)
            telemetryPacket.put("CurrentH", pose.h)

            telemetryPacket.put("DistanceToTarget", Maths.dist(pose, targetPose.toPose()))
            telemetryPacket.put("SearchRadius", searchRad)

            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket) // the ONLY send per iteration
        }
        hasReachedDestination = true
    }

    fun followPath(path: BezierPath, packet: TelemetryPacket) = followPath(path, Defaults.posTolerance, Defaults.hTolerance, packet)

    fun cancel() {
        hasReachedDestination = true
        mecDrive.setPowers()
    }
    fun drawRobot(c: Canvas, t: Pose2d) {
        val ROBOT_RADIUS = 9.0

        c.setStrokeWidth(1)
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS)

        val halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS)
        val p1 = t.position.plus(halfv)
        val p2 = p1.plus(halfv)
        c.strokeLine(p1.x, p1.y, p2.x, p2.y)
    }
}

private fun Double.toRadians(): Double {
    return Math.toRadians(this)
}

fun main() {
    val bezier = PurePursuit.builder
        .addControlPoint(60.4, 23.0)
        .addControlPoint(-61.0, -25.0)
        .addControlPoint(-50.3, -75.7)
        .addControlPoint(94.7, -62.0)
        .addControlPoint(-16.0, 43.7)
        .build()

//
//    println(PurePursuit.calcIntersections(bezier, PurePursuit.searchRadius))
//    println(PurePursuit.calculateTargetPose(bezier))

}