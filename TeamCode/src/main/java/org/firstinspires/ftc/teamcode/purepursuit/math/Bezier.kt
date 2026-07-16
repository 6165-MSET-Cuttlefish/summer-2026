/*
 * Copyright (c) 2026 Vikram Janakiraman. All rights reserved.
 */

package org.firstinspires.ftc.teamcode.purepursuit.math

import com.acmerobotics.dashboard.canvas.Canvas
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt


/**
 * Kotlin data class that represents a single [Bezier] curve, defined with a list of [Vector] control points.
 * The curve is parametric in nature, so it can be iterated through with a t-value from 0-1.
 */
@Parameterized
data class Bezier(val controlPoints: MutableList<Vector>): ParameterizedObject, PathComponent, Path {
    private var type: BezierType? = null
    var headingTargets: MutableList<ParametricHeading> = emptyList<ParametricHeading>().toMutableList()
        get() {
            return field
        }

    var searchRadTargets: MutableList<SearchRadiusTarget> = emptyList<SearchRadiusTarget>().toMutableList()
        get() {
            return field
        }

    companion object {
        const val WARNING: String = """Don't be stupid. N-point curves accepted, but don't do less than 2."""

        fun Vector.magnitude(): Double {
            return sqrt(x.pow(2)+ y.pow(2))
        }
        fun Int.factorial(): Long {
            require(this >= 0) { "Factorial is not defined for negative numbers" }
            var product = 1L
            for (num in 1..this) {
                product *= num
            }
            val finalResult = if (product >= 0) product else throw IllegalArgumentException("Your input was too large, and the variable overflowed.")
            return finalResult
        }

        fun Int.nCr(other: Int): Long {
            require(other >= 0) { "r must be non-negative" }
            require(other <= this) { "r cannot be greater than n" }
            val r = minOf(other, this - other)
            var result = 1L
            for (i in 1..r) {
                result = result * (this - i + 1) / i
            }
            val finalResult = if (result >= 0) result else throw IllegalArgumentException("Your input was too large, and the variable overflowed.")
            return finalResult
        }
    }

    constructor(points: MutableList<Vector>, headings: MutableList<ParametricHeading>): this(points) {
        headingTargets = headings
    }
    constructor(vararg points: Vector) : this(points.toMutableList()) {
        type = when (controlPoints.size) {
            2 -> BezierType.GENERAL
            4 -> BezierType.FOUR_POINT
            else -> BezierType.GENERAL
        }
    }

    init {
        type = when (controlPoints.size) {
            2 -> BezierType.GENERAL
            4 -> BezierType.FOUR_POINT
            else -> BezierType.GENERAL
        }
    }
    fun headingTargetsExist(): Boolean = headingTargets.size != 0

    /**
     * Function to solve for the point that a defined [Bezier] curve passes through at a value of a parameter t,
     * ranging from 0 to 1.
     */
    @Override
    override fun solve(t: Double): Pose? {
        if (t in 0.0..1.0) {
            if (type == BezierType.TWO_POINT) {
                val two = controlPoints[1]
                val one = controlPoints[0]

                val deltaX = two.x - one.x
                val deltaY = two.y - one.y
                val x = t * deltaX + one.x
                val y = t * deltaY + one.y
                val h = atan2(deltaY, deltaX)

                return Pose(x,y,h)
            }
            if (type == BezierType.FOUR_POINT) {
                val four = controlPoints[3]
                val three = controlPoints[2]
                val two = controlPoints[1]
                val one = controlPoints[0]

                val x =
                    (1 - t).pow(3) * one.x + 3 * (1 - t).pow(2) * t * two.x + 3 * (1 - t) * t.pow(2) * three.x + t.pow(
                        3
                    ) * four.x
                val y =
                    (1 - t).pow(3) * one.y + 3 * (1 - t).pow(2) * t * two.y + 3 * (1 - t) * t.pow(2) * three.y + t.pow(
                        3
                    ) * four.y

                val h = Math.toRadians(angleOfCurve(t))
                return Pose(x, y, h)
            }
            if (type == BezierType.GENERAL) {
                var x = 0.0
                var y = 0.0
                val nPts = controlPoints.size - 1
                for (n in 0..nPts) {
                    val coeff = nPts.nCr(n) * ((1 - t).pow(nPts - n)) * (t.pow(n))
                    x += coeff * controlPoints[n].x
                    y += coeff * controlPoints[n].y
                }
                val h = Math.toRadians(angleOfCurve(t))
                return Pose(x, y, h)
            }
        }
//        P = (1−t)2P1 + 2(1−t)tP2 + t2P3
//        P = (1−t)3P1 + 3(1−t)2tP2 +3(1−t)t2P3 + t3P4
        return null
    }
    fun solveParametric(t: Double): ParametricPose? {
        if (t in 0.0..1.0) {
            if (type == BezierType.TWO_POINT) {
                val two = controlPoints[1]
                val one = controlPoints[0]

                val deltaX = two.x - one.x
                val deltaY = two.y - one.y
                val x = t * deltaX + one.x
                val y = t * deltaY + one.y
                val h = atan2(deltaY, deltaX)

                return ParametricPose(x,y,h,t)
            }
            if (type == BezierType.FOUR_POINT) {
                val four = controlPoints[3]
                val three = controlPoints[2]
                val two = controlPoints[1]
                val one = controlPoints[0]

                val x =
                    (1 - t).pow(3) * one.x + 3 * (1 - t).pow(2) * t * two.x + 3 * (1 - t) * t.pow(2) * three.x + t.pow(
                        3
                    ) * four.x
                val y =
                    (1 - t).pow(3) * one.y + 3 * (1 - t).pow(2) * t * two.y + 3 * (1 - t) * t.pow(2) * three.y + t.pow(
                        3
                    ) * four.y

                val h = Math.toRadians(angleOfCurve(t))
                return ParametricPose(x, y, h, t)
            }
            if (type == BezierType.GENERAL) {
                var x = 0.0
                var y = 0.0
                val nPts = controlPoints.size - 1
                for (n in 0..nPts) {
                    val coeff = nPts.nCr(n) * ((1 - t).pow(nPts - n)) * (t.pow(n))
                    x += coeff * controlPoints[n].x
                    y += coeff * controlPoints[n].y
                }
                val h = Math.toRadians(angleOfCurve(t))
                return ParametricPose(x, y, h, t)
            }
        }
//        P = (1−t)2P1 + 2(1−t)tP2 + t2P3
//        P = (1−t)3P1 + 3(1−t)2tP2 +3(1−t)t2P3 + t3P4
        return null
    }


    /**
     * Function to calculate the tangent vector, or first derivative, of a [Bezier] curve.
     */
    fun bezierCurveDerivative(t: Double): Vector? {
        if (t in 0.0..1.0) {
            if (type == BezierType.TWO_POINT) {
                val deltaX = controlPoints[1].x - controlPoints[0].x
                val deltaY = controlPoints[1].y - controlPoints[0].y
                return Vector(deltaX, deltaY)
            }
            else if (type == BezierType.FOUR_POINT) {
                val oneMinusT = 1 - t
                val oneMinusTSquared = oneMinusT.pow(2)
                val tSquared = t.pow(2)
                val four = controlPoints[3]
                val three = controlPoints[2]
                val two = controlPoints[1]
                val one = controlPoints[0]
                val x =
                    -3 * oneMinusTSquared * one.x + (3 * oneMinusTSquared - 6 * oneMinusT * t) * two.x + (6 * oneMinusT * t - 3 * tSquared) * three.x + 3 * tSquared * four.x
                val y =
                    -3 * oneMinusTSquared * one.y + (3 * oneMinusTSquared - 6 * oneMinusT * t) * two.y + (6 * oneMinusT * t - 3 * tSquared) * three.y + 3 * tSquared * four.y
                return Vector(x, y)
            }
            else if (type == BezierType.GENERAL) {
                val n = controlPoints.size - 1
                var dx = 0.0
                var dy = 0.0
                for (i in 0 until n) {
                    val coeff = n * ( (n-1).nCr(i) * (1-t).pow((n-1)-i) * t.pow(i) )
                    dx += coeff * (controlPoints[i+1].x - controlPoints[i].x)
                    dy += coeff * (controlPoints[i+1].y - controlPoints[i].y)
                }
                return Vector(dx, dy)
            }
        }
        return null
    }

    override fun isFollowable(): Boolean = true

    /**
     * Iterative method to approximate the length of a [Bezier] curve by adding up the lengths of small segments along it
     */
    override fun length(): Double {
        val step = 0.001
        var t: Double = step
        var totalLength = 0.0
        while (t <= 1.0) {
            if (t != 0.0) {
                totalLength += Maths.dist(solve(t - step), solve(t))
            }
            t += step
        }
        return totalLength
    }

    /**
     * Function to calculate the second derivative of a [Bezier] curve
     */
    fun bezierCurveSecondDerivative(t: Double): Vector? {
        if (t in 0.0..1.0) {
            if (type == BezierType.TWO_POINT) {
                return bezierCurveDerivative(t)!!
            }
            else if (type == BezierType.FOUR_POINT) {
                val oneMinusT = 1 - t
                val four = controlPoints[3]
                val three = controlPoints[2]
                val two = controlPoints[1]
                val one = controlPoints[0]
                val x =
                    6 * oneMinusT * (one.x - 2 * two.x + three.x) + 6 * t * (two.x - 2 * three.x + four.x)
                val y =
                    6 * oneMinusT * (one.y - 2 * two.y + three.y) + 6 * t * (two.y - 2 * three.y + four.y)
                return Vector(x, y)
            }
            else if (type == BezierType.GENERAL) {
                val n = controlPoints.size - 1
                if (n < 2) return Vector(0.0, 0.0) // second derivative undefined for < quadratic

                var dx = 0.0
                var dy = 0.0
                for (i in 0 until n-1) {
                    val coeff = n * (n-1) *
                            ((n-2).nCr(i) * (1 - t).pow((n-2) - i) * t.pow(i))
                    val deltaX = controlPoints[i+2].x - 2*controlPoints[i+1].x + controlPoints[i].x
                    val deltaY = controlPoints[i+2].y - 2*controlPoints[i+1].y + controlPoints[i].y
                    dx += coeff * deltaX
                    dy += coeff * deltaY
                }
                return Vector(dx, dy)
            }
        }
        return null
    }

    /**
     * Function to determine whether a point is sufficiently close to, or "on" a [Bezier] curve
     */
    fun isOn(pose: Pose): Boolean {
        val epsilon = 1e-3
        val step = 1e-3
        var t = 0.0
        while (t <= 1.0) {
            if (abs(findT(pose) - t) < epsilon) {
                if (pose == start().toPose()) {
                    return true
                }
                if (pose == end().toPose()) {
                    return true
                }
                if (t != 0.0) {
                    return true
                }
            }
            t += step
        }
        return false
    }

    /**
     * Function to find the parameter, t, that can be applied to a [Bezier] curve to result in the input point.
     * Only will give accurate answers if point has been run through above [isOn] function first.
     */
    fun findT(pose: Pose): Double {
        val epsilon = 4e-3
        val step = 1e-3
        var t = 0.0
        var test = 0.0
        if (pose == start().toPose()) return 0.0
        else if (pose == end().toPose()) return 1.0
        while (test <= 1.0) {
            if (Maths.dist(solve(test), pose) <= epsilon) {
                t = test
            }
            test += step
        }
        return t
    }
    fun findT(pose: ParametricPose): Double {
        return pose.t
    }

    /**
     * Function that uses the curvature formula to find a curvature value at any point along the [Bezier] curve,
     * accessed through [t]
     */
    fun curvature(t: Double): Double {
        val dxdt = bezierCurveDerivative(t)!!.x
        val dydt = bezierCurveDerivative(t)!!.y
        val d2xdt2 = bezierCurveSecondDerivative(t)!!.x
        val d2ydt2 = bezierCurveSecondDerivative(t)!!.y

        val numerator = abs(dxdt * d2ydt2 - dydt * d2xdt2)
        val denominator = sqrt(dxdt.pow(2) + dydt.pow(2)).pow(3)

        return numerator / denominator
    }

    /**
     * Function that inverts the curvature at a point to find the radius of the [Bezier] curves tangent circle
     * at a specific point along the curve
     */
    fun radius(t: Double): Double {
        return 1 / (curvature(t))
    }

    /**
     * Finds ratio of the components of the tangent [Vector] of the [Bezier] curve, thus finding it's "slope", if it were a line
     */
    fun slope(t: Double): Double {
        return bezierCurveDerivative(t)!!.y / bezierCurveDerivative(t)!!.x
    }

    /**
     * Uses [Maths] to find the angle of the tangent [Vector] of the [Bezier] curve, with respect to the origin
     * and the positive x-axis
     */
    fun angleOfCurve(t: Double): Double {
        // Get the tangent vector at the specified point on the Bezier curve
        val tangent = bezierCurveDerivative(t)

        // Calculate the angle of the tangent vector with respect to the positive x-axis

//        val angle = Math.toDegrees(Maths.angle(Vector(1.0,0.0), tangent))
        val angle = Maths.angleAtan(Vector(1.0,0.0), tangent)

        return angle
    }

    /**
     * Returns the [start] and [end] of the [Bezier] curve, ie. the first and last control points
     */
    fun start(): ParametricPose {
        return solveParametric(0.0)!!
    }
    fun end(): ParametricPose {
        return solveParametric(1.0)!!
    }

    /**
     * Utilizes the [radius] function to calculate the possible positions of the center of the [Bezier] curves
     * tangent circle at any point
     */
    fun findPerpendicularPoints(point: Vector, distance: Double, slope: Double): MutableList<Vector> {
        // Calculate the slope of the perpendicular line
        val x = point.x
        val y = point.y

        val perpendicularSlope = -1 / slope

        // Normalize the direction vector (1, -perpendicularSlope)
        val magnitude = sqrt(1 + perpendicularSlope * perpendicularSlope)
        val normalizedVector = Vector(1 / magnitude, -perpendicularSlope / magnitude)

        // Scale the normalized vector by the given distance
        val dx = normalizedVector.x * distance
        val dy = normalizedVector.y * distance

        // Calculate the two points
        val point1 = Vector(x + dx, y - dy)
        val point2 = Vector(x - dx, y + dy)

        return mutableListOf(point1, point2)
    }

    /**
     * To be used with the [findPerpendicularPoints] function above in order to determine which of the
     * points is the correct one (the above function returns the two plausible points)
     */
    fun findCorrectPoint(point: Vector, distance: Double, slope: Double, bezierTangent: Vector): Vector {
        val perpendicularPoints = findPerpendicularPoints(point, distance, slope)

        // Compute the direction vectors from the original point to the candidate points
        val direction1 = Vector(perpendicularPoints[0].x - point.x, perpendicularPoints[0].y - point.y)
        val direction2 = Vector(perpendicularPoints[1].x - point.x, perpendicularPoints[1].y - point.y)

        // Normalize the Bezier tangent vector
        val tangentMagnitude = bezierTangent.magnitude()
        val normalizedTangent = Vector(bezierTangent.x / tangentMagnitude, bezierTangent.y / tangentMagnitude)

        // Compute the dot products
        val dot1 = normalizedTangent.dot(direction1)
        val dot2 = normalizedTangent.dot(direction2)

        // Choose the point with the greater dot product (more aligned with the tangent)
        return if (dot1 > dot2) perpendicularPoints[0] else perpendicularPoints[1]
    }

    /**
     * Function for easy use of the above [findCorrectPoint] function
     */
    fun findCircleCenter(t: Double): Vector {
        return findCorrectPoint(solve(t)!!.vec(), radius(t), slope(t), bezierCurveDerivative(t)!!)
    }

    fun reverse(): Bezier {
        val points = controlPoints.reversed().toMutableList()
        return Bezier(points)
    }


    /**
     * Function that uses iterative approximation to draw the [Bezier] curve on the FTCDashboard
     * [Canvas]
     */
    fun draw(c: Canvas) {
        val incAmount = 20
        val xPoints = mutableListOf<Double>()
        val yPoints = mutableListOf<Double>()
        c.setStroke("#faaccd")

        for (i in 0..incAmount) {
            val t = i.toDouble() / incAmount
            solve(t)?.let { pose ->
                xPoints.add(pose.x)
                yPoints.add(pose.y)
            }
        }

        if (xPoints.isNotEmpty()) {
            c.strokePolyline(xPoints.toDoubleArray(), yPoints.toDoubleArray())
        }
    }

}
fun main() {
    val x = PurePursuit.builder
        .addControlPoint(0.0,0.0)
        .addControlPoint(48.0,-6.0)
        .addControlPoint(60.0, 6.0)
        .addControlPoint(60.0, 40.0)
        .build()
    println(x.bezierCurveDerivative(1.0))
}



