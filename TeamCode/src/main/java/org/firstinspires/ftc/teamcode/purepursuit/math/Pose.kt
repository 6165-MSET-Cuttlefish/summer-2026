/*
 * Copyright (c) 2026 Vikram Janakiraman. All rights reserved.
 */
package org.firstinspires.ftc.teamcode.purepursuit.math

import com.acmerobotics.roadrunner.Pose2d


/**
 * 2d Pose object, stores and [x] value, a [y] value, and an [h] (heading) value
 */
data class Pose(@JvmField val x: Double, @JvmField val y: Double, @JvmField var h: Double): PathComponent {
    constructor(vect: Vector, h: Double) : this(vect.x, vect.y, h)

    constructor(): this(0.0,0.0,0.0)

    constructor(vect: Vector): this(vect, 0.0)

    constructor(x: Double, y: Double): this(x,y, 0.0)

    operator fun plus(other: Pose) = Pose(x + other.x, y + other.y, h + other.h)

    operator fun minus(other: Pose) = Pose(x - other.x, y - other.y, h - other.h)

    operator fun times(scalar: Double) = Pose(x * scalar, y * scalar, h * scalar)

    operator fun div(divisor: Double) = Pose(x/divisor, y/divisor, h/divisor)

    fun toRad() = Pose(x,y, Math.toRadians(h))

    fun toDeg() = Pose(x,y, Math.toDegrees(h))

    fun vec() = Vector(x,y)

    fun toPose2d() = Pose2d(x,y,h)

    fun toParametric(t: Double) = ParametricPose(this, t)
}
/**
 * Stores a 2d [Pose] with a [t] value representing the parameter that can be inputted into a bezier curve to produce this point.
 */
data class ParametricPose(@JvmField val x: Double, @JvmField val y: Double, @JvmField var h: Double, @JvmField val t: Double): PathComponent {
    var pose: Pose
    init {
        pose = Pose(x,y,h)
    }

    constructor(pose: Pose, t: Double): this(pose.x, pose.y, pose.h, t)
    constructor(pose: ParametricPose, headingTarget: Double? = null): this(pose.x, pose.y, headingTarget?: pose.h, pose.t)

    constructor(): this(0.0, 0.0, 0.0, 0.0)

    operator fun plus(other: ParametricPose) = ParametricPose(pose + other.pose, t)

    operator fun minus(other: ParametricPose) = ParametricPose(pose - other.pose, t)

    operator fun times(scalar: Double) = ParametricPose(pose * scalar, t)

    operator fun div(divisor: Double) = ParametricPose(pose / divisor, t)

    fun toRad() = ParametricPose(pose.toRad(), t)

    fun toDeg() = ParametricPose(pose.toDeg(), t)

    fun vec() = Vector(pose.x, pose.y)

    fun setT(t: Double) = ParametricPose(pose, t)

    fun toPose() = pose
}

/**
 * Stores a [heading] value to be set as the target at a specific [t] value along a bezier curve
 */
data class ParametricHeading(@JvmField val heading: Double, @JvmField val t: Double): PathComponent
data class SearchRadiusTarget(@JvmField val searchRadius: Double, @JvmField val t: Double): PathComponent