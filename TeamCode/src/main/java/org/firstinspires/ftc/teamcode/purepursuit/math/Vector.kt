/*
 * Copyright (c) 2026 Vikram Janakiraman. All rights reserved.
 */
package org.firstinspires.ftc.teamcode.purepursuit.math


data class Vector(@JvmField val x: Double, @JvmField val y: Double): PathComponent {
    operator fun plus(other: Vector) = Vector(x + other.x, y + other.y)
    operator fun minus(other: Vector) = Vector(x - other.x, y - other.y)
    operator fun times(other: Vector) = Vector(x * other.x, y * other.y)
    operator fun div(other: Vector) = Vector(x / other.x, y / other.y)
    fun dot(other: Vector): Double = x * other.x + y * other.y
}