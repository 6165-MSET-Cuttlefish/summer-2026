/*
 * Copyright (c) 2026 Vikram Janakiraman. All rights reserved.
 */
package org.firstinspires.ftc.teamcode.offseason.math

import org.firstinspires.ftc.teamcode.purepursuit.math.Parameterized
import org.firstinspires.ftc.teamcode.purepursuit.math.ParameterizedObject
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose
import org.firstinspires.ftc.teamcode.purepursuit.math.Vector


@Parameterized
data class ParameterizedCircle(var center: Vector, var radius: Double): ParameterizedObject {
    override fun solve(t: Double): Pose? {
        if (t in 0.0..1.0) {
            val angRad = t * 2*Math.PI

            val x = Math.cos(angRad) * radius + center.x
            val y = Math.sin(angRad) * radius + center.y

            return Pose(Vector(x,y))
        }
        return null
    }
}
