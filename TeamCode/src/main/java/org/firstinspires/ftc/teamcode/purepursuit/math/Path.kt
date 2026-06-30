/*
 * Copyright (c) 2026 Vikram Janakiraman. All rights reserved.
 */
package org.firstinspires.ftc.teamcode.purepursuit.math

interface Path {
    fun isFollowable(): Boolean
    fun length(): Double
}