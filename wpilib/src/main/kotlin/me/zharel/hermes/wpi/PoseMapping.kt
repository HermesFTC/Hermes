package me.zharel.hermes.wpi

import com.acmerobotics.roadrunner.geometry.Pose2dDual
import edu.wpi.first.math.geometry.Pose2d

fun interface PoseMap {
    fun map(pose: Pose2d): Pose2d

    companion object {
        @JvmStatic
        val Identity = PoseMap { it }
    }
}

internal fun PoseMap.hermes() = com.acmerobotics.roadrunner.paths.PoseMap {
    Pose2dDual.constant(this.map(it.value().wpilib()).hermes(), 1)
}

