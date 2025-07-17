@file:JvmName("PathGen")
package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.paths.CompositePosePath
import com.acmerobotics.roadrunner.profiles.forwardProfile
import com.acmerobotics.roadrunner.trajectories.DisplacementTrajectory
import com.acmerobotics.roadrunner.trajectories.PathBuilder
import com.acmerobotics.roadrunner.trajectories.Trajectory

fun interface GenerableTrajectory {
    fun generate(): Trajectory<Arclength>
}

fun Drive.lineTo(target: Vector2d) = GenerableTrajectory {
    PathBuilder(localizer.pose, 1e-6)
        .strafeTo(target)
        .build()
        .let { CompositePosePath(it) }
        .let {
            DisplacementTrajectory(
                it,
                forwardProfile(
                    followerParams.profileParams,
                    it,
                    0.0,
                    followerParams.velConstraint,
                    followerParams.accelConstraint
                )
            )
        }
}

fun Drive.splineTo(target: Vector2d, tangent: Rotation2d) = PathBuilder(localizer.pose, 1e-6)
    .splineTo(target, tangent)
    .build()
    .let { CompositePosePath(it) }
    .let {
        DisplacementTrajectory(
            it,
            forwardProfile(
                followerParams.profileParams,
                it,
                0.0,
                followerParams.velConstraint,
                followerParams.accelConstraint
            )
        )
    }