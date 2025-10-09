@file:JvmName("PathGen")
package gay.zharel.hermes.ftc

import gay.zharel.hermes.math.Arclength
import gay.zharel.hermes.geometry.Rotation2d
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.paths.CompositePosePath
import gay.zharel.hermes.profiles.DisplacementProfile
import gay.zharel.hermes.trajectories.DisplacementTrajectory
import gay.zharel.hermes.trajectories.PathBuilder
import gay.zharel.hermes.trajectories.Trajectory

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
                DisplacementProfile.generate(
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
            DisplacementProfile.generate(
                followerParams.profileParams,
                it,
                0.0,
                followerParams.velConstraint,
                followerParams.accelConstraint
            )
        )
    }