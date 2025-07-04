package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.AngularVelConstraint
import com.acmerobotics.roadrunner.profiles.CancelableProfile
import com.acmerobotics.roadrunner.profiles.MinVelConstraint
import com.acmerobotics.roadrunner.profiles.ProfileAccelConstraint
import com.acmerobotics.roadrunner.profiles.ProfileParams
import com.acmerobotics.roadrunner.profiles.TimeProfile
import com.acmerobotics.roadrunner.profiles.TranslationalVelConstraint
import com.acmerobotics.roadrunner.profiles.VelConstraint
import com.acmerobotics.roadrunner.trajectories.PositionPathSeqBuilder
import com.acmerobotics.roadrunner.trajectories.TrajectoryBuilderParams
import kotlin.random.Random

val TEST_PROFILE_PARAMS = ProfileParams(
    0.25,
    0.1,
    1e-4,
)

val TEST_TRAJECTORY_BUILDER_PARAMS = TrajectoryBuilderParams(
    1e-6,
    TEST_PROFILE_PARAMS
)

val TEST_VEL_CONSTRAINT: VelConstraint = MinVelConstraint(
    listOf(
        TranslationalVelConstraint(50.0),
        AngularVelConstraint(Math.PI)
    )
)
val TEST_ACCEL_CONSTRAINT: AccelConstraint =
    ProfileAccelConstraint(-10.0, 30.0)

fun posPathSeqBuilder() = PositionPathSeqBuilder(
    Vector2d(0.0, 0.0),
    0.0,
    TEST_TRAJECTORY_BUILDER_PARAMS.arcLengthSamplingEps
)

fun CancelableProfile.duration() = TimeProfile(baseProfile).duration

val Rotation2d.deg get() = Math.toDegrees(log())
val Rotation2d.repr get() = "$deg°"
val Pose2d.repr get() = "(${position.x}, ${position.y}, ${heading.repr})"