package gay.zharel.hermes

import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.Rotation2d
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.profiles.AccelConstraint
import gay.zharel.hermes.profiles.AngularVelConstraint
import gay.zharel.hermes.profiles.CancelableProfile
import gay.zharel.hermes.profiles.MinVelConstraint
import gay.zharel.hermes.profiles.ProfileAccelConstraint
import gay.zharel.hermes.profiles.ProfileParams
import gay.zharel.hermes.profiles.TimeProfile
import gay.zharel.hermes.profiles.TranslationalVelConstraint
import gay.zharel.hermes.profiles.VelConstraint
import gay.zharel.hermes.trajectories.PositionPathSeqBuilder
import gay.zharel.hermes.trajectories.TrajectoryBuilderParams

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