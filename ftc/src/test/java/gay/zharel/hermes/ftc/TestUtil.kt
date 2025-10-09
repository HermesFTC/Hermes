package gay.zharel.hermes.ftc

import gay.zharel.hermes.TEST_ACCEL_CONSTRAINT
import gay.zharel.hermes.TEST_PROFILE_PARAMS
import gay.zharel.hermes.TEST_VEL_CONSTRAINT
import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.trajectories.TurnConstraints
import io.mockk.every
import io.mockk.mockk

val TEST_FOLLOWER_PARAMS = FollowerParams(
    TEST_PROFILE_PARAMS,
    TEST_VEL_CONSTRAINT,
    TEST_ACCEL_CONSTRAINT
)

val TEST_TURN_CONSTRAINTS = TurnConstraints(
    Math.PI,
    0.0,
    Math.PI
)

val TEST_DRIVE get() = mockk<Drive>(relaxUnitFun = true).also {
    every { it.followerParams } returns TEST_FOLLOWER_PARAMS
    every { it.defaultVelConstraint } returns TEST_VEL_CONSTRAINT
    every { it.defaultAccelConstraint } returns TEST_ACCEL_CONSTRAINT
    every { it.defaultTurnConstraints } returns TEST_TURN_CONSTRAINTS

    every { it.localizer.pose } returns Pose2d.zero
}