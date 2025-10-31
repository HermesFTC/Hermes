/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.geometry

import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter

/**
 * Represents a robot's pose on the field at a given time.
 *
 * @property pose The position and heading of the robot, in field coordinates.
 * @property vel The linear and angular velocity of the robot, in the robot frame.
 * @property accel The linear and angular acceleration of the robot, in the robot frame.
 */
data class RobotState(
    @JvmField val pose: Pose2d,
    @JvmField val vel: PoseVelocity2d,
    @JvmField val accel: Acceleration2d,
) {
    companion object {
        @JvmField val zero = RobotState(Pose2d.zero, PoseVelocity2d.zero, Acceleration2d.zero)

        @JvmStatic fun <Param : DualParameter> fromDualPose(dualPose: Pose2dDual<Param>) = RobotState(
            dualPose.value(),
            dualPose.velocity().value(),
            dualPose.velocity().acceleration()
        )
    }

    fun <Param : DualParameter> toDualPose() = Pose2dDual<Param>(
        DualNum(doubleArrayOf(pose.position.x, vel.linearVel.x, accel.linearAcc.x)),
        DualNum(doubleArrayOf(pose.position.y, vel.linearVel.y, accel.linearAcc.y)),
        Rotation2dDual.exp(DualNum(doubleArrayOf(pose.heading.log(), vel.angVel, accel.angAcc)))
    )
}