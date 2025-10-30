/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 *
 * Some code in this file is adapted from WPILib under a BSD-style license
 * that can be found in the External-Licenses file at the root of this repository.
 */

@file:Suppress("UNCHECKED_CAST")
package gay.zharel.hermes.kinematics

import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.math.MinMax
import gay.zharel.hermes.paths.PosePath
import gay.zharel.hermes.profiles.AccelConstraint
import gay.zharel.hermes.profiles.VelConstraint

/**
 * Velocity constraint based on maximum wheel velocities.
 *
 * This constraint calculates the maximum robot velocity by considering the
 * physical limits of the wheel velocities through inverse and forward kinematics.
 *
 * @property kinematics The robot kinematics model
 * @property maxWheelVel The maximum allowable wheel velocity
 */
class WheelVelConstraint<WI : WheelIncrements<*>, WV : WheelVelocities<*>>(
    @JvmField
    val kinematics: RobotKinematics<WI, WV>,
    @JvmField
    val maxWheelVel: Double
) : VelConstraint {
    override fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double): Double {
        val txRobotWorld = robotState.pose.inverse()
        val robotVelWorld = robotState.vel
        val robotVelRobot = txRobotWorld * robotVelWorld

        val wheelVels = kinematics.inverse(PoseVelocity2dDual.Companion.constant(robotVelRobot, 1))
            .desaturate(maxWheelVel) as WV

        val poseVel = kinematics.forward<DualParameter>(wheelVels)

        return poseVel.linearVel.norm().value()
    }
}

class VoltageConstraint<WI : WheelIncrements<*>, WV : WheelVelocities<*>>(
    @JvmField
    val kinematics: RobotKinematics<WI, WV>,
    @JvmField
    val feedforward: MotorFeedforward,
    @JvmField
    val maxVoltage: Double
) : AccelConstraint {
    /**
     * Returns the minimum and maximum profile acceleration at the specified state and path position.
     *
     * @param robotState The current state of the robot
     * @param path The path being followed
     * @param s The arc length parameter along the path
     * @return A [MinMax] containing the minimum and maximum allowable accelerations
     */
    override fun minMaxProfileAccel(
        robotState: RobotState,
        path: PosePath,
        s: Double
    ): MinMax {
        val txRobotWorld = robotState.pose.inverse()
        val robotVelWorld = robotState.vel
        val robotVelRobot = txRobotWorld * robotVelWorld

        val wheelVels = kinematics.inverse(PoseVelocity2dDual.Companion.constant(robotVelRobot, 1)) as WV

        val maxVel = wheelVels.all().maxOf { it.value() }
        val minVel = wheelVels.all().minOf { it.value() }

        val maxAccel = feedforward.maxAchievableAcceleration(maxVoltage, maxVel)
        val minAccel = feedforward.minAchievableAcceleration(maxVoltage, minVel)

        return MinMax(minAccel, maxAccel)
    }
}