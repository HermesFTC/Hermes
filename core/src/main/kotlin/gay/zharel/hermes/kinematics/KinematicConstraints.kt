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
     * Maps per-wheel voltage-limited accelerations back to a bound on scalar path acceleration a_s
     * using the kinematic sensitivity of wheel speeds to the path speed.
     */
    override fun minMaxProfileAccel(
        robotState: RobotState,
        path: PosePath,
        s: Double
    ): MinMax {
        // Convert the geometric velocity (per unit arclength) into the robot frame.
        val txRobotWorld = robotState.pose.inverse()
        val robotVelWorld = robotState.vel
        val robotVelRobot = txRobotWorld * robotVelWorld

        // Current wheel velocities corresponding to the geometric velocity direction. These are used
        // to determine the achievable wheel acceleration ranges from the feedforward model.
        val wheelVels = kinematics.inverse(PoseVelocity2dDual.constant(robotVelRobot, 1)) as WV
        val wheelVelValues = wheelVels.all().map { it.value() }

        val maxWheelAccels = wheelVelValues.map { v -> feedforward.maxAchievableAcceleration(maxVoltage, v) }
        val minWheelAccels = wheelVelValues.map { v -> feedforward.minAchievableAcceleration(maxVoltage, v) }

        // Build a dual robot velocity parameterized by a scalar path speed v such that
        // chassis velocity = robotVelRobot * v, with dv/dv = 1. The derivative of each wheel speed
        // w.r.t. v gives the sensitivity A_i = ∂w_i/∂v.
        val vVar = gay.zharel.hermes.math.DualNum.variable<gay.zharel.hermes.math.Time>(0.0, 2)
        val velDual = PoseVelocity2dDual(
            vVar * robotVelRobot.linearVel,
            vVar * robotVelRobot.angVel
        )
        @Suppress("UNCHECKED_CAST")
        val wheelVelsDual = kinematics.inverse(velDual).all()

        // Intersect the per-wheel constraints a_min,i <= A_i * a_s <= a_max,i over all wheels.
        var asMin = Double.NEGATIVE_INFINITY
        var asMax = Double.POSITIVE_INFINITY
        val eps = 1e-9
        for (i in wheelVelsDual.indices) {
            val Ai = wheelVelsDual[i][1]
            val wMin = minWheelAccels[i]
            val wMax = maxWheelAccels[i]

            if (kotlin.math.abs(Ai) < eps) {
                // If the wheel speed is insensitive to path speed at this instant, it does not constrain a_s.
                continue
            }

            val bound1 = wMin / Ai
            val bound2 = wMax / Ai
            val lo = kotlin.math.min(bound1, bound2)
            val hi = kotlin.math.max(bound1, bound2)

            if (lo > asMin) asMin = lo
            if (hi < asMax) asMax = hi
        }

        // Ensure bounds are ordered; if infeasible (asMin > asMax), collapse to zero window.
        if (asMin > asMax) {
            val mid = 0.0
            asMin = mid
            asMax = mid
        }

        return MinMax(asMin, asMax)
    }
}