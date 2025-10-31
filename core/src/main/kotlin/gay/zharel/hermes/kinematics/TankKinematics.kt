/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.kinematics

import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.geometry.Twist2dDual
import gay.zharel.hermes.geometry.Vector2dDual
import kotlin.math.abs
import kotlin.math.absoluteValue

/**
 * Kinematics for a tank (differential) drive train.
 *
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 */
data class TankKinematics(@JvmField val trackWidth: Double) :
    RobotKinematics<TankKinematics.TankWheelIncrements<*>, TankKinematics.TankWheelVelocities<*>> {

    data class TankWheelIncrements<Param : DualParameter>(
        @JvmField
        val left: DualNum<Param>,
        @JvmField
        val right: DualNum<Param>,
    ) : WheelIncrements<Param>

    override fun <Param : DualParameter> forward(increments: TankWheelIncrements<*>): Twist2dDual<Param> {
        @Suppress("UNCHECKED_CAST")
        increments as TankWheelIncrements<Param>

        return Twist2dDual(
            Vector2dDual(
                (increments.left + increments.right) * 0.5,
                DualNum.constant(0.0, increments.left.size()),
            ),
            (-increments.left + increments.right) / trackWidth,
        )
    }

    data class TankWheelVelocities<Param : DualParameter>(
        @JvmField
        val left: DualNum<Param>,
        @JvmField
        val right: DualNum<Param>,
    ) : WheelVelocities<Param> {
        override fun all() = listOf(left, right)

        override fun desaturate(maxPhysicalSpeed: Double): WheelVelocities<Param> {
            val realMax = all().maxOf { it.value().absoluteValue }
            return if (realMax > maxPhysicalSpeed) {
                TankWheelVelocities(
                    left * maxPhysicalSpeed / realMax,
                    right * maxPhysicalSpeed / realMax,
                )
            } else {
                this
            }
        }
    }

    override fun <Param : DualParameter> forward(velocities: TankWheelVelocities<*>): PoseVelocity2dDual<Param> {
        @Suppress("UNCHECKED_CAST")
        velocities as TankWheelVelocities<Param>

        return PoseVelocity2dDual(
            Vector2dDual(
                (velocities.left + velocities.right) * 0.5,
                DualNum.constant(0.0, velocities.left.size()),
            ),
            (-velocities.left + velocities.right) / trackWidth,
        )
    }

    override fun <Param : DualParameter> inverse(velocity: PoseVelocity2dDual<Param>): TankWheelVelocities<Param> {
        require(velocity.linearVel.y.values().all { abs(it) < 1e-6 }) {
            "Tank drive does not support lateral motion"
        }

        return TankWheelVelocities(
            velocity.linearVel.x - velocity.angVel * 0.5 * trackWidth,
            velocity.linearVel.x + velocity.angVel * 0.5 * trackWidth,
        )
    }
}

