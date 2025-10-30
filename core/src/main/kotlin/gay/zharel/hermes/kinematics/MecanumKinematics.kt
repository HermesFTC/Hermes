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

/**
 * Kinematics for a mecanum drive train.
 *
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 * @param[wheelbase] distance between the front and rear axles
 * @param[lateralMultiplier] factor that multiplies strafe velocity to compensate for slip; increase it to boost the
 * distance traveled in the strafe direction
 */
data class MecanumKinematics @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
    @JvmField
    val wheelbase: Double,
    @JvmField
    val lateralMultiplier: Double = 1.0
) : RobotKinematics<MecanumKinematics.MecanumWheelIncrements<*>, WheelVelocities<*>> {
    // The effective radius for rotation is the average of trackWidth and wheelbase
    private val effectiveRadius: Double = (trackWidth + wheelbase) / 2.0

    data class MecanumWheelIncrements<Param : DualParameter>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    ) : WheelIncrements<Param>

    override fun <Param : DualParameter> forward(increments: MecanumWheelIncrements<*>): Twist2dDual<Param> {
        @Suppress("UNCHECKED_CAST")
        increments as MecanumWheelIncrements<Param>

        return Twist2dDual(
            Vector2dDual(
                (increments.leftFront + increments.leftBack + increments.rightBack + increments.rightFront) * 0.25,
                (-increments.leftFront + increments.leftBack - increments.rightBack + increments.rightFront) * (0.25 / lateralMultiplier),
            ),
            (-increments.leftFront - increments.leftBack + increments.rightBack + increments.rightFront) * (0.25 / effectiveRadius),
        )
    }

    data class MecanumWheelVelocities<Param : DualParameter>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    ) : WheelVelocities<Param> {
        override fun all() = listOf(leftFront, leftBack, rightBack, rightFront)

        override fun desaturate(maxPhysicalSpeed: Double): WheelVelocities<Param> {
            val realMax = all().maxOf { it.value() }
            return if (realMax > maxPhysicalSpeed) {
                MecanumWheelVelocities(
                    leftFront * maxPhysicalSpeed / realMax,
                    leftBack * maxPhysicalSpeed / realMax,
                    rightBack * maxPhysicalSpeed / realMax,
                    rightFront * maxPhysicalSpeed / realMax,
                )
            } else {
                this
            }
        }
    }

    override fun <Param : DualParameter> forward(velocities: WheelVelocities<*>): PoseVelocity2dDual<Param> {
        @Suppress("UNCHECKED_CAST")
        velocities as MecanumWheelVelocities<Param>

        return PoseVelocity2dDual(
            Vector2dDual(
                (velocities.leftFront + velocities.leftBack + velocities.rightBack + velocities.rightFront) * 0.25,
                (-velocities.leftFront + velocities.leftBack - velocities.rightBack + velocities.rightFront) * (0.25 / lateralMultiplier),
            ),
            (-velocities.leftFront - velocities.leftBack + velocities.rightBack + velocities.rightFront) * (0.25 / effectiveRadius),
        )
    }

    override fun <Param : DualParameter> inverse(velocity: PoseVelocity2dDual<Param>) = MecanumWheelVelocities(
        velocity.linearVel.x - velocity.linearVel.y * lateralMultiplier - velocity.angVel * effectiveRadius,
        velocity.linearVel.x + velocity.linearVel.y * lateralMultiplier - velocity.angVel * effectiveRadius,
        velocity.linearVel.x - velocity.linearVel.y * lateralMultiplier + velocity.angVel * effectiveRadius,
        velocity.linearVel.x + velocity.linearVel.y * lateralMultiplier + velocity.angVel * effectiveRadius,
    )
}

