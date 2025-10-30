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
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.geometry.Vector2dDual
import gay.zharel.hermes.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * Represents a swerve module's change in position.
 *
 * @param wheelDelta change in wheel position
 * @param angle absolute angle
 */
data class SwerveModuleIncrements<Param : DualParameter>(
    @JvmField val wheelDelta: DualNum<Param>,
    @JvmField val angle: Double
) {
    constructor(wheelDelta: Double, angle: Double) :
            this(DualNum.constant(wheelDelta, 3), angle)
}

/**
 * Represents a swerve module's state (velocity and steering angle).
 *
 * @param velocity wheel velocity
 * @param angle steering angle
 */
data class SwerveModuleState<Param : DualParameter>(
    @JvmField val velocity: DualNum<Param>,
    @JvmField val angle: DualNum<Param>
) {
    constructor(velocity: Double, angle: Double) :
            this(DualNum.constant(velocity, 3), DualNum.constant(angle, 3))
}

/**
 * Kinematics for a swerve drive train.
 *
 * @param[modules] list of swerve module configurations (position and orientation)
 */
data class SwerveKinematics(
    @JvmField
    val modules: List<Vector2d>
) : RobotKinematics<SwerveKinematics.SwerveWheelIncrements<*>, SwerveKinematics.SwerveWheelVelocities<*>> {

    data class SwerveWheelIncrements<Param : DualParameter>(
        @JvmField
        val deltas: List<SwerveModuleIncrements<Param>>
    ) : WheelIncrements<Param>

    /**
     * Computes the average motion/velocity components from module contributions.
     * Returns a Triple of (avgX, avgY, avgAngular).
     */
    private fun <Param : DualParameter> averageModuleContributions(
        moduleContributions: List<Triple<DualNum<Param>, DualNum<Param>, DualNum<Param>>>
    ): Triple<DualNum<Param>, DualNum<Param>, DualNum<Param>> {
        val numModules = modules.size.toDouble()
        var sumX = moduleContributions[0].first
        var sumY = moduleContributions[0].second
        var sumAngular = moduleContributions[0].third

        for (i in 1 until moduleContributions.size) {
            sumX = sumX.plus(moduleContributions[i].first)
            sumY = sumY.plus(moduleContributions[i].second)
            sumAngular = sumAngular.plus(moduleContributions[i].third)
        }

        return Triple(
            sumX.div(numModules),
            sumY.div(numModules),
            sumAngular.div(numModules)
        )
    }

    override fun <Param : DualParameter> forward(increments: SwerveWheelIncrements<*>): Twist2dDual<Param> {
        @Suppress("UNCHECKED_CAST")
        increments as SwerveWheelIncrements<Param>

        val contributions = modules.zip(increments.deltas) { module, delta ->
            val cosAngle = cos(delta.angle)
            val sinAngle = sin(delta.angle)

            val moduleX = delta.wheelDelta * cosAngle
            val moduleY = delta.wheelDelta * sinAngle
            val angularContribution = (moduleY * module.x) - (moduleX * module.y)

            Triple(moduleX, moduleY, angularContribution)
        }

        val (avgX, avgY, avgAngular) = averageModuleContributions(contributions)

        return Twist2dDual(
            Vector2dDual(avgX, avgY),
            avgAngular
        )
    }

    data class SwerveWheelVelocities<Param : DualParameter>(
        @JvmField
        val states: List<SwerveModuleState<Param>>
    ) : WheelVelocities<Param> {
        override fun all() = states.map { it.velocity }

        override fun desaturate(maxPhysicalSpeed: Double): SwerveWheelVelocities<Param> {
            val realMax = all().maxOf { it.value() }
            return if (realMax > maxPhysicalSpeed) {
                SwerveWheelVelocities(
                    states.map { it.copy(velocity = it.velocity * maxPhysicalSpeed / realMax) }
                )
            } else {
                this
            }
        }
    }

    override fun <Param : DualParameter> forward(velocities: SwerveWheelVelocities<*>): PoseVelocity2dDual<Param> {
        @Suppress("UNCHECKED_CAST")
        velocities as SwerveWheelVelocities<Param>

        val contributions = modules.zip(velocities.states) { module, state ->
            val cosAngle = state.angle.cos()
            val sinAngle = state.angle.sin()

            val moduleX = state.velocity * cosAngle
            val moduleY = state.velocity * sinAngle
            val angularContribution = (moduleY * module.x) - (moduleX * module.y)

            Triple(moduleX, moduleY, angularContribution)
        }

        val (avgX, avgY, avgAngular) = averageModuleContributions(contributions)

        return PoseVelocity2dDual(
            Vector2dDual(avgX, avgY),
            avgAngular
        )
    }

    override fun <Param : DualParameter> inverse(velocity: PoseVelocity2dDual<Param>): SwerveWheelVelocities<Param> {
        val wheelVels = mutableListOf<DualNum<Param>>()
        val steeringAngles = mutableListOf<DualNum<Param>>()

        // Calculate wheel velocities and steering angles for each module
        modules.forEach { module ->
            // Calculate the velocity at the module position due to robot rotation
            // This is the cross product of angular velocity and the module position vector
            val rotVelX = velocity.angVel * -module.y
            val rotVelY = velocity.angVel * module.x

            // Combine the robot's linear velocity with the rotational velocity at this module
            val totalVelX = velocity.linearVel.x + rotVelX
            val totalVelY = velocity.linearVel.y + rotVelY

            // Calculate the wheel velocity (magnitude of the velocity vector)
            val wheelVel = totalVelX.times(totalVelX).plus(totalVelY.times(totalVelY)).sqrt()
            wheelVels.add(wheelVel)

            // Calculate the steering angle using Rotation2d
            // We use atan2 to get the angle of the velocity vector
            val steeringAngle = atan2(totalVelY, totalVelX)
            steeringAngles.add(steeringAngle)
        }

        // Find maximum wheel velocity for normalization if needed
        val maxWheelVel = wheelVels.maxBy { it.value() }

        // Normalize wheel velocities if any exceeds 1.0
        if (maxWheelVel.value() > 1.0) {
            wheelVels.forEachIndexed { index, wheelVel ->
                wheelVels[index] = wheelVel.div(maxWheelVel)
            }
        }

        return SwerveWheelVelocities(
            wheelVels.zip(steeringAngles).map { SwerveModuleState(it.first, it.second) }
        )
    }
}

