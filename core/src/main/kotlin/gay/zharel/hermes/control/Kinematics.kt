/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

@file:Suppress("UNCHECKED_CAST")
package gay.zharel.hermes.control

import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.geometry.Twist2dDual
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.geometry.Vector2dDual
import gay.zharel.hermes.math.atan2
import gay.zharel.hermes.paths.PosePath
import gay.zharel.hermes.profiles.VelConstraint
import kotlin.collections.forEach
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

interface WheelIncrements<Param : DualParameter>

/**
 * Represents the velocities of the individual drive wheels.
 */
interface WheelVelocities<Param : DualParameter> {
    /**
     * Returns a list of all wheel velocities.
     * The order should be consistent for a given implementation.
     */
    fun all(): List<DualNum<Param>>

    /**
     *
     */
    fun desaturate(maxPhysicalSpeed: Double): WheelVelocities<Param>
}

/**
 * Represents the kinematics of a robot drive train, providing methods for
 * inverse kinematics and velocity constraints based on wheel speeds.
 */
interface RobotKinematics<in WI: WheelIncrements<*>, in WV: WheelVelocities<*>> {
    /**
     * Performs forward kinematics: computes the twist (pose delta) that occurred
     * based on the given wheel increments.
     */
    fun <Param : DualParameter> forward(increments: WI): Twist2dDual<Param>

    /**
     * Performs forward kinematics: computes the chassis velocity required
     * to achieve the given wheel velocities.
     */
    fun <Param : DualParameter> forward(velocities: WV): PoseVelocity2dDual<Param>

    /**
     * Performs inverse kinematics: computes wheel velocities required to achieve
     * the desired robot velocity.
     *
     * @param velocity Robot velocity in the robot's local frame.
     * @return Wheel velocities.
     */
    fun <Param : DualParameter> inverse(velocity: PoseVelocity2dDual<Param>): WheelVelocities<Param>
}

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
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

/**
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
            val realMax = all().maxOf { it.value() }
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

/**
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

data class SwerveModuleState<Param : DualParameter>(
    @JvmField val velocity: DualNum<Param>,
    @JvmField val angle: DualNum<Param>
) {
    constructor(velocity: Double, angle: Double) :
            this(DualNum.constant(velocity, 3), DualNum.constant(angle, 3))

}

/**
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

        val wheelVels = kinematics.inverse(PoseVelocity2dDual.constant(robotVelRobot, 1))
            .desaturate(maxWheelVel) as WV

        val poseVel = kinematics.forward<DualParameter>(wheelVels)

        return poseVel.linearVel.norm().value()
    }
}
