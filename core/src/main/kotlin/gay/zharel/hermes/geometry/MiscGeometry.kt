@file:JvmName("Geometry")

package gay.zharel.hermes.geometry

import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter
import kotlinx.serialization.Serializable

@Serializable
data class PoseVelocity2d(@JvmField val linearVel: Vector2d, @JvmField val angVel: Double) {
    operator fun minus(pv: PoseVelocity2d) = PoseVelocity2d(linearVel - pv.linearVel, angVel - pv.angVel)

    companion object {
        @JvmField
        val zero = PoseVelocity2d(Vector2d.zero, 0.0)
    }
}

/**
 * Dual version of [PoseVelocity2d].
 */
@Serializable
data class PoseVelocity2dDual<Param : DualParameter>(
    @JvmField val linearVel: Vector2dDual<Param>,
    @JvmField val angVel: DualNum<Param>
) {
    companion object {
        @JvmStatic
        fun <Param : DualParameter> constant(pv: PoseVelocity2d, n: Int) =
            PoseVelocity2dDual<Param>(Vector2dDual.constant(pv.linearVel, n), DualNum.constant(pv.angVel, n))

        @JvmStatic
        fun <Param : DualParameter> zero() = constant<Param>(PoseVelocity2d.zero, 1)
    }

    operator fun plus(other: PoseVelocity2d) = PoseVelocity2dDual(linearVel + other.linearVel, angVel + other.angVel)

    fun value() = PoseVelocity2d(linearVel.value(), angVel.value())

    fun acceleration() = if (linearVel.x.size() > 1 && angVel.size() > 1) {
        Acceleration2d(linearVel.drop(1).value(), angVel.drop(1).value())
    } else {
        Acceleration2d.zero
    }
}

@Serializable
data class Twist2d(@JvmField val line: Vector2d, @JvmField val angle: Double) {
    companion object {
        @JvmField
        val zero = Twist2d(Vector2d.zero, 0.0)
    }
}

@Serializable
data class Twist2dDual<Param : DualParameter>(
    @JvmField val line: Vector2dDual<Param>,
    @JvmField val angle: DualNum<Param>
) {
    fun value() = Twist2d(line.value(), angle.value())
    fun velocity() = PoseVelocity2dDual(line.drop(1), angle.drop(1))
}

@Serializable
data class Acceleration2d(@JvmField val linearAcc: Vector2d, @JvmField val angAcc: Double) {
    companion object {
        @JvmField
        val zero = Acceleration2d(Vector2d.zero, 0.0)
    }

    /**
     * Uses kinematic integration to compute a new velocity given a time step and initial velocity.
     * @param initial initial velocity; default is 0.
     */
    @JvmOverloads
    fun integrateToVel(dt: Double, initial: PoseVelocity2d = PoseVelocity2d.zero) =
        PoseVelocity2d(initial.linearVel + linearAcc * dt, initial.angVel + angAcc * dt)
}

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
