package gay.zharel.hermes.geometry

import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.math.snz
import kotlinx.serialization.Serializable
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

/**
 * @usesMathJax
 *
 * 2D rigid transform comprised of [heading] followed by [position].
 *
 * The pose `destPoseSource` denotes the transform from frame `Source` into frame `Dest`. It can be applied with
 * `times()` to change the coordinates of `xSource` into `xDest` where `x` is a vector, twist, or even another pose:
 * `xDest = destPoseSource * xSource`. The awkward names take some getting used to, but they avoid many routine errors.
 *
 * Transforms into the world frame are common enough to warrant a shorthand. The pose `worldPoseSource` can be shortened
 * to `poseSource` for any frame `Source`.
 *
 * Advanced: Transforms in two dimensions comprise a Lie group referred to as SE(2). The terminology [exp] and [log]
 * comes from the Lie theory, and [this paper](https://arxiv.org/abs/1812.01537) gives a targeted exposition of the key
 * fundamentals.
 */
@Serializable
data class Pose2d(
    @JvmField
    val position: Vector2d,
    @JvmField
    val heading: Rotation2d,
) {
    constructor(position: Vector2d, heading: Double) : this(position, Rotation2d.Companion.exp(heading))
    constructor(positionX: Double, positionY: Double, heading: Double) : this(Vector2d(positionX, positionY), heading)

    companion object {
        @JvmStatic
        fun exp(t: Twist2d): Pose2d {
            val heading = Rotation2d.Companion.exp(t.angle)

            val u = t.angle + snz(t.angle)
            val c = 1 - cos(u)
            val s = sin(u)
            val translation = Vector2d(
                (s * t.line.x - c * t.line.y) / u,
                (c * t.line.x + s * t.line.y) / u
            )

            return Pose2d(translation, heading)
        }

        @JvmField
        val zero = Pose2d(Vector2d.zero, Rotation2d.zero)
    }

    operator fun plus(t: Twist2d) = this * exp(t)
    fun minusExp(t: Pose2d) = t.inverse() * this
    operator fun minus(t: Pose2d) = minusExp(t).log()

    operator fun times(p: Pose2d) = Pose2d(heading * p.position + position, heading * p.heading)
    operator fun times(v: Vector2d) = heading * v + position
    operator fun times(pv: PoseVelocity2d) = PoseVelocity2d(heading * pv.linearVel, pv.angVel)

    fun inverse() = Pose2d(heading.inverse() * -position, heading.inverse())

    fun log(): Twist2d {
        val theta = heading.log()

        val halfu = 0.5 * theta + snz(theta)
        val v = halfu / tan(halfu)
        return Twist2d(
            Vector2d(
                v * position.x + halfu * position.y,
                -halfu * position.x + v * position.y,
            ),
            theta,
        )
    }
}

/**
 * Dual version of [Pose2d].
 */
@Serializable
data class Pose2dDual<Param : DualParameter>(
    @JvmField
    val position: Vector2dDual<Param >,
    @JvmField
    val heading: Rotation2dDual<Param>,
) {
    constructor(positionX: DualNum<Param>, positionY: DualNum<Param>, heading: Rotation2dDual<Param>) :
        this(Vector2dDual(positionX, positionY), heading)
    constructor(positionX: DualNum<Param>, positionY: DualNum<Param>, heading: DualNum<Param>) :
        this(positionX, positionY, Rotation2dDual.exp(heading))

    companion object {
        @JvmStatic
        fun <Param : DualParameter> constant(p: Pose2d, n: Int) =
            Pose2dDual<Param>(Vector2dDual.constant(p.position, n), Rotation2dDual.constant(p.heading, n))

        @JvmStatic
        fun <Param : DualParameter> zero() = constant<Param>(Pose2d.zero, 1)
    }

    operator fun plus(t: Twist2d) = this * Pose2d.exp(t)

    operator fun times(p: Pose2d) = Pose2dDual(heading * p.position + position, heading * p.heading)
    operator fun times(p: Pose2dDual<Param>) = Pose2dDual(heading * p.position + position, heading * p.heading)
    operator fun times(pv: PoseVelocity2dDual<Param>) = PoseVelocity2dDual(heading * pv.linearVel, pv.angVel)

    fun inverse() = heading.inverse().let {
        Pose2dDual(it * -position, it)
    }

    fun <NewParam : DualParameter> reparam(oldParam: DualNum<NewParam>) =
        Pose2dDual(position.reparam(oldParam), heading.reparam(oldParam))

    fun value() = Pose2d(position.value(), heading.value())
    fun velocity() = PoseVelocity2dDual(position.drop(1), heading.velocity())
}

/**
 * Linearly interpolates between two Pose2d objects.
 *
 * This function calculates a new Pose2d object that lies between the start and end Pose2d objects
 * based on the provided interpolation parameter `t`. The `t` parameter determines how far along the
 * path from `start` to `end` the resulting Pose2d will be.
 *
 * @param start The starting Pose2d.
 * @param end The ending Pose2d.
 * @param t The interpolation parameter, where 0.0 returns `start`, 1.0 returns `end`, and values
 *          in between return a Pose2d linearly interpolated between `start` and `end`.
 * @return A new Pose2d that is linearly interpolated between `start` and `end`.
 * @throws IllegalArgumentException if `t` is not in the range [0.0, 1.0].
 */
fun lerpPose2d(start: Pose2d, end: Pose2d, t: Double): Pose2d {
    require(t in 0.0..1.0) { "Interpolation parameter t must be between 0.0 and 1.0, but was $t" }

    // Interpolate position
    val position = lerpVector2d(start.position, end.position, t)

    // Interpolate heading, handling wrap-around
    val heading = lerpRotation2d(start.heading, end.heading, t)

    return Pose2d(position, heading)
}

fun <Param : DualParameter> lerpPose2dDual(start: Pose2dDual<Param>, end: Pose2dDual<Param>, t: Double): Pose2dDual<Param> {
    require(t in 0.0..1.0) { "Interpolation parameter t must be between 0.0 and 1.0, but was $t" }

    // Interpolate position
    val position = lerpVector2dDual(start.position, end.position, t)

    // Interpolate heading, handling wrap-around
    val heading = lerpRotation2dDual(start.heading, end.heading, t)

    return Pose2dDual(position, heading)
}