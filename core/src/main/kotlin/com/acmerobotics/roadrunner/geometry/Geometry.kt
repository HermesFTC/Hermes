@file:JvmName("Geometry")

package com.acmerobotics.roadrunner.geometry

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * @usesMathJax
 *
 * Vector \((x, y)\)
 */
data class Vector2d(@JvmField val x: Double, @JvmField val y: Double) {
    operator fun plus(v: Vector2d) = Vector2d(x + v.x, y + v.y)
    operator fun minus(v: Vector2d) = Vector2d(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2d(-x, -y)

    operator fun times(z: Double) = Vector2d(x * z, y * z)
    operator fun div(z: Double) = Vector2d(x / z, y / z)

    infix fun dot(v: Vector2d) = x * v.x + y * v.y
    fun sqrNorm() = this dot this
    fun norm() = sqrt(sqrNorm())

    // precondition: this is normalized
    fun angleCast() = Rotation2d(x, y)

    fun asPair() = x to y

    companion object {
        @JvmField
        val zero = Vector2d(0.0, 0.0)
    }
}

fun List<Vector2d>.xs() = map { it.asPair() }.unzip().first
fun List<Vector2d>.ys() = map { it.asPair() }.unzip().second

/**
 * Dual version of [Vector2d].
 */
data class Vector2dDual<Param>(@JvmField val x: DualNum<Param>, @JvmField val y: DualNum<Param>) {
    companion object {
        @JvmStatic
        fun <Param> constant(v: Vector2d, n: Int) =
            Vector2dDual<Param>(DualNum.constant(v.x, n), DualNum.constant(v.y, n))

        @JvmStatic
        fun <Param> zero() = constant<Param>(Vector2d.zero, 1)
    }

    operator fun plus(v: Vector2d) = Vector2dDual(x + v.x, y + v.y)
    operator fun plus(v: Vector2dDual<Param>) = Vector2dDual(x + v.x, y + v.y)
    operator fun minus(v: Vector2dDual<Param>) = Vector2dDual(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2dDual(-x, -y)

    operator fun div(z: Double) = Vector2dDual(x / z, y / z)

    infix fun dot(v: Vector2dDual<Param>) = x * v.x + y * v.y
    fun sqrNorm() = this dot this
    fun norm() = sqrNorm().sqrt()

    fun bind() = Vector2dDual(x, y)

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Vector2dDual(x.reparam(oldParam), y.reparam(oldParam))

    fun drop(n: Int) = Vector2dDual(x.drop(n), y.drop(n))
    fun value() = Vector2d(x.value(), y.value())

    // precondition: this is normalized
    fun angleCast() = Rotation2dDual(x, y)

    fun asPair() = x to y
}

fun <Param> List<Vector2dDual<Param>>.xsDual() = map { it.asPair() }.unzip().first
fun <Param> List<Vector2dDual<Param>>.ysDual() = map { it.asPair() }.unzip().second

fun List<Vector2dDual<*>>.dva(): List<Triple<Double, Double, Double>> {
    require(first().x.size() >= 3)
    return map {
        Triple(
            it.value().norm(),
            it.drop(1).value().norm(),
            it.drop(2).value().norm()
        )
    }
}

fun <A, B, C> Iterable<Triple<A, B, C>>.unzip(): Triple<List<A>, List<B>, List<C>> {
    val first = mutableListOf<A>()
    val second = mutableListOf<B>()
    val third = mutableListOf<C>()

    for (triple in this) {
        first.add(triple.first)
        second.add(triple.second)
        third.add(triple.third)
    }

    return Triple(first, second, third)
}

/**
 * @usesMathJax
 *
 * Rotation \(\theta\) represented by the unit circle point \((\cos \theta, \sin \theta)\) or unit-modulus complex
 * number \(\cos \theta + i \sin \theta\).
 *
 * Advanced: Rotations in two dimensions comprise a Lie group referred to as SO(2). The terminology [exp] and [log]
 * comes from the Lie theory, and [this paper](https://arxiv.org/abs/1812.01537) gives a targeted exposition of the key
 * fundamentals.
 */
data class Rotation2d(@JvmField val real: Double, @JvmField val imag: Double) {
    companion object {
        /**
         * Turns an unnormalized angle into a rotation.
         */
        @JvmStatic
        fun exp(theta: Double) = Rotation2d(cos(theta), sin(theta))

        /**
         * Alias for [exp].
         */
        @JvmStatic
        fun fromDouble(theta: Double) = exp(theta)

        @JvmField
        val zero = Rotation2d(0.0, 0.0)
    }

    operator fun plus(x: Double) = this * exp(x)
    operator fun minus(r: Rotation2d) = (r.inverse() * this).log()

    operator fun times(v: Vector2d) = Vector2d(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(pv: PoseVelocity2d) = PoseVelocity2d(this * pv.linearVel, pv.angVel)
    operator fun times(r: Rotation2d) =
        Rotation2d(real * r.real - imag * r.imag, real * r.imag + imag * r.real)

    fun vec() = Vector2d(real, imag)

    fun inverse() = Rotation2d(real, -imag)

    /**
     * Get the rotation as a normalized [Double].
     */
    fun log() = atan2(imag, real)

    /**
     * Alias for [log].
     */
    fun toDouble() = log()
}

/**
 * Dual version of [Rotation2d].
 */
data class Rotation2dDual<Param>(@JvmField val real: DualNum<Param>, @JvmField val imag: DualNum<Param>) {
    init {
        require(real.size() == imag.size()) { "Real and imaginary parts must have the same size" }
        require(real.size() <= 3) { "Only derivatives up to 2nd order are supported" }
    }

    companion object {
        @JvmStatic
        fun <Param> exp(theta: DualNum<Param>) = Rotation2dDual(theta.cos(), theta.sin())

        @JvmStatic
        fun <Param> constant(r: Rotation2d, n: Int) =
            Rotation2dDual<Param>(DualNum.constant(r.real, n), DualNum.constant(r.imag, n))

        @JvmStatic
        fun <Param> zero() = constant<Param>(Rotation2d.zero, 1)
    }

    fun size() = real.size()

    operator fun plus(x: Double) = this * Rotation2d.exp(x)
    operator fun plus(d: DualNum<Param>) = this * exp(d)

    operator fun times(pv: PoseVelocity2dDual<Param>) = PoseVelocity2dDual(this * pv.linearVel, pv.angVel)
    operator fun times(v: Vector2dDual<Param>) = Vector2dDual(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(v: Vector2d) = Vector2dDual(real * v.x - imag * v.y, imag * v.x + real * v.y)
    operator fun times(r: Rotation2dDual<Param>) =
        Rotation2dDual(real * r.real - imag * r.imag, real * r.imag + imag * r.real)
    operator fun times(r: Rotation2d) =
        Rotation2dDual(real * r.real - imag * r.imag, real * r.imag + imag * r.real)
    operator fun times(p: Pose2d) = Pose2dDual(this * p.position, this * p.heading)

    fun inverse() = Rotation2dDual(real, -imag)

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Rotation2dDual(real.reparam(oldParam), imag.reparam(oldParam))

    // derivative of atan2 under unit norm assumption
    fun velocity() = real * imag.drop(1) - imag * real.drop(1)
    fun value() = Rotation2d(real.value(), imag.value())

    fun log() = DualNum<Param>(DoubleArray(size()) {
        Rotation2d(real[it], imag[it]).log()
    })
}

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
data class Pose2d(
    @JvmField
    val position: Vector2d,
    @JvmField
    val heading: Rotation2d,
) {
    constructor(position: Vector2d, heading: Double) : this(position, Rotation2d.exp(heading))
    constructor(positionX: Double, positionY: Double, heading: Double) : this(Vector2d(positionX, positionY), heading)

    companion object {
        @JvmStatic
        fun exp(t: Twist2d): Pose2d {
            val heading = Rotation2d.exp(t.angle)

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
data class Pose2dDual<Param>(
    @JvmField
    val position: Vector2dDual<Param>,
    @JvmField
    val heading: Rotation2dDual<Param>,
) {
    constructor(positionX: DualNum<Param>, positionY: DualNum<Param>, heading: Rotation2dDual<Param>) :
        this(Vector2dDual(positionX, positionY), heading)
    constructor(positionX: DualNum<Param>, positionY: DualNum<Param>, heading: DualNum<Param>) :
        this(positionX, positionY, Rotation2dDual.exp(heading))

    companion object {
        @JvmStatic
        fun <Param> constant(p: Pose2d, n: Int) =
            Pose2dDual<Param>(Vector2dDual.constant(p.position, n), Rotation2dDual.constant(p.heading, n))

        @JvmStatic
        fun <Param> zero() = constant<Param>(Pose2d.zero, 1)
    }

    operator fun plus(t: Twist2d) = this * Pose2d.exp(t)

    operator fun times(p: Pose2d) = Pose2dDual(heading * p.position + position, heading * p.heading)
    operator fun times(p: Pose2dDual<Param>) = Pose2dDual(heading * p.position + position, heading * p.heading)
    operator fun times(pv: PoseVelocity2dDual<Param>) = PoseVelocity2dDual(heading * pv.linearVel, pv.angVel)

    fun inverse() = heading.inverse().let {
        Pose2dDual(it * -position, it)
    }

    fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
        Pose2dDual(position.reparam(oldParam), heading.reparam(oldParam))

    fun value() = Pose2d(position.value(), heading.value())
    fun velocity() = PoseVelocity2dDual(position.drop(1), heading.velocity())
}

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
data class PoseVelocity2dDual<Param>(
    @JvmField val linearVel: Vector2dDual<Param>,
    @JvmField val angVel: DualNum<Param>
) {
    companion object {
        @JvmStatic
        fun <Param> constant(pv: PoseVelocity2d, n: Int) =
            PoseVelocity2dDual<Param>(Vector2dDual.constant(pv.linearVel, n), DualNum.constant(pv.angVel, n))

        @JvmStatic
        fun <Param> zero() = constant<Param>(PoseVelocity2d.zero, 1)
    }

    operator fun plus(other: PoseVelocity2d) = PoseVelocity2dDual(linearVel + other.linearVel, angVel + other.angVel)

    fun value() = PoseVelocity2d(linearVel.value(), angVel.value())
}

data class Twist2d(@JvmField val line: Vector2d, @JvmField val angle: Double) {
    companion object {
        @JvmField
        val zero = Twist2d(Vector2d.zero, 0.0)
    }
}

data class Twist2dDual<Param>(
    @JvmField val line: Vector2dDual<Param>,
    @JvmField val angle: DualNum<Param>
) {
    fun value() = Twist2d(line.value(), angle.value())
    fun velocity() = PoseVelocity2dDual(line.drop(1), angle.drop(1))
}

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
 * Linearly interpolates between two Vector2d objects.
 *
 * @param start The starting Vector2d.
 * @param end The ending Vector2d.
 * @param t The interpolation parameter, where 0.0 returns `start`, 1.0 returns `end`, and values
 *          in between return a Vector2d linearly interpolated between `start` and `end`.
 * @return A new Vector2d that is linearly interpolated between `start` and `end`.
 */
fun lerpVector2d(start: Vector2d, end: Vector2d, t: Double): Vector2d {
    val x = lerp(t, 0.0, 1.0, start.x, end.x)
    val y = lerp(t, 0.0, 1.0, start.y, end.y)
    return Vector2d(x, y)
}

fun <Param> lerpVector2dDual(start: Vector2dDual<Param>, end: Vector2dDual<Param>, t: Double): Vector2dDual<Param> {
    val x = lerpDual(t, 0.0, 1.0, start.x, end.x)
    val y = lerpDual(t, 0.0, 1.0, start.y, end.y)
    return Vector2dDual(x, y)
}

/**
 * Linearly interpolates an angle, handling wrap-around.
 *
 * @param start The starting angle (in radians).
 * @param end The ending angle (in radians).
 * @param t The interpolation parameter, where 0.0 returns `start`, 1.0 returns `end`, and values
 *          in between return an angle linearly interpolated between `start` and `end`.
 * @return The interpolated angle (in radians).
 */
fun lerpRotation2d(start: Rotation2d, end: Rotation2d, t: Double): Rotation2d {
    // Calculate the shortest distance between the two angles
    val diff = (end - start)

    return Rotation2d.exp(start.log() + diff * t)
}

fun <Param> lerpRotation2dDual(start: Rotation2dDual<Param>, end: Rotation2dDual<Param>, t: Double): Rotation2dDual<Param> {
    return Rotation2dDual.exp(lerpDual(t, 0.0, 1.0, start.log(), end.log()))
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

fun <Param> lerpPose2dDual(start: Pose2dDual<Param>, end: Pose2dDual<Param>, t: Double): Pose2dDual<Param> {
    require(t in 0.0..1.0) { "Interpolation parameter t must be between 0.0 and 1.0, but was $t" }

    // Interpolate position
    val position = lerpVector2dDual(start.position, end.position, t)

    // Interpolate heading, handling wrap-around
    val heading = lerpRotation2dDual(start.heading, end.heading, t)

    return Pose2dDual(position, heading)
}

fun lerpPoseLookup(times: List<Double>, poses: List<Pose2d>, query: Double): Pose2d {
    val index = times.binarySearch(query)

    if (index >= 0) return poses[index]

    val nextIdx = -(index + 1)
    val prevIdx = -index

    return lerpPose2d(
        poses[prevIdx],
        poses[nextIdx],
        lerp(query, times[prevIdx], times[nextIdx], 0.0, 1.0)
    )
}

fun <Param> lerpPoseLookupDual(times: List<Double>, poses: List<Pose2dDual<Param>>, query: Double): Pose2dDual<Param> {
    val index = times.binarySearch(query)

    if (index >= 0) return poses[index]

    val nextIdx = -(index + 1)
    val prevIdx = -index

    return lerpPose2dDual(
        poses[prevIdx],
        poses[nextIdx],
        lerp(query, times[prevIdx], times[nextIdx], 0.0, 1.0)
    )
}