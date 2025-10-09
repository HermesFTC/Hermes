package gay.zharel.hermes.geometry

import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.math.lerpDual
import kotlinx.serialization.Serializable
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

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
@Serializable
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
@Serializable
data class Rotation2dDual<Param : DualParameter>(@JvmField val real: DualNum<Param>, @JvmField val imag: DualNum<Param>) {
    init {
        require(real.size() == imag.size()) { "Real and imaginary parts must have the same size" }
        require(real.size() <= 3) { "Only derivatives up to 2nd order are supported" }
    }

    companion object {
        @JvmStatic
        fun <Param : DualParameter> exp(theta: DualNum<Param>) = Rotation2dDual(theta.cos(), theta.sin())

        @JvmStatic
        fun <Param : DualParameter> constant(r: Rotation2d, n: Int) =
            Rotation2dDual<Param>(DualNum.Companion.constant(r.real, n), DualNum.Companion.constant(r.imag, n))

        @JvmStatic
        fun <Param : DualParameter> zero() = constant<Param>(Rotation2d.zero, 1)
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

    fun <NewParam : DualParameter> reparam(oldParam: DualNum<NewParam>) =
        Rotation2dDual(real.reparam(oldParam), imag.reparam(oldParam))

    // derivative of atan2 under unit norm assumption
    fun velocity() = real * imag.drop(1) - imag * real.drop(1)
    fun value() = Rotation2d(real.value(), imag.value())

    fun log() = DualNum<Param>(
        DoubleArray(size()) {
            Rotation2d(real[it], imag[it]).log()
        }
    )
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

fun <Param : DualParameter> lerpRotation2dDual(start: Rotation2dDual<Param >, end: Rotation2dDual<Param>, t: Double): Rotation2dDual<Param> {
    return Rotation2dDual.exp(lerpDual(t, 0.0, 1.0, start.log(), end.log()))
}