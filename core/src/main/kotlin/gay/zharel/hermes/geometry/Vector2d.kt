/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.geometry

import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.math.lerp
import gay.zharel.hermes.math.lerpDual
import kotlinx.serialization.Serializable
import kotlin.math.sqrt

/**
 * @usesMathJax
 *
 * Vector \((x, y)\)
 */
@Serializable
data class Vector2d(@JvmField val x: Double, @JvmField val y: Double) {
    operator fun plus(v: Vector2d) = Vector2d(x + v.x, y + v.y)
    operator fun minus(v: Vector2d) = Vector2d(x - v.x, y - v.y)
    operator fun unaryMinus() = Vector2d(-x, -y)

    operator fun times(z: Double) = Vector2d(x * z, y * z)
    operator fun div(z: Double) = Vector2d(x / z, y / z)

    infix fun dot(v: Vector2d) = x * v.x + y * v.y
    fun sqrNorm() = this dot this
    fun norm() = sqrt(sqrNorm())

    /**
     * Returns the angle of this vector as a [Rotation2d],
     * assuming this is normalized.
     */
    fun angleCast() = Rotation2d(x, y)

    /**
     * Returns the angle of this vector as a [Rotation2d],
     * normalizing this first.
     */
    fun angle() = (this/norm()).angleCast()

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
@Serializable
data class Vector2dDual<Param : DualParameter>(@JvmField val x: DualNum<Param>, @JvmField val y: DualNum<Param>) {
    companion object {
        @JvmStatic
        fun <Param : DualParameter> constant(v: Vector2d, n: Int) =
            Vector2dDual<Param>(DualNum.Companion.constant(v.x, n), DualNum.Companion.constant(v.y, n))

        @JvmStatic
        fun <Param : DualParameter> zero() = constant<Param>(Vector2d.zero, 1)
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

    fun <NewParam : DualParameter> reparam(oldParam: DualNum<NewParam>) =
        Vector2dDual(x.reparam(oldParam), y.reparam(oldParam))

    fun drop(n: Int) = Vector2dDual(x.drop(n), y.drop(n))
    fun value() = Vector2d(x.value(), y.value())

    // precondition: this is normalized
    fun angleCast() = Rotation2dDual(x, y)

    fun angle() = (this/norm().value()).angleCast()

    fun asPair() = x to y
}

fun <Param : DualParameter> List<Vector2dDual<Param>>.xsDual() = map { it.asPair() }.unzip().first
fun <Param : DualParameter> List<Vector2dDual<Param>>.ysDual() = map { it.asPair() }.unzip().second

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

fun <Param : DualParameter> lerpVector2dDual(start: Vector2dDual<Param>, end: Vector2dDual<Param>, t: Double): Vector2dDual<Param> {
    val x = lerpDual(t, 0.0, 1.0, start.x, end.x)
    val y = lerpDual(t, 0.0, 1.0, start.y, end.y)
    return Vector2dDual(x, y)
}