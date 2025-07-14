@file:JvmName("BezierCurves")
package com.acmerobotics.roadrunner.paths

import com.acmerobotics.roadrunner.geometry.*
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlin.math.pow

/**
 * @usesMathJax
 *
 * Bezier spline of degree k with equation \(\sum_{i=0}^{k} a_i t^i\) where \(0 \leq t \leq 1\)
 *
 * @property[coefficients] Array of coefficients \(a_i\) for \(i = 0, 1, ..., k\)
 */
@Serializable
data class BezierCurve1d(
    val coefficients: List<Double>,
) {
    constructor(coefficients: DoubleArray) : this(coefficients.toList())

    val dim = coefficients.size - 1
    val cache = coefficients.indices.map { dim bi it }

    operator fun get(t: Double): Double {
        return (0..dim).sumOf { i ->
            cache[i] * (1 - t).pow(dim - i) * t.pow(i) * coefficients[i]
        }
    }

    operator fun get(t: Double, n: Int): DualNum<Internal> {
        return DualNum<Internal>(DoubleArray(n) {
            nthDerivative(it)[t]
        })
    }

    fun derivative(): BezierCurve1d {
        return BezierCurve1d(List(dim) { i ->
            dim * (coefficients[i + 1] - coefficients[i])
        })
    }

    fun nthDerivative(n: Int): BezierCurve1d {
        if (n == 0) return this
        if (coefficients.size <= n) return BezierCurve1d(List(coefficients.size) { 0.0 })
        return (0..<n).fold(this) { acc, _ -> acc.derivative() }
    }

    companion object {
        private infix fun Int.bi(other: Int): Int {
            if (other < 0 || other > this) return 0
            if (other == 0 || other == this) return 1

            return this.fact() / (other.fact() * (this - other).fact())
        }

        fun fromRRSpline(spline: QuinticSpline1d): BezierCurve1d =
            BezierCurve1d(doubleArrayOf(
                spline.f,
                spline.f + 0.2 * spline.e,
                spline.f + 0.4 * spline.e + 0.1 * spline.d,
                spline.f + 0.6 * spline.e + 0.3 * spline.d + 0.1 * spline.c,
                spline.f + 0.8 * spline.e + 0.6 * spline.d + 0.4 * spline.c + 0.2 * spline.b,
                spline.coefficients.sum()
                ))
    }
}

@Serializable
@SerialName("BezierCurve2d")
class BezierCurve2dInternal(val x: BezierCurve1d, val y: BezierCurve1d) : PositionPath<Internal> {
    constructor(xs:  List<Double>, ys: List<Double>) : this(BezierCurve1d(xs), BezierCurve1d(ys))
    constructor(points: List<Vector2d>) : this(points.xs(), points.ys())

    override operator fun get(t: Double, n: Int) =
        Vector2dDual(x[t, n], y[t, n])

    val length = 1.0
    override fun length() = length

    companion object {
        fun fromPoints(points: List<Vector2d>) = BezierCurve2dInternal(points.xs(), points.ys())
        fun fromPoints(vararg points: Vector2d) = fromPoints(points.toList())

        fun fromRRSpline(spline: QuinticSpline2dInternal) = BezierCurve2dInternal(
            BezierCurve1d.fromRRSpline(spline.x),
            BezierCurve1d.fromRRSpline(spline.y)
        )
    }
}

fun fromPoints(vararg points: Vector2d) = fromPoints(points.toList())

fun fromPoints(points: List<Vector2d>) = ArclengthReparamCurve2d(
    BezierCurve2dInternal.fromPoints(points),
    1e-6
)
