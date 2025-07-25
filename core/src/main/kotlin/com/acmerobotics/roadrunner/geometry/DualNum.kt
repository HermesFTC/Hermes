package com.acmerobotics.roadrunner.geometry

import kotlinx.serialization.Serializable
import kotlin.math.atan2
import kotlin.math.min

/**
 * Parameter for [DualNum] for internal use only.
 */
@Serializable class Internal

/**
 * Arclength parameter for [DualNum]
 */
@Serializable class Arclength

/**
 * Time parameter for [DualNum]
 */
@Serializable class Time

/**
 * @usesMathJax
 *
 * [Dual number](https://en.wikipedia.org/wiki/Dual_number) to implement forward autodifferentiation.
 *
 * @param[Param] \(x\)
 * @property[values] \(\left(u, \frac{du}{dx}, \frac{d^2u}{dx^2}, \ldots, \frac{d^{n - 1} u}{dx^{n - 1}} \right)\)
 */
@Serializable
class DualNum<Param> constructor(private val values: DoubleArray) {
    constructor(values: List<Double>) : this(values.toDoubleArray())

    companion object {
        /**
         * @usesMathJax
         *
         * Makes dual number \((c, 0, \ldots, 0)\) with size [n] representing a constant function \(c\).
         *
         * @param[Param] \(x\)
         * @param[c] \(c\)
         */
        @JvmStatic
        fun <Param> constant(c: Double, n: Int) = DualNum<Param>(
            DoubleArray(n) {
                when (it) {
                    0 -> c
                    else -> 0.0
                }
            }
        )

        /**
         * @usesMathJax
         *
         * Makes dual number \((x_0, 1, 0, \ldots, 0)\) with size [n] representing a variable function \(x\) at \(x = x_0\).
         *
         * @param[Param] \(x\)
         * @param[x0] \(x_0\)
         */
        @JvmStatic
        fun <Param> variable(x0: Double, n: Int) = DualNum<Param>(
            DoubleArray(n) {
                when (it) {
                    0 -> x0
                    1 -> 1.0
                    else -> 0.0
                }
            }
        )

        @JvmStatic
        fun <Param> cons(x: Double, d: DualNum<Param>) =
            DualNum<Param>(
                DoubleArray(d.size() + 1) {
                    if (it == 0) {
                        x
                    } else {
                        d.values[it - 1]
                    }
                }
            )
    }

    /**
     * @usesMathJax
     *
     * \(n\)
     */
    fun size() = values.size

    init {
        require(size() <= 4) {
            "Only derivatives up to 3rd order are supported"
        }
    }

    fun value() = values.first()
    operator fun get(i: Int) = values[i]
    fun values() = values.toList()

    fun drop(n: Int) = DualNum<Param>(DoubleArray(size() - n) { values[it + n] })

    operator fun plus(d: DualNum<Param>): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(min(size(), d.size())))
        for (i in out.values.indices) {
            out.values[i] = values[i] + d.values[i]
        }

        return out
    }

    operator fun minus(d: DualNum<Param>): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(min(size(), d.size())))
        for (i in out.values.indices) {
            out.values[i] = values[i] - d.values[i]
        }

        return out
    }

    operator fun times(d: DualNum<Param>): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(min(size(), d.size())))
        if (out.values.isEmpty()) return out

        out.values[0] = values[0] * d.values[0]
        if (out.size() == 1) return out

        out.values[1] = values[0] * d.values[1] + values[1] * d.values[0]
        if (out.size() == 2) return out

        out.values[2] = values[0] * d.values[2] + values[2] * d.values[0] +
            2 * values[1] * d.values[1]
        if (out.size() == 3) return out

        out.values[3] = values[0] * d.values[3] + values[3] * d.values[0] +
            3 * (values[2] * d.values[1] + values[1] * d.values[2])
        return out
    }

    operator fun unaryMinus(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size()))
        for (i in out.values.indices) {
            out.values[i] = -values[i]
        }

        return out
    }

    fun recip(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size()))
        if (out.values.isEmpty()) return out

        val recip = 1.0 / values[0]
        out.values[0] = recip
        if (out.size() == 1) return out

        val negRecip = -recip
        val negRecip2 = recip * negRecip
        val deriv = negRecip2 * values[1]
        out.values[1] = deriv
        if (out.size() == 2) return out

        val int1 = 2 * negRecip * deriv
        val deriv2 = int1 * values[1] + negRecip2 * values[2]
        out.values[2] = deriv2
        if (out.size() == 3) return out

        val int2 = int1 * values[2]
        out.values[3] =
            int2 + negRecip2 * values[3] +
            int2 - 2 * (deriv * deriv + recip * deriv2) * values[1]
        return out
    }

    operator fun div(d: DualNum<Param>) = this * d.recip()

    fun sqrt(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size()))
        if (out.values.isEmpty()) return out

        val sqrt = kotlin.math.sqrt(values[0])
        out.values[0] = sqrt
        if (out.size() == 1) return out

        val recip = 1 / (2 * sqrt)
        val deriv = recip * values[1]
        out.values[1] = deriv
        if (out.size() == 2) return out

        val negRecip = -2 * recip
        val negRecip2 = recip * negRecip
        val int1 = negRecip2 * deriv
        val secondDeriv = int1 * values[1] + recip * values[2]
        out.values[2] = secondDeriv
        if (out.size() == 3) return out

        val int2 = 2 * int1
        out.values[3] = recip * values[3] + int2 * values[2] +
            (deriv * negRecip * int2 + negRecip2 * secondDeriv) * values[1]

        return out
    }

    fun sin(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size()))
        if (out.values.isEmpty()) return out

        val sin = kotlin.math.sin(values[0])
        out.values[0] = sin
        if (out.size() == 1) return out

        val cos = kotlin.math.cos(values[0])
        val deriv = cos * values[1]
        out.values[1] = deriv
        if (out.size() == 2) return out

        val inDeriv2 = values[1] * values[1]
        out.values[2] = cos * values[2] - sin * inDeriv2
        if (out.size() == 3) return out

        out.values[3] = cos * values[3] -
            3 * sin * values[1] * values[2] -
            deriv * inDeriv2

        return out
    }

    fun cos(): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size()))
        if (out.values.isEmpty()) return out

        val cos = kotlin.math.cos(values[0])
        out.values[0] = cos
        if (out.size() == 1) return out

        val sin = kotlin.math.sin(values[0])
        val negInDeriv = -values[1]
        val deriv = sin * negInDeriv
        out.values[1] = deriv
        if (out.size() == 2) return out

        val int = cos * negInDeriv
        out.values[2] = int * values[1] - sin * values[2]
        if (out.size() == 3) return out

        out.values[3] = deriv * negInDeriv * values[1] +
            3 * int * values[2] -
            sin * values[3]

        return out
    }

    /**
     * @usesMathJax
     *
     * Reparameterizes \(\left(x, \frac{dx}{du}, \frac{d^2x}{du^2}, \ldots, \frac{d^{n - 1} x}{du^{n - 1}}\right)\) into
     * \(\left(x, \frac{dx}{dt}, \frac{d^2x}{dt^2}, \ldots, \frac{d^{n - 1} x}{dt^{n - 1}}\right)\) using [oldParam] and
     * the chain rule.
     *
     * @param[oldParam] \(\left(u, \frac{du}{dt}, \frac{d^2u}{dt^2}, \ldots, \frac{d^{n - 1} u}{dt^{n - 1}}\right)\)
     */
    fun <NewParam> reparam(oldParam: DualNum<NewParam>): DualNum<NewParam> {
        val out = DualNum<NewParam>(DoubleArray(min(size(), oldParam.size())))
        if (out.values.isEmpty()) return out

        out.values[0] = values[0]
        if (out.size() == 1) return out

        out.values[1] = values[1] * oldParam[1]
        if (out.size() == 2) return out

        val oldDeriv2 = oldParam.values[1] * oldParam.values[1]
        out.values[2] = oldDeriv2 * values[2] + oldParam.values[2] * values[1]
        if (out.size() == 3) return out

        out.values[3] = values[1] * oldParam.values[3] +
            (3 * values[2] * oldParam.values[2] + values[3] * oldDeriv2) * oldParam.values[1]

        return out
    }

    operator fun plus(c: Double) = DualNum<Param>(
        DoubleArray(size()) {
            when (it) {
                0 -> values[0] + c
                else -> values[it]
            }
        }
    )

    operator fun minus(c: Double) = DualNum<Param>(
        DoubleArray(size()) {
            when (it) {
                0 -> values[0] - c
                else -> values[it]
            }
        }
    )

    operator fun times(c: Double): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size()))
        for (i in out.values.indices) {
            out.values[i] = values[i] * c
        }

        return out
    }

    operator fun div(c: Double): DualNum<Param> {
        val out = DualNum<Param>(DoubleArray(size()))
        for (i in out.values.indices) {
            out.values[i] = values[i] / c
        }

        return out
    }

    operator fun times(c: Vector2d) =
        Vector2dDual(this * c.x, this * c.y)
}

fun <Param> lerpDual(x: Double, fromLo: Double, fromHi: Double, toLo: DualNum<Param>, toHi: DualNum<Param>): DualNum<Param> {
    require(toLo.size() == toHi.size()) { "Lower and upper bounds must have the same size" }

    return DualNum(
        DoubleArray(toLo.size()) {
            lerp(x, fromLo, fromHi, toLo[it], toHi[it])
        }
    )
}

fun <Param> atan2(y: DualNum<Param>, x: DualNum<Param>): DualNum<Param> {
    require(x.size() == y.size()) { "x and y must have the same size" }

    val out = DualNum<Param>(DoubleArray(x.size()))
    if (out.size() == 0) return out

    // Calculate the base atan2 value
    val atan2Val = atan2(y[0], x[0])
    val result = DoubleArray(x.size())
    result[0] = atan2Val
    if (out.size() == 1) return DualNum(result)

    // Calculate the denominator (x^2 + y^2)
    val denom = x[0] * x[0] + y[0] * y[0]

    // First derivative: (x * dy/dt - y * dx/dt) / (x^2 + y^2)
    result[1] = (x[0] * y[1] - y[0] * x[1]) / denom
    if (out.size() == 2) return DualNum(result)

    // For the second derivative, we need to use the quotient rule and chain rule
    // Let f(t) = x * dy/dt - y * dx/dt and g(t) = x^2 + y^2
    // Then d/dt(f/g) = (g * df/dt - f * dg/dt) / g^2

    // Calculate f(t) = x * dy/dt - y * dx/dt
    val f = x[0] * y[1] - y[0] * x[1]

    // Calculate df/dt = dx/dt * dy/dt + x * d2y/dt2 - dy/dt * dx/dt - y * d2x/dt2
    //                 = x * d2y/dt2 - y * d2x/dt2
    val dfdt = x[0] * y[2] - y[0] * x[2]

    // Calculate dg/dt = 2 * x * dx/dt + 2 * y * dy/dt
    val dgdt = 2 * (x[0] * x[1] + y[0] * y[1])

    // Second derivative: (g * df/dt - f * dg/dt) / g^2
    result[2] = (denom * dfdt - f * dgdt) / (denom * denom)
    if (out.size() == 3) return DualNum(result)

    // For the third derivative, we need to continue applying the quotient rule and chain rule
    // This gets very complex, so we'll use a more systematic approach

    // Let's define some helper variables
    val x0 = x[0]
    val y0 = y[0]
    val x1 = x[1]
    val y1 = y[1]
    val x2 = x[2]
    val y2 = y[2]
    val x3 = x[3]
    val y3 = y[3]

    // Calculate d2f/dt2 = d/dt(x * d2y/dt2 - y * d2x/dt2)
    //                    = dx/dt * d2y/dt2 + x * d3y/dt3 - dy/dt * d2x/dt2 - y * d3x/dt3
    val d2fdt2 = x1 * y2 + x0 * y3 - y1 * x2 - y0 * x3

    // Calculate d2g/dt2 = d/dt(2 * (x * dx/dt + y * dy/dt))
    //                    = 2 * (dx/dt * dx/dt + x * d2x/dt2 + dy/dt * dy/dt + y * d2y/dt2)
    val d2gdt2 = 2 * (x1 * x1 + x0 * x2 + y1 * y1 + y0 * y2)

    // Calculate d/dt(g * df/dt - f * dg/dt) using the product rule
    val numerator = denom * d2fdt2 + dgdt * dfdt - dgdt * dfdt - f * d2gdt2

    // Calculate d/dt(g^2) = 2 * g * dg/dt
    val denomDerivative = 2 * denom * dgdt

    // Apply the quotient rule for the third derivative
    result[3] = (denom * denom * numerator - (denom * dfdt - f * dgdt) * denomDerivative) / (denom * denom * denom)

    return DualNum(result)
}