@file:JvmName("PositionPaths")

package com.acmerobotics.roadrunner.paths

import com.acmerobotics.roadrunner.geometry.*
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

/**
 * @usesMathJax
 *
 * Path \((x(t), y(t))\)
 *
 * @param[Param] \(t\)
 */
interface PositionPath<Param> {

    /**
     * @usesMathJax
     *
     * @param[param] \(t\)
     */
    operator fun get(param: Double, n: Int): Vector2dDual<Param>

    fun length(): Double

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length(), n)

    fun project(query: Vector2d, init: Double = 0.0) = (1..10).fold(init) { s, _ ->
        val guess = this[s, 3]
        val ds = (query - guess.value()) dot guess.drop(1).value()
        clamp(s + ds, 0.0, this.length())
    }

    /**
     * Creates a PosePath using this path for position and its tangent for heading.
     */
    fun withTangentHeading() = TangentPath(this.wrtArclength(), 0.0)

    /**
     * Creates a PosePath using this path for position and constant [heading].
     */
    fun withConstantHeading(heading: Rotation2d) = this.wrtArclength().let {
        HeadingPosePath(
            it,
            ConstantHeadingPath(heading, it.length())
        )
    }

    /**
     * Creates a PosePath using this path for position and constant [heading].
     */
    fun withConstantHeading(heading: Double) = withConstantHeading(heading.toRotation())

    /**
     * Creates a PosePath using this path for position
     * and a linear interpolation between [startHeading] and [endHeading] for heading.
     */
    fun withLinearHeading(startHeading: Rotation2d, endHeading: Rotation2d) = this.wrtArclength().let {
        HeadingPosePath(
            it,
            LinearHeadingPath(startHeading, endHeading - startHeading, it.length())
            )
    }

    /**
     * Creates a PosePath using this path for position
     * and a linear interpolation between [startHeading] and [endHeading] for heading.
     */
    fun withLinearHeading(startHeading: Double, endHeading: Double) =
        withLinearHeading(startHeading.toRotation(), endHeading.toRotation())

    /**
     * Creates a PosePath using this path for position
     * and a linear interpolation between [startHeading] and [endHeading] for heading.
     */
    fun withSplineHeading(startHeading: Rotation2d, endHeading: Rotation2d) = this.wrtArclength().let {
        HeadingPosePath(
            it,
            SplineHeadingPath(
                startHeading.dual(),
                (endHeading-startHeading).toRotation().dual(),
                it.length()
            )
        )
    }

    /**
     * Creates a PosePath using this path for position
     * and a linear interpolation between [startHeading] and [endHeading] for heading.
     */
    fun withSplineHeading(startHeading: Double, endHeading: Double) =
        withSplineHeading(startHeading.toRotation(), endHeading.toRotation())
}

fun Double.toRotation() = Rotation2d.exp(this)
fun Rotation2d.dual() = Rotation2dDual.Companion.constant<Arclength>(this, 1)

/**
 * Project position [query] onto position path [path] starting with initial guess [init].
 */
fun project(path: PositionPath<Arclength>, query: Vector2d, init: Double): Double = path.project(query, init)

/**
 * Line beginning at position [begin], pointed in direction [dir], and having length [length].
 *
 * @param[dir] unit vector
 */
@Serializable
@SerialName("Line")
data class Line(
    @JvmField
    val begin: Vector2d,
    @JvmField
    val dir: Vector2d,
    @JvmField
    val length: Double,
) : PositionPath<Arclength> {

    /**
     * Makes line connecting [begin] to [end].
     */
    constructor(
        begin: Vector2d,
        end: Vector2d,
    ) : this(
        begin,
        (end - begin).let { diff ->
            val norm = diff.norm()
            if (norm < 1e-6) {
                Vector2d(1.0, 0.0)
            } else {
                diff / norm
            }
        },
        (end - begin).norm(),
    )

    override fun get(param: Double, n: Int) =
        DualNum.Companion.variable<Arclength>(param, n) * dir + begin

    override fun length() = length
}

/**
 * Arclength reparameterization of [curve].
 */
@Serializable
@SerialName("ArclengthReparamPath")
data class ArclengthReparamCurve2d(
    @JvmField
    val curve: PositionPath<*>,
    @JvmField
    val samples: IntegralScanResult,
) : PositionPath<Arclength> {

    /**
     * @param[eps] desired error in the approximate length [length]
     */
    constructor(
        curve: PositionPath<*>,
        eps: Double,
    ) : this(
        curve,
        integralScan(0.0, curve.length(), eps) {
            curve[it, 2].drop(1).value().norm()
        },
    )

    @JvmField
    val length = samples.sums.last()

    fun reparam(s: Double): Double {
        return lerpLookup(samples.sums, samples.values, s)
    }

    override fun get(param: Double, n: Int): Vector2dDual<Arclength> {
        val t = reparam(param)
        val point = curve[t, n]

        val tValues = DoubleArray(n)
        tValues[0] = t
        if (n <= 1) return point.reparam(DualNum(tValues))

        val tDerivs = point.drop(1).norm().recip()
        tValues[1] = tDerivs[0]
        if (n <= 2) return point.reparam(DualNum(tValues))

        tValues[2] = tDerivs.reparam(DualNum<Arclength>(tValues))[1]
        if (n <= 3) return point.reparam(DualNum(tValues))

        tValues[3] = tDerivs.reparam(DualNum<Arclength>(tValues))[2]
        return point.reparam(DualNum(tValues))
    }

    override fun length() = length
}

private fun <Param> PositionPath<Param>.wrtArclength(): PositionPath<Arclength> = when(this) {
    is Line -> this
    is ArclengthReparamCurve2d -> this
    else -> ArclengthReparamCurve2d(this, 1e-6)
}

@Serializable
@SerialName("CompositePositionPath")
data class CompositePositionPath<Param> @JvmOverloads constructor(
    @JvmField
    val paths: List<PositionPath<Param>>,
    @JvmField
    val offsets: List<Double> = paths.scan(0.0) { acc, path -> acc + path.length() },
) : PositionPath<Param> {
    @JvmField
    val length = offsets.last()

    init {
        require(paths.isNotEmpty()) {
            "Composite path must contain at least one path"
        }
    }

    override fun get(param: Double, n: Int): Vector2dDual<Param> {
        if (param > length) {
            return Vector2dDual.Companion.constant(paths.last().end(1).value(), n)
        }

        for ((offset, path) in offsets.zip(paths).reversed()) {
            if (param >= offset) {
                return path[param - offset, n]
            }
        }

        return Vector2dDual.Companion.constant(paths.first()[0.0, 1].value(), n)
    }

    override fun length() = length
}

@Serializable
@SerialName("PositionPathView")
data class PositionPathView<Param>(
    @JvmField
    val path: PositionPath<Param>,
    @JvmField
    val offset: Double,
    @JvmField
    val length: Double,
) : PositionPath<Param> {
    override fun get(param: Double, n: Int) = path[param + offset, n]

    override fun length() = length
}