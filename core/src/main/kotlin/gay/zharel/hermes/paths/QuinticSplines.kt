package gay.zharel.hermes.paths

import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.Internal
import gay.zharel.hermes.geometry.Vector2dDual
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

/**
 * @usesMathJax
 *
 * Quintic spline with equation \(a t^5 + b t^4 + c t^3 + d t^2 + e t + f\) where \(0 \leq t \leq 1\)
 *
 * @property[a] \(a\)
 * @property[b] \(b\)
 * @property[c] \(c\)
 * @property[d] \(d\)
 * @property[e] \(e\)
 * @property[f] \(f\)
 */
@Serializable
data class QuinticSpline1d(
    @JvmField
    val a: Double,
    @JvmField
    val b: Double,
    @JvmField
    val c: Double,
    @JvmField
    val d: Double,
    @JvmField
    val e: Double,
    @JvmField
    val f: Double,
) {

    /**
     * Fits a spline from [begin] at \(t = 0\) to [end] at \(t = 1\).
     */
    constructor(
        begin: DualNum<Internal>,
        end: DualNum<Internal>,
    ) : this(
        require(begin.size() >= 3).let {
            require(end.size() >= 3).let {
                -6.0 * begin[0] - 3.0 * begin[1] - 0.5 * begin[2] +
                        6.0 * end[0] - 3.0 * end[1] + 0.5 * end[2]
            }
        },
        15.0 * begin[0] + 8.0 * begin[1] + 1.5 * begin[2] -
                15.0 * end[0] + 7.0 * end[1] - end[2],
        -10.0 * begin[0] - 6.0 * begin[1] - 1.5 * begin[2] +
                10.0 * end[0] - 4.0 * end[1] + 0.5 * end[2],
        0.5 * begin[2],
        begin[1],
        begin[0],
    )

    val coefficients = listOf(a, b, c, d, e, f)

    /**
     * @usesMathJax
     *
     * @param[t] \(t\)
     */
    operator fun get(t: Double, n: Int) = DualNum<Internal>(
        DoubleArray(n) {
            when (it) {
                0 -> ((((a * t + b) * t + c) * t + d) * t + e) * t + f
                1 -> (((5.0 * a * t + 4.0 * b) * t + 3.0 * c) * t + 2.0 * d) * t + e
                2 -> ((20.0 * a * t + 12.0 * b) * t + 6.0 * c) * t + 2.0 * d
                3 -> (60.0 * a * t + 24.0 * b) * t + 6.0 * c
                4 -> 120.0 * a * t + 24.0 * b
                5 -> 120.0 * a
                else -> 0.0
            }
        }
    )
}

/**
 * Path comprised of two [QuinticSpline1d]s.
 */
@Serializable
@SerialName("QuinticSpline2d")
data class QuinticSpline2dInternal(
    @JvmField
    val x: QuinticSpline1d,
    @JvmField
    val y: QuinticSpline1d,
) : PositionPath<Internal> {
    constructor(begin: Vector2dDual<Internal>, end: Vector2dDual<Internal>) : this(
        QuinticSpline1d(begin.x, end.x),
        QuinticSpline1d(begin.y, end.y)
    )

    override fun get(param: Double, n: Int) = Vector2dDual(x[param, n], y[param, n])

    override fun length() = 1.0
}