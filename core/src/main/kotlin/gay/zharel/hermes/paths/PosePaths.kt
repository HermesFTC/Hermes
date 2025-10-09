@file:JvmName("PosePaths")

package gay.zharel.hermes.paths

import gay.zharel.hermes.geometry.*
import gay.zharel.hermes.math.Arclength
import gay.zharel.hermes.math.clamp
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable


/**
 * @usesMathJax
 *
 * Represents a parametric 2D pose path \((x(s), y(s), \theta(s))\) with automatic differentiation support.
 *
 * A pose path extends a position path by including heading/orientation information. The path
 * is parameterized by arc length [Arclength] and can be evaluated to obtain pose (position + heading)
 * and derivatives at any point along the path.
 *
 * Pose paths are fundamental building blocks for robot trajectory planning, as they define
 * the spatial configuration the robot should follow.
 */
interface PosePath {
    /**
     * @usesMathJax
     *
     * Evaluates the pose path at arc length \(s\) with derivatives up to order n.
     *
     * @param s The arc length parameter at which to evaluate the path
     * @param n The number of derivatives to compute (0 for pose only, 1 for velocity, etc.)
     * @return A [Pose2dDual] containing the pose and up to n derivatives with respect to arc length
     */
    operator fun get(s: Double, n: Int): Pose2dDual<Arclength>

    /**
     * Returns the total arc length of the path.
     */
    fun length(): Double

    /**
     * Evaluates the pose path at the beginning (arc length = 0).
     *
     * @param n The number of derivatives to compute
     * @return The pose and derivatives at the start of the path
     */
    fun begin(n: Int) = get(0.0, n)

    /**
     * Evaluates the pose path at the end (arc length = length).
     *
     * @param n The number of derivatives to compute
     * @return The pose and derivatives at the end of the path
     */
    fun end(n: Int) = get(length(), n)

    /**
     * Projects a query position onto the path using Newton's method.
     *
     * Finds the arc length parameter where the path's position component is closest
     * to the query point. Note that this projection only considers position, not heading.
     *
     * @param query The 2D position to project onto the path
     * @param init Initial guess for the arc length parameter (default: 0.0)
     * @return The arc length parameter at the closest point on the path to the query
     */
    fun project(query: Vector2d, init: Double = 0.0) = (1..10).fold(init) { s, _ ->
        val guess = this[s, 3].position.bind()
        val ds = (query - guess.value()) dot guess.drop(1).value()
        clamp(s + ds, 0.0, this.length())
    }

    /**
     * Applies a pose transformation map to this path.
     *
     * @param map The [PoseMap] transformation to apply
     * @return A new [MappedPosePath] with the transformation applied
     */
    fun map(map: PoseMap) = MappedPosePath(this, map)

    /**
     * Combines this path with another path to create a composite path.
     *
     * The resulting path will traverse this path first, then the other path.
     *
     * @param other The path to append after this one
     * @return A [CompositePosePath] containing both paths in sequence
     */
    operator fun plus(other: PosePath) = CompositePosePath(listOf(this, other))
}

@Serializable
@SerialName("TangentPath")
data class TangentPath(
    @JvmField
    val path: PositionPath<Arclength>,
    @JvmField
    val offset: Double
) : PosePath {
    // NOTE: n+1 guarantees enough derivatives for tangent
    override operator fun get(s: Double, n: Int) = path[s, n + 1].let {
        Pose2dDual(it, it.drop(1).angleCast() + offset)
    }

    override fun length() = path.length()
}

@Serializable
@SerialName("HeadingPath")
data class HeadingPosePath(
    @JvmField
    val posPath: PositionPath<Arclength>,
    @JvmField
    val headingPath: HeadingPath,
) : PosePath {
    init {
        require(posPath.length() == headingPath.length()) {
            "posPath.length (${posPath.length()}) != headingPath.length (${headingPath.length()})"
        }
    }

    override fun get(s: Double, n: Int) =
        Pose2dDual(posPath[s, n], headingPath[s, n])

    override fun length() = posPath.length()
}

@Serializable
@SerialName("CompositePosePath")
data class CompositePosePath(
    @JvmField
    val paths: List<PosePath>,
    @JvmField
    val offsets: List<Double> = paths.scan(0.0) { acc, path -> acc + path.length() },
) : PosePath {
    init {
        require(paths.size + 1 == offsets.size) {
            "paths.size (${paths.size}) + 1 != offsets.size (${offsets.size})"
        }
    }

    @JvmField
    val length = offsets.last()

    override fun get(s: Double, n: Int): Pose2dDual<Arclength> {
        if (s > length) {
            return Pose2dDual.Companion.constant(paths.last().end(1).value(), n)
        }

        for ((offset, path) in offsets.zip(paths).reversed()) {
            if (s >= offset) {
                return path[s - offset, n]
            }
        }

        return Pose2dDual.Companion.constant(paths.first()[0.0, 1].value(), n)
    }

    override fun length() = length

    override fun plus(other: PosePath) = when (other) {
        is CompositePosePath -> CompositePosePath(this.paths + other.paths)
        else -> CompositePosePath(paths + other)
    }
}

@Serializable
@SerialName("OffsetPosePath")
class OffsetPosePath(
    @JvmField
    val path: PosePath,
    @JvmField
    val offsets: List<Double> = listOf(0.0, path.length())
) : PosePath {
    init {
        require(offsets.size == 2) {
            "offsets.size (${offsets.size}) != 2"
        }
    }

    override fun get(s: Double, n: Int): Pose2dDual<Arclength> {
        val offset = offsets[0]
        val length = offsets[1] - offsets[0]
        return path[s * length + offset, n]
    }

    override fun length() = offsets[1] - offsets[0]
}

class TurnPath(
    @JvmField
    val position: Vector2d,
    @JvmField
    val headingPath: HeadingPath
) : PosePath {
    constructor(start: Pose2d, endAngle: Rotation2d) : this(
        start.position,
        LinearHeadingPath(
            start.heading,
            endAngle.log(),
            start.heading - endAngle
        )
    )

    override fun get(s: Double, n: Int): Pose2dDual<Arclength> =
        Pose2dDual(Vector2dDual.constant(position, n), headingPath[s, n])

    override fun length(): Double = headingPath.length()

}

fun interface PoseMap {
    fun map(pose: Pose2dDual<Arclength>): Pose2dDual<Arclength>

    fun map(pose: Pose2d) = map(Pose2dDual.constant(pose, 1)).value()
}

@Serializable
@SerialName("IdentityPoseMap")
object IdentityPoseMap : PoseMap {
    override fun map(pose: Pose2dDual<Arclength>) = pose
}

@Serializable
@SerialName("MappedPosePath")
data class MappedPosePath(
    val basePath: PosePath,
    val poseMap: PoseMap,
) : PosePath {
    override fun length() = basePath.length()
    override fun get(s: Double, n: Int) = poseMap.map(basePath[s, n])
}
