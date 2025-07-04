@file:JvmName("PosePaths")

package com.acmerobotics.roadrunner.paths

import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.Vector2dDual
import com.acmerobotics.roadrunner.geometry.clamp

/**
 * @usesMathJax
 *
 * Pose path \((x(s), y(s), \theta(s))\)
 */
interface PosePath {
    operator fun get(s: Double, n: Int): Pose2dDual<Arclength>

    fun length(): Double

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length(), n)

    fun project(query: Vector2d, init: Double = 0.0) = (1..10).fold(init) { s, _ ->
        val guess = this[s, 3].position.bind()
        val ds = (query - guess.value()) dot guess.drop(1).value()
        clamp(s + ds, 0.0, this.length())
    }

    fun map(map: PoseMap) = MappedPosePath(this, map)

    operator fun plus(other: PosePath) = CompositePosePath(listOf(this, other))
}

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

data class CompositePosePath @JvmOverloads constructor(
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

object IdentityPoseMap : PoseMap {
    override fun map(pose: Pose2dDual<Arclength>) = pose
}

data class MappedPosePath(
    val basePath: PosePath,
    val poseMap: PoseMap,
) : PosePath {
    override fun length() = basePath.length()
    override fun get(s: Double, n: Int) = poseMap.map(basePath[s, n])
}
