package com.acmerobotics.roadrunner.trajectories

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.paths.IdentityPoseMap
import com.acmerobotics.roadrunner.paths.MappedPosePath
import com.acmerobotics.roadrunner.paths.PoseMap
import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.CompositeAccelConstraint
import com.acmerobotics.roadrunner.profiles.CompositeVelConstraint
import com.acmerobotics.roadrunner.profiles.ProfileParams
import com.acmerobotics.roadrunner.profiles.VelConstraint
import com.acmerobotics.roadrunner.profiles.profile



data class TrajectoryBuilderParams(
    val arcLengthSamplingEps: Double,
    val profileParams: ProfileParams,
)

class TrajectoryBuilder private constructor(
    private val profileParams: ProfileParams,
    private val pathBuilder: PathBuilder,
    private val beginEndVel: Double,
    private val baseVelConstraint: VelConstraint,
    private val baseAccelConstraint: AccelConstraint,
    private val poseMap: PoseMap,
    private val velConstraints: List<VelConstraint>,
    private val accelConstraints: List<AccelConstraint>,
) {
    @JvmOverloads
    constructor(
        params: TrajectoryBuilderParams,
        beginPose: Pose2d,
        beginEndVel: Double,
        baseVelConstraint: VelConstraint,
        baseAccelConstraint: AccelConstraint,
        poseMap: PoseMap = IdentityPoseMap,
    ) :
        this(
            params.profileParams,
            PathBuilder(beginPose, params.arcLengthSamplingEps),
            beginEndVel, baseVelConstraint, baseAccelConstraint,
            poseMap, listOf(), listOf()
        )

    private fun add(
        newPathBuilder: PathBuilder,
        velConstraintOverride: VelConstraint?,
        accelConstraintOverride: AccelConstraint?
    ) =
        TrajectoryBuilder(
            profileParams,
            newPathBuilder, beginEndVel, baseVelConstraint, baseAccelConstraint, poseMap,
            velConstraints + listOf(velConstraintOverride ?: baseVelConstraint),
            accelConstraints + listOf(accelConstraintOverride ?: baseAccelConstraint)
        )

    /**
     * Sets the starting tangent of the next path segment.
     * See [RoadRunner docs](https://rr.brott.dev/docs/v1-0/guides/tangents/).
     */
    fun setTangent(r: Rotation2d) =
        TrajectoryBuilder(
            profileParams,
            pathBuilder.setTangent(r), beginEndVel, baseVelConstraint, baseAccelConstraint,
            poseMap, velConstraints, accelConstraints,
        )

    /**
     * Sets the starting tangent of the next path segment.
     * See [RoadRunner docs](https://rr.brott.dev/docs/v1-0/guides/tangents/).
     */
    fun setTangent(r: Double) = setTangent(Rotation2d.Companion.exp(r))

    /**
     * Reverses the next path segment; actually a call to [setTangent(Math.PI)][setTangent]!
     */
    fun setReversed(reversed: Boolean) =
        TrajectoryBuilder(
            profileParams,
            pathBuilder.setReversed(reversed), beginEndVel, baseVelConstraint, baseAccelConstraint,
            poseMap, velConstraints, accelConstraints,
        )

    /**
     * Adds a line segment that goes forward [ds].
     */
    @JvmOverloads
    fun forward(
        ds: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forward(ds), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes forward [ds] while maintaining current heading.
     * Equivalent to [forward].
     */
    @JvmOverloads
    fun forwardConstantHeading(
        ds: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forwardConstantHeading(ds), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes forward [ds],
     * changing heading from current to [heading] using linear interpolation.
     */
    @JvmOverloads
    fun forwardLinearHeading(
        ds: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forwardLinearHeading(ds, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes forward [ds],
     * changing heading from current to [heading] using linear interpolation.
     */
    @JvmOverloads
    fun forwardLinearHeading(
        ds: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forwardLinearHeading(ds, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes forward [ds],
     * changing heading from current to [heading] using spline interpolation.
     */
    @JvmOverloads
    fun forwardSplineHeading(
        ds: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forwardSplineHeading(ds, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes forward [ds],
     * changing heading from current to [heading] using spline interpolation.
     */
    @JvmOverloads
    fun forwardSplineHeading(
        ds: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.forwardSplineHeading(ds, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(x\)-coordinate [posX].
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToX(posX), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(x\)-coordinate [posX].
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
     * Equivalent to [lineToX].
     */
    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXConstantHeading(posX), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(x\)-coordinate [posX],
     * while changing heading from current to [heading] using linear interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
    */
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXLinearHeading(posX, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(x\)-coordinate [posX],
     * while changing heading from current to [heading] using linear interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXLinearHeading(posX, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(x\)-coordinate [posX],
     * while changing heading from current to [heading] using spline interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXSplineHeading(posX, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(x\)-coordinate [posX],
     * while changing heading from current to [heading] using spline interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToXSplineHeading(posX, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(y\)-coordinate [posY].
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToY(posY), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(y\)-coordinate [posY].
     * * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     * Equivalent to [lineToY].
     */
    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYConstantHeading(posY), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(y\)-coordinate [posY],
     * while changing heading from current to [heading] using linear interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYLinearHeading(posY, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(y\)-coordinate [posY],
     * while changing heading from current to [heading] using linear interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYLinearHeading(posY, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(y\)-coordinate [posY],
     * while changing heading from current to [heading] using spline interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYSplineHeading(posY, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to \(y\)-coordinate [posY],
     * while changing heading from current to [heading] using spline interpolation.
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     */
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.lineToYSplineHeading(posY, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to [pos].
     */
    @JvmOverloads
    fun strafeTo(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeTo(pos), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to [pos].
     * Equivalent to [strafeTo].
     */
    @JvmOverloads
    fun strafeToConstantHeading(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeToConstantHeading(pos), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using linear interpolation.
     */
    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeToLinearHeading(pos, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using linear interpolation.
     */
    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeToLinearHeading(pos, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using spline interpolation.
     */
    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeToSplineHeading(pos, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using spline interpolation.
     */
    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.strafeToSplineHeading(pos, heading), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     */
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineTo(pos, tangent), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     */
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineTo(pos, tangent), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     * The robot's heading remains constant
     * as opposed to matching the tangent.
     */
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToConstantHeading(pos, tangent), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     * The robot's heading remains constant
     * as opposed to matching the tangent.
     */
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToConstantHeading(pos, tangent), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.position][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading linearly interpolates from its current heading
     * to [pose.heading][pose].
     */
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToLinearHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.position][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading linearly interpolates from its current heading
     * to [pose.heading][pose].
     */
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToLinearHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)


    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.position][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading interpolates from its current heading
     * to [pose.heading][pose] using spline interpolation.
     */
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToSplineHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.position][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading interpolates from its current heading
     * to [pose.heading][pose] using spline interpolation.
     */
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        add(pathBuilder.splineToSplineHeading(pose, tangent), velConstraintOverride, accelConstraintOverride)

    /**
     * Builds the specified trajectories, creating a new CancelableTrajectory
     * object for each discontinuity.
     * @return the resulting list of CancelableTrajectory objects
     */
    fun buildToList(): List<CancelableTrajectory> {
        val rawPaths = pathBuilder.build()
        val offsets = rawPaths.scan(0) { acc, rawPath -> acc + rawPath.paths.size }
        return rawPaths.zip(offsets).map { (rawPath, offset) ->
            val path = MappedPosePath(rawPath, poseMap)

            CancelableTrajectory(
                path,
                profile(
                    profileParams,
                    path, beginEndVel,
                    CompositeVelConstraint(
                        velConstraints.slice(offset until offset + rawPath.paths.size),
                        rawPath.offsets,
                    ),
                    CompositeAccelConstraint(
                        accelConstraints.slice(offset until offset + rawPath.paths.size),
                        rawPath.offsets,
                    ),
                ),
                rawPath.offsets
            )
        }
    }

    /**
     * Builds the trajectory, creating a new CancelableTrajectory object
     * for each discontinuity, and then composes them into a single CompositeTrajectory object.
     * There may be complete stops in this composite based on where its components start and end.
     * @return the resulting CompositeTrajectory object
     */
    fun buildToComposite() = CompositeTrajectory(buildToList())

    /**
     * Builds the specified trajectories, creating a new CancelableTrajectory
     * object for each discontinuity.
     * Equivalent to [buildToList].
     * @return the resulting list of CancelableTrajectory objects
     */
    fun build() = buildToList()
}