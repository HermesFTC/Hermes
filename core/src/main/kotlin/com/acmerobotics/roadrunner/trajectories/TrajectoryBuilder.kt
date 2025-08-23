package com.acmerobotics.roadrunner.trajectories

import com.acmerobotics.roadrunner.geometry.Arclength
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
    private val markers: List<Marker> = listOf(),
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
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading follows the path tangent.
     */
    @JvmOverloads
    fun bezierTo(
        controlPoints: List<Vector2d>,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = add(
        pathBuilder.bezierTo(controlPoints),
        velConstraintOverride,
        accelConstraintOverride
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     */
    fun bezierTo(
        vararg controlPoints: Vector2d,
    ) = add(
        pathBuilder.bezierTo(*controlPoints),
        null,
        null
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading remains constant at its current heading.
     */
    @JvmOverloads
    fun bezierToConstantHeading(
        controlPoints: List<Vector2d>,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = add(
        pathBuilder.bezierToConstantHeading(controlPoints),
        velConstraintOverride,
        accelConstraintOverride
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading is linearly interpolated
     * from its current heading to [heading].
     */
    @JvmOverloads
    fun bezierToLinearHeading(
        controlPoints: List<Vector2d>,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = add(
        pathBuilder.bezierToLinearHeading(controlPoints, heading),
        velConstraintOverride,
        accelConstraintOverride
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading is linearly interpolated
     * from its current heading to [heading].
     */
    @JvmOverloads
    fun bezierToLinearHeading(
        controlPoints: List<Vector2d>,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = add(
        pathBuilder.bezierToLinearHeading(controlPoints, heading),
        velConstraintOverride,
        accelConstraintOverride
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading is interpolated from
     * its current heading to [heading] using a spline.
     */
    @JvmOverloads
    fun bezierToSplineHeading(
        controlPoints: List<Vector2d>,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = add(
        pathBuilder.bezierToSplineHeading(controlPoints, heading),
        velConstraintOverride,
        accelConstraintOverride
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading is interpolated from
     * its current heading to [heading] using a spline.
     */
    @JvmOverloads
    fun bezierToSplineHeading(
        controlPoints: List<Vector2d>,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = add(
        pathBuilder.bezierToSplineHeading(controlPoints, heading),
        velConstraintOverride,
        accelConstraintOverride
    )

    /**
     * Adds a marker to the trajectory builder.
     * @param marker The [Marker] to add.
     * @return A new [TrajectoryBuilder] with the marker added.
     */
    fun addMarker(marker: Marker): TrajectoryBuilder =
        TrajectoryBuilder(
            profileParams,
            pathBuilder, beginEndVel, baseVelConstraint, baseAccelConstraint, poseMap,
            velConstraints, accelConstraints,
            markers + marker
        )

    /**
     * Adds a marker to the trajectory at the current position.
     * @param trigger The marker trigger function.
     * @param callback The marker callback function.
     * @return A new [TrajectoryBuilder] with the marker added.
     */
    fun addMarker(trigger: MarkerTrigger, callback: MarkerCallback) =
        addMarker(Marker(trigger, callback))

    /**
     * Adds a marker that triggers after a specified displacement.
     * @param disp The displacement value at which to trigger the marker.
     * @param callback The callback to execute when triggered.
     * @return A new [TrajectoryBuilder] with the marker added.
     */
    fun addDispMarker(disp: Double, callback: MarkerCallback) =
        addMarker(Marker.afterDisp(disp, callback))

    /**
     * Adds a marker that triggers after a specified time.
     * @param time The time value at which to trigger the marker.
     * @param callback The callback to execute when triggered.
     * @return A new [TrajectoryBuilder] with the marker added.
     */
    fun addTimeMarker(time: Double, callback: MarkerCallback) =
        addMarker(Marker.afterTime(time, callback))

    /**
     * Adds a marker that triggers when the robot is within a certain tolerance of a given point.
     * @param point The [Vector2d] point to check proximity against.
     * @param tolerance The distance tolerance for triggering the marker (default: 2.0 units).
     * @param callback The callback to execute when triggered.
     * @return A new [TrajectoryBuilder] with the marker added.
     */
    fun addPointMarker(point: Vector2d, tolerance: Double = 2.0, callback: MarkerCallback) =
        addMarker(Marker.atPoint(point, tolerance, callback))

    /**
     * Adds a marker that triggers when the robot is within a certain linear and angular tolerance of a given pose.
     * @param pose The [Pose2d] to check proximity and orientation against.
     * @param linearTolerance The distance tolerance for triggering the marker (default: 2.0 units).
     * @param angularTolerance The angular tolerance in radians for triggering the marker (default: 5 degrees).
     * @param callback The callback to execute when triggered.
     * @return A new [TrajectoryBuilder] with the marker added.
     */
    fun addPoseMarker(pose: Pose2d, linearTolerance: Double = 2.0, angularTolerance: Double = Math.toRadians(5.0), callback: MarkerCallback) =
        addMarker(Marker.atPose(pose, linearTolerance, angularTolerance, callback))

    /**
     * Builds the specified trajectories, creating a new CancelableTrajectory
     * object for each discontinuity.
     * Returns a [TrajectoryWithMarkers] using a [CompositeCancelableTrajectory] as the base trajectory.
     * @return the resulting [TrajectoryWithMarkers] object
     */
    fun build(): TrajectoryWithMarkers<Arclength> =
        TrajectoryWithMarkers(buildToComposite(), markers)

    /**
     * Builds the specified trajectories, creating a new CancelableTrajectory
     * object for each discontinuity.
     * This does not include markers.
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
     * Builds the specified trajectories,
     * creating a new CancelableTrajectory for each discontinuity,
     * and then packing them into a [CompositeCancelableTrajectory] object.
     * This does not include any markers.
     * @return the resulting [CompositeCancelableTrajectory] object
     */
    fun buildToComposite() = CompositeCancelableTrajectory(buildToList())
}