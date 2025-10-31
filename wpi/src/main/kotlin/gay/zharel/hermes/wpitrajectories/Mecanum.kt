/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.wpitrajectories

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.trajectory.MecanumSample
import edu.wpi.first.math.trajectory.MecanumTrajectory
import edu.wpi.first.units.measure.LinearVelocity
import gay.zharel.hermes.paths.IdentityPoseMap
import gay.zharel.hermes.paths.PoseMap
import gay.zharel.hermes.profiles.AccelConstraint
import gay.zharel.hermes.profiles.VelConstraint
import gay.zharel.hermes.trajectories.CompositeCancelableTrajectory
import gay.zharel.hermes.trajectories.TrajectoryBuilder
import gay.zharel.hermes.trajectories.TrajectoryBuilderParams
import gay.zharel.hermes.wpiconversions.hermes
import gay.zharel.hermes.wpiconversions.ips
import gay.zharel.hermes.wpiconversions.mps

/**
 * A trajectory builder for mecanum drive robots that uses WPILib geometry types.
 *
 * Mecanum drives are holonomic and can control their heading independently of their
 * path direction. This builder supports constant heading, linear heading interpolation,
 * and spline-based heading control.
 *
 * Example usage:
 * ```java
 * var kinematics = new MecanumDriveKinematics(
 *     frontLeft, frontRight, backLeft, backRight
 * );
 * var constraint = new MecanumDriveConstraint(
 *     new Translation2d[] { frontLeft, frontRight, backLeft, backRight },
 *     new SimpleMotorFeedforward(kS, kV, kA),
 *     Units.MetersPerSecond.of(maxVelocity),
 *     Units.Volts.of(12.0)
 * );
 *
 * var trajectory = new MecanumTrajectoryBuilder(kinematics, constraint, startPose)
 *     .strafeToLinearHeading(new Translation2d(24.0, 24.0), Rotation2d.fromDegrees(90.0))
 *     .splineToConstantHeading(new Translation2d(48.0, 24.0), 0.0)
 *     .build();
 * ```
 *
 * @see MecanumTrajectory
 */
class MecanumTrajectoryBuilder internal constructor(
    private val builder: TrajectoryBuilder,
    private val kinematics: MecanumDriveKinematics
) {
    /**
     * Creates a new MecanumTrajectoryBuilder with the specified parameters.
     *
     * @param kinematics The mecanum drive kinematics object
     * @param constraint The velocity and acceleration constraints for the drive
     * @param startPose The starting pose of the trajectory (WPILib Pose2d)
     * @param params Trajectory builder parameters (arc length sampling and profile params)
     * @param beginEndVel The starting/ending velocity of the trajectory (default: 0)
     * @param poseMap Optional pose transformation map for custom coordinate systems
     */
    @JvmOverloads constructor(
        kinematics: MecanumDriveKinematics,
        constraint: MecanumDriveConstraint,
        startPose: Pose2d,
        params: TrajectoryBuilderParams = DEFAULT_BUILDER_PARAMS,
        beginEndVel: LinearVelocity = 0.0.mps,
        poseMap: PoseMap = IdentityPoseMap,
    ) : this(
        TrajectoryBuilder(
            params = params,
            beginPose = startPose.hermes,
            beginEndVel = beginEndVel.ips,
            baseVelConstraint = constraint,
            baseAccelConstraint = constraint,
            poseMap = poseMap
        ),
        kinematics
    )

    /**
     * Sets the starting tangent of the next path segment.
     * See [RoadRunner docs](https://rr.brott.dev/docs/v1-0/guides/tangents/).
     */
    fun setTangent(r: Rotation2d) =
        MecanumTrajectoryBuilder(builder.setTangent(r.hermes), kinematics)

    /**
     * Sets the starting tangent of the next path segment.
     * See [RoadRunner docs](https://rr.brott.dev/docs/v1-0/guides/tangents/).
     */
    fun setTangent(r: Double) =
        MecanumTrajectoryBuilder(builder.setTangent(r), kinematics)

    /**
     * Reverses the next path segment; actually a call to [setTangent(Math.PI)][setTangent]!
     */
    fun setReversed(reversed: Boolean) =
        MecanumTrajectoryBuilder(builder.setReversed(reversed), kinematics)

    /**
     * Adds a line segment that goes forward [ds].
     */
    @JvmOverloads
    fun forward(
        ds: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.forward(ds, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.forwardConstantHeading(ds, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.forwardLinearHeading(ds, heading.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.forwardLinearHeading(ds, heading, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.forwardSplineHeading(ds, heading.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.forwardSplineHeading(ds, heading, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToX(posX, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToXConstantHeading(posX, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToXLinearHeading(posX, heading.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToXLinearHeading(posX, heading, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToXSplineHeading(posX, heading.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToXSplineHeading(posX, heading, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToY(posY, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToYConstantHeading(posY, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToYLinearHeading(posY, heading.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToYLinearHeading(posY, heading, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToYSplineHeading(posY, heading.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

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
        MecanumTrajectoryBuilder(builder.lineToYSplineHeading(posY, heading, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a line segment that goes to [pos].
     */
    @JvmOverloads
    fun strafeTo(
        pos: Translation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.strafeTo(pos.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a line segment that goes to [pos].
     * Equivalent to [strafeTo].
     */
    @JvmOverloads
    fun strafeToConstantHeading(
        pos: Translation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.strafeToConstantHeading(pos.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using linear interpolation.
     */
    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Translation2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.strafeToLinearHeading(pos.hermes, heading.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using linear interpolation.
     */
    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Translation2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.strafeToLinearHeading(pos.hermes, heading, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using spline interpolation.
     */
    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Translation2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.strafeToSplineHeading(pos.hermes, heading.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a line segment that goes to [pos],
     * changing heading from current to [heading] using spline interpolation.
     */
    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Translation2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.strafeToSplineHeading(pos.hermes, heading, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     */
    @JvmOverloads
    fun splineTo(
        pos: Translation2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.splineTo(pos.hermes, tangent.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     */
    @JvmOverloads
    fun splineTo(
        pos: Translation2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.splineTo(pos.hermes, tangent, velConstraintOverride, accelConstraintOverride), kinematics)

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
        pos: Translation2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.splineToConstantHeading(pos.hermes, tangent.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

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
        pos: Translation2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.splineToConstantHeading(pos.hermes, tangent, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.translation][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading linearly interpolates from its current heading
     * to [pose.rotation][pose].
     */
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.splineToLinearHeading(pose.hermes, tangent.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.translation][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading linearly interpolates from its current heading
     * to [pose.rotation][pose].
     */
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.splineToLinearHeading(pose.hermes, tangent, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.translation][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading interpolates from its current heading
     * to [pose.rotation][pose] using spline interpolation.
     */
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.splineToSplineHeading(pose.hermes, tangent.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pose.translation][pose] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending position and [tangent].
     * The robot's heading interpolates from its current heading
     * to [pose.rotation][pose] using spline interpolation.
     */
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        MecanumTrajectoryBuilder(builder.splineToSplineHeading(pose.hermes, tangent, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     * The robot's heading follows the path tangent.
     */
    @JvmOverloads
    fun bezierTo(
        controlPoints: List<Translation2d>,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = MecanumTrajectoryBuilder(
        builder.bezierTo(controlPoints.map { it.hermes }, velConstraintOverride, accelConstraintOverride),
        kinematics
    )

    /**
     * Adds a curved path using a Bezier curve,
     * with the current pose of this builder
     * as the first control point and [controlPoints]
     * as the remaining control points.
     */
    fun bezierTo(
        vararg controlPoints: Translation2d,
    ) = MecanumTrajectoryBuilder(
        builder.bezierTo(*controlPoints.map { it.hermes }.toTypedArray()),
        kinematics
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
        controlPoints: List<Translation2d>,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = MecanumTrajectoryBuilder(
        builder.bezierToConstantHeading(controlPoints.map { it.hermes }, velConstraintOverride, accelConstraintOverride),
        kinematics
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
        controlPoints: List<Translation2d>,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = MecanumTrajectoryBuilder(
        builder.bezierToLinearHeading(controlPoints.map { it.hermes }, heading.hermes, velConstraintOverride, accelConstraintOverride),
        kinematics
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
        controlPoints: List<Translation2d>,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = MecanumTrajectoryBuilder(
        builder.bezierToLinearHeading(controlPoints.map { it.hermes }, heading, velConstraintOverride, accelConstraintOverride),
        kinematics
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
        controlPoints: List<Translation2d>,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = MecanumTrajectoryBuilder(
        builder.bezierToSplineHeading(controlPoints.map { it.hermes }, heading.hermes, velConstraintOverride, accelConstraintOverride),
        kinematics
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
        controlPoints: List<Translation2d>,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = MecanumTrajectoryBuilder(
        builder.bezierToSplineHeading(controlPoints.map { it.hermes }, heading, velConstraintOverride, accelConstraintOverride),
        kinematics
    )

    /**
     * Builds the trajectory, returning a `MecanumTrajectory` object.
     */
    fun build() = MecanumTrajectory(
        kinematics,
        builder.build().wrtTime().sample().map { MecanumSample(it, kinematics) }.toTypedArray()
    )

    /**
     * Builds the trajectory,
     * returning the [CompositeCancelableTrajectory] object for use with other Hermes core functions.
     * The returned trajectory can be smoothly canceled at any time.
     */
    fun buildToComposite(): CompositeCancelableTrajectory = builder.buildToComposite()
}

