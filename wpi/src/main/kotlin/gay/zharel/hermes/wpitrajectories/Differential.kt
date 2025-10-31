package gay.zharel.hermes.wpitrajectories

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.trajectory.DifferentialSample
import edu.wpi.first.math.trajectory.DifferentialTrajectory
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
 * A trajectory builder for differential drive robots that uses WPILib geometry types.
 *
 * Differential drives are non-holonomic and cannot control their heading independently
 * of their path tangent. The robot's heading will always follow the direction of travel.
 *
 * Example usage:
 * ```java
 * var kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(trackWidth));
 * var constraint = new DifferentialDriveConstraint(
 *     Units.inchesToMeters(trackWidth),
 *     new SimpleMotorFeedforward(kS, kV, kA),
 *     Units.MetersPerSecond.of(maxVelocity),
 *     Units.Volts.of(12.0)
 * );
 *
 * var trajectory = new DifferentialTrajectoryBuilder(kinematics, constraint, startPose)
 *     .forward(24.0)
 *     .splineTo(new Translation2d(48.0, 24.0), 0.0)
 *     .forward(12.0)
 *     .build();
 * ```
 *
 * @see DifferentialTrajectory
 */
class DifferentialTrajectoryBuilder internal constructor(
    private val builder: TrajectoryBuilder,
    private val kinematics: DifferentialDriveKinematics
) {
    /**
     * Creates a new DifferentialTrajectoryBuilder with the specified parameters.
     *
     * @param kinematics The differential drive kinematics object
     * @param constraint The velocity and acceleration constraints for the drive
     * @param startPose The starting pose of the trajectory (WPILib Pose2d)
     * @param params Trajectory builder parameters (arc length sampling and profile params)
     * @param beginEndVel The starting/ending velocity of the trajectory (default: 0)
     * @param poseMap Optional pose transformation map for custom coordinate systems
     */
    @JvmOverloads constructor(
        kinematics: DifferentialDriveKinematics,
        constraint: DifferentialDriveConstraint,
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
        DifferentialTrajectoryBuilder(builder.setTangent(r.hermes), kinematics)

    /**
     * Sets the starting tangent of the next path segment.
     * See [RoadRunner docs](https://rr.brott.dev/docs/v1-0/guides/tangents/).
     */
    fun setTangent(r: Double) =
        DifferentialTrajectoryBuilder(builder.setTangent(r), kinematics)

    /**
     * Reverses the next path segment; actually a call to [setTangent(Math.PI)][setTangent]!
     */
    fun setReversed(reversed: Boolean) =
        DifferentialTrajectoryBuilder(builder.setReversed(reversed), kinematics)

    /**
     * Adds a line segment that goes forward [ds].
     * The robot's heading follows the path tangent.
     */
    @JvmOverloads
    fun forward(
        ds: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        DifferentialTrajectoryBuilder(builder.forward(ds, velConstraintOverride, accelConstraintOverride), kinematics)


    /**
     * Adds a line segment that goes to \(x\)-coordinate [posX].
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the x-axis, this throws an error.
     * The robot's heading follows the path tangent.
     */
    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        DifferentialTrajectoryBuilder(builder.lineToX(posX, velConstraintOverride, accelConstraintOverride), kinematics)


    /**
     * Adds a line segment that goes to \(y\)-coordinate [posY].
     * The robot will continue traveling in the direction it is currently in;
     * if the robot is perpendicular to the y-axis, this throws an error.
     * The robot's heading follows the path tangent.
     */
    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        DifferentialTrajectoryBuilder(builder.lineToY(posY, velConstraintOverride, accelConstraintOverride), kinematics)


    /**
     * Adds a line segment that goes to [pos].
     * The robot's heading follows the path tangent.
     */
    @JvmOverloads
    fun strafeTo(
        pos: Translation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        DifferentialTrajectoryBuilder(builder.strafeTo(pos.hermes, velConstraintOverride, accelConstraintOverride), kinematics)


    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     * The robot's heading follows the path tangent.
     */
    @JvmOverloads
    fun splineTo(
        pos: Translation2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        DifferentialTrajectoryBuilder(builder.splineTo(pos.hermes, tangent.hermes, velConstraintOverride, accelConstraintOverride), kinematics)

    /**
     * Adds a curved path segment using quintic Hermite splines
     * that goes to [pos] with an end tangent of [tangent].
     * The shape of the curve is based off of the starting position and tangent
     * as well as the ending [pos] and [tangent].
     * The robot's heading follows the path tangent.
     */
    @JvmOverloads
    fun splineTo(
        pos: Translation2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) =
        DifferentialTrajectoryBuilder(builder.splineTo(pos.hermes, tangent, velConstraintOverride, accelConstraintOverride), kinematics)


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
    ) = DifferentialTrajectoryBuilder(
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
    ) = DifferentialTrajectoryBuilder(
        builder.bezierTo(*controlPoints.map { it.hermes }.toTypedArray()),
        kinematics
    )

    /**
     * Builds the trajectory, returning a [DifferentialTrajectory] object.
     */
    fun build(): DifferentialTrajectory = DifferentialTrajectory(
        builder.build().wrtTime().sample().map { DifferentialSample(it, kinematics) }.toTypedArray()
    )

    /**
     * Builds the trajectory,
     * returning the [CompositeCancelableTrajectory] object for use with other Hermes core functions.
     * The returned trajectory can be smoothly canceled at any time.
     */
    fun buildToComposite(): CompositeCancelableTrajectory = builder.buildToComposite()
}