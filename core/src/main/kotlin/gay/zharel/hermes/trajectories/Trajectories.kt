@file:JvmName("Trajectories")
package gay.zharel.hermes.trajectories

import gay.zharel.hermes.math.Arclength
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.geometry.Pose2dDual
import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.math.Time
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.paths.CompositePosePath
import gay.zharel.hermes.paths.MappedPosePath
import gay.zharel.hermes.paths.PoseMap
import gay.zharel.hermes.paths.PosePath
import gay.zharel.hermes.profiles.CancelableProfile
import gay.zharel.hermes.profiles.DisplacementProfile
import gay.zharel.hermes.profiles.TimeProfile
import gay.zharel.hermes.profiles.plus
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.Transient
import kotlin.time.Duration.Companion.seconds
import kotlin.time.DurationUnit

/**
 * @usesMathJax
 *
 * Represents a robot trajectory that combines a geometric path with a motion profile.
 *
 * A trajectory defines both the spatial path the robot follows and the timing/velocity
 * profile for traversing that path. It can be parameterized by displacement ([Arclength])
 * or time ([Time]), and provides conversions between these representations.
 *
 * Trajectories are the primary output of motion planning and can be queried to obtain
 * the robot's desired state (position, heading, velocity, acceleration) at any point.
 *
 * @param Param The parameter type used to parameterize the trajectory (e.g., [Arclength], [Time])
 */
interface Trajectory<Param : DualParameter> {
    /**
     * Returns the total arc length of the trajectory.
     */
    fun length(): Double

    /**
     * Returns the total time duration required to execute the trajectory.
     */
    fun duration() = wrtTime().duration

    /**
     * Evaluates the trajectory at the given parameter value to obtain the robot state.
     *
     * @param param The parameter value (displacement or time) at which to evaluate
     * @return The [RobotState] containing position, heading, velocities, and accelerations
     */
    operator fun get(param: Double): RobotState

    /**
     * Returns the robot state at the start of the trajectory (parameter = 0).
     */
    fun start() = get(0.0)

    /**
     * Returns the robot state at the end of the trajectory with respect to displacement.
     */
    fun endWrtDisp() = wrtDisp()[length()]

    /**
     * Returns the robot state at the end of the trajectory with respect to time.
     */
    fun endWrtTime() = wrtTime()[duration().toDouble(DurationUnit.SECONDS)]

    /**
     * Projects a query position onto the trajectory's path using Newton's method.
     *
     * Finds the parameter value where the trajectory's path is closest to the query point.
     *
     * @param query The 2D position to project onto the trajectory
     * @param init Initial guess for the parameter value (default: 0.0)
     * @return The parameter value at the closest point on the trajectory to the query
     */
    fun project(query: Vector2d, init: Double): Double

    /**
     * Converts this trajectory to a displacement-parameterized representation.
     *
     * In displacement parameterization, the trajectory is indexed by arc length along the path.
     */
    fun wrtDisp(): DisplacementTrajectory

    /**
     * Converts this trajectory to a time-parameterized representation.
     *
     * In time parameterization, the trajectory is indexed by elapsed time.
     */
    fun wrtTime(): TimeTrajectory

    /**
     * Combines this trajectory with another trajectory to create a composite trajectory.
     *
     * The resulting trajectory will execute this trajectory first, then the other trajectory.
     *
     * @param other The trajectory to append after this one
     */
    operator fun plus(other: Trajectory<Param>) = CompositeTrajectory(this, other)

    /**
     * Applies a pose transformation map to this trajectory.
     *
     * Transforms the trajectory's path using the given [PoseMap] while preserving the motion profile.
     *
     * @return A new [DisplacementTrajectory] with the transformation applied
     */
    fun map(map: PoseMap) = wrtDisp().let {
        DisplacementTrajectory(
            MappedPosePath(it.path, map),
            it.profile
        )
    }
}

@Serializable
@SerialName("CancelableTrajectory")
class CancelableTrajectory(
    val thisPath: PosePath,
    @JvmField
    val cProfile: CancelableProfile,
    @JvmField
    val offsets: List<Double>
) : DisplacementTrajectory(thisPath, cProfile.baseProfile) {
    fun cancel(s: Double): DisplacementTrajectory {
        val offset = s
        return DisplacementTrajectory(
            object : PosePath {
                override fun length() = path.length() - offset
                override fun get(s: Double, n: Int) = path[s + offset, n]
            },
            cProfile.cancel(s)
        )
    }
}

@Serializable
@SerialName("DisplacementTrajectory")
open class DisplacementTrajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val profile: DisplacementProfile
) : Trajectory<Arclength> {
    constructor(t: CancelableTrajectory) : this(t.path, t.cProfile.baseProfile)

    override fun wrtDisp() = this
    override fun wrtTime() = TimeTrajectory(this)

    override fun length() = path.length()

    override fun project(query: Vector2d, init: Double) = path.project(query, init)

    override operator fun get(s: Double): RobotState =
        RobotState.fromDualPose(path[s, 3].reparam(profile[s]))
}

@Serializable
@SerialName("TimeTrajectory")
class TimeTrajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val profile: TimeProfile
) : Trajectory<Time> {
    val duration = profile.duration.seconds

    constructor(t: CancelableTrajectory) : this(t.path, TimeProfile(t.cProfile.baseProfile))

    constructor(t: DisplacementTrajectory) : this(t.path, TimeProfile(t.profile))

    override fun wrtDisp() = DisplacementTrajectory(this.path, this.profile.dispProfile)
    override fun wrtTime() = this

    override fun length(): Double = path.length()

    override operator fun get(t: Double): RobotState {
        val s = profile[t]
        return RobotState.fromDualPose(path[s.value(), 3].reparam(s))
    }

    override fun project(query: Vector2d, init: Double): Double = path.project(query, init)
}

@Serializable
@SerialName("CompositeTrajectory")
class CompositeTrajectory @JvmOverloads constructor(
    @JvmField
    val trajectories: List<DisplacementTrajectory>,
    @JvmField
    val offsets: List<Double> = trajectories.scan(0.0) { acc, t -> acc + t.length() }
) : Trajectory<Arclength> {
    constructor(trajectories: Collection<Trajectory<*>>) : this(trajectories.map { it.wrtDisp() })
    constructor(vararg trajectories: DisplacementTrajectory) : this(trajectories.toList())
    constructor(vararg trajectories: Trajectory<*>) : this(trajectories.map { it.wrtDisp() })

    @Transient val path = CompositePosePath(trajectories.map { it.path }, offsets)
    @Transient val profile = trajectories.map { it.profile }.reduce { acc, profile -> acc + profile }

    @JvmField
    val length = offsets.last()

    init {
        require(trajectories.size + 1 == offsets.size) {
            "trajectories.size (${trajectories.size}) + 1 != offsets.size (${offsets.size})"
        }
    }

    override fun get(s: Double): RobotState {
        if (s > length) {
            return RobotState.fromDualPose(Pose2dDual.Companion.constant(trajectories.last().path.end(1).value(), 3))
        }

        for ((offset, traj) in offsets.zip(trajectories).reversed()) {
            if (s >= offset) {
                return traj[s - offset]
            }
        }

        return trajectories.first()[0.0]
    }

    override fun length() = length

    override fun wrtDisp() = DisplacementTrajectory(path, profile)
    override fun wrtTime() = TimeTrajectory(path, TimeProfile(profile))

    override fun project(query: Vector2d, init: Double) = path.project(query, init)
}

/**
 * Represents a composite trajectory made up of multiple [CancelableTrajectory] segments.
 * Allows cancellation at any displacement along the combined trajectory, returning a new composite trajectory.
 */
@Serializable
@SerialName("CompositeCancelableTrajectory")
class CompositeCancelableTrajectory @JvmOverloads constructor(
    @JvmField
    val trajectories: List<CancelableTrajectory>,
    @JvmField
    val offsets: List<Double> = trajectories.scan(0.0) { acc, t -> acc + t.length() }
) : Trajectory<Arclength> by CompositeTrajectory(trajectories, offsets) {

    /**
     * Cancels the composite trajectory at the given displacement,
     * returning a new trajectory starting from that point.
     * @param s The displacement at which to cancel.
     * @return A new [DisplacementTrajectory] representing the remaining trajectory.
     */
    fun cancel(s: Double): DisplacementTrajectory {
        if (s > length()) {
            return trajectories.last().cancel(s - length())
        }

        for ((offset, traj) in offsets.zip(trajectories).reversed()) {
            if (s >= offset) {
                return traj.cancel(s - offset)
            }
        }

        return trajectories.first().cancel(0.0)
    }
}

fun compose(vararg trajectories: Trajectory<*>) = CompositeTrajectory(*trajectories)
fun List<Trajectory<*>>.compose() = CompositeTrajectory(this)
