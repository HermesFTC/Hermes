package gay.zharel.hermes.trajectories

import gay.zharel.hermes.geometry.DualParameter
import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.geometry.Vector2d
import kotlinx.serialization.Serializable
import kotlinx.serialization.SerialName
import kotlinx.serialization.Transient


/**
 * Functional interface representing a marker trigger.
 * A marker trigger determines whether a specific condition is met
 * during the execution of a trajectory, based on the robot's state,
 * the trajectory, and the current progress along the trajectory.
 */
fun interface MarkerTrigger {
    /**
     * Determines if the marker should trigger.
     *
     * @param robotState The current state of the robot.
     * @param trajectory The trajectory being followed, parameterized by arclength.
     * @param s The current progress along the trajectory, typically represented as arclength.
     */
    fun shouldTrigger(robotState: RobotState, trajectory: Trajectory<*>, s: Double): Boolean
}


@Serializable
@SerialName("AfterDisp")
class AfterDispTrigger(val disp: Double) : MarkerTrigger {
    override fun shouldTrigger(robotState: RobotState, trajectory: Trajectory<*>, s: Double): Boolean = s == disp
}

@Serializable
@SerialName("AfterTime")
class AfterTimeTrigger(val time: Double) : MarkerTrigger {
    override fun shouldTrigger(robotState: RobotState, trajectory: Trajectory<*>, s: Double): Boolean = s > trajectory.wrtTime().profile[time].value()
}

@Serializable
@SerialName("AtPoint")
class AtPointTrigger @JvmOverloads constructor(val point: Vector2d, val tolerance: Double = 2.0) : MarkerTrigger {
    override fun shouldTrigger(robotState: RobotState, trajectory: Trajectory<*>, s: Double): Boolean = (robotState.pose.position - point).norm() < tolerance
}

@Serializable
@SerialName("AtPose")
class AtPoseTrigger @JvmOverloads constructor(val pose: Pose2d, val linearTolerance: Double = 2.0, val angularTolerance: Double = Math.toRadians(5.0)) : MarkerTrigger {
    override fun shouldTrigger(robotState: RobotState, trajectory: Trajectory<*>, s: Double): Boolean = (robotState.pose - pose).let { it.line.norm() < linearTolerance && it.angle < angularTolerance }
}

/**
 * Functional interface for marker callbacks triggered during trajectory execution.
 * Implementations define actions to perform when a marker is triggered.
 */
fun interface MarkerCallback {
    fun onTrigger()
}

/**
 * Represents a trajectory marker, which consists of a trigger condition and a callback to execute when triggered.
 * @property trigger The condition under which the marker should be triggered.
 * @property callback The action to execute when the marker is triggered.
 */
@Serializable
class Marker(
    @JvmField
    val trigger: MarkerTrigger,
    @JvmField
    @Transient
    val callback: MarkerCallback = MarkerCallback {},
) {
    companion object {
        /**
         * Creates a marker that triggers after a specified displacement.
         * @param disp The displacement value at which to trigger the marker.
         * @param callback The callback to execute when triggered.
         * @return A [Marker] that triggers after the given displacement.
         */
        @JvmStatic
        fun afterDisp(disp: Double, callback: MarkerCallback) = Marker(AfterDispTrigger(disp), callback)

        /**
         * Creates a marker that triggers after a specified time.
         * @param time The time value at which to trigger the marker.
         * @param callback The callback to execute when triggered.
         * @return A [Marker] that triggers after the given time.
         */
        @JvmStatic
        fun afterTime(time: Double, callback: MarkerCallback) = Marker(AfterTimeTrigger(time), callback)

        /**
         * Creates a marker that triggers when the robot is within a certain tolerance of a given point.
         * @param point The [Vector2d] point to check proximity against.
         * @param tolerance The distance tolerance for triggering the marker (default: 2.0 units).
         * @param callback The callback to execute when triggered.
         * @return A [Marker] that triggers when the robot is near the specified point.
         */
        @JvmStatic
        @JvmOverloads
        fun atPoint(point: Vector2d, tolerance: Double = 2.0, callback: MarkerCallback) = Marker(AtPointTrigger(point, tolerance), callback)

        /**
         * Creates a marker that triggers when the robot is within a certain linear and angular tolerance of a given pose.
         * @param pose The [Pose2d] to check proximity and orientation against.
         * @param linearTolerance The distance tolerance for triggering the marker (default: 2.0 units).
         * @param angularTolerance The angular tolerance in radians for triggering the marker (default: 5 degrees).
         * @param callback The callback to execute when triggered.
         * @return A [Marker] that triggers when the robot is near the specified pose.
         */
        @JvmStatic
        @JvmOverloads
        fun atPose(pose: Pose2d, linearTolerance: Double = 2.0, angularTolerance: Double = Math.toRadians(5.0), callback: MarkerCallback) = Marker(AtPoseTrigger(pose, linearTolerance, angularTolerance), callback)
    }

    /**
     * Returns a copy of this marker with a new callback.
     */
    fun withCallback(callback: MarkerCallback) = Marker(trigger, callback)
}

/**
 * Trajectory with markers.
 */
@Serializable
@SerialName("TrajectoryWithMarkers")
data class TrajectoryWithMarkers<Param : DualParameter>(
    val trajectory: Trajectory<Param>,
    val markers: List<Marker>
) : Trajectory<Param> by trajectory
