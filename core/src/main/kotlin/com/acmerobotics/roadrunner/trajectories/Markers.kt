package com.acmerobotics.roadrunner.trajectories

import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.geometry.DualParameter
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.RobotState
import com.acmerobotics.roadrunner.geometry.Vector2d


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
data class Marker(
    @JvmField
    val trigger: MarkerTrigger,
    @JvmField
    val callback: MarkerCallback,
) {
    companion object {
        /**
         * Creates a marker that triggers after a specified displacement.
         * @param disp The displacement value at which to trigger the marker.
         * @param callback The callback to execute when triggered.
         * @return A [Marker] that triggers after the given displacement.
         */
        @JvmStatic
        fun afterDisp(disp: Double, callback: MarkerCallback) = Marker(
            {_, _, s -> s == disp},
            callback,
        )

        /**
         * Creates a marker that triggers after a specified time.
         * @param time The time value at which to trigger the marker.
         * @param callback The callback to execute when triggered.
         * @return A [Marker] that triggers after the given time.
         */
        @JvmStatic
        fun afterTime(time: Double, callback: MarkerCallback) = Marker(
            { _, traj, s ->
                s > traj.wrtTime().profile[time].value()
            },
            callback
        )

        /**
         * Creates a marker that triggers when the robot is within a certain tolerance of a given point.
         * @param point The [Vector2d] point to check proximity against.
         * @param tolerance The distance tolerance for triggering the marker (default: 2.0 units).
         * @param callback The callback to execute when triggered.
         * @return A [Marker] that triggers when the robot is near the specified point.
         */
        @JvmStatic
        @JvmOverloads
        fun atPoint(point: Vector2d, tolerance: Double = 2.0, callback: MarkerCallback) = Marker(
            { robotState, _, _ -> (robotState.pose.position - point).norm() < tolerance},
            callback,
        )

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
        fun atPose(pose: Pose2d, linearTolerance: Double = 2.0, angularTolerance: Double = Math.toRadians(5.0), callback: MarkerCallback) = Marker(
            { robotState, _, _ ->
                (robotState.pose - pose).let { it.line.norm() < linearTolerance && it.angle < angularTolerance }
            },
            callback,
        )
    }
}

/**
 * Trajectory with markers.
 */
data class TrajectoryWithMarkers<Param : DualParameter>(
    val trajectory: Trajectory<Param>,
    val markers: List<Marker>
) : Trajectory<Param> by trajectory
