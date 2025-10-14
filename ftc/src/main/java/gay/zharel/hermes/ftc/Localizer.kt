@file:Suppress("unused")
package gay.zharel.hermes.ftc

import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.PoseVelocity2d

/**
 * A localizer is responsible for tracking the robot's position and velocity (i.e., its pose) in the field frame.
 */
interface Localizer {
    /**
     * The current robot pose.
     */
    var pose: Pose2d

    /**
     * The current robot velocity.
     */
    val vel: PoseVelocity2d

    /**
     * A list of recent poses.
     */
    val poseHistory: MutableList<Pose2d>

    /**
     * Updates the localizer and returns the new velocity.
     */
    fun update(): PoseVelocity2d
}
