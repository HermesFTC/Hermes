@file:Suppress("DuplicatedCode")

package me.zharel.hermes.talaria

import com.acmerobotics.roadrunner.geometry.*
import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.VelConstraint
import com.acmerobotics.roadrunner.trajectories.DisplacementTrajectory
import com.acmerobotics.roadrunner.trajectories.TimeTrajectory
import com.acmerobotics.roadrunner.trajectories.Trajectory
import edu.wpi.first.wpilibj.Timer
import kotlin.math.ceil
import kotlin.math.max
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.toKotlinDuration

data class FollowerParams(
    @JvmField
    val profileParams: ProfileParams,
    @JvmField
    val velConstraint: VelConstraint,
    @JvmField
    val accelConstraint: AccelConstraint,
)

fun interface EndCondition {
    fun shouldEnd(follower: Follower): Boolean

    companion object {
        @JvmSynthetic
        fun createEndCondition(condition: Follower.() -> Boolean) =
            EndCondition { follower -> follower.condition() }

        @JvmStatic
        fun overTime(dt: Duration) =
            createEndCondition { (timer.get().seconds - trajectory.duration()) > dt }

        @JvmStatic
        fun overTime(dt: java.time.Duration) = overTime(dt.toKotlinDuration())

        @JvmStatic
        fun overTime(dt: Double) = overTime(dt.seconds)

        @JvmStatic
        fun dispFromEnd(disp: Double) =
            createEndCondition { trajectory.endWrtDisp().value().position.norm() < disp}

        @JvmStatic
        fun robotVel(vel: Double) =
            createEndCondition { drive.localizer.getVel().hermes().linearVel.norm() < vel }

        @JvmStatic
        val default = setOf(
            overTime(2.0.seconds),
            dispFromEnd(2.0)
        )
    }
}

interface Follower {
    val trajectory: Trajectory<*>
    val drive: Drive
    val endConditions: Set<EndCondition>

    val currentTarget: Pose2d
    val lastCommand: PoseVelocity2dDual<Time>
    val timer: Timer

    val isDone: Boolean

    fun follow()

    val points: List<Vector2d> get() = listOf(Vector2d.zero)
}

class DisplacementFollower @JvmOverloads constructor(
    override val trajectory: DisplacementTrajectory,
    override val drive: Drive,
    override val endConditions: Set<EndCondition> = EndCondition.default
) : Follower {
    @JvmOverloads constructor(trajectory: Trajectory<*>, drive: Drive, endConditions: Set<EndCondition> = EndCondition.default) :
            this(trajectory.wrtDisp(), drive, endConditions)

    override var currentTarget: Pose2d = trajectory[0.0].value()
        private set
    override var lastCommand: PoseVelocity2dDual<Time> = PoseVelocity2dDual.zero();
    override val timer = Timer()
    private var started = false

    override var isDone: Boolean = false
        private set
    private var ds = 0.0

    override val points: List<Vector2d> = range(
                0.0,
                trajectory.length(),
                max(2, ceil(trajectory.length() / 2).toInt()),
            ).map {
               trajectory[it].value().position
            }

    val driveCommand: PoseVelocity2dDual<Time>
        get() {
            if (!started) {
                timer.reset()
                started = true
            }

            val robotVel = drive.localizer.update()
            val robotPose = drive.localizer.pose

            ds = trajectory.project(robotPose.hermes().position, ds)

            val error = trajectory[trajectory.length()].value() - robotPose.hermes()
            if (endConditions.any { it.shouldEnd(this) }) {
                isDone = true
                return PoseVelocity2dDual.zero()
            }

            val target: Pose2dDual<Time> = trajectory[ds]
            currentTarget = target.value()

            return drive.controller.compute(
                target,
                robotPose.hermes(),
                PoseVelocity2d.fromChassisSpeeds(robotVel.hermes()),
            )
        }

    override fun follow() {
        lastCommand = this.driveCommand
        drive.setDrivePowers(ChassisSpeeds.fromPoseVelocity(lastCommand.value()).wpilib())
    }
}

class TimeFollower @JvmOverloads constructor(
    override val trajectory: TimeTrajectory,
    override val drive: Drive,
    override val endConditions: Set<EndCondition> = EndCondition.default
) : Follower {
    @JvmOverloads constructor(trajectory: Trajectory<*>, drive: Drive, endConditions: Set<EndCondition> = EndCondition.default) :
            this(trajectory.wrtTime(), drive, endConditions)

    override var currentTarget: Pose2d = trajectory[0.0].value()
        private set
    override var lastCommand: PoseVelocity2dDual<Time> = PoseVelocity2dDual.zero();

    override var isDone: Boolean = false
        private set
    override val timer = Timer()
    private var started = false


    override val points: List<Vector2d> = range(
        0.0,
        trajectory.length(),
        max(2, ceil(trajectory.length() / 2).toInt()),
    ).map {
        trajectory[it].value().position
    }

    val driveCommand: PoseVelocity2dDual<Time>
        get() {
            if (!started) {
                timer.reset()
                started = true
            }

            val dt = timer.get()

            val robotVel = drive.localizer.update()
            val robotPose = drive.localizer.pose

            val error = trajectory.endWrtTime().value() - robotPose.hermes()
            if (endConditions.any { it.shouldEnd(this) }) {
                isDone = true
                return PoseVelocity2dDual.zero()
            }

            val target: Pose2dDual<Time> = trajectory[dt]

            currentTarget = target.value()

            return drive.controller.compute(
                target,
                robotPose.hermes(),
                PoseVelocity2d.fromChassisSpeeds(robotVel.hermes()),
            )
        }

    override fun follow() {
        lastCommand = this.driveCommand
        drive.setDrivePowers(ChassisSpeeds.fromPoseVelocity(lastCommand.value()).wpilib())
    }
}
