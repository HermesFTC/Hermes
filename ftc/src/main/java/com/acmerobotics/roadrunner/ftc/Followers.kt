package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.geometry.*
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual.Companion.zero
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.profiles.*
import com.acmerobotics.roadrunner.trajectories.DisplacementTrajectory
import com.acmerobotics.roadrunner.trajectories.TimeTrajectory
import com.acmerobotics.roadrunner.trajectories.Trajectory
import com.acmerobotics.roadrunner.trajectories.endWrtTime
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.ceil
import kotlin.math.max

data class FollowerParams(
    @JvmField
    val profileParams: ProfileParams,
    @JvmField
    val velConstraint: VelConstraint,
    @JvmField
    val accelConstraint: AccelConstraint,
)

interface Follower {
    val currentTarget: Pose2d
    val lastCommand: PoseVelocity2dDual<Time>

    val isDone: Boolean

    fun follow()

    val points: List<Vector2d> get() = listOf(Vector2d.zero)
}

class DisplacementFollower(trajectory: Trajectory<*>, private val drive: Drive) : Follower {
    private val trajectory: DisplacementTrajectory = trajectory.wrtDisp()

    @JvmOverloads
    constructor(
        path: PosePath,
        drive: Drive,
        velConstraintOverride: VelConstraint = drive.defaultVelConstraint,
        accelConstraintOverride: AccelConstraint = drive.defaultAccelConstraint,
    ) : this(
        DisplacementTrajectory(
            path,
            forwardProfile(
                drive.followerParams.profileParams,
                path,
                0.0,
                velConstraintOverride,
                accelConstraintOverride,
            ),
        ),
        drive,
    )

    constructor(path: PosePath, drive: Drive, velConstraint: VelConstraint) : this(
        path,
        drive,
        velConstraint,
        drive.defaultAccelConstraint,
    )

    constructor(path: PosePath, drive: Drive, accelConstraint: AccelConstraint) : this(
        path,
        drive,
        drive.defaultVelConstraint,
        accelConstraint,
    )

    override var currentTarget: Pose2d = trajectory[0.0].value()
        private set
    override var lastCommand: PoseVelocity2dDual<Time> = PoseVelocity2dDual.zero();

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
            val robotVel = drive.localizer.update()
            val robotPose = drive.localizer.pose

            ds = trajectory.project(robotPose.position, ds)

            val error = trajectory[trajectory.length()].value() - robotPose
            if (ds >= trajectory.length() || (error.line.norm() < 1.0 && error.angle < Math.toDegrees(5.0))) {
                isDone = true
                return PoseVelocity2dDual.zero()
            }

            val target: Pose2dDual<Time> = trajectory[ds]
            currentTarget = target.value()

            return drive.controller.compute(
                target,
                robotPose,
                robotVel,
            )
        }

    override fun follow() {
        lastCommand = this.driveCommand
        drive.setDrivePowersWithFF(lastCommand)
    }
}

class TimeFollower(trajectory: Trajectory<*>, private val drive: Drive) : Follower {
    private val trajectory: TimeTrajectory = trajectory.wrtTime()

    @JvmOverloads
    constructor(
        path: PosePath,
        drive: Drive,
        velConstraintOverride: VelConstraint = drive.defaultVelConstraint,
        accelConstraintOverride: AccelConstraint = drive.defaultAccelConstraint,
    ) : this(
        TimeTrajectory(
            path,
            TimeProfile(
                forwardProfile(
                    drive.followerParams.profileParams,
                    path,
                    0.0,
                    velConstraintOverride,
                    accelConstraintOverride,
                ),
            ),
        ),
        drive,
    )

    constructor(path: PosePath, drive: Drive, velConstraint: VelConstraint) : this(
        path,
        drive,
        velConstraint,
        drive.defaultAccelConstraint,
    )

    constructor(path: PosePath, drive: Drive, accelConstraint: AccelConstraint) : this(
        path,
        drive,
        drive.defaultVelConstraint,
        accelConstraint,
    )

    override var currentTarget: Pose2d = trajectory[0.0].value()
        private set
    override var lastCommand: PoseVelocity2dDual<Time> = PoseVelocity2dDual.zero();

    override var isDone: Boolean = false
        private set
    private val timer: ElapsedTime = ElapsedTime()
    private val duration: Double = this.trajectory.duration
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

            val dt = timer.seconds()

            val robotVel = drive.localizer.update()
            val robotPose = drive.localizer.pose

            val error = trajectory.endWrtTime.value() - robotPose
            if (dt >= duration || (error.line.norm() < 1.0 && error.angle < Math.toDegrees(5.0))) {
                isDone = true
                return PoseVelocity2dDual.zero()
            }

            val target: Pose2dDual<Time> = trajectory[dt]

            currentTarget = target.value()

            return drive.controller.compute(
                target,
                robotPose,
                robotVel,
            )
        }

    override fun follow() {
        lastCommand = this.driveCommand
        drive.setDrivePowersWithFF(lastCommand)
    }
}
