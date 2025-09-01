@file:Suppress("DuplicatedCode")

package gay.zharel.hermes.ftc

import gay.zharel.hermes.geometry.*
import gay.zharel.hermes.paths.PosePath
import gay.zharel.hermes.profiles.*
import gay.zharel.hermes.trajectories.CancelableTrajectory
import gay.zharel.hermes.trajectories.DisplacementTrajectory
import gay.zharel.hermes.trajectories.Marker
import gay.zharel.hermes.trajectories.TimeTrajectory
import gay.zharel.hermes.trajectories.Trajectory
import gay.zharel.hermes.trajectories.TrajectoryWithMarkers
import com.qualcomm.robotcore.util.ElapsedTime
import gay.zharel.hermes.math.Arclength
import gay.zharel.hermes.math.Time
import gay.zharel.hermes.math.range
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
            createEndCondition { (timer.seconds().seconds - trajectory.duration()) > dt }

        @JvmStatic
        fun overTime(dt: java.time.Duration) = overTime(dt.toKotlinDuration())

        @JvmStatic
        fun overTime(dt: Double) = overTime(dt.seconds)

        @JvmStatic
        fun dispFromEnd(disp: Double) =
            createEndCondition { trajectory.endWrtDisp().value().position.norm() < disp}

        @JvmStatic
        fun robotVel(vel: Double) =
            createEndCondition { drive.localizer.vel.linearVel.norm() < vel }

        @JvmStatic
        val default = setOf(
            overTime(2.0.seconds),
            dispFromEnd(2.0)
        )
    }
}

interface Follower {
    val trajectory: TrajectoryWithMarkers<*>
    val drive: Drive
    val endConditions: Set<EndCondition>

    val currentTarget: Pose2d
    val lastCommand: PoseVelocity2dDual<Time>
    val timer: ElapsedTime

    val isDone: Boolean

    fun follow()

    val points: List<Vector2d> get() = listOf(Vector2d.zero)

    /**
     * Tracks which markers have been triggered.
     */
    val triggeredMarkers: MutableSet<Marker>

    /**
     * Cancels the follower. If [trajectory] is cancelable,
     * this function will cancel smoothly,
     * otherwise, it will simply stop the robot.
     *
     * **Note: This function blocks until the robot is stopped.**
     */
    fun cancel()
}

class DisplacementFollower @JvmOverloads constructor(
    override val trajectory: TrajectoryWithMarkers<Arclength>,
    override val drive: Drive,
    override val endConditions: Set<EndCondition> = EndCondition.default
) : Follower {
    @JvmOverloads constructor(trajectory: Trajectory<*>, drive: Drive, endConditions: Set<EndCondition> = EndCondition.default) :
            this(TrajectoryWithMarkers(trajectory.wrtDisp(), emptyList()), drive, endConditions)

    @JvmOverloads
    constructor(
        path: PosePath,
        drive: Drive,
        velConstraintOverride: VelConstraint = drive.defaultVelConstraint,
        accelConstraintOverride: AccelConstraint = drive.defaultAccelConstraint,
        endConditions: Set<EndCondition> = EndCondition.default,
    ) : this(
        TrajectoryWithMarkers(
            DisplacementTrajectory(
                path,
                createOptimalDisplacementProfile(
                    drive.followerParams.profileParams,
                    path,
                    0.0,
                    velConstraintOverride,
                    accelConstraintOverride,
                ),
            ),
            listOf()
        ),
        drive,
        endConditions
    )

    override var currentTarget: Pose2d = trajectory[0.0].value()
        private set
    override var lastCommand: PoseVelocity2dDual<Time> = PoseVelocity2dDual.zero();
    override val timer: ElapsedTime = ElapsedTime()
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

    override val triggeredMarkers: MutableSet<Marker> = mutableSetOf()

    private val disptraj = trajectory.wrtDisp()

    val driveCommand: PoseVelocity2dDual<Time>
        get() {
            if (!started) {
                timer.reset()
                started = true
            }

            val robotVel = drive.localizer.update()
            val robotPose = drive.localizer.pose

            ds = trajectory.project(robotPose.position, ds)

            if (endConditions.any { it.shouldEnd(this) }) {
                isDone = true
                return PoseVelocity2dDual.zero()
            }

            val target: Pose2dDual<Time> = disptraj[ds]
            currentTarget = target.value()

            // Marker handling
            trajectory.markers.forEach { marker ->
                if (marker !in triggeredMarkers && marker.trigger.shouldTrigger(
                        RobotState(robotPose, robotVel, Acceleration2d.zero), disptraj, ds)) {
                    marker.callback.onTrigger()
                    triggeredMarkers.add(marker)
                }
            }

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

    override fun cancel() {
        if (trajectory.trajectory is CancelableTrajectory) {
            val traj = (trajectory.trajectory as CancelableTrajectory).cancel(ds)
            val follower = DisplacementFollower(traj, drive, endConditions)
            while (!follower.isDone) {
                follower.follow()
            }
            return
        } else {
            drive.setDrivePowers(PoseVelocity2d.zero)
        }
    }
}

class TimeFollower @JvmOverloads constructor(
    override val trajectory: TrajectoryWithMarkers<Time>,
    override val drive: Drive,
    override val endConditions: Set<EndCondition> = EndCondition.default
) : Follower {
    @JvmOverloads constructor(trajectory: Trajectory<*>, drive: Drive, endConditions: Set<EndCondition> = EndCondition.default) :
            this(TrajectoryWithMarkers(trajectory.wrtTime(), emptyList()), drive, endConditions)

    @JvmOverloads
    constructor(
        path: PosePath,
        drive: Drive,
        velConstraintOverride: VelConstraint = drive.defaultVelConstraint,
        accelConstraintOverride: AccelConstraint = drive.defaultAccelConstraint,
        endConditions: Set<EndCondition>,
    ) : this(
        TrajectoryWithMarkers(
            TimeTrajectory(
                path,
                TimeProfile(
                    DisplacementProfile.generate(
                        drive.followerParams.profileParams,
                        path,
                        0.0,
                        velConstraintOverride,
                        accelConstraintOverride,
                    ),
                ),
            ),
            listOf()
        ),
        drive,
        endConditions
    )

    private val timetraj = trajectory.wrtTime()

    override var currentTarget: Pose2d = trajectory[0.0].value()
        private set
    override var lastCommand: PoseVelocity2dDual<Time> = PoseVelocity2dDual.zero();

    override var isDone: Boolean = false
        private set
    override val timer: ElapsedTime = ElapsedTime()
    private var started = false

    override val points: List<Vector2d> = range(
        0.0,
        trajectory.length(),
        max(2, ceil(trajectory.length() / 2).toInt()),
    ).map {
        trajectory[it].value().position
    }

    override val triggeredMarkers: MutableSet<Marker> = mutableSetOf()

    val driveCommand: PoseVelocity2dDual<Time>
        get() {
            if (!started) {
                timer.reset()
                started = true
            }

            val dt = timer.seconds()

            val robotVel = drive.localizer.update()
            val robotPose = drive.localizer.pose

            if (endConditions.any { it.shouldEnd(this) }) {
                isDone = true
                return PoseVelocity2dDual.zero()
            }

            val target: Pose2dDual<Time> = trajectory[dt]

            currentTarget = target.value()

            // Marker handling
            val s = timetraj.profile[dt].value()

            trajectory.markers.forEach { marker ->
                if (marker !in triggeredMarkers && marker.trigger.shouldTrigger(
                        RobotState(robotPose, robotVel, Acceleration2d.zero), timetraj, s)) {
                    marker.callback.onTrigger()
                    triggeredMarkers.add(marker)
                }
            }

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

    override fun cancel() {
        if (trajectory.trajectory is CancelableTrajectory) {
            val traj = (trajectory.trajectory as CancelableTrajectory).cancel(timetraj.profile[timer.seconds()].value())
            val follower = DisplacementFollower(traj, drive, endConditions)
            while (!follower.isDone) {
                follower.follow()
            }
            return
        } else {
            drive.setDrivePowers(PoseVelocity2d.zero)
        }
    }
}
