package me.zharel.hermes.wpi

import com.acmerobotics.roadrunner.geometry.ChassisSpeeds.Companion.fromPoseVelocity
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import kotlin.time.DurationUnit

typealias iTrajectory<Param> = com.acmerobotics.roadrunner.trajectories.Trajectory<Param>

interface Trajectory {
    val length: Distance
    val duration: Time

    operator fun get(s: Distance): Pair<Pose2d, ChassisSpeeds>
    operator fun get(t: Time): Pair<Pose2d, ChassisSpeeds>
}

class TrajectoryWrapper internal constructor(internal: iTrajectory<*>) : Trajectory {
    internal val dispTraj = internal.wrtDisp()
    internal val timeTraj = internal.wrtTime()

    override val length = dispTraj.length().inches()
    override val duration = timeTraj.duration().time()

    override fun get(s: Distance): Pair<Pose2d, ChassisSpeeds> = dispTraj[s.inchValue()].let {
        it.value().wpilib() to fromPoseVelocity(it.velocity().value()).wpilib()
    }

    override fun get(t: Time): Pair<Pose2d, ChassisSpeeds> =
        timeTraj[t.duration().toDouble(DurationUnit.SECONDS)].let {
            it.value().wpilib() to fromPoseVelocity(it.velocity().value()).wpilib()
        }
}