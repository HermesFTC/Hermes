package me.zharel.hermes.talaria

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time

internal typealias iTrajectory<Param> = com.acmerobotics.roadrunner.trajectories.Trajectory<Param>

interface Trajectory {
    val length: Distance
    val duration: Time

    val path: Path

    operator fun get(s: Distance): Sample
    operator fun get(t: Time): Sample

     data class Sample(
        val time: Time,
        val displacement: Distance,
        val pose: Pose2d,
        val vel: ChassisSpeeds,
        val acc: ChassisAcceleration
    ) {
         constructor(
             time: Time,
             displacement: Distance,
             state: RobotMotionState
         ) : this(
             time,
             displacement,
             state.pos,
             state.vel,
             state.acc
         )
     }
}

class TrajectoryWrapper internal constructor(internal: iTrajectory<*>) : Trajectory {
    internal val dispTraj = internal.wrtDisp()
    internal val timeTraj = internal.wrtTime()

    override val length = dispTraj.length().inches()
    override val duration = timeTraj.duration().time()

    override val path = PathWrapper(dispTraj.path)

    override fun get(s: Distance): Trajectory.Sample {
        val dualPose = dispTraj[s.inchValue()]
        val time = timeTraj.profile.inverse(s.inchValue())

        return Trajectory.Sample(
            time.seconds(),
            s,
            dualPose.value().wpilib(),
            dualPose.velocity().value().wpilib(),
            dualPose.velocity().acceleration().value().talaria()
        )
    }

    override fun get(t: Time): Trajectory.Sample {
        val dualPose = timeTraj[t.secondValue()]
        val disp = timeTraj.profile[t.secondValue()]

        return Trajectory.Sample(
            t,
            disp.value().inches(),
            dualPose.value().wpilib(),
            dualPose.velocity().value().wpilib(),
            dualPose.velocity().acceleration().value().talaria()
        )
    }
}