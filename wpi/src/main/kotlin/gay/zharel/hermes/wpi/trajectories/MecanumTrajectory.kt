package gay.zharel.hermes.wpi.trajectories

import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.trajectories.TimeTrajectory
import gay.zharel.hermes.wpi.conversions.into
import gay.zharel.hermes.wpi.conversions.seconds
import gay.zharel.hermes.wpi.conversions.wpilib
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.kinematics.ChassisAccelerations
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.kinematics.MecanumDriveKinematics
import org.wpilib.math.kinematics.MecanumDriveWheelVelocities
import org.wpilib.units.Units.Seconds
import org.wpilib.units.measure.Time

data class MecanumTrajectorySample(
  val timestamp: Time,
  val pose: Pose2d,
  val velocity: ChassisVelocities,
  val acceleration: ChassisAccelerations,
  val wheelSpeeds: MecanumDriveWheelVelocities,
)

internal fun MecanumDriveKinematics.convertState(timestamp: Time, state: RobotState) = state.wpilib.let {
  MecanumTrajectorySample(
    timestamp,
    it.first,
    it.second,
    it.third,
    this.toWheelVelocities(it.second.toRobotRelative(it.first.rotation)),
  )
}

class MecanumTrajectory internal constructor(
  val kinematics: MecanumDriveKinematics,
  val trajectory: TimeTrajectory,
) {
  val duration = trajectory.duration.seconds

  val start = this[0.0.seconds]
  val end = this[duration]

  fun samples() = trajectory.profile.times.map { sampleAt(it.seconds) }

  fun sampleAt(timestamp: Time): MecanumTrajectorySample =
    kinematics.convertState(timestamp, trajectory[timestamp.into(Seconds)])

  operator fun get(timestamp: Time) = sampleAt(timestamp)
}
