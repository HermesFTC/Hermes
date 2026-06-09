package gay.zharel.hermes.wpi.trajectories

import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.trajectories.TimeTrajectory
import gay.zharel.hermes.wpi.conversions.into
import gay.zharel.hermes.wpi.conversions.seconds
import gay.zharel.hermes.wpi.conversions.wpilib
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.kinematics.ChassisAccelerations
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.kinematics.SwerveDriveKinematics
import org.wpilib.math.kinematics.SwerveModuleVelocity
import org.wpilib.units.Units.Seconds
import org.wpilib.units.measure.Time

data class SwerveTrajectorySample(
  val timestamp: Time,
  val pose: Pose2d,
  val velocity: ChassisVelocities,
  val acceleration: ChassisAccelerations,
  val moduleStates: Array<SwerveModuleVelocity>,
) {
  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is SwerveTrajectorySample) return false

    if (timestamp != other.timestamp) return false
    if (pose != other.pose) return false
    if (velocity != other.velocity) return false
    if (acceleration != other.acceleration) return false
    if (!moduleStates.contentEquals(other.moduleStates)) return false

    return true
  }

  override fun hashCode(): Int {
    var result = timestamp.hashCode()
    result = 31 * result + pose.hashCode()
    result = 31 * result + velocity.hashCode()
    result = 31 * result + acceleration.hashCode()
    result = 31 * result + moduleStates.contentHashCode()
    return result
  }
}

internal fun SwerveDriveKinematics.convertState(timestamp: Time, state: RobotState) = state.wpilib.let {
  SwerveTrajectorySample(
    timestamp,
    it.first,
    it.second,
    it.third,
    this.toSwerveModuleVelocities(it.second.toRobotRelative(it.first.rotation)),
  )
}

class SwerveTrajectory internal constructor(
  val kinematics: SwerveDriveKinematics,
  val trajectory: TimeTrajectory,
) {
  val duration = trajectory.duration.seconds

  val start = this[0.0.seconds]
  val end = this[duration]

  fun samples() = trajectory.profile.times.map { sampleAt(it.seconds) }

  fun sampleAt(timestamp: Time): SwerveTrajectorySample =
    kinematics.convertState(timestamp, trajectory[timestamp.into(Seconds)])

  operator fun get(timestamp: Time) = sampleAt(timestamp)
}
