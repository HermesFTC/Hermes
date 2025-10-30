package gay.zharel.hermes.wpitrajectories

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint
import edu.wpi.first.math.trajectory.constraint.MecanumDriveKinematicsConstraint
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint
import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.math.MinMax
import gay.zharel.hermes.paths.PosePath
import gay.zharel.hermes.profiles.AccelConstraint
import gay.zharel.hermes.profiles.VelConstraint
import gay.zharel.hermes.wpiconversions.wpilib
import kotlin.math.absoluteValue
import kotlin.math.pow

infix fun Vector2d.cross(other: Vector2d) = this.x * other.y - this.y * other.x

fun PosePath.curvatureAt(s: Double): Double {
    val state = RobotState.fromDualPose(this[s, 3])
    val numerator = state.vel.linearVel cross state.accel.linearAcc
    val denominator = state.vel.linearVel.norm().pow(3)

    return if (denominator.absoluteValue < 1e-9) 0.0 else numerator / denominator
}

interface HermesTrajectoryConstraint : VelConstraint, AccelConstraint {
    override fun minMaxProfileAccel(robotState: RobotState, path: PosePath, s: Double): MinMax {
        return MinMax(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)
    }
}

class WPILibTrajectoryConstraint @JvmOverloads constructor(
    val wpilibConstraint: TrajectoryConstraint,
    minAccel: Double = Double.POSITIVE_INFINITY,
    maxAccel: Double = minAccel
) : HermesTrajectoryConstraint {
    val minMaxAccels = MinMax(-minAccel.coerceAtLeast(0.0), maxAccel.coerceAtLeast(0.0))

    override fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double): Double = wpilibConstraint.getMaxVelocity(
            robotState.pose.wpilib,
            path.curvatureAt(s),
            robotState.vel.linearVel.norm()
        )

    override fun minMaxProfileAccel(robotState: RobotState, path: PosePath, s: Double): MinMax = minMaxAccels
}

class DifferentialDriveConstraint(
    val kinematics: DifferentialDriveKinematics,
    maxTransVel: Double,
    minTransAccel: Double = Double.POSITIVE_INFINITY,
    maxTransAccel: Double = minTransAccel
) : HermesTrajectoryConstraint by WPILibTrajectoryConstraint(
    DifferentialDriveKinematicsConstraint(kinematics, maxTransVel),
    minTransAccel,
    maxTransAccel
)

class MecanumDriveConstraint(
    val kinematics: MecanumDriveKinematics,
    maxTransVel: Double,
    minTransAccel: Double = Double.POSITIVE_INFINITY,
    maxTransAccel: Double = minTransAccel
) : HermesTrajectoryConstraint by WPILibTrajectoryConstraint(
    MecanumDriveKinematicsConstraint(kinematics, maxTransVel),
    minTransAccel,
    maxTransAccel
)

class SwerveDriveConstraint(
    val kinematics: SwerveDriveKinematics,
    maxTransVel: Double,
    minTransAccel: Double = Double.POSITIVE_INFINITY,
    maxTransAccel: Double = minTransAccel
) : HermesTrajectoryConstraint by WPILibTrajectoryConstraint(
    SwerveDriveKinematicsConstraint(kinematics, maxTransVel),
    minTransAccel,
    maxTransAccel
)


