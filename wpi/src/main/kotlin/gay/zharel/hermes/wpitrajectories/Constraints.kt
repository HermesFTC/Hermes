package gay.zharel.hermes.wpitrajectories

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint
import edu.wpi.first.math.trajectory.constraint.MecanumDriveKinematicsConstraint
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.kinematics.MecanumKinematics
import gay.zharel.hermes.kinematics.MotorFeedforward
import gay.zharel.hermes.kinematics.RobotKinematics
import gay.zharel.hermes.kinematics.SwerveKinematics
import gay.zharel.hermes.kinematics.TankKinematics
import gay.zharel.hermes.kinematics.VoltageConstraint
import gay.zharel.hermes.kinematics.WheelIncrements
import gay.zharel.hermes.kinematics.WheelVelConstraint
import gay.zharel.hermes.kinematics.WheelVelocities
import gay.zharel.hermes.math.MinMax
import gay.zharel.hermes.paths.PosePath
import gay.zharel.hermes.profiles.AccelConstraint
import gay.zharel.hermes.profiles.VelConstraint
import gay.zharel.hermes.wpiconversions.hermes
import gay.zharel.hermes.wpiconversions.inches
import gay.zharel.hermes.wpiconversions.ips
import gay.zharel.hermes.wpiconversions.meters
import gay.zharel.hermes.wpiconversions.volts
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

open class DriveConstraint(
    kinematics: RobotKinematics<*, *>,
    feedforward: MotorFeedforward,
    maxWheelVel: Double ,
    maxVoltage: Double = 12.0,
) : HermesTrajectoryConstraint {
    constructor(
        kinematics: RobotKinematics<*, *>,
        feedforward: SimpleMotorFeedforward,
        maxWheelVel: LinearVelocity,
        maxVoltage: Voltage = 12.0.volts,
    ) : this(
        kinematics,
        MotorFeedforward(feedforward.ks, feedforward.kv, feedforward.ka),
        maxWheelVel.ips,
        maxVoltage.volts
    )

    val velConstraint = WheelVelConstraint(kinematics, maxWheelVel)
    val accelConstraint = VoltageConstraint(kinematics, feedforward, maxVoltage)

    override fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double): Double {
        return velConstraint.maxRobotVel(robotState, path, s)
    }

    override fun minMaxProfileAccel(robotState: RobotState, path: PosePath, s: Double): MinMax {
        return accelConstraint.minMaxProfileAccel(robotState, path, s)
    }
}

class DifferentialDriveConstraint(
    trackwidth: Distance,
    feedforward: SimpleMotorFeedforward,
    maxWheelVel: LinearVelocity,
    maxVoltage: Voltage = 12.0.volts,
) : DriveConstraint(
    TankKinematics(trackwidth.inches),
    feedforward,
    maxWheelVel,
    maxVoltage
)

class MecanumDriveConstraint(
    trackwidth: Distance,
    wheelbase: Distance,
    feedforward: SimpleMotorFeedforward,
    maxWheelVel: LinearVelocity,
    maxVoltage: Voltage = 12.0.volts,
) : DriveConstraint(
    MecanumKinematics(trackwidth.inches, wheelbase.inches),
    feedforward,
    maxWheelVel,
    maxVoltage
) {
    constructor(
        wheelLocations: Array<Translation2d>,
        feedforward: SimpleMotorFeedforward,
        maxWheelVel: LinearVelocity,
        maxVoltage: Voltage = 12.0.volts,
    ) : this(
        trackwidth = (((wheelLocations[0].x - wheelLocations[1].x).absoluteValue +
                      (wheelLocations[2].x - wheelLocations[3].x).absoluteValue) / 2.0).meters,
        wheelbase = (((wheelLocations[0].y - wheelLocations[2].y).absoluteValue +
                     (wheelLocations[1].y - wheelLocations[3].y).absoluteValue) / 2.0).meters,
        feedforward,
        maxWheelVel,
        maxVoltage
    )
}

class SwerveDriveConstraint(
    wheelLocations: Array<Translation2d>,
    feedforward: SimpleMotorFeedforward,
    maxWheelVel: LinearVelocity,
    maxVoltage: Voltage = 12.0.volts,
) : DriveConstraint(
    SwerveKinematics(wheelLocations.map(Translation2d::hermes)),
    feedforward,
    maxWheelVel,
    maxVoltage
)