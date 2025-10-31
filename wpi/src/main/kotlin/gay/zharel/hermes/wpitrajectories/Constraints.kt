package gay.zharel.hermes.wpitrajectories

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.kinematics.MecanumKinematics
import gay.zharel.hermes.kinematics.MotorFeedforward
import gay.zharel.hermes.kinematics.RobotKinematics
import gay.zharel.hermes.kinematics.SwerveKinematics
import gay.zharel.hermes.kinematics.TankKinematics
import gay.zharel.hermes.kinematics.VoltageConstraint
import gay.zharel.hermes.kinematics.WheelVelConstraint
import gay.zharel.hermes.math.MinMax
import gay.zharel.hermes.paths.PosePath
import gay.zharel.hermes.profiles.AccelConstraint
import gay.zharel.hermes.profiles.VelConstraint
import gay.zharel.hermes.wpiconversions.hermes
import gay.zharel.hermes.wpiconversions.inches
import gay.zharel.hermes.wpiconversions.ips
import gay.zharel.hermes.wpiconversions.meters
import gay.zharel.hermes.wpiconversions.volts
import kotlin.math.absoluteValue

/**
 * A combined velocity and acceleration constraint interface for Hermes trajectory planning.
 *
 * This interface combines both velocity and acceleration constraints, allowing a single
 * constraint object to enforce both types of limits simultaneously.
 */
interface HermesTrajectoryConstraint : VelConstraint, AccelConstraint {
    override fun minMaxProfileAccel(robotState: RobotState, path: PosePath, s: Double): MinMax {
        return MinMax(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)
    }
}

/**
 * Base drive constraint class that combines wheel velocity and voltage-based acceleration constraints.
 *
 * This constraint enforces physical limits on the robot's wheels by combining:
 * - Maximum wheel velocity constraints
 * - Voltage-based acceleration constraints using motor feedforward
 *
 * @param kinematics The robot kinematics model
 * @param feedforward The motor feedforward model
 * @param maxWheelVel Maximum wheel velocity in inches per second
 * @param maxVoltage Maximum voltage available to motors (default: 12.0V)
 */
open class DriveConstraint(
    kinematics: RobotKinematics<*, *>,
    feedforward: MotorFeedforward,
    maxWheelVel: Double ,
    maxVoltage: Double = 12.0,
) : HermesTrajectoryConstraint {
    /**
     * Creates a DriveConstraint using WPILib units and SimpleMotorFeedforward.
     *
     * @param kinematics The robot kinematics model
     * @param feedforward WPILib SimpleMotorFeedforward for motor modeling
     * @param maxWheelVel Maximum wheel velocity (WPILib LinearVelocity)
     * @param maxVoltage Maximum voltage available to motors (default: 12.0V)
     */
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

/**
 * Velocity and acceleration constraint for differential drive robots.
 *
 * Differential drives (also known as tank drives) have two independently controlled
 * wheel sides. This constraint enforces velocity and acceleration limits based on
 * the trackwidth and motor characteristics.
 *
 * @param trackwidth The distance between the left and right wheel centers
 * @param feedforward Motor feedforward model for acceleration constraints
 * @param maxWheelVel Maximum velocity of the drive wheels
 * @param maxVoltage Maximum voltage available to motors (default: 12.0V)
 */
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

/**
 * Velocity and acceleration constraint for mecanum drive robots.
 *
 * Mecanum drives use four omni-directional wheels that allow holonomic movement.
 * This constraint enforces velocity and acceleration limits based on the robot's
 * geometry and motor characteristics.
 *
 * @param trackwidth The distance between the left and right wheels
 * @param wheelbase The distance between the front and rear wheels
 * @param feedforward Motor feedforward model for acceleration constraints
 * @param maxWheelVel Maximum velocity of the mecanum wheels
 * @param maxVoltage Maximum voltage available to motors (default: 12.0V)
 */
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
    /**
     * Creates a MecanumDriveConstraint from wheel locations.
     *
     * The wheel locations should be provided in the order:
     * front-left, front-right, back-left, back-right.
     *
     * @param wheelLocations Array of exactly 4 wheel positions (front-left, front-right, back-left, back-right)
     * @param feedforward Motor feedforward model for acceleration constraints
     * @param maxWheelVel Maximum velocity of the mecanum wheels
     * @param maxVoltage Maximum voltage available to motors (default: 12.0V)
     * @throws IllegalArgumentException if wheelLocations does not contain exactly 4 elements
     */
    constructor(
        wheelLocations: Array<Translation2d>,
        feedforward: SimpleMotorFeedforward,
        maxWheelVel: LinearVelocity,
        maxVoltage: Voltage = 12.0.volts,
    ) : this(
        trackwidth = wheelLocations.let {
            require(it.size == 4) { "Mecanum drive requires exactly 4 wheel locations" }
            (((it[0].x - it[1].x).absoluteValue +
                      (it[2].x - it[3].x).absoluteValue) / 2.0).meters
        },
        wheelbase = (((wheelLocations[0].y - wheelLocations[2].y).absoluteValue +
                     (wheelLocations[1].y - wheelLocations[3].y).absoluteValue) / 2.0).meters,
        feedforward,
        maxWheelVel,
        maxVoltage
    )
}

/**
 * Velocity and acceleration constraint for swerve drive robots.
 *
 * Swerve drives use independently steerable and powered wheels that allow
 * full holonomic movement with maximum maneuverability. This constraint enforces
 * velocity and acceleration limits based on the wheel positions and motor characteristics.
 *
 * Swerve drives typically use 4 modules, but this constraint supports any number of modules.
 *
 * @param wheelLocations Array of wheel module positions relative to the robot center
 * @param feedforward Motor feedforward model for acceleration constraints
 * @param maxWheelVel Maximum velocity of the swerve modules
 * @param maxVoltage Maximum voltage available to motors (default: 12.0V)
 */
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
