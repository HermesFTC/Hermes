/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.MecanumDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import gay.zharel.hermes.kinematics.MecanumKinematics
import gay.zharel.hermes.kinematics.SwerveKinematics
import gay.zharel.hermes.wpiconversions.hermes

// Neo Motor Constants
const val NEO_FREE_SPEED_RPM: Double = 5676.0

// Drive Constants
const val NOMINAL_VOLTAGE: Double = 12.0

// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
const val MAX_SPEED_METERS_PER_SECOND: Double = 4.8
const val MAX_ANGULAR_SPEED: Double = 2 * Math.PI // radians per second

// Chassis configuration
val TRACK_WIDTH: Double = Units.inchesToMeters(26.5) // Distance between centers of right and left wheels on robot
val WHEEL_BASE: Double = Units.inchesToMeters(26.5) // Distance between front and back wheels on robot

val SWERVE_DRIVE_KINEMATICS: SwerveDriveKinematics = SwerveDriveKinematics(
    Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
    Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
    Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
)

val SWERVE_KINEMATICS_HERMES = SwerveKinematics(listOf(
    Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2).hermes,
    Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2).hermes,
    Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2).hermes,
    Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2).hermes
))

val MECANUM_DRIVE_KINEMATICS: MecanumDriveKinematics = MecanumDriveKinematics(
    Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
    Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
    Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
)

val MECANUM_KINEMATICS_HERMES = MecanumKinematics(TRACK_WIDTH, WHEEL_BASE)

// Angular offsets of the modules relative to the chassis in radians
const val FRONT_LEFT_CHASSIS_ANGULAR_OFFSET: Double = -Math.PI / 2
const val FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET: Double = 0.0
const val BACK_LEFT_CHASSIS_ANGULAR_OFFSET: Double = Math.PI
const val BACK_RIGHT_CHASSIS_ANGULAR_OFFSET: Double = Math.PI / 2

// SPARK MAX CAN IDs
const val FRONT_LEFT_DRIVING_CAN_ID: Int = 11
const val REAR_LEFT_DRIVING_CAN_ID: Int = 13
const val FRONT_RIGHT_DRIVING_CAN_ID: Int = 15
const val REAR_RIGHT_DRIVING_CAN_ID: Int = 17

const val FRONT_LEFT_TURNING_CAN_ID: Int = 10
const val REAR_LEFT_TURNING_CAN_ID: Int = 12
const val FRONT_RIGHT_TURNING_CAN_ID: Int = 14
const val REAR_RIGHT_TURNING_CAN_ID: Int = 16

const val GYRO_REVERSED: Boolean = false

// Module Constants
// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
const val DRIVING_MOTOR_PINION_TEETH: Int = 14

// Calculations required for driving motor conversion factors and feed forward
const val DRIVING_MOTOR_FREE_SPEED_RPS: Double = NEO_FREE_SPEED_RPM / 60
const val WHEEL_DIAMETER_METERS: Double = 0.0762
const val WHEEL_CIRCUMFERENCE_METERS: Double = WHEEL_DIAMETER_METERS * Math.PI

// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
const val DRIVING_MOTOR_REDUCTION: Double = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15)
const val DRIVE_WHEEL_FREE_SPEED_RPS: Double = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION

// Operator Input Constants
const val DRIVER_CONTROLLER_PORT: Int = 0
const val DRIVE_DEADBAND: Double = 0.05

// Auto Constants
const val AUTO_MAX_SPEED_METERS_PER_SECOND: Double = 3.0
const val AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED: Double = 3.0
const val AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND: Double = Math.PI
const val AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED: Double = Math.PI

const val K_STATIC: Double = 0.8
const val K_VELOCITY: Double = 2.38
const val K_ACCELERATION: Double = 0.39

val AUTO_FEEDFORWARD: SimpleMotorFeedforward = SimpleMotorFeedforward(K_STATIC, K_VELOCITY, K_ACCELERATION)

const val PX_CONTROLLER: Double = 1.0
const val PY_CONTROLLER: Double = 1.0
const val P_THETA_CONTROLLER: Double = 1.0

// Constraint for the motion profiled robot angle controller
val THETA_CONTROLLER_CONSTRAINTS: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(
    AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
    AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED
)
