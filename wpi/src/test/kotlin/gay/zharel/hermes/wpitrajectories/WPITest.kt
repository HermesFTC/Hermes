/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.wpitrajectories

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import gay.zharel.hermes.AUTO_FEEDFORWARD
import gay.zharel.hermes.MAX_SPEED_METERS_PER_SECOND
import gay.zharel.hermes.NOMINAL_VOLTAGE
import gay.zharel.hermes.SWERVE_DRIVE_KINEMATICS
import gay.zharel.hermes.TRACK_WIDTH
import gay.zharel.hermes.WHEEL_BASE
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.trajectories.TrajectoryBuilder
import gay.zharel.hermes.wpiconversions.HPose2d
import gay.zharel.hermes.wpiconversions.hermes
import gay.zharel.hermes.wpiconversions.meters
import org.junit.jupiter.api.Assertions.assertDoesNotThrow
import kotlin.test.Test

class WPITest {
    // Create constraints for trajectory
    val swerveConstraints = SwerveDriveConstraint(
        SWERVE_DRIVE_KINEMATICS.modules,
        AUTO_FEEDFORWARD,
        Units.MetersPerSecond.of(MAX_SPEED_METERS_PER_SECOND),
        Units.Volts.of(NOMINAL_VOLTAGE)
    )

    val mecanumConstraints = MecanumDriveConstraint(
        TRACK_WIDTH.meters,
        WHEEL_BASE.meters,
        AUTO_FEEDFORWARD,
        Units.MetersPerSecond.of(MAX_SPEED_METERS_PER_SECOND),
        Units.Volts.of(NOMINAL_VOLTAGE)
    )

    @Test
    fun main() {


        // An example trajectory to follow.  All units in meters.
//    val builder = SwerveTrajectoryBuilder(
//        SWERVE_DRIVE_KINEMATICS,
//        swerveConstraints,
//        Pose2d.kZero
//    ).bezierTo(Translation2d(1.0, 1.0), Translation2d(2.0, -1.0))
//        .splineTo(Translation2d(3.0, 0.0), Rotation2d(0.0))

        val swerveBuilder = TrajectoryBuilder(
            DEFAULT_BUILDER_PARAMS,
            HPose2d.zero,
            0.0,
            swerveConstraints,
            swerveConstraints,
        ).bezierTo(Vector2d(1.0, 1.0), Vector2d(2.0, -1.0))
            .splineTo(Vector2d(3.0, 0.0), Rotation2d(0.0).hermes)

        val mecanumBuilder = TrajectoryBuilder(
            DEFAULT_BUILDER_PARAMS,
            HPose2d.zero,
            0.0,
            mecanumConstraints,
            mecanumConstraints,
        ).bezierTo(Vector2d(1.0, 1.0), Vector2d(2.0, -1.0))
            .splineTo(Vector2d(3.0, 0.0), Rotation2d(0.0).hermes)

        val swerveTraj = swerveBuilder.buildToComposite().wrtDisp()
        println(swerveTraj.profile)

        val mecanumTraj = mecanumBuilder.buildToComposite().wrtDisp()
        println(mecanumTraj.profile)

        assertDoesNotThrow {
            println(swerveTraj.wrtTime())
            println(swerveTraj.wrtTime().length())
            println(swerveTraj.wrtTime().duration())

            println(mecanumTraj.wrtTime())
            println(mecanumTraj.wrtTime().length())
            println(mecanumTraj.wrtTime().duration())
        }
    }
}