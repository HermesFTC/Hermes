/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.kinematics

import gay.zharel.hermes.TEST_PROFILE_PARAMS
import gay.zharel.hermes.control.concat
import gay.zharel.hermes.geometry.Acceleration2d
import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.geometry.Rotation2d
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.math.Time
import gay.zharel.hermes.math.range
import gay.zharel.hermes.paths.TangentPath
import gay.zharel.hermes.paths.saveProfiles
import gay.zharel.hermes.profiles.CancelableProfile
import gay.zharel.hermes.profiles.ProfileAccelConstraint
import gay.zharel.hermes.profiles.TimeProfile
import gay.zharel.hermes.trajectories.PositionPathSeqBuilder
import kotlin.math.PI
import kotlin.math.abs
import kotlin.random.Random
import kotlin.test.Test
import kotlin.test.assertEquals

private const val EPSILON = 1e-5

class KinematicsTest {
    @Test
    fun testMecKinematicsComposition() {
        val kinematics = MecanumKinematics(10.0, 10.0)

        val r = Random.Default
        repeat(100) {
            val t = PoseVelocity2d(
                Vector2d(r.nextDouble(), r.nextDouble()),
                r.nextDouble()
            )

            val vs = kinematics.inverse(PoseVelocity2dDual.Companion.constant<Time>(t, 1)).all()

            val t2 = kinematics.forward<Time>(
                MecanumKinematics.MecanumWheelIncrements(
                    vs[0], vs[1], vs[2], vs[3],
                )
            ).value()

            assertEquals(t.linearVel.x, t2.line.x, EPSILON)
            assertEquals(t.linearVel.y, t2.line.y, EPSILON)
            assertEquals(t.angVel, t2.angle, EPSILON)
        }
    }

    @Test
    fun testMecWheelVelocityLimiting() {
        val kinematics = MecanumKinematics(10.0, 10.0)

        val posPath = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            Rotation2d.Companion.exp(0.0),
            EPSILON,
        )
            .splineTo(
                Vector2d(15.0, 15.0),
                Rotation2d.Companion.exp(PI),
            )
            .splineTo(
                Vector2d(5.0, 35.0),
                Rotation2d.Companion.exp(PI / 3),
            )
            .build()
            .first()

        val path = TangentPath(posPath, 0.0)
        val profile = CancelableProfile.Companion.generate(
            TEST_PROFILE_PARAMS,
            path, 0.0,
            WheelVelConstraint(kinematics, 10.0),
            ProfileAccelConstraint(-10.0, 10.0),
        ).baseProfile

        val ts = range(0.0, profile.disps.last(), 100)
        val maxWheelVelMag = ts.maxOf { time ->
            val s = profile[time]
            val pose = path[s.value(), 2].reparam(s)
            kinematics.inverse(pose.inverse() * pose.velocity())
                .all()
                .map { it.value() }
                .maxOf { abs(it) }
        }
        assert(maxWheelVelMag < 10.1)

        saveProfiles("mec", TimeProfile(profile))
    }

    @Test
    fun testTankKinematicsComposition() {
        val kinematics = TankKinematics(10.0)

        val r = Random.Default
        repeat(100) {
            val t = PoseVelocity2d(
                Vector2d(r.nextDouble(), 0.0),
                r.nextDouble()
            )

            val vs = kinematics.inverse(PoseVelocity2dDual.Companion.constant<Time>(t, 1)).all()

            val t2 = kinematics.forward<Time>(
                TankKinematics.TankWheelIncrements(
                    vs[0], vs[1],
                )
            ).value()

            assertEquals(t.linearVel.x, t2.line.x, EPSILON)
            assertEquals(t.linearVel.y, t2.line.y, EPSILON)
            assertEquals(t.angVel, t2.angle, EPSILON)
        }
    }

    @Test
    fun testTankWheelVelocityLimiting() {
        val kinematics = TankKinematics(10.0)

        val posPath = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            Rotation2d.Companion.exp(0.0),
            EPSILON,
        )
            .splineTo(
                Vector2d(15.0, 15.0),
                Rotation2d.Companion.exp(PI),
            )
            .splineTo(
                Vector2d(5.0, 35.0),
                Rotation2d.Companion.exp(PI / 3),
            )
            .build()
            .first()

        val path = TangentPath(posPath, 0.0)
        val profile = CancelableProfile.Companion.generate(
            TEST_PROFILE_PARAMS,
            path, 0.0,
            WheelVelConstraint(kinematics,10.0),
            ProfileAccelConstraint(-10.0, 10.0),
        ).baseProfile

        val ts = range(0.0, profile.disps.last(), 100)
        val maxWheelVelMag = ts.maxOf { time ->
            val s = profile[time]
            val pose = path[s.value(), 2].reparam(s)
            kinematics.inverse(pose.inverse() * pose.velocity())
                .all()
                .map { it.value() }
                .maxOf { abs(it) }
        }
        assert(maxWheelVelMag < 10.1)

        saveProfiles("tank", TimeProfile(profile))
    }

    @Test
    fun testMecanumInverseAccelerationsStraightLine() {
        // Match WPILib geometry: modules at (±12, ±12) => effectiveRadius = 24
        val kinematics = MecanumKinematics(24.0, 24.0)
        val cmd = PoseVelocity2dDual<Time>(
            gay.zharel.hermes.geometry.Vector2dDual(
                gay.zharel.hermes.math.DualNum(doubleArrayOf(0.0, 5.0)),
                gay.zharel.hermes.math.DualNum(doubleArrayOf(0.0, 0.0))
            ),
            gay.zharel.hermes.math.DualNum(doubleArrayOf(0.0, 0.0))
        )
        val wheels = kinematics.inverse(cmd)
        assertEquals(5.0, wheels.leftFront[1], EPSILON)
        assertEquals(5.0, wheels.rightFront[1], EPSILON)
        assertEquals(5.0, wheels.leftBack[1], EPSILON)
        assertEquals(5.0, wheels.rightBack[1], EPSILON)
    }

    @Test
    fun testMecanumForwardAccelerationsStraightLine() {
        val kinematics = MecanumKinematics(24.0, 24.0)
        fun dn(a: Double) = DualNum<Time>(doubleArrayOf(0.0, a))
        val wheels = MecanumKinematics.MecanumWheelVelocities(
            dn(3.536), dn(3.536), dn(3.536), dn(3.536)
        )
        val chassis = kinematics.forward<Time>(wheels).acceleration()
        assertEquals(3.536, chassis.linearAcc.x, EPSILON)
        assertEquals(0.0, chassis.linearAcc.y, EPSILON)
        assertEquals(0.0, chassis.angAcc, EPSILON)
    }

    @Test
    fun testMecanumInverseAccelerationsStrafe() {
        val kinematics = MecanumKinematics(24.0, 24.0)
        val cmd = PoseVelocity2dDual<Time>(
            gay.zharel.hermes.geometry.Vector2dDual(
                gay.zharel.hermes.math.DualNum<Time>(doubleArrayOf(0.0, 0.0)),
                gay.zharel.hermes.math.DualNum<Time>(doubleArrayOf(0.0, 4.0))
            ),
            gay.zharel.hermes.math.DualNum<Time>(doubleArrayOf(0.0, 0.0))
        )
        val wheels = kinematics.inverse(cmd)
        assertEquals(-4.0, wheels.leftFront[1], EPSILON)
        assertEquals(4.0, wheels.rightFront[1], EPSILON)
        assertEquals(4.0, wheels.leftBack[1], EPSILON)
        assertEquals(-4.0, wheels.rightBack[1], EPSILON)
    }

    @Test
    fun testMecanumForwardAccelerationsStrafe() {
        val kinematics = MecanumKinematics(24.0, 24.0)
        fun dn(a: Double) = gay.zharel.hermes.math.DualNum<Time>(doubleArrayOf(0.0, a))
        val wheels = MecanumKinematics.MecanumWheelVelocities(
            dn(-2.828427), dn(2.828427), dn(-2.828427), dn(2.828427)
        )
        val chassis = kinematics.forward<Time>(wheels).acceleration()
        assertEquals(0.0, chassis.linearAcc.x, EPSILON)
        assertEquals(2.8284, chassis.linearAcc.y, 1e-4)
        assertEquals(0.0, chassis.angAcc, EPSILON)
    }

    @Test
    fun testMecanumInverseAccelerationsRotation() {
        val kinematics = MecanumKinematics(24.0, 24.0)
        val cmd = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
            .concat<Time>(Acceleration2d(Vector2d(0.0, 0.0), 2 * PI))
        val wheels = kinematics.inverse(cmd)
        assertEquals(-150.79645, wheels.leftFront[1], EPSILON)
        assertEquals(150.79645, wheels.rightFront[1], EPSILON)
        assertEquals(-150.79645, wheels.leftBack[1], EPSILON)
        assertEquals(150.79645, wheels.rightBack[1], EPSILON)
    }

    @Test
    fun testMecanumForwardAccelerationsRotation() {
        val kinematics = MecanumKinematics(24.0, 24.0)
        fun dn(a: Double) = DualNum<Time>(doubleArrayOf(0.0, a))
        val wheels = MecanumKinematics.MecanumWheelVelocities(
            dn(-150.79645), dn(-150.79645), dn(150.79645), dn(150.79645)
        )
        val chassis = kinematics.forward<Time>(wheels).acceleration()
        assertEquals(0.0, chassis.linearAcc.x, EPSILON)
        assertEquals(0.0, chassis.linearAcc.y, EPSILON)
        assertEquals(2 * PI, chassis.angAcc, EPSILON)
    }

    @Test
    fun testMecanumInverseAccelerationsMixed() {
        val kinematics = MecanumKinematics(24.0, 24.0)
        val cmd = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
            .concat<Time>(Acceleration2d(Vector2d(2.0, 3.0), 1.0))
        val wheels = kinematics.inverse(cmd)
        assertEquals(-25.0, wheels.leftFront[1], EPSILON)
        assertEquals(23.0, wheels.rightBack[1], EPSILON)
        assertEquals(29.0, wheels.rightFront[1], EPSILON)
        assertEquals(-19.0, wheels.leftBack[1], EPSILON)
    }

    @Test
    fun testMecanumForwardAccelerationsMixed() {
        val kinematics = MecanumKinematics(24.0, 24.0)
        fun dn(a: Double) = DualNum<Time>(doubleArrayOf(0.0, a))
        // WPILib order: FL, FR, RL, RR -> Hermes order: LF, LB, RB, RF
        val wheels = MecanumKinematics.MecanumWheelVelocities(
            dn(-17.677670), dn(-13.44), dn(16.26), dn(20.51)
        )
        val chassis = kinematics.forward<Time>(wheels).acceleration()
        assertEquals(1.413, chassis.linearAcc.x, 1e-3)
        assertEquals(2.122, chassis.linearAcc.y, 1e-3)
        assertEquals(0.707, chassis.angAcc, 1e-3)
    }

    @Test
    fun testTankInverseAccelerationsStraightLine() {
        val kinematics = TankKinematics(0.381 * 2)
        val cmd = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
            .concat<Time>(Acceleration2d(Vector2d(3.0, 0.0), 0.0))
        val wheels = kinematics.inverse(cmd)
        assertEquals(3.0, wheels.left[1], 1e-9)
        assertEquals(3.0, wheels.right[1], 1e-9)
    }

    @Test
    fun testTankForwardAccelerationsStraightLine() {
        val kinematics = TankKinematics(0.381 * 2)
        fun dn(a: Double) = DualNum<Time>(doubleArrayOf(0.0, a))
        val wheels = TankKinematics.TankWheelVelocities(
            dn(3.0), dn(3.0)
        )
        val chassis = kinematics.forward<Time>(wheels).acceleration()
        assertEquals(3.0, chassis.linearAcc.x, 1e-9)
        assertEquals(0.0, chassis.linearAcc.y, 1e-9)
        assertEquals(0.0, chassis.angAcc, 1e-9)
    }

    @Test
    fun testTankInverseAccelerationsRotateInPlace() {
        val kinematics = TankKinematics(0.381 * 2)
        val cmd = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
            .concat<Time>(Acceleration2d(Vector2d(0.0, 0.0), PI))
        val wheels = kinematics.inverse(cmd)
        assertEquals(-0.381 * PI, wheels.left[1], 1e-9)
        assertEquals(+0.381 * PI, wheels.right[1], 1e-9)
    }

    @Test
    fun testTankForwardAccelerationsRotateInPlace() {
        val kinematics = TankKinematics(0.381 * 2)
        fun dn(a: Double) = DualNum<Time>(doubleArrayOf(0.0, a))
        val wheels = TankKinematics.TankWheelVelocities(
            dn(+0.381 * PI), dn(-0.381 * PI)
        )
        val chassis = kinematics.forward<Time>(wheels).acceleration()
        assertEquals(0.0, chassis.linearAcc.x, 1e-9)
        assertEquals(0.0, chassis.linearAcc.y, 1e-9)
        assertEquals(-PI, chassis.angAcc, 1e-9)
    }
}