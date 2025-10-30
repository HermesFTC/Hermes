/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.kinematics

import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.Time
import kotlin.math.PI
import kotlin.math.sqrt
import kotlin.test.Test
import kotlin.test.assertEquals

private const val EPSILON = 1e-9

class SwerveKinematicsTest {
    private val fl = Vector2d(12.0, 12.0)
    private val fr = Vector2d(12.0, -12.0)
    private val bl = Vector2d(-12.0, 12.0)
    private val br = Vector2d(-12.0, -12.0)

    private val kinematics = SwerveKinematics(listOf(fl, fr, bl, br))

    @Test
    fun testStraightLineInverseKinematics() {
        // Test inverse kinematics going in a straight line
        val velocity = PoseVelocity2d(Vector2d(5.0, 0.0), 0.0)
        val moduleStates = kinematics.inverse(PoseVelocity2dDual.constant<Time>(velocity, 1))

        assertEquals(5.0, moduleStates.states[0].velocity.value(), EPSILON)
        assertEquals(5.0, moduleStates.states[1].velocity.value(), EPSILON)
        assertEquals(5.0, moduleStates.states[2].velocity.value(), EPSILON)
        assertEquals(5.0, moduleStates.states[3].velocity.value(), EPSILON)
        assertEquals(0.0, moduleStates.states[0].angle.value(), EPSILON)
        assertEquals(0.0, moduleStates.states[1].angle.value(), EPSILON)
        assertEquals(0.0, moduleStates.states[2].angle.value(), EPSILON)
        assertEquals(0.0, moduleStates.states[3].angle.value(), EPSILON)
    }

    @Test
    fun testStraightLineForwardKinematics() {
        // Test forward kinematics going in a straight line
        val state = SwerveModuleState<Time>(
            DualNum.constant(5.0, 1),
            DualNum.constant(0.0, 1)
        )
        val velocities = SwerveKinematics.SwerveWheelVelocities(listOf(state, state, state, state))
        val chassisVel = kinematics.forward<Time>(velocities)

        assertEquals(5.0, chassisVel.linearVel.x.value(), EPSILON)
        assertEquals(0.0, chassisVel.linearVel.y.value(), EPSILON)
        assertEquals(0.0, chassisVel.angVel.value(), EPSILON)
    }

    @Test
    fun testStraightLineForwardKinematicsWithDeltas() {
        // Test forward kinematics going in a straight line with increments
        val delta = SwerveModuleIncrements<Time>(DualNum.constant(5.0, 1), 0.0)
        val increments = SwerveKinematics.SwerveWheelIncrements(listOf(delta, delta, delta, delta))
        val twist = kinematics.forward<Time>(increments)

        assertEquals(5.0, twist.line.x.value(), EPSILON)
        assertEquals(0.0, twist.line.y.value(), EPSILON)
        assertEquals(0.0, twist.angle.value(), EPSILON)
    }

    @Test
    fun testStraightStrafeInverseKinematics() {
        val velocity = PoseVelocity2d(Vector2d(0.0, 5.0), 0.0)
        val moduleStates = kinematics.inverse(PoseVelocity2dDual.constant<Time>(velocity, 1))

        assertEquals(5.0, moduleStates.states[0].velocity.value(), EPSILON)
        assertEquals(5.0, moduleStates.states[1].velocity.value(), EPSILON)
        assertEquals(5.0, moduleStates.states[2].velocity.value(), EPSILON)
        assertEquals(5.0, moduleStates.states[3].velocity.value(), EPSILON)
        assertEquals(90.0, Math.toDegrees(moduleStates.states[0].angle.value()), EPSILON)
        assertEquals(90.0, Math.toDegrees(moduleStates.states[1].angle.value()), EPSILON)
        assertEquals(90.0, Math.toDegrees(moduleStates.states[2].angle.value()), EPSILON)
        assertEquals(90.0, Math.toDegrees(moduleStates.states[3].angle.value()), EPSILON)
    }

    @Test
    fun testStraightStrafeForwardKinematics() {
        val state = SwerveModuleState<Time>(
            DualNum.constant(5.0, 1),
            DualNum.constant(PI / 2, 1)
        )
        val velocities = SwerveKinematics.SwerveWheelVelocities(listOf(state, state, state, state))
        val chassisVel = kinematics.forward<Time>(velocities)

        assertEquals(0.0, chassisVel.linearVel.x.value(), EPSILON)
        assertEquals(5.0, chassisVel.linearVel.y.value(), EPSILON)
        assertEquals(0.0, chassisVel.angVel.value(), EPSILON)
    }

    @Test
    fun testStraightStrafeForwardKinematicsWithDeltas() {
        val delta = SwerveModuleIncrements<Time>(DualNum.constant(5.0, 1), PI / 2)
        val increments = SwerveKinematics.SwerveWheelIncrements(listOf(delta, delta, delta, delta))
        val twist = kinematics.forward<Time>(increments)

        assertEquals(0.0, twist.line.x.value(), EPSILON)
        assertEquals(5.0, twist.line.y.value(), EPSILON)
        assertEquals(0.0, twist.angle.value(), EPSILON)
    }

    @Test
    fun testTurnInPlaceInverseKinematics() {
        val velocity = PoseVelocity2d(Vector2d(0.0, 0.0), 2 * PI)
        val moduleStates = kinematics.inverse(PoseVelocity2dDual.constant<Time>(velocity, 1))

        // The circumference of the wheels about the COR is π * diameter, or 2π * radius
        // the radius is the √(12²in + 12²in), or 16.9706in, so the circumference the wheels
        // trace out is 106.629190516in. since we want our robot to rotate at 1 rotation per second,
        // our wheels must trace out 1 rotation (or 106.63 inches) per second.

        assertEquals(106.63, moduleStates.states[0].velocity.value(), 0.1)
        assertEquals(106.63, moduleStates.states[1].velocity.value(), 0.1)
        assertEquals(106.63, moduleStates.states[2].velocity.value(), 0.1)
        assertEquals(106.63, moduleStates.states[3].velocity.value(), 0.1)
        assertEquals(135.0, Math.toDegrees(moduleStates.states[0].angle.value()), EPSILON)
        assertEquals(45.0, Math.toDegrees(moduleStates.states[1].angle.value()), EPSILON)
        assertEquals(-135.0, Math.toDegrees(moduleStates.states[2].angle.value()), EPSILON)
        assertEquals(-45.0, Math.toDegrees(moduleStates.states[3].angle.value()), EPSILON)
    }

    @Test
    fun testTurnInPlaceForwardKinematics() {
        val flState = SwerveModuleState<Time>(
            DualNum.constant(106.629, 1),
            DualNum.constant(Math.toRadians(135.0), 1)
        )
        val frState = SwerveModuleState<Time>(
            DualNum.constant(106.629, 1),
            DualNum.constant(Math.toRadians(45.0), 1)
        )
        val blState = SwerveModuleState<Time>(
            DualNum.constant(106.629, 1),
            DualNum.constant(Math.toRadians(-135.0), 1)
        )
        val brState = SwerveModuleState<Time>(
            DualNum.constant(106.629, 1),
            DualNum.constant(Math.toRadians(-45.0), 1)
        )

        val velocities = SwerveKinematics.SwerveWheelVelocities(listOf(flState, frState, blState, brState))
        val chassisVel = kinematics.forward<Time>(velocities)

        assertEquals(0.0, chassisVel.linearVel.x.value(), EPSILON)
        assertEquals(0.0, chassisVel.linearVel.y.value(), EPSILON)
        assertEquals(2 * PI, chassisVel.angVel.value(), 0.1)
    }

    @Test
    fun testTurnInPlaceForwardKinematicsWithDeltas() {
        val flDelta = SwerveModuleIncrements<Time>(DualNum.constant(106.629, 1), Math.toRadians(135.0))
        val frDelta = SwerveModuleIncrements<Time>(DualNum.constant(106.629, 1), Math.toRadians(45.0))
        val blDelta = SwerveModuleIncrements<Time>(DualNum.constant(106.629, 1), Math.toRadians(-135.0))
        val brDelta = SwerveModuleIncrements<Time>(DualNum.constant(106.629, 1), Math.toRadians(-45.0))

        val increments = SwerveKinematics.SwerveWheelIncrements(listOf(flDelta, frDelta, blDelta, brDelta))
        val twist = kinematics.forward<Time>(increments)

        assertEquals(0.0, twist.line.x.value(), EPSILON)
        assertEquals(0.0, twist.line.y.value(), EPSILON)
        assertEquals(2 * PI, twist.angle.value(), 0.1)
    }

    @Test
    fun testCombinedTranslationAndRotationInverseKinematics() {
        // Test combined translation and rotation about robot center
        val velocity = PoseVelocity2d(Vector2d(0.0, 3.0), 1.5)
        val moduleStates = kinematics.inverse(PoseVelocity2dDual.constant<Time>(velocity, 1))

        // All modules should have non-zero velocities
        for (state in moduleStates.states) {
            assert(state.velocity.value() > 0.0) { "Module velocity should be positive" }
        }

        // Forward kinematics should reconstruct the original velocity
        val reconstructed = kinematics.forward<Time>(moduleStates)
        assertEquals(velocity.linearVel.x, reconstructed.linearVel.x.value(), 1e-6)
        assertEquals(velocity.linearVel.y, reconstructed.linearVel.y.value(), 1e-6)
        assertEquals(velocity.angVel, reconstructed.angVel.value(), 1e-6)
    }

    @Test
    fun testCombinedMotionSymmetry() {
        // Test that symmetric module configurations produce expected results
        val velocity = PoseVelocity2d(Vector2d(5.0, 5.0), 0.0)
        val moduleStates = kinematics.inverse(PoseVelocity2dDual.constant<Time>(velocity, 1))

        // For diagonal motion with no rotation, all modules should have same speed and angle
        val speeds = moduleStates.states.map { it.velocity.value() }
        val angles = moduleStates.states.map { it.angle.value() }

        // All speeds should be equal for this motion
        val avgSpeed = speeds.average()
        speeds.forEach { speed ->
            assertEquals(avgSpeed, speed, 1e-6)
        }

        // All angles should be equal (45 degrees for diagonal)
        val avgAngle = angles.average()
        angles.forEach { angle ->
            assertEquals(avgAngle, angle, 1e-6)
        }
        assertEquals(Math.toRadians(45.0), avgAngle, 0.01)
    }

    @Test
    fun testDesaturate() {
        val fl = SwerveModuleState<Time>(DualNum.constant(5.0, 1), DualNum.constant(0.0, 1))
        val fr = SwerveModuleState<Time>(DualNum.constant(6.0, 1), DualNum.constant(0.0, 1))
        val bl = SwerveModuleState<Time>(DualNum.constant(4.0, 1), DualNum.constant(0.0, 1))
        val br = SwerveModuleState<Time>(DualNum.constant(7.0, 1), DualNum.constant(0.0, 1))

        val velocities = SwerveKinematics.SwerveWheelVelocities(listOf(fl, fr, bl, br))
        val desaturated = velocities.desaturate(5.5)

        val factor = 5.5 / 7.0

        assertEquals(5.0 * factor, desaturated.states[0].velocity.value(), EPSILON)
        assertEquals(6.0 * factor, desaturated.states[1].velocity.value(), EPSILON)
        assertEquals(4.0 * factor, desaturated.states[2].velocity.value(), EPSILON)
        assertEquals(7.0 * factor, desaturated.states[3].velocity.value(), EPSILON)
    }

    @Test
    fun testDesaturateNegativeSpeed() {
        val fl = SwerveModuleState<Time>(DualNum.constant(1.0, 1), DualNum.constant(0.0, 1))
        val fr = SwerveModuleState<Time>(DualNum.constant(1.0, 1), DualNum.constant(0.0, 1))
        val bl = SwerveModuleState<Time>(DualNum.constant(-2.0, 1), DualNum.constant(0.0, 1))
        val br = SwerveModuleState<Time>(DualNum.constant(-2.0, 1), DualNum.constant(0.0, 1))

        val velocities = SwerveKinematics.SwerveWheelVelocities(listOf(fl, fr, bl, br))
        val desaturated = velocities.desaturate(1.0)

        assertEquals(0.5, desaturated.states[0].velocity.value(), EPSILON)
        assertEquals(0.5, desaturated.states[1].velocity.value(), EPSILON)
        assertEquals(-1.0, desaturated.states[2].velocity.value(), EPSILON)
        assertEquals(-1.0, desaturated.states[3].velocity.value(), EPSILON)
    }

    @Test
    fun testKinematicsComposition() {
        // Test that forward and inverse kinematics are inverses of each other
        val originalVelocity = PoseVelocity2d(Vector2d(3.0, 2.0), 1.0)

        // Convert to wheel velocities and back
        val wheelVels = kinematics.inverse(PoseVelocity2dDual.constant<Time>(originalVelocity, 1))
        val reconstructed = kinematics.forward<Time>(wheelVels)

        assertEquals(originalVelocity.linearVel.x, reconstructed.linearVel.x.value(), 1e-6)
        assertEquals(originalVelocity.linearVel.y, reconstructed.linearVel.y.value(), 1e-6)
        assertEquals(originalVelocity.angVel, reconstructed.angVel.value(), 1e-6)
    }

    @Test
    fun testModulePositions() {
        // Verify that module positions are correctly configured
        val centerRadius = sqrt(12.0 * 12.0 + 12.0 * 12.0)

        assertEquals(centerRadius, fl.norm(), EPSILON)
        assertEquals(centerRadius, fr.norm(), EPSILON)
        assertEquals(centerRadius, bl.norm(), EPSILON)
        assertEquals(centerRadius, br.norm(), EPSILON)
    }
}

