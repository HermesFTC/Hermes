/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 *
 * Some code in this file is adapted from WPILib under a BSD-style license
 * that can be found in the External-Licenses file at the root of this repository.
 */

package gay.zharel.hermes.kinematics

import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.geometry.Twist2dDual
import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.math.Time
import kotlin.math.sign
import kotlin.math.withSign

interface WheelIncrements<Param : DualParameter>

/**
 * Represents the velocities of the individual drive wheels.
 */
interface WheelVelocities<Param : DualParameter> {
    /**
     * Returns a list of all wheel velocities.
     * The order should be consistent for a given implementation.
     */
    fun all(): List<DualNum<Param>>

    /**
     * Desaturates wheel velocities to ensure none exceed the maximum physical speed.
     *
     * @param maxPhysicalSpeed The maximum allowable wheel speed
     * @return A new [WheelVelocities] instance with scaled velocities if necessary
     */
    fun desaturate(maxPhysicalSpeed: Double): WheelVelocities<Param>
}

/**
 * Represents the kinematics of a robot drive train, providing methods for
 * inverse kinematics and velocity constraints based on wheel speeds.
 */
interface RobotKinematics<in WI: WheelIncrements<*>, in WV: WheelVelocities<*>> {
    /**
     * Performs forward kinematics: computes the twist (pose delta) that occurred
     * based on the given wheel increments.
     *
     * @param increments The change in wheel positions
     * @return The resulting twist (change in pose)
     */
    fun <Param : DualParameter> forward(increments: WI): Twist2dDual<Param>

    /**
     * Performs forward kinematics: computes the chassis velocity required
     * to achieve the given wheel velocities.
     *
     * @param velocities The wheel velocities
     * @return The resulting robot velocity
     */
    fun <Param : DualParameter> forward(velocities: WV): PoseVelocity2dDual<Param>

    /**
     * Performs inverse kinematics: computes wheel velocities required to achieve
     * the desired robot velocity.
     *
     * @param velocity Robot velocity in the robot's local frame.
     * @return Wheel velocities.
     */
    fun <Param : DualParameter> inverse(velocity: PoseVelocity2dDual<Param>): WheelVelocities<Param>
}

/**
 * @usesMathJax
 *
 * Kinematic motor feedforward
 *
 * @property[kS] kStatic, \(k_s\)
 * @property[kV] kVelocity, \(k_v\)
 * @property[kA] kStatic, \(k_a\)
 */
data class MotorFeedforward(
    @JvmField
    val kS: Double,
    @JvmField
    val kV: Double,
    @JvmField
    val kA: Double,
) {
    /**
     * @usesMathJax
     *
     * Computes the (normalized) voltage \(k_s \cdot \operatorname{sign}(k_v \cdot v + k_a \cdot a) + k_v \cdot v + k_a \cdot a\).
     *
     * @param[vel] \(v\)
     * @param[accel] \(a\)
     */
    fun compute(vel: Double, accel: Double) = kS.withSign(vel) + kV * vel + kA * accel

    fun compute(vel: DualNum<Time>) = compute(vel[0], vel[1])

    /**
     * Computes the maximum achievable acceleration given the maximum voltage and velocity.
     */
    fun maxAchievableAcceleration(maxVoltage: Double, velocity: Double) =
        (maxVoltage - kS * sign(velocity) - velocity * kV) / kA

    /**
     * Computes the maximum achievable acceleration given the maximum voltage and velocity.
     */
    fun minAchievableAcceleration(maxVoltage: Double, velocity: Double) =
        maxAchievableAcceleration(-maxVoltage, velocity)
}