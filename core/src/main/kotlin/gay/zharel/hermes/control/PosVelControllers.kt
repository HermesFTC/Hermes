/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.control

import gay.zharel.hermes.math.Arclength
import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.Pose2dDual
import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.math.Time
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.math.sinc
import gay.zharel.hermes.math.snz
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.withSign

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
}

/**
 * Abstract controller for computing the velocity and acceleration commands for a robot.
 */
interface RobotPosVelController {

    /**
     * Computes the velocity and acceleration command. The frame `Target` is the reference robot, and the frame `Actual`
     * is the measured, physical robot.
     *
     * @return velocity command in the actual frame
     */
    fun compute(
        targetPose: Pose2dDual<Time>,
        actualPose: Pose2d,
        actualVelActual: PoseVelocity2d,
    ): PoseVelocity2dDual<Time>
}

/**
 * Gains for the position-velocity controller.
 *
 * @property posGain proportional gain for position
 * @property velGain proportional gain for velocity
 */
data class PosVelGain @JvmOverloads constructor(
    @JvmField var posGain: Double,
    @JvmField var velGain: Double = 0.0,
)

/**
 * Proportional position-velocity controller for a holonomic robot.
 * This is essentially a P controller on the robot's position and a P controller on its velocity.
 *
 * @property axialGains gain for position and velocity in the robot's forward direction
 * @property lateralGains gain for position and velocity in the robot's strafe direction
 * @property headingGains gain for the robot's heading
 */
class HolonomicController(
    @JvmField
    val axialGains: PosVelGain,
    @JvmField
    val lateralGains: PosVelGain,
    @JvmField
    val headingGains: PosVelGain,
) : RobotPosVelController {

    /**
     * Proportional position-velocity controller for a holonomic robot.
     * This is essentially a P controller on the robot's position and a P controller on its velocity.
     *
     * @param axialPosGain gain for position in the robot's forward direction
     * @param lateralPosGain gain for position in the robot's strafe direction
     * @param headingGain gain for the robot's heading
     * @param axialVelGain gain for velocity in the robot's forward direction
     * @param lateralVelGain gain for velocity in the robot's strafe direction
     * @param headingVelGain gain for the robot's heading velocity
     */
    constructor(
        axialPosGain: Double, lateralPosGain: Double,
        headingGain: Double, axialVelGain: Double,
        lateralVelGain: Double, headingVelGain: Double
    ) : this(
        PosVelGain(axialPosGain, axialVelGain),
        PosVelGain(lateralPosGain, lateralVelGain),
        PosVelGain(headingGain, headingVelGain)
    )

    constructor(
        axialPosGain: Double,
        lateralPosGain: Double,
        headingGain: Double,
    ) : this(axialPosGain, lateralPosGain, headingGain, 0.0, 0.0, 0.0)

    /**
     * Computes the velocity and acceleration command. The frame `Target` is the reference robot, and the frame `Actual`
     * is the measured, physical robot.
     *
     * @return velocity command in the actual frame
     */
    override fun compute(
        targetPose: Pose2dDual<Time>,
        actualPose: Pose2d,
        actualVelActual: PoseVelocity2d,
    ): PoseVelocity2dDual<Time> {
        // TODO: Are these names useful for anyone else?
        val targetVelWorld = targetPose.velocity()
        val txTargetWorld = Pose2dDual.constant<Time>(targetPose.value().inverse(), 2)
        val targetVelTarget = txTargetWorld * targetVelWorld

        val velErrorActual = targetVelTarget.value() - actualVelActual

        val error = targetPose.value().minusExp(actualPose)
        return targetVelTarget +
                PoseVelocity2d(
                    Vector2d(
                        axialGains.posGain * error.position.x,
                        lateralGains.posGain * error.position.y,
                    ),
                    headingGains.posGain * error.heading.log(),
                ) +
                PoseVelocity2d(
                    Vector2d(
                        axialGains.velGain * velErrorActual.linearVel.x,
                        lateralGains.velGain * velErrorActual.linearVel.y,
                    ),
                    headingGains.velGain * velErrorActual.angVel,
                )
    }
}

/**
 * @usesMathJax
 *
 * Ramsete controller for tracking tank trajectories.
 *
 * The standard Ramsete control law from equation \((5.12)\) of [this paper](https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf) is
 * \[
 *   \begin{pmatrix}v\\ \omega\end{pmatrix} =
 *   \begin{pmatrix}
 *     v_d \cos (\theta_d - \theta) + 2 \zeta \sqrt{\omega_d^2 + b v_d^2} \Big\lbrack (x_d - x) \cos \theta + (y_d - y) \sin \theta \Big\rbrack\\
 *     \omega_d + b v_d \frac{\sin(\theta_d - \theta)}{\theta_d - \theta} \Big\lbrack (x_d - x) \cos \theta - (y_d - y) \sin \theta \Big\rbrack + 2 \zeta \sqrt{\omega_d^2 + b v_d^2} (\theta_d - \theta)
 *   \end{pmatrix}
 * \]
 * where \(\zeta \in (0, 1)\) and \(b \gt 0\). Since \(b\) has units, we substitute \(b = \frac{\bar{b}}{l^2}\) where
 * \(l\) is the track width.
 */
// defaults taken from https://github.com/wpilibsuite/allwpilib/blob/3fdb2f767d466e00d19e487fdb64d33c22ccc7d5/wpimath/src/main/native/cpp/controller/RamseteController.cpp#L31-L33
// with a track width of 1 meter
class RamseteController @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
    @JvmField
    val zeta: Double = 0.7,
    @JvmField
    val bBar: Double = 2.0,
) : RobotPosVelController {
    @JvmField
    val b = bBar / (trackWidth * trackWidth)

    /**
     * Computes the velocity and acceleration command. The frame `Target` is the reference robot, and the frame `Actual`
     * is the measured, physical robot.
     *
     * @return velocity command in the actual frame
     */
    override fun compute(
        targetPose: Pose2dDual<Time>,
        actualPose: Pose2d,
        actualVelActual: PoseVelocity2d
    ): PoseVelocity2dDual<Time> {
        val omegaRef = targetPose.heading.velocity()[0]
        val vRef = targetPose.velocity().value().linearVel.norm() * sign(actualVelActual.angVel)

        val k = 2.0 * zeta * sqrt(omegaRef * omegaRef + b * vRef * vRef)

        val error = targetPose.value().minusExp(actualPose)
        return PoseVelocity2dDual.constant(
            PoseVelocity2d(
                Vector2d(
                    vRef * error.heading.real + k * error.position.x,
                    0.0
                ),
                omegaRef + k * error.heading.log() + b * vRef * sinc(error.heading.log()) * error.position.y,
            ),
            2
        )
    }

    /**
     * Computes the velocity and acceleration command. The frame `Target` is the reference robot, and the frame `Actual`
     * is the measured, physical robot.
     *
     * @return velocity command in the actual frame
     */
    fun compute(
        s: DualNum<Time>,
        targetPose: Pose2dDual<Arclength>,
        actualPose: Pose2d,
    ): PoseVelocity2dDual<Time> {
        val targetHeading = targetPose.heading.value()
        val tangentHeading = targetPose.position.drop(1).value().angleCast()
        val dir = tangentHeading.real * targetHeading.real + tangentHeading.imag * targetHeading.imag
        val vRef = dir * s[1]

        return compute(targetPose.reparam(s), actualPose, PoseVelocity2d(Vector2d(vRef, 0.0), dir))
    }
}
