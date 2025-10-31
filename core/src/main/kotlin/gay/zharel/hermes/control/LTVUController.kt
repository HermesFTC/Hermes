/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 *
 * Code in this file is adapted from WPILib under a BSD-style license
 * that can be found in the External-Licenses file at the root of this repository.
 */

package gay.zharel.hermes.control

import gay.zharel.hermes.geometry.Acceleration2d
import gay.zharel.hermes.math.Matrix
import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.Pose2dDual
import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.math.Time
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.math.InterpolatingMap
import gay.zharel.hermes.math.lerpMatrix
import gay.zharel.hermes.math.makeBrysonMatrix
import gay.zharel.hermes.math.rangeCentered
import gay.zharel.hermes.math.times
import org.ejml.simple.SimpleMatrix
import kotlin.math.absoluteValue
import kotlin.time.TimeSource

/**
 * Linear Time-Varying Unicycle controller. 
 * This controller is similar to the LQR-based holonomic controller, 
 * but the unicycle model is linearized at a range of forward velocities. 
 * The feedback gains are scheduled according to the target robot velocity.
 *
 * This controller is intended as a replacement for the Ramsete controller,
 * as its tuning is more intuitive.
 *
 * This should be used by differential/tank drive robots only.
 *
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ltv.html">LTV Unicycle in WPILib</a>
 */
class LTVUController : RobotPosVelController {
    private val timeSource = TimeSource.Monotonic
    private var lastTimeStamp = timeSource.markNow()
    private val times = ArrayDeque<Double>()
    private val matrices = ArrayDeque<Matrix>()

    private val interpolator = InterpolatingMap<Matrix>(::lerpMatrix)

    /**
     * Constructs an LTV Unicycle controller for a differential/tank drive robot.
     *
     * This constructor assumes a kinematic acceleration model where the state error `x` is
     * `[xError, yError, headingError, vError, omegaError]` and the control input `u`
     * is the desired robot acceleration `[a, alpha]`.
     *
     * @param qX The maximum error in the x-position.
     * @param qY The maximum error in the y-position.
     * @param qHeading The maximum error in the heading.
     * @param qV The maximum error in the linear velocity.
     * @param qOmega The maximum error in the angular velocity.
     * @param rA The maximum linear acceleration.
     * @param rAlpha The maximum angular acceleration.
     * @param maxVel The maximum velocity for the controller gain lookup table.
     * @param dt The discrete time step (control loop period) in seconds.
     */
    @JvmOverloads constructor(
        qX: Double, qY: Double, qHeading: Double,
        qV: Double, qOmega: Double,
        rA: Double, rAlpha: Double,
        maxVel: Double,
        dt: Double = 0.0303
    ) {
        val A_cont = Matrix.zero(5, 5)
        val B_cont = Matrix.zero(5, 2).apply {
            this[0, 0] = 1.0
            this[1, 2] = 1.0
        }
        val Q = makeBrysonMatrix(qX, qY, qHeading, qV, qOmega)
        val R = makeBrysonMatrix(rA, rAlpha)

        rangeCentered(-maxVel, maxVel, 100).map {
            A_cont[1, 2] = if (it.absoluteValue < 1e-4) {
                1e-4
            } else {
                it
            }

            val (Ad, Bd) = discretizeSystem(A_cont, B_cont, dt)
            val (_, K) = computeLQRGain(Ad, Bd, Q, R)

            times.add(it)
            matrices.add(K)
        }


    }

    override fun compute(
        targetPose: Pose2dDual<Time>,
        actualPose: Pose2d,
        actualVelActual: PoseVelocity2d,
    ): PoseVelocity2dDual<Time> {
        val newStamp = timeSource.markNow()
        val targetVel = targetPose.velocity().value()

        val posError = targetPose.value() - actualPose
        val velError = (targetPose.velocity().value() - actualVelActual)

        val K = interpolator[targetVel.linearVel.x]

        val error = Matrix.column(
            posError.line.x, posError.line.y, posError.angle,
            velError.linearVel.x, velError.angVel
        )

        val u = K * error

        val acc = Acceleration2d(Vector2d(u[0, 0], 0.0), u[1, 0])
        val dt = (newStamp - lastTimeStamp).toDouble(kotlin.time.DurationUnit.SECONDS)
        val vel = acc.integrateToVel(dt, actualVelActual)

        lastTimeStamp = newStamp

        return vel.concat(acc)
    }
}