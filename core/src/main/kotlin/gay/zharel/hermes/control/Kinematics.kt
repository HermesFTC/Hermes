package gay.zharel.hermes.control

import gay.zharel.hermes.math.Arclength
import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.DualParameter
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.geometry.RobotState
import gay.zharel.hermes.geometry.Twist2dDual
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.geometry.Vector2dDual
import gay.zharel.hermes.math.atan2
import gay.zharel.hermes.paths.PosePath
import gay.zharel.hermes.profiles.VelConstraint
import kotlin.collections.forEach
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

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
}

/**
 * Represents the kinematics of a robot drive train, providing methods for
 * inverse kinematics and velocity constraints based on wheel speeds.
 */
interface RobotKinematics<in WI: WheelIncrements<*>, out WV: WheelVelocities<*>> {
    fun <Param : DualParameter> forward(w: WI): Twist2dDual<Param>

    /**
     * Performs inverse kinematics: computes wheel velocities required to achieve
     * the desired robot velocity.
     *
     * @param t Robot velocity in the robot's local frame.
     * @return Wheel velocities.
     */
    fun <Param : DualParameter> inverse(t: PoseVelocity2dDual<Param>): WheelVelocities<Param>
}

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 * @param[lateralMultiplier] factor that multiplies strafe velocity to compensate for slip; increase it to boost the
 * distance traveled in the strafe direction
 */
data class MecanumKinematics @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
    @JvmField
    val wheelbase: Double,
    @JvmField
    val lateralMultiplier: Double = 1.0
) : RobotKinematics<MecanumKinematics.MecanumWheelIncrements<*>, WheelVelocities<*>> {
    // The effective radius for rotation is the average of trackWidth and wheelbase
    private val effectiveRadius: Double = (trackWidth + wheelbase) / 2.0

    data class MecanumWheelIncrements<Param : DualParameter>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    ) : WheelIncrements<Param>

    override fun <Param : DualParameter> forward(w: MecanumWheelIncrements<*>): Twist2dDual<Param> {
        w as MecanumWheelIncrements<Param>

        return Twist2dDual(
            Vector2dDual(
                (w.leftFront + w.leftBack + w.rightBack + w.rightFront) * 0.25,
                (-w.leftFront + w.leftBack - w.rightBack + w.rightFront) * (0.25 / lateralMultiplier),
            ),
            (-w.leftFront - w.leftBack + w.rightBack + w.rightFront) * (0.25 / effectiveRadius),
        )
    }

    data class MecanumWheelVelocities<Param : DualParameter>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    ) : WheelVelocities<Param> {
        override fun all() = listOf(leftFront, leftBack, rightBack, rightFront)
    }

    override fun <Param : DualParameter> inverse(t: PoseVelocity2dDual<Param>) = MecanumWheelVelocities(
        t.linearVel.x - t.linearVel.y * lateralMultiplier - t.angVel * effectiveRadius,
        t.linearVel.x + t.linearVel.y * lateralMultiplier - t.angVel * effectiveRadius,
        t.linearVel.x - t.linearVel.y * lateralMultiplier + t.angVel * effectiveRadius,
        t.linearVel.x + t.linearVel.y * lateralMultiplier + t.angVel * effectiveRadius,
    )
}

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 */
data class TankKinematics(@JvmField val trackWidth: Double) :
    RobotKinematics<TankKinematics.TankWheelIncrements<*>, TankKinematics.TankWheelVelocities<*>> {

    data class TankWheelIncrements<Param : DualParameter>(
        @JvmField
        val left: DualNum<Param>,
        @JvmField
        val right: DualNum<Param>,
    ) : WheelIncrements<Param>

    override fun <Param : DualParameter> forward(w: TankWheelIncrements<*>): Twist2dDual<Param> {
        w as TankWheelIncrements<Param>

        return Twist2dDual(
            Vector2dDual(
                (w.left + w.right) * 0.5,
                DualNum.constant(0.0, w.left.size()),
            ),
            (-w.left + w.right) / trackWidth,
        )
    }

    data class TankWheelVelocities<Param : DualParameter>(
        @JvmField
        val left: DualNum<Param>,
        @JvmField
        val right: DualNum<Param>,
    ) : WheelVelocities<Param> {
        override fun all() = listOf(left, right)
    }

    override fun <Param : DualParameter> inverse(t: PoseVelocity2dDual<Param>): TankWheelVelocities<Param> {
        require(t.linearVel.y.values().all { abs(it) < 1e-6 }) {
            "Tank drive does not support lateral motion"
        }

        return TankWheelVelocities(
            t.linearVel.x - t.angVel * 0.5 * trackWidth,
            t.linearVel.x + t.angVel * 0.5 * trackWidth,
        )
    }
}

/**
 * @param wheelDelta change in wheel position
 * @param angle absolute angle
 */
data class SwerveModuleIncrements<Param : DualParameter>(
    @JvmField val wheelDelta: DualNum<Param>,
    @JvmField val angle: Double
) {
    constructor(wheelDelta: Double, angle: Double) :
            this(DualNum.constant(wheelDelta, 3), angle)
}

data class SwerveModuleState<Param : DualParameter>(
    @JvmField val velocity: DualNum<Param>,
    @JvmField val angle: DualNum<Param>
) {
    constructor(velocity: Double, angle: Double) :
            this(DualNum.constant(velocity, 3), DualNum.constant(angle, 3))

}

/**
 * @param[modules] list of swerve module configurations (position and orientation)
 */
data class SwerveKinematics(
    @JvmField
    val modules: List<Vector2d>
) : RobotKinematics<SwerveKinematics.SwerveWheelIncrements<*>, SwerveKinematics.SwerveWheelVelocities<*>> {

    data class SwerveWheelIncrements<Param : DualParameter>(
        @JvmField
        val deltas: List<SwerveModuleIncrements<Param>>
    ) : WheelIncrements<Param>

    override fun <Param : DualParameter> forward(w: SwerveWheelIncrements<*>): Twist2dDual<Param> {
        w as SwerveWheelIncrements<Param>

        // Initialize accumulators for the robot's overall motion
        var sumX = DualNum.constant<Param>(0.0, w.deltas.size)
        var sumY = DualNum.constant<Param>(0.0, w.deltas.size)
        var sumAngular = DualNum.constant<Param>(0.0, w.deltas.size)

        // Process each module's contribution to the robot's motion
       modules.zip(w.deltas) { module, delta ->
            // Convert wheel delta and steering angle to x and y components
            // Using the steering angle to determine the direction of motion
            val cosAngle = cos(delta.angle)
            val sinAngle = sin(delta.angle)

            // Calculate the module's contribution to linear motion
            val moduleX = delta.wheelDelta * cosAngle
            val moduleY = delta.wheelDelta * sinAngle

            // Add to the linear motion accumulators
            sumX = sumX.plus(moduleX)
            sumY = sumY.plus(moduleY)

            // Calculate the module's contribution to angular motion
            // This is the cross product of the module position and its velocity vector
            val angularContribution = (moduleY * module.x) - (moduleX * module.y)
            sumAngular = sumAngular.plus(angularContribution)
        }

        // Average the contributions from all modules
        val numModules = modules.size.toDouble()
        val avgX = sumX.div(numModules)
        val avgY = sumY.div(numModules)
        val avgAngular = sumAngular.div(numModules)

        // Return the resulting twist
        return Twist2dDual(
            Vector2dDual(avgX, avgY),
            avgAngular
        )
    }

    data class SwerveWheelVelocities<Param : DualParameter>(
        @JvmField
        val states: List<SwerveModuleState<Param>>
    ) : WheelVelocities<Param> {
        override fun all() = states.map { it.velocity }
    }

    override fun <Param : DualParameter> inverse(t: PoseVelocity2dDual<Param>): SwerveWheelVelocities<Param> {
        val wheelVels = mutableListOf<DualNum<Param>>()
        val steeringAngles = mutableListOf<DualNum<Param>>()

        // Calculate wheel velocities and steering angles for each module
        modules.forEach { module ->
            // Calculate the velocity at the module position due to robot rotation
            // This is the cross product of angular velocity and the module position vector
            val rotVelX = t.angVel * -module.y
            val rotVelY = t.angVel * module.x

            // Combine the robot's linear velocity with the rotational velocity at this module
            val totalVelX = t.linearVel.x + rotVelX
            val totalVelY = t.linearVel.y + rotVelY

            // Calculate the wheel velocity (magnitude of the velocity vector)
            val wheelVel = totalVelX.times(totalVelX).plus(totalVelY.times(totalVelY)).sqrt()
            wheelVels.add(wheelVel)

            // Calculate the steering angle using Rotation2d
            // We use atan2 to get the angle of the velocity vector
            val steeringAngle = atan2(totalVelY, totalVelX)
            steeringAngles.add(steeringAngle)
        }

        // Find maximum wheel velocity for normalization if needed
        val maxWheelVel = wheelVels.maxBy { it.value() }

        // Normalize wheel velocities if any exceeds 1.0
        if (maxWheelVel.value() > 1.0) {
            wheelVels.forEachIndexed { index, wheelVel ->
                wheelVels[index] = wheelVel.div(maxWheelVel)
            }
        }

        return SwerveWheelVelocities(
            wheelVels.zip(steeringAngles).map { SwerveModuleState(it.first, it.second) }
        )
    }
}

class WheelVelConstraint(
    @JvmField
    val kinematics: RobotKinematics<*, *>,
    @JvmField
    val maxWheelVel: Double
) : VelConstraint {
    override fun maxRobotVel(robotState: RobotState, path: PosePath, s: Double): Double {
        val txRobotWorld = robotState.pose.inverse()
        val robotVelWorld = robotState.vel
        val robotVelRobot = txRobotWorld * robotVelWorld

        return kinematics.inverse(PoseVelocity2dDual.constant<Arclength>(robotVelRobot, 1))
            .all()
            .minOf { abs(maxWheelVel / it.value()) }
    }
}
