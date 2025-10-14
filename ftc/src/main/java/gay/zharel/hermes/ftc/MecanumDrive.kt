package gay.zharel.hermes.ftc

import com.acmerobotics.dashboard.canvas.Canvas
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import gay.zharel.hermes.actions.TrajectoryActionBuilder
import gay.zharel.hermes.control.HolonomicController
import gay.zharel.hermes.control.MecanumKinematics
import gay.zharel.hermes.control.MotorFeedforward
import gay.zharel.hermes.control.PosVelGain
import gay.zharel.hermes.control.WheelVelConstraint
import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.math.Time
import gay.zharel.hermes.logs.DownsampledWriter
import gay.zharel.hermes.logs.FlightRecorder
import gay.zharel.hermes.logs.MecanumCommandMessage
import gay.zharel.hermes.logs.PoseMessage
import gay.zharel.hermes.profiles.*
import gay.zharel.hermes.trajectories.TrajectoryBuilder
import gay.zharel.hermes.trajectories.TrajectoryBuilderParams
import gay.zharel.hermes.trajectories.TurnConstraints
import gay.zharel.hermes.tuning.HermesConfig
import gay.zharel.hermes.tuning.MecanumParameters
import gay.zharel.hermes.tuning.TuningLocalizerFactory

class MecanumDrive @JvmOverloads constructor(
    val params: Parameters,
    hardwareMap: HardwareMap, 
    pose: Pose2d = Pose2d.zero
) : Drive {
    internal val driveConfig get() = HermesConfig.config.drive as MecanumParameters
    internal val feedforwardConfig get() = HermesConfig.config.feedforward
    
    val inPerTick: Double get() = HermesConfig.config.localizer.inPerTick

    data class Parameters(
        @JvmField var axialGains: PosVelGain = PosVelGain(0.1),
        @JvmField var lateralGains: PosVelGain = PosVelGain(0.1),
        @JvmField var headingGains: PosVelGain = PosVelGain(0.1),
        @JvmField var maxWheelVel: Double = 50.0,
        @JvmField var minTransAccel: Double = -30.0,
        @JvmField var maxTransAccel: Double = 50.0,
        @JvmField var maxAngVel: Double = Math.PI,
        @JvmField var maxAngAccel: Double = Math.PI,
    )

    val feedforward = MotorFeedforward(
        feedforwardConfig.translational.kStatic,
        feedforwardConfig.translational.kV / inPerTick, 
        feedforwardConfig.translational.kA / inPerTick
    )

    val kinematics: MecanumKinematics = MecanumKinematics(
        inPerTick * driveConfig.trackWidth,
        inPerTick * driveConfig.wheelBase,
    )


    override val defaultTurnConstraints: TurnConstraints = TurnConstraints(
        params.maxAngVel, -params.maxAngAccel, params.maxAngAccel
    )

    override val defaultVelConstraint: VelConstraint = MinVelConstraint(
        listOf(
            WheelVelConstraint(kinematics, params.maxWheelVel),
            AngularVelConstraint(params.maxAngVel)
        )
    )

    override val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(params.minTransAccel, params.maxTransAccel)

    override val followerParams: FollowerParams = FollowerParams(
        ProfileParams(
            0.25, 0.1, 1e-2
        ),
        defaultVelConstraint, defaultAccelConstraint
    )

    val leftFront: DcMotorEx = driveConfig.leftFront.toDcMotorEx(hardwareMap)
    val leftBack: DcMotorEx = driveConfig.leftBack.toDcMotorEx(hardwareMap)
    val rightBack: DcMotorEx = driveConfig.rightBack.toDcMotorEx(hardwareMap)
    val rightFront: DcMotorEx = driveConfig.rightFront.toDcMotorEx(hardwareMap)

    val voltageSensor: VoltageSensor

    // if you would like to use a custom localizer, overwrite this line with your own.
    override val localizer: Localizer = TuningLocalizerFactory.make(hardwareMap)

    override val controller = HolonomicController(params.axialGains, params.lateralGains, params.headingGains,)
    private val poseHistory: ArrayDeque<Pose2d> = ArrayDeque()

    private val estimatedPoseWriter: DownsampledWriter = DownsampledWriter("ESTIMATED_POSE", 50000000)
    private val targetPoseWriter: DownsampledWriter = DownsampledWriter("TARGET_POSE", 50000000)
    private val driveCommandWriter: DownsampledWriter = DownsampledWriter("DRIVE_COMMAND", 50000000)
    private val mecanumCommandWriter: DownsampledWriter = DownsampledWriter("MECANUM_COMMAND", 50000000)

    init {
        for (module in hardwareMap.getAll<LynxModule>(LynxModule::class.java)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        FlightRecorder.write("MECANUM_PARAMS", params)
    }

    override fun setDrivePowers(powers: PoseVelocity2dDual<Time>) {
        val wheelVels: MecanumKinematics.MecanumWheelVelocities<Time> = kinematics.inverse(powers)

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = maxPowerMag.coerceAtLeast(power.value())
        }

        leftFront.power = wheelVels.leftFront[0] / maxPowerMag
        leftBack.power = wheelVels.leftBack[0] / maxPowerMag
        rightBack.power = wheelVels.rightBack[0] / maxPowerMag
        rightFront.power = wheelVels.rightFront[0] / maxPowerMag
    }

    override fun setDrivePowersWithFF(powers: PoseVelocity2dDual<Time>) {
        val wheelVels: MecanumKinematics.MecanumWheelVelocities<Time> = kinematics.inverse(powers)
        val voltage = voltageSensor.voltage

        val leftFrontPower: Double = feedforward.compute(wheelVels.leftFront) / voltage
        val leftBackPower: Double = feedforward.compute(wheelVels.leftBack) / voltage
        val rightBackPower: Double = feedforward.compute(wheelVels.rightBack) / voltage
        val rightFrontPower: Double = feedforward.compute(wheelVels.rightFront) / voltage
        mecanumCommandWriter.write(
            MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            )
        )

        leftFront.power = leftFrontPower
        leftBack.power = leftBackPower
        rightBack.power = rightBackPower
        rightFront.power = rightFrontPower
    }

    override fun updatePoseEstimate(): PoseVelocity2d {
        val vel: PoseVelocity2d = localizer.update()
        poseHistory.add(localizer.pose)

        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(localizer.pose))


        return vel
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        var i = 0
        for (t in poseHistory) {
            xPoints[i] = t.position.x
            yPoints[i] = t.position.y

            i++
        }

        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            { turn -> TurnAction(turn, this) },
            { traj ->
                FollowTrajectoryAction(
                    TimeFollower(traj, this),
                    this
                )
            },
            TrajectoryBuilderParams(
                1e-6,
                followerParams.profileParams
            ),
            beginPose, 0.0,
            defaultTurnConstraints,
            defaultVelConstraint, defaultAccelConstraint
        )
    }

    fun actionBuilder(): TrajectoryActionBuilder {
        return actionBuilder(localizer.pose)
    }

    override fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(
            TrajectoryBuilderParams(
                1e-6,
                followerParams.profileParams
            ),
            startPose, 0.0,
            defaultVelConstraint,
            defaultAccelConstraint
        )
    }

    override fun trajectoryBuilder(): TrajectoryBuilder {
        return trajectoryBuilder(localizer.pose)
    }
}

/**
 * Builder class for constructing a [MecanumDrive] instance with custom parameters.
 *
 * This builder provides a fluent API for configuring drive parameters before constructing
 * the final MecanumDrive object.
 *
 * @see MecanumDrive
 */
class MecanumDriveBuilder {
    private var parameters = MecanumDrive.Parameters()

    /**
     * Sets the axial (forward/backward) PID gains.
     *
     * @param posGain The proportional gain for axial position control
     * @param velGain The derivative gain for axial velocity control
     * @return This builder instance for method chaining
     */
    fun withAxialGains(posGain: Double, velGain: Double) = apply {
        parameters.axialGains = PosVelGain(posGain, velGain)
    }

    /**
     * Sets the lateral (left/right strafe) PID gains.
     *
     * @param posGain The proportional gain for lateral position control
     * @param velGain The derivative gain for lateral velocity control
     * @return This builder instance for method chaining
     */
    fun withLateralGains(posGain: Double, velGain: Double) = apply {
        parameters.lateralGains = PosVelGain(posGain, velGain)
    }

    /**
     * Sets the heading (rotational) PID gains.
     *
     * @param posGain The proportional gain for heading position control
     * @param velGain The derivative gain for heading velocity control
     * @return This builder instance for method chaining
     */
    fun withHeadingGains(posGain: Double, velGain: Double) = apply {
        parameters.headingGains = PosVelGain(posGain, velGain)
    }

    /**
     * Sets the maximum wheel velocity.
     *
     * @param maxWheelVel The maximum velocity for any wheel in inches per second
     * @return This builder instance for method chaining
     */
    fun withMaxWheelVel(maxWheelVel: Double) = apply {
        parameters.maxWheelVel = maxWheelVel
    }

    /**
     * Sets the translational acceleration limits.
     *
     * @param minTransAccel The minimum (most negative) translational acceleration in inches per second squared
     * @param maxTransAccel The maximum translational acceleration in inches per second squared
     * @return This builder instance for method chaining
     */
    fun withTransAccelLimits(minTransAccel: Double, maxTransAccel: Double) = apply {
        parameters.minTransAccel = minTransAccel
        parameters.maxTransAccel = maxTransAccel
    }

    /**
     * Sets the maximum angular velocity.
     *
     * @param maxAngVel The maximum angular velocity in radians per second
     * @return This builder instance for method chaining
     */
    fun withMaxAngVel(maxAngVel: Double) = apply {
        parameters.maxAngVel = maxAngVel
    }

    /**
     * Sets the maximum angular acceleration.
     *
     * @param maxAngAccel The maximum angular acceleration in radians per second squared
     * @return This builder instance for method chaining
     */
    fun withMaxAngAccel(maxAngAccel: Double) = apply {
        parameters.maxAngAccel = maxAngAccel
    }

    /**
     * Builds and returns a new [MecanumDrive] instance with the configured parameters.
     *
     * @param hardwareMap The FTC hardware map containing motor and sensor configurations
     * @param pose The initial pose of the robot (defaults to zero)
     * @return A new MecanumDrive instance
     */
    fun build(hardwareMap: HardwareMap, pose: Pose2d = Pose2d.zero): MecanumDrive {
        return MecanumDrive(parameters, hardwareMap, pose)
    }
}