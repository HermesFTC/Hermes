package gay.zharel.hermes.ftc

import com.acmerobotics.dashboard.canvas.Canvas
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.hardware.*
import gay.zharel.hermes.actions.TrajectoryActionBuilder
import gay.zharel.hermes.control.HolonomicController
import gay.zharel.hermes.control.MecanumKinematics
import gay.zharel.hermes.control.MotorFeedforward
import gay.zharel.hermes.control.WheelVelConstraint
import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.geometry.Time
import gay.zharel.hermes.logs.DownsampledWriter
import gay.zharel.hermes.logs.FlightRecorder
import gay.zharel.hermes.logs.MecanumCommandMessage
import gay.zharel.hermes.logs.PoseMessage
import gay.zharel.hermes.profiles.*
import gay.zharel.hermes.trajectories.TrajectoryBuilder
import gay.zharel.hermes.trajectories.TrajectoryBuilderParams
import gay.zharel.hermes.trajectories.TurnConstraints
import java.util.*

class MecanumDrive(hardwareMap: HardwareMap, pose: Pose2d) : Drive {
    object PARAMS {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        var logoFacingDirection: RevHubOrientationOnRobot.LogoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP
        var usbFacingDirection: UsbFacingDirection = UsbFacingDirection.FORWARD

        // drive model parameters
        var inPerTick: Double = 1.0
        var lateralInPerTick: Double = inPerTick
        var trackWidthTicks: Double = 0.0

        // feedforward parameters (in tick units)
        var kS: Double = 0.0
        var kV: Double = 0.0
        var kA: Double = 0.0

        // path profile parameters (in inches)
        var maxWheelVel: Double = 50.0
        var minProfileAccel: Double = -30.0
        var maxProfileAccel: Double = 50.0

        // turn profile parameters (in radians)
        var maxAngVel: Double = Math.PI // shared with path
        var maxAngAccel: Double = Math.PI

        // path controller gains
        var axialGain: Double = 0.0
        var lateralGain: Double = 0.0
        var headingGain: Double = 0.0 // shared with turn

        var axialVelGain: Double = 0.0
        var lateralVelGain: Double = 0.0
        var headingVelGain: Double = 0.0 // shared with turn
    }

    val kinematics: MecanumKinematics = MecanumKinematics(
        PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick
    )


    override val defaultTurnConstraints: TurnConstraints = TurnConstraints(
        PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel
    )

    override val defaultVelConstraint: VelConstraint = MinVelConstraint(
        listOf(
            WheelVelConstraint(kinematics, PARAMS.maxWheelVel),
            AngularVelConstraint(PARAMS.maxAngVel)
        )
    )

    override val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)

    override val followerParams: FollowerParams = FollowerParams(
        ProfileParams(
            0.25, 0.1, 1e-2
        ),
        defaultVelConstraint, defaultAccelConstraint
    )

    val leftFront: DcMotorEx
    val leftBack: DcMotorEx
    val rightBack: DcMotorEx
    val rightFront: DcMotorEx

    val voltageSensor: VoltageSensor

    override val localizer: Localizer
    override val controller: HolonomicController
    private val poseHistory: LinkedList<Pose2d> = LinkedList<Pose2d>()

    private val estimatedPoseWriter: DownsampledWriter = DownsampledWriter("ESTIMATED_POSE", 50000000)
    private val targetPoseWriter: DownsampledWriter = DownsampledWriter("TARGET_POSE", 50000000)
    private val driveCommandWriter: DownsampledWriter = DownsampledWriter("DRIVE_COMMAND", 50000000)
    private val mecanumCommandWriter: DownsampledWriter = DownsampledWriter("MECANUM_COMMAND", 50000000)

    init {
        for (module in hardwareMap.getAll<LynxModule>(LynxModule::class.java)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx::class.java, "leftFront")
        leftBack = hardwareMap.get(DcMotorEx::class.java, "leftBack")
        rightBack = hardwareMap.get(DcMotorEx::class.java, "rightBack")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "rightFront")

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        val imu = (hardwareMap["imu"] as IMU).also {
            it.initialize(IMU.Parameters(
                RevHubOrientationOnRobot(
                    PARAMS.logoFacingDirection,
                    PARAMS.usbFacingDirection
                )
            ))
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        localizer = MecanumDriveLocalizer(hardwareMap, PARAMS.inPerTick, imu, kinematics)
            .fromMotors(leftFront, leftBack, rightFront, rightBack)
            .withInitialPose(pose)

        controller = HolonomicController(
            PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
            PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
        )

        FlightRecorder.write("MECANUM_PARAMS", PARAMS)
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

        val feedforward = MotorFeedforward(
            PARAMS.kS,
            PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
        )
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

    override fun trajectoryBuilder(beginPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(
            TrajectoryBuilderParams(
                1e-6,
                followerParams.profileParams
            ),
            beginPose, 0.0,
            defaultVelConstraint,
            defaultAccelConstraint
        )
    }

    override fun trajectoryBuilder(): TrajectoryBuilder {
        return trajectoryBuilder(localizer.pose)
    }
}