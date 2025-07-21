package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.MecanumKinematics
import com.acmerobotics.roadrunner.control.MecanumKinematics.MecanumWheelIncrements
import com.acmerobotics.roadrunner.control.SwerveKinematics
import com.acmerobotics.roadrunner.control.SwerveModuleIncrements
import com.acmerobotics.roadrunner.control.TankKinematics
import com.acmerobotics.roadrunner.control.TankKinematics.TankWheelIncrements
import com.acmerobotics.roadrunner.geometry.*
import com.acmerobotics.roadrunner.geometry.Rotation2d.Companion.fromDouble
import com.acmerobotics.roadrunner.hardware.*
import com.acmerobotics.roadrunner.logs.*
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import java.util.*
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

interface Localizer {
    var pose: Pose2d
    val vel: PoseVelocity2d
    val poseHistory: MutableList<Pose2d>

    fun update(): PoseVelocity2d
}

internal fun HardwareMap.overflowEncoder(name: String) =
    OverflowEncoder(RawEncoder(get(DcMotorEx::class.java, name)))

class TwoDeadWheelLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    val imu: IMU,
    @JvmField var inPerTick: Double,
    val parName: String,
    val perpName: String,
    @JvmField var parYTicks: Double = 0.0, // y position of the parallel encoder (in tick units)
    @JvmField var perpXTicks: Double = 0.0,
    val parDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val perpDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD, // x position of the perpendicular encoder (in tick units)
    val initialPose: Pose2d = Pose2d.zero,
) : Localizer {
    val par: Encoder = hardwareMap.overflowEncoder(parName)
    val perp: Encoder = hardwareMap.overflowEncoder(perpName)

    private var lastParPos = 0
    private var lastPerpPos = 0
    private var lastHeading: Rotation2d? = null

    private var lastRawHeadingVel = 0.0
    private var headingVelOffset = 0.0
    private var initialized = false
    override var pose: Pose2d
    override var vel: PoseVelocity2d = PoseVelocity2d.zero
        private set
    override val poseHistory = mutableListOf<Pose2d>()

    init {
        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", parYTicks to perpXTicks)

        pose = initialPose

        par.direction = parDirection
        perp.direction = perpDirection
    }

    override fun update(): PoseVelocity2d {
        val parPosVel: PositionVelocityPair = par.getPositionAndVelocity()
        val perpPosVel: PositionVelocityPair = perp.getPositionAndVelocity()

        val angles = imu.robotYawPitchRollAngles
        // Use degrees here to work around https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070
        val angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS)

        FlightRecorder.write(
            "TWO_DEAD_WHEEL_INPUTS",
            TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity),
        )

        val heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        val rawHeadingVel = angularVelocity.zRotationRate.toDouble()
        if (abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= sign(rawHeadingVel) * 2 * Math.PI
        }
        lastRawHeadingVel = rawHeadingVel
        val headingVel = headingVelOffset + rawHeadingVel

        if (!initialized) {
            initialized = true

            lastParPos = parPosVel.position
            lastPerpPos = perpPosVel.position
            lastHeading = heading

            return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        }

        val parPosDelta: Int = parPosVel.position - lastParPos
        val perpPosDelta: Int = perpPosVel.position - lastPerpPos
        val headingDelta = heading.minus(lastHeading!!)

        val twist = Twist2dDual(
            Vector2dDual(
                DualNum<Time?>(
                    doubleArrayOf(
                        parPosDelta - parYTicks * headingDelta,
                        parPosVel.velocity - parYTicks * headingVel,
                    ),
                ).times(inPerTick),
                DualNum<Time?>(
                    doubleArrayOf(
                        perpPosDelta - perpXTicks * headingDelta,
                        perpPosVel.velocity - perpXTicks * headingVel,
                    ),
                ).times(inPerTick),
            ),
            DualNum(
                doubleArrayOf(
                    headingDelta,
                    headingVel,
                ),
            ),
        )

        lastParPos = parPosVel.position
        lastPerpPos = perpPosVel.position
        lastHeading = heading

        pose = pose.plus(twist.value())

        poseHistory.add(0, pose)
        if (poseHistory.size > 100) {
            poseHistory.removeAt(poseHistory.lastIndex)
        }

        vel = twist.velocity().value()
        return vel
    }
    
    fun withNames(parName: String, perpName: String) = TwoDeadWheelLocalizer(
        hardwareMap,
        imu,
        inPerTick,
        parName,
        perpName,
        parYTicks,
        perpXTicks,
        parDirection,
        perpDirection,
        initialPose,
    )

    fun withLocations(parYTicks: Double, perpXTicks: Double) = TwoDeadWheelLocalizer(
        hardwareMap,
        imu,
        inPerTick,
        parName,
        perpName,
        parYTicks,
        perpXTicks,
        parDirection,
        perpDirection,
        initialPose,
    )

    fun withDirections(parDirection: DcMotorSimple.Direction, perpDirection: DcMotorSimple.Direction) =
        TwoDeadWheelLocalizer(
            hardwareMap,
            imu,
            inPerTick,
            parName,
            perpName,
            parYTicks,
            perpXTicks,
            parDirection,
            perpDirection,
            initialPose,
        )

    fun withInitialPose(initialPose: Pose2d) = TwoDeadWheelLocalizer(
        hardwareMap,
        imu,
        inPerTick,
        parName,
        perpName,
        parYTicks,
        perpXTicks,
        parDirection,
        perpDirection,
        initialPose,
    )
}

@Config
class ThreeDeadWheelLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    @JvmField var inPerTick: Double, 
    val par0Name: String = "par0",
    val par1Name: String = "par1", 
    val perpName: String = "perp",
    @JvmField var par0YTicks: Double = 0.0, // y position of the first parallel encoder (in tick units)
    @JvmField var par1YTicks: Double = 1.0, // y position of the second parallel encoder (in tick units)
    @JvmField var perpXTicks: Double = 0.0, // x position of the perpendicular encoder (in tick units)
    val par0Direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val par1Direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val PerpDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val initialPose: Pose2d = Pose2d.zero,
    ) : Localizer {
    val par0: Encoder = hardwareMap.overflowEncoder(par0Name)
    val par1: Encoder = hardwareMap.overflowEncoder(par1Name)
    val perp: Encoder = hardwareMap.overflowEncoder(perpName)

    private var lastPar0Pos = 0
    private var lastPar1Pos = 0
    private var lastPerpPos = 0
    private var initialized = false
    override var pose: Pose2d
    override var vel: PoseVelocity2d = PoseVelocity2d.zero
        private set
    override val poseHistory = mutableListOf<Pose2d>()

    init {
        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", arrayOf(par0YTicks, par1YTicks, perpXTicks))

        pose = initialPose
    }

    override fun update(): PoseVelocity2d {
        val par0PosVel = par0.getPositionAndVelocity()
        val par1PosVel = par1.getPositionAndVelocity()
        val perpPosVel = perp.getPositionAndVelocity()

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel))

        if (!initialized) {
            initialized = true

            lastPar0Pos = par0PosVel.position
            lastPar1Pos = par1PosVel.position
            lastPerpPos = perpPosVel.position

            return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        }

        val par0PosDelta = par0PosVel.position - lastPar0Pos
        val par1PosDelta = par1PosVel.position - lastPar1Pos
        val perpPosDelta = perpPosVel.position - lastPerpPos

        val twist = Twist2dDual(
            Vector2dDual(
                DualNum<Time>(
                    doubleArrayOf(
                        (par0YTicks * par1PosDelta - par1YTicks * par0PosDelta) / (par0YTicks - par1YTicks),
                        (par0YTicks * par1PosVel.velocity - par1YTicks * par0PosVel.velocity) / (par0YTicks - par1YTicks),
                    ),
                ).times(inPerTick),
                DualNum<Time>(
                    doubleArrayOf(
                        (perpXTicks / (par0YTicks - par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                        (perpXTicks / (par0YTicks - par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                    ),
                ).times(inPerTick),
            ),
            DualNum(
                doubleArrayOf(
                    (par0PosDelta - par1PosDelta) / (par0YTicks - par1YTicks),
                    (par0PosVel.velocity - par1PosVel.velocity) / (par0YTicks - par1YTicks),
                ),
            ),
        )

        lastPar0Pos = par0PosVel.position
        lastPar1Pos = par1PosVel.position
        lastPerpPos = perpPosVel.position

        pose = pose.plus(twist.value())

        poseHistory.add(0, pose)
        if (poseHistory.size > 100) {
            poseHistory.removeAt(poseHistory.lastIndex)
        }

        vel = twist.velocity().value()
        return vel
    }

    fun withNames(par0Name: String, par1Name: String, perpName: String) = ThreeDeadWheelLocalizer(
        hardwareMap,
        inPerTick,
        par0Name,
        par1Name,
        perpName,
        par0YTicks,
        par1YTicks,
        perpXTicks,
        par0Direction,
        par1Direction,
        PerpDirection,
    )

    fun withLocations(par0YTicks: Double, par1YTicks: Double, perpXTicks: Double) =
        ThreeDeadWheelLocalizer(
            hardwareMap,
            inPerTick,
            par0Name,
            par1Name,
            perpName,
            par0YTicks,
            par1YTicks,
            perpXTicks,
            par0Direction,
            par1Direction,
            PerpDirection,
        )

    fun withDirections(par0Direction: DcMotorSimple.Direction, par1Direction: DcMotorSimple.Direction, perpDirection: DcMotorSimple.Direction) =
        ThreeDeadWheelLocalizer(
            hardwareMap,
            inPerTick,
            par0Name,
            par1Name,
            perpName,
            par0YTicks,
            par1YTicks,
            perpXTicks,
            par0Direction,
            par1Direction,
            perpDirection,
        )

    fun withInitialPose(initialPose: Pose2d) = ThreeDeadWheelLocalizer(
        hardwareMap,
        inPerTick,
        par0Name,
        par1Name,
        perpName,
        par0YTicks,
        par1YTicks,
        perpXTicks,
        par0Direction,
        par1Direction,
        PerpDirection,
        initialPose,
    )
}

@Config
class PinpointLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    @JvmField var inPerTick: Double,
    val name: String = "pinpoint",
    @JvmField var parYTicks: Double = 0.0,
    @JvmField var perpXTicks: Double = 0.0,
    val parDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val perpDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val initialPose: Pose2d = Pose2d.zero,
) : Localizer {
    val driver: GoBildaPinpointDriver =
        hardwareMap.get(GoBildaPinpointDriver::class.java, name)

    private var txWorldPinpoint: Pose2d
    private var txPinpointRobot = Pose2d(0.0, 0.0, 0.0)
    private var currentPose: Pose2d = Pose2d.zero

    override val poseHistory = mutableListOf<Pose2d>()

    init {
        driver.setEncoderResolution(inPerTick, DistanceUnit.INCH)
        driver.setOffsets(inPerTick * parYTicks, inPerTick * perpXTicks, DistanceUnit.INCH)

        driver.setEncoderDirections(parDirection.pinpointDirection, perpDirection.pinpointDirection)

        driver.resetPosAndIMU()

        txWorldPinpoint = initialPose
    }

    override var pose: Pose2d
        get() = currentPose
        set(pose) {
            txWorldPinpoint = pose.times(txPinpointRobot.inverse())
        }
    override var vel: PoseVelocity2d = PoseVelocity2d.zero
        private set

    override fun update(): PoseVelocity2d {
        driver.update()
        if (Objects.requireNonNull(driver.deviceStatus) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = Pose2d(
                driver.getPosX(DistanceUnit.INCH),
                driver.getPosY(DistanceUnit.INCH),
                driver.getHeading(UnnormalizedAngleUnit.RADIANS),
            )
            val worldVelocity = Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH))
            val robotVelocity = fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity)

            currentPose = txWorldPinpoint.times(txPinpointRobot)
            poseHistory.add(0, currentPose)

            if (poseHistory.size > 100) {
                poseHistory.removeAt(poseHistory.lastIndex)
            }

            vel = PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS))
            return vel
        }
        return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
    }

    fun withName(name: String) = PinpointLocalizer(
        hardwareMap,
        inPerTick,
        name,
        parYTicks,
        perpXTicks,
        parDirection,
        perpDirection,
        initialPose,
    )

    fun withOffsets(parYTicks: Double, perpXTicks: Double) = PinpointLocalizer(
        hardwareMap,
        inPerTick,
        name,
        parYTicks,
        perpXTicks,
        parDirection,
        perpDirection,
        initialPose,
    )

    fun withDirections(parDirection: DcMotorSimple.Direction, perpDirection: DcMotorSimple.Direction) =
        PinpointLocalizer(
            hardwareMap,
            inPerTick,
            name,
            parYTicks,
            perpXTicks,
            parDirection,
            perpDirection,
            initialPose,
        )

    fun withInitialPose(initialPose: Pose2d) = PinpointLocalizer(
        hardwareMap,
        inPerTick,
        name,
        parYTicks,
        perpXTicks,
        parDirection,
        perpDirection,
        initialPose,
    )
}

@Config
class OTOSLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    val otosName: String = "sensor_otos",
    @JvmField var linearScalar: Double = 1.0,
    @JvmField var angularScalar: Double = 1.0,
    @JvmField var offset: Pose2d = Pose2d.zero,
    val initialPose: Pose2d = Pose2d.zero,
) : Localizer {
    val otos: SparkFunOTOS = hardwareMap.get(SparkFunOTOS::class.java, "sensor_otos")
    override val poseHistory = mutableListOf<Pose2d>()

    init {
        otos.position = initialPose.toOTOSPose()
        otos.setLinearUnit(DistanceUnit.INCH)
        otos.setAngularUnit(AngleUnit.RADIANS)

        otos.calibrateImu()
        otos.setLinearScalar(linearScalar)
        otos.setAngularScalar(angularScalar)
        otos.offset = offset.toOTOSPose()
    }

    override var pose: Pose2d = initialPose
        set(value) {
            otos.setPosition(value.toOTOSPose())
        }
    override var vel: PoseVelocity2d = PoseVelocity2d.zero
        private set

    override fun update(): PoseVelocity2d {
        val otosPose = SparkFunOTOS.Pose2D()
        val otosVel = SparkFunOTOS.Pose2D()
        val otosAcc = SparkFunOTOS.Pose2D()
        otos.getPosVelAcc(otosPose, otosVel, otosAcc)

        pose = otosPose.toRRPose()
        val fieldVel = Vector2d(otosVel.x, otosVel.y)
        val robotVel = Rotation2d.exp(otosPose.h).inverse().times(fieldVel)

        poseHistory.add(0, pose)

        if (poseHistory.size > 100) {
            poseHistory.removeAt(poseHistory.lastIndex)
        }

        vel =  PoseVelocity2d(robotVel, otosVel.h)
        return vel
    }

    fun withName(otosName: String) = OTOSLocalizer(
        hardwareMap,
        otosName,
        linearScalar,
        angularScalar,
        offset,
        initialPose,
    )

    fun withScalars(linearScalar: Double, angularScalar: Double) = OTOSLocalizer(
        hardwareMap,
        otosName,
        linearScalar,
        angularScalar,
        offset,
        initialPose,
    )

    fun withOffset(offset: Pose2d) = OTOSLocalizer(
        hardwareMap,
        otosName,
        linearScalar,
        angularScalar,
        offset,
        initialPose,
    )

    fun withOffset(x: Double, y: Double, h: Double) =
        withOffset(Pose2d(x, y, h))

    fun withInitialPose(initialPose: Pose2d) = OTOSLocalizer(
        hardwareMap,
        otosName,
        linearScalar,
        angularScalar,
        offset,
        initialPose,
    )
}

class MecanumDriveLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    @JvmField var inPerTick: Double,
    val imu: IMU,
    val kinematics: MecanumKinematics,
    val lfName: String = "leftFront",
    val lbName: String = "leftBack",
    val rfName: String = "rightFront",
    val rbName: String = "rightBack",
    val lfDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val lbDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val rfDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val rbDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val initialPose: Pose2d = Pose2d.zero,
) : Localizer {
    val leftFront: Encoder = hardwareMap.overflowEncoder(lfName)
    val leftBack: Encoder = hardwareMap.overflowEncoder(lbName)
    val rightFront: Encoder = hardwareMap.overflowEncoder(rfName)
    val rightBack: Encoder = hardwareMap.overflowEncoder(rbName)
    private var lastLeftFrontPos = 0
    private var lastLeftBackPos: Int = 0
    private var lastRightBackPos: Int = 0
    private var lastRightFrontPos: Int = 0
    private var lastHeading = Rotation2d.zero

    override var pose: Pose2d = initialPose
    override var vel: PoseVelocity2d = PoseVelocity2d.zero
        private set

    override val poseHistory = mutableListOf<Pose2d>()

    private var initialized = false

    init {
        leftFront.direction = lfDirection
        leftBack.direction = lbDirection
        rightFront.direction = rfDirection
        rightBack.direction = rbDirection
    }

    override fun update(): PoseVelocity2d {
        val leftFrontPosVel = leftFront.getPositionAndVelocity()
        val leftBackPosVel = leftBack.getPositionAndVelocity()
        val rightBackPosVel = rightBack.getPositionAndVelocity()
        val rightFrontPosVel = rightFront.getPositionAndVelocity()

        val angles = imu.robotYawPitchRollAngles

        FlightRecorder.write(
            "MECANUM_LOCALIZER_INPUTS",
            MecanumLocalizerInputsMessage(
                leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles,
            ),
        )

        val heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

        if (!initialized) {
            initialized = true

            lastLeftFrontPos = leftFrontPosVel.position
            lastLeftBackPos = leftBackPosVel.position
            lastRightBackPos = rightBackPosVel.position
            lastRightFrontPos = rightFrontPosVel.position

            lastHeading = heading

            return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        }

        val headingDelta = heading.minus(lastHeading)
        val twist = kinematics.forward<Time>(
            MecanumWheelIncrements(
                DualNum<Time>(
                    doubleArrayOf(
                        ((leftFrontPosVel.position - lastLeftFrontPos).toDouble()),
                        leftFrontPosVel.velocity.toDouble(),
                    ),
                ) * inPerTick,
                DualNum<Time>(
                    doubleArrayOf(
                        ((leftBackPosVel.position - lastLeftBackPos).toDouble()),
                        leftBackPosVel.velocity.toDouble(),
                    ),
                ) * inPerTick,
                DualNum<Time>(
                    doubleArrayOf(
                        ((rightBackPosVel.position - lastRightBackPos).toDouble()),
                        rightBackPosVel.velocity.toDouble(),
                    ),
                ) * inPerTick,
                DualNum<Time>(
                    doubleArrayOf(
                        ((rightFrontPosVel.position - lastRightFrontPos).toDouble()),
                        rightFrontPosVel.velocity.toDouble(),
                    ),
                ) * inPerTick,
            ),
        )

        lastLeftFrontPos = leftFrontPosVel.position
        lastLeftBackPos = leftBackPosVel.position
        lastRightBackPos = rightBackPosVel.position
        lastRightFrontPos = rightFrontPosVel.position

        lastHeading = heading

        pose += Twist2d(twist.line.value(), headingDelta)

        poseHistory.add(0, pose)

        if (poseHistory.size > 100) {
            poseHistory.removeLast()
        }

        vel = twist.velocity().value()
        return vel
    }

    fun withNames(lfName: String, lbName: String, rfName: String, rbName: String) =
        MecanumDriveLocalizer(
            hardwareMap,
            inPerTick,
            imu,
            kinematics,
            lfName,
            lbName,
            rfName,
            rbName,
            lfDirection,
            lbDirection,
            rfDirection,
            rbDirection,
            initialPose,
        )

    fun withDirections(lfDirection: DcMotorSimple.Direction, lbDirection: DcMotorSimple.Direction, rfDirection: DcMotorSimple.Direction, rbDirection: DcMotorSimple.Direction) =
        MecanumDriveLocalizer(
            hardwareMap,
            inPerTick,
            imu,
            kinematics,
            lfName,
            lbName,
            rfName,
            rbName,
            lfDirection,
            lbDirection,
            rfDirection,
            rbDirection,
            initialPose,
        )

    fun withInitialPose(initialPose: Pose2d) =
        MecanumDriveLocalizer(
            hardwareMap,
            inPerTick,
            imu,
            kinematics,
            lfName,
            lbName,
            rfName,
            rbName,
            lfDirection,
            lbDirection,
            rfDirection,
            rbDirection,
            initialPose,
        )
}

class TankLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    @JvmField var inPerTick: Double,
    val kinematics: TankKinematics,
    val leftNames: List<String> = listOf("leftFront", "leftBack"),
    val rightNames: List<String> = listOf("rightFront", "rightBack"),
    val leftDirections: List<DcMotorSimple.Direction> = listOf(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD),
    val rightDirections: List<DcMotorSimple.Direction> = listOf(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD),
    val initialPose: Pose2d = Pose2d.zero,
) : Localizer {
    val leftEncoders: List<Encoder> = leftNames.map { hardwareMap.overflowEncoder(it) }
    val rightEncoders: List<Encoder> = rightNames.map { hardwareMap.overflowEncoder(it) }

    private var lastLeftPos = 0.0
    private var lastRightPos = 0.0

    override var pose: Pose2d = initialPose
    override var vel: PoseVelocity2d = PoseVelocity2d.zero
        private set

    override val poseHistory = mutableListOf<Pose2d>()

    private var initialized = false

    init {
        leftEncoders.zip(leftDirections).forEach { (encoder, direction) -> encoder.direction = direction }
        rightEncoders.zip(rightDirections).forEach { (encoder, direction) -> encoder.direction = direction }
    }

    override fun update(): PoseVelocity2d {
        val leftReadings = leftEncoders.map { it.getPositionAndVelocity() }
        val rightReadings = rightEncoders.map { it.getPositionAndVelocity() }

        val leftPos = leftReadings.sumOf { it.position.toDouble() } / leftReadings.size
        val leftVel = leftReadings.sumOf { it.velocity.toDouble() } / leftReadings.size
        val rightPos = rightReadings.sumOf { it.position.toDouble() } / rightReadings.size
        val rightVel = rightReadings.sumOf { it.velocity.toDouble() } / rightReadings.size

        FlightRecorder.write(
            "TANK_LOCALIZER_INPUTS",
            TankLocalizerInputsMessage(leftReadings, rightReadings),
        )

        if (!initialized) {
            initialized = true

            lastLeftPos = leftPos
            lastRightPos = rightPos

            return PoseVelocity2d.zero
        }

        val twist = kinematics.forward<Time>(
            TankWheelIncrements(
                DualNum<Time>(
                    doubleArrayOf(
                        (leftPos - lastLeftPos),
                        leftVel,
                    ),
                ) * inPerTick,
                DualNum<Time>(
                    doubleArrayOf(
                        (rightPos - lastRightPos),
                        rightVel,
                    ),
                ) * inPerTick,
            ),
        )

        lastLeftPos = leftPos
        lastRightPos = rightPos

        pose += twist.value()

        poseHistory.add(0, pose)
        if (poseHistory.size > 100) {
            poseHistory.removeAt(poseHistory.lastIndex)
        }

        vel = twist.velocity().value()
        return vel
    }

    fun withNames(leftNames: List<String>, rightNames: List<String>) =
        TankLocalizer(
            hardwareMap,
            inPerTick,
            kinematics,
            leftNames,
            rightNames,
            leftDirections,
            rightDirections,
            initialPose,
        )

    fun withDirections(leftDirections: List<DcMotorSimple.Direction>, rightDirections: List<DcMotorSimple.Direction>) =
        TankLocalizer(
            hardwareMap,
            inPerTick,
            kinematics,
            leftNames,
            rightNames,
            leftDirections,
            rightDirections,
            initialPose,
        )

    fun withInitialPose(initialPose: Pose2d) =
        TankLocalizer(
            hardwareMap,
            inPerTick,
            kinematics,
            leftNames,
            rightNames,
            leftDirections,
            rightDirections,
            initialPose,
        )
}

class SwerveLocalizer(
    val hardwareMap: HardwareMap,
    val cpr: Int,
    @JvmField var inPerTick: Double,
    val imu: IMU,
    val kinematics: SwerveKinematics,
    val driveNames: List<String> = listOf("lfDrive", "frDrive", "blDrive", "brDrive"),
    val steeringNames: List<String> = listOf("lfSteering", "rfSteering", "lbSteering", "rbSteering"),
    val driveDirections: List<DcMotorSimple.Direction> = driveNames.map { DcMotorSimple.Direction.FORWARD },
    val steeringDirections: List<DcMotorSimple.Direction> = steeringNames.map { DcMotorSimple.Direction.FORWARD },
    val initialPose: Pose2d = Pose2d.zero
) : Localizer {
    private val driveEncoders: List<Encoder> = driveNames.map { hardwareMap.overflowEncoder(it) }
    private val steeringEncoders: List<Encoder> = steeringNames.map {
        WrappingEncoder(hardwareMap.overflowEncoder(it), cpr)
    }

    private var lastDrivePositions = steeringNames.map { 0 }
    private var lastSteeringPositions = steeringNames.map { 0 }

    private var lastHeading = Rotation2d.zero

    override var pose = initialPose
    override var vel: PoseVelocity2d = PoseVelocity2d.zero
        private set

    override val poseHistory = ArrayList<Pose2d>()

    private var initialized = false
    override fun update(): PoseVelocity2d {
        val drivePositionsAndVelocities = driveEncoders.map { it.getPositionAndVelocity() }
        val steeringPositionsAndVelocities = steeringEncoders.map { it.getPositionAndVelocity() }

        val angles: YawPitchRollAngles = imu.robotYawPitchRollAngles
        val heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

        if (!initialized) {
            initialized = true

            lastDrivePositions = drivePositionsAndVelocities.map { it.position }
            lastSteeringPositions = steeringPositionsAndVelocities.map { it.position }

            lastHeading = heading

            return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        }

        val headingDelta = heading.minus(lastHeading)

        val wheelDeltas = drivePositionsAndVelocities.indices.map {
            DualNum<Time>(doubleArrayOf(
                (drivePositionsAndVelocities[it].position - lastDrivePositions[it]).toDouble(),
                (drivePositionsAndVelocities[it].velocity - lastDrivePositions[it]).toDouble()
            )) * inPerTick
        }
        val steeringAngles = steeringPositionsAndVelocities.indices.map {
            DualNum<Time>(doubleArrayOf(
                (steeringPositionsAndVelocities[it].position - lastDrivePositions[it]).toDouble(),
                (steeringPositionsAndVelocities[it].velocity - lastDrivePositions[it]).toDouble()
            )) * (cpr / 2 * PI)
        }

        lastDrivePositions = drivePositionsAndVelocities.map { it.position }
        lastSteeringPositions = steeringPositionsAndVelocities.map { it.position }

        val twist = kinematics.forward<Time>(SwerveKinematics.SwerveWheelIncrements(
            List(wheelDeltas.size) {
                SwerveModuleIncrements<Time>(wheelDeltas[it], steeringAngles[it].value())
            }))

        lastHeading = heading

        // Update pose using the calculated twist
        pose += Twist2d(twist.line.value(), headingDelta)


        poseHistory.add(0, pose)
        if (poseHistory.size > 100) {
            poseHistory.removeAt(poseHistory.size - 1)
        }

        vel = twist.velocity().value()
        return vel
    }

    fun withNames(driveNames: List<String>, steeringNames: List<String>) =
        SwerveLocalizer(
            hardwareMap,
            cpr,
            inPerTick,
            imu,
            kinematics,
            driveNames,
            steeringNames,
            driveDirections,
            steeringDirections,
            initialPose,
        )

    fun withDirections(driveDirections: List<DcMotorSimple.Direction>, steeringDirections: List<DcMotorSimple.Direction>) =
        SwerveLocalizer(
            hardwareMap,
            cpr,
            inPerTick,
            imu,
            kinematics,
            driveNames,
            steeringNames,
            driveDirections,
            steeringDirections,
            initialPose
        )

    fun withInitialPose(initialPose: Pose2d) =
        SwerveLocalizer(
            hardwareMap,
            cpr,
            inPerTick,
            imu,
            kinematics,
            driveNames,
            steeringNames,
            driveDirections,
            steeringDirections,
            initialPose,
        )
}