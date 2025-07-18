package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.*
import com.acmerobotics.roadrunner.geometry.Rotation2d.Companion.exp
import com.acmerobotics.roadrunner.geometry.Rotation2d.Companion.fromDouble
import com.acmerobotics.roadrunner.hardware.Encoder
import com.acmerobotics.roadrunner.hardware.OverflowEncoder
import com.acmerobotics.roadrunner.hardware.PositionVelocityPair
import com.acmerobotics.roadrunner.hardware.RawEncoder
import com.acmerobotics.roadrunner.hardware.pinpointDirection
import com.acmerobotics.roadrunner.hardware.toOTOSPose
import com.acmerobotics.roadrunner.hardware.toRRPose
import com.acmerobotics.roadrunner.logs.FlightRecorder
import com.acmerobotics.roadrunner.logs.ThreeDeadWheelInputsMessage
import com.acmerobotics.roadrunner.logs.TwoDeadWheelInputsMessage
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import java.util.*
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
    val inPerTick: Double,
    val parName: String,
    val perpName: String,
    val parYTicks: Double = 0.0, // y position of the parallel encoder (in tick units)
    val perpXTicks: Double = 0.0,
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
    val inPerTick: Double, 
    val par0Name: String = "par0",
    val par1Name: String = "par1", 
    val perpName: String = "perp",
    val par0YTicks: Double = 0.0, // y position of the first parallel encoder (in tick units)
    val par1YTicks: Double = 1.0, // y position of the second parallel encoder (in tick units)
    val perpXTicks: Double = 0.0, // x position of the perpendicular encoder (in tick units)
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
class PinpointLocalizer(
    val hardwareMap: HardwareMap,
    val inPerTick: Double,
    val name: String = "pinpoint",
    val parYTicks: Double = 0.0,
    val perpXTicks: Double = 0.0,
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
class OTOSLocalizer(
    val hardwareMap: HardwareMap,
    val otosName: String = "sensor_otos",
    val linearScalar: Double = 1.0,
    val angularScalar: Double = 1.0,
    val offset: Pose2d = Pose2d.zero,
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
        val robotVel = exp(otosPose.h).inverse().times(fieldVel)

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