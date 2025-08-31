@file:Suppress("unused")
package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.MecanumKinematics
import com.acmerobotics.roadrunner.control.MecanumKinematics.MecanumWheelIncrements
import com.acmerobotics.roadrunner.control.SwerveKinematics
import com.acmerobotics.roadrunner.control.SwerveModuleIncrements
import com.acmerobotics.roadrunner.control.TankKinematics
import com.acmerobotics.roadrunner.control.TankKinematics.TankWheelIncrements
import com.acmerobotics.roadrunner.geometry.*
import com.acmerobotics.roadrunner.logs.*
import com.acmerobotics.roadrunner.tuning.PinpointParameters
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

/**
 * A localizer is responsible for tracking the robot's position and velocity (i.e., its pose) in the field frame.
 */
interface Localizer {
    /**
     * The current robot pose.
     */
    var pose: Pose2d

    /**
     * The current robot velocity.
     */
    val vel: PoseVelocity2d

    /**
     * A list of recent poses.
     */
    val poseHistory: MutableList<Pose2d>

    /**
     * Updates the localizer and returns the new velocity.
     */
    fun update(): PoseVelocity2d
}

internal fun HardwareMap.overflowEncoder(name: String) =
    OverflowEncoder(RawEncoder(get(DcMotorEx::class.java, name)))

/**
 * Localizer based on two dead wheels and an IMU.
 *
 * @param hardwareMap hardware map
 * @param imu IMU
 * @param inPerTick inches per tick
 * @param parName name of the parallel encoder
 * @param perpName name of the perpendicular encoder
 * @param parYTicks y-position of the parallel encoder (in tick units)
 * @param perpXTicks x-position of the perpendicular encoder (in tick units)
 * @param parDirection direction of the parallel encoder
 * @param perpDirection direction of the perpendicular encoder
 * @param initialPose initial pose
 */
class TwoDeadWheelLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    val imu: IMU,
    @JvmField var inPerTick: Double,
    val parName: String,
    val perpName: String,
    @JvmField var parYTicks: Double = 0.0,
    @JvmField var perpXTicks: Double = 0.0,
    val parDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val perpDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val initialPose: Pose2d = Pose2d.zero,
) : Localizer {
    val par: Encoder = hardwareMap.overflowEncoder(parName)
    val perp: Encoder = hardwareMap.overflowEncoder(perpName)

    private var lastParPos = 0.0
    private var lastPerpPos = 0.0
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

        val parPosDelta = parPosVel.position - lastParPos
        val perpPosDelta = perpPosVel.position - lastPerpPos
        val headingDelta = heading.minus(lastHeading!!)

        val twist = Twist2dDual(
            Vector2dDual(
                DualNum<Time>(
                    doubleArrayOf(
                        parPosDelta - parYTicks * headingDelta,
                        parPosVel.velocity - parYTicks * headingVel,
                    ),
                ).times(inPerTick),
                DualNum<Time>(
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
    
    /**
     * Returns a new localizer with the given encoder names.
     */
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

    /**
     * Returns a new localizer with the given encoder locations.
     */
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

    /**
     * Returns a new localizer with the given encoder directions.
     */
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

    /**
     * Returns a new localizer with the given initial pose.
     */
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

/**
 * Localizer based on three dead wheels.
 *
 * @param hardwareMap hardware map
 * @param inPerTick inches per tick
 * @param par0Name name of the first parallel encoder
 * @param par1Name name of the second parallel encoder
 * @param perpName name of the perpendicular encoder
 * @param par0YTicks y-position of the first parallel encoder (in tick units)
 * @param par1YTicks y-position of the second parallel encoder (in tick units)
 * @param perpXTicks x-position of the perpendicular encoder (in tick units)
 * @param par0Direction direction of the first parallel encoder
 * @param par1Direction direction of the second parallel encoder
 * @param perpDirection direction of the perpendicular encoder
 * @param initialPose initial pose
 */
@Config
class ThreeDeadWheelLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    @JvmField var inPerTick: Double,
    val par0Name: String = "par0",
    val par1Name: String = "par1",
    val perpName: String = "perp",
    @JvmField var par0YTicks: Double = 0.0,
    @JvmField var par1YTicks: Double = 1.0,
    @JvmField var perpXTicks: Double = 0.0,
    val par0Direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val par1Direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val perpDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val initialPose: Pose2d = Pose2d.zero,
    ) : Localizer {
    val par0: Encoder = hardwareMap.overflowEncoder(par0Name)
    val par1: Encoder = hardwareMap.overflowEncoder(par1Name)
    val perp: Encoder = hardwareMap.overflowEncoder(perpName)

    private var lastPar0Pos = 0.0
    private var lastPar1Pos = 0.0
    private var lastPerpPos = 0.0
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

    /**
     * Returns a new localizer with the given encoder names.
     */
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
        perpDirection,
    )

    /**
     * Returns a new localizer with the given encoder locations.
     */
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
            perpDirection,
        )

    /**
     * Returns a new localizer with the given encoder directions.
     */
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

    /**
     * Returns a new localizer with the given initial pose.
     */
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
        perpDirection,
        initialPose,
    )
}

/**
 * Localizer based on the GoBilda Pinpoint dead wheel module.
 *
 * @param hardwareMap hardware map
 * @param inPerTick inches per tick
 * @param name name of the Pinpoint device
 * @param parYTicks y-position of the parallel encoder (in tick units)
 * @param perpXTicks x-position of the perpendicular encoder (in tick units)
 * @param parDirection direction of the parallel encoder
 * @param perpDirection direction of the perpendicular encoder
 * @param initialPose initial pose
 */
class PinpointLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    val parameters: PinpointParameters,
    val initialPose: Pose2d = Pose2d.zero,
) : Localizer {

    val inPerTick: Double by parameters::inPerTick
    val name: String by parameters::name
    val parYTicks: Double by parameters::parYTicks
    val perpXTicks: Double by parameters::perpXTicks
    val parDirection: DcMotorSimple.Direction by parameters::parDirection
    val perpDirection: DcMotorSimple.Direction by parameters::perpDirection

    // old constructor
    constructor(
        hardwareMap: HardwareMap,
        inPerTick: Double,
        name: String = "pinpoint",
        parYTicks: Double = 0.0,
        perpXTicks: Double = 0.0,
        parDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
        perpDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
        initialPose: Pose2d = Pose2d.zero,
    ) : this(
        hardwareMap,
        PinpointParameters(
            inPerTick,
            name,
            parYTicks,
            perpXTicks,
            parDirection,
            perpDirection,
        ),
        initialPose
    )


    val driver: GoBildaPinpointDriver =
        hardwareMap.get(GoBildaPinpointDriver::class.java, parameters.name)

    private var txWorldPinpoint: Pose2d
    private var txPinpointRobot = Pose2d(0.0, 0.0, 0.0)
    private var currentPose: Pose2d = Pose2d.zero

    override val poseHistory = mutableListOf<Pose2d>()

    init {
        driver.setEncoderResolution(parameters.inPerTick, DistanceUnit.INCH)
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
            val robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity)

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

    /**
     * Returns a new localizer with the given device name.
     */
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

    /**
     * Returns a new localizer with the given offsets.
     */
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

    /**
     * Returns a new localizer with the given encoder directions.
     */
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

    /**
     * Returns a new localizer with the given initial pose.
     */
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

    companion object {
        val DcMotorSimple.Direction.pinpointDirection get() = when(this) {
            DcMotorSimple.Direction.FORWARD -> GoBildaPinpointDriver.EncoderDirection.FORWARD
            DcMotorSimple.Direction.REVERSE -> GoBildaPinpointDriver.EncoderDirection.REVERSED
        }
    }
}

/**
 * Localizer based on the SparkFun OTOS sensor.
 *
 * @param hardwareMap hardware map
 * @param otosName name of the OTOS sensor in the hardware map
 * @param linearScalar scalar for the linear measurements
 * @param angularScalar scalar for the angular measurements
 * @param offset sensor offset
 * @param initialPose initial pose
 */
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

    /**
     * Returns a new localizer with the given sensor name.
     */
    fun withName(otosName: String) = OTOSLocalizer(
        hardwareMap,
        otosName,
        linearScalar,
        angularScalar,
        offset,
        initialPose,
    )

    /**
     * Returns a new localizer with the given scalars.
     */
    fun withScalars(linearScalar: Double, angularScalar: Double) = OTOSLocalizer(
        hardwareMap,
        otosName,
        linearScalar,
        angularScalar,
        offset,
        initialPose,
    )

    /**
     * Returns a new localizer with the given offset.
     */
    fun withOffset(offset: Pose2d) = OTOSLocalizer(
        hardwareMap,
        otosName,
        linearScalar,
        angularScalar,
        offset,
        initialPose,
    )

    /**
     * Returns a new localizer with the given offset.
     */
    fun withOffset(x: Double, y: Double, h: Double) =
        withOffset(Pose2d(x, y, h))

    /**
     * Returns a new localizer with the given initial pose.
     */
    fun withInitialPose(initialPose: Pose2d) = OTOSLocalizer(
        hardwareMap,
        otosName,
        linearScalar,
        angularScalar,
        offset,
        initialPose,
    )

    companion object {
        @JvmStatic
        fun SparkFunOTOS.Pose2D.toRRPose() = Pose2d(x, y, h)

        @JvmStatic
        fun Pose2d.toOTOSPose() = SparkFunOTOS.Pose2D(position.x, position.y, heading.toDouble())
    }
}

/**
 * Localizer based on mecanum drive encoders and an IMU. This localizer is not recommended for use with dead wheels.
 *
 * @param hardwareMap hardware map
 * @param inPerTick inches per tick
 * @param imu IMU
 * @param kinematics mecanum kinematics
 * @param leftFront left front encoder
 * @param leftBack left back encoder
 * @param rightFront right front encoder
 * @param rightBack right back encoder
 * @param initialPose initial pose
 */
@Config
class MecanumDriveLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    @JvmField var inPerTick: Double,
    val imu: IMU,
    val kinematics: MecanumKinematics,
    val leftFront: Encoder,
    val leftBack: Encoder,
    val rightFront: Encoder,
    val rightBack: Encoder,
    val initialPose: Pose2d = Pose2d.zero,
) : Localizer {
    /**
     * Creates a new localizer.
     *
     * @param hardwareMap hardware map
     * @param inPerTick inches per tick
     * @param imu IMU
     * @param kinematics mecanum kinematics
     * @param lfName name of the left front motor
     * @param lbName name of the left back motor
     * @param rfName name of the right front motor
     * @param rbName name of the right back motor
     * @param lfDirection direction of the left front motor
     * @param lbDirection direction of the left back motor
     * @param rfDirection direction of the right front motor
     * @param rbDirection direction of the right back motor
     * @param initialPose initial pose
     */
    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        inPerTick: Double,
        imu: IMU,
        kinematics: MecanumKinematics,
        lfName: String = "leftFront",
        lbName: String = "leftBack",
        rfName: String = "rightFront",
        rbName: String = "rightBack",
        lfDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
        lbDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
        rfDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
        rbDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
        initialPose: Pose2d = Pose2d.zero,
    ) : this(
        hardwareMap,
        inPerTick,
        imu,
        kinematics,
        hardwareMap.overflowEncoder(lfName).apply { direction = lfDirection },
        hardwareMap.overflowEncoder(lbName).apply { direction = lbDirection },
        hardwareMap.overflowEncoder(rfName).apply { direction = rfDirection },
        hardwareMap.overflowEncoder(rbName).apply { direction = rbDirection },
        initialPose,
    )

    private var lastLeftFrontPos = 0.0
    private var lastLeftBackPos = 0.0
    private var lastRightBackPos = 0.0
    private var lastRightFrontPos = 0.0
    private var lastHeading = Rotation2d.zero

    override var pose: Pose2d = initialPose
    override var vel: PoseVelocity2d = PoseVelocity2d.zero
        private set

    override val poseHistory = mutableListOf<Pose2d>()

    private var initialized = false

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
                .0
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
            poseHistory.removeAt(poseHistory.lastIndex)
        }

        vel = twist.velocity().value()
        return vel
    }

    /**
     * Returns a new localizer with the given motor names.
     */
    fun withNames(lfName: String, lbName: String, rfName: String, rbName: String) =
        MecanumDriveLocalizer(
            hardwareMap,
            inPerTick,
            imu,
            kinematics,
            hardwareMap.overflowEncoder(lfName),
            hardwareMap.overflowEncoder(lbName),
            hardwareMap.overflowEncoder(rfName),
            hardwareMap.overflowEncoder(rbName),
            initialPose,
        )

    /**
     * Returns a new localizer with the given motor directions.
     */
    fun withDirections(lfDirection: DcMotorSimple.Direction, lbDirection: DcMotorSimple.Direction, rfDirection: DcMotorSimple.Direction, rbDirection: DcMotorSimple.Direction) =
        MecanumDriveLocalizer(
            hardwareMap,
            inPerTick,
            imu,
            kinematics,
            leftFront.apply { direction = lfDirection },
            leftBack.apply { direction = lbDirection },
            rightFront.apply { direction = rfDirection },
            rightBack.apply { direction = rbDirection },
            initialPose,
        )

    /**
     * Returns a new localizer with the given initial pose.
     */
    fun withInitialPose(initialPose: Pose2d) =
        MecanumDriveLocalizer(
            hardwareMap,
            inPerTick,
            imu,
            kinematics,
            leftFront,
            leftBack,
            rightFront,
            rightBack,
            initialPose,
        )

    /**
     * Returns a new localizer with the given motors.
     */
    fun fromMotors(lf: DcMotorEx, lb: DcMotorEx, rf: DcMotorEx, rb: DcMotorEx) =
        MecanumDriveLocalizer(
            hardwareMap,
            inPerTick,
            imu,
            kinematics,
            OverflowEncoder(RawEncoder(lf)),
            OverflowEncoder(RawEncoder(lb)),
            OverflowEncoder(RawEncoder(rf)),
            OverflowEncoder(RawEncoder(rb)),
            initialPose,
        )
}

/**
 * Localizer based on tank drive encoders. This localizer is not recommended for use with dead wheels.
 *
 * @param hardwareMap hardware map
 * @param inPerTick inches per tick
 * @param kinematics tank kinematics
 * @param leftEncoders left encoders
 * @param rightEncoders right encoders
 * @param initialPose initial pose
 */
@Config
class TankLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    @JvmField var inPerTick: Double,
    val kinematics: TankKinematics,
    val leftEncoders: List<Encoder>,
    val rightEncoders: List<Encoder>,
    val initialPose: Pose2d = Pose2d.zero,
) : Localizer {
    /**
     * Creates a new localizer.
     *
     * @param hardwareMap hardware map
     * @param kinematics tank kinematics
     * @param inPerTick inches per tick
     * @param leftNames names of the left motors
     * @param rightNames names of the right motors
     * @param leftDirections directions of the left motors
     * @param rightDirections directions of the right motors
     * @param initialPose initial pose
     */
    @JvmOverloads constructor(
        hardwareMap: HardwareMap,
        kinematics: TankKinematics,
        inPerTick: Double,
        leftNames: List<String> = listOf("leftFront", "leftBack"),
        rightNames: List<String> = listOf("rightFront", "rightBack"),
        leftDirections: List<DcMotorSimple.Direction> = listOf(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD),
        rightDirections: List<DcMotorSimple.Direction> = listOf(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD),
        initialPose: Pose2d = Pose2d.zero
        ) : this(
            hardwareMap,
            inPerTick,
            kinematics,
            leftNames.map { hardwareMap.overflowEncoder(it) },
            rightNames.map { hardwareMap.overflowEncoder(it) },
            initialPose,
        ) {
            require(leftNames.size == leftDirections.size) { "each left motor must have a direction" }
            require(rightNames.size == rightDirections.size) { "each right motor must have a direction" }

            leftNames.indices.forEach {
                leftEncoders[it].direction = leftDirections[it]
            }
            rightNames.indices.forEach {
                rightEncoders[it].direction = rightDirections[it]
            }
        }

    private var lastLeftPos = 0.0
    private var lastRightPos = 0.0

    override var pose: Pose2d = initialPose
    override var vel: PoseVelocity2d = PoseVelocity2d.zero
        private set

    override val poseHistory = mutableListOf<Pose2d>()

    private var initialized = false

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

    /**
     * Returns a new localizer with the given motor names.
     */
    fun withNames(leftNames: List<String>, rightNames: List<String>) =
        TankLocalizer(
            hardwareMap,
            inPerTick,
            kinematics,
            leftNames.map { hardwareMap.overflowEncoder(it) },
            rightNames.map { hardwareMap.overflowEncoder(it) },
            initialPose,
        )

    /**
     * Returns a new localizer with the given motor directions.
     */
    fun withDirections(leftDirections: List<DcMotorSimple.Direction>, rightDirections: List<DcMotorSimple.Direction>) =
        apply {
            require(leftEncoders.size == leftDirections.size) { "each left encoder must have a direction" }
            require(rightEncoders.size == rightDirections.size) { "each right encoder must have a direction" }

            leftEncoders.indices.forEach {
                leftEncoders[it].direction = leftDirections[it]
            }
            rightEncoders.indices.forEach {
                rightEncoders[it].direction = rightDirections[it]
        }
    }

    /**
     * Returns a new localizer with the given initial pose.
     */
    fun withInitialPose(initialPose: Pose2d) =
        TankLocalizer(
            hardwareMap,
            inPerTick,
            kinematics,
            leftEncoders,
            rightEncoders,
            initialPose,
        )

    /**
     * Returns a new localizer with the given motors.
     */
    fun fromMotors(leftMotors: List<DcMotorEx>, rightMotors: List<DcMotorEx>) =
        TankLocalizer(
            hardwareMap,
            inPerTick,
            kinematics,
            leftMotors.map { OverflowEncoder(RawEncoder(it)) },
            rightMotors.map { OverflowEncoder(RawEncoder(it)) },
            initialPose,
        )
}

/**
 * Localizer based on swerve drive encoders and an IMU. This localizer is not recommended for use with dead wheels.
 *
 * @param hardwareMap hardware map
 * @param cpr counts per revolution of the steering encoders
 * @param inPerTick inches per tick of the drive encoders
 * @param imu IMU
 * @param kinematics swerve kinematics
 * @param driveEncoders drive encoders
 * @param steeringEncoders steering encoders
 * @param initialPose initial pose
 */
@Config
class SwerveLocalizer @JvmOverloads constructor(
    val hardwareMap: HardwareMap,
    val cpr: Int,
    @JvmField var inPerTick: Double,
    val imu: IMU,
    val kinematics: SwerveKinematics,
    private val driveEncoders: List<Encoder>,
    private val steeringEncoders: List<Encoder>,
    val initialPose: Pose2d = Pose2d.zero
) : Localizer {
    /**
     * Creates a new localizer.
     *
     * @param hardwareMap hardware map
     * @param cpr counts per revolution of the steering encoders
     * @param inPerTick inches per tick of the drive encoders
     * @param imu IMU
     * @param kinematics swerve kinematics
     * @param driveNames names of the drive motors
     * @param steeringNames names of the steering motors
     * @param driveDirections directions of the drive motors
     * @param steeringDirections directions of the steering motors
     * @param initialPose initial pose
     */
    constructor(
        hardwareMap: HardwareMap,
        cpr: Int,
        inPerTick: Double,
        imu: IMU,
        kinematics: SwerveKinematics,
        driveNames: List<String> = listOf("lfDrive", "frDrive", "blDrive", "brDrive"),
        steeringNames: List<String> = listOf("lfSteering", "rfSteering", "lbSteering", "rbSteering"),
        driveDirections: List<DcMotorSimple.Direction> = driveNames.map { DcMotorSimple.Direction.FORWARD },
        steeringDirections: List<DcMotorSimple.Direction> = steeringNames.map { DcMotorSimple.Direction.FORWARD },
        initialPose: Pose2d = Pose2d.zero,
    ) : this(
        hardwareMap,
        cpr,
        inPerTick,
        imu,
        kinematics,
        driveNames.map { hardwareMap.overflowEncoder(it) },
        steeringNames.map { WrappingEncoder(hardwareMap.overflowEncoder(it), cpr) },
        initialPose,
    ) {
        require(driveNames.size == driveDirections.size) { "each drive motor must have a direction" }
        require(steeringNames.size == steeringDirections.size) { "each steering motor must have a direction" }

        driveNames.indices.forEach {
            driveEncoders[it].direction = driveDirections[it]
        }

        steeringNames.indices.forEach {
            steeringEncoders[it].direction = steeringDirections[it]
        }
    }


    private var lastDrivePositions = driveEncoders.map { 0.0 }
    private var lastSteeringPositions = steeringEncoders.map { 0.0 }

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
                SwerveModuleIncrements(wheelDeltas[it], steeringAngles[it].value())
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

    /**
     * Returns a new localizer with the given motor names.
     */
    fun withNames(driveNames: List<String>, steeringNames: List<String>) =
        SwerveLocalizer(
            hardwareMap,
            cpr,
            inPerTick,
            imu,
            kinematics,
            driveNames.map { hardwareMap.overflowEncoder(it) },
            steeringNames.map { WrappingEncoder(hardwareMap.overflowEncoder(it), cpr) },
            initialPose,
        )

    /**
     * Returns a new localizer with the given motor directions.
     */
    fun withDirections(driveDirections: List<DcMotorSimple.Direction>, steeringDirections: List<DcMotorSimple.Direction>) =
        apply {
            require(driveEncoders.size == driveDirections.size) { "each drive motor must have a direction" }
            require(steeringEncoders.size == steeringDirections.size) { "each steering motor must have a direction" }

            driveEncoders.indices.forEach {
                driveEncoders[it].direction = driveDirections[it]
            }

            steeringEncoders.indices.forEach {
                steeringEncoders[it].direction = steeringDirections[it]
            }
        }

    /**
     * Returns a new localizer with the given initial pose.
     */
    fun withInitialPose(initialPose: Pose2d) =
        SwerveLocalizer(
            hardwareMap,
            cpr,
            inPerTick,
            imu,
            kinematics,
            driveEncoders,
            steeringEncoders,
            initialPose,
        )

    /**
     * Returns a new localizer with the given motors.
     */
    fun fromMotors(driveMotors: List<DcMotorEx>, steeringMotors: List<DcMotorEx>) =
        SwerveLocalizer(
            hardwareMap,
            cpr,
            inPerTick,
            imu,
            kinematics,
            driveMotors.map { OverflowEncoder(RawEncoder(it)) },
            steeringMotors.map { WrappingEncoder(OverflowEncoder(RawEncoder(it)), cpr) },
            initialPose,
        )

}
