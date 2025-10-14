@file:Suppress("unused")
package gay.zharel.hermes.ftc

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import gay.zharel.hermes.geometry.Pose2d
import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.geometry.Rotation2d
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.tuning.PinpointParameters
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import java.util.*

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
        otos.linearUnit = DistanceUnit.INCH
        otos.angularUnit = AngleUnit.RADIANS

        otos.calibrateImu()
        otos.linearScalar = linearScalar
        otos.angularScalar = angularScalar
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

