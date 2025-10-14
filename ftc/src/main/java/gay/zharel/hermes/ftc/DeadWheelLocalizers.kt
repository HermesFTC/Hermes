@file:Suppress("unused")
package gay.zharel.hermes.ftc

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import gay.zharel.fateweaver.flight.FlightRecorder
import gay.zharel.hermes.geometry.*
import gay.zharel.hermes.logs.ThreeDeadWheelInputsMessage
import gay.zharel.hermes.logs.TwoDeadWheelInputsMessage
import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.Time
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

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

