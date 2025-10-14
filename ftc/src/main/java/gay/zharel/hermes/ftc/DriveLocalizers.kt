@file:Suppress("unused")
package gay.zharel.hermes.ftc

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import gay.zharel.fateweaver.flight.FlightRecorder
import gay.zharel.hermes.control.MecanumKinematics
import gay.zharel.hermes.control.MecanumKinematics.MecanumWheelIncrements
import gay.zharel.hermes.control.SwerveKinematics
import gay.zharel.hermes.control.SwerveModuleIncrements
import gay.zharel.hermes.control.TankKinematics
import gay.zharel.hermes.control.TankKinematics.TankWheelIncrements
import gay.zharel.hermes.geometry.*
import gay.zharel.hermes.logs.MecanumLocalizerInputsMessage
import gay.zharel.hermes.logs.TankLocalizerInputsMessage
import gay.zharel.hermes.math.DualNum
import gay.zharel.hermes.math.Time
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import kotlin.math.PI

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

            lastHeading = heading

            return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        }

        val headingDelta = heading.minus(lastHeading)
        val twist = kinematics.forward<Time>(
            MecanumWheelIncrements(
                DualNum<Time>(
                    doubleArrayOf(
                        ((leftFrontPosVel.position - lastLeftFrontPos)),
                        leftFrontPosVel.velocity,
                    ),
                ) * inPerTick,
                DualNum<Time>(
                    doubleArrayOf(
                        ((leftBackPosVel.position - lastLeftBackPos)),
                        leftBackPosVel.velocity,
                    ),
                ) * inPerTick,
                DualNum<Time>(
                    doubleArrayOf(
                        ((rightBackPosVel.position - lastRightBackPos)),
                        rightBackPosVel.velocity,
                    ),
                ) * inPerTick,
                DualNum<Time>(
                    doubleArrayOf(
                        ((rightFrontPosVel.position - lastRightFrontPos)),
                        rightFrontPosVel.velocity,
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

        val leftPos = leftReadings.sumOf { it.position } / leftReadings.size
        val leftVel = leftReadings.sumOf { it.velocity } / leftReadings.size
        val rightPos = rightReadings.sumOf { it.position } / rightReadings.size
        val rightVel = rightReadings.sumOf { it.velocity } / rightReadings.size

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
            DualNum<Time>(
                doubleArrayOf(
                    (drivePositionsAndVelocities[it].position - lastDrivePositions[it]),
                    (drivePositionsAndVelocities[it].velocity - lastDrivePositions[it])
                )
            ) * inPerTick
        }
        val steeringAngles = steeringPositionsAndVelocities.indices.map {
            DualNum<Time>(
                doubleArrayOf(
                    (steeringPositionsAndVelocities[it].position - lastDrivePositions[it]),
                    (steeringPositionsAndVelocities[it].velocity - lastDrivePositions[it])
                )
            ) * (cpr / 2 * PI)
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

