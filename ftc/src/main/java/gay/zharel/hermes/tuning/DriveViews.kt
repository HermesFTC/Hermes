package gay.zharel.hermes.tuning

import gay.zharel.hermes.control.MecanumKinematics
import gay.zharel.hermes.control.MotorFeedforward
import gay.zharel.hermes.control.TankKinematics
import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.math.Time
import gay.zharel.hermes.hardware.Encoder
import gay.zharel.hermes.hardware.EncoderGroup
import gay.zharel.hermes.hardware.LazyImu
import gay.zharel.hermes.hardware.LynxQuadratureEncoderGroup
import com.google.gson.annotations.SerializedName
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import kotlin.math.absoluteValue
import kotlin.math.max


enum class DriveType {
    @SerializedName("mecanum")
    MECANUM,
    @SerializedName("tank")
    TANK
}

fun interface FeedforwardFactory {
    fun make(): MotorFeedforward
}

data class EncoderRef(
    val groupIndex: Int,
    val index: Int,
)

class DriveView(
    val type: DriveType,
    val inPerTick: Double,
    val maxVel: Double,
    val minAccel: Double,
    val maxAccel: Double,
    val encoderGroups: List<EncoderGroup>,
    // ordered front to rear
    val leftMotors: List<DcMotorEx>,
    val rightMotors: List<DcMotorEx>,
    // invariant: (leftEncs.isEmpty() && rightEncs.isEmpty()) ||
    //                  (parEncs.isEmpty() && perpEncs.isEmpty())
    val leftEncs: List<EncoderRef>,
    val rightEncs: List<EncoderRef>,
    val parEncs: List<EncoderRef>,
    val perpEncs: List<EncoderRef>,
    val imu: LazyImu,
    val voltageSensor: VoltageSensor,
    val feedforwardFactory: FeedforwardFactory,
    bogus: Int,
) {
    // Legacy constructor to preserve compatibility with older quickstarts.
    constructor(
        type: DriveType,
        inPerTick: Double,
        maxVel: Double,
        minAccel: Double,
        maxAccel: Double,
        lynxModules: List<LynxModule>,
        // ordered front to rear
        leftMotors: List<DcMotorEx>,
        rightMotors: List<DcMotorEx>,
        // invariant: (leftEncs.isEmpty() && rightEncs.isEmpty()) ||
        //                  (parEncs.isEmpty() && perpEncs.isEmpty())
        leftEncs: List<Encoder>,
        rightEncs: List<Encoder>,
        parEncs: List<Encoder>,
        perpEncs: List<Encoder>,
        imu: LazyImu,
        voltageSensor: VoltageSensor,
        feedforwardFactory: FeedforwardFactory,
    ) : this(
        type,
        inPerTick,
        maxVel,
        minAccel,
        maxAccel,
        listOf(LynxQuadratureEncoderGroup(lynxModules, leftEncs + rightEncs + parEncs + perpEncs)),
        leftMotors,
        rightMotors,
        List(leftEncs.size) { i -> EncoderRef(0, i) },
        List(rightEncs.size) { i -> EncoderRef(0, leftEncs.size + i) },
        List(parEncs.size) { i -> EncoderRef(0, leftEncs.size + rightEncs.size + i) },
        List(perpEncs.size) { i -> EncoderRef(0, leftEncs.size + rightEncs.size + parEncs.size + i) },
        imu,
        voltageSensor,
        feedforwardFactory,
        0,
    )

    val motors = leftMotors + rightMotors

    val forwardEncs = leftEncs + rightEncs + parEncs

    init {
        require((leftEncs.isEmpty() && rightEncs.isEmpty()) || (parEncs.isEmpty() && perpEncs.isEmpty()))
    }

    fun wrappedEncoder(ref: EncoderRef) = encoderGroups[ref.groupIndex].encoders[ref.index]
    fun encoder(ref: EncoderRef) = encoderGroups[ref.groupIndex].encoders[ref.index]

    fun setDrivePowers(powers: PoseVelocity2d) {
        when (type) {
            DriveType.MECANUM -> {
                val wheelPowers = MecanumKinematics(1.0).inverse(PoseVelocity2dDual.constant<Time>(powers, 1))
                val maxPowerMag = wheelPowers.all().maxOfOrNull { it.value().absoluteValue }!!
                val divisor = max(1.0, maxPowerMag)

                leftMotors[0].power = wheelPowers.leftFront.value() / divisor
                leftMotors[1].power = wheelPowers.leftBack.value() / divisor
                rightMotors[0].power = wheelPowers.rightFront.value() / divisor
                rightMotors[1].power = wheelPowers.rightBack.value() / divisor
            }

            DriveType.TANK -> {
                val wheelPowers = TankKinematics(2.0).inverse(PoseVelocity2dDual.constant<Time>(powers, 1))
                val maxPowerMag = wheelPowers.all().maxOfOrNull { it.value().absoluteValue }!!
                val divisor = max(1.0, maxPowerMag)

                for (m in leftMotors) {
                    m.power = wheelPowers.left.value() / divisor
                }
                for (m in rightMotors) {
                    m.power = wheelPowers.right.value() / divisor
                }
            }
        }
    }
}

interface DriveViewFactory {
    fun make(h: HardwareMap): DriveView
}