package gay.zharel.hermes.tuning

import gay.zharel.hermes.control.MecanumKinematics
import gay.zharel.hermes.geometry.DualNum
import gay.zharel.hermes.geometry.PoseVelocity2dDual
import gay.zharel.hermes.geometry.Time
import gay.zharel.hermes.geometry.Twist2d
import gay.zharel.hermes.geometry.Vector2d
import gay.zharel.hermes.geometry.Vector2dDual
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.collections.indexOf

interface DriveView {

    /**
     * Voltage overflows scale both to max voltage.
     * @param rotationVoltage Counter-clockwise (standard).
     */
    fun voltageDrive(forwardVoltage: Double, rotationVoltage: Double)

}

interface DriveViewFactory {

    fun make(hardwareMap: HardwareMap): DriveView

}

object TuningDriveViewFactory : DriveViewFactory {
    override fun make(hardwareMap: HardwareMap): DriveView {
        return when (HermesConfig.config.drive) {
            is MecanumParameters -> MecanumDriveView(HermesConfig.config.drive as MecanumParameters, hardwareMap)
            CustomDrive -> error("Custom drive views may not be used with the default tuning factory. Consider implementing your own.")
            DummyParameters -> error("Please select a drive type before running this opmode!")
        }
    }
}

interface HolonomicDriveView : DriveView {

    /**
     * Voltage overflows scale all to max voltage.
     * @param strafeVoltage Positive goes left.
     * @param rotationVoltage Counter-clockwise (standard).
     */
    fun voltageDrive(forwardVoltage: Double, strafeVoltage: Double, rotationVoltage: Double)

}

class MecanumDriveView(parameters: MecanumParameters, hardwareMap: HardwareMap) : HolonomicDriveView {

    private val motors: List<DcMotor> = arrayListOf(
        parameters.leftFront.toDcMotor(hardwareMap),
        parameters.leftBack.toDcMotor(hardwareMap),
        parameters.rightBack.toDcMotor(hardwareMap),
        parameters.rightFront.toDcMotor(hardwareMap),
    )

    /**
     * Voltage overflows scale all to max voltage.
     * @param strafeVoltage Positive goes left.
     * @param rotationVoltage Counter-clockwise (standard).
     */
    override fun voltageDrive(
        forwardVoltage: Double,
        strafeVoltage: Double,
        rotationVoltage: Double,
    ) {
        // normalize by voltage
        val (x, y, r) = arrayListOf(
            forwardVoltage,
            strafeVoltage,
            rotationVoltage,
        ).map { it / VoltageCache.currentVoltage }

        val wheelPowers = MecanumKinematics(1.0).inverse(
            PoseVelocity2dDual(
                Vector2dDual.Companion.constant<Time>(Vector2d(x, y), 1),
                DualNum.Companion.constant<Time>(rotationVoltage, 1),
            ),
        )

        motors.zip(wheelPowers.all()).forEach { (motor, power) -> motor.power = power.value() }
    }

    /**
     * Voltage overflows scale both to max voltage.
     * @param rotationVoltage Counter-clockwise (standard).
     */
    override fun voltageDrive(forwardVoltage: Double, rotationVoltage: Double) {
        voltageDrive(forwardVoltage, 0.0, rotationVoltage)
    }

}

interface DrivetrainConfigDriveView {
    val actuators: List<DrivetrainActuator>

    fun updateActuator(actuator: DrivetrainActuator, deltaPose: Twist2d)

    interface DrivetrainActuator {

        /**
         * Move the actuator as desired.
         * This method should be blocking!!
         */
        fun moveActuator(input: Double)

    }

    class MotorActuator(val motor: DcMotor) : DrivetrainActuator {
        constructor(config: MotorConfig, hardwareMap: HardwareMap) : this(config.toDcMotorEx(hardwareMap))

        override fun moveActuator(input: Double) {
            motor.power = input
            try {
                Thread.sleep(250)
            } catch (e: InterruptedException) {
                throw e
            }
            motor.power = 0.0
        }
    }
}

interface DrivetrainConfigViewFactory {

    fun make(hardwareMap: HardwareMap): DrivetrainConfigDriveView

}

object TuningDrivetrainConfigViewFactory : DrivetrainConfigViewFactory {
    override fun make(hardwareMap: HardwareMap): DrivetrainConfigDriveView {
        return when (HermesConfig.config.drive) {
            is MecanumParameters -> MecanumConfigDriveView(hardwareMap)
            CustomDrive -> error("Custom drive views may not be used with the default tuning factory. Consider implementing your own.")
            DummyParameters -> error("Please select a drive type before running this opmode!")
        }
    }
}

class MecanumConfigDriveView(hardwareMap: HardwareMap) : DrivetrainConfigDriveView {

    val config get() = HermesConfig.tuningConfig.drivetrainConfig.chosenMotors
    val mecConfig get() = HermesConfig.config.drive as MecanumParameters

    override val actuators = listOf(
        DrivetrainConfigDriveView.MotorActuator(hardwareMap.dcMotor.get(config[0])),
        DrivetrainConfigDriveView.MotorActuator(hardwareMap.dcMotor.get(config[1])),
        DrivetrainConfigDriveView.MotorActuator(hardwareMap.dcMotor.get(config[2])),
        DrivetrainConfigDriveView.MotorActuator(hardwareMap.dcMotor.get(config[3])),
    )

    override fun updateActuator(actuator: DrivetrainConfigDriveView.DrivetrainActuator, deltaPose: Twist2d) {
        val idx = actuators.indexOf(actuator)
        val direction = if (deltaPose.line.x > 0) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE


        val yDir = (deltaPose.line.y * if (direction == DcMotorSimple.Direction.FORWARD) 1.0 else -1.0) > 0
        val rDir = (deltaPose.angle * if (direction == DcMotorSimple.Direction.FORWARD) 1.0 else -1.0) > 0

        // figure out what wheel it was based on strafe and rot
        if (yDir && rDir) {
            // left + ccw -> fr
            mecConfig.rightFront = MotorConfig(config[idx], direction)
        } else if (yDir) {
            // left + cw -> bl
            mecConfig.leftBack = MotorConfig(config[idx], direction)
        } else if (rDir) {
            // right + ccw -> br
            mecConfig.rightBack = MotorConfig(config[idx], direction)
        } else {
            // right + cw -> fl
            mecConfig.leftFront = MotorConfig(config[idx], direction)
        }
    }
}