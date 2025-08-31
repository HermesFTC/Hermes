package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.roadrunner.control.MecanumKinematics
import com.acmerobotics.roadrunner.geometry.DualNum
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.geometry.Twist2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.Vector2dDual
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

    class MotorActuator(val motor: DcMotorEx) : DrivetrainActuator {
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

class MecanumConfigDriveView(hardwareMap: HardwareMap) : DrivetrainConfigDriveView {

    val config get() = HermesConfig.config.drive as MecanumParameters

    override val actuators = listOf(
        DrivetrainConfigDriveView.MotorActuator(config.leftFront, hardwareMap),
        DrivetrainConfigDriveView.MotorActuator(config.leftBack, hardwareMap),
        DrivetrainConfigDriveView.MotorActuator(config.rightBack, hardwareMap),
        DrivetrainConfigDriveView.MotorActuator(config.rightFront, hardwareMap),
    )

    override fun updateActuator(actuator: DrivetrainConfigDriveView.DrivetrainActuator, deltaPose: Twist2d) {
        val idx = actuators.indexOf(actuator)
        val direction = if (deltaPose.line.x > 0) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE

        when (idx) {
            0 -> config.leftFront
            1 -> config.leftBack
            2 -> config.rightBack
            3 -> config.rightFront
            else -> error("Too many mecanum wheels. Wait, what?")
        }.direction = direction
    }
}