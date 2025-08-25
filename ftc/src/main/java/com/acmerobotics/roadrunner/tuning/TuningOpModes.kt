package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.ftc.Localizer
import com.acmerobotics.roadrunner.logs.TuningFiles
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

class ForwardPushTest(val localizerView: ForwardPushLocalizerView) : OpMode() {

    val actualInches: Double get() = HermesConfig.tuningConfig.forwardPush.actualInchesTravelled

    override fun init() {

    }

    override fun loop() {

    }

    override fun stop() {
        val params = localizerView.getParameters(actualInches)
        HermesConfig.addConfigVariable("ForwardPushResults", params)
        HermesConfig.config.localizer = HermesConfig.config.localizer?.assembleIfAble()
    }

}

class LateralPushTest(val localizerView: LateralPushLocalizerView) : OpMode() {

    val actualInches: Double get() = HermesConfig.tuningConfig.lateralPush.actualInchesTravelled

    override fun init() {

    }

    override fun loop() {

    }

    override fun stop() {
        val params = localizerView.getParameters(actualInches)
        HermesConfig.addConfigVariable("LateralPushResults", params)
        HermesConfig.config.localizer = HermesConfig.config.localizer?.assembleIfAble()
    }

}

class AngularPushTest(val localizerView: AngularPushLocalizerView) : OpMode() {

    val actualRevolutions: Double get() = HermesConfig.tuningConfig.angularPush.actualRevolutions

    override fun init() {

    }

    override fun loop() {

    }

    override fun stop() {
        val params = localizerView.getParameters(actualRevolutions)
        HermesConfig.addConfigVariable("AngularPushResults", params)
        HermesConfig.config.localizer = HermesConfig.config.localizer?.assembleIfAble()
    }

}

/**
 * Quasistatic SysID routine for the drivetrain.
 * Determines: kV, kS
 * @param direction +direction = forward
 */
class ForwardRampTest(val driveView: DriveView, val localizer: Localizer, val direction: Double) : LinearOpMode() {

    val sign = sign(direction)

    // cap voltage at current voltage to prevent overflow
    fun voltage(seconds: Double) = if (sign == 1.0)
            min(HermesConfig.tuningConfig.forwardRamp.voltagePerSecond * seconds, VoltageCache.currentVoltage)
        else
            max(HermesConfig.tuningConfig.forwardRamp.voltagePerSecond * seconds * sign, VoltageCache.currentVoltage * sign)

    override fun runOpMode() {

        val forwardVoltage = MutableSignal()
        val forwardVelocity = MutableSignal()

        waitForStart()

        val t = MidpointTimer()
        while (opModeIsActive()) {
            val voltage = voltage(t.seconds()) // for consistency, use the same voltage throughout the entire loop

            forwardVoltage.times.add(t.addSplit())
            forwardVoltage.values.add(voltage * sign) // always log voltage as positive, even if backwards. hey it works

            forwardVelocity.times.add(t.addSplit())
            forwardVelocity.values.add(localizer.vel.linearVel.x * sign) // same with velocity

            driveView.voltageDrive(voltage, 0.0)
            localizer.update()
        }

        driveView.voltageDrive(0.0, 0.0)

        TuningFiles.save(TuningFiles.FileType.FORWARD_RAMP,
            DataFilter.filterQuasistaticByVelocity(HermesConfig.tuningConfig.forwardRamp.thresholdInchesPerSecond,
                QuasistaticParameters(
                    forwardVoltage,
                    forwardVelocity
                )
            )
        )
    }
}

/**
 * Dynamic SysID routine for the drivetrain.
 * Determines: kA
 * @param direction +direction = forward
 */
class ForwardStepTest(val driveView: DriveView, val localizer: Localizer, val direction: Double) : LinearOpMode() {

    val sign = sign(direction)

    fun voltage(seconds: Double) = if (seconds > 0.5) HermesConfig.tuningConfig.forwardStep.voltageStep * sign else 0.0

    override fun runOpMode() {

        val deltaVoltage = MutableSignal()
        val forwardAcceleration = MutableSignal()

        waitForStart()

        val t = MidpointTimer()
        var lastVelX = 0.0
        var lastSeconds = 0.0
        while (opModeIsActive()) {
            val voltage = voltage(t.seconds()) // for consistency, use the same voltage throughout the entire loop

            deltaVoltage.times.add(t.addSplit())
            if (HermesConfig.config.feedforward == null) {
                throw IllegalStateException("go do forward ramp u troll")
            }

            // u - kV * v gives voltage difference between being stable at velocity and voltage we are actually providing
            deltaVoltage.values.add((voltage - (HermesConfig.config.feedforward!!.kV * localizer.vel.linearVel.x)) * sign)

            forwardAcceleration.times.add(t.addSplit())
            forwardAcceleration.values.add((localizer.vel.linearVel.x - lastVelX) * sign / (t.seconds() - lastSeconds))

            lastVelX = localizer.vel.linearVel.x
            lastSeconds = t.seconds()

            driveView.voltageDrive(voltage, 0.0)
            localizer.update()
        }

        driveView.voltageDrive(0.0, 0.0)

        TuningFiles.save(TuningFiles.FileType.FORWARD_STEP,
            DynamicParameters(
                forwardAcceleration,
                deltaVoltage
            )
        )
    }
}

/**
 * Quasistatic SysID routine for the drivetrain.
 * Determines: kV, kS
 * @param direction +direction = counter-clockwise.
 */
class AngularRampTest(val driveView: DriveView, val localizer: Localizer, val direction: Double) : LinearOpMode() {

    val sign = sign(direction)

    // cap voltage at current voltage to prevent overflow
    fun voltage(seconds: Double) = if (sign == 1.0)
        min(HermesConfig.tuningConfig.angularRamp.voltagePerSecond * seconds, VoltageCache.currentVoltage)
    else
        max(HermesConfig.tuningConfig.angularRamp.voltagePerSecond * seconds * sign, VoltageCache.currentVoltage * sign)

    override fun runOpMode() {

        val angularVoltage = MutableSignal()
        val angularVelocity = MutableSignal()

        waitForStart()

        val t = MidpointTimer()
        while (opModeIsActive()) {
            val voltage = voltage(t.seconds()) // for consistency, use the same voltage throughout the entire loop

            angularVoltage.times.add(t.addSplit())
            angularVoltage.values.add(voltage * sign) // always log voltage as positive, even if backwards. hey it works

            angularVelocity.times.add(t.addSplit())
            angularVelocity.values.add(localizer.vel.angVel * sign) // same with velocity

            driveView.voltageDrive(0.0, voltage)
            localizer.update()
        }

        driveView.voltageDrive(0.0, 0.0)

        TuningFiles.save(TuningFiles.FileType.ANGULAR_RAMP,
            DataFilter.filterQuasistaticByVelocity(HermesConfig.tuningConfig.angularRamp.thresholdRadiansPerSecond,
                QuasistaticParameters(
                    angularVoltage,
                    angularVelocity
                )
            )
        )
    }
}

/**
 * Dynamic SysID routine for the drivetrain.
 * Determines: kA
 * @param direction +direction = counter-clockwise
 */
class AngularStepTest(val driveView: DriveView, val localizer: Localizer, val direction: Double) : LinearOpMode() {

    val sign = sign(direction)

    fun voltage(seconds: Double) = if (seconds > 0.5) HermesConfig.tuningConfig.angularStep.voltageStep * sign else 0.0

    override fun runOpMode() {

        val deltaVoltage = MutableSignal()
        val angularAcceleration = MutableSignal()

        waitForStart()

        val t = MidpointTimer()
        var lastVel = 0.0
        var lastSeconds = 0.0
        while (opModeIsActive()) {
            val voltage = voltage(t.seconds()) // for consistency, use the same voltage throughout the entire loop

            deltaVoltage.times.add(t.addSplit())
            if (HermesConfig.config.feedforward == null) {
                throw IllegalStateException("go do forward ramp u troll")
            }

            // u - kV * v gives voltage difference between being stable at velocity and voltage we are actually providing
            deltaVoltage.values.add((voltage - (HermesConfig.config.feedforward!!.kV * localizer.vel.angVel)) * sign)

            angularAcceleration.times.add(t.addSplit())
            angularAcceleration.values.add((localizer.vel.angVel - lastVel) * sign / (t.seconds() - lastSeconds))

            lastVel = localizer.vel.angVel
            lastSeconds = t.seconds()

            driveView.voltageDrive(0.0, voltage)
            localizer.update()
        }

        driveView.voltageDrive(0.0, 0.0)

        TuningFiles.save(TuningFiles.FileType.FORWARD_STEP,
            DynamicParameters(
                angularAcceleration,
                deltaVoltage
            )
        )
    }
}