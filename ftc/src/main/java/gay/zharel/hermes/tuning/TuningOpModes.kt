package gay.zharel.hermes.tuning

import gay.zharel.hermes.logs.TuningFiles
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

// ===== Localizer Tuning =====

class ForwardPushTest(val lvf: LocalizerViewFactory) : OpMode() {

    val actualInches: Double get() = HermesConfig.tuningConfig.forwardPush.actualInchesTravelled

    val localizerView by lazy { lvf.make(hardwareMap) }

    override fun init() {
        localizerView
    }

    override fun loop() {

    }

    override fun stop() {
        localizerView.forwardPushUpdate(actualInches)
    }

}

class LateralPushTest(val lvf: LocalizerViewFactory) : OpMode() {

    val actualInches: Double get() = HermesConfig.tuningConfig.lateralPush.actualInchesTravelled

    val localizerView by lazy { lvf.make(hardwareMap) }

    override fun init() {
        localizerView
    }

    override fun loop() {

    }

    override fun stop() {
        localizerView.lateralPushUpdate(actualInches)
    }

}

class AngularPushTest(val lvf: LocalizerViewFactory) : OpMode() {

    val actualRevolutions: Double get() = HermesConfig.tuningConfig.angularPush.actualRevolutions

    val localizerView by lazy { lvf.make(hardwareMap) }

    override fun init() {
        localizerView
    }

    override fun loop() {

    }

    override fun stop() {
        localizerView.angularPushUpdate(actualRevolutions)
    }

}

// ===== Drive Tuning =====

class MotorFetchOpMode() : LinearOpMode() {
    override fun runOpMode() {
        HermesConfig.tuningConfig.drivetrainConfig.motorNames.addAll(
            hardwareMap.getAllNames(DcMotor::class.java)
        )

        waitForStart()

        while (opModeIsActive()) {}
    }
}

/**
 * Automatically configures the motors for a drivetrain.
 */
// TODO: replace constructor arguments with factories
class DrivetrainConfigTest(val dvf: DrivetrainConfigViewFactory, val lf: LocalizerFactory) : LinearOpMode() {

    override fun runOpMode() {

        val driveView = dvf.make(hardwareMap)
        val localizer = lf.make(hardwareMap)

        waitForStart()

        for (actuator in driveView.actuators) {
            val initialPose = localizer.pose

            actuator.moveActuator(0.5)

            localizer.update()

            val endPose = localizer.pose

            driveView.updateActuator(actuator, endPose.minus(initialPose))
        }

        while (opModeIsActive()) {} // make the sdk not freak out

    }

}


// ===== Feedforward Tuning =====

/**
 * Quasistatic SysID routine for the drivetrain.
 * Determines: kV, kS
 * direction: +direction = forward
 */
class ForwardRampTest(val dvf: DriveViewFactory, val lf: LocalizerFactory) : LinearOpMode() {

    val regressionParams: QuasistaticParameters by HermesConfig.tuningConfig.forwardRamp::regressionParams

    val direction: Double by HermesConfig.tuningConfig.forwardRamp::direction

    val sign = sign(direction)

    // cap voltage at current voltage to prevent overflow
    fun voltage(seconds: Double) = if (sign == 1.0)
            min(HermesConfig.tuningConfig.forwardRamp.voltagePerSecond * seconds, VoltageCache.currentVoltage)
        else
            max(HermesConfig.tuningConfig.forwardRamp.voltagePerSecond * seconds * sign, VoltageCache.currentVoltage * sign)

    override fun runOpMode() {

        val driveView = dvf.make(hardwareMap)
        val localizer = lf.make(hardwareMap)

        val forwardVoltage by regressionParams::voltages
        val forwardVelocity by regressionParams::velocities

        VoltageCache.init(hardwareMap)

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
                regressionParams
            )
        )
    }
}

/**
 * Dynamic SysID routine for the drivetrain.
 * Determines: kA
 * direction: +direction = forward
 */
class ForwardStepTest(val dvf: DriveViewFactory, val lf: LocalizerFactory) : LinearOpMode() {

    val regressionParams: DynamicParameters by HermesConfig.tuningConfig.forwardStep::regressionParams

    val direction: Double by HermesConfig.tuningConfig.forwardStep::direction

    val sign = sign(direction)

    fun voltage(seconds: Double) = if (seconds > 0.5) HermesConfig.tuningConfig.forwardStep.voltageStep * sign else 0.0

    override fun runOpMode() {

        val driveView = dvf.make(hardwareMap)
        val localizer = lf.make(hardwareMap)

        val deltaVoltage by regressionParams::voltages
        val forwardAcceleration by regressionParams::accelerations

        VoltageCache.init(hardwareMap)

        waitForStart()

        val t = MidpointTimer()
        var lastVelX = 0.0
        var lastSeconds = 0.0
        while (opModeIsActive()) {
            val voltage = voltage(t.seconds()) // for consistency, use the same voltage throughout the entire loop

            deltaVoltage.times.add(t.addSplit())

            // u - kV * v gives voltage difference between being stable at velocity and voltage we are actually providing
            deltaVoltage.values.add((voltage - (HermesConfig.config.feedforward.translational.kV * localizer.vel.linearVel.x)) * sign)

            forwardAcceleration.times.add(t.addSplit())
            forwardAcceleration.values.add((localizer.vel.linearVel.x - lastVelX) * sign / (t.seconds() - lastSeconds))

            lastVelX = localizer.vel.linearVel.x
            lastSeconds = t.seconds()

            driveView.voltageDrive(voltage, 0.0)
            localizer.update()
        }

        driveView.voltageDrive(0.0, 0.0)

        TuningFiles.save(TuningFiles.FileType.FORWARD_STEP,
            regressionParams
        )
    }
}

/**
 * Quasistatic SysID routine for the drivetrain.
 * Determines: kV, kS
 * direction: +direction = counter-clockwise.
 */
class AngularRampTest(val dvf: DriveViewFactory, val lf: LocalizerFactory) : LinearOpMode() {

    val regressionParams: QuasistaticParameters by HermesConfig.tuningConfig.angularRamp::regressionParams

    val direction: Double by HermesConfig.tuningConfig.angularRamp::direction

    val sign = sign(direction)

    // cap voltage at current voltage to prevent overflow
    fun voltage(seconds: Double) = if (sign == 1.0)
        min(HermesConfig.tuningConfig.angularRamp.voltagePerSecond * seconds, VoltageCache.currentVoltage)
    else
        max(HermesConfig.tuningConfig.angularRamp.voltagePerSecond * seconds * sign, VoltageCache.currentVoltage * sign)

    override fun runOpMode() {

        val driveView = dvf.make(hardwareMap)
        val localizer = lf.make(hardwareMap)

        val angularVoltage by regressionParams::voltages
        val angularVelocity by regressionParams::velocities

        VoltageCache.init(hardwareMap)

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
                regressionParams
            )
        )
    }
}

/**
 * Dynamic SysID routine for the drivetrain.
 * Determines: kA
 * direction: +direction = counter-clockwise
 */
class AngularStepTest(val dvf: DriveViewFactory, val lf: LocalizerFactory) : LinearOpMode() {

    val direction: Double by HermesConfig.tuningConfig.angularStep::direction

    val regressionParams: DynamicParameters by HermesConfig.tuningConfig.angularStep::regressionParams

    val sign = sign(direction)

    fun voltage(seconds: Double) = if (seconds > 0.5) HermesConfig.tuningConfig.angularStep.voltageStep * sign else 0.0

    override fun runOpMode() {

        val driveView = dvf.make(hardwareMap)
        val localizer = lf.make(hardwareMap)

        val deltaVoltage by regressionParams::voltages
        val angularAcceleration by regressionParams::accelerations

        VoltageCache.init(hardwareMap)

        waitForStart()

        val t = MidpointTimer()
        var lastVel = 0.0
        var lastSeconds = 0.0
        while (opModeIsActive()) {
            val voltage = voltage(t.seconds()) // for consistency, use the same voltage throughout the entire loop

            deltaVoltage.times.add(t.addSplit())

            // u - kV * v gives voltage difference between being stable at velocity and voltage we are actually providing
            deltaVoltage.values.add((voltage - (HermesConfig.config.feedforward.rotational.kV * localizer.vel.angVel)) * sign)

            angularAcceleration.times.add(t.addSplit())
            angularAcceleration.values.add((localizer.vel.angVel - lastVel) * sign / (t.seconds() - lastSeconds))

            lastVel = localizer.vel.angVel
            lastSeconds = t.seconds()

            driveView.voltageDrive(0.0, voltage)
            localizer.update()
        }

        driveView.voltageDrive(0.0, 0.0)

        TuningFiles.save(TuningFiles.FileType.ANGULAR_STEP, regressionParams)
    }
}



object TuningOpModes {

    @JvmStatic
    @OpModeRegistrar
    fun register(manager: AnnotatedOpModeManager) {

    }

}