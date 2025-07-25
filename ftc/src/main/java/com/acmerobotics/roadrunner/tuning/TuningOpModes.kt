package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.hardware.Encoder
import com.acmerobotics.roadrunner.hardware.OTOSEncoderGroup
import com.acmerobotics.roadrunner.hardware.OTOSIMU
import com.acmerobotics.roadrunner.hardware.RollingThreeMedian
import com.acmerobotics.roadrunner.logs.TuningFiles
import com.acmerobotics.roadrunner.profiles.TimeProfile
import com.acmerobotics.roadrunner.profiles.constantProfile
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.min


class AngularRampLogger(val dvf: DriveViewFactory) : LinearOpMode() {
    companion object {
        @JvmField
        var POWER_PER_SEC = 0.1
        @JvmField
        var POWER_MAX = 0.9
    }

    fun power(seconds: Double) = min(POWER_PER_SEC * seconds, POWER_MAX)

    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        val data = object {
            val type = view.type
            val leftPowers = view.leftMotors.map { MutableSignal() }
            val rightPowers = view.rightMotors.map { MutableSignal() }
            val voltages = MutableSignal()
            val leftEncPositions = view.leftEncs.map { MutableSignal() }
            val rightEncPositions = view.rightEncs.map { MutableSignal() }
            val parEncPositions = view.parEncs.map { MutableSignal() }
            val perpEncPositions = view.perpEncs.map { MutableSignal() }
            val leftEncVels = view.leftEncs.map { MutableSignal() }
            val rightEncVels = view.rightEncs.map { MutableSignal() }
            val parEncVels = view.parEncs.map { MutableSignal() }
            val perpEncVels = view.perpEncs.map { MutableSignal() }
            val leftEncFixVels = view.leftEncs.map { shouldFixVels(view, it) }
            val rightEncFixVels = view.rightEncs.map { shouldFixVels(view, it) }
            val parEncFixVels = view.parEncs.map { shouldFixVels(view, it) }
            val perpEncFixVels = view.perpEncs.map { shouldFixVels(view, it) }
            val angVels = listOf(MutableSignal(), MutableSignal(), MutableSignal())
        }

        waitForStart()

        val t = MidpointTimer()
        while (opModeIsActive()) {
            for (i in view.leftMotors.indices) {
                val power = -power(t.seconds())
                view.leftMotors[i].power = power

                val s = data.leftPowers[i]
                s.times.add(t.addSplit())
                s.values.add(power)
            }

            for (i in view.rightMotors.indices) {
                val power = power(t.seconds())
                view.rightMotors[i].power = power

                val s = data.rightPowers[i]
                s.times.add(t.addSplit())
                s.values.add(power)
            }

            data.voltages.values.add(view.voltageSensor.voltage)
            data.voltages.times.add(t.addSplit())

            val encTimes = view.encoderGroups.map {
                it.bulkRead()
                t.addSplit()
            }

            for (i in view.leftEncs.indices) {
                recordUnwrappedEncoderData(
                    view.encoderGroups,
                    encTimes,
                    view.leftEncs[i],
                    data.leftEncPositions[i],
                    data.leftEncVels[i]
                )
            }

            for (i in view.rightEncs.indices) {
                recordUnwrappedEncoderData(
                    view.encoderGroups,
                    encTimes,
                    view.rightEncs[i],
                    data.rightEncPositions[i],
                    data.rightEncVels[i]
                )
            }

            for (i in view.parEncs.indices) {
                recordUnwrappedEncoderData(
                    view.encoderGroups,
                    encTimes,
                    view.parEncs[i],
                    data.parEncPositions[i],
                    data.parEncVels[i]
                )
            }

            for (i in view.perpEncs.indices) {
                recordUnwrappedEncoderData(
                    view.encoderGroups,
                    encTimes,
                    view.perpEncs[i],
                    data.perpEncPositions[i],
                    data.perpEncVels[i]
                )
            }

            t.addSplit()
            // Use degrees here to work around https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070
            val av = view.imu.get().getRobotAngularVelocity(AngleUnit.DEGREES)
            val time = t.addSplit()

            data.angVels[0].times.add(time)
            data.angVels[1].times.add(time)
            data.angVels[2].times.add(time)

            data.angVels[0].values.add(Math.toRadians(av.xRotationRate.toDouble()))
            data.angVels[1].values.add(Math.toRadians(av.yRotationRate.toDouble()))
            data.angVels[2].values.add(Math.toRadians(av.zRotationRate.toDouble()))
        }

        for (m in view.motors) {
            m.power = 0.0
        }

        TuningFiles.save(TuningFiles.FileType.ANGULAR_RAMP, data)
    }
}

class ForwardPushTest(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val view = dvf.make(hardwareMap)

        for (m in view.motors) {
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }

        waitForStart()

        val es = view.forwardEncs.map { view.encoder(it) }
        val initAvgPos = avgPos(es)
        while (opModeIsActive()) {
            for (g in view.encoderGroups) {
                g.bulkRead()
            }

            telemetry.addData("ticks traveled", avgPos(es) - initAvgPos)
            telemetry.update()
        }
    }
}

class ForwardRampLogger(val dvf: DriveViewFactory) : LinearOpMode() {
    companion object {
        @JvmField
        var POWER_PER_SEC = 0.1
        @JvmField
        var POWER_MAX = 0.9
    }

    fun power(seconds: Double) = min(POWER_PER_SEC * seconds, POWER_MAX)

    override fun runOpMode() {
        val view = dvf.make(hardwareMap)
        require(view.perpEncs.isNotEmpty()) {
            "Only run this op mode if you're using dead wheels."
        }

        val data = object {
            val type = view.type
            val powers = view.motors.map { MutableSignal() }
            val voltages = MutableSignal()
            val forwardEncPositions = view.forwardEncs.map { MutableSignal() }
            val forwardEncVels = view.forwardEncs.map { MutableSignal() }
            val forwardEncFixVels = view.forwardEncs.map { shouldFixVels(view, it) }
        }

        waitForStart()

        val t = MidpointTimer()
        while (opModeIsActive()) {
            for (i in view.motors.indices) {
                val power = power(t.seconds())
                view.motors[i].power = power

                val s = data.powers[i]
                s.times.add(t.addSplit())
                s.values.add(power)
            }

            data.voltages.values.add(view.voltageSensor.voltage)
            data.voltages.times.add(t.addSplit())

            val encTimes = view.encoderGroups.map {
                it.bulkRead()
                t.addSplit()
            }

            for (i in view.forwardEncs.indices) {
                recordUnwrappedEncoderData(
                    view.encoderGroups,
                    encTimes,
                    view.forwardEncs[i],
                    data.forwardEncPositions[i],
                    data.forwardEncVels[i]
                )
            }
        }

        for (m in view.motors) {
            m.power = 0.0
        }

        TuningFiles.save(TuningFiles.FileType.FORWARD_RAMP, data)
    }
}

class LateralRampLogger(val dvf: DriveViewFactory) : LinearOpMode() {
    companion object {
        @JvmField
        var POWER_PER_SEC = 0.1
        @JvmField
        var POWER_MAX = 0.9
    }

    fun power(seconds: Double) = min(POWER_PER_SEC * seconds, POWER_MAX)

    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        require(view.type == DriveType.MECANUM) {
            "Only mecanum drives should run this op mode."
        }

        val data = object {
            val type = view.type
            val frontLeftPower = MutableSignal()
            val backLeftPower = MutableSignal()
            val frontRightPower = MutableSignal()
            val backRightPower = MutableSignal()
            val voltages = MutableSignal()
            val perpEncPositions = view.perpEncs.map { MutableSignal() }
            val perpEncVels = view.perpEncs.map { MutableSignal() }
            val perpEncFixVels = view.perpEncs.map { shouldFixVels(view, it) }
        }

        waitForStart()

        val t = MidpointTimer()

        fun setMotorPower(m: DcMotorEx, sign: Int, signal: MutableSignal) {
            val power = sign * power(t.seconds())
            m.power = power

            signal.times.add(t.addSplit())
            signal.values.add(power)
        }

        while (opModeIsActive()) {
            setMotorPower(view.leftMotors[0], -1, data.frontLeftPower)
            setMotorPower(view.rightMotors[0], +1, data.frontRightPower)
            setMotorPower(view.leftMotors[1], +1, data.backLeftPower)
            setMotorPower(view.rightMotors[1], -1, data.backRightPower)

            data.voltages.values.add(view.voltageSensor.voltage)
            data.voltages.times.add(t.addSplit())

            val encTimes = view.encoderGroups.map {
                it.bulkRead()
                t.addSplit()
            }

            for (i in view.perpEncs.indices) {
                recordUnwrappedEncoderData(
                    view.encoderGroups,
                    encTimes,
                    view.perpEncs[i],
                    data.perpEncPositions[i],
                    data.perpEncVels[i]
                )
            }
        }

        for (m in view.motors) {
            m.power = 0.0
        }

        TuningFiles.save(TuningFiles.FileType.LATERAL_RAMP, data)
    }
}

fun lateralSum(view: DriveView): Double {
    return 0.25 * (
            -view.encoder(view.leftEncs[0]).getPositionAndVelocity().position
                    +view.encoder(view.leftEncs[1]).getPositionAndVelocity().position
                    -view.encoder(view.rightEncs[1]).getPositionAndVelocity().position
                    +view.encoder(view.rightEncs[0]).getPositionAndVelocity().position)
}

class LateralPushTest(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val view = dvf.make(hardwareMap)

        require(view.type == DriveType.MECANUM) {
            "Only mecanum drives should run this op mode."
        }
        require(view.parEncs.isEmpty() && view.perpEncs.isEmpty()) {
            "Do not run this op mode if using dead wheels."
        }

        for (m in view.motors) {
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }

        waitForStart()

        val initLateralSum = lateralSum(view)
        while (opModeIsActive()) {
            for (g in view.encoderGroups) {
                g.bulkRead()
            }

            telemetry.addData("ticks traveled", lateralSum(view) - initLateralSum)
            telemetry.update()
        }
    }
}

class ManualFeedforwardTuner(val dvf: DriveViewFactory) : LinearOpMode() {
    companion object {
        @JvmField
        var DISTANCE = 64.0
    }

    enum class Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val view = dvf.make(hardwareMap)
        val profile = TimeProfile(constantProfile(
            DISTANCE, 0.0, view.maxVel, view.minAccel, view.maxAccel).baseProfile)

        var mode = Mode.TUNING_MODE

        telemetry.addLine("Ready!")
        telemetry.update()
        telemetry.clearAll()

        waitForStart()

        if (isStopRequested) return

        var movingForwards = true
        var startTs = System.nanoTime() / 1e9

        val lastPositions = MutableList(view.forwardEncs.size) { 0 }
        val lastTimes = view.forwardEncs.map { ElapsedTime() }
        val velEstimates = view.forwardEncs.map { RollingThreeMedian() }
        while (!isStopRequested) {
            telemetry.addData("mode", mode)

            when (mode) {
                Mode.TUNING_MODE -> {
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE
                    }

                    for (g in view.encoderGroups) {
                        g.bulkRead()
                    }

                    for (i in view.forwardEncs.indices) {
                        val ref = view.forwardEncs[i]

                        val pv = view.encoder(ref).getPositionAndVelocity()
                        val v = if (pv.velocity == null) {
                            val lastPos = lastPositions[i]
                            val currVelocity = velEstimates[i].update((pv.position - lastPos) / lastTimes[i].seconds())
                            lastPositions[i] = pv.position
                            lastTimes[i].reset()
                            currVelocity
                        } else {
                            pv.velocity.toDouble()
                        }

                        telemetry.addData("v$i", view.inPerTick * v)
                    }

                    val ts = System.nanoTime() / 1e9
                    val t = ts - startTs
                    if (t > profile.duration) {
                        movingForwards = !movingForwards
                        startTs = ts
                    }

                    var v = profile[t].drop(1)
                    if (!movingForwards) {
                        v = v.unaryMinus()
                    }
                    telemetry.addData("vref", v[0])

                    val power = view.feedforwardFactory.make().compute(v) / view.voltageSensor.voltage
                    view.setDrivePowers(PoseVelocity2d(Vector2d(power, 0.0), 0.0))
                }
                Mode.DRIVER_MODE -> {
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE
                        movingForwards = true
                        startTs = System.nanoTime() / 1e9
                    }

                    view.setDrivePowers(PoseVelocity2d(
                        Vector2d(
                            -gamepad1.left_stick_y.toDouble(),
                            -gamepad1.left_stick_x.toDouble()
                        ),
                        -gamepad1.right_stick_x.toDouble()
                    ))
                }
            }

            telemetry.update()
        }
    }
}

class MecanumMotorDirectionDebugger(val dvf: DriveViewFactory) : LinearOpMode() {
    companion object {
        @JvmField
        var MOTOR_POWER = 0.7
    }

    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        require(view.type == DriveType.MECANUM) {
            "Only mecanum drives should run this op mode."
        }

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        telemetry.addLine("Press play to begin the debugging op mode")
        telemetry.update()

        waitForStart()

        if (isStopRequested) return

        telemetry.clearAll()
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)

        while (opModeIsActive()) {
            telemetry.addLine("Press each button to turn on its respective motor")
            telemetry.addLine()
            telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>")
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>")
            telemetry.addLine()

            val hasDriveEncoders = view.leftEncs.isNotEmpty() && view.rightEncs.isNotEmpty()

            for (g in view.encoderGroups) {
                g.bulkRead()
            }

            if (gamepad1.x) {
                view.leftMotors[0].power = MOTOR_POWER
                telemetry.addLine("Running Motor: Front Left")
                if (hasDriveEncoders) {
                    val pv = view.encoder(view.leftEncs[0]).getPositionAndVelocity()
                    telemetry.addLine("Encoder Position: " + pv.position)
                    telemetry.addLine("Encoder Velocity: " + pv.velocity)
                }
            } else if (gamepad1.y) {
                view.rightMotors[0].power = MOTOR_POWER
                telemetry.addLine("Running Motor: Front Right")
                if (hasDriveEncoders) {
                    val pv = view.encoder(view.rightEncs[0]).getPositionAndVelocity()
                    telemetry.addLine("Encoder Position: " + pv.position)
                    telemetry.addLine("Encoder Velocity: " + pv.velocity)
                }
            } else if (gamepad1.b) {
                view.rightMotors[1].power = MOTOR_POWER
                telemetry.addLine("Running Motor: Rear Right")
                if (hasDriveEncoders) {
                    val pv = view.encoder(view.rightEncs[1]).getPositionAndVelocity()
                    telemetry.addLine("Encoder Position: " + pv.position)
                    telemetry.addLine("Encoder Velocity: " + pv.velocity)
                }
            } else if (gamepad1.a) {
                view.leftMotors[1].power = MOTOR_POWER
                telemetry.addLine("Running Motor: Rear Left")
                if (hasDriveEncoders) {
                    val pv = view.encoder(view.leftEncs[1]).getPositionAndVelocity()
                    telemetry.addLine("Encoder Position: " + pv.position)
                    telemetry.addLine("Encoder Velocity: " + pv.velocity)
                }
            } else {
                for (m in view.motors) {
                    m.power = 0.0
                }
                telemetry.addLine("Running Motor: None")
            }

            telemetry.update()
        }
    }
}

class DeadWheelDirectionDebugger(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        require(view.parEncs.isNotEmpty() && view.perpEncs.isNotEmpty()) {
            "Only run this op mode if you're using dead wheels."
        }

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()

        if (isStopRequested) return

        while (opModeIsActive()) {
            telemetry.addLine("Move each dead wheel individually and make sure the direction is correct")
            telemetry.addLine()

            for (g in view.encoderGroups) {
                g.bulkRead()
            }

            telemetry.addLine("Parallel Dead Wheels (should increase forward)")
            for (i in view.parEncs.indices) {
                telemetry.addLine("  Wheel $i Position: ${view.encoder(view.parEncs[i]).getPositionAndVelocity().position}")
            }
            telemetry.addLine()

            telemetry.addLine("Perpendicular Dead Wheels (should increase leftward)")
            for (i in view.perpEncs.indices) {
                telemetry.addLine("  Wheel $i Position: ${view.encoder(view.perpEncs[i]).getPositionAndVelocity().position}")
            }

            telemetry.update()
        }
    }
}

const val OTOS_ERROR_MSG =
    """
    Only run this OpMode if you are using a Sparkfun OTOS.
    This OpMode requires OTOS to be properly configured. 
    See Tuning docs for details.
    """

/* Originally written by j5155; ported to Kotlin by zach.waffle */
class OTOSAngularScalarTuner(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        require(view.imu is OTOSIMU) { OTOS_ERROR_MSG }

        val imu = view.imu.get()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        var radsTurned = 0.0
        var lastHeading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

        telemetry.addLine("OTOS Angular Scalar Tuner")
        telemetry.addLine("Press START, then rotate the robot on the ground 10 times (3600 degrees).")
        telemetry.update()
        waitForStart()
        while (opModeIsActive()) {
            val otosHeading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

            radsTurned += (Rotation2d.exp(otosHeading) - Rotation2d.exp(lastHeading))
            lastHeading = otosHeading

            telemetry.addData("OTOS Heading (radians)", otosHeading)
            telemetry.addData("Uncorrected Degrees Turned", Math.toDegrees(radsTurned))
            telemetry.addData("Calculated Angular Scalar", 3600 / Math.toDegrees(radsTurned))
            telemetry.update()
        }
    }
}

class OTOSLinearScalarTuner(val dvf: DriveViewFactory) : LinearOpMode() {
    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        require(view.encoderGroups.first() is OTOSEncoderGroup) { OTOS_ERROR_MSG }

        val parallelEnc = view.encoderGroups.first().encoders.first()
        val perpEnc = view.encoderGroups.first().encoders.last()

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        telemetry.addLine("OTOS Linear Scalar Tuner")
        telemetry.addLine("Press START, then push the robot a known distance.")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val xPos = parallelEnc.getPositionAndVelocity().position * view.inPerTick
            val yPos = perpEnc.getPositionAndVelocity().position * view.inPerTick

            telemetry.addData("Uncorrected Distance Traveled Y", xPos)
            telemetry.addData("Uncorrected Distance Traveled X", yPos)
            telemetry.update()
        }
    }
}

/* Originally written by j5155; ported to Kotlin by zach.waffle */
class OTOSHeadingOffsetTuner(val dvf: DriveViewFactory) : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        require(view.encoderGroups.first() is OTOSEncoderGroup) { OTOS_ERROR_MSG }

        val parallelEnc = view.encoderGroups.first().encoders.first()
        val perpEnc = view.encoderGroups.first().encoders.last()

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        telemetry.addLine("OTOS Heading Offset Tuner")
        telemetry.addLine("Line the side of the robot against a wall and Press START.")
        telemetry.addLine("Then push the robot forward some distance.")
        telemetry.update()

        waitForStart()
        while (opModeIsActive()) {
            val xPos = parallelEnc.getPositionAndVelocity().position * view.inPerTick
            val yPos = perpEnc.getPositionAndVelocity().position * view.inPerTick

            telemetry.addData("Heading Offset (radians, enter this one into OTOSLocalizer!)", atan2(yPos, xPos))
            telemetry.addData("Heading Offset (degrees)", Math.toDegrees(atan2(yPos, xPos)))
            telemetry.update()
        }
    }
}

/* Originally written by j5155; ported to Kotlin by zach.waffle */
class OTOSPositionOffsetTuner(val dvf: DriveViewFactory) : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val view = dvf.make(hardwareMap)

        require(view.imu is OTOSIMU) { OTOS_ERROR_MSG }
        require(view.encoderGroups.first() is OTOSEncoderGroup) { OTOS_ERROR_MSG }

        val parallelEnc = view.encoderGroups.first().encoders.first()
        val perpEnc = view.encoderGroups.first().encoders.last()

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        telemetry.addLine("OTOS Position Offset Tuner")
        telemetry.addLine("Line the robot against the corner of two walls facing forward and Press START.")
        telemetry.addLine("Then rotate the robot exactly 180 degrees and press it back into the corner.")
        telemetry.addLine("Finally, copy the pose offset into OTOSLocalizer.")
        telemetry.update()
        waitForStart()
        while (opModeIsActive()) {
            val heading = view.imu.get().robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
            val xPos = parallelEnc.getPositionAndVelocity().position * view.inPerTick
            val yPos = perpEnc.getPositionAndVelocity().position * view.inPerTick

            telemetry.addData("Heading (deg)", Math.toDegrees(heading))
            if (abs(Math.toDegrees(heading)) > 175) {
                telemetry.addData("X Offset", xPos / 2)
                telemetry.addData("Y Offset", yPos / 2)
            } else {
                telemetry.addLine("Rotate the robot 180 degrees and align it to the corner again.")
            }
            telemetry.update()
        }
    }
}