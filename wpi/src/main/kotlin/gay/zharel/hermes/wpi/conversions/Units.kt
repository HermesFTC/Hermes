package gay.zharel.hermes.wpi.conversions

import org.wpilib.units.Measure
import org.wpilib.units.Unit
import org.wpilib.units.Units.Inches
import org.wpilib.units.Units.InchesPerSecond
import org.wpilib.units.Units.Meters
import org.wpilib.units.Units.MetersPerSecond
import org.wpilib.units.Units.Radians
import org.wpilib.units.Units.RadiansPerSecond
import org.wpilib.units.Units.Seconds
import org.wpilib.units.Units.Volts
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Distance
import org.wpilib.units.measure.LinearVelocity
import org.wpilib.units.measure.Time
import org.wpilib.units.measure.Voltage
import kotlin.time.Duration
import kotlin.time.DurationUnit

fun <U : Unit> Measure<U>.into(unit: U) = this.`in`(unit)

inline val Double.inches: Distance get() = Inches.of(this)
inline val Double.ips: LinearVelocity get() = InchesPerSecond.of(this)
inline val Double.meters: Distance get() = Meters.of(this)
inline val Double.mps: LinearVelocity get() = MetersPerSecond.of(this)
inline val Double.radians: Angle get() = Radians.of(this)
inline val Double.radps: AngularVelocity get() = RadiansPerSecond.of(this)
inline val Double.volts: Voltage get() = Volts.of(this)

inline val Distance.inches get() = this.into(Inches)
inline val LinearVelocity.ips get() = this.into(InchesPerSecond)
inline val Distance.meters get() = this.into(Meters)
inline val LinearVelocity.mps get() = this.into(MetersPerSecond)
inline val Angle.radians get() = this.into(Radians)
inline val AngularVelocity.radps get() = this.into(RadiansPerSecond)
inline val Voltage.volts get() = this.into(Volts)

inline val Double.seconds: Time get() = Seconds.of(this)
inline val Duration.seconds: Time get() = Seconds.of(toDouble(DurationUnit.SECONDS))
