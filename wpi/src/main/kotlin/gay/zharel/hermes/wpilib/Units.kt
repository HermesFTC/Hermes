package gay.zharel.hermes.wpilib

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.InchesPerSecond
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import kotlin.time.Duration
import kotlin.time.DurationUnit

fun <U : Unit> Measure<U>.into(unit: U) = this.`in`(unit)

inline val Double.inches: Distance get() = Inches.of(this)
inline val Double.ips: LinearVelocity get() = InchesPerSecond.of(this)
inline val Double.meters: Distance get() = Meters.of(this)
inline val Double.mps: LinearVelocity get() = MetersPerSecond.of(this)
inline val Double.radians: Angle get() = Radians.of(this)
inline val Double.radps: AngularVelocity get() = RadiansPerSecond.of(this)

inline val Distance.inches get() = this.into(Inches)
inline val LinearVelocity.ips get() = this.into(InchesPerSecond)
inline val Distance.meters get() = this.into(Meters)
inline val LinearVelocity.mps get() = this.into(MetersPerSecond)
inline val Angle.radians get() = this.into(Radians)
inline val AngularVelocity.radps get() = this.into(RadiansPerSecond)

inline val Double.seconds: Time get() = Seconds.of(this)
inline val Duration.seconds: Time get() = Seconds.of(toDouble(DurationUnit.SECONDS))