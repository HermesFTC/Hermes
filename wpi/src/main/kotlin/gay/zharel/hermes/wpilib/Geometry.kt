package gay.zharel.hermes.wpilib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisAccelerations
import edu.wpi.first.math.kinematics.ChassisSpeeds
import gay.zharel.hermes.geometry.Acceleration2d
import gay.zharel.hermes.geometry.PoseVelocity2d
import gay.zharel.hermes.geometry.Vector2d


@get:JvmName("wpilib")
inline val Vector2d.wpilib get() = Translation2d(x.inches, y.inches)
@get:JvmName("hermes")
inline val Translation2d.hermes get() = Vector2d(x.meters.inches, y.meters.inches)

@get:JvmName("wpilib")
inline val HRotation2d.wpilib get() = Rotation2d(real, imag)
@get:JvmName("hermes")
inline val Rotation2d.hermes get() = HRotation2d(cos, sin)

@get:JvmName("wpilib")
inline val HPose2d.wpilib get() = Pose2d(position.wpilib, heading.wpilib)
@get:JvmName("hermes")
inline val Pose2d.hermes get() = HPose2d(translation.hermes, rotation.hermes)

@get:JvmName("wpilib")
inline val PoseVelocity2d.wpilib get() = ChassisSpeeds(linearVel.x.ips, linearVel.y.ips, angVel.radps)
@get:JvmName("hermes")
inline val ChassisSpeeds.hermes get() = PoseVelocity2d(Vector2d(vx.mps.ips, vy.mps.ips), omega)

@get:JvmName("wpilib")
inline val Acceleration2d.wpilib get() = ChassisAccelerations(linearAcc.x.inches.meters, linearAcc.y.inches.meters, angAcc)
@get:JvmName("hermes")
inline val ChassisAccelerations.hermes get() = Acceleration2d(Vector2d(ax.meters.inches, ay.meters.inches), alpha)