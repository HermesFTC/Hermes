package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.DualNum
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Rotation2dDual
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.Vector2dDual
import io.kotest.matchers.Matcher
import io.kotest.matchers.MatcherResult

fun <P> dualEqual(other: DualNum<P>) = Matcher<DualNum<P>> { value ->
    MatcherResult(
        value.values().size == other.values().size &&
                (value.values().zip(other.values()).all { (a, b) -> a == b }),
        { "DualNum should have had values ${other.values()} but had ${value.values()}" },
        { "DualNum should not have had values ${other.values()}" }
    )}

fun vectorEqual(other: Vector2d) = Matcher<Vector2d> { value -> 
    MatcherResult(
        value.x == other.x && value.y == other.y,
        { "Vector2d should have had x ${other.x} and y ${other.y}, but had x ${value.x} and y ${value.y}" },
        { "Vector2d should not have had x ${other.x} and y ${other.y}" }
    )
}

fun rotationEqual(other: Rotation2d) = Matcher<Rotation2d> { value ->
    MatcherResult(
        value.real == other.real && value.imag == other.imag,
        { "Rotation2d should have had real ${other.real} and imag ${other.imag}, but had real ${value.real} and imag ${value.imag}" },
        { "Rotation2d should not have had real ${other.real} and imag ${other.imag}" }
    )
}

fun poseEqual(other: Pose2d) = Matcher<Pose2d> { value ->
    MatcherResult(
        value.position.x == other.position.x && 
        value.position.y == other.position.y && 
        value.heading.real == other.heading.real && 
        value.heading.imag == other.heading.imag,
        { "Pose2d should have had position (${other.position.x}, ${other.position.y}) and heading (${other.heading.real}, ${other.heading.imag}), " +
          "but had position (${value.position.x}, ${value.position.y}) and heading (${value.heading.real}, ${value.heading.imag})" },
        { "Pose2d should not have had position (${other.position.x}, ${other.position.y}) and heading (${other.heading.real}, ${other.heading.imag})" }
    )
}

fun <P> dualVectorEqual(other: Vector2dDual<P>) = Matcher<Vector2dDual<P>> { value ->
    MatcherResult(
        value.x.values().size == other.x.values().size &&
        value.y.values().size == other.y.values().size &&
        value.x.values().zip(other.x.values()).all { (a, b) -> a == b } &&
        value.y.values().zip(other.y.values()).all { (a, b) -> a == b },
        { "Vector2dDual should have had x ${other.x.values()} and y ${other.y.values()}, but had x ${value.x.values()} and y ${value.y.values()}" },
        { "Vector2dDual should not have had x ${other.x.values()} and y ${other.y.values()}" }
    )
}

fun <P> dualRotationEqual(other: Rotation2dDual<P>) = Matcher<Rotation2dDual<P>> { value ->
    MatcherResult(
        value.real.values().size == other.real.values().size &&
        value.imag.values().size == other.imag.values().size &&
        value.real.values().zip(other.real.values()).all { (a, b) -> a == b } &&
        value.imag.values().zip(other.imag.values()).all { (a, b) -> a == b },
        { "Rotation2dDual should have had real ${other.real.values()} and imag ${other.imag.values()}, but had real ${value.real.values()} and imag ${value.imag.values()}" },
        { "Rotation2dDual should not have had real ${other.real.values()} and imag ${other.imag.values()}" }
    )
}

fun <P> dualPoseEqual(other: Pose2dDual<P>) = Matcher<Pose2dDual<P>> { value ->
    MatcherResult(
        value.position.x.values().size == other.position.x.values().size &&
        value.position.y.values().size == other.position.y.values().size &&
        value.heading.real.values().size == other.heading.real.values().size &&
        value.heading.imag.values().size == other.heading.imag.values().size &&
        value.position.x.values().zip(other.position.x.values()).all { (a, b) -> a == b } &&
        value.position.y.values().zip(other.position.y.values()).all { (a, b) -> a == b } &&
        value.heading.real.values().zip(other.heading.real.values()).all { (a, b) -> a == b } &&
        value.heading.imag.values().zip(other.heading.imag.values()).all { (a, b) -> a == b },
        { "Pose2dDual should have had position.x ${other.position.x.values()}, position.y ${other.position.y.values()}, " +
          "heading.real ${other.heading.real.values()}, and heading.imag ${other.heading.imag.values()}, " +
          "but had position.x ${value.position.x.values()}, position.y ${value.position.y.values()}, " +
          "heading.real ${value.heading.real.values()}, and heading.imag ${value.heading.imag.values()}" },
        { "Pose2dDual should not have had position.x ${other.position.x.values()}, position.y ${other.position.y.values()}, " +
          "heading.real ${other.heading.real.values()}, and heading.imag ${other.heading.imag.values()}" }
    )
}
