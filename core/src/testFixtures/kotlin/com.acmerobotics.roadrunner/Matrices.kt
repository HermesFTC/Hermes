package com.acmerobotics.roadrunner

import io.kotest.matchers.Matcher
import io.kotest.matchers.MatcherResult
import io.kotest.matchers.doubles.ToleranceMatcher
import org.ejml.simple.SimpleMatrix

operator fun ToleranceMatcher.contains(value: Double): Boolean {
    return this.test(value).passed()
}
fun SimpleMatrix.hasNaN() = toArray2().any { row -> row.any { it.isNaN() } }
fun SimpleMatrix.hasInfinite() = toArray2().any { row -> row.any { it.isInfinite() } }

fun SimpleMatrix.isSymmetric() =
    this.toArray2().contentDeepEquals(this.transpose().toArray2())

fun beSymmetric() = Matcher<SimpleMatrix> { value ->
    MatcherResult(
        value.isSymmetric(),
        { "Matrix should be symmetric" },
        { "Matrix should not be symmetric" }
    )
}