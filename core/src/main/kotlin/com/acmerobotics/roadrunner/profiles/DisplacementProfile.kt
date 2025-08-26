package com.acmerobotics.roadrunner.profiles

import com.acmerobotics.roadrunner.geometry.DualNum
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.paths.PosePath
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlin.math.sqrt

/**
 * Acceleration-limited motion profile parameterized by displacement.
 *
 * This class represents a piecewise profile where:
 * - [disps] contains displacement waypoints, beginning at zero and sorted ascending
 * - [vels] contains velocities at each displacement waypoint
 * - [accels] contains constant accelerations applied over each displacement interval
 *
 * The profile can be queried at any displacement to get velocity and acceleration values.
 */
@Serializable
@SerialName("DisplacementProfile")
data class DisplacementProfile(
    @JvmField
    val disps: List<Double>,
    @JvmField
    val vels: List<Double>,
    @JvmField
    val accels: List<Double>,
) : Profile {
    @JvmField
    val length = disps.last()

    init {
        validateProfileData()
    }

    /**
     * Validates that the profile data is consistent and well-formed.
     */
    private fun validateProfileData() {
        require(disps.size == vels.size) {
            "disps.size() (${disps.size}) != vels.size() (${vels.size})"
        }
        require(disps.size == accels.size + 1) {
            "disps.size() (${disps.size}) != accels.size() + 1 (${accels.size + 1})"
        }
    }

    /**
     * Evaluates the profile at the given displacement, returning position, velocity, and acceleration.
     * Uses binary search to efficiently find the correct interval and interpolates as needed.
     */
    override operator fun get(x: Double): DualNum<Time> {
        val index = disps.binarySearch(x)

        return when {
            index >= disps.lastIndex -> createDualNumAtEnd(x, index)
            index >= 0 -> createDualNumAtKnownPoint(x, index)
            else -> createDualNumByInterpolation(x, index)
        }
    }

    /**
     * Creates a DualNum for displacements at or beyond the end of the profile.
     */
    private fun createDualNumAtEnd(x: Double, index: Int): DualNum<Time> {
        return DualNum(doubleArrayOf(x, vels[index], 0.0))
    }

    /**
     * Creates a DualNum for displacements exactly at a known waypoint.
     */
    private fun createDualNumAtKnownPoint(x: Double, index: Int): DualNum<Time> {
        return DualNum(doubleArrayOf(x, vels[index], accels[index]))
    }

    /**
     * Creates a DualNum by interpolating between waypoints using kinematic equations.
     */
    private fun createDualNumByInterpolation(x: Double, negativeIndex: Int): DualNum<Time> {
        val insertionIndex = -(negativeIndex + 1)

        return when {
            insertionIndex <= 0 -> DualNum(doubleArrayOf(x, vels.first(), 0.0))
            insertionIndex >= disps.size -> DualNum(doubleArrayOf(x, vels.last(), 0.0))
            else -> interpolateVelocityAndAcceleration(x, insertionIndex)
        }
    }

    /**
     * Interpolates velocity using kinematic equations: v² = v₀² + 2aΔx
     */
    private fun interpolateVelocityAndAcceleration(x: Double, insertionIndex: Int): DualNum<Time> {
        val dx = x - disps[insertionIndex - 1]
        val v0 = vels[insertionIndex - 1]
        val a = accels[insertionIndex - 1]

        val interpolatedVelocity = sqrt(v0 * v0 + 2 * a * dx)

        return DualNum(doubleArrayOf(x, interpolatedVelocity, a))
    }

    companion object {
        /**
         * Creates an optimal displacement profile using forward and backward passes.
         * This is the main entry point for generating time-optimal profiles.
         *
         * @param params Profile generation parameters
         * @param path The path to follow
         * @param beginVel Beginning and ending velocity (must be the same to guarantee feasibility)
         * @param velConstraint Velocity constraint function
         * @param accelConstraint Acceleration constraint function
         * @return DisplacementProfile optimized for the given path and constraints
         */
        fun generate(
            params: ProfileParams,
            path: PosePath,
            beginVel: Double,
            velConstraint: VelConstraint,
            accelConstraint: AccelConstraint,
        ): DisplacementProfile = createOptimalDisplacementProfile(
            params, path, beginVel, velConstraint, accelConstraint
        )

        /**
         * Creates a simple profile with uniform constraints.
         */
        fun createSimple(
            length: Double,
            beginVel: Double,
            maxVel: (Double) -> Double,
            maxAccel: (Double) -> Double,
            resolution: Double,
        ): DisplacementProfile = createSimpleDisplacementProfile(
            length, beginVel, maxVel, maxAccel, resolution
        )
    }
}

/**
 * Combines two displacement profiles end-to-end.
 * Requires that the ending velocity of the first profile matches the beginning velocity of the second.
 */
operator fun DisplacementProfile.plus(other: DisplacementProfile): DisplacementProfile {
    require(this.vels.last() == other.vels.first()) {
        "Velocity mismatch: this.vels.last() (${this.vels.last()}) != other.vels.first() (${other.vels.first()})"
    }

    return DisplacementProfile(
        this.disps + other.disps.drop(1).map { it + this.length },
        this.vels + other.vels.drop(1),
        this.accels + other.accels
    )
}
