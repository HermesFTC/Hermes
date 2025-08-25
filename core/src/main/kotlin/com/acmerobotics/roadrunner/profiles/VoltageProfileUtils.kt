package com.acmerobotics.roadrunner.profiles

/**
 * Utility functions for converting between voltage, velocity, and acceleration in motor models.
 * These functions implement the basic motor model: V = kV * v + kA * a + kS
 */

/**
 * Converts available voltage to velocity using the velocity constant kV.
 * @param kV Unit of velocity per unit of voltage
 * @return Function that converts voltage to velocity
 */
fun velocityFromVoltage(kV: Double): (Double) -> Double {
    return { availableVoltage: Double -> availableVoltage * kV }
}

/**
 * Converts velocity to required voltage using the velocity constant kV.
 * @param kV Unit of velocity per unit of voltage
 * @return Function that converts velocity to voltage
 */
fun voltageFromVelocity(kV: Double): (Double) -> Double {
    return { velocity: Double -> velocity / kV }
}

/**
 * Converts available voltage to acceleration using the acceleration constant kA.
 * @param kA Unit of acceleration per unit of voltage
 * @return Function that converts voltage to acceleration
 */
fun accelerationFromVoltage(kA: Double): (Double) -> Double {
    return { availableVoltage: Double -> availableVoltage * kA }
}

/**
 * Converts acceleration to required voltage using the acceleration constant kA.
 * @param kA Unit of acceleration per unit of voltage
 * @return Function that converts acceleration to voltage
 */
fun voltageFromAcceleration(kA: Double): (Double) -> Double {
    return { acceleration: Double -> acceleration / kA }
}

/**
 * Calculates the maximum velocity achievable with the given voltage and motor constants.
 * @param availableVoltage The voltage available for velocity (after accounting for kS)
 * @param kV Velocity constant
 * @return Maximum achievable velocity
 */
fun maxVelocityFromVoltage(availableVoltage: Double, kV: Double): Double {
    return availableVoltage * kV
}

/**
 * Calculates the maximum acceleration achievable with the given voltage and motor constants.
 * @param availableVoltage The voltage available for acceleration
 * @param velocityVoltage The voltage already being used for maintaining velocity
 * @param kA Acceleration constant
 * @return Maximum achievable acceleration
 */
fun maxAccelerationFromVoltage(
    availableVoltage: Double,
    velocityVoltage: Double,
    kA: Double
): Double {
    return (availableVoltage - velocityVoltage) * kA
}
