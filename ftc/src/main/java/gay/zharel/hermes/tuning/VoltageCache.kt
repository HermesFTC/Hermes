/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.tuning

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime

/**
 * Global object for synchronized and IO-efficient voltage reads.
 */
object VoltageCache {

    private lateinit var voltageSensor: VoltageSensor
    private var cachedVoltage: Double = 0.0
    private val timeElapsed: ElapsedTime = ElapsedTime()

    private val voltageConfig: VoltageConfig by PersistentConfigDelegate(
        "Voltage Configuration", VoltageConfig(),
        HermesConfig,
    )

    val nominalVoltage: Double by voltageConfig::nominalVoltage

    private val readIntervalSeconds: Double by voltageConfig::readIntervalSeconds

    val currentVoltage: Double
        get() {
            if (timeElapsed.seconds() > readIntervalSeconds) {
                cachedVoltage = voltageSensor.voltage
                timeElapsed.reset()
            }
            return cachedVoltage
        }

    /**
     * Initializes the VoltageCache with the voltage sensor for the OpMode.
     */
    fun init(hardwareMap: HardwareMap) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next()
        cachedVoltage = voltageSensor.voltage
        timeElapsed.reset()
    }

}