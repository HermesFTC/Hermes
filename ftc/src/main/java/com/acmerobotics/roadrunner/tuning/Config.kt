package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.ValueProvider
import com.acmerobotics.dashboard.config.variable.ConfigVariable
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.roadrunner.serialization.HermesJsonFormat
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.json.decodeFromStream
import kotlinx.serialization.json.encodeToStream
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.File
import kotlin.reflect.KProperty

object HermesConfig : PersistentConfig("HermesConfig", AppUtil.ROOT_FOLDER.resolve("hermes/config/config.json")) {

    var config: RobotConfig by PersistentConfigDelegate("RobotConfig", RobotConfig(null, null, null))

    var tuningConfig: TuningConfig by PersistentConfigDelegate("TuningConfig", TuningConfig())

}

open class PersistentConfig(val configName: String, configFile: File) {

    private val configReader: PersistentConfigReader = PersistentConfigReader(configFile)
    private val configWriter: PersistentConfigWriter = PersistentConfigWriter(configFile)

    private var configVariables: MutableMap<String, ValueProvider<Any?>> = mutableMapOf()

    @Suppress("UNCHECKED_CAST")
    fun <T> addConfigVariable(name: String, value: T?) {
        val provider: ValueProvider<T?> = PersistentConfigValueProvider(value, this)

        // validate that the config variable doesn't exist
        if (configVariables.containsKey(name)) {
            // if it does, take out the old one and put in the new one
            configVariables[name] = provider as ValueProvider<Any?>
            FtcDashboard.getInstance().removeConfigVariable(configName, name)
            FtcDashboard.getInstance().addConfigVariable(configName, name, provider)
            return
        }

        FtcDashboard.getInstance().addConfigVariable(configName, name, provider)
        configVariables[name] = provider as ValueProvider<Any?>

    }

    operator fun get(key: String): Any? {
        return configVariables[key]?.get()
    }

    fun getProvider(key: String): ValueProvider<Any?>? {
        return configVariables[key]
    }

    operator fun <T: Any?> set(key: String, value: T): T? {
        val provider = getProvider(key)
        if (provider == null) {
            addConfigVariable(key, value)
            return value
        }

        try {
            provider.set(value)
        } catch (e: ClassCastException) {
            return null
        }

        return value
    }

    fun updateConfig() {
        configWriter.writeConfig(this)
    }

    fun loadConfig() {
        val other = configReader.readConfig() // necessary for the ide to not crash out

        this.configVariables = other.configVariables
    }

}

class PersistentConfigValueProvider<T: Any?>(initialValue: T, val config: PersistentConfig) : ValueProvider<T> {
    private var value: T? = initialValue

    override fun get(): T? {
        return value
    }

    override fun set(value: T?) {
        this.value = value
        config.updateConfig()
    }

}

open class PersistentConfigWriter(val configFile: File) {

    @OptIn(ExperimentalSerializationApi::class)
    fun writeConfig(config: Any?) {
        HermesJsonFormat.encodeToStream(config, configFile.outputStream())
    }
}

class PersistentConfigReader(val configFile: File) {

    @OptIn(ExperimentalSerializationApi::class)
    fun readConfig(): PersistentConfig {
        return HermesJsonFormat.decodeFromStream(configFile.inputStream())
    }

}

val CustomVariable.values get() = this.value as Map<String, ConfigVariable<Any?>> // legal typecast because that's what the return actually is

class PersistentConfigDelegate<T : Any?>(val keyName: String, val defaultValue: T) {

    operator fun getValue(config: PersistentConfig, property: KProperty<*>): T {
        val v = config[keyName]
        if (v == null) {
            config.addConfigVariable(keyName, defaultValue)
        }
        return config[keyName] as T
    }

    operator fun setValue(config: PersistentConfig, property: KProperty<*>, value: T) {
        config[keyName] = value
    }

}