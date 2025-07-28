package com.acmerobotics.roadrunner.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.ValueProvider
import com.acmerobotics.dashboard.config.variable.BasicVariable
import com.acmerobotics.dashboard.config.variable.ConfigVariable
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.roadrunner.serialization.HermesJsonFormat
import com.google.gson.Gson
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.json.encodeToStream
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.File

object HermesConfig : PersistentConfig("HermesConfig", AppUtil.ROOT_FOLDER.resolve("hermes/config/config.txt")) {

}

open class PersistentConfig(val configName: String, var configFile: File) {

    private val configVariables: MutableMap<String, ValueProvider<Any?>> = mutableMapOf()

    private val configWriter get() = PersistentConfigWriter(configFile)
    private val configReader get() = PersistentConfigReader(configFile)

    @Suppress("UNCHECKED_CAST")
    fun <T> addConfigVariable(name: String, value: T?) {

        val provider: ValueProvider<T?> = PersistentConfigValueProvider(value, this)
        FtcDashboard.getInstance().addConfigVariable(configName, name, provider)
        configVariables.putIfAbsent(name, provider as ValueProvider<Any?>) // legal typecast since T is a subclass of Any?

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

    }

}

class PersistentConfigValueProvider<T: Any?>(initalValue: T, val config: PersistentConfig) : ValueProvider<T> {
    private var value: T? = initalValue

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
    fun writeConfig() {
        HermesJsonFormat.encodeToStream(this, configFile.outputStream())
    }
}

class PersistentConfigReader(val configFile: File) {

}


val CustomVariable.values get() = this.value as Map<String, ConfigVariable<Any?>> // legal typecast because that's what the return actually is