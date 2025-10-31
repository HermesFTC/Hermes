/*
 * Copyright (c) 2025 Hermes FTC
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file at the root of this repository or at
 * https://opensource.org/licenses/MIT.
 */

package gay.zharel.hermes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.ValueProvider
import com.acmerobotics.dashboard.config.reflection.ArrayProvider
import com.acmerobotics.dashboard.config.reflection.FieldProvider
import com.acmerobotics.dashboard.config.variable.BasicVariable
import com.acmerobotics.dashboard.config.variable.ConfigVariable
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.dashboard.config.variable.VariableType
import gay.zharel.hermes.serialization.HermesJsonFormat
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.descriptors.PrimitiveKind.LONG
import kotlinx.serialization.json.decodeFromStream
import kotlinx.serialization.json.encodeToStream
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.File
import java.lang.reflect.Array
import java.lang.reflect.Field
import java.lang.reflect.Modifier
import kotlin.reflect.KProperty

object HermesConfig : PersistentConfig("HermesConfig", AppUtil.ROOT_FOLDER.resolve("hermes/config/config.json")) {

    var config: RobotConfig by PersistentConfigDelegate("RobotConfig", RobotConfig(), this)

    var tuningConfig: TuningConfig by PersistentConfigDelegate("TuningConfig", TuningConfig(), this)

}

open class PersistentConfig(val configName: String, configFile: File) {

    private val configReader: PersistentConfigReader = PersistentConfigReader(configFile)
    private val configWriter: PersistentConfigWriter = PersistentConfigWriter(configFile)

    private var configVariables: MutableMap<String, Any?> = mutableMapOf()

    private val customDashVar = CustomVariable()

    init {
        DashUtility.addConfigVariable(configName, customDashVar, this)
    }

    fun <T : Any?> addConfigVariable(name: String, value: T?) {
        configVariables[name] = value
        customDashVar.putVariable(name, DashUtility.getConfigVariable(value as Any, this))
        FtcDashboard.getInstance().updateConfig()
        updateConfig()
    }

    operator fun get(key: String): Any? {
        return configVariables[key]
    }

    operator fun <T: Any?> set(key: String, value: T): T? {
        addConfigVariable(key, value)

        return value
    }

    fun updateConfig() {
        subscribers.forEach { it.onUpdate(this.configVariables) }
        configWriter.writeConfig(this)
    }

    fun loadConfig() {
        val other = configReader.readConfig() // necessary for the ide to not crash out

        this.configVariables = other.configVariables
    }

    private val subscribers: MutableList<Subscriber> = arrayListOf()

    fun addSubscriber(s: Subscriber) {
        subscribers.add(s)
    }

    fun removeSubscriber(s: Subscriber) {
        subscribers.remove(s)
    }

    interface Subscriber {
        fun onUpdate(configVariables: MutableMap<String, Any?>)
    }

}

class PersistentConfigValueProvider<T: Any?>(val wrappedProvider: ValueProvider<T>, val config: PersistentConfig) : ValueProvider<T> {

    override fun get(): T? {
        return wrappedProvider.get()
    }

    override fun set(value: T?) {
        this.wrappedProvider.set(value)
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

class PersistentConfigDelegate<T : Any?>(val keyName: String, val defaultValue: T, val config: PersistentConfig) {

    operator fun getValue(ref: Any?, property: KProperty<*>): T {
        val v = config[keyName]
        if (v == null) {
            config.addConfigVariable(keyName, defaultValue)
        }
        return config[keyName] as T
    }

    operator fun setValue(ref: Any?, property: KProperty<*>, value: T) {
        config[keyName] = value
    }

}

object DashUtility {

    fun <T : Any> getConfigVariable(value: T, config: PersistentConfig): CustomVariable {

        return createVariableFromInstance(value::class.java, value, config)

    }

    fun <T : Any> addConfigVariable(name: String, value: T, config: PersistentConfig) {

        FtcDashboard.getInstance().withConfigRoot { it.putVariable(name, createVariableFromInstance(value::class.java, value, config)) }

    }

    fun <T> createVariableFromInstance(configClass: Class<out T?>, obj: T?, config: PersistentConfig): CustomVariable {
        val customVariable = CustomVariable()

        for (field in configClass.fields) {
            if (Modifier.isStatic(field.modifiers) || Modifier.isFinal(field.modifiers)) {
                continue
            }
            field.isAccessible = true
            customVariable.putVariable(field.name, createVariableFromField(field, obj, config))
        }

        return customVariable
    }

    fun createVariableFromField(field: Field, parent: Any?, config: PersistentConfig): ConfigVariable<*> {
        val fieldClass = field.type
        val type = VariableType.fromClass(fieldClass)
        when (type) {
            VariableType.BOOLEAN,
            VariableType.INT,
            VariableType.LONG,
            VariableType.FLOAT,
            VariableType.DOUBLE,
            VariableType.STRING,
            VariableType.ENUM -> return BasicVariable<Boolean?>(
                type,
                PersistentConfigValueProvider(
                    FieldProvider<Boolean?>(field, parent),
                    config
                )
            )

            VariableType.CUSTOM -> {
                try {
                    val value = field.get(parent)
                    if (value == null) {
                        return CustomVariable(null)
                    }
                    val customVariable = CustomVariable()
                    if (fieldClass.isArray) {
                        var i = 0
                        while (i < Array.getLength(value)) {
                            customVariable.putVariable(
                                i.toString(),
                                createVariableFromArrayField(
                                    field,
                                    field.type.componentType!!, parent, intArrayOf(i), config
                                )
                            )
                            i++
                        }
                    } else {
                        for (nestedField in fieldClass.fields) {
                            if (Modifier.isFinal(field.modifiers)) {
                                continue
                            }

                            val name = nestedField.name
                            customVariable.putVariable(
                                name,
                                createVariableFromField(nestedField, value, config)
                            )
                        }
                    }
                    return customVariable
                } catch (e: IllegalAccessException) {
                    throw RuntimeException(e)
                }
                throw RuntimeException(
                    "Unsupported field type: " +
                            fieldClass.name
                )
            }

            else -> throw RuntimeException(
                "Unsupported field type: " +
                        fieldClass.name
            )
        }
    }

    fun createVariableFromArrayField(
        field: Field, fieldClass: Class<*>,
        parent: Any?, indices: IntArray, config: PersistentConfig
    ): ConfigVariable<*> {
        val type = VariableType.fromClass(fieldClass)
        when (type) {
            VariableType.BOOLEAN,
            VariableType.INT,
            VariableType.LONG,
            VariableType.FLOAT,
            VariableType.DOUBLE,
            VariableType.STRING,
            VariableType.ENUM -> return BasicVariable<Any?>(
                type, PersistentConfigValueProvider(
                    ArrayProvider(
                        field, parent,
                        *indices.copyOf(indices.size)
                    ),
                    config
                )
            )

            VariableType.CUSTOM -> {
                try {
                    var value: Any? = null
                    try {
                        value = ArrayProvider.getArrayRecursive(field.get(parent), indices)
                    } catch (ignored: ArrayIndexOutOfBoundsException) {
                    }

                    if (value == null) {
                        return CustomVariable(null)
                    }
                    val customVariable = CustomVariable()
                    if (fieldClass.isArray) {
                        val newIndices = indices.copyOf(indices.size + 1)

                        var i = 0
                        while (i < Array.getLength(value)) {
                            newIndices[newIndices.size - 1] = i
                            customVariable.putVariable(
                                i.toString(),
                                createVariableFromArrayField(
                                    field, fieldClass.componentType!!,
                                    parent, newIndices, config
                                )
                            )
                            i++
                        }
                    } else {
                        for (nestedField in fieldClass.fields) {
                            if (Modifier.isFinal(nestedField.modifiers)) {
                                continue
                            }

                            val name = nestedField.name
                            customVariable.putVariable(
                                name,
                                createVariableFromField(nestedField, value, config)
                            )
                        }
                    }
                    return customVariable
                } catch (e: IllegalAccessException) {
                    throw RuntimeException(e)
                }
                throw RuntimeException(
                    "Unsupported field type: " +
                            fieldClass.name
                )
            }

            else -> throw RuntimeException(
                "Unsupported field type: " +
                        fieldClass.name
            )
        }
    }

}