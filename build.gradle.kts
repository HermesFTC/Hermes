import org.jetbrains.dokka.gradle.DokkaTaskPartial

plugins {
    id("com.android.application") version "8.0.2" apply false
    id("com.android.library") version "8.0.2" apply false
    id("org.jetbrains.dokka") version "2.0.0"
    id("io.deepmedia.tools.deployer") version "0.18.0"

    kotlin("android") version "2.0.0" apply false
    kotlin("jvm") version "2.0.0" apply false
    kotlin("kapt") version "2.0.0" apply false
    kotlin("plugin.serialization") version "2.0.0" apply false
}

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}

dependencies {
    dokka(project(":core"))
    dokka(project(":actions"))
    dokka(project(":ftc"))
}