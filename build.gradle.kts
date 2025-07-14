plugins {
    kotlin("jvm") version libs.versions.kotlin apply false
    kotlin("android") version libs.versions.kotlin apply false
    libs.plugins.kotlin.kapt
    kotlin("plugin.serialization") version libs.versions.kotlin apply false

    id("org.jetbrains.dokka") version libs.versions.kotlin

    id("com.android.application") version libs.versions.android apply false
    id("com.android.library") version libs.versions.android apply false

    id("io.deepmedia.tools.deployer") version libs.versions.deployer
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