plugins {
    alias(libs.plugins.kotlin.jvm) apply false
    alias(libs.plugins.kotlin.android) apply false
    alias(libs.plugins.kotlin.kapt)
    alias(libs.plugins.kotlin.serialization) apply false

    alias(libs.plugins.dokka)

    alias(libs.plugins.android.application) apply false
    alias(libs.plugins.android.library) apply false

    alias(libs.plugins.deployer)
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