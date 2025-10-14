import kotlin.collections.mapOf

plugins {
    alias(libs.plugins.android.library)

    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.kotlin.serialization)

    alias(libs.plugins.dokka)

    `maven-publish`
}

android {
    namespace = "gay.zharel.hermes.ftc"
    //noinspection GradleDependency
    compileSdk = 33

    defaultConfig {
    minSdk = 24

      testInstrumentationRunner = "android.support.test.runner.AndroidJUnitRunner"
      consumerProguardFiles("consumer-rules.pro")
    }

    buildTypes {
       release {
           isMinifyEnabled = false
           proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
       }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    kotlinOptions {
        jvmTarget = "1.8"
        freeCompilerArgs += ("-Xjvm-default=all")
    }

    testOptions {
        unitTests {
            isReturnDefaultValues = true
        }
    }

    publishing {
        singleVariant("release")
    }
}

repositories {
    mavenCentral()
    maven("https://maven.brott.dev/")
}

dependencies {
    api(project(":core"))
    api(project(":actions"))

    api(libs.bundles.dashboard)
    api(libs.fateweaver)

    implementation(libs.bundles.ftcsdk)

    implementation(libs.jackson.databind)
    implementation(libs.kotlinx.serialization.json)

    testImplementation(kotlin("test"))
    testImplementation(libs.mockk)
    testImplementation(libs.bundles.kotest)

    testImplementation(testFixtures(project(":core")))
}

val dokkaJar = tasks.register<Jar>("dokkaJar") {
    dependsOn(tasks.named("dokkaGenerate"))
    from(dokka.basePublicationsDirectory.dir("html"))
    archiveClassifier.set("html-docs")
}

deployer {
    projectInfo {
        artifactId.set("ftc")
        description.set("Integration of Hermes' core library with FTC.")
    }

    content {
        androidComponents("release") {
            kotlinSources()
            docs(dokkaJar)
        }
    }
}
