import com.moowork.gradle.node.yarn.YarnTask
import kotlin.collections.mapOf

val nodeVersion: String = "18.12.1"

val webDir: File = file("${project.projectDir.parent}/web")

plugins {
    alias(libs.plugins.android.library)

    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.kotlin.serialization)

    alias(libs.plugins.dokka)

    `maven-publish`

    id("com.github.node-gradle.node") version "2.2.4"
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
}

node {
    version = nodeVersion
    download = true
    nodeModulesDir = webDir
}

val yarnInstall = tasks.named("yarn_install")

tasks.named<YarnTask>("yarn_build") {
    setEnvironment(
        mapOf("VITE_APP_VERSION" to (project.property("version") as String))
    )

    dependsOn(yarnInstall)
}

val yarnBuild = tasks.named("yarn_build")

val cleanWebAssets by tasks.registering(Delete::class) {
    delete(android.sourceSets["main"]?.assets?.srcDirs?.firstOrNull()?.let { file("$it/web") } ?: project.buildDir.resolve("web_assets_deletion_fallback"))
    // Added a fallback in case assets sourceDirs is null or empty
}

tasks.named("clean").configure {
    dependsOn(cleanWebAssets)
}

val copyWebAssets by tasks.registering(Copy::class) {
    from(file("${project.projectDir.parent}/web/dist"))
    into(android.sourceSets["main"]?.assets?.srcDirs?.firstOrNull()?.let { file("$it/web") } ?: project.buildDir.resolve("web_assets_copy_fallback"))
    // Added a fallback in case assets sourceDirs is null or empty
    dependsOn(cleanWebAssets, yarnBuild)
}

android {
    libraryVariants.all {
        preBuildProvider.get().dependsOn(copyWebAssets)
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

    implementation(libs.bundles.dashboard)

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
