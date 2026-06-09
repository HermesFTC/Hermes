import org.wpilib.gradlerio.wpi.WPIExtension

plugins {
  alias(libs.plugins.kotlin.jvm)
  alias(libs.plugins.kotlin.serialization)

  `java-library`
  `java-test-fixtures`

  alias(libs.plugins.dokka)

  `maven-publish`
  signing

  alias(libs.plugins.gradleRIO)
}

repositories {
  mavenCentral()
  maven("file:/Users/zach/releases/maven/development")
}

val wpi = the<WPIExtension>()

dependencies {
  api(project(":core"))

  implementation(libs.ejml)
  implementation(libs.kotlinx.serialization.json)

  testImplementation(libs.kotlin.test)

  api(libs.bundles.jackson)
  api(libs.quickbuf.runtime)

  testFixturesApi(libs.bundles.kotest)

  dokkaHtmlPlugin(libs.mathjax.plugin)

  wpi.java.deps.wpilibAnnotations().forEach(::implementation)
  wpi.java.deps.wpilib().forEach(::implementation)
  wpi.java.vendor.java().forEach(::implementation)

  wpi.java.deps.wpilib().forEach(::testFixturesImplementation)
  wpi.java.vendor.java().forEach(::testFixturesImplementation)

//  systemcoreDebug(wpi.java.deps.wpilibJniDebug(wpi.platforms.systemcore))
//  systemcoreDebug(wpi.java.vendor.jniDebug(wpi.platforms.systemcore))
//
//  systemcoreRelease(wpi.java.deps.wpilibJniRelease(wpi.platforms.systemcore))
//  systemcoreRelease(wpi.java.vendor.jniRelease(wpi.platforms.systemcore))
//
//  nativeDebug(wpi.java.deps.wpilibJniDebug(wpi.platforms.desktop))
//  nativeDebug(wpi.java.vendor.jniDebug(wpi.platforms.desktop))
//  simulationDebug(wpi.sim.enableDebug())
//
//  nativeRelease(wpi.java.deps.wpilibJniRelease(wpi.platforms.desktop))
//  nativeRelease(wpi.java.vendor.jniRelease(wpi.platforms.desktop))
//  simulationRelease(wpi.sim.enableRelease())
}

kotlin {
  jvmToolchain(25)
}

tasks.named<Test>("test") {
  useJUnitPlatform()
}

val dokkaJar = tasks.register<Jar>("dokkaJar") {
  dependsOn(tasks.named("dokkaGenerate"))
  from(dokka.basePublicationsDirectory.dir("html"))
  archiveClassifier.set("html-docs")
}

deployer {
  projectInfo {
    artifactId.set("wpi")
    description.set("Integrations between Hermes and WPILib.")
  }

  content {
    kotlinComponents {
      kotlinSources()
      docs(dokkaJar)
    }
  }
}
