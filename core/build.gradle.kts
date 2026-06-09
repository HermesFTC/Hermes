import org.jetbrains.kotlin.gradle.dsl.JvmTarget
import org.jetbrains.kotlin.gradle.tasks.KotlinJvmCompile

plugins {
  alias(libs.plugins.kotlin.jvm)
  alias(libs.plugins.kotlin.serialization)

  `java-library`
  `java-test-fixtures`

  alias(libs.plugins.dokka)

  `maven-publish`
  signing
}

repositories {
  mavenCentral()
}

dependencies {
  implementation(libs.ejml)
  implementation(libs.kotlinx.serialization.json)

  testImplementation(libs.kotlin.test)
  testImplementation(libs.xchart)

  testFixturesApi(libs.bundles.kotest)

  dokkaHtmlPlugin(libs.mathjax.plugin)
}

kotlin {
  compilerOptions {
    jvmTarget.set(JvmTarget.JVM_25)
    freeCompilerArgs.set(listOf("-jvm-default=all"))
  }
}

tasks.named<KotlinJvmCompile>("compileKotlin") {
  compilerOptions {
    jvmTarget.set(JvmTarget.JVM_1_8)
  }
}

java {
  toolchain {
    languageVersion.set(JavaLanguageVersion.of(25))
  }
}

tasks.named<JavaCompile>("compileJava") {
  options.release.set(8)
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
    artifactId.set("core")
    description.set("A modern fork of RoadRunner.")
  }

  content {
    kotlinComponents {
      kotlinSources()
      docs(dokkaJar)
    }
  }
}
