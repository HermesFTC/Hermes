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
    maven("file:/Users/zach/releases/maven/development")
}

dependencies {
    api(project(":core"))

    implementation(libs.bundles.wpilib)

    implementation(libs.ejml)
    implementation(libs.kotlinx.serialization.json)

    testImplementation(libs.kotlin.test)

    api(libs.bundles.jackson)
    api(libs.quickbuf.runtime)

    testFixturesApi(libs.ejml)
    testFixturesApi(libs.bundles.kotest)

    dokkaHtmlPlugin(libs.mathjax.plugin)
}

kotlin {
    compilerOptions {
        freeCompilerArgs.set(listOf("-Xjvm-default=all"))
    }
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