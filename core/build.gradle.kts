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

    testFixturesApi(libs.ejml)
    testFixturesApi(libs.bundles.kotest)

    testImplementation(libs.xchart)

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