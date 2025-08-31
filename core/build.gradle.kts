plugins {
    alias(libs.plugins.kotlin.jvm)
    alias(libs.plugins.kotlin.serialization)

    `java-library`
    `java-test-fixtures`

    alias(libs.plugins.dokka)

    `maven-publish`
    signing
    alias(libs.plugins.deployer)
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
        groupId.set("me.zharel.hermes")
        artifactId.set("core")

        name.set("Hermes")
        description.set("A modern fork of RoadRunner.")
        url.set("https://github.com/HermesFTC/Hermes")
        scm {
            fromGithub("HermesFTC", "Hermes")
        }
        license("MIT License", "https://opensource.org/license/mit")

        developer("Zachary Harel", "ftc@zharel.me", url = "https://github.com/zachwaffle4")
        developer("Ryan Brott", "rcbrott@gmail.com", url = "https://github.com/rbrott")
    }

    signing {
        key.set(secret("MVN_GPG_KEY"))
        password.set(secret("MVN_GPG_PASSWORD"))
    }

    content {
        kotlinComponents {
            kotlinSources()
            docs(dokkaJar)
        }
    }

    localSpec {
        release.version.set("$version")
    }

    nexusSpec("snapshot") {
        release.version.set("$version")
        repositoryUrl.set("https://central.sonatype.com/repository/maven-snapshots/")
        auth {
            user.set(secret("SONATYPE_USERNAME"))
            password.set(secret("SONATYPE_PASSWORD"))
        }
    }

    centralPortalSpec {
        auth {
            user.set(secret("SONATYPE_USERNAME"))
            password.set(secret("SONATYPE_PASSWORD"))
        }
        allowMavenCentralSync.set((property("automaticMavenCentralSync") as String).toBoolean())
    }
}