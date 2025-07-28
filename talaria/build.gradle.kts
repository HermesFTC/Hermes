import java.net.URI

plugins {
    kotlin("jvm")

    `java-library`
    `java-test-fixtures`

    id("org.jetbrains.dokka")

    `maven-publish`
    id("io.deepmedia.tools.deployer")
}

repositories {
    mavenCentral()
    maven("https://frcmaven.wpi.edu/artifactory/release/")
}

dependencies {
    api(project(":core"))

    implementation(libs.jackson.databind)
    implementation(libs.bundles.wpilib)
    implementation(libs.quickbuf.runtime)

    testImplementation(libs.jetbrains.kotlin.test)

    testImplementation(testFixtures(project(":core")))
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
        groupId.set("dev.nextftc.nextrunner")
        artifactId.set("wpilib")

        name.set("NextRunner")
        description.set("A modern fork of RoadRunner.")
        url.set("https://github.com/NextFTC/NextRunner")
        scm {
            fromGithub("NextFTC", "NextRunner")
        }
        license("GNU General Public License, version 3", "https://www.gnu.org/licenses/gpl-3.0.html")

        developer("Zachary Harel", "ftc@zharel.me", url = "https://github.com/zachwaffle4")
        developer("Ryan Brott", "ftc@zharel.me", url = "https://github.com/zachwaffle4")
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
