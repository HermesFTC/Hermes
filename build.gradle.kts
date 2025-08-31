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

subprojects {
    apply(plugin = "io.deepmedia.tools.deployer")

    configure<io.deepmedia.tools.deployer.DeployerExtension> {
        projectInfo {
            groupId.set("gay.zharel.hermes")
            name.set("Hermes")
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
}

dependencies {
    dokka(project(":core"))
    dokka(project(":actions"))
    dokka(project(":ftc"))
}