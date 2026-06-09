import com.diffplug.gradle.spotless.SpotlessExtension
import io.deepmedia.tools.deployer.DeployerExtension
import org.gradle.kotlin.dsl.configure

plugins {
  alias(libs.plugins.kotlin.jvm) apply false
  alias(libs.plugins.kotlin.kapt)
  alias(libs.plugins.kotlin.serialization) apply false

  alias(libs.plugins.dokka)

  alias(libs.plugins.deployer)
  alias(libs.plugins.spotless)

  alias(libs.plugins.gradleRIO) apply false
}

allprojects {
  apply(plugin = "com.diffplug.spotless")

  repositories {
    google()
    mavenCentral()
  }

  extensions.configure<SpotlessExtension> {
    kotlinGradle {
      ktlint().editorConfigOverride(
        mapOf(
          "ktlint_code_style" to "intellij_idea",
          "indent_size" to "2",
          "continuation_indent_size" to "2",
          "ktlint_standard_no-wildcard-imports" to "disabled",
          "max_line_length" to "108",
        ),
      )
    }
  }
}

subprojects {
  apply(plugin = "io.deepmedia.tools.deployer")

  configure<DeployerExtension> {
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

  extensions.configure<SpotlessExtension> {
    kotlin {
      target("src/*/kotlin/**/*.kt")
      ktlint().editorConfigOverride(
        mapOf(
          "ktlint_code_style" to "intellij_idea",
          "indent_size" to "2",
          "continuation_indent_size" to "2",
          "ktlint_standard_no-wildcard-imports" to "disabled",
          "max_line_length" to "108",
        ),
      )
    }
  }
}

dependencies {
  dokka(project(":core"))
  dokka(project(":wpi"))
}
