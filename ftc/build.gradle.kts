plugins {
    alias(libs.plugins.android.library)

    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.kotlin.serialization)

    alias(libs.plugins.dokka)

    `maven-publish`
    alias(libs.plugins.deployer)
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
        groupId.set("me.zharel.hermes")
        artifactId.set("ftc")

        name.set("Hermes")
        description.set("Integration of Hermes' core library with FTC.")
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
        androidComponents("release") {
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
