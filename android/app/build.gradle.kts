plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.real2sim.capture"
    compileSdk = 36

    defaultConfig {
        applicationId = "com.real2sim.capture"
        minSdk = 29
        targetSdk = 36
        versionCode = 1
        versionName = "0.1.0"
        val key = (project.findProperty("ARCORE_API_KEY") as String?).orEmpty()
        manifestPlaceholders["arcoreApiKey"] = key
        buildConfigField("boolean", "HAS_ARCORE_API_KEY", if (key.isNotEmpty()) "true" else "false")
    }

    buildTypes {
        release {
            isMinifyEnabled = false
        }
    }
    buildFeatures {
        buildConfig = true
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    packaging {
        resources.excludes += "/META-INF/{AL2.0,LGPL2.1}"
    }
}

kotlin {
    jvmToolchain(17)
}

dependencies {
    implementation("com.google.ar:core:1.56.0")
    implementation("androidx.core:core-ktx:1.16.0")
    implementation("androidx.appcompat:appcompat:1.7.1")
}
