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
    testOptions {
        unitTests.isReturnDefaultValues = true
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
    implementation("com.squareup.okhttp3:okhttp:4.12.0")
    // Neato / OpenBot over USB-OTG (MIT)
    implementation("com.github.mik3y:usb-serial-for-android:3.10.0")
    // Voice control: Gemini Nano on-device model (ML Kit GenAI Prompt API, supported phones only)
    implementation("com.google.mlkit:genai-prompt:1.0.0-beta4")
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.10.2")
    testImplementation("junit:junit:4.13.2")
    // real org.json for JVM tests (android.jar only has stubs)
    testImplementation("org.json:json:20260814")
}
