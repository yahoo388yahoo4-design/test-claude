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
        // phones are arm64; x86_64 keeps the emulator working. Drops MediaPipe's 32-bit native libraries.
        ndk { abiFilters += listOf("arm64-v8a", "x86_64") }
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
        // Robolectric screenshot test (UiScreenshotTest) needs the merged resources
        unitTests.isIncludeAndroidResources = true
    }
    androidResources {
        noCompress += "tflite"
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
    // Room map: on-device object detection (EfficientDet-Lite2 COCO model in assets, downloaded at build time)
    implementation("com.google.mediapipe:tasks-vision:0.10.26")
    testImplementation("junit:junit:4.13.2")
    // UiScreenshotTest: renders the redesigned screens off-device (native graphics) to build/screenshots/
    testImplementation("org.robolectric:robolectric:4.16")
    // real org.json for JVM tests (android.jar only has stubs)
    testImplementation("org.json:json:20260814")
}

// EfficientDet-Lite2 (COCO, int8, Apache-2.0, from the MediaPipe model zoo) for RoomScanner: fetched once into
// src/main/assets (git-ignored) so the repo does not carry the 7.5 MB binary.
val fetchDetector by tasks.registering {
    val out = file("src/main/assets/efficientdet_lite2.tflite")
    outputs.file(out)
    doLast {
        if (!out.exists() || out.length() < 1_000_000) {
            out.parentFile.mkdirs()
            val url = uri("https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite2/int8/1/efficientdet_lite2.tflite").toURL()
            url.openStream().use { input -> out.outputStream().use { o -> input.copyTo(o) } }
        }
    }
}
tasks.named("preBuild") { dependsOn(fetchDetector) }
