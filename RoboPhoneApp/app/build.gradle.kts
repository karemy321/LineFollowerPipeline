plugins {
    alias(libs.plugins.android.application)
}

// Redirect build output to a folder outside OneDrive so that OneDrive's file
// sync cannot lock intermediates and break incremental builds.
layout.buildDirectory.set(file("C:/RoboPhoneBuild/app"))

android {
    namespace = "com.example.firstproject"
    compileSdk {
        version = release(36) {
            minorApiLevel = 1
        }
    }

    defaultConfig {
        applicationId = "com.example.firstproject"
        minSdk = 24
        targetSdk = 36
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
}

dependencies {
    implementation(files("libs/LineFollowerPipeline-1.0.0.jar"))
    // CameraX — reliable Camera2-backed camera API, works on all modern Android devices
    implementation(libs.camera.core)
    implementation(libs.camera.camera2)
    implementation(libs.camera.lifecycle)
    implementation(libs.camera.view)
    // Guava ListenableFuture needed by CameraX ProcessCameraProvider.getInstance()
    implementation(libs.guava)

    implementation(libs.activity.ktx)
    implementation(libs.appcompat)
    implementation(libs.constraintlayout)
    implementation(libs.material)
    testImplementation(libs.junit)
    androidTestImplementation(libs.espresso.core)
    androidTestImplementation(libs.ext.junit)
}