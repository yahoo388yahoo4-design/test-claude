package com.real2sim.capture

import android.app.Application

/** Installs the crash logger before any activity runs. */
class CaptureApp : Application() {
    override fun onCreate() {
        super.onCreate()
        CrashLog.install(this)
    }
}
