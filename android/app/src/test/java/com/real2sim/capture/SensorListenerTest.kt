package com.real2sim.capture

import android.location.LocationListener
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import java.lang.reflect.Modifier

/**
 * On API 29 LocationListener.onProviderEnabled/onProviderDisabled/onStatusChanged are abstract, so the listener
 * class must carry concrete implementations (a SAM lambda compiled against API 36 would not -> AbstractMethodError).
 */
class SensorListenerTest {
    @Test fun locationListenerImplementsEveryCallback() {
        val type = SensorRecorder::class.java.getDeclaredField("locListener").type
        assertTrue(LocationListener::class.java.isAssignableFrom(type))
        for ((name, params) in listOf(
            "onLocationChanged" to arrayOf(android.location.Location::class.java),
            "onProviderEnabled" to arrayOf(String::class.java),
            "onProviderDisabled" to arrayOf(String::class.java),
            "onStatusChanged" to arrayOf(String::class.java, Int::class.javaPrimitiveType, android.os.Bundle::class.java),
        )) {
            val m = type.getMethod(name, *params)
            assertFalse("$name must be concrete, found in ${m.declaringClass}", Modifier.isAbstract(m.modifiers))
            assertFalse("$name must not be inherited from the platform stub", m.declaringClass == LocationListener::class.java)
        }
    }
}
