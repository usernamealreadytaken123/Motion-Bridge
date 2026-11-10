package com.example.myapplication

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.sensor.OrientationHistory
import kotlin.math.cos
import kotlin.math.sin
import org.junit.Assert.*
import org.junit.Test

class OrientationHistoryTest {
    private fun yaw(degrees: Double): QuaternionData {
        val half = Math.toRadians(degrees) / 2
        return QuaternionData(cos(half), 0.0, 0.0, sin(half))
    }

    @Test fun interpolatesAtFrameTimestampInsteadOfReturningLatest() {
        val history = OrientationHistory()
        history.add(1_000_000_000, yaw(0.0))
        history.add(1_020_000_000, yaw(20.0))
        val sample = checkNotNull(history.at(1_010_000_000))
        assertEquals(yaw(10.0).w, sample.quaternion.w, 1e-10)
        assertEquals(yaw(10.0).z, sample.quaternion.z, 1e-10)
        assertEquals(20_000_000L, sample.bracketGapNanos)
    }

    @Test fun doesNotExtrapolateOrBridgeMissingImuSamples() {
        val history = OrientationHistory()
        history.add(1_000_000_000, yaw(0.0))
        history.add(1_200_000_000, yaw(20.0))
        assertNull(history.at(900_000_000))
        assertNull(history.at(1_300_000_000))
        assertNull(history.at(1_050_000_000))
        assertNotNull(history.at(1_200_000_000))
    }

    @Test fun boundsHistoryAndClearsOnStop() {
        val history = OrientationHistory(capacity = 3)
        for (i in 1..5) history.add(i * 10_000_000L, yaw(i.toDouble()))
        assertNull(history.at(20_000_000))
        assertNotNull(history.at(35_000_000))
        history.clear()
        assertNull(history.at(40_000_000))
    }

    @Test fun ignoresInvalidDuplicateAndOutOfOrderSamples() {
        val history = OrientationHistory()
        history.add(10, yaw(0.0))
        history.add(10, yaw(90.0))
        history.add(9, yaw(90.0))
        history.add(15, QuaternionData(0.0, 0.0, 0.0, 0.0))
        history.add(16, QuaternionData(Double.NaN, 0.0, 0.0, 0.0))
        assertNull(history.at(9))
        assertNull(history.at(15))
        assertEquals(1.0, checkNotNull(history.at(10)).quaternion.w, 1e-10)
    }

    @Test fun quaternionSignChangeDoesNotCreateFalseRotation() {
        val history = OrientationHistory()
        val q = yaw(30.0)
        history.add(10, q)
        history.add(20, QuaternionData(-q.w, -q.x, -q.y, -q.z))
        assertEquals(q.w, checkNotNull(history.at(15)).quaternion.w, 1e-10)
        assertEquals(q.z, checkNotNull(history.at(15)).quaternion.z, 1e-10)
    }
}
