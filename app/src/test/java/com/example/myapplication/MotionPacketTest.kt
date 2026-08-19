package com.example.myapplication

import com.example.myapplication.model.MotionPacket
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class MotionPacketTest {
    @Test
    fun testPacketSerializesToExpectedJsonShape() {
        val json = MotionPacket.testPacket(
            sequence = 125,
            timestamp = 1723904521.256,
        ).toJson()

        assertTrue(json.startsWith("{\"sequence\":125,\"timestamp\":1723904521.256"))
        assertTrue(json.contains("\"quaternion\":{\"w\":0.9238,\"x\":0.1021,\"y\":0.3015,\"z\":0.2057}"))
        assertTrue(json.contains("\"gyroscope\":{\"x\":0.0100,\"y\":0.2400,\"z\":-0.1200}"))
        assertTrue(json.contains("\"linear_acceleration\":{\"x\":0.1500,\"y\":-0.0400,\"z\":0.8200}"))
        assertEquals('}', json.last())
    }
}
