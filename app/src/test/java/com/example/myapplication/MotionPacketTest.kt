package com.example.myapplication

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.MotionPacket
import com.example.myapplication.model.Vector3
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.util.Locale
import java.util.UUID
import com.example.myapplication.vision.PlanarOdometryState
import com.example.myapplication.vision.PlanarTrackingStatus
import com.example.myapplication.vision.PlanarOdometryMode

class MotionPacketTest {
    @Test
    fun lidarHeightAndFreshnessReachPcWithoutChangingLegacyHeightField() {
        for (valid in listOf(true, false)) {
            val json = MotionPacket(123456, 1789999999.123, QuaternionData(1.0, 0.0, 0.0, 0.0),
                Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0), sessionId = UUID.randomUUID(),
                deviceTimestampNanos = 999999999999999, quaternionTimestampNanos = 999999999999999,
                gyroscopeTimestampNanos = 999999999999999, accelerationTimestampNanos = 999999999999999,
                planarOdometry = PlanarOdometryState(mode = PlanarOdometryMode.TILT_COMPENSATED,
                    surfaceDistanceMetres = 1.2, usesLidarHeight = true, heightIsValid = valid,
                    lidarDepthMetres = if (valid) 1.4 else null, lidarAgeMillis = if (valid) 100.0 else null,
                    lidarRoundTripMillis = if (valid) 20.0 else null, lidarTimeSeconds = if (valid) 123.4 else null)
            ).toJson()
            val data = org.json.JSONObject(json)
            assertEquals(1.0, data.getDouble("height_above_surface_m"), 1e-9)
            val planar = data.getJSONObject("planar_odometry")
            assertEquals("IPHONE_LIDAR", planar.getString("height_source"))
            assertEquals(valid, planar.getBoolean("height_valid"))
            assertEquals(1.2, planar.getDouble("surface_distance_m"), 1e-9)
            assertEquals(!valid, planar.isNull("lidar_depth_m"))
            assertTrue("LiDAR packet size: ${json.length}", json.toByteArray().size <= 1400)
        }
    }
    @Test
    fun tiltDiagnosticsAreOptionalFiniteAndFitInOneDatagram() {
        for (tilt in listOf<Double?>(30.0, null, Double.NaN)) {
            val json = MotionPacket(1, 1.0, QuaternionData(1.0, 0.0, 0.0, 0.0),
                Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0),
                sessionId = UUID.fromString("12345678-1234-1234-1234-123456789abc"),
                deviceTimestampNanos = 10, quaternionTimestampNanos = 9,
                gyroscopeTimestampNanos = 8, accelerationTimestampNanos = 7,
                planarOdometry = PlanarOdometryState(mode = PlanarOdometryMode.TILT_COMPENSATED,
                    tiltDegrees = tilt, orientationGapMillis = 20.0, groundResidualMetres = .001),
            ).toJson()
            assertTrue(json.contains("\"mode\":\"TILT_COMPENSATED\""))
            assertTrue(json.contains("\"orientation_gap_ms\":20.00000"))
            assertTrue(json.contains("\"ground_residual_m\":0.00100"))
            assertTrue(json.contains(if (tilt == 30.0) "\"tilt_deg\":30.00000" else "\"tilt_deg\":null"))
            assertTrue(!json.contains("NaN"))
            assertTrue(json.toByteArray().size <= 1400)
        }
    }

    @Test
    fun packetSerializesToExpectedJsonShape() {
        val json = MotionPacket(
            sequence = 125,
            timestamp = 1723904521.256,
            quaternion = QuaternionData(
                w = 0.9238,
                x = 0.1021,
                y = 0.3015,
                z = 0.2057,
            ),
            gyroscope = Vector3(
                x = 0.01,
                y = 0.24,
                z = -0.12,
            ),
            linearAcceleration = Vector3(
                x = 0.15,
                y = -0.04,
                z = 0.82,
            ),
            heightAboveSurfaceMeters = 1.0,
            sessionId = UUID.fromString("12345678-1234-1234-1234-123456789abc"),
            deviceTimestampNanos = 845126735381L,
            quaternionTimestampNanos = 845126730100L,
            gyroscopeTimestampNanos = 845126729000L,
            accelerationTimestampNanos = 845126728000L,
            planarOdometry = PlanarOdometryState(
                trackingEpoch = 2L,
                timestampNanos = 845126721000L,
                status = PlanarTrackingStatus.TRACKING,
                isCameraActive = true,
                xMetres = 0.42,
                yMetres = -0.18,
                headingRadians = 0.12,
                lastDeltaXMetres = 0.002,
                lastDeltaYMetres = -0.001,
                trackedPoints = 287,
                inliers = 231,
                inlierRatio = 231.0 / 287,
                imageScale = 1.002,
                intrinsicsAreApproximate = false,
            ),
        ).toJson()

        assertTrue(json.startsWith("{\"sequence\":125,\"timestamp\":1723904521.256"))
        assertTrue(json.contains("\"quaternion\":{\"w\":0.9238,\"x\":0.1021,\"y\":0.3015,\"z\":0.2057}"))
        assertTrue(json.contains("\"gyroscope\":{\"x\":0.0100,\"y\":0.2400,\"z\":-0.1200}"))
        assertTrue(json.contains("\"linear_acceleration\":{\"x\":0.1500,\"y\":-0.0400,\"z\":0.8200}"))
        assertTrue(json.contains("\"height_above_surface_m\":1.000"))
        assertEquals('}', json.last())
        // Shared with the PC tests: checks exact wire compatibility across Kotlin and Python.
        val expected = checkNotNull(javaClass.getResource("/telemetry_v2.json")).readText().trim()
        assertEquals(expected, json)
        assertTrue("Keep the telemetry packet below a typical LAN MTU", json.toByteArray().size <= 1400)
    }

    @Test
    fun fixedHeightDefaultsToOneMeter() {
        val json = MotionPacket(
            sequence = 1,
            timestamp = 1.0,
            quaternion = QuaternionData(1.0, 0.0, 0.0, 0.0),
            gyroscope = Vector3(0.0, 0.0, 0.0),
            linearAcceleration = Vector3(0.0, 0.0, 0.0),
            sessionId = UUID.fromString("12345678-1234-1234-1234-123456789abc"),
            deviceTimestampNanos = 10L,
            quaternionTimestampNanos = 9L,
            gyroscopeTimestampNanos = 8L,
            accelerationTimestampNanos = 7L,
        ).toJson()

        assertTrue(json.contains("\"height_above_surface_m\":1.000"))
        assertTrue(json.contains("\"planar_odometry\":null"))
    }

    @Test
    fun serializationDoesNotDependOnSystemDecimalSeparator() {
        val original = Locale.getDefault()
        try {
            Locale.setDefault(Locale.GERMANY)
            packetSerializesToExpectedJsonShape()
        } finally {
            Locale.setDefault(original)
        }
    }
}
