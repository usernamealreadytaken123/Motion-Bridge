package com.example.myapplication

import com.example.myapplication.model.Vector3
import com.example.myapplication.processing.CalibrationController
import com.example.myapplication.processing.CalibrationStatus
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Test

class CalibrationControllerTest {
    @Test
    fun stationarySamplesProduceBiasAndNearZeroCorrectedValues() {
        val controller = CalibrationController()
        val gyroscopeBias = Vector3(0.01, -0.02, 0.03)
        val accelerationBias = Vector3(0.04, -0.05, 0.06)
        controller.startCalibration()

        repeat(101) { index ->
            controller.updateSensorSample(
                gyroscope = gyroscopeBias,
                linearAcceleration = accelerationBias,
                timestampNanos = 1_000_000_000L + index * 20_000_000L,
            )
        }

        val state = controller.state.value
        assertEquals(CalibrationStatus.CALIBRATED, state.status)
        assertVectorEquals(gyroscopeBias, state.gyroscopeBias)
        assertVectorEquals(accelerationBias, state.linearAccelerationBias)
        assertNotNull(state.correctedGyroscope)
        assertNotNull(state.correctedLinearAcceleration)
        assertVectorEquals(Vector3(0.0, 0.0, 0.0), state.correctedGyroscope!!)
        assertVectorEquals(Vector3(0.0, 0.0, 0.0), state.correctedLinearAcceleration!!)
    }

    @Test
    fun movementFailsCalibration() {
        val controller = CalibrationController()
        controller.startCalibration()

        controller.updateSensorSample(
            gyroscope = Vector3(0.0, 0.0, 0.3),
            linearAcceleration = Vector3(0.0, 0.0, 0.0),
            timestampNanos = 1_000_000_000L,
        )

        assertEquals(CalibrationStatus.FAILED, controller.state.value.status)
    }

    private fun assertVectorEquals(expected: Vector3, actual: Vector3) {
        assertEquals(expected.x, actual.x, 0.000001)
        assertEquals(expected.y, actual.y, 0.000001)
        assertEquals(expected.z, actual.z, 0.000001)
    }
}
