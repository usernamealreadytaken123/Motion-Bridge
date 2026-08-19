package com.example.myapplication

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.Vector3
import com.example.myapplication.processing.MotionAnalysisController
import com.example.myapplication.processing.MotionStatus
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class MotionAnalysisControllerTest {
    private val zero = Vector3(0.0, 0.0, 0.0)
    private val identityQuaternion = QuaternionData(1.0, 0.0, 0.0, 0.0)

    @Test
    fun uncalibratedInputKeepsAnalysisNotReady() {
        val controller = MotionAnalysisController()

        controller.update(
            isCalibrated = false,
            correctedGyroscope = zero,
            correctedLinearAcceleration = zero,
            orientationQuaternion = identityQuaternion,
            timestampNanos = 1_000_000_000L,
        )

        assertEquals(MotionStatus.NOT_READY, controller.state.value.status)
    }

    @Test
    fun stableLowValuesBecomeStationaryAfterConfirmationTime() {
        val controller = MotionAnalysisController()

        repeat(21) { index ->
            controller.update(
                isCalibrated = true,
                correctedGyroscope = Vector3(0.005, -0.004, 0.003),
                correctedLinearAcceleration = Vector3(0.01, -0.01, 0.015),
                orientationQuaternion = identityQuaternion,
                timestampNanos = 1_000_000_000L + index * 20_000_000L,
            )
        }

        assertEquals(MotionStatus.STATIONARY, controller.state.value.status)
    }

    @Test
    fun accelerationAboveExitThresholdChangesStationaryToMoving() {
        val controller = stationaryController()

        controller.update(
            isCalibrated = true,
            correctedGyroscope = zero,
            correctedLinearAcceleration = Vector3(1.0, 0.0, 0.0),
            orientationQuaternion = identityQuaternion,
            timestampNanos = 1_500_000_000L,
        )

        assertEquals(MotionStatus.MOVING, controller.state.value.status)
    }

    @Test
    fun lowPassFilterSoftensAccelerationStep() {
        val controller = stationaryController()

        controller.update(
            isCalibrated = true,
            correctedGyroscope = zero,
            correctedLinearAcceleration = Vector3(1.0, 0.0, 0.0),
            orientationQuaternion = identityQuaternion,
            timestampNanos = 1_500_000_000L,
        )

        val filteredX = controller.state.value.filteredLinearAcceleration!!.x
        assertTrue(filteredX > 0.0)
        assertTrue(filteredX < 1.0)
    }

    private fun stationaryController(): MotionAnalysisController =
        MotionAnalysisController().also { controller ->
            repeat(21) { index ->
                controller.update(
                    isCalibrated = true,
                    correctedGyroscope = zero,
                    correctedLinearAcceleration = zero,
                    orientationQuaternion = identityQuaternion,
                    timestampNanos = 1_000_000_000L + index * 20_000_000L,
                )
            }
            assertEquals(MotionStatus.STATIONARY, controller.state.value.status)
        }
}
