package com.example.myapplication

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.processing.OrientationController
import com.example.myapplication.processing.TrackingMode
import kotlin.math.sqrt
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class OrientationControllerTest {
    private val identity = QuaternionData(1.0, 0.0, 0.0, 0.0)
    private val quarterTurnAroundZ = QuaternionData(
        w = sqrt(0.5),
        x = 0.0,
        y = 0.0,
        z = sqrt(0.5),
    )

    @Test
    fun startingTrackingMakesCurrentOrientationNeutral() {
        val controller = OrientationController()
        controller.updateSensorQuaternion(quarterTurnAroundZ)

        controller.startOrResumeTracking()

        assertEquals(TrackingMode.TRACKING, controller.state.value.mode)
        assertQuaternionEquals(identity, controller.state.value.displayQuaternion)
    }

    @Test
    fun pausedModelDoesNotMoveAndResumeDoesNotJump() {
        val controller = OrientationController()
        controller.updateSensorQuaternion(identity)
        controller.startOrResumeTracking()
        controller.updateSensorQuaternion(quarterTurnAroundZ)
        val displayedBeforePause = controller.state.value.displayQuaternion

        controller.pauseTracking()
        controller.updateSensorQuaternion(QuaternionData(0.0, 0.0, 0.0, 1.0))

        assertEquals(TrackingMode.PAUSED, controller.state.value.mode)
        assertQuaternionEquals(displayedBeforePause, controller.state.value.displayQuaternion)

        controller.startOrResumeTracking()

        assertEquals(TrackingMode.TRACKING, controller.state.value.mode)
        assertQuaternionEquals(displayedBeforePause, controller.state.value.displayQuaternion)
    }

    @Test
    fun centeringMakesCurrentOrientationNeutral() {
        val controller = OrientationController()
        controller.updateSensorQuaternion(identity)
        controller.startOrResumeTracking()
        controller.updateSensorQuaternion(quarterTurnAroundZ)

        controller.centerOrientation()

        assertQuaternionEquals(identity, controller.state.value.displayQuaternion)
    }

    @Test
    fun visualQuaternionMovesTowardTargetWithoutJumpingDirectlyToIt() {
        val controller = OrientationController()
        controller.updateSensorQuaternion(identity, timestampNanos = 1_000_000_000L)
        controller.startOrResumeTracking()

        controller.updateSensorQuaternion(
            quarterTurnAroundZ,
            timestampNanos = 1_020_000_000L,
        )

        val state = controller.state.value
        assertTrue(state.displayQuaternion.z > 0.0)
        assertTrue(state.displayQuaternion.z < state.targetQuaternion.z)
    }

    private fun assertQuaternionEquals(expected: QuaternionData, actual: QuaternionData) {
        assertEquals(expected.w, actual.w, 0.0001)
        assertEquals(expected.x, actual.x, 0.0001)
        assertEquals(expected.y, actual.y, 0.0001)
        assertEquals(expected.z, actual.z, 0.0001)
    }
}
