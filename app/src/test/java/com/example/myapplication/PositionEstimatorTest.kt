package com.example.myapplication

import com.example.myapplication.model.Vector3
import com.example.myapplication.processing.MotionStatus
import com.example.myapplication.processing.PositionEstimator
import com.example.myapplication.processing.VelocityState
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class PositionEstimatorTest {
    @Test
    fun constantVelocityForOneSecondProducesOneMeter() {
        val estimator = PositionEstimator()

        repeat(51) { index ->
            estimator.update(
                isCalibrated = true,
                motionStatus = MotionStatus.MOVING,
                velocityState = movingVelocity(Vector3(1.0, 0.0, 0.0)),
                timestampNanos = 1_000_000_000L + index * 20_000_000L,
            )
        }

        assertEquals(1.0, estimator.state.value.position.x, 0.000001)
        assertTrue(estimator.state.value.isIntegrating)
    }

    @Test
    fun stationaryStatusPreservesPositionAndStopsIntegration() {
        val estimator = PositionEstimator()
        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.MOVING,
            velocityState = movingVelocity(Vector3(1.0, 0.0, 0.0)),
            timestampNanos = 1_000_000_000L,
        )
        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.MOVING,
            velocityState = movingVelocity(Vector3(1.0, 0.0, 0.0)),
            timestampNanos = 1_020_000_000L,
        )
        val positionBeforeStop = estimator.state.value.position

        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.STATIONARY,
            velocityState = VelocityState(
                velocity = Vector3(0.0, 0.0, 0.0),
                isAvailable = true,
                zeroedByStationary = true,
            ),
            timestampNanos = 1_040_000_000L,
        )

        assertVectorEquals(positionBeforeStop, estimator.state.value.position)
        assertFalse(estimator.state.value.isIntegrating)
    }

    @Test
    fun resetPositionKeepsEstimatorAvailableButReturnsToOrigin() {
        val estimator = PositionEstimator()
        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.MOVING,
            velocityState = movingVelocity(Vector3(1.0, 0.0, 0.0)),
            timestampNanos = 1_000_000_000L,
        )
        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.MOVING,
            velocityState = movingVelocity(Vector3(1.0, 0.0, 0.0)),
            timestampNanos = 1_020_000_000L,
        )

        estimator.resetPosition()

        assertVectorEquals(Vector3(0.0, 0.0, 0.0), estimator.state.value.position)
        assertTrue(estimator.state.value.isAvailable)
    }

    @Test
    fun longTimestampGapIsNotIntegrated() {
        val estimator = PositionEstimator()
        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.MOVING,
            velocityState = movingVelocity(Vector3(1.0, 0.0, 0.0)),
            timestampNanos = 1_000_000_000L,
        )
        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.MOVING,
            velocityState = movingVelocity(Vector3(1.0, 0.0, 0.0)),
            timestampNanos = 2_000_000_000L,
        )

        assertVectorEquals(Vector3(0.0, 0.0, 0.0), estimator.state.value.position)
        assertFalse(estimator.state.value.isIntegrating)
    }

    private fun movingVelocity(value: Vector3) = VelocityState(
        velocity = value,
        isAvailable = true,
        isIntegrating = true,
    )

    private fun assertVectorEquals(expected: Vector3, actual: Vector3) {
        assertEquals(expected.x, actual.x, 0.000001)
        assertEquals(expected.y, actual.y, 0.000001)
        assertEquals(expected.z, actual.z, 0.000001)
    }
}
