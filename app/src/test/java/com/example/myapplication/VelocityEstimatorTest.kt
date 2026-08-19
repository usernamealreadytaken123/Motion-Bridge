package com.example.myapplication

import com.example.myapplication.model.Vector3
import com.example.myapplication.processing.MotionStatus
import com.example.myapplication.processing.VelocityEstimator
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class VelocityEstimatorTest {
    @Test
    fun constantAccelerationForOneSecondProducesOneMeterPerSecond() {
        val estimator = VelocityEstimator()
        val acceleration = Vector3(1.0, 0.0, 0.0)

        repeat(51) { index ->
            estimator.update(
                isCalibrated = true,
                motionStatus = MotionStatus.MOVING,
                worldLinearAcceleration = acceleration,
                timestampNanos = 1_000_000_000L + index * 20_000_000L,
            )
        }

        assertEquals(1.0, estimator.state.value.velocity.x, 0.000001)
        assertTrue(estimator.state.value.isIntegrating)
    }

    @Test
    fun stationaryStatusImmediatelyZerosVelocity() {
        val estimator = VelocityEstimator()
        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.MOVING,
            worldLinearAcceleration = Vector3(1.0, 0.0, 0.0),
            timestampNanos = 1_000_000_000L,
        )
        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.MOVING,
            worldLinearAcceleration = Vector3(1.0, 0.0, 0.0),
            timestampNanos = 1_020_000_000L,
        )

        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.STATIONARY,
            worldLinearAcceleration = Vector3(0.0, 0.0, 0.0),
            timestampNanos = 1_040_000_000L,
        )

        assertVectorEquals(Vector3(0.0, 0.0, 0.0), estimator.state.value.velocity)
        assertTrue(estimator.state.value.zeroedByStationary)
        assertFalse(estimator.state.value.isIntegrating)
    }

    @Test
    fun longTimestampGapIsNotIntegrated() {
        val estimator = VelocityEstimator()
        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.MOVING,
            worldLinearAcceleration = Vector3(1.0, 0.0, 0.0),
            timestampNanos = 1_000_000_000L,
        )
        estimator.update(
            isCalibrated = true,
            motionStatus = MotionStatus.MOVING,
            worldLinearAcceleration = Vector3(1.0, 0.0, 0.0),
            timestampNanos = 2_000_000_000L,
        )

        assertVectorEquals(Vector3(0.0, 0.0, 0.0), estimator.state.value.velocity)
        assertFalse(estimator.state.value.isIntegrating)
    }

    private fun assertVectorEquals(expected: Vector3, actual: Vector3) {
        assertEquals(expected.x, actual.x, 0.000001)
        assertEquals(expected.y, actual.y, 0.000001)
        assertEquals(expected.z, actual.z, 0.000001)
    }
}
