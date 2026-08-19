package com.example.myapplication.processing

import com.example.myapplication.model.Vector3
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

data class PositionState(
    val position: Vector3 = ZERO_POSITION,
    val isAvailable: Boolean = false,
    val isIntegrating: Boolean = false,
    val timestampNanos: Long = 0L,
)

class PositionEstimator {
    private val _state = MutableStateFlow(PositionState())
    val state: StateFlow<PositionState> = _state.asStateFlow()

    private var previousVelocity: Vector3? = null
    private var lastTimestampNanos = 0L

    @Synchronized
    fun update(
        isCalibrated: Boolean,
        motionStatus: MotionStatus,
        velocityState: VelocityState,
        timestampNanos: Long,
    ) {
        if (
            !isCalibrated ||
            !velocityState.isAvailable ||
            motionStatus == MotionStatus.NOT_READY
        ) {
            reset()
            return
        }
        if (timestampNanos <= 0L) return

        if (
            motionStatus != MotionStatus.MOVING ||
            !velocityState.isIntegrating
        ) {
            synchronizeWithoutIntegration(
                velocity = velocityState.velocity,
                timestampNanos = timestampNanos,
            )
            return
        }

        integrate(
            velocity = velocityState.velocity,
            timestampNanos = timestampNanos,
        )
    }

    @Synchronized
    fun resetPosition() {
        _state.value = _state.value.copy(position = ZERO_POSITION)
    }

    @Synchronized
    fun reset() {
        previousVelocity = null
        lastTimestampNanos = 0L
        if (_state.value != PositionState()) {
            _state.value = PositionState()
        }
    }

    private fun synchronizeWithoutIntegration(
        velocity: Vector3,
        timestampNanos: Long,
    ) {
        previousVelocity = velocity
        lastTimestampNanos = timestampNanos
        _state.value = _state.value.copy(
            isAvailable = true,
            isIntegrating = false,
            timestampNanos = timestampNanos,
        )
    }

    private fun integrate(
        velocity: Vector3,
        timestampNanos: Long,
    ) {
        val previous = previousVelocity
        if (previous == null || lastTimestampNanos == 0L) {
            synchronizeWithoutIntegration(velocity, timestampNanos)
            return
        }

        val elapsedNanos = timestampNanos - lastTimestampNanos
        if (elapsedNanos <= 0L) return
        if (elapsedNanos > MAX_INTEGRATION_GAP_NANOS) {
            synchronizeWithoutIntegration(velocity, timestampNanos)
            return
        }

        val deltaSeconds = elapsedNanos / NANOS_PER_SECOND
        val averageVelocity = (previous + velocity) / 2.0
        val position = _state.value.position + averageVelocity * deltaSeconds

        previousVelocity = velocity
        lastTimestampNanos = timestampNanos
        _state.value = PositionState(
            position = position,
            isAvailable = true,
            isIntegrating = true,
            timestampNanos = timestampNanos,
        )
    }

    private operator fun Vector3.plus(other: Vector3) = Vector3(
        x = x + other.x,
        y = y + other.y,
        z = z + other.z,
    )

    private operator fun Vector3.times(multiplier: Double) = Vector3(
        x = x * multiplier,
        y = y * multiplier,
        z = z * multiplier,
    )

    private operator fun Vector3.div(divisor: Double) = Vector3(
        x = x / divisor,
        y = y / divisor,
        z = z / divisor,
    )

    private companion object {
        const val NANOS_PER_SECOND = 1_000_000_000.0
        const val MAX_INTEGRATION_GAP_NANOS = 100_000_000L
    }
}

private val ZERO_POSITION = Vector3(x = 0.0, y = 0.0, z = 0.0)
