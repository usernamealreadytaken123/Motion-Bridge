package com.example.myapplication.processing

import com.example.myapplication.model.Vector3
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

data class VelocityState(
    val velocity: Vector3 = ZERO_VELOCITY,
    val isAvailable: Boolean = false,
    val isIntegrating: Boolean = false,
    val zeroedByStationary: Boolean = false,
    val timestampNanos: Long = 0L,
)

class VelocityEstimator {
    private val _state = MutableStateFlow(VelocityState())
    val state: StateFlow<VelocityState> = _state.asStateFlow()

    private var previousWorldAcceleration: Vector3? = null
    private var lastTimestampNanos = 0L

    @Synchronized
    fun update(
        isCalibrated: Boolean,
        motionStatus: MotionStatus,
        worldLinearAcceleration: Vector3?,
        timestampNanos: Long,
    ) {
        if (!isCalibrated || motionStatus == MotionStatus.NOT_READY) {
            reset()
            return
        }
        if (worldLinearAcceleration == null || timestampNanos <= 0L) return

        when (motionStatus) {
            MotionStatus.STATIONARY -> {
                previousWorldAcceleration = worldLinearAcceleration
                lastTimestampNanos = timestampNanos
                _state.value = VelocityState(
                    velocity = ZERO_VELOCITY,
                    isAvailable = true,
                    isIntegrating = false,
                    zeroedByStationary = true,
                    timestampNanos = timestampNanos,
                )
            }

            MotionStatus.CHECKING -> {
                previousWorldAcceleration = worldLinearAcceleration
                lastTimestampNanos = timestampNanos
                _state.value = VelocityState(
                    velocity = ZERO_VELOCITY,
                    isAvailable = true,
                    isIntegrating = false,
                    timestampNanos = timestampNanos,
                )
            }

            MotionStatus.MOVING -> integrate(
                worldLinearAcceleration = worldLinearAcceleration,
                timestampNanos = timestampNanos,
            )

            MotionStatus.NOT_READY -> Unit
        }
    }

    @Synchronized
    fun reset() {
        previousWorldAcceleration = null
        lastTimestampNanos = 0L
        if (_state.value != VelocityState()) {
            _state.value = VelocityState()
        }
    }

    private fun integrate(
        worldLinearAcceleration: Vector3,
        timestampNanos: Long,
    ) {
        val previousAcceleration = previousWorldAcceleration
        if (previousAcceleration == null || lastTimestampNanos == 0L) {
            previousWorldAcceleration = worldLinearAcceleration
            lastTimestampNanos = timestampNanos
            _state.value = _state.value.copy(
                isAvailable = true,
                isIntegrating = false,
                zeroedByStationary = false,
                timestampNanos = timestampNanos,
            )
            return
        }

        val elapsedNanos = timestampNanos - lastTimestampNanos
        if (elapsedNanos <= 0L) return
        if (elapsedNanos > MAX_INTEGRATION_GAP_NANOS) {
            previousWorldAcceleration = worldLinearAcceleration
            lastTimestampNanos = timestampNanos
            _state.value = VelocityState(
                velocity = ZERO_VELOCITY,
                isAvailable = true,
                timestampNanos = timestampNanos,
            )
            return
        }

        val deltaSeconds = elapsedNanos / NANOS_PER_SECOND
        val averageAcceleration = (previousAcceleration + worldLinearAcceleration) / 2.0
        val velocity = _state.value.velocity + averageAcceleration * deltaSeconds

        previousWorldAcceleration = worldLinearAcceleration
        lastTimestampNanos = timestampNanos
        _state.value = VelocityState(
            velocity = velocity,
            isAvailable = true,
            isIntegrating = true,
            zeroedByStationary = false,
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

private val ZERO_VELOCITY = Vector3(x = 0.0, y = 0.0, z = 0.0)
