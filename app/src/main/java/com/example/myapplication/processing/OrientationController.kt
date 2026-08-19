package com.example.myapplication.processing

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.Vector3
import kotlin.math.exp
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

enum class TrackingMode {
    IDLE,
    TRACKING,
    PAUSED,
}

data class OrientationTrackingState(
    val mode: TrackingMode = TrackingMode.IDLE,
    val sensorQuaternion: QuaternionData? = null,
    val targetQuaternion: QuaternionData = IDENTITY_QUATERNION,
    val displayQuaternion: QuaternionData = IDENTITY_QUATERNION,
    val estimatedPosition: Vector3 = ZERO_TRACKING_POSITION,
    val displayPosition: Vector3 = ZERO_TRACKING_POSITION,
    val sensorTimestampNanos: Long = 0L,
)

class OrientationController {
    private val _state = MutableStateFlow(OrientationTrackingState())
    val state: StateFlow<OrientationTrackingState> = _state.asStateFlow()

    private var referenceQuaternion = IDENTITY_QUATERNION
    private var baseDisplayQuaternion = IDENTITY_QUATERNION
    private var lastSmoothingTimestampNanos = 0L
    private var referencePosition = ZERO_TRACKING_POSITION
    private var baseDisplayPosition = ZERO_TRACKING_POSITION

    @Synchronized
    fun updateSensorQuaternion(
        quaternion: QuaternionData,
        timestampNanos: Long = 0L,
    ) {
        val normalizedQuaternion = quaternion.normalized()
        val currentState = _state.value
        var targetQuaternion = currentState.targetQuaternion
        var displayQuaternion = currentState.displayQuaternion

        if (currentState.mode == TrackingMode.TRACKING) {
            targetQuaternion = calculateTargetQuaternion(normalizedQuaternion)
            val smoothingFactor = calculateSmoothingFactor(
                current = displayQuaternion,
                target = targetQuaternion,
                timestampNanos = timestampNanos,
            )
            displayQuaternion = displayQuaternion.slerp(targetQuaternion, smoothingFactor)
        }

        _state.value = currentState.copy(
            sensorQuaternion = normalizedQuaternion,
            targetQuaternion = targetQuaternion,
            displayQuaternion = displayQuaternion,
            sensorTimestampNanos = timestampNanos,
        )
    }

    @Synchronized
    fun updateEstimatedPosition(position: Vector3) {
        val currentState = _state.value
        val displayPosition = if (currentState.mode == TrackingMode.TRACKING) {
            baseDisplayPosition + (position - referencePosition)
        } else {
            currentState.displayPosition
        }
        _state.value = currentState.copy(
            estimatedPosition = position,
            displayPosition = displayPosition,
        )
    }

    @Synchronized
    fun startOrResumeTracking() {
        val currentState = _state.value
        val currentQuaternion = currentState.sensorQuaternion ?: return

        when (currentState.mode) {
            TrackingMode.IDLE -> {
                referenceQuaternion = currentQuaternion
                baseDisplayQuaternion = IDENTITY_QUATERNION
                lastSmoothingTimestampNanos = currentState.sensorTimestampNanos
                referencePosition = currentState.estimatedPosition
                baseDisplayPosition = ZERO_TRACKING_POSITION
                _state.value = currentState.copy(
                    mode = TrackingMode.TRACKING,
                    targetQuaternion = IDENTITY_QUATERNION,
                    displayQuaternion = IDENTITY_QUATERNION,
                    displayPosition = ZERO_TRACKING_POSITION,
                )
            }

            TrackingMode.PAUSED -> {
                // Новая опорная ориентация сохраняет модель на месте при продолжении.
                referenceQuaternion = currentQuaternion
                baseDisplayQuaternion = currentState.displayQuaternion
                lastSmoothingTimestampNanos = currentState.sensorTimestampNanos
                referencePosition = currentState.estimatedPosition
                baseDisplayPosition = currentState.displayPosition
                _state.value = currentState.copy(
                    mode = TrackingMode.TRACKING,
                    targetQuaternion = currentState.displayQuaternion,
                )
            }

            TrackingMode.TRACKING -> Unit
        }
    }

    @Synchronized
    fun pauseTracking() {
        val currentState = _state.value
        if (currentState.mode == TrackingMode.TRACKING) {
            _state.value = currentState.copy(mode = TrackingMode.PAUSED)
        }
    }

    @Synchronized
    fun centerOrientation() {
        val currentState = _state.value
        val currentQuaternion = currentState.sensorQuaternion ?: return

        referenceQuaternion = currentQuaternion
        baseDisplayQuaternion = IDENTITY_QUATERNION
        lastSmoothingTimestampNanos = currentState.sensorTimestampNanos
        _state.value = currentState.copy(
            targetQuaternion = IDENTITY_QUATERNION,
            displayQuaternion = IDENTITY_QUATERNION,
        )
    }

    @Synchronized
    fun resetDisplayPosition(estimatedPosition: Vector3 = _state.value.estimatedPosition) {
        referencePosition = estimatedPosition
        baseDisplayPosition = ZERO_TRACKING_POSITION
        _state.value = _state.value.copy(
            estimatedPosition = estimatedPosition,
            displayPosition = ZERO_TRACKING_POSITION,
        )
    }

    private fun calculateTargetQuaternion(currentQuaternion: QuaternionData): QuaternionData {
        val relativeQuaternion = referenceQuaternion
            .inverse()
            .multiply(currentQuaternion)
            .normalized()

        return baseDisplayQuaternion
            .multiply(relativeQuaternion)
            .normalized()
    }

    private fun calculateSmoothingFactor(
        current: QuaternionData,
        target: QuaternionData,
        timestampNanos: Long,
    ): Double {
        val deltaSeconds = if (
            lastSmoothingTimestampNanos > 0L &&
            timestampNanos > lastSmoothingTimestampNanos
        ) {
            ((timestampNanos - lastSmoothingTimestampNanos) / NANOS_PER_SECOND)
                .coerceIn(MIN_DELTA_SECONDS, MAX_DELTA_SECONDS)
        } else {
            DEFAULT_DELTA_SECONDS
        }
        lastSmoothingTimestampNanos = timestampNanos

        val timeBasedFactor = 1.0 - exp(-deltaSeconds / SMOOTHING_TIME_CONSTANT_SECONDS)
        val angularDistance = current.angularDistanceTo(target)
        val responseBoost = (angularDistance / FULL_BOOST_ANGLE_RADIANS)
            .coerceIn(0.0, 1.0) * MAX_RESPONSE_BOOST

        return (timeBasedFactor + responseBoost).coerceIn(0.0, MAX_SMOOTHING_FACTOR)
    }

    private companion object {
        const val NANOS_PER_SECOND = 1_000_000_000.0
        const val DEFAULT_DELTA_SECONDS = 0.02
        const val MIN_DELTA_SECONDS = 0.001
        const val MAX_DELTA_SECONDS = 0.1
        const val SMOOTHING_TIME_CONSTANT_SECONDS = 0.06
        const val FULL_BOOST_ANGLE_RADIANS = 0.7853981633974483 // 45 градусов
        const val MAX_RESPONSE_BOOST = 0.3
        const val MAX_SMOOTHING_FACTOR = 0.85
    }

    private operator fun Vector3.plus(other: Vector3) = Vector3(
        x = x + other.x,
        y = y + other.y,
        z = z + other.z,
    )

    private operator fun Vector3.minus(other: Vector3) = Vector3(
        x = x - other.x,
        y = y - other.y,
        z = z - other.z,
    )
}

private val ZERO_TRACKING_POSITION = Vector3(x = 0.0, y = 0.0, z = 0.0)
