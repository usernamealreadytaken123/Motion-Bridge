package com.example.myapplication.processing

import com.example.myapplication.model.Vector3
import com.example.myapplication.model.QuaternionData
import kotlin.math.exp
import kotlin.math.sqrt
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

enum class MotionStatus {
    NOT_READY,
    CHECKING,
    STATIONARY,
    MOVING,
}

data class MotionAnalysisState(
    val status: MotionStatus = MotionStatus.NOT_READY,
    val filteredGyroscope: Vector3? = null,
    val filteredLinearAcceleration: Vector3? = null,
    val worldLinearAcceleration: Vector3? = null,
    val timestampNanos: Long = 0L,
)

class MotionAnalysisController {
    private val _state = MutableStateFlow(MotionAnalysisState())
    val state: StateFlow<MotionAnalysisState> = _state.asStateFlow()

    private var lastTimestampNanos = 0L
    private var stationaryCandidateStartedAtNanos = 0L

    @Synchronized
    fun update(
        isCalibrated: Boolean,
        correctedGyroscope: Vector3?,
        correctedLinearAcceleration: Vector3?,
        orientationQuaternion: QuaternionData?,
        timestampNanos: Long,
    ) {
        if (!isCalibrated) {
            if (_state.value.status != MotionStatus.NOT_READY || lastTimestampNanos != 0L) {
                reset()
            }
            return
        }
        if (
            correctedGyroscope == null ||
            correctedLinearAcceleration == null ||
            timestampNanos <= 0L ||
            timestampNanos <= lastTimestampNanos
        ) {
            return
        }

        val deltaSeconds = if (lastTimestampNanos == 0L) {
            DEFAULT_DELTA_SECONDS
        } else {
            ((timestampNanos - lastTimestampNanos) / NANOS_PER_SECOND)
                .coerceIn(MIN_DELTA_SECONDS, MAX_DELTA_SECONDS)
        }
        lastTimestampNanos = timestampNanos

        val currentState = _state.value
        val filteredGyroscope = lowPass(
            previous = currentState.filteredGyroscope,
            current = correctedGyroscope,
            deltaSeconds = deltaSeconds,
            timeConstantSeconds = GYROSCOPE_TIME_CONSTANT_SECONDS,
        )
        val filteredAcceleration = lowPass(
            previous = currentState.filteredLinearAcceleration,
            current = correctedLinearAcceleration,
            deltaSeconds = deltaSeconds,
            timeConstantSeconds = ACCELERATION_TIME_CONSTANT_SECONDS,
        )
        val worldLinearAcceleration = orientationQuaternion
            ?.rotateDeviceVectorToSceneWorld(filteredAcceleration)

        _state.value = MotionAnalysisState(
            status = detectMotionStatus(
                currentStatus = currentState.status,
                gyroscope = filteredGyroscope,
                linearAcceleration = filteredAcceleration,
                timestampNanos = timestampNanos,
            ),
            filteredGyroscope = filteredGyroscope,
            filteredLinearAcceleration = filteredAcceleration,
            worldLinearAcceleration = worldLinearAcceleration,
            timestampNanos = timestampNanos,
        )
    }

    @Synchronized
    fun reset() {
        lastTimestampNanos = 0L
        stationaryCandidateStartedAtNanos = 0L
        _state.value = MotionAnalysisState()
    }

    private fun lowPass(
        previous: Vector3?,
        current: Vector3,
        deltaSeconds: Double,
        timeConstantSeconds: Double,
    ): Vector3 {
        if (previous == null) return current

        val factor = 1.0 - exp(-deltaSeconds / timeConstantSeconds)
        return Vector3(
            x = previous.x + factor * (current.x - previous.x),
            y = previous.y + factor * (current.y - previous.y),
            z = previous.z + factor * (current.z - previous.z),
        )
    }

    private fun detectMotionStatus(
        currentStatus: MotionStatus,
        gyroscope: Vector3,
        linearAcceleration: Vector3,
        timestampNanos: Long,
    ): MotionStatus {
        val gyroscopeMagnitude = gyroscope.magnitude()
        val accelerationMagnitude = linearAcceleration.magnitude()
        val belowStationaryThreshold =
            gyroscopeMagnitude <= ENTER_STATIONARY_GYROSCOPE &&
                accelerationMagnitude <= ENTER_STATIONARY_ACCELERATION
        val aboveMovingThreshold =
            gyroscopeMagnitude >= EXIT_STATIONARY_GYROSCOPE ||
                accelerationMagnitude >= EXIT_STATIONARY_ACCELERATION

        if (currentStatus == MotionStatus.STATIONARY) {
            return if (aboveMovingThreshold) {
                stationaryCandidateStartedAtNanos = 0L
                MotionStatus.MOVING
            } else {
                MotionStatus.STATIONARY
            }
        }

        if (belowStationaryThreshold) {
            if (stationaryCandidateStartedAtNanos == 0L) {
                stationaryCandidateStartedAtNanos = timestampNanos
            }
            if (
                timestampNanos - stationaryCandidateStartedAtNanos >=
                STATIONARY_CONFIRMATION_DURATION_NANOS
            ) {
                return MotionStatus.STATIONARY
            }
            return if (currentStatus == MotionStatus.MOVING) {
                MotionStatus.MOVING
            } else {
                MotionStatus.CHECKING
            }
        }

        stationaryCandidateStartedAtNanos = 0L
        return when {
            aboveMovingThreshold -> MotionStatus.MOVING
            currentStatus == MotionStatus.MOVING -> MotionStatus.MOVING
            else -> MotionStatus.CHECKING
        }
    }

    private fun Vector3.magnitude(): Double = sqrt(x * x + y * y + z * z)

    private companion object {
        const val NANOS_PER_SECOND = 1_000_000_000.0
        const val DEFAULT_DELTA_SECONDS = 0.02
        const val MIN_DELTA_SECONDS = 0.001
        const val MAX_DELTA_SECONDS = 0.1

        const val GYROSCOPE_TIME_CONSTANT_SECONDS = 0.05
        const val ACCELERATION_TIME_CONSTANT_SECONDS = 0.08

        const val ENTER_STATIONARY_GYROSCOPE = 0.035
        const val EXIT_STATIONARY_GYROSCOPE = 0.07
        const val ENTER_STATIONARY_ACCELERATION = 0.10
        const val EXIT_STATIONARY_ACCELERATION = 0.18
        const val STATIONARY_CONFIRMATION_DURATION_NANOS = 350_000_000L
    }
}
