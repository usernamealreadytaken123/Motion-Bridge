package com.example.myapplication.processing

import com.example.myapplication.model.Vector3
import kotlin.math.sqrt
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

enum class CalibrationStatus {
    NOT_CALIBRATED,
    CALIBRATING,
    CALIBRATED,
    FAILED,
}

data class CalibrationState(
    val status: CalibrationStatus = CalibrationStatus.NOT_CALIBRATED,
    val progress: Float = 0f,
    val sampleCount: Int = 0,
    val gyroscopeBias: Vector3 = ZERO_VECTOR,
    val linearAccelerationBias: Vector3 = ZERO_VECTOR,
    val correctedGyroscope: Vector3? = null,
    val correctedLinearAcceleration: Vector3? = null,
    val errorMessage: String? = null,
)

class CalibrationController {
    private val _state = MutableStateFlow(CalibrationState())
    val state: StateFlow<CalibrationState> = _state.asStateFlow()

    private var calibrationStartedAtNanos = 0L
    private var lastSampleTimestampNanos = 0L
    private var gyroscopeSum = ZERO_VECTOR
    private var linearAccelerationSum = ZERO_VECTOR

    @Synchronized
    fun startCalibration() {
        calibrationStartedAtNanos = 0L
        lastSampleTimestampNanos = 0L
        gyroscopeSum = ZERO_VECTOR
        linearAccelerationSum = ZERO_VECTOR
        _state.value = CalibrationState(status = CalibrationStatus.CALIBRATING)
    }

    @Synchronized
    fun updateSensorSample(
        gyroscope: Vector3?,
        linearAcceleration: Vector3?,
        timestampNanos: Long,
    ) {
        if (gyroscope == null || linearAcceleration == null || timestampNanos <= 0L) return

        when (_state.value.status) {
            CalibrationStatus.CALIBRATING -> collectCalibrationSample(
                gyroscope = gyroscope,
                linearAcceleration = linearAcceleration,
                timestampNanos = timestampNanos,
            )

            CalibrationStatus.CALIBRATED -> publishCorrectedValues(
                gyroscope = gyroscope,
                linearAcceleration = linearAcceleration,
            )

            CalibrationStatus.NOT_CALIBRATED,
            CalibrationStatus.FAILED,
            -> Unit
        }
    }

    @Synchronized
    fun cancelCalibration() {
        if (_state.value.status == CalibrationStatus.CALIBRATING) {
            _state.value = CalibrationState(
                status = CalibrationStatus.FAILED,
                errorMessage = "Калибровка прервана",
            )
        }
    }

    private fun collectCalibrationSample(
        gyroscope: Vector3,
        linearAcceleration: Vector3,
        timestampNanos: Long,
    ) {
        if (timestampNanos <= lastSampleTimestampNanos) return

        if (
            gyroscope.magnitude() > MAX_STATIONARY_GYROSCOPE ||
            linearAcceleration.magnitude() > MAX_STATIONARY_ACCELERATION
        ) {
            _state.value = CalibrationState(
                status = CalibrationStatus.FAILED,
                errorMessage = "Обнаружено движение. Положите телефон неподвижно и повторите.",
            )
            return
        }

        if (calibrationStartedAtNanos == 0L) {
            calibrationStartedAtNanos = timestampNanos
        }
        lastSampleTimestampNanos = timestampNanos

        gyroscopeSum += gyroscope
        linearAccelerationSum += linearAcceleration
        val sampleCount = _state.value.sampleCount + 1
        val elapsedNanos = timestampNanos - calibrationStartedAtNanos
        val progress = (elapsedNanos.toDouble() / CALIBRATION_DURATION_NANOS)
            .coerceIn(0.0, 1.0)
            .toFloat()

        if (elapsedNanos >= CALIBRATION_DURATION_NANOS) {
            finishCalibration(
                sampleCount = sampleCount,
                gyroscope = gyroscope,
                linearAcceleration = linearAcceleration,
            )
        } else {
            _state.value = _state.value.copy(
                progress = progress,
                sampleCount = sampleCount,
            )
        }
    }

    private fun finishCalibration(
        sampleCount: Int,
        gyroscope: Vector3,
        linearAcceleration: Vector3,
    ) {
        if (sampleCount < MIN_CALIBRATION_SAMPLES) {
            _state.value = CalibrationState(
                status = CalibrationStatus.FAILED,
                errorMessage = "Получено слишком мало измерений. Повторите калибровку.",
            )
            return
        }

        val gyroscopeBias = gyroscopeSum / sampleCount.toDouble()
        val linearAccelerationBias = linearAccelerationSum / sampleCount.toDouble()
        _state.value = CalibrationState(
            status = CalibrationStatus.CALIBRATED,
            progress = 1f,
            sampleCount = sampleCount,
            gyroscopeBias = gyroscopeBias,
            linearAccelerationBias = linearAccelerationBias,
            correctedGyroscope = gyroscope - gyroscopeBias,
            correctedLinearAcceleration = linearAcceleration - linearAccelerationBias,
        )
    }

    private fun publishCorrectedValues(
        gyroscope: Vector3,
        linearAcceleration: Vector3,
    ) {
        val currentState = _state.value
        _state.value = currentState.copy(
            correctedGyroscope = gyroscope - currentState.gyroscopeBias,
            correctedLinearAcceleration =
                linearAcceleration - currentState.linearAccelerationBias,
        )
    }

    private fun Vector3.magnitude(): Double = sqrt(x * x + y * y + z * z)

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

    private operator fun Vector3.div(divisor: Double) = Vector3(
        x = x / divisor,
        y = y / divisor,
        z = z / divisor,
    )

    private companion object {
        const val CALIBRATION_DURATION_NANOS = 2_000_000_000L
        const val MIN_CALIBRATION_SAMPLES = 30
        const val MAX_STATIONARY_GYROSCOPE = 0.15
        const val MAX_STATIONARY_ACCELERATION = 0.5
    }
}

private val ZERO_VECTOR = Vector3(x = 0.0, y = 0.0, z = 0.0)
