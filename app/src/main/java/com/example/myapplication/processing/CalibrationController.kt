package com.example.myapplication.processing

import com.example.myapplication.model.Vector3
import kotlin.math.exp
import kotlin.math.max
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
    val gyroscopeNoise: Vector3 = ZERO_VECTOR,
    val linearAccelerationNoise: Vector3 = ZERO_VECTOR,
    val correctedGyroscope: Vector3? = null,
    val correctedLinearAcceleration: Vector3? = null,
    val isAdaptiveBiasUpdating: Boolean = false,
    val adaptiveBiasUpdateCount: Int = 0,
    val errorMessage: String? = null,
)

class CalibrationController {
    private val _state = MutableStateFlow(CalibrationState())
    val state: StateFlow<CalibrationState> = _state.asStateFlow()

    private var calibrationStartedAtNanos = 0L
    private var lastSampleTimestampNanos = 0L
    private var gyroscopeSum = ZERO_VECTOR
    private var linearAccelerationSum = ZERO_VECTOR
    private var gyroscopeSquareSum = ZERO_VECTOR
    private var linearAccelerationSquareSum = ZERO_VECTOR
    private var lastAdaptiveTimestampNanos = 0L

    @Synchronized
    fun startCalibration() {
        calibrationStartedAtNanos = 0L
        lastSampleTimestampNanos = 0L
        gyroscopeSum = ZERO_VECTOR
        linearAccelerationSum = ZERO_VECTOR
        gyroscopeSquareSum = ZERO_VECTOR
        linearAccelerationSquareSum = ZERO_VECTOR
        lastAdaptiveTimestampNanos = 0L
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

    @Synchronized
    fun updateAdaptiveBias(
        gyroscope: Vector3?,
        linearAcceleration: Vector3?,
        isStationary: Boolean,
        timestampNanos: Long,
    ) {
        val currentState = _state.value
        if (
            currentState.status != CalibrationStatus.CALIBRATED ||
            gyroscope == null ||
            linearAcceleration == null ||
            timestampNanos <= 0L
        ) {
            return
        }

        val elapsedNanos = timestampNanos - lastAdaptiveTimestampNanos
        lastAdaptiveTimestampNanos = timestampNanos

        val correctedGyroscope = gyroscope - currentState.gyroscopeBias
        val correctedAcceleration =
            linearAcceleration - currentState.linearAccelerationBias
        val accelerationThreshold = adaptiveThreshold(
            noise = currentState.linearAccelerationNoise,
            minimum = MIN_ADAPTIVE_ACCELERATION_THRESHOLD,
            maximum = MAX_ADAPTIVE_ACCELERATION_THRESHOLD,
        )
        val gyroscopeThreshold = adaptiveThreshold(
            noise = currentState.gyroscopeNoise,
            minimum = MIN_ADAPTIVE_GYROSCOPE_THRESHOLD,
            maximum = MAX_ADAPTIVE_GYROSCOPE_THRESHOLD,
        )
        val canAdapt =
            isStationary &&
                correctedAcceleration.magnitude() <= accelerationThreshold &&
                correctedGyroscope.magnitude() <= gyroscopeThreshold &&
                elapsedNanos in 1..MAX_ADAPTIVE_GAP_NANOS

        if (!canAdapt) {
            if (currentState.isAdaptiveBiasUpdating) {
                _state.value = currentState.copy(isAdaptiveBiasUpdating = false)
            }
            return
        }

        val deltaSeconds = elapsedNanos / NANOS_PER_SECOND
        val factor = 1.0 - exp(-deltaSeconds / ADAPTIVE_BIAS_TIME_CONSTANT_SECONDS)
        val newGyroscopeBias = currentState.gyroscopeBias.moveToward(gyroscope, factor)
        val newAccelerationBias = currentState.linearAccelerationBias.moveToward(
            linearAcceleration,
            factor,
        )

        _state.value = currentState.copy(
            gyroscopeBias = newGyroscopeBias,
            linearAccelerationBias = newAccelerationBias,
            correctedGyroscope = gyroscope - newGyroscopeBias,
            correctedLinearAcceleration = linearAcceleration - newAccelerationBias,
            isAdaptiveBiasUpdating = true,
            adaptiveBiasUpdateCount = currentState.adaptiveBiasUpdateCount + 1,
        )
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
        gyroscopeSquareSum += gyroscope.squared()
        linearAccelerationSquareSum += linearAcceleration.squared()
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
        val gyroscopeNoise = standardDeviation(
            squareSum = gyroscopeSquareSum,
            mean = gyroscopeBias,
            sampleCount = sampleCount,
        )
        val linearAccelerationNoise = standardDeviation(
            squareSum = linearAccelerationSquareSum,
            mean = linearAccelerationBias,
            sampleCount = sampleCount,
        )
        _state.value = CalibrationState(
            status = CalibrationStatus.CALIBRATED,
            progress = 1f,
            sampleCount = sampleCount,
            gyroscopeBias = gyroscopeBias,
            linearAccelerationBias = linearAccelerationBias,
            gyroscopeNoise = gyroscopeNoise,
            linearAccelerationNoise = linearAccelerationNoise,
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

    private fun Vector3.squared() = Vector3(x = x * x, y = y * y, z = z * z)

    private fun Vector3.moveToward(target: Vector3, factor: Double) = Vector3(
        x = x + factor * (target.x - x),
        y = y + factor * (target.y - y),
        z = z + factor * (target.z - z),
    )

    private fun standardDeviation(
        squareSum: Vector3,
        mean: Vector3,
        sampleCount: Int,
    ) = Vector3(
        x = sqrt(max(0.0, squareSum.x / sampleCount - mean.x * mean.x)),
        y = sqrt(max(0.0, squareSum.y / sampleCount - mean.y * mean.y)),
        z = sqrt(max(0.0, squareSum.z / sampleCount - mean.z * mean.z)),
    )

    private fun adaptiveThreshold(
        noise: Vector3,
        minimum: Double,
        maximum: Double,
    ): Double = (NOISE_THRESHOLD_MULTIPLIER * noise.magnitude()).coerceIn(minimum, maximum)

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
        const val CALIBRATION_DURATION_NANOS = 3_000_000_000L
        const val MIN_CALIBRATION_SAMPLES = 30
        const val MAX_STATIONARY_GYROSCOPE = 0.15
        const val MAX_STATIONARY_ACCELERATION = 0.5

        const val NANOS_PER_SECOND = 1_000_000_000.0
        const val ADAPTIVE_BIAS_TIME_CONSTANT_SECONDS = 10.0
        const val MAX_ADAPTIVE_GAP_NANOS = 100_000_000L
        const val NOISE_THRESHOLD_MULTIPLIER = 4.0
        const val MIN_ADAPTIVE_ACCELERATION_THRESHOLD = 0.04
        const val MAX_ADAPTIVE_ACCELERATION_THRESHOLD = 0.10
        const val MIN_ADAPTIVE_GYROSCOPE_THRESHOLD = 0.015
        const val MAX_ADAPTIVE_GYROSCOPE_THRESHOLD = 0.05
    }
}

private val ZERO_VECTOR = Vector3(x = 0.0, y = 0.0, z = 0.0)
