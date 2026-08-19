package com.example.myapplication.sensor

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.Vector3
import java.io.Closeable
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

data class SensorAvailability(
    val rotationVector: Boolean,
    val gyroscope: Boolean,
    val linearAcceleration: Boolean,
)

data class SensorState(
    val availability: SensorAvailability,
    val isCollecting: Boolean = false,
    val quaternion: QuaternionData? = null,
    val gyroscope: Vector3? = null,
    val linearAcceleration: Vector3? = null,
    val timestampNanos: Long = 0L,
)

class SensorCollector(context: Context) : SensorEventListener, Closeable {
    private val sensorManager = context.getSystemService(SensorManager::class.java)
    private val rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
    private val gyroscopeSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val linearAccelerationSensor =
        sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)

    private val availability = SensorAvailability(
        rotationVector = rotationVectorSensor != null,
        gyroscope = gyroscopeSensor != null,
        linearAcceleration = linearAccelerationSensor != null,
    )

    private val _state = MutableStateFlow(SensorState(availability = availability))
    val state: StateFlow<SensorState> = _state.asStateFlow()

    private var isStarted = false
    private var latestQuaternion: QuaternionData? = null
    private var latestGyroscope: Vector3? = null
    private var latestLinearAcceleration: Vector3? = null
    private var latestTimestampNanos = 0L
    private var lastPublishedAtNanos = 0L

    @Synchronized
    fun start() {
        if (isStarted) return

        var registeredAtLeastOneSensor = false
        listOfNotNull(
            rotationVectorSensor,
            gyroscopeSensor,
            linearAccelerationSensor,
        ).forEach { sensor ->
            val registered = sensorManager.registerListener(
                this,
                sensor,
                SensorManager.SENSOR_DELAY_GAME,
            )
            registeredAtLeastOneSensor = registered || registeredAtLeastOneSensor
        }

        isStarted = registeredAtLeastOneSensor
        _state.value = _state.value.copy(isCollecting = isStarted)
    }

    @Synchronized
    fun stop() {
        if (!isStarted) return

        sensorManager.unregisterListener(this)
        isStarted = false
        _state.value = _state.value.copy(isCollecting = false)
    }

    override fun onSensorChanged(event: SensorEvent) {
        synchronized(this) {
            when (event.sensor.type) {
                Sensor.TYPE_ROTATION_VECTOR -> {
                    val quaternion = FloatArray(4)
                    SensorManager.getQuaternionFromVector(quaternion, event.values)
                    latestQuaternion = QuaternionData(
                        w = quaternion[0].toDouble(),
                        x = quaternion[1].toDouble(),
                        y = quaternion[2].toDouble(),
                        z = quaternion[3].toDouble(),
                    )
                }

                Sensor.TYPE_GYROSCOPE -> {
                    latestGyroscope = event.values.toVector3()
                }

                Sensor.TYPE_LINEAR_ACCELERATION -> {
                    latestLinearAcceleration = event.values.toVector3()
                }
            }

            latestTimestampNanos = maxOf(latestTimestampNanos, event.timestamp)
            if (
                lastPublishedAtNanos == 0L ||
                event.timestamp - lastPublishedAtNanos >= UI_PUBLISH_PERIOD_NANOS
            ) {
                publishState(event.timestamp)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun close() {
        stop()
    }

    private fun publishState(publishedAtNanos: Long) {
        lastPublishedAtNanos = publishedAtNanos
        _state.value = SensorState(
            availability = availability,
            isCollecting = isStarted,
            quaternion = latestQuaternion,
            gyroscope = latestGyroscope,
            linearAcceleration = latestLinearAcceleration,
            timestampNanos = latestTimestampNanos,
        )
    }

    private fun FloatArray.toVector3() = Vector3(
        x = this[0].toDouble(),
        y = this[1].toDouble(),
        z = this[2].toDouble(),
    )

    private companion object {
        const val UI_PUBLISH_PERIOD_NANOS = 20_000_000L
    }
}
