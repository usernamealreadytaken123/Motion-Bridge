package com.example.myapplication.model

import java.util.Locale

data class Vector3(
    val x: Double,
    val y: Double,
    val z: Double,
)

data class QuaternionData(
    val w: Double,
    val x: Double,
    val y: Double,
    val z: Double,
)

data class MotionPacket(
    val sequence: Long,
    val timestamp: Double,
    val quaternion: QuaternionData,
    val gyroscope: Vector3,
    val linearAcceleration: Vector3,
) {
    fun toJson(): String = String.format(
        Locale.US,
        """{"sequence":%d,"timestamp":%.3f,"quaternion":{"w":%.4f,"x":%.4f,"y":%.4f,"z":%.4f},"gyroscope":{"x":%.4f,"y":%.4f,"z":%.4f},"linear_acceleration":{"x":%.4f,"y":%.4f,"z":%.4f}}""",
        sequence,
        timestamp,
        quaternion.w,
        quaternion.x,
        quaternion.y,
        quaternion.z,
        gyroscope.x,
        gyroscope.y,
        gyroscope.z,
        linearAcceleration.x,
        linearAcceleration.y,
        linearAcceleration.z,
    )

    companion object {
        fun testPacket(sequence: Long, timestamp: Double) = MotionPacket(
            sequence = sequence,
            timestamp = timestamp,
            quaternion = QuaternionData(
                w = 0.9238,
                x = 0.1021,
                y = 0.3015,
                z = 0.2057,
            ),
            gyroscope = Vector3(
                x = 0.01,
                y = 0.24,
                z = -0.12,
            ),
            linearAcceleration = Vector3(
                x = 0.15,
                y = -0.04,
                z = 0.82,
            ),
        )
    }
}
