package com.example.myapplication.processing

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.Vector3

internal fun QuaternionData.rotateDeviceVectorToWorld(vector: Vector3): Vector3 {
    val orientation = normalized()
    val vectorQuaternion = QuaternionData(
        w = 0.0,
        x = vector.x,
        y = vector.y,
        z = vector.z,
    )
    val rotated = orientation
        .multiply(vectorQuaternion)
        .multiply(orientation.inverse())

    return Vector3(
        x = rotated.x,
        y = rotated.y,
        z = rotated.z,
    )
}

internal fun QuaternionData.rotateDeviceVectorToSceneWorld(vector: Vector3): Vector3 {
    val androidWorld = rotateDeviceVectorToWorld(vector)

    // Android: X — восток, Y — север, Z — вверх.
    // Сцена OpenGL: X — восток, Y — вверх, Z — юг.
    return Vector3(
        x = androidWorld.x,
        y = androidWorld.z,
        z = -androidWorld.y,
    )
}
