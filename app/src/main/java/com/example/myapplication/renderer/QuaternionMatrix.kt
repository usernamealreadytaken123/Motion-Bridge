package com.example.myapplication.renderer

import com.example.myapplication.model.QuaternionData
import kotlin.math.sqrt

internal fun QuaternionData.toOpenGlRotationMatrix(): FloatArray {
    val magnitude = sqrt(w * w + x * x + y * y + z * z)
    if (magnitude < MIN_QUATERNION_MAGNITUDE) return IDENTITY_MATRIX.copyOf()

    val normalizedW = w / magnitude
    val normalizedX = x / magnitude
    val normalizedY = y / magnitude
    val normalizedZ = z / magnitude

    val xx = normalizedX * normalizedX
    val yy = normalizedY * normalizedY
    val zz = normalizedZ * normalizedZ
    val xy = normalizedX * normalizedY
    val xz = normalizedX * normalizedZ
    val yz = normalizedY * normalizedZ
    val wx = normalizedW * normalizedX
    val wy = normalizedW * normalizedY
    val wz = normalizedW * normalizedZ

    // OpenGL хранит матрицы по столбцам (column-major).
    return floatArrayOf(
        (1.0 - 2.0 * (yy + zz)).toFloat(),
        (2.0 * (xy + wz)).toFloat(),
        (2.0 * (xz - wy)).toFloat(),
        0f,
        (2.0 * (xy - wz)).toFloat(),
        (1.0 - 2.0 * (xx + zz)).toFloat(),
        (2.0 * (yz + wx)).toFloat(),
        0f,
        (2.0 * (xz + wy)).toFloat(),
        (2.0 * (yz - wx)).toFloat(),
        (1.0 - 2.0 * (xx + yy)).toFloat(),
        0f,
        0f,
        0f,
        0f,
        1f,
    )
}

private const val MIN_QUATERNION_MAGNITUDE = 1e-9

private val IDENTITY_MATRIX = floatArrayOf(
    1f, 0f, 0f, 0f,
    0f, 1f, 0f, 0f,
    0f, 0f, 1f, 0f,
    0f, 0f, 0f, 1f,
)
