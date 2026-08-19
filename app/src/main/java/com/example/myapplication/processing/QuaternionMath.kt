package com.example.myapplication.processing

import com.example.myapplication.model.QuaternionData
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.sin
import kotlin.math.sqrt

internal fun QuaternionData.normalized(): QuaternionData {
    val magnitude = sqrt(w * w + x * x + y * y + z * z)
    if (magnitude < MIN_QUATERNION_MAGNITUDE) return IDENTITY_QUATERNION

    return QuaternionData(
        w = w / magnitude,
        x = x / magnitude,
        y = y / magnitude,
        z = z / magnitude,
    )
}

internal fun QuaternionData.inverse(): QuaternionData {
    val squaredMagnitude = w * w + x * x + y * y + z * z
    if (squaredMagnitude < MIN_QUATERNION_MAGNITUDE) return IDENTITY_QUATERNION

    return QuaternionData(
        w = w / squaredMagnitude,
        x = -x / squaredMagnitude,
        y = -y / squaredMagnitude,
        z = -z / squaredMagnitude,
    )
}

internal fun QuaternionData.multiply(other: QuaternionData): QuaternionData = QuaternionData(
    w = w * other.w - x * other.x - y * other.y - z * other.z,
    x = w * other.x + x * other.w + y * other.z - z * other.y,
    y = w * other.y - x * other.z + y * other.w + z * other.x,
    z = w * other.z + x * other.y - y * other.x + z * other.w,
)

internal fun QuaternionData.slerp(
    target: QuaternionData,
    factor: Double,
): QuaternionData {
    val start = normalized()
    var end = target.normalized()
    val interpolationFactor = factor.coerceIn(0.0, 1.0)
    var dotProduct = start.dot(end)

    // q и -q задают одну ориентацию. Меняем знак, чтобы идти кратчайшим путём.
    if (dotProduct < 0.0) {
        end = end.negated()
        dotProduct = -dotProduct
    }

    if (dotProduct > LINEAR_INTERPOLATION_THRESHOLD) {
        return QuaternionData(
            w = start.w + interpolationFactor * (end.w - start.w),
            x = start.x + interpolationFactor * (end.x - start.x),
            y = start.y + interpolationFactor * (end.y - start.y),
            z = start.z + interpolationFactor * (end.z - start.z),
        ).normalized()
    }

    val initialAngle = acos(dotProduct.coerceIn(-1.0, 1.0))
    val initialAngleSine = sin(initialAngle)
    val startWeight = sin((1.0 - interpolationFactor) * initialAngle) / initialAngleSine
    val endWeight = sin(interpolationFactor * initialAngle) / initialAngleSine

    return QuaternionData(
        w = startWeight * start.w + endWeight * end.w,
        x = startWeight * start.x + endWeight * end.x,
        y = startWeight * start.y + endWeight * end.y,
        z = startWeight * start.z + endWeight * end.z,
    ).normalized()
}

internal fun QuaternionData.angularDistanceTo(other: QuaternionData): Double {
    val dotProduct = abs(normalized().dot(other.normalized())).coerceIn(0.0, 1.0)
    return 2.0 * acos(dotProduct)
}

private fun QuaternionData.dot(other: QuaternionData): Double =
    w * other.w + x * other.x + y * other.y + z * other.z

private fun QuaternionData.negated() = QuaternionData(
    w = -w,
    x = -x,
    y = -y,
    z = -z,
)

internal val IDENTITY_QUATERNION = QuaternionData(
    w = 1.0,
    x = 0.0,
    y = 0.0,
    z = 0.0,
)

private const val MIN_QUATERNION_MAGNITUDE = 1e-9
private const val LINEAR_INTERPOLATION_THRESHOLD = 0.9995
