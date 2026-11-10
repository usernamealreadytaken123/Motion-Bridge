package com.example.myapplication.vision

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.Vector3
import com.example.myapplication.processing.rotateDeviceVectorToWorld
import kotlin.math.acos
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

data class ImagePoint(val x: Double, val y: Double)
data class PointMatch(val previous: ImagePoint, val current: ImagePoint)

/** Nominal back-camera mounting. No display-zeroing or scene-coordinate quaternion is used. */
data class GroundCameraPose(
    val deviceToWorld: QuaternionData,
    val sensorOrientationDegrees: Int,
    val imageRotationDegrees: Int,
) {
    init {
        require(sensorOrientationDegrees in setOf(0, 90, 180, 270))
        require(imageRotationDegrees in setOf(0, 90, 180, 270))
        val norm = deviceToWorld.w * deviceToWorld.w + deviceToWorld.x * deviceToWorld.x +
            deviceToWorld.y * deviceToWorld.y + deviceToWorld.z * deviceToWorld.z
        require(norm.isFinite() && norm > 1e-12)
    }

    // Undo the ImageProxy pixel rotation, then map natural upright pixels to device axes.
    // Precompute columns once per frame, not quaternion products for every tracked point.
    private val angle = Math.toRadians((sensorOrientationDegrees - imageRotationDegrees).toDouble())
    private val right = deviceToWorld.rotateDeviceVectorToWorld(Vector3(cos(angle), -sin(angle), 0.0))
    private val down = deviceToWorld.rotateDeviceVectorToWorld(Vector3(-sin(angle), -cos(angle), 0.0))
    private val forward = deviceToWorld.rotateDeviceVectorToWorld(Vector3(0.0, 0.0, -1.0))

    fun rayToWorld(ray: Vector3) = Vector3(
        right.x * ray.x + down.x * ray.y + forward.x * ray.z,
        right.y * ray.x + down.y * ray.y + forward.y * ray.z,
        right.z * ray.x + down.z * ray.y + forward.z * ray.z,
    )

    val tiltDegrees: Double
        get() = Math.toDegrees(acos((-rayToWorld(Vector3(0.0, 0.0, 1.0)).z).coerceIn(-1.0, 1.0)))
}

/** Fixed horizontal axes: X follows initial image-right projected onto the floor; Y image-down. */
data class GroundAxes(val rightX: Double, val rightY: Double, val referenceImageRotation: Int = 0) {
    fun fromWorld(x: Double, y: Double) = WorldPlaneDelta(
        x * rightX + y * rightY,
        x * rightY - y * rightX,
    )

    fun heading(pose: GroundCameraPose): Double {
        val right = pose.copy(imageRotationDegrees = referenceImageRotation).rayToWorld(Vector3(1.0, 0.0, 0.0))
        val local = fromWorld(right.x, right.y)
        return atan2(local.yMetres, local.xMetres)
    }

    companion object {
        fun fromPose(pose: GroundCameraPose): GroundAxes? {
            val right = pose.rayToWorld(Vector3(1.0, 0.0, 0.0))
            val length = hypot(right.x, right.y)
            return if (length < 0.2) null else GroundAxes(right.x / length, right.y / length, pose.imageRotationDegrees)
        }
    }
}

/** Ray-plane intersection relative to the vertical foot of the camera, in ENU horizontal axes. */
fun projectToGround(point: ImagePoint, intrinsics: FrameIntrinsics, pose: GroundCameraPose,
                    height: Double): WorldPlaneDelta? {
    if (!height.isFinite() || height <= 0 || !point.x.isFinite() || !point.y.isFinite()) return null
    if (!intrinsics.focalLengthXPixels.isFinite() || intrinsics.focalLengthXPixels <= 0 ||
        !intrinsics.focalLengthYPixels.isFinite() || intrinsics.focalLengthYPixels <= 0 ||
        !intrinsics.principalPointXPixels.isFinite() || !intrinsics.principalPointYPixels.isFinite()) return null
    val ray = pose.rayToWorld(Vector3(
        (point.x - intrinsics.principalPointXPixels) / intrinsics.focalLengthXPixels,
        (point.y - intrinsics.principalPointYPixels) / intrinsics.focalLengthYPixels, 1.0,
    ))
    val length = sqrt(ray.x * ray.x + ray.y * ray.y + ray.z * ray.z)
    if (!length.isFinite() || -ray.z / length < 0.35) return null
    val result = WorldPlaneDelta(-height * ray.x / ray.z, -height * ray.y / ray.z)
    return result.takeIf { it.xMetres.isFinite() && it.yMetres.isFinite() }
}

data class GroundMotionEstimate(
    val delta: WorldPlaneDelta,
    val inlierIndices: List<Int>,
    val residualMetres: Double,
    val metresPerPixel: Double,
)

/** Robust common ground translation; unlike an image similarity, permits perspective changes. */
fun estimateGroundMotion(matches: List<PointMatch>, intrinsics: FrameIntrinsics,
                         previousPose: GroundCameraPose, currentPose: GroundCameraPose,
                         height: Double, currentHeight: Double = height): GroundMotionEstimate? {
    data class Candidate(val index: Int, val dx: Double, val dy: Double, val tolerance: Double, val sensitivity: Double)
    fun sensitivity(point: ImagePoint, pose: GroundCameraPose, origin: WorldPlaneDelta, height: Double): Double? {
        val x = projectToGround(point.copy(x = point.x + 1), intrinsics, pose, height) ?: return null
        val y = projectToGround(point.copy(y = point.y + 1), intrinsics, pose, height) ?: return null
        return maxOf(hypot(x.xMetres - origin.xMetres, x.yMetres - origin.yMetres),
            hypot(y.xMetres - origin.xMetres, y.yMetres - origin.yMetres))
    }
    val candidates = matches.mapIndexedNotNull { index, match ->
        val old = projectToGround(match.previous, intrinsics, previousPose, height) ?: return@mapIndexedNotNull null
        val new = projectToGround(match.current, intrinsics, currentPose, currentHeight) ?: return@mapIndexedNotNull null
        val noise = maxOf(sensitivity(match.previous, previousPose, old, height) ?: return@mapIndexedNotNull null,
            sensitivity(match.current, currentPose, new, currentHeight) ?: return@mapIndexedNotNull null)
        Candidate(index, old.xMetres - new.xMetres, old.yMetres - new.yMetres,
            maxOf(maxOf(height, currentHeight) * 0.002, noise * 2.5), noise)
    }
    if (candidates.size < 3) return null
    fun agrees(candidate: Candidate, dx: Double, dy: Double): Boolean {
        val ex = candidate.dx - dx
        val ey = candidate.dy - dy
        return ex * ex + ey * ey <= candidate.tolerance * candidate.tolerance
    }
    // Deterministic translation RANSAC: one correspondence defines a hypothesis.
    val seed = candidates.maxBy { candidate ->
        candidates.count { agrees(it, candidate.dx, candidate.dy) }
    }
    fun inliers(dx: Double, dy: Double) = candidates.filter { agrees(it, dx, dy) }
    val initial = inliers(seed.dx, seed.dy)
    if (initial.size < 3 || initial.size.toDouble() / matches.size < 0.6) return null
    val dx = median(initial.map { it.dx })
    val dy = median(initial.map { it.dy })
    val accepted = inliers(dx, dy)
    if (accepted.size < 3 || accepted.size.toDouble() / matches.size < 0.6) return null
    return GroundMotionEstimate(
        WorldPlaneDelta(dx, dy), accepted.map { it.index },
        sqrt(accepted.sumOf { (it.dx - dx) * (it.dx - dx) + (it.dy - dy) * (it.dy - dy) } / accepted.size),
        median(accepted.map { it.sensitivity }),
    )
}

private fun median(values: List<Double>): Double {
    val sorted = values.sorted()
    return (sorted[(sorted.size - 1) / 2] + sorted[sorted.size / 2]) / 2
}
