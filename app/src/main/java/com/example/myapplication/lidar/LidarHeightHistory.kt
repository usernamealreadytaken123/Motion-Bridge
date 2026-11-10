package com.example.myapplication.lidar

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.Vector3
import com.example.myapplication.processing.rotateDeviceVectorToWorld
import com.example.myapplication.sensor.FrameOrientation
import java.util.ArrayDeque
import kotlin.math.cos

data class LidarFrameHeight(
    val heightMetres: Double,
    val depthMetres: Double,
    val experimentTimeSeconds: Double,
    val ageMillis: Double,
    val roundTripMillis: Double,
    val generation: Long,
)

/** Central depth ROI, parallel rear optical axes, horizontal static plane. */
fun lidarCameraHeight(depthMetres: Double, orientation: QuaternionData, lidarToCamera: Vector3): Double? {
    val norm = orientation.w * orientation.w + orientation.x * orientation.x +
        orientation.y * orientation.y + orientation.z * orientation.z
    if (!norm.isFinite() || norm < 1e-12 || !depthMetres.isFinite() || depthMetres !in 0.05..5.0 ||
        listOf(lidarToCamera.x, lidarToCamera.y, lidarToCamera.z).any { !it.isFinite() }) return null
    val forward = orientation.rotateDeviceVectorToWorld(Vector3(0.0, 0.0, -1.0))
    if (-forward.z < cos(Math.toRadians(55.0))) return null
    val offset = orientation.rotateDeviceVectorToWorld(lidarToCamera)
    return (-depthMetres * forward.z + offset.z).takeIf { it.isFinite() && it in 0.05..20.0 }
}

/**
 * Approximate clock alignment, NOT hardware synchronization. The minimum observed
 * (HTTP midpoint - experiment time) includes an unknown camera/analysis latency.
 * No wall-clock subtraction; duplicate source timestamps never refresh the data.
 * Frames must be bracketed by two valid depth measurements; no height extrapolation.
 */
class LidarHeightHistory {
    private data class Sample(val timeNanos: Long, val timeSeconds: Double, val depth: Double,
                              val received: Long, val rtt: Long)
    private val samples = ArrayDeque<Sample>()
    private var session: String? = null
    private var lastSourceTime: Double? = null
    private var offsetNanos: Long? = null
    private var generation = 0L
    private var mount = Vector3(0.0, 0.0, 0.0)

    @Synchronized fun configure(lidarToCamera: Vector3) {
        mount = lidarToCamera
        session = null
        lastSourceTime = null
        invalidate()
    }

    @Synchronized fun invalidate() {
        samples.clear()
        offsetNanos = null
        generation++
    }

    @Synchronized fun accept(reading: PhyphoxReading, requested: Long, received: Long): Boolean {
        if (reading.session != session) {
            invalidate()
            lastSourceTime = null
            session = reading.session
        }
        if (!reading.measuring) { invalidate(); return false }
        val time = reading.experimentTimeSeconds
        val depth = reading.depthMetres
        if (time == null || !time.isFinite() || time !in 0.0..100_000_000.0 ||
            depth == null || !depth.isFinite() || depth !in 0.05..5.0) {
            invalidate(); return false
        }
        val last = lastSourceTime
        if (last != null && time < last) { invalidate(); lastSourceTime = time; return false }
        if (last == time) return false
        lastSourceTime = time
        val rtt = received - requested
        if (requested <= 0 || rtt !in 0..MAX_RTT_NANOS) { invalidate(); return false }
        val previous = samples.peekLast()
        if (previous != null && (received - previous.received !in 1..MAX_STALE_NANOS ||
                (time - previous.timeSeconds) * 1e9 > MAX_STALE_NANOS)) invalidate()
        val timeNanos = (time * 1e9).toLong()
        val candidateOffset = requested + rtt / 2 - timeNanos
        offsetNanos = minOf(offsetNanos ?: candidateOffset, candidateOffset)
        samples.addLast(Sample(timeNanos, time, depth, received, rtt))
        while (samples.size > 64) samples.removeFirst()
        return true
    }

    @Synchronized fun isFresh(now: Long): Boolean = samples.size >= 3 &&
        samples.peekLast()?.let { now - it.received in 0..MAX_STALE_NANOS } == true

    @Synchronized fun at(timestamp: Long, now: Long, orientationAt: (Long) -> FrameOrientation?): LidarFrameHeight? {
        if (!isFresh(now)) return null
        val offset = offsetNanos ?: return null
        var before: Sample? = null
        for (after in samples) {
            val afterTime = after.timeNanos + offset
            if (afterTime < timestamp) { before = after; continue }
            val first = before ?: return null
            val firstTime = first.timeNanos + offset
            val gap = afterTime - firstTime
            if (gap !in 1..MAX_BRACKET_NANOS) return null
            val firstPose = orientationAt(firstTime) ?: return null
            val secondPose = orientationAt(afterTime) ?: return null
            val h0 = lidarCameraHeight(first.depth, firstPose.quaternion, mount) ?: return null
            val h1 = lidarCameraHeight(after.depth, secondPose.quaternion, mount) ?: return null
            // Avoid interpolating a gross range jump or a different surface.
            if (kotlin.math.abs(h1 - h0) > 0.02 + gap / 1e9) return null
            val fraction = (timestamp - firstTime).toDouble() / gap
            return LidarFrameHeight(h0 + fraction * (h1 - h0),
                first.depth + fraction * (after.depth - first.depth),
                first.timeSeconds + fraction * (after.timeSeconds - first.timeSeconds),
                (timestamp - firstTime) / 1e6, maxOf(first.rtt, after.rtt) / 1e6, generation)
        }
        return null
    }

    companion object {
        const val MAX_RTT_NANOS = 150_000_000L
        const val MAX_STALE_NANOS = 500_000_000L
        const val MAX_BRACKET_NANOS = 250_000_000L
        const val FRAME_DELAY_NANOS = 200_000_000L
    }
}
