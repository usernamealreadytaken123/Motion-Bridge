package com.example.myapplication.sensor

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.processing.normalized
import com.example.myapplication.processing.slerp
import java.util.ArrayDeque

data class FrameOrientation(val quaternion: QuaternionData, val bracketGapNanos: Long)

/** Sensor-clock samples, independent of the conflated/throttled UI StateFlow. No extrapolation. */
class OrientationHistory(private val capacity: Int = 512, private val maxGapNanos: Long = 80_000_000L) {
    private data class Sample(val timestamp: Long, val quaternion: QuaternionData)
    private val samples = ArrayDeque<Sample>()

    init { require(capacity >= 2 && maxGapNanos > 0) }

    @Synchronized
    fun add(timestampNanos: Long, quaternion: QuaternionData) {
        val norm = quaternion.w * quaternion.w + quaternion.x * quaternion.x +
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        if (timestampNanos <= 0 || !norm.isFinite() || norm < 1e-12) return
        if (samples.peekLast()?.timestamp?.let { timestampNanos <= it } == true) return
        samples.addLast(Sample(timestampNanos, quaternion.normalized()))
        while (samples.size > capacity) samples.removeFirst()
    }

    @Synchronized
    fun at(timestampNanos: Long): FrameOrientation? {
        if (samples.isEmpty() || timestampNanos < samples.first.timestamp || timestampNanos > samples.last.timestamp) return null
        var before: Sample? = null
        for (after in samples) {
            if (after.timestamp == timestampNanos) return FrameOrientation(after.quaternion, 0)
            if (after.timestamp > timestampNanos) {
                val start = before ?: return null
                val gap = after.timestamp - start.timestamp
                if (gap > maxGapNanos) return null
                val fraction = (timestampNanos - start.timestamp).toDouble() / gap
                return FrameOrientation(start.quaternion.slerp(after.quaternion, fraction), gap)
            }
            before = after
        }
        return null
    }

    @Synchronized
    fun clear() = samples.clear()
}
