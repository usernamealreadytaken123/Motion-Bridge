package com.example.myapplication

import com.example.myapplication.lidar.*
import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.Vector3
import com.example.myapplication.sensor.FrameOrientation
import kotlin.math.cos
import kotlin.math.sin
import org.junit.Assert.*
import org.junit.Test

class LidarHeightHistoryTest {
    private val identity = QuaternionData(1.0, 0.0, 0.0, 0.0)
    private val base = 5_000_000_000L
    private val pose: (Long) -> FrameOrientation? = { FrameOrientation(identity, 20_000_000) }
    private fun reading(t: Double, d: Double = 1.0, session: String = "a") = PhyphoxReading(session, true, d, t)
    private fun feed(history: LidarHeightHistory, i: Int, depth: Double = 1.0) =
        history.accept(reading(i * .1, depth), base + i * 100_000_000L, base + i * 100_000_000L + 20_000_000L)
    private fun ready() = LidarHeightHistory().also { for (i in 0..2) feed(it, i) }

    @Test fun depthIsProjectedWithTiltAndRotatedLeverArm() {
        val half = Math.toRadians(30.0) / 2
        val tilted = QuaternionData(cos(half), sin(half), 0.0, 0.0)
        assertEquals(2 * cos(Math.toRadians(30.0)) + .05,
            lidarCameraHeight(2.0, tilted, Vector3(0.0, .1, 0.0))!!, 1e-9)
        assertEquals(1.05, lidarCameraHeight(1.0, identity, Vector3(0.0, 0.0, .05))!!, 1e-9)
        assertNull(lidarCameraHeight(Double.NaN, identity, Vector3(0.0, 0.0, 0.0)))
        assertNull(lidarCameraHeight(1.0, QuaternionData(0.0, 1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
    }

    @Test fun bracketsFrameAndInterpolatesHeightWithoutClockEpochAssumption() {
        val history = LidarHeightHistory()
        for (i in 0..2) feed(history, i, 1 + i * .02)
        val result = history.at(base + 160_000_000, base + 230_000_000, pose)!!
        assertEquals(1.03, result.heightMetres, 1e-9)
        assertEquals(.15, result.experimentTimeSeconds, 1e-9)
        assertEquals(50.0, result.ageMillis, 1e-9)
        assertEquals(20.0, result.roundTripMillis, 1e-9)
        val otherEpoch = LidarHeightHistory()
        for (i in 0..2) otherEpoch.accept(reading(10000 + i * .1),
            base + i * 100_000_000L, base + i * 100_000_000L + 20_000_000)
        assertEquals(1.0, otherEpoch.at(base + 160_000_000, base + 230_000_000, pose)!!.heightMetres, 1e-9)
    }

    @Test fun noExtrapolationNoMissingOrientationAndNoStaleHeights() {
        val history = ready()
        assertNull(history.at(base, base + 230_000_000, pose))
        assertNull(history.at(base + 220_000_000, base + 230_000_000, pose))
        assertNull(history.at(base + 160_000_000, base + 230_000_000) { null })
        assertNull(history.at(base + 160_000_000, base + 800_000_000, pose))
    }

    @Test fun repeatedTimestampDoesNotRefreshStationaryOrFrozenData() {
        val history = ready()
        assertFalse(history.accept(reading(.2), base + 700_000_000, base + 720_000_000))
        assertFalse(history.isFresh(base + 730_000_000))
        // A stationary depth with advancing time IS a new measurement.
        val stationary = ready()
        assertTrue(feed(stationary, 3))
        assertTrue(stationary.isFresh(base + 330_000_000))
    }

    @Test fun pauseSessionChangeDisconnectAndTimeResetBreakReferenceContinuity() {
        for (operation in 0..3) {
            val history = ready()
            val old = history.at(base + 160_000_000, base + 230_000_000, pose)!!
            when (operation) {
                0 -> history.accept(PhyphoxReading("a", false, null, null), base + 300_000_000, base + 320_000_000)
                1 -> history.accept(reading(.3, session = "b"), base + 300_000_000, base + 320_000_000)
                2 -> history.invalidate()
                3 -> history.accept(reading(.0), base + 300_000_000, base + 320_000_000)
            }
            assertFalse(history.isFresh(base + 330_000_000))
            for (i in 4..6) feed(history, i)
            val recovered = history.at(base + 560_000_000, base + 630_000_000, pose)!!
            assertTrue(recovered.generation > old.generation)
        }
    }

    @Test fun slowHttpAndTooLargeSamplingGapsAreRejected() {
        val history = ready()
        history.accept(reading(.3), base + 300_000_000, base + 500_000_000)
        assertFalse(history.isFresh(base + 510_000_000))
        val sparse = LidarHeightHistory()
        for (i in 0..2) sparse.accept(reading(i * .3), base + i * 300_000_000L, base + i * 300_000_000L + 20_000_000)
        assertNull(sparse.at(base + 410_000_000, base + 630_000_000, pose))
    }

    @Test fun invalidRangeAndSuddenSurfaceJumpDoNotInterpolate() {
        val history = ready()
        feed(history, 3, 2.0)
        assertNull(history.at(base + 260_000_000, base + 330_000_000, pose))
        history.accept(reading(.4, Double.NaN), base + 400_000_000, base + 420_000_000)
        assertFalse(history.isFresh(base + 430_000_000))
    }
}
