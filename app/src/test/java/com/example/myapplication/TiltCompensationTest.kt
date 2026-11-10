package com.example.myapplication

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.Vector3
import com.example.myapplication.processing.inverse
import com.example.myapplication.processing.multiply
import com.example.myapplication.processing.rotateDeviceVectorToWorld
import com.example.myapplication.vision.*
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan
import org.junit.Assert.*
import org.junit.Test

class TiltCompensationTest {
    @Test fun sharedPcGeometryCases() {
        val cases = org.json.JSONArray(checkNotNull(javaClass.getResource("/ground_motion_cases.json")).readText())
        for (i in 0 until cases.length()) {
            val c = cases.getJSONObject(i)
            fun casePose(key: String): GroundCameraPose {
                val a = c.getJSONArray(key)
                return pose(a.getDouble(0), a.getDouble(1), a.getDouble(2))
            }
            val old = casePose("old_angles")
            val new = casePose("angles")
            val h0 = c.getDouble("old_height")
            val h = c.getDouble("height")
            val dx = c.getDouble("dx")
            val dy = c.getDouble("dy")
            val result = checkNotNull(estimateGroundMotion(matches(old, new, dx, dy, h0, h), k, old, new, h0, h))
            assertEquals(dx, result.delta.xMetres, 1e-9)
            assertEquals(dy, result.delta.yMetres, 1e-9)
        }
    }
    private val k = FrameIntrinsics(800.0, 780.0, 319.5, 239.5, false)
    private val identity = QuaternionData(1.0, 0.0, 0.0, 0.0)
    private fun rotation(axis: Int, degrees: Double): QuaternionData {
        val half = Math.toRadians(degrees) / 2
        return QuaternionData(cos(half), if (axis == 0) sin(half) else 0.0,
            if (axis == 1) sin(half) else 0.0, if (axis == 2) sin(half) else 0.0)
    }
    private fun pose(pitch: Double = 0.0, roll: Double = 0.0, yaw: Double = 0.0) = GroundCameraPose(
        rotation(2, yaw).multiply(rotation(0, pitch)).multiply(rotation(1, roll)), 90, 90)

    // Forward camera model, independent of the inverse ray-ground function under test.
    private fun observe(point: Vector3, camera: Vector3, pose: GroundCameraPose): ImagePoint {
        val device = pose.deviceToWorld.inverse().rotateDeviceVectorToWorld(
            Vector3(point.x - camera.x, point.y - camera.y, point.z - camera.z))
        val angle = Math.toRadians((pose.imageRotationDegrees - pose.sensorOrientationDegrees).toDouble())
        val x = cos(angle) * device.x + sin(angle) * device.y
        val y = sin(angle) * device.x - cos(angle) * device.y
        val z = -device.z
        check(z > 0)
        return ImagePoint(k.principalPointXPixels + k.focalLengthXPixels * x / z,
            k.principalPointYPixels + k.focalLengthYPixels * y / z)
    }
    private fun matches(old: GroundCameraPose, new: GroundCameraPose, dx: Double = 0.0,
                        dy: Double = 0.0, h: Double = 1.0, nextHeight: Double = h): List<PointMatch> {
        val centre = old.deviceToWorld.rotateDeviceVectorToWorld(Vector3(0.0, 0.0, -1.0))
        val cx = -h * centre.x / centre.z
        val cy = -h * centre.y / centre.z
        return (-3..3).flatMap { ix -> (-3..3).map { iy ->
            val point = Vector3(cx + ix * h * 0.08, cy + iy * h * 0.08, 0.0)
            PointMatch(observe(point, Vector3(0.0, 0.0, h), old),
                observe(point, Vector3(dx, dy, nextHeight), new))
        } }
    }

    @Test fun perpendicularCameraMatchesLegacyMetricScaleAndSigns() {
        val p = pose()
        val estimate = checkNotNull(estimateGroundMotion(matches(p, p, 0.05, 0.02), k, p, p, 1.0))
        val delta = checkNotNull(GroundAxes.fromPose(p)).fromWorld(estimate.delta.xMetres, estimate.delta.yMetres)
        assertEquals(0.05, delta.xMetres, 1e-9)
        assertEquals(-0.02, delta.yMetres, 1e-9)
        val legacy = estimatePlanarOdometryStep(ImageSimilarityTransform(1.0, 0.0, -40.0, 0.0, 1.0, 15.6),
            640, 480, 800.0, 780.0, 1.0)
        assertEquals(legacy.cameraDeltaXMetres, delta.xMetres, 1e-9)
        assertEquals(legacy.cameraDeltaYMetres, delta.yMetres, 1e-9)
    }

    @Test fun fixedTiltRecoversMetricTranslationAtDifferentHeightsAndYaws() {
        for (tilt in listOf(0.0, 15.0, 30.0, 45.0)) for (height in listOf(0.3, 1.0, 2.0))
            for (yaw in listOf(0.0, 67.0, -110.0)) {
                val p = pose(tilt, 8.0, yaw)
                val estimate = checkNotNull(estimateGroundMotion(matches(p, p, .07, -.03, height), k, p, p, height))
                assertEquals(.07, estimate.delta.xMetres, 1e-8)
                assertEquals(-.03, estimate.delta.yMetres, 1e-8)
            }
    }

    @Test fun pureRotationDoesNotCreateTranslation() {
        val old = pose(20.0, -8.0, 35.0)
        val new = pose(30.0, 7.0, 45.0)
        val result = checkNotNull(estimateGroundMotion(matches(old, new), k, old, new, 1.0))
        assertEquals(0.0, result.delta.xMetres, 1e-9)
        assertEquals(0.0, result.delta.yMetres, 1e-9)
    }

    @Test fun simultaneousPitchRollYawAndTranslationRecoverTheGroundMotion() {
        val old = pose(20.0, -8.0, 35.0)
        val new = pose(30.0, 7.0, 45.0)
        val result = checkNotNull(estimateGroundMotion(matches(old, new, -.11, .06), k, old, new, 1.0))
        assertEquals(-.11, result.delta.xMetres, 1e-9)
        assertEquals(.06, result.delta.yMetres, 1e-9)
    }

    @Test fun measuredHeightChangeDoesNotBecomeFalseHorizontalTranslation() {
        for (tilt in listOf(0.0, 15.0, 30.0, 45.0)) for (nextHeight in listOf(.8, 1.05, 1.3)) {
            val old = pose(tilt, -5.0, 20.0)
            val new = pose(tilt + 2.0, 3.0, 23.0)
            val result = checkNotNull(estimateGroundMotion(matches(old, new, nextHeight = nextHeight),
                k, old, new, 1.0, nextHeight))
            assertEquals(0.0, result.delta.xMetres, 1e-9)
            assertEquals(0.0, result.delta.yMetres, 1e-9)
        }
    }

    @Test fun translationRotationAndHeightChangeAreRecoveredTogether() {
        val old = pose(25.0, -8.0, 35.0)
        val new = pose(30.0, 7.0, 45.0)
        val result = checkNotNull(estimateGroundMotion(matches(old, new, -.11, .06, nextHeight = 1.2),
            k, old, new, 1.0, 1.2))
        assertEquals(-.11, result.delta.xMetres, 1e-9)
        assertEquals(.06, result.delta.yMetres, 1e-9)
    }

    @Test fun pitchedCentreRayIntersectsAtHeightTimesTangent() {
        val p = pose(30.0)
        val result = checkNotNull(projectToGround(ImagePoint(k.principalPointXPixels, k.principalPointYPixels), k, p, 2.0))
        assertEquals(0.0, result.xMetres, 1e-10)
        assertEquals(2 * tan(Math.toRadians(30.0)), result.yMetres, 1e-10)
        assertEquals(30.0, p.tiltDegrees, 1e-10)
    }

    @Test fun nominalRearCameraMountAndUprightRotationHaveKnownAxes() {
        val raw = GroundCameraPose(identity, 90, 0)
        assertVector(Vector3(0.0, -1.0, 0.0), raw.rayToWorld(Vector3(1.0, 0.0, 0.0)))
        assertVector(Vector3(-1.0, 0.0, 0.0), raw.rayToWorld(Vector3(0.0, 1.0, 0.0)))
        assertVector(Vector3(0.0, 0.0, -1.0), raw.rayToWorld(Vector3(0.0, 0.0, 1.0)))
        assertVector(Vector3(1.0, 0.0, 0.0), pose().rayToWorld(Vector3(1.0, 0.0, 0.0)))
        assertVector(Vector3(0.0, -1.0, 0.0), pose().rayToWorld(Vector3(0.0, 1.0, 0.0)))
    }

    @Test fun imageRotationDoesNotChangePhysicalRayOrHeading() {
        val calibration = SensorCameraCalibration(640.0, 480.0, 800.0, 780.0, 319.5, 239.5, false)
        val rawPixel = ImagePoint(220.0, 180.0)
        for (sensor in listOf(0, 90, 180, 270)) {
            val p = pose(20.0, 10.0).copy(sensorOrientationDegrees = sensor, imageRotationDegrees = 0)
            val axes = checkNotNull(GroundAxes.fromPose(p))
            val expected = checkNotNull(projectToGround(rawPixel, calibration.forFrame(640, 480, 0), p, 1.0))
            for (rotation in listOf(0, 90, 180, 270)) {
                val pixel = when (rotation) {
                    90 -> ImagePoint(479.0 - rawPixel.y, rawPixel.x)
                    180 -> ImagePoint(639.0 - rawPixel.x, 479.0 - rawPixel.y)
                    270 -> ImagePoint(rawPixel.y, 639.0 - rawPixel.x)
                    else -> rawPixel
                }
                val current = p.copy(imageRotationDegrees = rotation)
                val actual = checkNotNull(projectToGround(pixel, calibration.forFrame(640, 480, rotation), current, 1.0))
                assertEquals(expected.xMetres, actual.xMetres, 1e-9)
                assertEquals(expected.yMetres, actual.yMetres, 1e-9)
                assertEquals(0.0, axes.heading(current), 1e-9)
            }
        }
    }

    @Test fun outliersDoNotBiasTranslation() {
        val p = pose(30.0)
        val good = matches(p, p, .04, -.02)
        val mixed = good.mapIndexed { i, match ->
            if (i < 12) match.copy(current = match.current.copy(x = match.current.x + 100 + i * 3)) else match
        }
        val result = checkNotNull(estimateGroundMotion(mixed, k, p, p, 1.0))
        assertEquals(.04, result.delta.xMetres, 1e-9)
        assertEquals(-.02, result.delta.yMetres, 1e-9)
        assertTrue(result.inlierIndices.none { it < 12 })
    }

    @Test fun inconsistentMotionAndLargeHeightChangeAreRejected() {
        val p = pose()
        val mixed = matches(p, p).mapIndexed { i, match ->
            match.copy(current = ImagePoint(match.current.x + i * 20, match.current.y + i * 10))
        }
        assertNull(estimateGroundMotion(mixed, k, p, p, 1.0))
        assertNull(estimateGroundMotion(matches(p, p, nextHeight = 1.3), k, p, p, 1.0))
    }

    @Test fun raysNearHorizonBehindCameraAndInvalidInputsAreRejected() {
        val centre = ImagePoint(319.5, 239.5)
        assertNull(projectToGround(centre, k, pose(90.0), 1.0))
        assertNull(projectToGround(centre, k, pose(180.0), 1.0))
        assertNull(projectToGround(centre, k, pose(), 0.0))
        assertNull(projectToGround(centre, k.copy(focalLengthXPixels = Double.NaN), pose(), 1.0))
        assertNull(projectToGround(ImagePoint(Double.NaN, 0.0), k, pose(), 1.0))
        assertNull(estimateGroundMotion(emptyList(), k, pose(), pose(), 1.0))
    }

    @Test fun configuredHeightSetsScaleAndCannotCorrectAnIncorrectHeight() {
        val p = pose(30.0)
        val result = checkNotNull(estimateGroundMotion(matches(p, p, .1, .02, h = 1.0), k, p, p, 2.0))
        assertEquals(.2, result.delta.xMetres, 1e-9)
        assertEquals(.04, result.delta.yMetres, 1e-9)
    }

    @Test fun worldBasisStaysFixedWhenDeviceYawChanges() {
        val axes = checkNotNull(GroundAxes.fromPose(pose(yaw = 90.0)))
        val delta = axes.fromWorld(0.0, .1)
        assertEquals(.1, delta.xMetres, 1e-9)
        assertEquals(0.0, delta.yMetres, 1e-9)
        assertEquals(-Math.toRadians(20.0), axes.heading(pose(yaw = 110.0)), 1e-9)
    }

    private fun assertVector(expected: Vector3, actual: Vector3) {
        assertEquals(expected.x, actual.x, 1e-9)
        assertEquals(expected.y, actual.y, 1e-9)
        assertEquals(expected.z, actual.z, 1e-9)
    }
}
