package com.example.myapplication

import com.example.myapplication.vision.ImageSimilarityTransform
import com.example.myapplication.vision.cameraDeltaToWorld
import com.example.myapplication.vision.estimatePlanarOdometryStep
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import org.junit.Assert.assertEquals
import org.junit.Test

class PlanarOdometryMathTest {
    @Test
    fun fortyPixelSceneShiftAtOneMetreMeansFiveCentimetresOfCameraMotion() {
        val step = estimatePlanarOdometryStep(
            transform = translationTransform(xPixels = -40.0, yPixels = 16.0),
            imageWidth = 640,
            imageHeight = 480,
            focalLengthXPixels = 800.0,
            focalLengthYPixels = 800.0,
            surfaceDistanceMetres = 1.0,
        )

        assertEquals(0.05, step.cameraDeltaXMetres, 0.000001)
        assertEquals(-0.02, step.cameraDeltaYMetres, 0.000001)
    }

    @Test
    fun rotationAroundImageCentreDoesNotLookLikeTranslation() {
        val angle = Math.toRadians(12.0)
        val centreX = (640 - 1) / 2.0
        val centreY = (480 - 1) / 2.0
        val cosine = cos(angle)
        val sine = sin(angle)
        val transform = ImageSimilarityTransform(
            a00 = cosine,
            a01 = -sine,
            translationX = centreX - cosine * centreX + sine * centreY,
            a10 = sine,
            a11 = cosine,
            translationY = centreY - sine * centreX - cosine * centreY,
        )

        val step = estimatePlanarOdometryStep(
            transform = transform,
            imageWidth = 640,
            imageHeight = 480,
            focalLengthXPixels = 800.0,
            focalLengthYPixels = 800.0,
            surfaceDistanceMetres = 1.0,
        )

        assertEquals(0.0, step.cameraDeltaXMetres, 0.000001)
        assertEquals(0.0, step.cameraDeltaYMetres, 0.000001)
        assertEquals(-angle, step.cameraDeltaHeadingRadians, 0.000001)
    }

    @Test
    fun cameraDeltaIsAccumulatedInInitialWorldAxes() {
        val result = cameraDeltaToWorld(
            cameraDeltaXMetres = 1.0,
            cameraDeltaYMetres = 0.0,
            headingRadians = PI / 2.0,
        )

        assertEquals(0.0, result.xMetres, 0.000001)
        assertEquals(1.0, result.yMetres, 0.000001)
    }

    private fun translationTransform(xPixels: Double, yPixels: Double) =
        ImageSimilarityTransform(
            a00 = 1.0,
            a01 = 0.0,
            translationX = xPixels,
            a10 = 0.0,
            a11 = 1.0,
            translationY = yPixels,
        )
}
