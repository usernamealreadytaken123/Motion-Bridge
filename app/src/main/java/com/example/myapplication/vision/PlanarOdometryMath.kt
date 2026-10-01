package com.example.myapplication.vision

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/** A similarity transform that maps points from the previous frame to the current one. */
data class ImageSimilarityTransform(
    val a00: Double,
    val a01: Double,
    val translationX: Double,
    val a10: Double,
    val a11: Double,
    val translationY: Double,
)

data class PlanarOdometryStep(
    /** Camera displacement along the image X axis, in metres. */
    val cameraDeltaXMetres: Double,
    /** Camera displacement along the image Y axis, in metres. */
    val cameraDeltaYMetres: Double,
    /** Camera rotation around the optical axis, in radians. */
    val cameraDeltaHeadingRadians: Double,
    val imageScale: Double,
    val centreFlowXPixels: Double,
    val centreFlowYPixels: Double,
)

data class WorldPlaneDelta(
    val xMetres: Double,
    val yMetres: Double,
)

/**
 * Converts a robust image-plane similarity transform into a metric planar motion.
 *
 * This model intentionally assumes that the camera is perpendicular to a flat surface and that
 * [surfaceDistanceMetres] is constant. Translation of the scene in the image has the opposite sign
 * to translation of the camera.
 */
fun estimatePlanarOdometryStep(
    transform: ImageSimilarityTransform,
    imageWidth: Int,
    imageHeight: Int,
    focalLengthXPixels: Double,
    focalLengthYPixels: Double,
    surfaceDistanceMetres: Double,
    principalPointXPixels: Double = (imageWidth - 1) / 2.0,
    principalPointYPixels: Double = (imageHeight - 1) / 2.0,
): PlanarOdometryStep {
    require(imageWidth > 0 && imageHeight > 0)
    require(focalLengthXPixels > 0.0 && focalLengthYPixels > 0.0)
    require(surfaceDistanceMetres > 0.0)

    val centreX = principalPointXPixels
    val centreY = principalPointYPixels
    val mappedCentreX = transform.a00 * centreX +
        transform.a01 * centreY +
        transform.translationX
    val mappedCentreY = transform.a10 * centreX +
        transform.a11 * centreY +
        transform.translationY
    val centreFlowX = mappedCentreX - centreX
    val centreFlowY = mappedCentreY - centreY

    val imageRotation = atan2(transform.a10, transform.a00)
    val scaleX = hypot(transform.a00, transform.a10)
    val scaleY = hypot(transform.a01, transform.a11)

    return PlanarOdometryStep(
        cameraDeltaXMetres = -surfaceDistanceMetres * centreFlowX / focalLengthXPixels,
        cameraDeltaYMetres = -surfaceDistanceMetres * centreFlowY / focalLengthYPixels,
        cameraDeltaHeadingRadians = -imageRotation,
        imageScale = (scaleX + scaleY) / 2.0,
        centreFlowXPixels = centreFlowX,
        centreFlowYPixels = centreFlowY,
    )
}

/** Rotates a camera-plane displacement into the coordinate frame captured when tracking started. */
fun cameraDeltaToWorld(
    cameraDeltaXMetres: Double,
    cameraDeltaYMetres: Double,
    headingRadians: Double,
): WorldPlaneDelta {
    val cosine = cos(headingRadians)
    val sine = sin(headingRadians)
    return WorldPlaneDelta(
        xMetres = cosine * cameraDeltaXMetres - sine * cameraDeltaYMetres,
        yMetres = sine * cameraDeltaXMetres + cosine * cameraDeltaYMetres,
    )
}
