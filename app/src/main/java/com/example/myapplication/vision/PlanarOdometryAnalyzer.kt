package com.example.myapplication.vision

import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import com.example.myapplication.processing.angularDistanceTo
import com.example.myapplication.sensor.FrameOrientation
import com.example.myapplication.lidar.LidarFrameHeight
import com.example.myapplication.lidar.LidarHeightHistory
import java.nio.ByteBuffer
import kotlin.math.abs
import kotlin.math.hypot
import org.opencv.calib3d.Calib3d
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfByte
import org.opencv.core.MatOfFloat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Size
import org.opencv.core.TermCriteria
import org.opencv.imgproc.Imgproc
import org.opencv.video.Video
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

enum class PlanarTrackingStatus {
    STOPPED,
    STARTING_CAMERA,
    INITIALIZING,
    TRACKING,
    DEGRADED,
    LOST,
    ERROR,
}

enum class PlanarOdometryMode { LEGACY, TILT_COMPENSATED }

data class TiltCameraConfiguration(
    val sensorOrientationDegrees: Int?,
    val timestampsAreRealtime: Boolean,
    val isBackCamera: Boolean,
)

data class FeatureTrack(
    val fromX: Float,
    val fromY: Float,
    val toX: Float,
    val toY: Float,
)

data class PlanarOdometryState(
    val trackingEpoch: Long = 0L,
    val isCameraActive: Boolean = false,
    val status: PlanarTrackingStatus = PlanarTrackingStatus.STOPPED,
    val xMetres: Double = 0.0,
    val yMetres: Double = 0.0,
    val headingRadians: Double = 0.0,
    val lastDeltaXMetres: Double = 0.0,
    val lastDeltaYMetres: Double = 0.0,
    val surfaceDistanceMetres: Double = 1.0,
    val trackedPoints: Int = 0,
    val inliers: Int = 0,
    val inlierRatio: Double = 0.0,
    val imageScale: Double = 1.0,
    val framesPerSecond: Double = 0.0,
    val intrinsicsAreApproximate: Boolean = true,
    val frameWidth: Int = 0,
    val frameHeight: Int = 0,
    val featureTracks: List<FeatureTrack> = emptyList(),
    val message: String = "Камера не запущена",
    val timestampNanos: Long = 0L,
    val mode: PlanarOdometryMode = PlanarOdometryMode.LEGACY,
    val tiltDegrees: Double? = null,
    val orientationGapMillis: Double? = null,
    val groundResidualMetres: Double? = null,
    val usesLidarHeight: Boolean = false,
    val heightIsValid: Boolean = true,
    val lidarDepthMetres: Double? = null,
    val lidarAgeMillis: Double? = null,
    val lidarRoundTripMillis: Double? = null,
    val lidarTimeSeconds: Double? = null,
)

data class FrameIntrinsics(
    val focalLengthXPixels: Double,
    val focalLengthYPixels: Double,
    val principalPointXPixels: Double,
    val principalPointYPixels: Double,
    val isApproximate: Boolean,
)

/**
 * Legacy fixed-height similarity, or orientation-assisted horizontal ray projection.
 */
class PlanarOdometryAnalyzer(
    private val orientationAt: (Long) -> FrameOrientation? = { null },
    private val lidarHeightAt: (Long) -> LidarFrameHeight? = { null },
) : ImageAnalysis.Analyzer {
    private val _state = MutableStateFlow(PlanarOdometryState())
    val state: StateFlow<PlanarOdometryState> = _state.asStateFlow()

    @Volatile
    private var trackingRequested = false

    @Volatile
    private var surfaceDistanceMetres = DEFAULT_SURFACE_DISTANCE_METRES

    @Volatile
    private var sensorCalibration: SensorCameraCalibration? = null

    private var previousGray: Mat? = null
    private var previousPoints: MatOfPoint2f? = null
    private var previousTimestampNanos = 0L
    private var lastAcceptedFrameTimestampNanos = 0L
    private var mode = PlanarOdometryMode.LEGACY
    private var tiltConfiguration: TiltCameraConfiguration? = null
    private var currentPose: GroundCameraPose? = null
    private var previousPose: GroundCameraPose? = null
    private var groundAxes: GroundAxes? = null
    private data class Geometry(val width: Int, val height: Int, val rotation: Int, val intrinsics: FrameIntrinsics)
    private var geometry: Geometry? = null
    private var usesLidarHeight = false
    private var previousHeight = DEFAULT_SURFACE_DISTANCE_METRES
    private var lidarGeneration: Long? = null
    private data class PendingFrame(val gray: Mat, val intrinsics: FrameIntrinsics,
                                    val timestamp: Long, val rotation: Int)
    private val pendingFrames = java.util.ArrayDeque<PendingFrame>()

    fun onCameraStarting() {
        _state.value = _state.value.copy(
            isCameraActive = false,
            status = PlanarTrackingStatus.STARTING_CAMERA,
            message = "Запуск камеры…",
        )
    }

    @Synchronized
    fun onCameraReady(calibration: SensorCameraCalibration?, tiltCamera: TiltCameraConfiguration? = null) {
        sensorCalibration = calibration
        tiltConfiguration = tiltCamera
        releaseReferenceFrame()
        clearPendingFrames()
        _state.value = _state.value.copy(
            isCameraActive = true,
            status = if (trackingRequested) {
                PlanarTrackingStatus.INITIALIZING
            } else {
                PlanarTrackingStatus.STOPPED
            },
            message = if (trackingRequested) {
                "Поиск точек поверхности…"
            } else {
                "Камера готова"
            },
        )
    }

    fun onCameraStopped() {
        synchronized(this) {
            releaseReferenceFrame()
            clearPendingFrames()
            lastAcceptedFrameTimestampNanos = 0L
        }
        _state.value = _state.value.copy(
            isCameraActive = false,
            status = PlanarTrackingStatus.STOPPED,
            heightIsValid = !usesLidarHeight,
            trackedPoints = 0,
            inliers = 0,
            featureTracks = emptyList(),
            message = "Камера остановлена",
        )
    }

    fun onCameraError(message: String) {
        trackingRequested = false
        synchronized(this) {
            releaseReferenceFrame()
            clearPendingFrames()
            lastAcceptedFrameTimestampNanos = 0L
        }
        _state.value = _state.value.copy(
            isCameraActive = false,
            status = PlanarTrackingStatus.ERROR,
            heightIsValid = !usesLidarHeight,
            featureTracks = emptyList(),
            message = message,
        )
    }

    @Synchronized
    fun startTracking(distanceMetres: Double, requestedMode: PlanarOdometryMode = PlanarOdometryMode.LEGACY,
                      useLidarHeight: Boolean = false) {
        require(distanceMetres in MIN_SURFACE_DISTANCE_METRES..MAX_SURFACE_DISTANCE_METRES)
        require(!useLidarHeight || requestedMode == PlanarOdometryMode.TILT_COMPENSATED)
        surfaceDistanceMetres = distanceMetres
        usesLidarHeight = useLidarHeight
        trackingRequested = true
        mode = requestedMode
        resetReferenceAndPose()
        _state.value = PlanarOdometryState(
            trackingEpoch = _state.value.trackingEpoch + 1L,
            isCameraActive = _state.value.isCameraActive,
            status = PlanarTrackingStatus.INITIALIZING,
            surfaceDistanceMetres = distanceMetres,
            mode = mode,
            usesLidarHeight = usesLidarHeight,
            heightIsValid = !usesLidarHeight,
            message = "Поиск точек поверхности…",
        )
    }

    @Synchronized
    fun pauseTracking() {
        trackingRequested = false
        releaseReferenceFrame()
        clearPendingFrames()
        lastAcceptedFrameTimestampNanos = 0L
        _state.value = _state.value.copy(
            status = PlanarTrackingStatus.STOPPED,
            heightIsValid = !usesLidarHeight,
            lastDeltaXMetres = 0.0,
            lastDeltaYMetres = 0.0,
            trackedPoints = 0,
            inliers = 0,
            featureTracks = emptyList(),
            message = "2D-одометрия остановлена",
        )
    }

    @Synchronized
    fun resetPosition() {
        resetReferenceAndPose()
        _state.value = PlanarOdometryState(
            trackingEpoch = _state.value.trackingEpoch + 1L,
            isCameraActive = _state.value.isCameraActive,
            status = if (trackingRequested) {
                PlanarTrackingStatus.INITIALIZING
            } else {
                PlanarTrackingStatus.STOPPED
            },
            surfaceDistanceMetres = surfaceDistanceMetres,
            mode = mode,
            usesLidarHeight = usesLidarHeight,
            heightIsValid = !usesLidarHeight,
            message = if (trackingRequested) {
                "Траектория сброшена, повторная инициализация…"
            } else {
                "Траектория сброшена"
            },
        )
    }

    override fun analyze(image: ImageProxy) {
        try {
            if (!trackingRequested) return

            val timestampNanos = image.imageInfo.timestamp
            if (
                lastAcceptedFrameTimestampNanos != 0L &&
                timestampNanos - lastAcceptedFrameTimestampNanos < PROCESSING_PERIOD_NANOS
            ) {
                return
            }

            val gray = image.toUprightGrayMat()
            try {
                val intrinsics = sensorCalibration?.forFrame(
                    rawWidth = image.width,
                    rawHeight = image.height,
                    rotationDegrees = image.imageInfo.rotationDegrees,
                ) ?: approximateIntrinsics(gray.cols(), gray.rows())

                synchronized(this) {
                    if (trackingRequested) {
                        val nextGeometry = Geometry(gray.cols(), gray.rows(), image.imageInfo.rotationDegrees, intrinsics)
                        if (geometry != nextGeometry) { releaseReferenceFrame(); clearPendingFrames() }
                        geometry = nextGeometry
                        if (usesLidarHeight) {
                            // Delayed, bounded processing permits interpolation, not stale-height hold.
                            pendingFrames.addLast(PendingFrame(gray.clone(), intrinsics, timestampNanos, nextGeometry.rotation))
                            while (pendingFrames.size > 8) pendingFrames.removeFirst().gray.release()
                            while (pendingFrames.isNotEmpty() &&
                                timestampNanos - pendingFrames.first.timestamp >= LidarHeightHistory.FRAME_DELAY_NANOS) {
                                val frame = pendingFrames.removeFirst()
                                try {
                                    if (prepareTiltFrame(frame.intrinsics, frame.timestamp, frame.rotation))
                                        processFrame(frame.gray, frame.intrinsics, frame.timestamp)
                                } finally { frame.gray.release() }
                            }
                        } else if (mode == PlanarOdometryMode.LEGACY || prepareTiltFrame(intrinsics, timestampNanos, nextGeometry.rotation)) {
                            processFrame(gray, intrinsics, timestampNanos)
                        }
                        lastAcceptedFrameTimestampNanos = timestampNanos
                    }
                }
            } finally {
                gray.release()
            }
        } catch (exception: Exception) {
            onCameraError(
                "Ошибка анализа кадра: ${exception.message ?: exception::class.java.simpleName}",
            )
        } finally {
            image.close()
        }
    }

    private fun prepareTiltFrame(intrinsics: FrameIntrinsics, timestamp: Long, rotation: Int): Boolean {
        currentPose = null
        _state.value = _state.value.copy(tiltDegrees = null, orientationGapMillis = null, groundResidualMetres = null,
            heightIsValid = !usesLidarHeight, lidarDepthMetres = null, lidarAgeMillis = null,
            lidarRoundTripMillis = null, lidarTimeSeconds = null)
        val configuration = tiltConfiguration
        val sample = orientationAt(timestamp)
        val reason = when {
            configuration?.isBackCamera != true -> "Компенсация доступна только для задней камеры"
            !configuration.timestampsAreRealtime -> "Часы камеры не синхронизированы с IMU; используйте прежний режим"
            configuration.sensorOrientationDegrees !in setOf(0, 90, 180, 270) -> "Неизвестно расположение сенсора камеры"
            sample == null -> "Нет синхронной ориентации для кадра; ожидаем данные датчиков"
            else -> null
        }
        if (reason != null) {
            releaseReferenceFrame()
            publishDegraded(reason, 0, intrinsics = intrinsics, timestampNanos = timestamp)
            return false
        }
        val pose = GroundCameraPose(checkNotNull(sample).quaternion,
            checkNotNull(configuration?.sensorOrientationDegrees), rotation)
        _state.value = _state.value.copy(tiltDegrees = pose.tiltDegrees,
            orientationGapMillis = sample.bracketGapNanos / 1_000_000.0)
        if (pose.tiltDegrees > MAX_TILT_DEGREES) {
            releaseReferenceFrame()
            publishDegraded("Наклон больше 55° от направления вниз; направьте камеру к поверхности",
                0, intrinsics = intrinsics, timestampNanos = timestamp)
            return false
        }
        currentPose = pose
        if (usesLidarHeight) {
            val height = lidarHeightAt(timestamp)
            if (height == null) {
                releaseReferenceFrame()
                publishDegraded("Нет согласованной свежей высоты LiDAR; XY не обновляется",
                    0, intrinsics = intrinsics, timestampNanos = timestamp)
                return false
            }
            if (lidarGeneration != height.generation) releaseReferenceFrame()
            lidarGeneration = height.generation
            surfaceDistanceMetres = height.heightMetres
            _state.value = _state.value.copy(surfaceDistanceMetres = height.heightMetres, heightIsValid = true,
                lidarDepthMetres = height.depthMetres, lidarAgeMillis = height.ageMillis,
                lidarRoundTripMillis = height.roundTripMillis, lidarTimeSeconds = height.experimentTimeSeconds)
        }
        if (groundAxes == null) groundAxes = GroundAxes.fromPose(pose)
        return groundAxes != null
    }

    private fun processFrame(
        gray: Mat,
        intrinsics: FrameIntrinsics,
        timestampNanos: Long,
    ) {
        val oldGray = previousGray
        val oldPoints = previousPoints
        if (oldGray == null || oldPoints == null || oldPoints.rows() < MIN_INITIAL_POINTS) {
            initializeFrom(gray, intrinsics, timestampNanos)
            return
        }

        val nextPoints = MatOfPoint2f()
        val forwardStatus = MatOfByte()
        val forwardError = MatOfFloat()
        val backwardPoints = MatOfPoint2f()
        val backwardStatus = MatOfByte()
        val backwardError = MatOfFloat()
        try {
            Video.calcOpticalFlowPyrLK(
                oldGray,
                gray,
                oldPoints,
                nextPoints,
                forwardStatus,
                forwardError,
                FLOW_WINDOW_SIZE,
                FLOW_PYRAMID_LEVELS,
                FLOW_TERM_CRITERIA,
                0,
                MIN_EIGENVALUE,
            )
            Video.calcOpticalFlowPyrLK(
                gray,
                oldGray,
                nextPoints,
                backwardPoints,
                backwardStatus,
                backwardError,
                FLOW_WINDOW_SIZE,
                FLOW_PYRAMID_LEVELS,
                FLOW_TERM_CRITERIA,
                0,
                MIN_EIGENVALUE,
            )

            val oldArray = oldPoints.toArray()
            val nextArray = nextPoints.toArray()
            val backwardArray = backwardPoints.toArray()
            val forwardStatusArray = forwardStatus.toArray()
            val backwardStatusArray = backwardStatus.toArray()
            val forwardErrorArray = forwardError.toArray()
            val validPrevious = ArrayList<Point>(oldArray.size)
            val validCurrent = ArrayList<Point>(oldArray.size)

            for (index in oldArray.indices) {
                if (
                    index >= nextArray.size ||
                    index >= backwardArray.size ||
                    index >= forwardStatusArray.size ||
                    index >= backwardStatusArray.size ||
                    index >= forwardErrorArray.size ||
                    forwardStatusArray[index].toInt() == 0 ||
                    backwardStatusArray[index].toInt() == 0 ||
                    forwardErrorArray[index] > MAX_FLOW_ERROR
                ) {
                    continue
                }
                val next = nextArray[index]
                if (next.x !in 0.0..<gray.cols().toDouble() || next.y !in 0.0..<gray.rows().toDouble()) {
                    continue
                }
                if (pointDistance(oldArray[index], backwardArray[index]) > MAX_FORWARD_BACKWARD_ERROR) {
                    continue
                }
                validPrevious += oldArray[index]
                validCurrent += next
            }

            if (validPrevious.size < MIN_MATCHED_POINTS) {
                replaceReferenceFrame(gray, timestampNanos)
                publishDegraded(
                    message = "Недостаточно устойчивых соответствий",
                    trackedPoints = validPrevious.size,
                    timestampNanos = timestampNanos,
                    intrinsics = intrinsics,
                )
                return
            }

            estimateAndIntegrate(
                previous = validPrevious,
                current = validCurrent,
                gray = gray,
                intrinsics = intrinsics,
                timestampNanos = timestampNanos,
            )
        } finally {
            nextPoints.release()
            forwardStatus.release()
            forwardError.release()
            backwardPoints.release()
            backwardStatus.release()
            backwardError.release()
        }
    }

    private fun estimateAndIntegrate(
        previous: List<Point>,
        current: List<Point>,
        gray: Mat,
        intrinsics: FrameIntrinsics,
        timestampNanos: Long,
    ) {
        val previousMat = MatOfPoint2f(*previous.toTypedArray())
        val currentMat = MatOfPoint2f(*current.toTypedArray())
        val homographyMask = Mat()
        var homography: Mat? = null
        var affine: Mat? = null
        val affineMask = Mat()
        try {
            homography = Calib3d.findHomography(
                previousMat,
                currentMat,
                Calib3d.RANSAC,
                HOMOGRAPHY_RANSAC_THRESHOLD_PIXELS,
                homographyMask,
            )
            if (homography.empty()) {
                replaceReferenceFrame(gray, timestampNanos)
                publishDegraded(
                    message = "Плоскость поверхности не найдена",
                    trackedPoints = previous.size,
                    timestampNanos = timestampNanos,
                    intrinsics = intrinsics,
                )
                return
            }

            val homographyInliers = maskIndices(homographyMask, previous.size)
            if (homographyInliers.size < MIN_INLIERS) {
                replaceReferenceFrame(gray, timestampNanos)
                publishDegraded(
                    message = "Слишком мало точек принадлежат одной плоскости",
                    trackedPoints = previous.size,
                    inliers = homographyInliers.size,
                    timestampNanos = timestampNanos,
                    intrinsics = intrinsics,
                )
                return
            }

            if (mode == PlanarOdometryMode.TILT_COMPENSATED) {
                integrateGroundMotion(previous, current, homographyInliers, gray, intrinsics, timestampNanos)
                return
            }

            val planarPrevious = MatOfPoint2f(
                *homographyInliers.map(previous::get).toTypedArray(),
            )
            val planarCurrent = MatOfPoint2f(
                *homographyInliers.map(current::get).toTypedArray(),
            )
            try {
                affine = Calib3d.estimateAffinePartial2D(
                    planarPrevious,
                    planarCurrent,
                    affineMask,
                    Calib3d.RANSAC,
                    AFFINE_RANSAC_THRESHOLD_PIXELS,
                    AFFINE_RANSAC_ITERATIONS,
                    AFFINE_RANSAC_CONFIDENCE,
                    AFFINE_REFINE_ITERATIONS,
                )
                if (affine.empty()) {
                    replaceReferenceFrame(gray, timestampNanos)
                    publishDegraded(
                        message = "Не удалось оценить смещение кадра",
                        trackedPoints = previous.size,
                        inliers = homographyInliers.size,
                        timestampNanos = timestampNanos,
                        intrinsics = intrinsics,
                    )
                    return
                }

                val transform = ImageSimilarityTransform(
                    a00 = affine.get(0, 0)[0],
                    a01 = affine.get(0, 1)[0],
                    translationX = affine.get(0, 2)[0],
                    a10 = affine.get(1, 0)[0],
                    a11 = affine.get(1, 1)[0],
                    translationY = affine.get(1, 2)[0],
                )
                val step = estimatePlanarOdometryStep(
                    transform = transform,
                    imageWidth = gray.cols(),
                    imageHeight = gray.rows(),
                    focalLengthXPixels = intrinsics.focalLengthXPixels,
                    focalLengthYPixels = intrinsics.focalLengthYPixels,
                    surfaceDistanceMetres = surfaceDistanceMetres,
                    principalPointXPixels = intrinsics.principalPointXPixels,
                    principalPointYPixels = intrinsics.principalPointYPixels,
                )
                val affineInliers = maskIndices(affineMask, homographyInliers.size)
                val finalIndices = if (affineInliers.isEmpty()) {
                    homographyInliers
                } else {
                    affineInliers.map(homographyInliers::get)
                }
                val inlierCount = finalIndices.size
                val inlierRatio = inlierCount.toDouble() / previous.size
                val elapsedNanos = timestampNanos - previousTimestampNanos
                val stepDistance = hypot(step.cameraDeltaXMetres, step.cameraDeltaYMetres)
                val fixedHeightModelIsValid = abs(step.imageScale - 1.0) <= MAX_SCALE_DEVIATION
                val stepIsPlausible =
                    elapsedNanos in 1..MAX_FRAME_INTERVAL_NANOS &&
                        stepDistance <= MAX_STEP_DISTANCE_METRES &&
                        abs(step.cameraDeltaHeadingRadians) <= MAX_STEP_HEADING_RADIANS
                val qualityIsValid =
                    inlierCount >= MIN_INLIERS &&
                        inlierRatio >= MIN_INLIER_RATIO
                val featureTracks = finalIndices.take(MAX_VISIBLE_TRACKS).map { index ->
                    FeatureTrack(
                        fromX = (previous[index].x / gray.cols()).toFloat(),
                        fromY = (previous[index].y / gray.rows()).toFloat(),
                        toX = (current[index].x / gray.cols()).toFloat(),
                        toY = (current[index].y / gray.rows()).toFloat(),
                    )
                }

                if (!fixedHeightModelIsValid || !stepIsPlausible || !qualityIsValid) {
                    replaceReferenceFrame(gray, timestampNanos)
                    val reason = when {
                        !fixedHeightModelIsValid ->
                            "Изменилась высота или камера наклонена (масштаб %.3f)"
                                .format(step.imageScale)
                        !stepIsPlausible -> "Слишком большой скачок между кадрами"
                        else -> "Недостаточная доля точек поверхности"
                    }
                    _state.value = _state.value.copy(
                        status = PlanarTrackingStatus.DEGRADED,
                        lastDeltaXMetres = 0.0,
                        lastDeltaYMetres = 0.0,
                        trackedPoints = previous.size,
                        inliers = inlierCount,
                        inlierRatio = inlierRatio,
                        imageScale = step.imageScale,
                        framesPerSecond = framesPerSecond(elapsedNanos),
                        intrinsicsAreApproximate = intrinsics.isApproximate,
                        frameWidth = gray.cols(),
                        frameHeight = gray.rows(),
                        featureTracks = featureTracks,
                        message = reason,
                        timestampNanos = timestampNanos,
                    )
                    return
                }

                val oldState = _state.value
                val motionPixels = hypot(step.centreFlowXPixels, step.centreFlowYPixels)
                val cameraDeltaX = if (motionPixels >= MOTION_DEADBAND_PIXELS) {
                    step.cameraDeltaXMetres
                } else {
                    0.0
                }
                val cameraDeltaY = if (motionPixels >= MOTION_DEADBAND_PIXELS) {
                    step.cameraDeltaYMetres
                } else {
                    0.0
                }
                val middleHeading = oldState.headingRadians + step.cameraDeltaHeadingRadians / 2.0
                val worldDelta = cameraDeltaToWorld(
                    cameraDeltaXMetres = cameraDeltaX,
                    cameraDeltaYMetres = cameraDeltaY,
                    headingRadians = middleHeading,
                )
                _state.value = oldState.copy(
                    status = PlanarTrackingStatus.TRACKING,
                    xMetres = oldState.xMetres + worldDelta.xMetres,
                    yMetres = oldState.yMetres + worldDelta.yMetres,
                    headingRadians = normalizeRadians(
                        oldState.headingRadians + step.cameraDeltaHeadingRadians,
                    ),
                    lastDeltaXMetres = worldDelta.xMetres,
                    lastDeltaYMetres = worldDelta.yMetres,
                    surfaceDistanceMetres = surfaceDistanceMetres,
                    trackedPoints = previous.size,
                    inliers = inlierCount,
                    inlierRatio = inlierRatio,
                    imageScale = step.imageScale,
                    framesPerSecond = framesPerSecond(elapsedNanos),
                    intrinsicsAreApproximate = intrinsics.isApproximate,
                    frameWidth = gray.cols(),
                    frameHeight = gray.rows(),
                    featureTracks = featureTracks,
                    message = if (motionPixels >= MOTION_DEADBAND_PIXELS) {
                        "Перемещение рассчитывается"
                    } else {
                        "Камера неподвижна"
                    },
                    timestampNanos = timestampNanos,
                )
                replaceReferenceFrame(gray, timestampNanos)
            } finally {
                planarPrevious.release()
                planarCurrent.release()
            }
        } finally {
            previousMat.release()
            currentMat.release()
            homographyMask.release()
            affineMask.release()
            homography?.release()
            affine?.release()
        }
    }

    private fun integrateGroundMotion(
        previous: List<Point>, current: List<Point>, planarIndices: List<Int>, gray: Mat,
        intrinsics: FrameIntrinsics, timestampNanos: Long,
    ) {
        val oldPose = previousPose
        val newPose = currentPose
        val axes = groundAxes
        val elapsed = timestampNanos - previousTimestampNanos
        if (oldPose == null || newPose == null || axes == null || elapsed !in 1..MAX_FRAME_INTERVAL_NANOS ||
            oldPose.deviceToWorld.angularDistanceTo(newPose.deviceToWorld) > MAX_STEP_HEADING_RADIANS) {
            replaceReferenceFrame(gray, timestampNanos)
            publishDegraded("Пропуск кадров или слишком быстрый поворот; новая опорная пара",
                previous.size, intrinsics = intrinsics, timestampNanos = timestampNanos)
            return
        }
        val matches = planarIndices.map { i ->
            PointMatch(ImagePoint(previous[i].x, previous[i].y), ImagePoint(current[i].x, current[i].y))
        }
        val estimate = estimateGroundMotion(matches, intrinsics, oldPose, newPose, previousHeight, surfaceDistanceMetres)
        val indices = estimate?.inlierIndices?.map { planarIndices[it] } ?: emptyList()
        val ratio = indices.size.toDouble() / previous.size
        val spreadX = indices.maxOfOrNull { previous[it].x }?.minus(indices.minOf { previous[it].x }) ?: 0.0
        val spreadY = indices.maxOfOrNull { previous[it].y }?.minus(indices.minOf { previous[it].y }) ?: 0.0
        if (estimate == null || indices.size < MIN_INLIERS || ratio < MIN_INLIER_RATIO ||
            spreadX < gray.cols() * 0.15 || spreadY < gray.rows() * 0.15 ||
            hypot(estimate.delta.xMetres, estimate.delta.yMetres) > MAX_STEP_DISTANCE_METRES) {
            replaceReferenceFrame(gray, timestampNanos)
            publishDegraded("Точки не согласуются с горизонтальной плоскостью и заданной h",
                previous.size, indices.size, timestampNanos, intrinsics)
            return
        }
        val worldDelta = axes.fromWorld(estimate.delta.xMetres, estimate.delta.yMetres)
        val moving = hypot(worldDelta.xMetres, worldDelta.yMetres) >= MOTION_DEADBAND_PIXELS * estimate.metresPerPixel
        val dx = if (moving) worldDelta.xMetres else 0.0
        val dy = if (moving) worldDelta.yMetres else 0.0
        val oldState = _state.value
        _state.value = oldState.copy(
            status = PlanarTrackingStatus.TRACKING,
            xMetres = oldState.xMetres + dx, yMetres = oldState.yMetres + dy,
            headingRadians = axes.heading(newPose), lastDeltaXMetres = dx, lastDeltaYMetres = dy,
            trackedPoints = previous.size, inliers = indices.size, inlierRatio = ratio,
            imageScale = 1.0, groundResidualMetres = estimate.residualMetres,
            framesPerSecond = framesPerSecond(elapsed), intrinsicsAreApproximate = intrinsics.isApproximate,
            frameWidth = gray.cols(), frameHeight = gray.rows(), timestampNanos = timestampNanos,
            featureTracks = indices.take(MAX_VISIBLE_TRACKS).map { i ->
                FeatureTrack((previous[i].x / gray.cols()).toFloat(), (previous[i].y / gray.rows()).toFloat(),
                    (current[i].x / gray.cols()).toFloat(), (current[i].y / gray.rows()).toFloat())
            },
            message = if (moving) "Перемещение с компенсацией наклона" else "Смещение ниже порога; наклон учтён",
        )
        replaceReferenceFrame(gray, timestampNanos)
    }

    private fun initializeFrom(
        gray: Mat,
        intrinsics: FrameIntrinsics,
        timestampNanos: Long,
    ) {
        replaceReferenceFrame(gray, timestampNanos)
        val count = previousPoints?.rows() ?: 0
        _state.value = _state.value.copy(
            status = if (count >= MIN_INITIAL_POINTS) {
                PlanarTrackingStatus.INITIALIZING
            } else {
                PlanarTrackingStatus.DEGRADED
            },
            surfaceDistanceMetres = surfaceDistanceMetres,
            lastDeltaXMetres = 0.0,
            lastDeltaYMetres = 0.0,
            trackedPoints = count,
            inliers = 0,
            inlierRatio = 0.0,
            intrinsicsAreApproximate = intrinsics.isApproximate,
            frameWidth = gray.cols(),
            frameHeight = gray.rows(),
            featureTracks = emptyList(),
            message = if (count >= MIN_INITIAL_POINTS) {
                "Опорный кадр создан"
            } else {
                "Недостаточно текстуры на поверхности"
            },
            timestampNanos = timestampNanos,
        )
    }

    private fun publishDegraded(
        message: String,
        trackedPoints: Int,
        inliers: Int = 0,
        timestampNanos: Long,
        intrinsics: FrameIntrinsics,
    ) {
        _state.value = _state.value.copy(
            status = PlanarTrackingStatus.DEGRADED,
            lastDeltaXMetres = 0.0,
            lastDeltaYMetres = 0.0,
            trackedPoints = trackedPoints,
            inliers = inliers,
            inlierRatio = if (trackedPoints > 0) inliers.toDouble() / trackedPoints else 0.0,
            intrinsicsAreApproximate = intrinsics.isApproximate,
            featureTracks = emptyList(),
            message = message,
            timestampNanos = timestampNanos,
        )
    }

    private fun replaceReferenceFrame(gray: Mat, timestampNanos: Long) {
        val newPoints = detectFeatures(gray)
        previousGray?.release()
        previousPoints?.release()
        previousGray = gray.clone()
        previousPoints = newPoints
        previousTimestampNanos = timestampNanos
        previousPose = currentPose
        previousHeight = surfaceDistanceMetres
    }

    private fun detectFeatures(gray: Mat): MatOfPoint2f {
        val corners = MatOfPoint()
        return try {
            Imgproc.goodFeaturesToTrack(
                gray,
                corners,
                MAX_FEATURES,
                FEATURE_QUALITY,
                MIN_FEATURE_DISTANCE_PIXELS,
            )
            MatOfPoint2f(*corners.toArray())
        } finally {
            corners.release()
        }
    }

    private fun resetReferenceAndPose() {
        releaseReferenceFrame()
        clearPendingFrames()
        lidarGeneration = null
        previousTimestampNanos = 0L
        lastAcceptedFrameTimestampNanos = 0L
        groundAxes = null
        currentPose = null
        geometry = null
    }

    private fun releaseReferenceFrame() {
        previousGray?.release()
        previousPoints?.release()
        previousGray = null
        previousPoints = null
        previousTimestampNanos = 0L
        previousPose = null
    }

    private fun clearPendingFrames() {
        while (pendingFrames.isNotEmpty()) pendingFrames.removeFirst().gray.release()
    }

    private fun maskIndices(mask: Mat, fallbackSize: Int): List<Int> {
        if (mask.empty()) return (0 until fallbackSize).toList()
        val values = ByteArray(mask.total().toInt())
        mask.get(0, 0, values)
        return values.indices.filter { values[it].toInt() != 0 }
    }

    private fun framesPerSecond(elapsedNanos: Long): Double =
        if (elapsedNanos > 0L) NANOS_PER_SECOND / elapsedNanos else 0.0

    private fun pointDistance(first: Point, second: Point): Double =
        hypot(first.x - second.x, first.y - second.y)

    private fun normalizeRadians(value: Double): Double {
        var result = value
        while (result > Math.PI) result -= 2.0 * Math.PI
        while (result < -Math.PI) result += 2.0 * Math.PI
        return result
    }

    private fun ImageProxy.toUprightGrayMat(): Mat {
        val raw = copyLumaPlane(planes[0].buffer, planes[0].rowStride, planes[0].pixelStride)
        val rotationCode = when (imageInfo.rotationDegrees) {
            90 -> Core.ROTATE_90_CLOCKWISE
            180 -> Core.ROTATE_180
            270 -> Core.ROTATE_90_COUNTERCLOCKWISE
            else -> null
        }
        if (rotationCode == null) return raw

        val rotated = Mat()
        Core.rotate(raw, rotated, rotationCode)
        raw.release()
        return rotated
    }

    private fun ImageProxy.copyLumaPlane(
        source: ByteBuffer,
        rowStride: Int,
        pixelStride: Int,
    ): Mat {
        val gray = Mat(height, width, CvType.CV_8UC1)
        val buffer = source.duplicate()
        val initialPosition = buffer.position()
        val row = ByteArray(width)
        for (y in 0 until height) {
            val rowStart = initialPosition + y * rowStride
            if (pixelStride == 1) {
                buffer.position(rowStart)
                buffer.get(row, 0, width)
            } else {
                for (x in 0 until width) {
                    row[x] = buffer.get(rowStart + x * pixelStride)
                }
            }
            gray.put(y, 0, row)
        }
        return gray
    }

    private fun approximateIntrinsics(width: Int, height: Int): FrameIntrinsics {
        val focal = maxOf(width, height) * DEFAULT_FOCAL_LENGTH_FACTOR
        return FrameIntrinsics(
            focalLengthXPixels = focal,
            focalLengthYPixels = focal,
            principalPointXPixels = (width - 1) / 2.0,
            principalPointYPixels = (height - 1) / 2.0,
            isApproximate = true,
        )
    }

    private companion object {
        const val DEFAULT_SURFACE_DISTANCE_METRES = 1.0
        const val MIN_SURFACE_DISTANCE_METRES = 0.05
        const val MAX_SURFACE_DISTANCE_METRES = 20.0
        const val MAX_FEATURES = 500
        const val MIN_INITIAL_POINTS = 60
        const val MIN_MATCHED_POINTS = 35
        const val MIN_INLIERS = 25
        const val MIN_INLIER_RATIO = 0.45
        const val FEATURE_QUALITY = 0.015
        const val MIN_FEATURE_DISTANCE_PIXELS = 8.0
        const val FEATURE_BLOCK_SIZE = 7
        const val FLOW_PYRAMID_LEVELS = 3
        const val MIN_EIGENVALUE = 0.0001
        const val MAX_FLOW_ERROR = 30.0f
        const val MAX_FORWARD_BACKWARD_ERROR = 1.25
        const val HOMOGRAPHY_RANSAC_THRESHOLD_PIXELS = 2.5
        const val AFFINE_RANSAC_THRESHOLD_PIXELS = 2.0
        const val AFFINE_RANSAC_ITERATIONS = 2_000L
        const val AFFINE_RANSAC_CONFIDENCE = 0.99
        const val AFFINE_REFINE_ITERATIONS = 10L
        const val MAX_SCALE_DEVIATION = 0.06
        const val MAX_STEP_DISTANCE_METRES = 0.30
        const val MAX_STEP_HEADING_RADIANS = 0.45
        const val MAX_TILT_DEGREES = 55.0
        const val MOTION_DEADBAND_PIXELS = 0.65
        const val MAX_VISIBLE_TRACKS = 100
        const val PROCESSING_PERIOD_NANOS = 66_666_667L
        const val MAX_FRAME_INTERVAL_NANOS = 350_000_000L
        const val NANOS_PER_SECOND = 1_000_000_000.0
        const val DEFAULT_FOCAL_LENGTH_FACTOR = 0.9
        val FLOW_WINDOW_SIZE = Size(21.0, 21.0)
        val FLOW_TERM_CRITERIA = TermCriteria(
            TermCriteria.COUNT + TermCriteria.EPS,
            30,
            0.01,
        )
    }
}
