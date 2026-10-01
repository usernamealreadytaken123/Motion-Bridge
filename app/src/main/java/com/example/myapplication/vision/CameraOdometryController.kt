package com.example.myapplication.vision

import android.content.Context
import android.graphics.Rect
import android.hardware.camera2.CameraCharacteristics
import android.util.Size
import android.util.SizeF
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.Camera
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import java.io.Closeable
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import org.opencv.android.OpenCVLoader

data class SensorCameraCalibration(
    val activeArrayWidthPixels: Double,
    val activeArrayHeightPixels: Double,
    val focalLengthXPixels: Double,
    val focalLengthYPixels: Double,
    val principalPointXPixels: Double,
    val principalPointYPixels: Double,
    val isApproximate: Boolean,
) {
    fun forFrame(
        rawWidth: Int,
        rawHeight: Int,
        rotationDegrees: Int,
    ): FrameIntrinsics {
        val sensorAspect = activeArrayWidthPixels / activeArrayHeightPixels
        val outputAspect = rawWidth.toDouble() / rawHeight
        val cropWidth: Double
        val cropHeight: Double
        val cropLeft: Double
        val cropTop: Double
        if (sensorAspect > outputAspect) {
            cropHeight = activeArrayHeightPixels
            cropWidth = cropHeight * outputAspect
            cropLeft = (activeArrayWidthPixels - cropWidth) / 2.0
            cropTop = 0.0
        } else {
            cropWidth = activeArrayWidthPixels
            cropHeight = cropWidth / outputAspect
            cropLeft = 0.0
            cropTop = (activeArrayHeightPixels - cropHeight) / 2.0
        }
        val scaleX = rawWidth / cropWidth
        val scaleY = rawHeight / cropHeight
        val rawFx = focalLengthXPixels * scaleX
        val rawFy = focalLengthYPixels * scaleY
        val rawCx = (principalPointXPixels - cropLeft) * scaleX
        val rawCy = (principalPointYPixels - cropTop) * scaleY

        return when (rotationDegrees) {
            90 -> FrameIntrinsics(
                focalLengthXPixels = rawFy,
                focalLengthYPixels = rawFx,
                principalPointXPixels = rawHeight - 1.0 - rawCy,
                principalPointYPixels = rawCx,
                isApproximate = isApproximate,
            )

            180 -> FrameIntrinsics(
                focalLengthXPixels = rawFx,
                focalLengthYPixels = rawFy,
                principalPointXPixels = rawWidth - 1.0 - rawCx,
                principalPointYPixels = rawHeight - 1.0 - rawCy,
                isApproximate = isApproximate,
            )

            270 -> FrameIntrinsics(
                focalLengthXPixels = rawFy,
                focalLengthYPixels = rawFx,
                principalPointXPixels = rawCy,
                principalPointYPixels = rawWidth - 1.0 - rawCx,
                isApproximate = isApproximate,
            )

            else -> FrameIntrinsics(
                focalLengthXPixels = rawFx,
                focalLengthYPixels = rawFy,
                principalPointXPixels = rawCx,
                principalPointYPixels = rawCy,
                isApproximate = isApproximate,
            )
        }
    }
}

class CameraOdometryController(context: Context) : Closeable {
    private val applicationContext = context.applicationContext
    private val cameraProviderFuture = ProcessCameraProvider.getInstance(applicationContext)
    private val analyzer = PlanarOdometryAnalyzer()
    private val analysisExecutor: ExecutorService = Executors.newSingleThreadExecutor()
    private var provider: ProcessCameraProvider? = null
    private var preview: Preview? = null
    private var imageAnalysis: ImageAnalysis? = null
    private var bindingGeneration = 0L
    private var closed = false

    val state = analyzer.state

    @Synchronized
    fun startCamera(
        lifecycleOwner: LifecycleOwner,
        surfaceProvider: Preview.SurfaceProvider,
    ) {
        if (closed) return
        if (!OpenCVLoader.initLocal()) {
            analyzer.onCameraError("Не удалось загрузить OpenCV")
            return
        }

        val generation = ++bindingGeneration
        analyzer.onCameraStarting()
        cameraProviderFuture.addListener(
            {
                if (closed || generation != bindingGeneration) return@addListener
                try {
                    val cameraProvider = cameraProviderFuture.get()
                    val newPreview = Preview.Builder()
                        .build()
                        .also { it.setSurfaceProvider(surfaceProvider) }
                    val newAnalysis = ImageAnalysis.Builder()
                        .setTargetResolution(ANALYSIS_RESOLUTION)
                        .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                        .build()
                        .also { it.setAnalyzer(analysisExecutor, analyzer) }

                    synchronized(this) {
                        preview?.let { oldPreview -> cameraProvider.unbind(oldPreview) }
                        imageAnalysis?.let { oldAnalysis -> cameraProvider.unbind(oldAnalysis) }
                        val camera = cameraProvider.bindToLifecycle(
                            lifecycleOwner,
                            CameraSelector.DEFAULT_BACK_CAMERA,
                            newPreview,
                            newAnalysis,
                        )
                        provider = cameraProvider
                        preview = newPreview
                        imageAnalysis = newAnalysis
                        analyzer.onCameraReady(readSensorCalibration(camera))
                    }
                } catch (exception: Exception) {
                    analyzer.onCameraError(
                        "Не удалось запустить камеру: " +
                            (exception.message ?: exception::class.java.simpleName),
                    )
                }
            },
            ContextCompat.getMainExecutor(applicationContext),
        )
    }

    @Synchronized
    fun stopCamera() {
        bindingGeneration += 1
        imageAnalysis?.clearAnalyzer()
        preview?.let { provider?.unbind(it) }
        imageAnalysis?.let { provider?.unbind(it) }
        preview = null
        imageAnalysis = null
        analyzer.onCameraStopped()
    }

    fun startTracking(surfaceDistanceMetres: Double) {
        analyzer.startTracking(surfaceDistanceMetres)
    }

    fun pauseTracking() {
        analyzer.pauseTracking()
    }

    fun resetPosition() {
        analyzer.resetPosition()
    }

    fun reportPermissionDenied() {
        analyzer.onCameraError("Нет разрешения на использование камеры")
    }

    override fun close() {
        synchronized(this) {
            if (closed) return
            stopCamera()
            closed = true
        }
        analysisExecutor.shutdown()
    }

    @androidx.annotation.OptIn(markerClass = [ExperimentalCamera2Interop::class])
    private fun readSensorCalibration(camera: Camera): SensorCameraCalibration? {
        val cameraInfo = Camera2CameraInfo.from(camera.cameraInfo)
        val activeArray = cameraInfo.getCameraCharacteristic(
            CameraCharacteristics.SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE,
        ) ?: cameraInfo.getCameraCharacteristic(
            CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE,
        ) ?: return null
        val intrinsicCalibration = cameraInfo.getCameraCharacteristic(
            CameraCharacteristics.LENS_INTRINSIC_CALIBRATION,
        )

        if (intrinsicCalibration != null && intrinsicCalibration.size >= 4) {
            return SensorCameraCalibration(
                activeArrayWidthPixels = activeArray.width().toDouble(),
                activeArrayHeightPixels = activeArray.height().toDouble(),
                focalLengthXPixels = intrinsicCalibration[0].toDouble(),
                focalLengthYPixels = intrinsicCalibration[1].toDouble(),
                principalPointXPixels = intrinsicCalibration[2].toDouble() - activeArray.left,
                principalPointYPixels = intrinsicCalibration[3].toDouble() - activeArray.top,
                isApproximate = false,
            )
        }

        val physicalSize = cameraInfo.getCameraCharacteristic(
            CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE,
        ) ?: return approximateFromActiveArray(activeArray, null, null)
        val focalLength = cameraInfo.getCameraCharacteristic(
            CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS,
        )?.firstOrNull()?.toDouble()
        return approximateFromActiveArray(activeArray, physicalSize, focalLength)
    }

    private fun approximateFromActiveArray(
        activeArray: Rect,
        physicalSize: SizeF?,
        focalLengthMillimetres: Double?,
    ): SensorCameraCalibration {
        val width = activeArray.width().toDouble()
        val height = activeArray.height().toDouble()
        val fx = if (
            physicalSize != null &&
            focalLengthMillimetres != null &&
            physicalSize.width > 0f
        ) {
            focalLengthMillimetres / physicalSize.width * width
        } else {
            width * DEFAULT_FOCAL_LENGTH_FACTOR
        }
        val fy = if (
            physicalSize != null &&
            focalLengthMillimetres != null &&
            physicalSize.height > 0f
        ) {
            focalLengthMillimetres / physicalSize.height * height
        } else {
            maxOf(width, height) * DEFAULT_FOCAL_LENGTH_FACTOR
        }
        return SensorCameraCalibration(
            activeArrayWidthPixels = width,
            activeArrayHeightPixels = height,
            focalLengthXPixels = fx,
            focalLengthYPixels = fy,
            principalPointXPixels = (width - 1.0) / 2.0,
            principalPointYPixels = (height - 1.0) / 2.0,
            isApproximate = true,
        )
    }

    private companion object {
        val ANALYSIS_RESOLUTION = Size(640, 480)
        const val DEFAULT_FOCAL_LENGTH_FACTOR = 0.9
    }
}
