package com.example.myapplication.vision

import androidx.camera.view.PreviewView
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.compose.LocalLifecycleOwner
import java.util.Locale
import kotlin.math.PI

@Composable
fun CameraOdometryCard(
    controller: CameraOdometryController?,
    state: PlanarOdometryState,
    cameraPermissionGranted: Boolean,
    onRequestCameraPermission: () -> Unit,
    modifier: Modifier = Modifier,
) {
    var distanceText by rememberSaveable { mutableStateOf("1.00") }
    val distanceMetres = distanceText.replace(',', '.').toDoubleOrNull()
    val distanceIsValid = distanceMetres != null && distanceMetres in 0.05..20.0
    val trackingIsRunning = state.status in setOf(
        PlanarTrackingStatus.INITIALIZING,
        PlanarTrackingStatus.TRACKING,
        PlanarTrackingStatus.DEGRADED,
        PlanarTrackingStatus.LOST,
    )

    Card(modifier = modifier.fillMaxWidth()) {
        Column(
            modifier = Modifier.padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp),
        ) {
            Text(
                text = "Камерная 2D-одометрия",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.SemiBold,
            )
            Text(
                text = "Камера должна смотреть перпендикулярно плоской неподвижной поверхности.",
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )

            if (cameraPermissionGranted && controller != null) {
                CameraPreviewWithTracks(
                    controller = controller,
                    state = state,
                    modifier = Modifier.fillMaxWidth(),
                )
            } else {
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .aspectRatio(4f / 3f)
                        .background(Color.Black),
                ) {
                    Text(
                        text = "Для превью требуется разрешение камеры",
                        modifier = Modifier.padding(16.dp),
                        color = Color.White,
                    )
                }
                Button(
                    onClick = onRequestCameraPermission,
                    modifier = Modifier.fillMaxWidth(),
                ) {
                    Text("Разрешить камеру")
                }
            }

            OutlinedTextField(
                value = distanceText,
                onValueChange = { value ->
                    if (value.length <= 6 && value.all { it.isDigit() || it == '.' || it == ',' }) {
                        distanceText = value
                    }
                },
                modifier = Modifier.fillMaxWidth(),
                enabled = !trackingIsRunning,
                label = { Text("Расстояние до поверхности, м") },
                singleLine = true,
                keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
                isError = distanceText.isNotBlank() && !distanceIsValid,
                supportingText = {
                    Text(
                        if (distanceIsValid) {
                            "Допустимо 0.05–20 м; точность масштаба зависит от этого значения"
                        } else {
                            "Введите значение от 0.05 до 20 м"
                        },
                    )
                },
            )

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                Button(
                    onClick = {
                        controller?.startTracking(checkNotNull(distanceMetres))
                    },
                    modifier = Modifier.weight(1f),
                    enabled = cameraPermissionGranted &&
                        controller != null &&
                        state.isCameraActive &&
                        distanceIsValid &&
                        !trackingIsRunning,
                ) {
                    Text("Начать")
                }
                OutlinedButton(
                    onClick = { controller?.pauseTracking() },
                    modifier = Modifier.weight(1f),
                    enabled = controller != null && trackingIsRunning,
                ) {
                    Text("Стоп")
                }
            }

            OutlinedButton(
                onClick = { controller?.resetPosition() },
                modifier = Modifier.fillMaxWidth(),
                enabled = controller != null &&
                    (trackingIsRunning || state.xMetres != 0.0 || state.yMetres != 0.0),
            ) {
                Text("Обнулить 2D-позицию")
            }

            Text(
                text = trackingStatusText(state.status),
                style = MaterialTheme.typography.titleSmall,
                color = trackingStatusColor(state.status),
            )
            Text(
                text = state.message,
                style = MaterialTheme.typography.bodyMedium,
            )
            Text(
                text = String.format(
                    Locale.US,
                    "X=%+.3f м   Y=%+.3f м   угол=%+.1f°\n" +
                        "ΔX=%+.4f м   ΔY=%+.4f м",
                    state.xMetres,
                    state.yMetres,
                    state.headingRadians * 180.0 / PI,
                    state.lastDeltaXMetres,
                    state.lastDeltaYMetres,
                ),
                style = MaterialTheme.typography.bodyLarge,
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.SemiBold,
            )
            Text(
                text = String.format(
                    Locale.US,
                    "Точки: %d, inliers: %d (%.0f%%), scale=%.3f, анализ=%.1f FPS",
                    state.trackedPoints,
                    state.inliers,
                    state.inlierRatio * 100.0,
                    state.imageScale,
                    state.framesPerSecond,
                ),
                style = MaterialTheme.typography.labelMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
            if (state.intrinsicsAreApproximate && state.frameWidth > 0) {
                Text(
                    text = "Камера не сообщила точные intrinsics — масштаб пока приблизительный.",
                    style = MaterialTheme.typography.labelMedium,
                    color = MaterialTheme.colorScheme.error,
                )
            }
            Text(
                text = "Оси X/Y зафиксированы при нажатии «Начать». Движущиеся объекты и изменение высоты ухудшают результат.",
                style = MaterialTheme.typography.labelMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
}

@Composable
private fun CameraPreviewWithTracks(
    controller: CameraOdometryController,
    state: PlanarOdometryState,
    modifier: Modifier = Modifier,
) {
    val lifecycleOwner = LocalLifecycleOwner.current
    val frameAspect = if (state.frameWidth > 0 && state.frameHeight > 0) {
        state.frameWidth.toFloat() / state.frameHeight
    } else {
        4f / 3f
    }
    val trackColor = MaterialTheme.colorScheme.tertiary
    val pointColor = MaterialTheme.colorScheme.primary

    Box(
        modifier = modifier
            .aspectRatio(frameAspect)
            .background(Color.Black),
    ) {
        AndroidView(
            factory = { context ->
                PreviewView(context).apply {
                    implementationMode = PreviewView.ImplementationMode.COMPATIBLE
                    scaleType = PreviewView.ScaleType.FIT_CENTER
                }
            },
            modifier = Modifier.fillMaxSize(),
            update = { previewView ->
                if (previewView.tag != controller) {
                    previewView.tag = controller
                    controller.startCamera(lifecycleOwner, previewView.surfaceProvider)
                }
            },
        )
        DisposableEffect(controller, lifecycleOwner) {
            onDispose { controller.stopCamera() }
        }
        Canvas(modifier = Modifier.fillMaxSize()) {
            state.featureTracks.forEach { track ->
                val start = Offset(track.fromX * size.width, track.fromY * size.height)
                val end = Offset(track.toX * size.width, track.toY * size.height)
                drawLine(
                    color = trackColor,
                    start = start,
                    end = end,
                    strokeWidth = 2f,
                )
                drawCircle(
                    color = pointColor,
                    radius = 3f,
                    center = end,
                )
            }
        }
    }
}

@Composable
private fun trackingStatusColor(status: PlanarTrackingStatus): Color = when (status) {
    PlanarTrackingStatus.TRACKING -> MaterialTheme.colorScheme.primary
    PlanarTrackingStatus.DEGRADED,
    PlanarTrackingStatus.LOST,
    PlanarTrackingStatus.ERROR,
    -> MaterialTheme.colorScheme.error
    else -> MaterialTheme.colorScheme.onSurfaceVariant
}

private fun trackingStatusText(status: PlanarTrackingStatus): String = when (status) {
    PlanarTrackingStatus.STOPPED -> "Остановлено"
    PlanarTrackingStatus.STARTING_CAMERA -> "Запуск камеры"
    PlanarTrackingStatus.INITIALIZING -> "Инициализация"
    PlanarTrackingStatus.TRACKING -> "Трекинг"
    PlanarTrackingStatus.DEGRADED -> "Низкое качество"
    PlanarTrackingStatus.LOST -> "Трекинг потерян"
    PlanarTrackingStatus.ERROR -> "Ошибка"
}
