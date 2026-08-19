package com.example.myapplication.renderer

import android.content.Context
import android.opengl.GLSurfaceView
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.viewinterop.AndroidView
import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.Vector3

private class PhoneGLSurfaceView(context: Context) : GLSurfaceView(context) {
    private val phoneRenderer = PhoneRenderer()

    init {
        setEGLContextClientVersion(2)
        setPreserveEGLContextOnPause(true)
        setRenderer(phoneRenderer)
        renderMode = RENDERMODE_WHEN_DIRTY
    }

    fun updatePose(
        quaternion: QuaternionData?,
        position: Vector3?,
    ) {
        phoneRenderer.updatePose(
            quaternion = quaternion ?: QuaternionData(w = 1.0, x = 0.0, y = 0.0, z = 0.0),
            position = position ?: Vector3(x = 0.0, y = 0.0, z = 0.0),
        )
        requestRender()
    }
}

@Composable
fun PhoneScene(
    quaternion: QuaternionData?,
    position: Vector3?,
    modifier: Modifier = Modifier,
) {
    val context = LocalContext.current
    val surfaceView = remember(context) { PhoneGLSurfaceView(context) }

    DisposableEffect(surfaceView) {
        surfaceView.onResume()
        onDispose {
            surfaceView.onPause()
        }
    }

    AndroidView(
        factory = { surfaceView },
        modifier = modifier,
        update = { view -> view.updatePose(quaternion, position) },
    )
}
