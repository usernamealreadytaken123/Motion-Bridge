package com.example.myapplication.renderer

import com.example.myapplication.model.Vector3

internal data class SceneTranslation(
    val x: Float,
    val y: Float,
    val z: Float,
)

internal fun Vector3.toSceneTranslation() = SceneTranslation(
    x = (x * SCENE_UNITS_PER_METER).coerceIn(-MAX_SCENE_TRANSLATION, MAX_SCENE_TRANSLATION).toFloat(),
    y = (y * SCENE_UNITS_PER_METER).coerceIn(-MAX_SCENE_TRANSLATION, MAX_SCENE_TRANSLATION).toFloat(),
    z = (z * SCENE_UNITS_PER_METER).coerceIn(-MAX_SCENE_TRANSLATION, MAX_SCENE_TRANSLATION).toFloat(),
)

internal const val SCENE_UNITS_PER_METER = 2.0
internal const val MAX_SCENE_TRANSLATION = 1.4
