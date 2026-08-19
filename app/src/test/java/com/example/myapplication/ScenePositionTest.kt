package com.example.myapplication

import com.example.myapplication.model.Vector3
import com.example.myapplication.renderer.MAX_SCENE_TRANSLATION
import com.example.myapplication.renderer.toSceneTranslation
import org.junit.Assert.assertEquals
import org.junit.Test

class ScenePositionTest {
    @Test
    fun metersAreScaledIntoSceneUnits() {
        val translation = Vector3(0.5, -0.25, 0.1).toSceneTranslation()

        assertEquals(1.0f, translation.x, 0.0001f)
        assertEquals(-0.5f, translation.y, 0.0001f)
        assertEquals(0.2f, translation.z, 0.0001f)
    }

    @Test
    fun largePositionIsClampedInsideVisibleScene() {
        val translation = Vector3(10.0, -10.0, 3.0).toSceneTranslation()

        assertEquals(MAX_SCENE_TRANSLATION.toFloat(), translation.x, 0.0001f)
        assertEquals(-MAX_SCENE_TRANSLATION.toFloat(), translation.y, 0.0001f)
        assertEquals(MAX_SCENE_TRANSLATION.toFloat(), translation.z, 0.0001f)
    }
}
