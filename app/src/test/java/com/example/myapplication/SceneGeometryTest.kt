package com.example.myapplication

import com.example.myapplication.renderer.GRID_DIVISIONS
import com.example.myapplication.renderer.GRID_HEIGHT
import com.example.myapplication.renderer.SCENE_FLOATS_PER_VERTEX
import com.example.myapplication.renderer.buildAxisVertices
import com.example.myapplication.renderer.buildGridVertices
import org.junit.Assert.assertEquals
import org.junit.Test

class SceneGeometryTest {
    @Test
    fun `grid contains two lines for every division`() {
        val vertices = buildGridVertices()
        val expectedVertexCount = (GRID_DIVISIONS + 1) * 4

        assertEquals(expectedVertexCount * SCENE_FLOATS_PER_VERTEX, vertices.size)
    }

    @Test
    fun `every grid vertex lies on the floor`() {
        val vertices = buildGridVertices()

        for (offset in vertices.indices step SCENE_FLOATS_PER_VERTEX) {
            assertEquals(GRID_HEIGHT, vertices[offset + 1], 0.0001f)
        }
    }

    @Test
    fun `axes contain three colored lines`() {
        val vertices = buildAxisVertices()

        assertEquals(6 * SCENE_FLOATS_PER_VERTEX, vertices.size)
    }
}
