package com.example.myapplication

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.renderer.toOpenGlRotationMatrix
import kotlin.math.sqrt
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

class QuaternionMatrixTest {
    @Test
    fun identityQuaternionProducesIdentityMatrix() {
        val matrix = QuaternionData(1.0, 0.0, 0.0, 0.0).toOpenGlRotationMatrix()

        assertArrayEquals(
            floatArrayOf(
                1f, 0f, 0f, 0f,
                0f, 1f, 0f, 0f,
                0f, 0f, 1f, 0f,
                0f, 0f, 0f, 1f,
            ),
            matrix,
            0.0001f,
        )
    }

    @Test
    fun ninetyDegreesAroundZRotatesXAxisTowardYAxis() {
        val halfAngleComponent = sqrt(0.5)
        val matrix = QuaternionData(
            w = halfAngleComponent,
            x = 0.0,
            y = 0.0,
            z = halfAngleComponent,
        ).toOpenGlRotationMatrix()

        val transformedX = matrix[0]
        val transformedY = matrix[1]
        val transformedZ = matrix[2]

        assertEquals(0f, transformedX, 0.0001f)
        assertEquals(1f, transformedY, 0.0001f)
        assertEquals(0f, transformedZ, 0.0001f)
    }
}
