package com.example.myapplication

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.Vector3
import com.example.myapplication.processing.rotateDeviceVectorToSceneWorld
import kotlin.math.sqrt
import org.junit.Assert.assertEquals
import org.junit.Test

class WorldCoordinatesTest {
    @Test
    fun identityOrientationMapsDeviceAxesToSceneWorldAxes() {
        val identity = QuaternionData(1.0, 0.0, 0.0, 0.0)

        assertVectorEquals(
            expected = Vector3(1.0, 0.0, 0.0),
            actual = identity.rotateDeviceVectorToSceneWorld(Vector3(1.0, 0.0, 0.0)),
        )
        assertVectorEquals(
            expected = Vector3(0.0, 0.0, -1.0),
            actual = identity.rotateDeviceVectorToSceneWorld(Vector3(0.0, 1.0, 0.0)),
        )
        assertVectorEquals(
            expected = Vector3(0.0, 1.0, 0.0),
            actual = identity.rotateDeviceVectorToSceneWorld(Vector3(0.0, 0.0, 1.0)),
        )
    }

    @Test
    fun quarterTurnAroundAndroidZRotatesDeviceXIntoNorth() {
        val halfAngleComponent = sqrt(0.5)
        val quarterTurnAroundZ = QuaternionData(
            w = halfAngleComponent,
            x = 0.0,
            y = 0.0,
            z = halfAngleComponent,
        )

        val sceneWorld = quarterTurnAroundZ.rotateDeviceVectorToSceneWorld(
            Vector3(1.0, 0.0, 0.0),
        )

        assertVectorEquals(Vector3(0.0, 0.0, -1.0), sceneWorld)
    }

    private fun assertVectorEquals(expected: Vector3, actual: Vector3) {
        assertEquals(expected.x, actual.x, 0.000001)
        assertEquals(expected.y, actual.y, 0.000001)
        assertEquals(expected.z, actual.z, 0.000001)
    }
}
