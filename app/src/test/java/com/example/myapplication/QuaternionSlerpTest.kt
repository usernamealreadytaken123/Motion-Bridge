package com.example.myapplication

import com.example.myapplication.model.QuaternionData
import com.example.myapplication.processing.slerp
import kotlin.math.sqrt
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class QuaternionSlerpTest {
    private val identity = QuaternionData(1.0, 0.0, 0.0, 0.0)
    private val quarterTurnAroundZ = QuaternionData(
        w = sqrt(0.5),
        x = 0.0,
        y = 0.0,
        z = sqrt(0.5),
    )

    @Test
    fun zeroAndOneFactorsReturnEndpoints() {
        assertQuaternionEquals(identity, identity.slerp(quarterTurnAroundZ, 0.0))
        assertQuaternionEquals(quarterTurnAroundZ, identity.slerp(quarterTurnAroundZ, 1.0))
    }

    @Test
    fun intermediateFactorProducesOrientationBetweenEndpoints() {
        val halfway = identity.slerp(quarterTurnAroundZ, 0.5)

        assertTrue(halfway.w < identity.w)
        assertTrue(halfway.w > quarterTurnAroundZ.w)
        assertTrue(halfway.z > identity.z)
        assertTrue(halfway.z < quarterTurnAroundZ.z)
    }

    @Test
    fun oppositeQuaternionSignUsesSameOrientation() {
        val negativeIdentity = QuaternionData(-1.0, 0.0, 0.0, 0.0)

        assertQuaternionEquals(identity, identity.slerp(negativeIdentity, 0.5))
    }

    private fun assertQuaternionEquals(expected: QuaternionData, actual: QuaternionData) {
        assertEquals(expected.w, actual.w, 0.0001)
        assertEquals(expected.x, actual.x, 0.0001)
        assertEquals(expected.y, actual.y, 0.0001)
        assertEquals(expected.z, actual.z, 0.0001)
    }
}
