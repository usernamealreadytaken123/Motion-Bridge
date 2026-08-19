package com.example.myapplication.renderer

internal const val SCENE_FLOATS_PER_VERTEX = 7
internal const val GRID_HALF_SIZE = 2.4f
internal const val GRID_HEIGHT = -1.05f
internal const val GRID_DIVISIONS = 12

internal fun buildGridVertices(): FloatArray {
    val result = ArrayList<Float>((GRID_DIVISIONS + 1) * 4 * SCENE_FLOATS_PER_VERTEX)
    val gridColor = floatArrayOf(0.18f, 0.26f, 0.38f, 1f)
    val step = GRID_HALF_SIZE * 2f / GRID_DIVISIONS

    fun addVertex(x: Float, y: Float, z: Float) {
        result.add(x)
        result.add(y)
        result.add(z)
        result.addAll(gridColor.toList())
    }

    for (index in 0..GRID_DIVISIONS) {
        val coordinate = -GRID_HALF_SIZE + index * step

        addVertex(-GRID_HALF_SIZE, GRID_HEIGHT, coordinate)
        addVertex(GRID_HALF_SIZE, GRID_HEIGHT, coordinate)

        addVertex(coordinate, GRID_HEIGHT, -GRID_HALF_SIZE)
        addVertex(coordinate, GRID_HEIGHT, GRID_HALF_SIZE)
    }

    return result.toFloatArray()
}

internal fun buildAxisVertices(): FloatArray {
    val result = ArrayList<Float>(6 * SCENE_FLOATS_PER_VERTEX)

    fun addLine(
        startX: Float,
        startY: Float,
        startZ: Float,
        endX: Float,
        endY: Float,
        endZ: Float,
        color: FloatArray,
    ) {
        result.addAll(listOf(startX, startY, startZ))
        result.addAll(color.toList())
        result.addAll(listOf(endX, endY, endZ))
        result.addAll(color.toList())
    }

    val red = floatArrayOf(0.96f, 0.25f, 0.25f, 1f)
    val green = floatArrayOf(0.25f, 0.92f, 0.42f, 1f)
    val blue = floatArrayOf(0.24f, 0.55f, 1f, 1f)

    addLine(-GRID_HALF_SIZE, 0f, 0f, GRID_HALF_SIZE, 0f, 0f, red)
    addLine(0f, GRID_HEIGHT, 0f, 0f, GRID_HALF_SIZE, 0f, green)
    addLine(0f, 0f, -GRID_HALF_SIZE, 0f, 0f, GRID_HALF_SIZE, blue)

    return result.toFloatArray()
}
