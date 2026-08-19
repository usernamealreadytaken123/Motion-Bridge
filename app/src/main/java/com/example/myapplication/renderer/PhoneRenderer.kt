package com.example.myapplication.renderer

import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import com.example.myapplication.model.QuaternionData
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

internal class PhoneRenderer : GLSurfaceView.Renderer {
    private val vertices = buildPhoneVertices()
    private val vertexBuffer = vertices.toFloatBuffer()
    private val gridVertices = buildGridVertices()
    private val gridBuffer = gridVertices.toFloatBuffer()
    private val axisVertices = buildAxisVertices()
    private val axisBuffer = axisVertices.toFloatBuffer()

    private val projectionMatrix = FloatArray(16)
    private val viewMatrix = FloatArray(16)
    private val modelMatrix = FloatArray(16)
    private val modelViewMatrix = FloatArray(16)
    private val mvpMatrix = FloatArray(16)
    private val sceneMvpMatrix = FloatArray(16)

    @Volatile
    private var quaternion = QuaternionData(w = 1.0, x = 0.0, y = 0.0, z = 0.0)

    private var program = 0
    private var positionHandle = 0
    private var colorHandle = 0
    private var mvpMatrixHandle = 0

    fun updateQuaternion(quaternion: QuaternionData) {
        this.quaternion = quaternion
    }

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.035f, 0.055f, 0.09f, 1f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)

        program = createProgram(VERTEX_SHADER, FRAGMENT_SHADER)
        positionHandle = GLES20.glGetAttribLocation(program, "aPosition")
        colorHandle = GLES20.glGetAttribLocation(program, "aColor")
        mvpMatrixHandle = GLES20.glGetUniformLocation(program, "uMvpMatrix")
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        val aspectRatio = width.toFloat() / height.coerceAtLeast(1).toFloat()
        Matrix.perspectiveM(
            projectionMatrix,
            0,
            FIELD_OF_VIEW_DEGREES,
            aspectRatio,
            NEAR_PLANE,
            FAR_PLANE,
        )
        Matrix.setLookAtM(
            viewMatrix,
            0,
            2.8f,
            1.8f,
            4.8f,
            0f,
            0f,
            0f,
            0f,
            1f,
            0f,
        )
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        GLES20.glUseProgram(program)

        Matrix.multiplyMM(sceneMvpMatrix, 0, projectionMatrix, 0, viewMatrix, 0)
        drawVertices(
            buffer = gridBuffer,
            vertexCount = gridVertices.size / FLOATS_PER_VERTEX,
            matrix = sceneMvpMatrix,
            primitive = GLES20.GL_LINES,
            lineWidth = 1f,
        )
        drawVertices(
            buffer = axisBuffer,
            vertexCount = axisVertices.size / FLOATS_PER_VERTEX,
            matrix = sceneMvpMatrix,
            primitive = GLES20.GL_LINES,
            lineWidth = 3f,
        )

        val rotationMatrix = quaternion.toOpenGlRotationMatrix()
        rotationMatrix.copyInto(modelMatrix)
        Matrix.scaleM(modelMatrix, 0, PHONE_WIDTH, PHONE_HEIGHT, PHONE_DEPTH)
        Matrix.multiplyMM(modelViewMatrix, 0, viewMatrix, 0, modelMatrix, 0)
        Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, modelViewMatrix, 0)

        drawVertices(
            buffer = vertexBuffer,
            vertexCount = VERTEX_COUNT,
            matrix = mvpMatrix,
            primitive = GLES20.GL_TRIANGLES,
        )
    }

    private fun drawVertices(
        buffer: FloatBuffer,
        vertexCount: Int,
        matrix: FloatArray,
        primitive: Int,
        lineWidth: Float = 1f,
    ) {
        buffer.position(POSITION_OFFSET_FLOATS)
        GLES20.glVertexAttribPointer(
            positionHandle,
            POSITION_COMPONENTS,
            GLES20.GL_FLOAT,
            false,
            VERTEX_STRIDE_BYTES,
            buffer,
        )
        GLES20.glEnableVertexAttribArray(positionHandle)

        buffer.position(COLOR_OFFSET_FLOATS)
        GLES20.glVertexAttribPointer(
            colorHandle,
            COLOR_COMPONENTS,
            GLES20.GL_FLOAT,
            false,
            VERTEX_STRIDE_BYTES,
            buffer,
        )
        GLES20.glEnableVertexAttribArray(colorHandle)

        GLES20.glUniformMatrix4fv(mvpMatrixHandle, 1, false, matrix, 0)
        GLES20.glLineWidth(lineWidth)
        GLES20.glDrawArrays(primitive, 0, vertexCount)

        GLES20.glDisableVertexAttribArray(positionHandle)
        GLES20.glDisableVertexAttribArray(colorHandle)
    }

    private fun FloatArray.toFloatBuffer(): FloatBuffer = ByteBuffer
        .allocateDirect(size * Float.SIZE_BYTES)
        .order(ByteOrder.nativeOrder())
        .asFloatBuffer()
        .apply {
            put(this@toFloatBuffer)
            position(0)
        }

    private fun createProgram(vertexShaderCode: String, fragmentShaderCode: String): Int {
        val vertexShader = compileShader(GLES20.GL_VERTEX_SHADER, vertexShaderCode)
        val fragmentShader = compileShader(GLES20.GL_FRAGMENT_SHADER, fragmentShaderCode)
        return GLES20.glCreateProgram().also { result ->
            GLES20.glAttachShader(result, vertexShader)
            GLES20.glAttachShader(result, fragmentShader)
            GLES20.glLinkProgram(result)

            val linkStatus = IntArray(1)
            GLES20.glGetProgramiv(result, GLES20.GL_LINK_STATUS, linkStatus, 0)
            check(linkStatus[0] != 0) {
                "Не удалось связать OpenGL-программу: ${GLES20.glGetProgramInfoLog(result)}"
            }

            GLES20.glDeleteShader(vertexShader)
            GLES20.glDeleteShader(fragmentShader)
        }
    }

    private fun compileShader(type: Int, code: String): Int =
        GLES20.glCreateShader(type).also { shader ->
            GLES20.glShaderSource(shader, code)
            GLES20.glCompileShader(shader)

            val compileStatus = IntArray(1)
            GLES20.glGetShaderiv(shader, GLES20.GL_COMPILE_STATUS, compileStatus, 0)
            check(compileStatus[0] != 0) {
                "Не удалось скомпилировать OpenGL-шейдер: ${GLES20.glGetShaderInfoLog(shader)}"
            }
        }

    private companion object {
        const val POSITION_COMPONENTS = 3
        const val COLOR_COMPONENTS = 4
        const val POSITION_OFFSET_FLOATS = 0
        const val COLOR_OFFSET_FLOATS = POSITION_COMPONENTS
        const val FLOATS_PER_VERTEX = POSITION_COMPONENTS + COLOR_COMPONENTS
        const val VERTEX_STRIDE_BYTES = FLOATS_PER_VERTEX * Float.SIZE_BYTES
        const val VERTEX_COUNT = 36

        const val PHONE_WIDTH = 0.9f
        const val PHONE_HEIGHT = 1.6f
        const val PHONE_DEPTH = 0.13f

        const val FIELD_OF_VIEW_DEGREES = 45f
        const val NEAR_PLANE = 0.1f
        const val FAR_PLANE = 100f

        const val VERTEX_SHADER = """
            uniform mat4 uMvpMatrix;
            attribute vec3 aPosition;
            attribute vec4 aColor;
            varying vec4 vColor;

            void main() {
                gl_Position = uMvpMatrix * vec4(aPosition, 1.0);
                vColor = aColor;
            }
        """

        const val FRAGMENT_SHADER = """
            precision mediump float;
            varying vec4 vColor;

            void main() {
                gl_FragColor = vColor;
            }
        """

        fun buildPhoneVertices(): FloatArray {
            val result = ArrayList<Float>(VERTEX_COUNT * FLOATS_PER_VERTEX)

            fun addVertex(position: FloatArray, color: FloatArray) {
                result.addAll(position.toList())
                result.addAll(color.toList())
            }

            fun addFace(
                bottomLeft: FloatArray,
                bottomRight: FloatArray,
                topRight: FloatArray,
                topLeft: FloatArray,
                color: FloatArray,
            ) {
                addVertex(bottomLeft, color)
                addVertex(bottomRight, color)
                addVertex(topRight, color)
                addVertex(bottomLeft, color)
                addVertex(topRight, color)
                addVertex(topLeft, color)
            }

            val darkBlue = floatArrayOf(0.05f, 0.18f, 0.38f, 1f)
            val blue = floatArrayOf(0.08f, 0.38f, 0.85f, 1f)
            val lightBlue = floatArrayOf(0.24f, 0.62f, 1f, 1f)
            val cyan = floatArrayOf(0.25f, 0.78f, 0.92f, 1f)
            val navy = floatArrayOf(0.03f, 0.09f, 0.18f, 1f)
            val steel = floatArrayOf(0.22f, 0.36f, 0.55f, 1f)

            val left = -0.5f
            val right = 0.5f
            val bottom = -0.5f
            val top = 0.5f
            val back = -0.5f
            val front = 0.5f

            addFace(
                floatArrayOf(left, bottom, front),
                floatArrayOf(right, bottom, front),
                floatArrayOf(right, top, front),
                floatArrayOf(left, top, front),
                blue,
            )
            addFace(
                floatArrayOf(right, bottom, back),
                floatArrayOf(left, bottom, back),
                floatArrayOf(left, top, back),
                floatArrayOf(right, top, back),
                navy,
            )
            addFace(
                floatArrayOf(right, bottom, front),
                floatArrayOf(right, bottom, back),
                floatArrayOf(right, top, back),
                floatArrayOf(right, top, front),
                lightBlue,
            )
            addFace(
                floatArrayOf(left, bottom, back),
                floatArrayOf(left, bottom, front),
                floatArrayOf(left, top, front),
                floatArrayOf(left, top, back),
                darkBlue,
            )
            addFace(
                floatArrayOf(left, top, front),
                floatArrayOf(right, top, front),
                floatArrayOf(right, top, back),
                floatArrayOf(left, top, back),
                cyan,
            )
            addFace(
                floatArrayOf(left, bottom, back),
                floatArrayOf(right, bottom, back),
                floatArrayOf(right, bottom, front),
                floatArrayOf(left, bottom, front),
                steel,
            )

            return result.toFloatArray()
        }
    }
}
