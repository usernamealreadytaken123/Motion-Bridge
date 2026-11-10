package com.example.myapplication.capture

import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import java.io.Closeable
import java.nio.ByteBuffer
import java.util.ArrayDeque
import java.util.TreeMap
import org.json.JSONObject

data class EncodedCaptureMessage(val header: JSONObject, val payload: ByteArray)

/**
 * Synchronous, hardware-backed AVC encoder used only from RawCaptureSender's codec thread.
 * Input is tightly packed GRAY8. Neutral chroma is added because Android AVC encoders accept
 * YUV420 rather than a portable one-plane gray format.
 */
class H264CaptureEncoder(private val bitrate: Int) : Closeable {
    private var codec: MediaCodec? = null
    private var width = 0
    private var height = 0
    private var configurationSent = false
    private val headers = TreeMap<Long, ArrayDeque<JSONObject>>()

    fun encode(header: JSONObject, luma: ByteArray): List<EncodedCaptureMessage> {
        val frameWidth = header.getInt("width")
        val frameHeight = header.getInt("height")
        require(frameWidth % 2 == 0 && frameHeight % 2 == 0) { "H.264 требует чётные размеры кадра" }
        require(luma.size == frameWidth * frameHeight) { "Некорректный GRAY8-кадр" }
        if (codec == null || width != frameWidth || height != frameHeight) restart(frameWidth, frameHeight)
        val encoder = checkNotNull(codec)
        val timestampNanos = header.getLong("timestamp_ns")
        val presentationTimeUs = timestampNanos / 1_000L
        val inputIndex = encoder.dequeueInputBuffer(INPUT_TIMEOUT_US)
        check(inputIndex >= 0) { "H.264-кодировщик не принимает кадры" }
        putGrayFrame(encoder, inputIndex, luma, frameWidth, frameHeight)
        headers.getOrPut(presentationTimeUs) { ArrayDeque() }.addLast(JSONObject(header.toString()))
        encoder.queueInputBuffer(inputIndex, 0, frameWidth * frameHeight * 3 / 2, presentationTimeUs, 0)
        return drain(encoder)
    }

    private fun restart(frameWidth: Int, frameHeight: Int) {
        close()
        width = frameWidth
        height = frameHeight
        val format = MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, width, height).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible)
            setInteger(MediaFormat.KEY_BIT_RATE, bitrate)
            setInteger(MediaFormat.KEY_FRAME_RATE, FRAME_RATE)
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, I_FRAME_INTERVAL_SECONDS)
            setInteger(MediaFormat.KEY_PROFILE, MediaCodecInfo.CodecProfileLevel.AVCProfileBaseline)
            // Not every device exposes a low-latency key; zero B-frames plus synchronous draining
            // is the portable baseline for keeping capture order equal to decode order.
            if (android.os.Build.VERSION.SDK_INT >= 29) setInteger(MediaFormat.KEY_MAX_B_FRAMES, 0)
        }
        codec = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC).also {
            it.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
            it.start()
        }
        configurationSent = false
    }

    private fun putGrayFrame(encoder: MediaCodec, index: Int, luma: ByteArray, frameWidth: Int, frameHeight: Int) {
        val image = encoder.getInputImage(index)
        if (image != null && image.planes.size >= 3) {
            copyPlane(luma, frameWidth, frameHeight, image.planes[0].buffer,
                image.planes[0].rowStride, image.planes[0].pixelStride)
            fillPlane(128.toByte(), frameWidth / 2, frameHeight / 2, image.planes[1].buffer,
                image.planes[1].rowStride, image.planes[1].pixelStride)
            fillPlane(128.toByte(), frameWidth / 2, frameHeight / 2, image.planes[2].buffer,
                image.planes[2].rowStride, image.planes[2].pixelStride)
            return
        }
        // Portable fallback used by byte-buffer encoders: planar I420 (Y, U, V).
        val buffer = checkNotNull(encoder.getInputBuffer(index)) { "Нет входного буфера H.264" }
        buffer.clear()
        check(buffer.remaining() >= frameWidth * frameHeight * 3 / 2) { "Слишком мал входной буфер H.264" }
        buffer.put(luma)
        repeat(frameWidth * frameHeight / 2) { buffer.put(128.toByte()) }
    }

    private fun drain(encoder: MediaCodec): List<EncodedCaptureMessage> {
        val result = mutableListOf<EncodedCaptureMessage>()
        val info = MediaCodec.BufferInfo()
        var waits = 0
        while (true) {
            when (val index = encoder.dequeueOutputBuffer(info, if (waits++ == 0) OUTPUT_TIMEOUT_US else 0)) {
                MediaCodec.INFO_TRY_AGAIN_LATER -> return result
                MediaCodec.INFO_OUTPUT_FORMAT_CHANGED -> {
                    if (!configurationSent) {
                        val format = encoder.outputFormat
                        val config = listOf("csd-0", "csd-1").flatMap { key ->
                            val value = format.getByteBuffer(key) ?: return@flatMap emptyList()
                            val bytes = ByteArray(value.remaining())
                            value.duplicate().get(bytes)
                            annexB(bytes).asList()
                        }.toByteArray()
                        check(config.isNotEmpty()) { "H.264-кодировщик не выдал SPS/PPS" }
                        result += EncodedCaptureMessage(
                            JSONObject().put("type", "video_config").put("codec", "h264")
                                .put("width", width).put("height", height).put("fps", FRAME_RATE)
                                .put("bitrate", bitrate),
                            config,
                        )
                        configurationSent = true
                    }
                }
                else -> if (index >= 0) {
                    val output = checkNotNull(encoder.getOutputBuffer(index))
                    val bytes = ByteArray(info.size)
                    output.position(info.offset)
                    output.limit(info.offset + info.size)
                    output.get(bytes)
                    if (info.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG == 0 && bytes.isNotEmpty()) {
                        val queue = headers[info.presentationTimeUs]
                        val source = if (queue != null && queue.isNotEmpty()) queue.removeFirst() else null
                        checkNotNull(source) { "H.264 вернул кадр с неизвестной временной меткой" }
                        if (queue?.isEmpty() == true) headers.remove(info.presentationTimeUs)
                        source.put("type", "video_packet")
                            .put("pts_us", info.presentationTimeUs)
                            .put("keyframe", info.flags and MediaCodec.BUFFER_FLAG_KEY_FRAME != 0)
                        result += EncodedCaptureMessage(source, annexB(bytes))
                    }
                    encoder.releaseOutputBuffer(index, false)
                }
            }
        }
    }

    override fun close() {
        val old = codec
        codec = null
        headers.clear()
        configurationSent = false
        width = 0
        height = 0
        if (old != null) {
            try { old.stop() } catch (_: Exception) {}
            old.release()
        }
    }

    companion object {
        const val DEFAULT_BITRATE = 1_500_000
        const val FRAME_RATE = 15
        private const val I_FRAME_INTERVAL_SECONDS = 1
        private const val INPUT_TIMEOUT_US = 100_000L
        private const val OUTPUT_TIMEOUT_US = 20_000L

        internal fun copyPlane(source: ByteArray, width: Int, height: Int, destination: ByteBuffer,
                               rowStride: Int, pixelStride: Int) {
            require(source.size == width * height)
            val start = destination.position()
            require(start.toLong() + (height - 1L) * rowStride + (width - 1L) * pixelStride < destination.limit())
            for (y in 0 until height) for (x in 0 until width)
                destination.put(start + y * rowStride + x * pixelStride, source[y * width + x])
        }

        internal fun fillPlane(value: Byte, width: Int, height: Int, destination: ByteBuffer,
                               rowStride: Int, pixelStride: Int) {
            val start = destination.position()
            require(start.toLong() + (height - 1L) * rowStride + (width - 1L) * pixelStride < destination.limit())
            for (y in 0 until height) for (x in 0 until width)
                destination.put(start + y * rowStride + x * pixelStride, value)
        }

        /** Normalize vendor AVCC length prefixes to the Annex-B stream accepted by FFmpeg/PyAV. */
        internal fun annexB(source: ByteArray): ByteArray {
            if (source.size >= 4 && source[0] == 0.toByte() && source[1] == 0.toByte() &&
                source[2] == 0.toByte() && source[3] == 1.toByte()) return source
            val output = java.io.ByteArrayOutputStream(source.size + 16)
            var offset = 0
            while (offset + 4 <= source.size) {
                val length = ((source[offset].toInt() and 0xff) shl 24) or
                    ((source[offset + 1].toInt() and 0xff) shl 16) or
                    ((source[offset + 2].toInt() and 0xff) shl 8) or
                    (source[offset + 3].toInt() and 0xff)
                if (length <= 0 || offset + 4 + length > source.size) break
                output.write(byteArrayOf(0, 0, 0, 1))
                output.write(source, offset + 4, length)
                offset += 4 + length
            }
            if (offset == source.size && output.size() > 0) return output.toByteArray()
            if (source.size >= 3 && source[0] == 0.toByte() && source[1] == 0.toByte() &&
                source[2] == 1.toByte()) return source
            error("Неизвестный формат H.264 NAL")
        }
    }
}
