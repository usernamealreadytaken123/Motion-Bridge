package com.example.myapplication.capture

import android.os.SystemClock
import java.io.Closeable
import java.io.DataOutputStream
import java.io.OutputStream
import java.net.InetSocketAddress
import java.net.Socket
import java.nio.ByteBuffer
import java.util.ArrayDeque
import java.util.UUID
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import org.json.JSONArray
import org.json.JSONObject

enum class CaptureEncoding { RAW_GRAY8, H264 }

data class RawEndpoint(val host: String, val port: Int, val token: String, val compare: Boolean = false,
                       val encoding: CaptureEncoding = CaptureEncoding.RAW_GRAY8,
                       val videoBitrate: Int = H264CaptureEncoder.DEFAULT_BITRATE) {
    init {
        val parts = host.split('.')
        require(parts.size == 4 && parts.all { it.matches(Regex("0|[1-9][0-9]{0,2}")) && it.toInt() in 0..255 }) {
            "Введите IPv4 компьютера"
        }
        val ip = parts.map(String::toInt)
        require(ip[0] == 10 || ip[0] == 192 && ip[1] == 168 || ip[0] == 172 && ip[1] in 16..31 ||
            ip[0] == 169 && ip[1] == 254 || ip[0] == 127) { "Нужен адрес локальной сети" }
        require(port in 1..65535) { "Некорректный TCP-порт" }
        require(token.matches(Regex("[a-f0-9]{8}"))) { "Введите 8-значный код из ПК-приложения" }
        require(videoBitrate in 250_000..8_000_000) { "Битрейт H.264: 0.25–8 Мбит/с" }
    }
}

data class RawCaptureState(val running: Boolean = false, val connected: Boolean = false,
                           val compare: Boolean = false, val frames: Long = 0, val dropped: Long = 0,
                           val encoding: CaptureEncoding = CaptureEncoding.RAW_GRAY8,
                           val message: String = "Передача исходных данных выключена")

object RawWire {
    fun write(output: OutputStream, header: JSONObject, payload: ByteArray = byteArrayOf()) {
        val json = header.toString().toByteArray(Charsets.UTF_8)
        require(json.size in 1..65_536 && payload.size <= MAX_BINARY_BYTES)
        val data = DataOutputStream(output)
        data.writeBytes("MVO1")
        data.writeInt(json.size)
        data.writeInt(payload.size)
        data.write(json)
        data.write(payload)
    }

    const val MAX_BINARY_BYTES = 4 * 1024 * 1024

    /** Copy only Y pixels. ImageProxy is closed by the existing analyzer, not by this function. */
    fun copyLuma(source: ByteBuffer, width: Int, height: Int, rowStride: Int, pixelStride: Int): ByteArray {
        require(width in 16..1280 && height in 16..1280 && width * height <= 1280 * 960)
        require(rowStride > 0 && pixelStride > 0)
        val buffer = source.duplicate()
        val start = buffer.position()
        require(start.toLong() + (height - 1L) * rowStride + (width - 1L) * pixelStride < buffer.limit())
        val result = ByteArray(width * height)
        for (y in 0 until height) {
            if (pixelStride == 1) {
                buffer.position(start + y * rowStride)
                buffer.get(result, y * width, width)
            } else for (x in 0 until width) result[y * width + x] = buffer.get(start + y * rowStride + x * pixelStride)
        }
        return result
    }
}

/** One frame mailbox, bounded sensor queue; camera and sensor callbacks never write to the network. */
class RawCaptureSender(private val nowNanos: () -> Long = SystemClock::elapsedRealtimeNanos) : Closeable {
    private val executor = Executors.newSingleThreadExecutor { r -> Thread(r, "raw-capture").apply { isDaemon = true } }
    private val watchdog = Executors.newSingleThreadScheduledExecutor { r -> Thread(r, "raw-watchdog").apply { isDaemon = true } }
    private val codecExecutor = Executors.newSingleThreadExecutor { r -> Thread(r, "h264-capture").apply { isDaemon = true } }
    private val _state = MutableStateFlow(RawCaptureState())
    val state = _state.asStateFlow()
    private var generation = 0L
    private var socket: Socket? = null
    private val sensors = ArrayDeque<JSONArray>()
    private val lidar = ArrayDeque<JSONObject>()
    private var pendingFrame: Pair<JSONObject, ByteArray>? = null
    private var pendingEncode: Triple<JSONObject, ByteArray, Long>? = null
    private val encodedMessages = ArrayDeque<EncodedCaptureMessage>()
    private var encodeScheduled = false
    private var encoder: H264CaptureEncoder? = null
    private var encoding = CaptureEncoding.RAW_GRAY8
    private var videoBitrate = H264CaptureEncoder.DEFAULT_BITRATE
    private var lastFrameTime = 0L
    private var sequence = 0L
    private var dropped = 0L
    private val writingSince = AtomicLong(0)

    init {
        watchdog.scheduleAtFixedRate({
            synchronized(this) {
                val start = writingSince.get()
                if (start != 0L && nowNanos() - start > 2_000_000_000L)
                    fail("ПК не успевает принимать данные; соединение остановлено")
            }
        }, 250, 250, TimeUnit.MILLISECONDS)
    }

    @Synchronized fun start(endpoint: RawEndpoint) {
        stop()
        val run = generation
        val session = UUID.randomUUID().toString()
        encoding = endpoint.encoding
        videoBitrate = endpoint.videoBitrate
        _state.value = RawCaptureState(running = true, compare = endpoint.compare, encoding = encoding,
            message = "Подключение к ПК…")
        executor.submit {
            val connection = Socket()
            try {
                synchronized(this) {
                    if (run != generation) { connection.close(); return@submit }
                    socket = connection
                }
                connection.tcpNoDelay = true
                connection.sendBufferSize = 64 * 1024
                connection.connect(InetSocketAddress(endpoint.host, endpoint.port), 3000)
                connection.soTimeout = 3000
                val output = connection.getOutputStream()
                RawWire.write(output, JSONObject().put("type", "hello")
                    .put("protocol", if (encoding == CaptureEncoding.H264) 2 else 1)
                    .put("encoding", if (encoding == CaptureEncoding.H264) "h264" else "gray8")
                    .put("session", session).put("token", endpoint.token))
                check(connection.getInputStream().read() == 1) { "ПК не подтвердил код подключения" }
                synchronized(this) {
                    if (run != generation) return@submit
                    _state.value = _state.value.copy(connected = true,
                        message = if (encoding == CaptureEncoding.H264) "Передача H.264 + датчики на ПК"
                                  else "Передача GRAY8 + датчики на ПК")
                }
                var sent = 0L
                var previousWatermark = 1L
                while (true) {
                    val batches: List<JSONArray>
                    val watermark: Long
                    val lidarBatch: List<JSONObject>
                    val frame: Pair<JSONObject, ByteArray>?
                    val video: List<EncodedCaptureMessage>
                    synchronized(this) {
                        if (generation != run) return@submit
                        // Advance source time only AFTER all already queued samples have been sent.
                        batches = sensors.toList().chunked(256).map { JSONArray(it) }
                        sensors.clear()
                        watermark = nowNanos()
                        lidarBatch = lidar.toList()
                        lidar.clear()
                        frame = pendingFrame
                        pendingFrame = null
                        video = encodedMessages.toList()
                        encodedMessages.clear()
                        if (batches.isNotEmpty() || lidarBatch.isNotEmpty() || frame != null || video.isNotEmpty())
                            writingSince.set(nowNanos())
                    }
                    try {
                        if (batches.isNotEmpty() || lidarBatch.isNotEmpty() || frame != null || video.isNotEmpty()) {
                            for (batch in batches) RawWire.write(output, JSONObject().put("type", "sensors")
                                .put("samples", batch).put("watermark_ns", previousWatermark))
                            for (reading in lidarBatch) RawWire.write(output, reading)
                            if (frame != null) { RawWire.write(output, frame.first, frame.second); sent++ }
                            for (message in video) {
                                RawWire.write(output, message.header, message.payload)
                                if (message.header.getString("type") == "video_packet") sent++
                            }
                            RawWire.write(output, JSONObject().put("type", "sensors").put("samples", JSONArray())
                                .put("watermark_ns", watermark))
                            previousWatermark = watermark
                        }
                    } finally { writingSince.set(0) }
                    synchronized(this) {
                        if (generation != run) return@submit
                        _state.value = _state.value.copy(frames = sent, dropped = dropped)
                    }
                    Thread.sleep(20)
                }
            } catch (exception: Exception) {
                synchronized(this) {
                    if (generation == run) fail("TCP: ${exception.message ?: exception.javaClass.simpleName}")
                }
            } finally {
                connection.close()
                synchronized(this) { if (generation == run) socket = null }
            }
        }
    }

    @Synchronized fun offerSensor(kind: String, timestamp: Long, values: DoubleArray) {
        if (!_state.value.connected || values.any { !it.isFinite() }) return
        if (sensors.size >= 2048) { fail("Переполнение очереди датчиков; запись прервана"); return }
        val sample = JSONArray().put(kind).put(timestamp)
        values.forEach { sample.put(it) }
        sensors.addLast(sample)
    }

    @Synchronized fun offerLidar(reading: JSONObject) {
        if (!_state.value.connected) return
        if (lidar.size >= 64) { fail("Переполнение очереди LiDAR"); return }
        lidar.addLast(reading)
    }

    @Synchronized fun reserveFrame(timestamp: Long): Long? {
        if (!_state.value.connected || timestamp - lastFrameTime < 66_666_667L) return null
        lastFrameTime = timestamp
        return generation
    }

    @Synchronized fun offerFrame(header: JSONObject, pixels: ByteArray, run: Long) {
        if (!_state.value.connected || run != generation) return
        if (encoding == CaptureEncoding.RAW_GRAY8) {
            if (pendingFrame != null) dropped++
            header.put("type", "frame").put("seq", sequence++).put("dropped", dropped)
            pendingFrame = header to pixels
        } else {
            if (pendingEncode != null) dropped++
            header.put("seq", sequence++).put("dropped", dropped)
            pendingEncode = Triple(header, pixels, run)
            if (!encodeScheduled) {
                encodeScheduled = true
                codecExecutor.submit { encodePending(run) }
            }
        }
    }

    private fun encodePending(run: Long) {
        while (true) {
            val input = synchronized(this) {
                if (generation != run) { encodeScheduled = false; return }
                val next = pendingEncode
                pendingEncode = null
                if (next == null) { encodeScheduled = false; return }
                next
            }
            try {
                val active = encoder ?: H264CaptureEncoder(videoBitrate).also { encoder = it }
                val output = active.encode(input.first, input.second)
                synchronized(this) {
                    if (generation != run) return
                    if (encodedMessages.size + output.size > MAX_ENCODED_MESSAGES) {
                        fail("Очередь H.264 переполнена; соединение остановлено")
                        return
                    }
                    output.forEach(encodedMessages::addLast)
                }
            } catch (exception: Exception) {
                synchronized(this) {
                    if (generation == run) fail("H.264: ${exception.message ?: exception.javaClass.simpleName}")
                }
                return
            }
        }
    }

    @Synchronized fun fail(message: String) { stop(); _state.value = RawCaptureState(message = message) }

    @Synchronized fun stop() {
        generation++
        val oldEncoder = encoder
        encoder = null
        socket?.close()
        socket = null
        sensors.clear(); lidar.clear(); pendingFrame = null; pendingEncode = null; encodedMessages.clear()
        encodeScheduled = false
        sequence = 0; dropped = 0; lastFrameTime = 0; writingSince.set(0)
        encoding = CaptureEncoding.RAW_GRAY8
        _state.value = RawCaptureState()
        if (oldEncoder != null) codecExecutor.submit { oldEncoder.close() }
    }

    override fun close() { stop(); executor.shutdownNow(); watchdog.shutdownNow(); codecExecutor.shutdown() }

    private companion object { const val MAX_ENCODED_MESSAGES = 64 }
}
