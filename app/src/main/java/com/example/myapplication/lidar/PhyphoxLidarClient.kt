package com.example.myapplication.lidar

import android.os.SystemClock
import com.example.myapplication.sensor.FrameOrientation
import java.io.Closeable
import java.net.HttpURLConnection
import org.json.JSONArray
import org.json.JSONObject
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow

data class LidarConnectionState(
    val running: Boolean = false,
    val ready: Boolean = false,
    val message: String = "LiDAR не подключён",
    val depthMetres: Double? = null,
    val roundTripMillis: Double? = null,
)

class PhyphoxLidarClient(private val orientationAt: (Long) -> FrameOrientation?,
                         private val nowNanos: () -> Long = SystemClock::elapsedRealtimeNanos) : Closeable {
    @Volatile var rawReadingConsumer: ((JSONObject) -> Unit)? = null
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)
    private var job: Job? = null
    private var generation = 0L
    private var connection: HttpURLConnection? = null
    private val history = LidarHeightHistory()
    private val _state = MutableStateFlow(LidarConnectionState())
    val state = _state.asStateFlow()

    fun heightAt(timestamp: Long): LidarFrameHeight? =
        history.at(timestamp, nowNanos(), orientationAt)

    @Synchronized fun start(configuration: PhyphoxConfiguration) {
        stop()
        history.configure(configuration.lidarToCamera)
        val run = generation
        _state.value = LidarConnectionState(running = true, message = "Подключение к iPhone…")
        job = scope.launch {
            while (isActive) {
                val started = nowNanos()
                var http: HttpURLConnection? = null
                try {
                    http = configuration.url().openConnection() as HttpURLConnection
                    http.connectTimeout = 500
                    http.readTimeout = 500
                    http.instanceFollowRedirects = false
                    http.useCaches = false
                    http.setRequestProperty("Accept", "application/json")
                    synchronized(this@PhyphoxLidarClient) {
                        if (generation != run) { http.disconnect(); return@launch }
                        connection = http
                    }
                    check(http.responseCode == 200) { "HTTP ${http.responseCode}" }
                    val text = http.inputStream.use { input ->
                        val bytes = java.io.ByteArrayOutputStream()
                        val buffer = ByteArray(4096)
                        while (true) {
                            val count = input.read(buffer)
                            if (count < 0) break
                            check(bytes.size() + count <= PhyphoxProtocol.MAX_RESPONSE_BYTES) { "Слишком большой ответ" }
                            check(nowNanos() - started < 1_000_000_000L) { "Ответ phyphox слишком медленный" }
                            bytes.write(buffer, 0, count)
                        }
                        bytes.toString("UTF-8")
                    }
                    val reading = PhyphoxProtocol.parse(text, configuration)
                    val received = nowNanos()
                    synchronized(this@PhyphoxLidarClient) {
                        if (generation != run) return@launch
                        rawReadingConsumer?.invoke(JSONObject().put("type", "lidar").put("ok", true)
                            .put("session", reading.session).put("measuring", reading.measuring)
                            .put("depth_m", reading.depthMetres ?: JSONObject.NULL)
                            .put("time_s", reading.experimentTimeSeconds ?: JSONObject.NULL)
                            .put("requested_ns", started).put("received_ns", received)
                            .put("mount", JSONArray(listOf(configuration.lidarToCamera.x,
                                configuration.lidarToCamera.y, configuration.lidarToCamera.z))))
                        history.accept(reading, started, received)
                        val ready = history.isFresh(received)
                        _state.value = LidarConnectionState(true, ready,
                            when {
                                !reading.measuring -> "phyphox на паузе — запустите измерение на iPhone"
                                received - started > LidarHeightHistory.MAX_RTT_NANOS -> "Слишком большая задержка Wi-Fi"
                                !ready -> "Ожидаем новые метки времени LiDAR (буфер t)"
                                else -> "LiDAR: данные поступают; синхронизация приблизительная"
                            }, reading.depthMetres.takeIf { ready }, (received - started) / 1e6)
                    }
                } catch (cancelled: CancellationException) {
                    throw cancelled
                } catch (exception: Exception) {
                    synchronized(this@PhyphoxLidarClient) {
                        if (generation != run) return@launch
                        history.invalidate()
                        rawReadingConsumer?.invoke(JSONObject().put("type", "lidar").put("ok", false))
                        _state.value = LidarConnectionState(running = true,
                            message = "LiDAR: ${exception.message?.take(160) ?: "ошибка соединения"}")
                    }
                } finally {
                    http?.disconnect()
                    synchronized(this@PhyphoxLidarClient) { if (generation == run) connection = null }
                }
                delay(maxOf(10L, 100L - (nowNanos() - started) / 1_000_000L))
            }
        }
    }

    @Synchronized fun stop() {
        generation++
        job?.cancel()
        job = null
        connection?.disconnect()
        connection = null
        history.invalidate()
        rawReadingConsumer?.invoke(JSONObject().put("type", "lidar").put("ok", false))
        _state.value = LidarConnectionState()
    }

    fun permissionDenied() { stop(); _state.value = LidarConnectionState(message = "Нет разрешения на локальную сеть") }
    override fun close() { stop(); scope.cancel() }
}
