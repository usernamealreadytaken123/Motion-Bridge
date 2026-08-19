package com.example.myapplication.network

import android.os.SystemClock
import com.example.myapplication.model.MotionPacket
import java.io.Closeable
import java.util.concurrent.atomic.AtomicLong
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.currentCoroutineContext
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

data class StreamingState(
    val isActive: Boolean = false,
    val status: String = "Передача остановлена",
    val sentPackets: Long = 0,
    val lastSequence: Long? = null,
    val error: String? = null,
)

class TestPacketStreamer : Closeable {
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)
    private val sessionCounter = AtomicLong(0)
    private var streamingJob: Job? = null

    private val _state = MutableStateFlow(StreamingState())
    val state: StateFlow<StreamingState> = _state.asStateFlow()

    @Synchronized
    fun start(host: String, port: Int) {
        if (streamingJob?.isActive == true) return

        val session = sessionCounter.incrementAndGet()
        _state.value = StreamingState(
            isActive = true,
            status = "Подготовка передачи…",
        )

        streamingJob = scope.launch {
            try {
                UdpSender(host, port).use { sender ->
                    updateState(session) {
                        it.copy(status = "Отправка на $host:$port")
                    }

                    var sequence = 0L
                    var sentPackets = 0L
                    var nextSendAt = SystemClock.elapsedRealtimeNanos()

                    while (currentCoroutineContext().isActive && sessionCounter.get() == session) {
                        val now = SystemClock.elapsedRealtimeNanos()
                        val remainingNanos = nextSendAt - now

                        if (remainingNanos > 0) {
                            delay((remainingNanos + NANOS_PER_MILLISECOND - 1) / NANOS_PER_MILLISECOND)
                        } else if (-remainingNanos >= SEND_PERIOD_NANOS) {
                            // Не догоняем график пачкой устаревших пакетов после задержки.
                            nextSendAt = now
                        }

                        val timestamp = System.currentTimeMillis() / 1_000.0
                        sender.send(MotionPacket.testPacket(sequence, timestamp).toJson())
                        sentPackets += 1

                        if (sentPackets == 1L || sentPackets % UI_UPDATE_EVERY_PACKETS == 0L) {
                            val lastSentSequence = sequence
                            updateState(session) {
                                it.copy(
                                    sentPackets = sentPackets,
                                    lastSequence = lastSentSequence,
                                    error = null,
                                )
                            }
                        }

                        sequence += 1
                        nextSendAt += SEND_PERIOD_NANOS
                    }
                }
            } catch (exception: Exception) {
                updateState(session) {
                    it.copy(
                        isActive = false,
                        status = "Ошибка передачи",
                        error = exception.message ?: exception::class.java.simpleName,
                    )
                }
            } finally {
                updateState(session) {
                    it.copy(
                        isActive = false,
                        status = if (it.error == null) "Передача остановлена" else it.status,
                    )
                }
            }
        }
    }

    @Synchronized
    fun stop() {
        sessionCounter.incrementAndGet()
        streamingJob?.cancel()
        streamingJob = null
        _state.value = _state.value.copy(
            isActive = false,
            status = "Передача остановлена",
        )
    }

    override fun close() {
        stop()
        scope.cancel()
    }

    private inline fun updateState(
        session: Long,
        transform: (StreamingState) -> StreamingState,
    ) {
        if (sessionCounter.get() == session) {
            _state.value = transform(_state.value)
        }
    }

    private companion object {
        const val SEND_FREQUENCY_HZ = 50L
        const val NANOS_PER_SECOND = 1_000_000_000L
        const val NANOS_PER_MILLISECOND = 1_000_000L
        const val SEND_PERIOD_NANOS = NANOS_PER_SECOND / SEND_FREQUENCY_HZ
        const val UI_UPDATE_EVERY_PACKETS = 5L
    }
}
