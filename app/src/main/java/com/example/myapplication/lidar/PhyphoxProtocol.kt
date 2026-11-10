package com.example.myapplication.lidar

import com.example.myapplication.model.Vector3
import org.json.JSONObject
import java.net.URL

/** Only explicit LAN IPv4 addresses: no redirects, DNS, credentials or arbitrary URLs. */
data class PhyphoxConfiguration(
    val host: String,
    val port: Int = 80,
    val depthBuffer: String = "x",
    val timeBuffer: String = "t",
    // Vector FROM the iPhone depth-camera origin TO the Android camera, in Android device axes.
    val lidarToCamera: Vector3 = Vector3(0.0, 0.0, 0.0),
) {
    init {
        val parts = host.split('.')
        require(parts.size == 4 && parts.all { it.matches(Regex("0|[1-9][0-9]{0,2}")) && it.toInt() in 0..255 }) {
            "Введите IPv4-адрес iPhone без http://"
        }
        val ip = parts.map(String::toInt)
        require(ip[0] == 10 || ip[0] == 192 && ip[1] == 168 ||
            ip[0] == 172 && ip[1] in 16..31 || ip[0] == 169 && ip[1] == 254 || ip[0] == 127) {
            "Поддерживаются только адреса локальной сети"
        }
        require(port in 1..65535) { "Порт должен быть от 1 до 65535" }
        require(listOf(depthBuffer, timeBuffer).all { it.matches(Regex("[A-Za-z][A-Za-z0-9_]{0,31}")) } &&
            depthBuffer != timeBuffer) { "Нужны два разных имени буферов: глубина и время" }
        require(listOf(lidarToCamera.x, lidarToCamera.y, lidarToCamera.z).all { it.isFinite() && it in -1.0..1.0 }) {
            "Смещение объективов: от −1 до 1 м по каждой оси"
        }
    }

    fun url() = URL("http://$host:$port/get?$depthBuffer&$timeBuffer")
}

data class PhyphoxReading(val session: String, val measuring: Boolean,
                         val depthMetres: Double?, val experimentTimeSeconds: Double?)

object PhyphoxProtocol {
    const val MAX_RESPONSE_BYTES = 65_536

    fun parse(json: String, configuration: PhyphoxConfiguration): PhyphoxReading {
        require(json.length <= MAX_RESPONSE_BYTES) { "Слишком большой ответ phyphox" }
        val root = JSONObject(json)
        val status = root.getJSONObject("status")
        val session = status.get("session")
        require(session is String && session.length in 1..128) { "Некорректная сессия phyphox" }
        val measuring = status.get("measuring")
        require(measuring is Boolean) { "Нет статуса измерения phyphox" }
        if (!measuring) return PhyphoxReading(session, false, null, null)
        val buffers = root.getJSONObject("buffer")
        fun lastSingle(name: String): Double {
            val values = buffers.getJSONObject(name).getJSONArray("buffer")
            require(values.length() == 1) { "Буфер $name: ожидается одно актуальное значение" }
            val value = values.get(0)
            require(value is Number && value.toDouble().isFinite()) { "Буфер $name: нет числового измерения" }
            return value.toDouble()
        }
        val depth = lastSingle(configuration.depthBuffer)
        val time = lastSingle(configuration.timeBuffer)
        require(depth in 0.05..5.0) { "Глубина вне рабочего диапазона 0.05–5 м" }
        require(time in 0.0..100_000_000.0) { "Некорректное время LiDAR" }
        return PhyphoxReading(session, true, depth, time)
    }
}
