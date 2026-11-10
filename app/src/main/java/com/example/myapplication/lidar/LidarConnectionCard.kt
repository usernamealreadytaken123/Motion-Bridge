package com.example.myapplication.lidar

import android.content.Context
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import com.example.myapplication.model.Vector3
import java.util.Locale

@Composable
fun LidarConnectionCard(state: LidarConnectionState, onConnect: (PhyphoxConfiguration) -> Unit,
                        onDisconnect: () -> Unit, trackingRunning: Boolean) {
    val preferences = LocalContext.current.getSharedPreferences("phyphox_lidar", Context.MODE_PRIVATE)
    var host by rememberSaveable { mutableStateOf(preferences.getString("host", "") ?: "") }
    var port by rememberSaveable { mutableStateOf(preferences.getString("port", "80") ?: "80") }
    var depthBuffer by rememberSaveable { mutableStateOf(preferences.getString("depth", "x") ?: "x") }
    var timeBuffer by rememberSaveable { mutableStateOf(preferences.getString("time", "t") ?: "t") }
    var offsetX by rememberSaveable { mutableStateOf(preferences.getString("offset_x", "0") ?: "0") }
    var offsetY by rememberSaveable { mutableStateOf(preferences.getString("offset_y", "0") ?: "0") }
    var offsetZ by rememberSaveable { mutableStateOf(preferences.getString("offset_z", "0") ?: "0") }
    var mountConfirmed by rememberSaveable { mutableStateOf(false) }
    var error by remember { mutableStateOf<String?>(null) }
    val editable = !state.running && !trackingRunning
    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("iPhone LiDAR · phyphox", style = MaterialTheme.typography.titleMedium)
            Text("Включите удалённый доступ в phyphox. Укажите адрес iPhone, не адрес ПК. HTTP, около 10 запросов/с.")
            OutlinedTextField(host, { host = it.take(15); error = null }, enabled = editable,
                label = { Text("IPv4 iPhone, например 192.168.1.20") }, singleLine = true, modifier = Modifier.fillMaxWidth())
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedTextField(port, { port = it.take(5) }, enabled = editable,
                    label = { Text("Порт") }, singleLine = true, modifier = Modifier.weight(1f))
                OutlinedTextField(depthBuffer, { depthBuffer = it.take(32) }, enabled = editable,
                    label = { Text("Глубина, м") }, singleLine = true, modifier = Modifier.weight(1f))
                OutlinedTextField(timeBuffer, { timeBuffer = it.take(32) }, enabled = editable,
                    label = { Text("Время, с") }, singleLine = true, modifier = Modifier.weight(1f))
            }
            Text("Вектор от камеры глубины iPhone к объективу Android, в метрах. Оси Android в естественной ориентации: X вправо по экрану, Y вверх, Z из экрана. Нули — приближение без учёта расстояния между объективами.",
                style = MaterialTheme.typography.bodySmall)
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedTextField(offsetX, { offsetX = it.take(10) }, enabled = editable,
                    label = { Text("ΔX, м") }, singleLine = true, modifier = Modifier.weight(1f))
                OutlinedTextField(offsetY, { offsetY = it.take(10) }, enabled = editable,
                    label = { Text("ΔY, м") }, singleLine = true, modifier = Modifier.weight(1f))
                OutlinedTextField(offsetZ, { offsetZ = it.take(10) }, enabled = editable,
                    label = { Text("ΔZ, м") }, singleLine = true, modifier = Modifier.weight(1f))
            }
            Row {
                Checkbox(mountConfirmed, { mountConfirmed = it }, enabled = editable)
                Text("Задние камеры параллельны; небольшая область LiDAR в центре смотрит на ту же горизонтальную поверхность (режим среднего, не минимума).",
                    style = MaterialTheme.typography.bodySmall)
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(enabled = editable && mountConfirmed, onClick = {
                    try {
                        fun number(text: String) = text.replace(',', '.').toDoubleOrNull()
                            ?: throw IllegalArgumentException("Смещение должно быть числом в метрах")
                        val configuration = PhyphoxConfiguration(host.trim(), port.toIntOrNull() ?: 0,
                            depthBuffer.trim(), timeBuffer.trim(), Vector3(number(offsetX), number(offsetY), number(offsetZ)))
                        preferences.edit().putString("host", host.trim()).putString("port", port)
                            .putString("depth", depthBuffer.trim()).putString("time", timeBuffer.trim())
                            .putString("offset_x", offsetX).putString("offset_y", offsetY).putString("offset_z", offsetZ).apply()
                        error = null
                        onConnect(configuration)
                    } catch (exception: IllegalArgumentException) { error = exception.message }
                }) { Text("Подключить LiDAR") }
                OutlinedButton(onClick = onDisconnect, enabled = state.running) { Text("Отключить") }
            }
            Text(error ?: state.message, color = if (error != null) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.onSurface)
            state.depthMetres?.let {
                Text(String.format(Locale.US, "Глубина = %.3f м · HTTP RTT = %.0f мс", it, state.roundTripMillis ?: 0.0))
            }
            Text("Не аппаратная синхронизация. Рекомендуется эксперимент из phyphox/odometry_lidar.phyphox; начните с медленных движений.",
                style = MaterialTheme.typography.bodySmall)
        }
    }
}
