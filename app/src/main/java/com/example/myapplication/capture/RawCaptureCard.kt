package com.example.myapplication.capture

import android.content.Context
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp

@Composable
fun RawCaptureCard(state: RawCaptureState, cameraActive: Boolean,
                   onStart: (RawEndpoint) -> Unit, onStop: () -> Unit) {
    val preferences = LocalContext.current.getSharedPreferences("pc_capture", Context.MODE_PRIVATE)
    var host by rememberSaveable { mutableStateOf(preferences.getString("host", "") ?: "") }
    var port by rememberSaveable { mutableStateOf(preferences.getString("port", "5006") ?: "5006") }
    var token by rememberSaveable { mutableStateOf("") }
    var compare by rememberSaveable { mutableStateOf(false) }
    var useH264 by rememberSaveable { mutableStateOf(preferences.getBoolean("h264", true)) }
    var error by remember { mutableStateOf<String?>(null) }
    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Расчёт одометрии на ПК", style = MaterialTheme.typography.titleMedium)
            Text("Передаются кадры (~15 FPS) и события датчиков. H.264 уменьшает трафик; RAW GRAY8 оставлен для эталонного сравнения. На ПК нажмите «Ожидать Android».")
            OutlinedTextField(host, { host = it.take(15) }, label = { Text("IPv4 компьютера") },
                enabled = !state.running, singleLine = true, modifier = Modifier.fillMaxWidth())
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedTextField(port, { port = it.take(5) }, label = { Text("TCP-порт") }, enabled = !state.running,
                    singleLine = true, modifier = Modifier.weight(1f))
                OutlinedTextField(token, { token = it.lowercase().take(8) }, label = { Text("Код с ПК") }, enabled = !state.running,
                    singleLine = true, modifier = Modifier.weight(1f))
            }
            Row {
                Checkbox(useH264, { useH264 = it }, enabled = !state.running)
                Text("Сжатый поток H.264 · 1,5 Мбит/с")
            }
            Row {
                Checkbox(compare, { compare = it }, enabled = !state.running)
                Text("Разрешить параллельный расчёт на Android (сравнение). Запускается кнопкой «Начать» в камерной одометрии выше.")
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(enabled = cameraActive && !state.running, onClick = {
                    try {
                        val endpoint = RawEndpoint(host.trim(), port.toIntOrNull() ?: 0, token.trim(), compare,
                            if (useH264) CaptureEncoding.H264 else CaptureEncoding.RAW_GRAY8)
                        preferences.edit().putString("host", host.trim()).putString("port", port)
                            .putBoolean("h264", useH264).apply()
                        error = null
                        onStart(endpoint)
                    } catch (exception: IllegalArgumentException) { error = exception.message }
                }) { Text("Передавать на ПК") }
                OutlinedButton(onClick = onStop, enabled = state.running) { Text("Остановить") }
            }
            Text(error ?: state.message)
            Text("Режим: ${if ((state.running && state.encoding == CaptureEncoding.H264) || (!state.running && useH264)) "H.264" else "RAW GRAY8"}; " +
                "кадров отправлено: ${state.frames}; заменено до кодирования: ${state.dropped}")
            Text("Без флажка сравнения локальная одометрия остановлена. Для камеры требуется разрешение. Поток не шифруется — используйте доверенную Wi-Fi-сеть.",
                style = MaterialTheme.typography.bodySmall)
        }
    }
}
