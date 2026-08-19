package com.example.myapplication

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.example.myapplication.model.QuaternionData
import com.example.myapplication.model.Vector3
import com.example.myapplication.network.StreamingState
import com.example.myapplication.network.TestPacketStreamer
import com.example.myapplication.sensor.SensorAvailability
import com.example.myapplication.sensor.SensorCollector
import com.example.myapplication.sensor.SensorState
import com.example.myapplication.ui.theme.MyApplicationTheme
import java.util.Locale

class MainActivity : ComponentActivity() {
    private val streamer = TestPacketStreamer()
    private lateinit var sensorCollector: SensorCollector

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        sensorCollector = SensorCollector(applicationContext)
        enableEdgeToEdge()
        setContent {
            MyApplicationTheme {
                val streamingState by streamer.state.collectAsState()
                val sensorState by sensorCollector.state.collectAsState()
                MotionSenderScreen(
                    streamingState = streamingState,
                    sensorState = sensorState,
                    onStart = streamer::start,
                    onStop = streamer::stop,
                )
            }
        }
    }

    override fun onStart() {
        super.onStart()
        sensorCollector.start()
    }

    override fun onStop() {
        streamer.stop()
        sensorCollector.stop()
        super.onStop()
    }

    override fun onDestroy() {
        streamer.close()
        sensorCollector.close()
        super.onDestroy()
    }
}

@Composable
private fun MotionSenderScreen(
    streamingState: StreamingState,
    sensorState: SensorState,
    onStart: (String, Int) -> Unit,
    onStop: () -> Unit,
    modifier: Modifier = Modifier,
) {
    var host by rememberSaveable { mutableStateOf("192.168.1.100") }
    var portText by rememberSaveable { mutableStateOf("5005") }
    val port = portText.toIntOrNull()
    val portIsValid = port != null && port in 1..65535
    val endpointIsValid = host.isNotBlank() && portIsValid

    Scaffold(modifier = modifier.fillMaxSize()) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .verticalScroll(rememberScrollState())
                .padding(horizontal = 20.dp, vertical = 24.dp),
            verticalArrangement = Arrangement.spacedBy(16.dp),
        ) {
            Text(
                text = "Motion Tracker",
                style = MaterialTheme.typography.headlineMedium,
                fontWeight = FontWeight.Bold,
            )
            Text(
                text = "Живые данные датчиков телефона и тестовая UDP-передача 50 Гц.",
                style = MaterialTheme.typography.bodyLarge,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )

            SensorDataCard(sensorState)

            Text(
                text = "UDP-передача",
                style = MaterialTheme.typography.titleLarge,
                fontWeight = FontWeight.SemiBold,
            )

            OutlinedTextField(
                value = host,
                onValueChange = { host = it },
                modifier = Modifier.fillMaxWidth(),
                enabled = !streamingState.isActive,
                label = { Text("IP-адрес компьютера") },
                placeholder = { Text("192.168.1.100") },
                singleLine = true,
                keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Ascii),
            )
            OutlinedTextField(
                value = portText,
                onValueChange = { value ->
                    if (value.all(Char::isDigit) && value.length <= 5) {
                        portText = value
                    }
                },
                modifier = Modifier.fillMaxWidth(),
                enabled = !streamingState.isActive,
                label = { Text("UDP-порт") },
                placeholder = { Text("5005") },
                singleLine = true,
                keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                isError = portText.isNotEmpty() && !portIsValid,
                supportingText = {
                    if (portText.isNotEmpty() && !portIsValid) {
                        Text("Допустимый диапазон: 1–65535")
                    }
                },
            )

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(12.dp),
            ) {
                Button(
                    onClick = { onStart(host.trim(), checkNotNull(port)) },
                    modifier = Modifier.weight(1f),
                    enabled = !streamingState.isActive && endpointIsValid,
                ) {
                    Text("Начать")
                }
                OutlinedButton(
                    onClick = onStop,
                    modifier = Modifier.weight(1f),
                    enabled = streamingState.isActive,
                ) {
                    Text("Остановить")
                }
            }

            StatusCard(streamingState)

            Spacer(modifier = Modifier.height(4.dp))
            Text(
                text = "Показания сверху уже реальные. UDP пока отправляет фиксированные тестовые значения — подключим к датчикам отдельным шагом.",
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
}

@Composable
private fun SensorDataCard(state: SensorState) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.secondaryContainer,
        ),
    ) {
        Column(
            modifier = Modifier.padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp),
        ) {
            Text(
                text = "Датчики: ${if (state.isCollecting) "активны" else "остановлены"}",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.SemiBold,
            )
            SensorValue(
                title = "Rotation vector",
                isAvailable = state.availability.rotationVector,
                value = state.quaternion?.formatted(),
            )
            SensorValue(
                title = "Гироскоп, рад/с",
                isAvailable = state.availability.gyroscope,
                value = state.gyroscope?.formatted(),
            )
            SensorValue(
                title = "Линейное ускорение, м/с²",
                isAvailable = state.availability.linearAcceleration,
                value = state.linearAcceleration?.formatted(),
            )
        }
    }
}

@Composable
private fun SensorValue(
    title: String,
    isAvailable: Boolean,
    value: String?,
) {
    Column(verticalArrangement = Arrangement.spacedBy(2.dp)) {
        Text(
            text = title,
            style = MaterialTheme.typography.labelLarge,
        )
        Text(
            text = when {
                !isAvailable -> "Недоступен на этом устройстве"
                value == null -> "Ожидание данных…"
                else -> value
            },
            style = MaterialTheme.typography.bodyMedium,
            fontFamily = FontFamily.Monospace,
            color = if (isAvailable) {
                MaterialTheme.colorScheme.onSecondaryContainer
            } else {
                MaterialTheme.colorScheme.error
            },
        )
    }
}

private fun QuaternionData.formatted(): String = String.format(
    Locale.US,
    "w=% .3f  x=% .3f  y=% .3f  z=% .3f",
    w,
    x,
    y,
    z,
)

private fun Vector3.formatted(): String = String.format(
    Locale.US,
    "x=% .3f  y=% .3f  z=% .3f",
    x,
    y,
    z,
)

@Composable
private fun StatusCard(state: StreamingState) {
    val containerColor = when {
        state.error != null -> MaterialTheme.colorScheme.errorContainer
        state.isActive -> MaterialTheme.colorScheme.primaryContainer
        else -> MaterialTheme.colorScheme.surfaceVariant
    }

    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = containerColor),
    ) {
        Column(
            modifier = Modifier.padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(6.dp),
        ) {
            Text(
                text = state.status,
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.SemiBold,
            )
            Text("Отправлено пакетов: ${state.sentPackets}")
            Text("Последний sequence: ${state.lastSequence ?: "—"}")
            state.error?.let { error ->
                Text(
                    text = error,
                    color = MaterialTheme.colorScheme.onErrorContainer,
                )
            }
        }
    }
}

@Preview(showBackground = true)
@Composable
private fun MotionSenderScreenPreview() {
    MyApplicationTheme {
        MotionSenderScreen(
            streamingState = StreamingState(),
            sensorState = SensorState(
                availability = SensorAvailability(
                    rotationVector = true,
                    gyroscope = true,
                    linearAcceleration = true,
                ),
                isCollecting = true,
                quaternion = QuaternionData(1.0, 0.0, 0.0, 0.0),
                gyroscope = Vector3(0.01, 0.24, -0.12),
                linearAcceleration = Vector3(0.15, -0.04, 0.82),
            ),
            onStart = { _, _ -> },
            onStop = {},
        )
    }
}
