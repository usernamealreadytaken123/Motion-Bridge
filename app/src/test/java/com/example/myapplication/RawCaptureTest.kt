package com.example.myapplication

import com.example.myapplication.capture.*
import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream
import java.io.DataInputStream
import java.net.ServerSocket
import java.nio.ByteBuffer
import org.json.JSONObject
import org.junit.Assert.*
import org.junit.Test

class RawCaptureTest {
    private fun read(input: DataInputStream): Pair<JSONObject, ByteArray> {
        val magic = ByteArray(4); input.readFully(magic)
        assertEquals("MVO1", magic.toString(Charsets.US_ASCII))
        val headerSize = input.readInt()
        val payloadSize = input.readInt()
        assertTrue(headerSize in 1..65536)
        assertTrue(payloadSize in 0..1280 * 960)
        val header = ByteArray(headerSize); input.readFully(header)
        val pixels = ByteArray(payloadSize); input.readFully(pixels)
        return JSONObject(header.toString(Charsets.UTF_8)) to pixels
    }

    @Test fun binaryEnvelopeUsesBigEndianLengthsAndUnmodifiedPixels() {
        val output = ByteArrayOutputStream()
        val pixels = byteArrayOf(0, 10, 13, -1, 77)
        RawWire.write(output, JSONObject().put("type", "frame").put("timestamp_ns", 987654321012345L), pixels)
        val (header, payload) = read(DataInputStream(ByteArrayInputStream(output.toByteArray())))
        assertEquals(987654321012345L, header.getLong("timestamp_ns"))
        assertArrayEquals(pixels, payload)
    }

    @Test fun lumaCopyHandlesRowPaddingPixelStrideAndBufferOffsetWithoutMovingSource() {
        for (pixelStride in listOf(1, 2)) {
            val rowStride = 16 * pixelStride + 7
            val buffer = ByteBuffer.allocate(rowStride * 16 + 9)
            for (y in 0 until 16) for (x in 0 until 16) buffer.put(9 + y * rowStride + x * pixelStride, (y * 16 + x).toByte())
            buffer.position(9)
            assertArrayEquals(ByteArray(256) { it.toByte() }, RawWire.copyLuma(buffer, 16, 16, rowStride, pixelStride))
            assertEquals(9, buffer.position())
        }
        assertThrows(IllegalArgumentException::class.java) { RawWire.copyLuma(ByteBuffer.allocate(20), 16, 16, 16, 1) }
    }

    @Test fun endpointsAreRestrictedToLocalIpv4AndPairingCode() {
        for (host in listOf("127.0.0.1", "192.168.1.22", "10.0.1.2", "172.31.2.3", "169.254.1.2"))
            RawEndpoint(host, 5006, "1234abcd")
        for (host in listOf("0.0.0.0", "8.8.8.8", "172.32.1.1", "example.com", "192.168.1.999"))
            assertThrows(IllegalArgumentException::class.java) { RawEndpoint(host, 5006, "1234abcd") }
        assertThrows(IllegalArgumentException::class.java) { RawEndpoint("127.0.0.1", 0, "1234abcd") }
        assertThrows(IllegalArgumentException::class.java) { RawEndpoint("127.0.0.1", 5006, "bad") }
        RawEndpoint("127.0.0.1", 5006, "1234abcd", encoding = CaptureEncoding.H264)
        assertThrows(IllegalArgumentException::class.java) {
            RawEndpoint("127.0.0.1", 5006, "1234abcd", encoding = CaptureEncoding.H264, videoBitrate = 100_000)
        }
    }

    @Test fun h264PlaneHelpersRespectStrideAndUseNeutralChroma() {
        val source = ByteArray(16) { it.toByte() }
        val target = ByteBuffer.allocate(40)
        target.position(3)
        H264CaptureEncoder.copyPlane(source, 4, 4, target, 8, 2)
        for (y in 0 until 4) for (x in 0 until 4)
            assertEquals(source[y * 4 + x], target.get(3 + y * 8 + x * 2))
        H264CaptureEncoder.fillPlane(128.toByte(), 2, 2, target, 8, 2)
        for (y in 0 until 2) for (x in 0 until 2)
            assertEquals(128.toByte(), target.get(3 + y * 8 + x * 2))
        assertEquals(3, target.position())
        val avcc = byteArrayOf(0, 0, 0, 3, 0x67, 1, 2, 0, 0, 0, 2, 0x68, 3)
        assertArrayEquals(byteArrayOf(0, 0, 0, 1, 0x67, 1, 2, 0, 0, 0, 1, 0x68, 3),
            H264CaptureEncoder.annexB(avcc))
        val annexB = byteArrayOf(0, 0, 0, 1, 0x65, 9)
        assertSame(annexB, H264CaptureEncoder.annexB(annexB))
    }

    @Test fun actualTcpTransmitsAllQueuedSensorsBeforeAdvancingWatermarkAndFrame() {
        ServerSocket(0).use { server ->
            server.soTimeout = 3000
            val sender = RawCaptureSender(System::nanoTime)
            try {
                sender.start(RawEndpoint("127.0.0.1", server.localPort, "1234abcd"))
                server.accept().use { socket ->
                    socket.soTimeout = 3000
                    val input = DataInputStream(socket.getInputStream())
                    val (hello, _) = read(input)
                    assertEquals("hello", hello.getString("type"))
                    assertEquals("1234abcd", hello.getString("token"))
                    socket.getOutputStream().write(1)
                    await { sender.state.value.connected }
                    val t = System.nanoTime()
                    synchronized(sender) {
                        repeat(600) { i -> sender.offerSensor("gyro", t + i, doubleArrayOf(i.toDouble(), 2.0, 3.0)) }
                        val run = checkNotNull(sender.reserveFrame(t))
                        assertNull(sender.reserveFrame(t + 1))
                        sender.offerFrame(JSONObject().put("timestamp_ns", t), ByteArray(256) { 42 }, run)
                    }
                    var count = 0
                    var frame = false
                    var advanced = false
                    var lastWatermark = 0L
                    while (!frame || !advanced) {
                        val (header, payload) = read(input)
                        if (header.getString("type") == "sensors") {
                            val samples = header.getJSONArray("samples")
                            assertTrue(samples.length() <= 256)
                            repeat(samples.length()) { index ->
                                assertEquals(count.toDouble(), samples.getJSONArray(index).getDouble(2), 0.0)
                                count++
                            }
                            val watermark = header.getLong("watermark_ns")
                            assertTrue(watermark >= lastWatermark)
                            lastWatermark = watermark
                            if (watermark > t) { assertEquals(600, count); advanced = true }
                        } else if (header.getString("type") == "frame") {
                            assertEquals(600, count)
                            assertEquals(0L, header.getLong("seq"))
                            assertArrayEquals(ByteArray(256) { 42 }, payload)
                            frame = true
                        }
                    }
                    sender.stop()
                    assertFalse(sender.state.value.running)
                    assertNull(sender.reserveFrame(System.nanoTime()))
                }
            } finally { sender.close() }
        }
    }

    @Test fun invalidHandshakeStopsWithoutSendingSensors() {
        ServerSocket(0).use { server ->
            server.soTimeout = 3000
            val sender = RawCaptureSender(System::nanoTime)
            try {
                sender.start(RawEndpoint("127.0.0.1", server.localPort, "1234abcd"))
                server.accept().use { socket ->
                    read(DataInputStream(socket.getInputStream()))
                    socket.getOutputStream().write(0)
                    await { !sender.state.value.running }
                    assertTrue(sender.state.value.message.contains("код"))
                }
            } finally { sender.close() }
        }
    }

    @Test fun h264HandshakeUsesProtocolTwoWithoutStartingCodecEarly() {
        ServerSocket(0).use { server ->
            server.soTimeout = 3000
            val sender = RawCaptureSender(System::nanoTime)
            try {
                sender.start(RawEndpoint("127.0.0.1", server.localPort, "1234abcd",
                    encoding = CaptureEncoding.H264))
                server.accept().use { socket ->
                    val (hello, payload) = read(DataInputStream(socket.getInputStream()))
                    assertEquals(2, hello.getInt("protocol"))
                    assertEquals("h264", hello.getString("encoding"))
                    assertEquals(0, payload.size)
                    socket.getOutputStream().write(1)
                    await { sender.state.value.connected }
                    assertEquals(CaptureEncoding.H264, sender.state.value.encoding)
                }
            } finally { sender.close() }
        }
    }

    private fun await(condition: () -> Boolean) {
        val deadline = System.nanoTime() + 3_000_000_000L
        while (!condition() && System.nanoTime() < deadline) Thread.sleep(5)
        assertTrue("Timed out", condition())
    }
}
