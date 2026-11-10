package com.example.myapplication

import com.example.myapplication.lidar.*
import com.example.myapplication.model.QuaternionData
import com.example.myapplication.sensor.FrameOrientation
import com.sun.net.httpserver.HttpServer
import java.net.InetSocketAddress
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicReference
import org.junit.Assert.*
import org.junit.Test

class PhyphoxLidarClientTest {
    @Test fun actualHttpPollingPauseMalformedResponseRecoveryAndStop() {
        val mode = AtomicReference("valid")
        val requests = AtomicInteger()
        val lastQuery = AtomicReference("")
        val started = System.nanoTime()
        val server = HttpServer.create(InetSocketAddress("127.0.0.1", 0), 0)
        server.createContext("/get") { exchange ->
            requests.incrementAndGet()
            lastQuery.set(exchange.requestURI.rawQuery)
            val time = (System.nanoTime() - started) / 1e9
            val body = when (mode.get()) {
                "paused" -> """{"status":{"session":"test","measuring":false}}"""
                "broken" -> "{not json"
                else -> """{"status":{"session":"test","measuring":true},"buffer":{"x":{"buffer":[1.0]},"t":{"buffer":[$time]}}}"""
            }.toByteArray()
            exchange.sendResponseHeaders(200, body.size.toLong())
            exchange.responseBody.use { it.write(body) }
        }
        server.start()
        val client = PhyphoxLidarClient({ FrameOrientation(QuaternionData(1.0, 0.0, 0.0, 0.0), 20_000_000) }, System::nanoTime)
        try {
            client.start(PhyphoxConfiguration("127.0.0.1", server.address.port))
            await { client.state.value.ready && client.heightAt(System.nanoTime() - 150_000_000) != null }
            assertEquals("x&t", lastQuery.get())
            assertEquals(1.0, client.state.value.depthMetres!!, 1e-9)
            mode.set("paused")
            await { !client.state.value.ready && client.state.value.message.contains("паузе") }
            assertNull(client.heightAt(System.nanoTime() - 150_000_000))
            mode.set("broken")
            await { client.state.value.message.startsWith("LiDAR:") && !client.state.value.ready }
            mode.set("valid")
            await { client.state.value.ready }
            client.stop()
            Thread.sleep(150)
            val stoppedCount = requests.get()
            Thread.sleep(250)
            assertEquals(stoppedCount, requests.get())
            assertFalse(client.state.value.running)
            assertNull(client.heightAt(System.nanoTime() - 150_000_000))
        } finally { client.close(); server.stop(0) }
    }

    @Test fun redirectsAndOversizedResponsesAreNotAccepted() {
        val mode = AtomicReference("redirect")
        val targetRequests = AtomicInteger()
        val server = HttpServer.create(InetSocketAddress("127.0.0.1", 0), 0)
        server.createContext("/target") { exchange ->
            targetRequests.incrementAndGet(); exchange.sendResponseHeaders(204, -1); exchange.close()
        }
        server.createContext("/get") { exchange ->
            if (mode.get() == "redirect") {
                exchange.responseHeaders.add("Location", "/target")
                exchange.sendResponseHeaders(302, -1)
                exchange.close()
            } else {
                val body = ByteArray(PhyphoxProtocol.MAX_RESPONSE_BYTES + 1) { 32 }
                exchange.sendResponseHeaders(200, body.size.toLong())
                exchange.responseBody.use { it.write(body) }
            }
        }
        server.start()
        val client = PhyphoxLidarClient({ null }, System::nanoTime)
        try {
            client.start(PhyphoxConfiguration("127.0.0.1", server.address.port))
            await { client.state.value.message.contains("302") }
            assertEquals(0, targetRequests.get())
            mode.set("large")
            await { client.state.value.message.contains("Слишком большой") }
            assertFalse(client.state.value.ready)
        } finally { client.close(); server.stop(0) }
    }

    private fun await(predicate: () -> Boolean) {
        val deadline = System.nanoTime() + 4_000_000_000
        while (!predicate() && System.nanoTime() < deadline) Thread.sleep(10)
        assertTrue("HTTP client did not reach expected state", predicate())
    }
}
