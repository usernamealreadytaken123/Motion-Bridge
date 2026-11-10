package com.example.myapplication

import com.example.myapplication.lidar.*
import org.junit.Assert.*
import org.junit.Test

class PhyphoxProtocolTest {
    private val configuration = PhyphoxConfiguration("192.168.1.20", 555)
    private fun response(depth: String = "1.2", time: String = "12.3") =
        """{"status":{"session":"abc","measuring":true},"buffer":{"x":{"buffer":[$depth]},"t":{"buffer":[$time]}}}"""

    @Test fun requestMatchesArchiveAndAddsSensorTime() {
        assertEquals("http://192.168.1.20:555/get?x&t", configuration.url().toString())
        val reading = PhyphoxProtocol.parse(response(), configuration)
        assertEquals(1.2, reading.depthMetres!!, 1e-10)
        assertEquals(12.3, reading.experimentTimeSeconds!!, 1e-10)
        assertTrue(reading.measuring)
    }

    @Test fun pausedResponseNeedsNoNumericSample() {
        val reading = PhyphoxProtocol.parse("""{"status":{"session":"abc","measuring":false}}""", configuration)
        assertFalse(reading.measuring)
        assertNull(reading.depthMetres)
    }

    @Test fun invalidOrAmbiguousSamplesNeverBecomeZeroOrFreshHeight() {
        for (depth in listOf("", "null", "\"1.2\"", "true", "-1", "0", "6", "1e999", "1.0,1.2")) {
            rejects { PhyphoxProtocol.parse(response(depth = depth), configuration) }
        }
        for (time in listOf("", "null", "\"12.3\"", "-1", "1e999", "1,2")) {
            rejects { PhyphoxProtocol.parse(response(time = time), configuration) }
        }
        for (body in listOf("{}", "not JSON", response().replace("true", "\"true\""),
            response().replace("\"t\"", "\"wrong\""), " ".repeat(65_537))) {
            rejects { PhyphoxProtocol.parse(body, configuration) }
        }
    }

    @Test fun configurationRejectsNonLanUrlsAndQueryInjection() {
        for (host in listOf("example.com", "8.8.8.8", "http://192.168.1.1", "192.168.1.999",
            "192.168.1.1/", "192.168.001.1", "224.0.0.1")) rejects { PhyphoxConfiguration(host) }
        rejects { PhyphoxConfiguration("192.168.1.1", 0) }
        rejects { PhyphoxConfiguration("192.168.1.1", depthBuffer = "x&cmd=clear") }
        rejects { PhyphoxConfiguration("192.168.1.1", timeBuffer = "x") }
        for (host in listOf("10.0.0.1", "172.16.0.1", "192.168.1.1", "169.254.1.1", "127.0.0.1"))
            PhyphoxConfiguration(host)
    }

    private fun rejects(action: () -> Unit) {
        try { action(); fail("Expected invalid input to be rejected") } catch (_: Exception) { }
    }
}
