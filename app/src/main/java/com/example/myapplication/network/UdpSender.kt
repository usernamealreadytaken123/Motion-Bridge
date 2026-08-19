package com.example.myapplication.network

import java.io.Closeable
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress

class UdpSender(
    host: String,
    private val port: Int,
) : Closeable {
    private val address: InetAddress = InetAddress.getByName(host)
    private val socket = DatagramSocket()

    fun send(json: String) {
        val bytes = json.toByteArray(Charsets.UTF_8)
        val packet = DatagramPacket(bytes, bytes.size, address, port)
        socket.send(packet)
    }

    override fun close() {
        socket.close()
    }
}
