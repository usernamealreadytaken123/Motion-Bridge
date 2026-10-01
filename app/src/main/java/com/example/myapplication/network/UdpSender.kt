package com.example.myapplication.network

import android.content.Context
import android.net.ConnectivityManager
import android.net.NetworkCapabilities
import java.io.Closeable
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.SocketException

class UdpSender(
    context: Context,
    host: String,
    private val port: Int,
) : Closeable {
    private val address: InetAddress = InetAddress.getByName(host)
    private val socket = DatagramSocket()

    init {
        val connectivityManager = context.getSystemService(ConnectivityManager::class.java)
        val wifiNetwork = connectivityManager.allNetworks.firstOrNull { network ->
            connectivityManager.getNetworkCapabilities(network)
                ?.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) == true
        } ?: throw SocketException("Телефон не подключён к Wi-Fi")

        wifiNetwork.bindSocket(socket)
    }

    fun send(json: String) {
        val bytes = json.toByteArray(Charsets.UTF_8)
        val packet = DatagramPacket(bytes, bytes.size, address, port)
        socket.send(packet)
    }

    override fun close() {
        socket.close()
    }
}
