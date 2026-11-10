package com.example.myapplication.model

import java.util.Locale
import java.util.UUID
import com.example.myapplication.vision.PlanarOdometryState
import com.example.myapplication.vision.PlanarOdometryMode

data class Vector3(
    val x: Double,
    val y: Double,
    val z: Double,
)

data class QuaternionData(
    val w: Double,
    val x: Double,
    val y: Double,
    val z: Double,
)

data class MotionPacket(
    val sequence: Long,
    val timestamp: Double,
    val quaternion: QuaternionData,
    val gyroscope: Vector3,
    val linearAcceleration: Vector3,
    val heightAboveSurfaceMeters: Double =
        TrackingConfiguration.FIXED_HEIGHT_ABOVE_SURFACE_METERS,
    val sessionId: UUID,
    val deviceTimestampNanos: Long,
    val quaternionTimestampNanos: Long,
    val gyroscopeTimestampNanos: Long,
    val accelerationTimestampNanos: Long,
    val planarOdometry: PlanarOdometryState? = null,
) {
    fun toJson(): String = String.format(
        Locale.US,
        """{"sequence":%d,"timestamp":%.3f,"quaternion":{"w":%.4f,"x":%.4f,"y":%.4f,"z":%.4f},"gyroscope":{"x":%.4f,"y":%.4f,"z":%.4f},"linear_acceleration":{"x":%.4f,"y":%.4f,"z":%.4f},"height_above_surface_m":%.3f,"protocol_version":2,"packet_type":"telemetry","session_id":"%s","device_timestamp_ns":%d,"sensor_timestamps_ns":{"quaternion":%d,"gyroscope":%d,"linear_acceleration":%d},"planar_odometry":%s}""",
        sequence,
        timestamp,
        quaternion.w,
        quaternion.x,
        quaternion.y,
        quaternion.z,
        gyroscope.x,
        gyroscope.y,
        gyroscope.z,
        linearAcceleration.x,
        linearAcceleration.y,
        linearAcceleration.z,
        heightAboveSurfaceMeters,
        sessionId,
        deviceTimestampNanos,
        quaternionTimestampNanos,
        gyroscopeTimestampNanos,
        accelerationTimestampNanos,
        planarOdometry?.toTelemetryJson() ?: "null",
    )
}

/** Absolute position repeats at 50 Hz; delta is per camera frame, not per UDP packet. */
private fun PlanarOdometryState.toTelemetryJson(): String {
    val base = String.format(
        Locale.US,
        """{"epoch":%d,"timestamp_ns":%d,"status":"%s","camera_active":%s,"x_m":%.4f,"y_m":%.4f,"heading_rad":%.5f,"delta_x_m":%.5f,"delta_y_m":%.5f,"surface_distance_m":%.3f,"tracked_points":%d,"inliers":%d,"inlier_ratio":%.4f,"image_scale":%.5f,"intrinsics_approximate":%s}""",
        trackingEpoch,
        timestampNanos,
        status.name,
        isCameraActive,
        xMetres,
        yMetres,
        headingRadians,
        lastDeltaXMetres,
        lastDeltaYMetres,
        surfaceDistanceMetres,
        trackedPoints,
        inliers,
        inlierRatio,
        imageScale,
        intrinsicsAreApproximate,
    )
    // Optional protocol-v2 extension: existing PC clients keep reading the absolute X/Y fields.
    if (mode == PlanarOdometryMode.LEGACY) return base
    fun Double?.jsonNumber() = if (this != null && isFinite()) String.format(Locale.US, "%.5f", this) else "null"
    val tilt = base.dropLast(1) + ",\"mode\":\"TILT_COMPENSATED\",\"tilt_deg\":" + tiltDegrees.jsonNumber() +
        ",\"orientation_gap_ms\":" + orientationGapMillis.jsonNumber() +
        ",\"ground_residual_m\":" + groundResidualMetres.jsonNumber()
    if (!usesLidarHeight) return "$tilt}"
    return tilt + ",\"height_source\":\"IPHONE_LIDAR\",\"height_valid\":" + heightIsValid +
        ",\"lidar_depth_m\":" + lidarDepthMetres.jsonNumber() +
        ",\"lidar_age_ms\":" + lidarAgeMillis.jsonNumber() +
        ",\"lidar_rtt_ms\":" + lidarRoundTripMillis.jsonNumber() +
        ",\"lidar_time_s\":" + lidarTimeSeconds.jsonNumber() + "}"
}
