package com.example.covenantgroundstation

import android.app.Activity
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.LinearGradient
import android.graphics.Paint
import android.graphics.RadialGradient
import android.graphics.RectF
import android.graphics.Shader
import android.os.Bundle
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.Window
import android.view.WindowManager
import org.json.JSONObject
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.Inet4Address
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.ArrayDeque
import java.util.Locale
import kotlin.concurrent.thread
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt


private const val UDP_LIDAR_PORT = 56000
private const val UDP_TELEMETRY_PORT = 56010
private const val UDP_ARM_PORT = 56020

private const val RASPBERRY_IP = "192.168.1.100"

private const val UDP_RECV_SIZE_BYTES = 4096

private const val PACKET_MAGIC = "CVL1"
private const val PROTOCOL_VERSION = 1
private const val PACKET_TYPE_RAW_LIDAR = 1
private const val UDP_HEADER_SIZE = 20

private const val FRAME_HEADER = 0x54
private const val FRAME_VER_LEN = 0x2C
private const val FRAME_LENGTH = 47
private const val POINTS_PER_FRAME = 12

private const val MIN_DISTANCE_M = 0.05f
private const val MAX_DISTANCE_M = 12.0f

private const val GRID_SPACING_M = 5.0f
private const val MAX_POINTS_IN_MEMORY = 420_000
private const val MAX_POINTS_DISPLAYED = 36_000
private const val UI_REFRESH_MS = 350L

private const val CONNECTION_TIMEOUT_MS = 2500L
private const val MAP_YAW_SIGN = 1.0f
private const val POSE_POSITION_EPSILON_M = 0.02f


data class ScanStats(
    val elapsedTimeS: Float = 0.0f,
    val scans: Int = 0,
    val frames: Int = 0,
    val pointsReceived: Int = 0,
)


data class TelemetryState(
    val lastPacketWallMs: Long = 0L,

    val batteryReceived: Boolean = false,
    val imuReceived: Boolean = false,
    val poseReceived: Boolean = false,
    val twistReceived: Boolean = false,

    val batteryPercent: Float? = null,
    val voltageV: Float? = null,
    val currentA: Float? = null,
    val batteryValid: Boolean? = null,

    val velocityMps: Float? = null,
    val velocityXMps: Float? = null,
    val velocityYMps: Float? = null,
    val velocityZMps: Float? = null,

    val altitudeM: Float? = null,
    val positionXM: Float? = null,
    val positionYM: Float? = null,
    val positionZM: Float? = null,

    val flightMode: String? = null,
    val armed: Boolean? = null,
    val health: String? = null,

    val rollDeg: Float? = null,
    val pitchDeg: Float? = null,
    val yawDeg: Float? = null,
    val yawRelativeDeg: Float? = null,

    val poseYawRelativeDeg: Float? = null,
    val mapYawRelativeDeg: Float? = null,
    val mapYawSource: String? = null,

    val gyroZRadS: Float? = null,
    val accelZMS2: Float? = null,
    val imuFrameId: String? = null,
) {
    fun isOnline(nowMs: Long): Boolean {
        return lastPacketWallMs > 0L && nowMs - lastPacketWallMs < CONNECTION_TIMEOUT_MS
    }

    fun hasUsablePosePosition(): Boolean {
        val x = positionXM ?: return false
        val y = positionYM ?: return false
        val z = positionZM ?: 0.0f

        return poseReceived && (abs(x) + abs(y) + abs(z)) > POSE_POSITION_EPSILON_M
    }

    companion object {
        fun fromJson(json: JSONObject): TelemetryState {
            return TelemetryState(
                lastPacketWallMs = System.currentTimeMillis(),

                batteryReceived = json.optBoolean("battery_received", false),
                imuReceived = json.optBoolean("imu_received", false),
                poseReceived = json.optBoolean("pose_received", false),
                twistReceived = json.optBoolean("twist_received", false),

                batteryPercent = json.optFloatOrNull("battery_percent", "battery_pct", "battery"),
                voltageV = json.optFloatOrNull("voltage_v", "battery_voltage_v", "voltage"),
                currentA = json.optFloatOrNull("current_a", "battery_current_a", "current"),
                batteryValid = json.optBooleanOrNull("battery_valid"),

                velocityMps = json.optFloatOrNull("velocity_mps", "speed_mps", "ground_speed_mps", "speed"),
                velocityXMps = json.optFloatOrNull("velocity_x_mps"),
                velocityYMps = json.optFloatOrNull("velocity_y_mps"),
                velocityZMps = json.optFloatOrNull("velocity_z_mps"),

                altitudeM = json.optFloatOrNull("altitude_m", "alt_m", "relative_altitude_m", "z_m"),
                positionXM = json.optFloatOrNull("position_x_m"),
                positionYM = json.optFloatOrNull("position_y_m"),
                positionZM = json.optFloatOrNull("position_z_m"),

                flightMode = json.optStringOrNull("flight_mode", "mode", "nav_mode"),
                armed = json.optBooleanOrNull("armed", "is_armed"),
                health = json.optStringOrNull("health", "state", "status"),

                rollDeg = json.optFloatOrNull("roll_deg"),
                pitchDeg = json.optFloatOrNull("pitch_deg"),
                yawDeg = json.optFloatOrNull("yaw_deg"),
                yawRelativeDeg = json.optFloatOrNull("yaw_relative_deg"),

                poseYawRelativeDeg = json.optFloatOrNull("pose_yaw_relative_deg"),
                mapYawRelativeDeg = json.optFloatOrNull("map_yaw_relative_deg"),
                mapYawSource = json.optStringOrNull("map_yaw_source"),

                gyroZRadS = json.optFloatOrNull("gyro_z_rad_s"),
                accelZMS2 = json.optFloatOrNull("accel_z_m_s2"),
                imuFrameId = json.optStringOrNull("imu_frame_id")
            )
        }
    }
}


class MainActivity : Activity() {

    private lateinit var groundStationView: GroundStationView

    private var lidarReceiver: UdpLidarReceiver? = null
    private var telemetryReceiver: RosTelemetryReceiver? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        requestWindowFeature(Window.FEATURE_NO_TITLE)

        window.setFlags(
            WindowManager.LayoutParams.FLAG_FULLSCREEN,
            WindowManager.LayoutParams.FLAG_FULLSCREEN
        )

        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        groundStationView = GroundStationView(this)
        setContentView(groundStationView)
    }

    override fun onStart() {
        super.onStart()

        lidarReceiver = UdpLidarReceiver(
            onScanReady = { localScanPoints, stats ->
                runOnUiThread {
                    groundStationView.addScan(localScanPoints, stats)
                }
            }
        )

        telemetryReceiver = RosTelemetryReceiver(
            onTelemetry = { telemetry ->
                runOnUiThread {
                    groundStationView.updateTelemetry(telemetry)
                }
            }
        )

        lidarReceiver?.start()
        telemetryReceiver?.start()
    }

    override fun onStop() {
        lidarReceiver?.stop()
        lidarReceiver = null

        telemetryReceiver?.stop()
        telemetryReceiver = null

        super.onStop()
    }
}


class ArmCommandSender {

    fun sendArmState(armed: Boolean) {
        thread(start = true, name = "CovenantArmCommandSender") {
            try {
                val payload = if (armed) "1" else "0"
                val data = payload.toByteArray(Charsets.US_ASCII)

                DatagramSocket().use { socket ->
                    val packet = DatagramPacket(
                        data,
                        data.size,
                        InetAddress.getByName(RASPBERRY_IP),
                        UDP_ARM_PORT
                    )

                    socket.send(packet)
                }
            } catch (_: Exception) {
            }
        }
    }
}


class UdpLidarReceiver(
    private val onScanReady: (FloatArray, ScanStats) -> Unit,
) {
    @Volatile
    private var running = false

    private var socket: DatagramSocket? = null
    private var receiverThread: Thread? = null

    fun start() {
        if (running) return

        running = true

        receiverThread = thread(start = true, name = "CovenantUdpLidarReceiver") {
            receiveLoop()
        }
    }

    fun stop() {
        running = false
        socket?.close()
        socket = null
        receiverThread?.interrupt()
        receiverThread = null
    }

    private fun receiveLoop() {
        val lidarBuffer = ByteArrayBuffer()
        val currentScanAngles = ArrayList<Float>(900)
        val currentScanDistances = ArrayList<Float>(900)

        var previousStartAngle: Float? = null
        var scanCount = 0
        var frameCount = 0
        var pointsReceived = 0

        val startTimeNs = System.nanoTime()

        try {
            val udpSocket = DatagramSocket(null)
            udpSocket.reuseAddress = true
            udpSocket.bind(InetSocketAddress("0.0.0.0", UDP_LIDAR_PORT))
            socket = udpSocket

            val packetBuffer = ByteArray(UDP_RECV_SIZE_BYTES)
            val packet = DatagramPacket(packetBuffer, packetBuffer.size)

            while (running) {
                packet.length = packetBuffer.size
                udpSocket.receive(packet)

                val payload = parseUdpPacket(packet.data, packet.length) ?: continue
                lidarBuffer.append(payload)

                while (lidarBuffer.size() >= FRAME_LENGTH) {
                    val frame = lidarBuffer.extractNextLidarFrame() ?: break
                    val parsedFrame = parseLidarFrame(frame) ?: continue

                    frameCount += 1

                    val startAngle = parsedFrame.startAngleDeg

                    val newScanDetected =
                        previousStartAngle != null &&
                                startAngle < previousStartAngle!! &&
                                currentScanAngles.size > 100

                    if (newScanDetected) {
                        scanCount += 1

                        val elapsedTimeS =
                            (System.nanoTime() - startTimeNs).toFloat() / 1_000_000_000.0f

                        val localScanPoints = buildLocalScan(
                            currentScanAngles,
                            currentScanDistances
                        )

                        pointsReceived += localScanPoints.size / 3

                        val stats = ScanStats(
                            elapsedTimeS = elapsedTimeS,
                            scans = scanCount,
                            frames = frameCount,
                            pointsReceived = pointsReceived,
                        )

                        onScanReady(localScanPoints, stats)

                        currentScanAngles.clear()
                        currentScanDistances.clear()
                    }

                    for (point in parsedFrame.points) {
                        currentScanAngles.add(point.angleDeg)
                        currentScanDistances.add(point.distanceM)
                    }

                    previousStartAngle = startAngle
                }
            }
        } catch (_: Exception) {
        } finally {
            socket?.close()
            socket = null
        }
    }

    private fun parseUdpPacket(data: ByteArray, length: Int): ByteArray? {
        if (length < UDP_HEADER_SIZE) return null

        val buffer = ByteBuffer.wrap(data, 0, length)
        buffer.order(ByteOrder.BIG_ENDIAN)

        val magicBytes = ByteArray(4)
        buffer.get(magicBytes)

        val magic = magicBytes.toString(Charsets.US_ASCII)
        if (magic != PACKET_MAGIC) return null

        val version = buffer.get().toInt() and 0xFF
        val packetType = buffer.get().toInt() and 0xFF

        if (version != PROTOCOL_VERSION) return null
        if (packetType != PACKET_TYPE_RAW_LIDAR) return null

        buffer.int
        buffer.long

        val payloadLength = buffer.short.toInt() and 0xFFFF
        val payloadStart = UDP_HEADER_SIZE
        val payloadEnd = payloadStart + payloadLength

        if (payloadEnd > length) return null

        return data.copyOfRange(payloadStart, payloadEnd)
    }

    private fun parseLidarFrame(frame: ByteArray): LidarFrame? {
        if (frame.size != FRAME_LENGTH) return null

        val header = frame[0].toInt() and 0xFF
        val versionLength = frame[1].toInt() and 0xFF

        if (header != FRAME_HEADER || versionLength != FRAME_VER_LEN) return null

        val startAngleDeg = readUInt16LE(frame, 4) / 100.0f

        val rawPoints = ArrayList<LidarPoint>(POINTS_PER_FRAME)

        var offset = 6

        repeat(POINTS_PER_FRAME) {
            val distanceMm = readUInt16LE(frame, offset)
            val confidence = frame[offset + 2].toInt() and 0xFF
            offset += 3

            rawPoints.add(
                LidarPoint(
                    angleDeg = 0.0f,
                    distanceM = distanceMm / 1000.0f,
                    confidence = confidence
                )
            )
        }

        val endAngleDeg = readUInt16LE(frame, offset) / 100.0f

        var angleDelta = endAngleDeg - startAngleDeg

        if (angleDelta < 0.0f) {
            angleDelta += 360.0f
        }

        val angleStep = angleDelta / (POINTS_PER_FRAME - 1)
        val points = ArrayList<LidarPoint>(POINTS_PER_FRAME)

        for (i in rawPoints.indices) {
            val angleDeg = (startAngleDeg + i * angleStep) % 360.0f
            val distanceM = rawPoints[i].distanceM

            if (distanceM in MIN_DISTANCE_M..MAX_DISTANCE_M) {
                points.add(
                    LidarPoint(
                        angleDeg = angleDeg,
                        distanceM = distanceM,
                        confidence = rawPoints[i].confidence
                    )
                )
            }
        }

        return LidarFrame(
            startAngleDeg = startAngleDeg,
            endAngleDeg = endAngleDeg,
            points = points
        )
    }

    private fun readUInt16LE(data: ByteArray, offset: Int): Int {
        val b0 = data[offset].toInt() and 0xFF
        val b1 = data[offset + 1].toInt() and 0xFF
        return b0 or (b1 shl 8)
    }

    private fun buildLocalScan(
        angles: List<Float>,
        distances: List<Float>,
    ): FloatArray {
        val pointCount = min(angles.size, distances.size)
        val xyz = FloatArray(pointCount * 3)

        var out = 0

        for (i in 0 until pointCount) {
            val angleRad = angles[i] * PI.toFloat() / 180.0f
            val distance = distances[i]

            val x = 0.0f
            val y = distance * cos(angleRad)
            val z = distance * sin(angleRad)

            xyz[out++] = x
            xyz[out++] = y
            xyz[out++] = z
        }

        return xyz
    }
}


class RosTelemetryReceiver(
    private val onTelemetry: (TelemetryState) -> Unit,
) {
    @Volatile
    private var running = false

    private var socket: DatagramSocket? = null
    private var receiverThread: Thread? = null

    fun start() {
        if (running) return

        running = true

        receiverThread = thread(start = true, name = "CovenantRosTelemetryReceiver") {
            receiveLoop()
        }
    }

    fun stop() {
        running = false
        socket?.close()
        socket = null
        receiverThread?.interrupt()
        receiverThread = null
    }

    private fun receiveLoop() {
        try {
            val udpSocket = DatagramSocket(null)
            udpSocket.reuseAddress = true
            udpSocket.bind(InetSocketAddress("0.0.0.0", UDP_TELEMETRY_PORT))

            socket = udpSocket

            val packetBuffer = ByteArray(UDP_RECV_SIZE_BYTES)
            val packet = DatagramPacket(packetBuffer, packetBuffer.size)

            while (running) {
                packet.length = packetBuffer.size
                udpSocket.receive(packet)

                try {
                    val text = String(
                        packet.data,
                        packet.offset,
                        packet.length,
                        Charsets.UTF_8
                    ).trim()

                    val json = JSONObject(text)
                    val telemetry = TelemetryState.fromJson(json)

                    onTelemetry(telemetry)
                } catch (_: Exception) {
                }
            }
        } catch (_: Exception) {
        } finally {
            socket?.close()
            socket = null
        }
    }
}


class ByteArrayBuffer {
    private var buffer = ByteArray(0)

    fun size(): Int = buffer.size

    fun append(data: ByteArray) {
        if (data.isEmpty()) return

        val newBuffer = ByteArray(buffer.size + data.size)

        System.arraycopy(buffer, 0, newBuffer, 0, buffer.size)
        System.arraycopy(data, 0, newBuffer, buffer.size, data.size)

        buffer = newBuffer

        if (buffer.size > 16384) {
            buffer = buffer.copyOfRange(buffer.size - 16384, buffer.size)
        }
    }

    fun extractNextLidarFrame(): ByteArray? {
        while (buffer.size >= FRAME_LENGTH) {
            val headerIndex = buffer.indexOfByte(FRAME_HEADER.toByte())

            if (headerIndex < 0) {
                buffer = ByteArray(0)
                return null
            }

            if (headerIndex > 0) {
                buffer = buffer.copyOfRange(headerIndex, buffer.size)
            }

            if (buffer.size < FRAME_LENGTH) return null

            val secondByte = buffer[1].toInt() and 0xFF

            if (secondByte != FRAME_VER_LEN) {
                buffer = buffer.copyOfRange(1, buffer.size)
                continue
            }

            val frame = buffer.copyOfRange(0, FRAME_LENGTH)
            buffer = buffer.copyOfRange(FRAME_LENGTH, buffer.size)

            return frame
        }

        return null
    }

    private fun ByteArray.indexOfByte(value: Byte): Int {
        for (i in indices) {
            if (this[i] == value) return i
        }

        return -1
    }
}


data class LidarPoint(
    val angleDeg: Float,
    val distanceM: Float,
    val confidence: Int,
)


data class LidarFrame(
    val startAngleDeg: Float,
    val endAngleDeg: Float,
    val points: List<LidarPoint>,
)


data class DashboardLayout(
    val mapRect: RectF,
    val panelRect: RectF,
    val isLandscape: Boolean,
)


data class MapPose(
    val x: Float,
    val y: Float,
    val z: Float,
    val yawDeg: Float,
    val source: String,
)


private enum class TouchMode {
    NONE,
    MAP,
    PANEL_SCROLL,
    ARM_BUTTON,
}


class GroundStationView(
    context: android.content.Context
) : android.view.View(context) {

    private val lock = Any()
    private val armCommandSender = ArmCommandSender()

    private val pointChunks = ArrayDeque<FloatArray>()
    private var totalPointCount = 0

    private val trajectoryPoints = ArrayDeque<FloatArray>()

    private var latestStats = ScanStats()
    private var latestTelemetry = TelemetryState()
    private var latestScanWallMs = 0L

    private val localIpAddress = findLocalIpv4Address() ?: "0.0.0.0"

    private var scalePxPerMeter = 54.0f
    private var panX = 0.0f
    private var panY = 0.0f

    private var integratedX = 0.0f
    private var integratedY = 0.0f
    private var integratedZ = 0.0f
    private var lastIntegrationWallMs = 0L

    private var lastTouchX = 0.0f
    private var lastTouchY = 0.0f

    private var touchMode = TouchMode.NONE

    private var landscapePanelScrollY = 0.0f
    private var landscapePanelMaxScrollY = 0.0f

    private var armButtonRect = RectF()
    private var landscapeScrollableRect = RectF()

    private var requestedArmed = false

    private val scaleDetector = ScaleGestureDetector(
        context,
        object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
            override fun onScale(detector: ScaleGestureDetector): Boolean {
                if (touchMode != TouchMode.MAP) return false

                scalePxPerMeter *= detector.scaleFactor
                scalePxPerMeter = scalePxPerMeter.coerceIn(10.0f, 240.0f)

                invalidate()
                return true
            }
        }
    )

    private val backgroundPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(2, 4, 8)
        style = Paint.Style.FILL
    }

    private val mapBackgroundPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
    }

    private val panelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(7, 12, 20)
        style = Paint.Style.FILL
    }

    private val panelStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(34, 51, 76)
        style = Paint.Style.STROKE
        strokeWidth = 2.0f
    }

    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(32, 48, 70)
        strokeWidth = 1.2f
        alpha = 115
    }

    private val gridStrongPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(55, 84, 118)
        strokeWidth = 1.8f
        alpha = 150
    }

    private val gridTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(105, 132, 162)
        textSize = 21.0f
    }

    private val trajectoryPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(82, 255, 40, 62)
        strokeWidth = 3.2f
        style = Paint.Style.STROKE
    }

    private val takeoffPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(255, 215, 100)
        style = Paint.Style.STROKE
        strokeWidth = 3.4f
    }

    private val takeoffFillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(55, 255, 215, 100)
        style = Paint.Style.FILL
    }

    private val dronePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(255, 48, 70)
        style = Paint.Style.FILL
    }

    private val titlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 42.0f
        isFakeBoldText = true
    }

    private val sectionPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(126, 196, 255)
        textSize = 22.0f
        isFakeBoldText = true
    }

    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(122, 145, 174)
        textSize = 21.0f
    }

    private val valuePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(232, 248, 255)
        textSize = 26.0f
        isFakeBoldText = true
    }

    private val smallValuePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(220, 236, 245)
        textSize = 22.0f
        isFakeBoldText = true
    }

    private val pointHaloPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = Color.argb(155, 120, 235, 255)
    }

    private val pointCorePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = Color.WHITE
        alpha = 255
    }

    private val statusOnlineColor = Color.rgb(70, 255, 150)
    private val statusOfflineColor = Color.rgb(255, 72, 88)

    fun addScan(localScanPoints: FloatArray, stats: ScanStats) {
        val poseSnapshot = synchronized(lock) {
            computeCurrentMapPoseLocked()
        }

        val globalPoints = transformScanToMap(localScanPoints, poseSnapshot)

        synchronized(lock) {
            latestStats = stats
            latestScanWallMs = System.currentTimeMillis()

            pointChunks.addLast(globalPoints)
            totalPointCount += globalPoints.size / 3

            while (totalPointCount > MAX_POINTS_IN_MEMORY && !pointChunks.isEmpty()) {
                val removed = pointChunks.removeFirst()
                totalPointCount -= removed.size / 3
            }

            if (
                trajectoryPoints.isEmpty() ||
                distance2D(
                    trajectoryPoints.last()[0],
                    trajectoryPoints.last()[1],
                    poseSnapshot.x,
                    poseSnapshot.y
                ) > 0.05f
            ) {
                trajectoryPoints.addLast(floatArrayOf(poseSnapshot.x, poseSnapshot.y, poseSnapshot.z))

                while (trajectoryPoints.size > 3000) {
                    trajectoryPoints.removeFirst()
                }
            }
        }

        invalidate()
    }

    fun updateTelemetry(telemetry: TelemetryState) {
        synchronized(lock) {
            integrateTwistLocked(telemetry)
            latestTelemetry = telemetry

            if (telemetry.armed != null) {
                requestedArmed = telemetry.armed
            }
        }

        invalidate()
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        val layout = computeLayout()

        if (event.actionMasked == MotionEvent.ACTION_DOWN) {
            lastTouchX = event.x
            lastTouchY = event.y

            touchMode = when {
                armButtonRect.contains(event.x, event.y) -> TouchMode.ARM_BUTTON
                layout.isLandscape && landscapeScrollableRect.contains(event.x, event.y) -> TouchMode.PANEL_SCROLL
                layout.mapRect.contains(event.x, event.y) -> TouchMode.MAP
                else -> TouchMode.NONE
            }
        }

        if (touchMode == TouchMode.MAP) {
            scaleDetector.onTouchEvent(event)
        }

        when (event.actionMasked) {
            MotionEvent.ACTION_MOVE -> {
                when (touchMode) {
                    TouchMode.MAP -> {
                        if (event.pointerCount <= 1) {
                            val dx = event.x - lastTouchX
                            val dy = event.y - lastTouchY

                            panX += dx
                            panY += dy

                            lastTouchX = event.x
                            lastTouchY = event.y

                            invalidate()
                        }
                    }

                    TouchMode.PANEL_SCROLL -> {
                        val dy = event.y - lastTouchY

                        landscapePanelScrollY -= dy
                        landscapePanelScrollY = landscapePanelScrollY.coerceIn(
                            0.0f,
                            landscapePanelMaxScrollY
                        )

                        lastTouchY = event.y

                        invalidate()
                    }

                    else -> {
                    }
                }

                return true
            }

            MotionEvent.ACTION_UP -> {
                if (touchMode == TouchMode.ARM_BUTTON && armButtonRect.contains(event.x, event.y)) {
                    toggleArmState()
                }

                touchMode = TouchMode.NONE
                return true
            }

            MotionEvent.ACTION_CANCEL -> {
                touchMode = TouchMode.NONE
                return true
            }
        }

        return true
    }

    private fun integrateTwistLocked(newTelemetry: TelemetryState) {
        val now = newTelemetry.lastPacketWallMs
        val previous = latestTelemetry

        if (now <= 0L) return

        if (newTelemetry.hasUsablePosePosition()) {
            integratedX = newTelemetry.positionXM ?: integratedX
            integratedY = newTelemetry.positionYM ?: integratedY
            integratedZ = newTelemetry.positionZM ?: integratedZ
            lastIntegrationWallMs = now
            return
        }

        if (lastIntegrationWallMs == 0L) {
            lastIntegrationWallMs = now
            return
        }

        val dt = ((now - lastIntegrationWallMs).toFloat() / 1000.0f).coerceIn(0.0f, 0.5f)
        lastIntegrationWallMs = now

        if (!newTelemetry.twistReceived) return

        val vx = newTelemetry.velocityXMps ?: return
        val vy = newTelemetry.velocityYMps ?: 0.0f
        val vz = newTelemetry.velocityZMps ?: 0.0f

        val yawDeg =
            newTelemetry.mapYawRelativeDeg
                ?: newTelemetry.poseYawRelativeDeg
                ?: newTelemetry.yawRelativeDeg
                ?: previous.mapYawRelativeDeg
                ?: previous.yawRelativeDeg
                ?: 0.0f

        val yawRad = yawDeg * MAP_YAW_SIGN * PI.toFloat() / 180.0f
        val cosYaw = cos(yawRad)
        val sinYaw = sin(yawRad)

        val globalVx = vx * cosYaw - vy * sinYaw
        val globalVy = vx * sinYaw + vy * cosYaw

        integratedX += globalVx * dt
        integratedY += globalVy * dt
        integratedZ += vz * dt
    }

    private fun computeCurrentMapPoseLocked(): MapPose {
        val telemetry = latestTelemetry

        val yaw =
            telemetry.mapYawRelativeDeg
                ?: telemetry.poseYawRelativeDeg
                ?: telemetry.yawRelativeDeg
                ?: 0.0f

        if (telemetry.hasUsablePosePosition()) {
            return MapPose(
                x = telemetry.positionXM ?: 0.0f,
                y = telemetry.positionYM ?: 0.0f,
                z = telemetry.positionZM ?: 0.0f,
                yawDeg = yaw,
                source = "POSE"
            )
        }

        return MapPose(
            x = integratedX,
            y = integratedY,
            z = integratedZ,
            yawDeg = yaw,
            source = "TWIST"
        )
    }

    private fun transformScanToMap(localScanPoints: FloatArray, pose: MapPose): FloatArray {
        val yawRad = pose.yawDeg * MAP_YAW_SIGN * PI.toFloat() / 180.0f
        val cosYaw = cos(yawRad)
        val sinYaw = sin(yawRad)

        val transformed = FloatArray(localScanPoints.size)

        var i = 0

        while (i + 2 < localScanPoints.size) {
            val localX = localScanPoints[i]
            val localY = localScanPoints[i + 1]
            val localZ = localScanPoints[i + 2]

            val globalX = pose.x + localX * cosYaw - localY * sinYaw
            val globalY = pose.y + localX * sinYaw + localY * cosYaw
            val globalZ = pose.z + localZ

            transformed[i] = globalX
            transformed[i + 1] = globalY
            transformed[i + 2] = globalZ

            i += 3
        }

        return transformed
    }

    private fun distance2D(x1: Float, y1: Float, x2: Float, y2: Float): Float {
        val dx = x2 - x1
        val dy = y2 - y1
        return sqrt(dx * dx + dy * dy)
    }

    private fun currentArmStateForUi(): Boolean {
        return latestTelemetry.armed ?: requestedArmed
    }

    private fun armStateLabel(): String {
        return if (currentArmStateForUi()) "ARMED" else "DISARMED"
    }

    private fun armStateColor(): Int {
        return if (currentArmStateForUi()) statusOfflineColor else statusOnlineColor
    }

    private fun toggleArmState() {
        requestedArmed = !currentArmStateForUi()
        armCommandSender.sendArmState(requestedArmed)
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        canvas.drawRect(0.0f, 0.0f, width.toFloat(), height.toFloat(), backgroundPaint)

        val layout = computeLayout()

        drawMap(canvas, layout.mapRect)

        if (layout.isLandscape) {
            drawLandscapePanel(canvas, layout.panelRect)
        } else {
            drawPortraitPanel(canvas, layout.panelRect)
        }

        postInvalidateDelayed(UI_REFRESH_MS)
    }

    private fun computeLayout(): DashboardLayout {
        val w = width.toFloat()
        val h = height.toFloat()
        val margin = 18.0f
        val isLandscape = w >= h

        return if (isLandscape) {
            val panelWidth = (w * 0.31f).coerceIn(430.0f, 620.0f)

            val mapRect = RectF(
                0.0f,
                0.0f,
                w - panelWidth,
                h
            )

            val panelRect = RectF(
                w - panelWidth + margin,
                margin,
                w - margin,
                h - margin
            )

            DashboardLayout(mapRect, panelRect, true)
        } else {
            val panelHeight = h * 0.50f

            val mapRect = RectF(
                0.0f,
                0.0f,
                w,
                h - panelHeight
            )

            val panelRect = RectF(
                margin,
                h - panelHeight + margin,
                w - margin,
                h - margin
            )

            DashboardLayout(mapRect, panelRect, false)
        }
    }

    private fun drawMap(canvas: Canvas, mapRect: RectF) {
        val gradient = LinearGradient(
            mapRect.left,
            mapRect.top,
            mapRect.right,
            mapRect.bottom,
            Color.rgb(4, 8, 15),
            Color.rgb(1, 3, 7),
            Shader.TileMode.CLAMP
        )

        mapBackgroundPaint.shader = gradient
        canvas.drawRect(mapRect, mapBackgroundPaint)
        mapBackgroundPaint.shader = null

        canvas.save()
        canvas.clipRect(mapRect)

        val originX = mapRect.left + mapRect.width() * 0.18f + panX
        val originY = mapRect.top + mapRect.height() * 0.72f + panY

        drawGrid(canvas, mapRect, originX, originY)
        drawTakeoffMarker(canvas, originX, originY)
        drawTrajectory(canvas, originX, originY)
        drawPointCloud(canvas, originX, originY)
        drawDrone(canvas, originX, originY)

        canvas.restore()
    }

    private fun drawGrid(canvas: Canvas, mapRect: RectF, originX: Float, originY: Float) {
        val maxDistanceM = 40.0f
        val xTickCount = ceil(maxDistanceM / GRID_SPACING_M).toInt()

        for (i in -2..xTickCount) {
            val xM = i * GRID_SPACING_M
            val p1 = projectPoint(xM, -15.0f, 0.0f, originX, originY)
            val p2 = projectPoint(xM, 15.0f, 0.0f, originX, originY)

            val paint = if (i % 2 == 0) gridStrongPaint else gridPaint

            canvas.drawLine(p1.first, p1.second, p2.first, p2.second, paint)

            if (i >= 0 && i % 2 == 0) {
                canvas.drawText(
                    "${xM.toInt()} m",
                    p1.first,
                    (p1.second + 30.0f).coerceAtMost(mapRect.bottom - 12.0f),
                    gridTextPaint
                )
            }
        }

        for (yM in -15..15 step 5) {
            val p1 = projectPoint(-10.0f, yM.toFloat(), 0.0f, originX, originY)
            val p2 = projectPoint(maxDistanceM, yM.toFloat(), 0.0f, originX, originY)
            canvas.drawLine(p1.first, p1.second, p2.first, p2.second, gridPaint)
        }
    }

    private fun drawTakeoffMarker(canvas: Canvas, originX: Float, originY: Float) {
        val p = projectPoint(0.0f, 0.0f, 0.0f, originX, originY)

        canvas.drawCircle(p.first, p.second, 18.0f, takeoffFillPaint)
        canvas.drawCircle(p.first, p.second, 18.0f, takeoffPaint)

        canvas.drawLine(p.first - 26.0f, p.second, p.first - 10.0f, p.second, takeoffPaint)
        canvas.drawLine(p.first + 10.0f, p.second, p.first + 26.0f, p.second, takeoffPaint)
        canvas.drawLine(p.first, p.second - 26.0f, p.first, p.second - 10.0f, takeoffPaint)
        canvas.drawLine(p.first, p.second + 10.0f, p.first, p.second + 26.0f, takeoffPaint)

        gridTextPaint.color = Color.rgb(255, 220, 130)
        canvas.drawText("TAKEOFF", p.first + 24.0f, p.second - 18.0f, gridTextPaint)
        gridTextPaint.color = Color.rgb(105, 132, 162)
    }

    private fun drawTrajectory(canvas: Canvas, originX: Float, originY: Float) {
        val snapshot: List<FloatArray> = synchronized(lock) {
            trajectoryPoints.toList()
        }

        if (snapshot.size < 2) return

        var previous = snapshot.first()

        for (i in 1 until snapshot.size) {
            val current = snapshot[i]

            val p1 = projectPoint(previous[0], previous[1], previous[2], originX, originY)
            val p2 = projectPoint(current[0], current[1], current[2], originX, originY)

            canvas.drawLine(p1.first, p1.second, p2.first, p2.second, trajectoryPaint)

            previous = current
        }
    }

    private fun drawPointCloud(canvas: Canvas, originX: Float, originY: Float) {
        val chunksSnapshot: List<FloatArray>
        val totalPointsSnapshot: Int

        synchronized(lock) {
            chunksSnapshot = pointChunks.toList()
            totalPointsSnapshot = totalPointCount
        }

        if (chunksSnapshot.isEmpty() || totalPointsSnapshot <= 0) return

        val step = max(1, totalPointsSnapshot / MAX_POINTS_DISPLAYED)
        var globalPointIndex = 0

        for (chunk in chunksSnapshot) {
            var base = 0

            while (base + 2 < chunk.size) {
                if (globalPointIndex % step == 0) {
                    val x = chunk[base]
                    val y = chunk[base + 1]
                    val z = chunk[base + 2]

                    val projected = projectPoint(x, y, z, originX, originY)

                    pointHaloPaint.alpha = 150
                    pointCorePaint.alpha = 255

                    canvas.drawCircle(projected.first, projected.second, 7.2f, pointHaloPaint)
                    canvas.drawCircle(projected.first, projected.second, 4.3f, pointCorePaint)
                }

                globalPointIndex += 1
                base += 3
            }
        }
    }

    private fun drawDrone(canvas: Canvas, originX: Float, originY: Float) {
        val pose = synchronized(lock) {
            computeCurrentMapPoseLocked()
        }

        val projected = projectPoint(pose.x, pose.y, pose.z, originX, originY)

        val glow = RadialGradient(
            projected.first,
            projected.second,
            44.0f,
            Color.argb(190, 255, 48, 70),
            Color.argb(0, 255, 48, 70),
            Shader.TileMode.CLAMP
        )

        val glowPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            shader = glow
            style = Paint.Style.FILL
        }

        canvas.drawCircle(projected.first, projected.second, 44.0f, glowPaint)
        canvas.drawCircle(projected.first, projected.second, 13.0f, dronePaint)
    }

    private fun drawLandscapePanel(canvas: Canvas, panel: RectF) {
        drawPanelBase(canvas, panel)

        val headerBottom = drawLandscapeFixedHeader(canvas, panel)

        landscapeScrollableRect = RectF(
            panel.left,
            headerBottom,
            panel.right,
            panel.bottom
        )

        canvas.save()
        canvas.clipRect(landscapeScrollableRect)

        val contentTop = headerBottom + 24.0f
        var y = contentTop - landscapePanelScrollY
        val x = panel.left + 28.0f

        drawSection(canvas, x, y, "NETWORK")
        y += 36.0f

        drawStat(canvas, x, y, "IP", localIpAddress)
        y += 54.0f
        drawStat(canvas, x, y, "LIDAR PORT", UDP_LIDAR_PORT.toString())
        y += 54.0f
        drawStat(canvas, x, y, "TELEMETRY PORT", UDP_TELEMETRY_PORT.toString())
        y += 54.0f
        drawStat(canvas, x, y, "ARM PORT", UDP_ARM_PORT.toString())

        y += 66.0f
        drawSection(canvas, x, y, "INFO")
        y += 36.0f

        drawStat(canvas, x, y, "TIME", formatFloat(latestStats.elapsedTimeS, "s", 1))
        y += 54.0f
        drawStat(canvas, x, y, "POINTS", totalPointCount.toString())
        y += 54.0f
        drawStatColored(canvas, x, y, "ARM STATE", armStateLabel(), armStateColor())
        y += 54.0f
        drawStat(canvas, x, y, "MODE", latestTelemetry.flightMode ?: "—")
        y += 54.0f
        drawStat(canvas, x, y, "HEALTH", latestTelemetry.health ?: "—")

        y += 66.0f
        drawSection(canvas, x, y, "DRONE TELEMETRY")
        y += 38.0f

        drawStat(canvas, x, y, "BATTERY", formatNullable(latestTelemetry.batteryPercent, "%", 0))
        y += 50.0f
        drawStat(canvas, x, y, "VOLTAGE", formatNullable(latestTelemetry.voltageV, "V", 2))
        y += 50.0f
        drawStat(canvas, x, y, "CURRENT", formatNullable(latestTelemetry.currentA, "A", 3))
        y += 50.0f
        drawStat(canvas, x, y, "SPEED", formatNullable(latestTelemetry.velocityMps, "m/s", 3))
        y += 50.0f
        drawStat(canvas, x, y, "ALTITUDE", formatNullable(latestTelemetry.altitudeM, "m", 2))

        y += 66.0f
        drawSection(canvas, x, y, "IMU")
        y += 38.0f

        drawStat(canvas, x, y, "ROLL", formatNullable(latestTelemetry.rollDeg, "°", 1))
        y += 50.0f
        drawStat(canvas, x, y, "PITCH", formatNullable(latestTelemetry.pitchDeg, "°", 1))
        y += 50.0f
        drawStat(canvas, x, y, "YAW", formatNullable(latestTelemetry.mapYawRelativeDeg, "°", 1))
        y += 50.0f
        drawStat(canvas, x, y, "YAW SOURCE", latestTelemetry.mapYawSource ?: "—")

        val contentHeight = y - contentTop + landscapePanelScrollY + 30.0f
        val viewportHeight = landscapeScrollableRect.height() - 24.0f

        landscapePanelMaxScrollY = max(0.0f, contentHeight - viewportHeight)
        landscapePanelScrollY = landscapePanelScrollY.coerceIn(0.0f, landscapePanelMaxScrollY)

        canvas.restore()

        drawLandscapeScrollIndicator(canvas, panel)
    }

    private fun drawLandscapeFixedHeader(canvas: Canvas, panel: RectF): Float {
        val now = System.currentTimeMillis()
        val lidarOnline = latestScanWallMs > 0L && now - latestScanWallMs < CONNECTION_TIMEOUT_MS
        val telemetryOnline = latestTelemetry.isOnline(now)

        val x = panel.left + 28.0f
        var y = panel.top + 58.0f

        titlePaint.textSize = (panel.width() * 0.115f).coerceIn(30.0f, 42.0f)
        canvas.drawText("COVENANT", x, y, titlePaint)

        y += 30.0f
        labelPaint.textSize = 19.0f
        canvas.drawText("ANDROID GROUND STATION", x, y, labelPaint)

        y += 54.0f
        drawConnectionRow(canvas, x, y, "LIDAR", lidarOnline)
        y += 38.0f
        drawConnectionRow(canvas, x, y, "DRONE INFO", telemetryOnline)

        y += 30.0f

        armButtonRect = RectF(
            panel.left + 28.0f,
            y,
            panel.right - 28.0f,
            y + 78.0f
        )

        drawArmButton(canvas, armButtonRect)

        return armButtonRect.bottom + 20.0f
    }

    private fun drawLandscapeScrollIndicator(canvas: Canvas, panel: RectF) {
        if (landscapePanelMaxScrollY <= 1.0f) return

        val trackTop = landscapeScrollableRect.top + 16.0f
        val trackBottom = panel.bottom - 16.0f
        val trackHeight = trackBottom - trackTop

        val ratio =
            landscapeScrollableRect.height() /
                    (landscapeScrollableRect.height() + landscapePanelMaxScrollY)

        val thumbHeight = (trackHeight * ratio).coerceIn(42.0f, trackHeight)

        val thumbY =
            trackTop +
                    (trackHeight - thumbHeight) *
                    (landscapePanelScrollY / landscapePanelMaxScrollY)

        val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.argb(120, 120, 180, 220)
            style = Paint.Style.FILL
        }

        canvas.drawRoundRect(
            panel.right - 10.0f,
            thumbY,
            panel.right - 5.0f,
            thumbY + thumbHeight,
            4.0f,
            4.0f,
            paint
        )
    }

    private fun drawPortraitPanel(canvas: Canvas, panel: RectF) {
        drawPanelBase(canvas, panel)

        val now = System.currentTimeMillis()
        val lidarOnline = latestScanWallMs > 0L && now - latestScanWallMs < CONNECTION_TIMEOUT_MS
        val telemetryOnline = latestTelemetry.isOnline(now)

        val left = panel.left + 28.0f

        val col1 = panel.left + 28.0f
        val col2 = panel.left + panel.width() * 0.28f
        val col3 = panel.left + panel.width() * 0.52f
        val col4 = panel.left + panel.width() * 0.76f

        var y = panel.top + 50.0f

        titlePaint.textSize = (panel.width() * 0.055f).coerceIn(32.0f, 46.0f)
        canvas.drawText("COVENANT", left, y, titlePaint)

        y += 34.0f
        labelPaint.textSize = 20.0f
        canvas.drawText("ANDROID GROUND STATION", left, y, labelPaint)

        val statusX = panel.centerX() + 20.0f
        var statusY = panel.top + 46.0f

        drawConnectionRow(canvas, statusX, statusY, "LIDAR", lidarOnline)
        statusY += 40.0f
        drawConnectionRow(canvas, statusX, statusY, "DRONE INFO", telemetryOnline)

        y = panel.top + 130.0f

        drawSection(canvas, left, y, "NETWORK")
        y += 34.0f

        drawCompactStat(canvas, col1, y, "IP", localIpAddress)
        drawCompactStat(canvas, col2, y, "LIDAR", UDP_LIDAR_PORT.toString())
        drawCompactStat(canvas, col3, y, "TELEM", UDP_TELEMETRY_PORT.toString())
        drawCompactStat(canvas, col4, y, "ARM", UDP_ARM_PORT.toString())

        y += 74.0f

        drawSection(canvas, left, y, "INFO")
        y += 34.0f

        drawCompactStat(canvas, col1, y, "TIME", formatFloat(latestStats.elapsedTimeS, "s", 1))
        drawCompactStat(canvas, col2, y, "POINTS", totalPointCount.toString())
        drawCompactStatColored(canvas, col3, y, "ARM", armStateLabel(), armStateColor())
        drawCompactStat(canvas, col4, y, "MODE", latestTelemetry.flightMode ?: "—")

        y += 74.0f

        drawCompactStat(canvas, col1, y, "HEALTH", latestTelemetry.health ?: "—")

        y += 70.0f

        drawSection(canvas, left, y, "DRONE TELEMETRY")
        y += 34.0f

        drawCompactStat(canvas, col1, y, "BAT", formatNullable(latestTelemetry.batteryPercent, "%", 0))
        drawCompactStat(canvas, col2, y, "VOLT", formatNullable(latestTelemetry.voltageV, "V", 2))
        drawCompactStat(canvas, col3, y, "CURR", formatNullable(latestTelemetry.currentA, "A", 3))
        drawCompactStat(canvas, col4, y, "SPEED", formatNullable(latestTelemetry.velocityMps, "m/s", 3))

        y += 74.0f

        drawCompactStat(canvas, col1, y, "ALT", formatNullable(latestTelemetry.altitudeM, "m", 2))

        y += 70.0f

        drawSection(canvas, left, y, "IMU")
        y += 34.0f

        drawCompactStat(canvas, col1, y, "ROLL", formatNullable(latestTelemetry.rollDeg, "°", 1))
        drawCompactStat(canvas, col2, y, "PITCH", formatNullable(latestTelemetry.pitchDeg, "°", 1))
        drawCompactStat(canvas, col3, y, "YAW", formatNullable(latestTelemetry.mapYawRelativeDeg, "°", 1))
        drawCompactStat(canvas, col4, y, "SRC", latestTelemetry.mapYawSource ?: "—")

        armButtonRect = RectF(
            panel.left + 28.0f,
            panel.bottom - 98.0f,
            panel.right - 28.0f,
            panel.bottom - 18.0f
        )

        drawArmButton(canvas, armButtonRect)
    }

    private fun drawArmButton(canvas: Canvas, rect: RectF) {
        val armed = currentArmStateForUi()

        val fillColor = if (armed) {
            Color.rgb(125, 24, 36)
        } else {
            Color.rgb(20, 120, 72)
        }

        val strokeColor = if (armed) {
            Color.rgb(255, 78, 96)
        } else {
            Color.rgb(90, 255, 170)
        }

        val text = if (armed) "DISARM MOTORS" else "ARM MOTORS"

        val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = fillColor
            style = Paint.Style.FILL
        }

        val strokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = strokeColor
            style = Paint.Style.STROKE
            strokeWidth = 3.5f
        }

        val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE
            textSize = 27.0f
            isFakeBoldText = true
            textAlign = Paint.Align.CENTER
        }

        canvas.drawRoundRect(rect, 24.0f, 24.0f, fillPaint)
        canvas.drawRoundRect(rect, 24.0f, 24.0f, strokePaint)

        val centerY = rect.centerY() - (textPaint.descent() + textPaint.ascent()) / 2.0f
        canvas.drawText(text, rect.centerX(), centerY, textPaint)
    }

    private fun drawPanelBase(canvas: Canvas, panel: RectF) {
        canvas.drawRoundRect(panel, 28.0f, 28.0f, panelPaint)
        canvas.drawRoundRect(panel, 28.0f, 28.0f, panelStrokePaint)
    }

    private fun drawSection(canvas: Canvas, x: Float, y: Float, text: String) {
        sectionPaint.textSize = 22.0f
        canvas.drawText(text, x, y, sectionPaint)
    }

    private fun drawConnectionRow(canvas: Canvas, x: Float, y: Float, label: String, online: Boolean) {
        val color = if (online) statusOnlineColor else statusOfflineColor
        val text = if (online) "ONLINE" else "OFFLINE"

        val dotPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            this.color = color
            style = Paint.Style.FILL
        }

        canvas.drawCircle(x + 9.0f, y - 8.0f, 8.0f, dotPaint)

        labelPaint.textSize = 20.0f
        valuePaint.textSize = 20.0f

        canvas.drawText(label, x + 26.0f, y, labelPaint)

        valuePaint.color = color
        canvas.drawText(text, x + 160.0f, y, valuePaint)
        valuePaint.color = Color.rgb(232, 248, 255)
    }

    private fun drawStat(canvas: Canvas, x: Float, y: Float, label: String, value: String) {
        labelPaint.textSize = 20.0f
        smallValuePaint.textSize = 22.0f

        canvas.drawText(label, x, y, labelPaint)
        canvas.drawText(value, x, y + 28.0f, smallValuePaint)
    }

    private fun drawCompactStat(canvas: Canvas, x: Float, y: Float, label: String, value: String) {
        labelPaint.textSize = 19.0f
        smallValuePaint.textSize = 22.0f

        canvas.drawText(label, x, y, labelPaint)
        canvas.drawText(value, x, y + 31.0f, smallValuePaint)
    }

    private fun drawStatColored(
        canvas: Canvas,
        x: Float,
        y: Float,
        label: String,
        value: String,
        color: Int
    ) {
        labelPaint.textSize = 20.0f
        smallValuePaint.textSize = 22.0f

        canvas.drawText(label, x, y, labelPaint)

        val previousColor = smallValuePaint.color
        smallValuePaint.color = color
        canvas.drawText(value, x, y + 28.0f, smallValuePaint)
        smallValuePaint.color = previousColor
    }

    private fun drawCompactStatColored(
        canvas: Canvas,
        x: Float,
        y: Float,
        label: String,
        value: String,
        color: Int
    ) {
        labelPaint.textSize = 19.0f
        smallValuePaint.textSize = 22.0f

        canvas.drawText(label, x, y, labelPaint)

        val previousColor = smallValuePaint.color
        smallValuePaint.color = color
        canvas.drawText(value, x, y + 31.0f, smallValuePaint)
        smallValuePaint.color = previousColor
    }

    private fun projectPoint(
        x: Float,
        y: Float,
        z: Float,
        originX: Float,
        originY: Float
    ): Pair<Float, Float> {
        val screenX =
            originX +
                    x * scalePxPerMeter +
                    y * scalePxPerMeter * 0.38f

        val screenY =
            originY -
                    z * scalePxPerMeter -
                    y * scalePxPerMeter * 0.20f

        return Pair(screenX, screenY)
    }

    private fun formatFloat(value: Float, unit: String, decimals: Int): String {
        return String.format(Locale.US, "%.${decimals}f %s", value, unit)
    }

    private fun formatNullable(value: Float?, unit: String, decimals: Int): String {
        if (value == null) return "—"
        return String.format(Locale.US, "%.${decimals}f %s", value, unit)
    }

    companion object {
        fun findLocalIpv4Address(): String? {
            return try {
                val interfaces = mutableListOf<NetworkInterface>()
                val enumeration = NetworkInterface.getNetworkInterfaces()

                while (enumeration.hasMoreElements()) {
                    interfaces.add(enumeration.nextElement())
                }

                val preferredInterfaces = interfaces.sortedWith(
                    compareBy<NetworkInterface> {
                        when {
                            it.name.startsWith("wlan", ignoreCase = true) -> 0
                            it.name.startsWith("eth", ignoreCase = true) -> 1
                            else -> 2
                        }
                    }
                )

                for (networkInterface in preferredInterfaces) {
                    if (!networkInterface.isUp || networkInterface.isLoopback) continue

                    val addresses = networkInterface.inetAddresses

                    while (addresses.hasMoreElements()) {
                        val address = addresses.nextElement()

                        if (!address.isLoopbackAddress && address is Inet4Address) {
                            return address.hostAddress
                        }
                    }
                }

                null
            } catch (_: Exception) {
                null
            }
        }
    }
}


private fun JSONObject.optFloatOrNull(vararg keys: String): Float? {
    for (key in keys) {
        if (!has(key) || isNull(key)) continue

        val value = optDouble(key, Double.NaN)

        if (!value.isNaN()) {
            return value.toFloat()
        }
    }

    return null
}


private fun JSONObject.optStringOrNull(vararg keys: String): String? {
    for (key in keys) {
        if (!has(key) || isNull(key)) continue

        val value = optString(key, "").trim()

        if (value.isNotEmpty()) {
            return value
        }
    }

    return null
}


private fun JSONObject.optBooleanOrNull(vararg keys: String): Boolean? {
    for (key in keys) {
        if (!has(key) || isNull(key)) continue

        return optBoolean(key)
    }

    return null
}