package com.example.covenantgroundstation

import android.app.Activity
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RadialGradient
import android.graphics.Shader
import android.os.Bundle
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.Window
import android.view.WindowManager
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.Inet4Address
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.concurrent.thread
import kotlin.math.PI
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin


private const val UDP_LISTEN_PORT = 56000
private const val UDP_RECV_SIZE_BYTES = 2048
private const val UDP_PROTOCOL_DISPLAY = "UDP"

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

private const val SIMULATED_SPEED_X_M_S = 0.25f

private const val GRID_SPACING_M = 5.0f
private const val MAX_POINTS_IN_MEMORY = 120_000
private const val MAX_POINTS_DISPLAYED = 45_000


data class ScanStats(
    val distanceM: Float = 0.0f,
    val elapsedTimeS: Float = 0.0f,
    val scans: Int = 0,
    val frames: Int = 0,
    val pointsReceived: Int = 0,
)


class MainActivity : Activity() {

    private lateinit var groundStationView: GroundStationView
    private var receiver: UdpLidarReceiver? = null

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

        receiver = UdpLidarReceiver(
            onScanReady = { xyzPoints, stats ->
                runOnUiThread {
                    groundStationView.addScan(xyzPoints, stats)
                }
            }
        )

        receiver?.start()
    }

    override fun onStop() {
        receiver?.stop()
        receiver = null

        super.onStop()
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

        receiverThread = thread(
            start = true,
            name = "CovenantUdpLidarReceiver"
        ) {
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

        val currentScanAngles = ArrayList<Float>(600)
        val currentScanDistances = ArrayList<Float>(600)

        var previousStartAngle: Float? = null

        var scanCount = 0
        var frameCount = 0
        var pointsReceived = 0

        val startTimeNs = System.nanoTime()

        try {
            val udpSocket = DatagramSocket(null)
            udpSocket.reuseAddress = true
            udpSocket.bind(InetSocketAddress("0.0.0.0", UDP_LISTEN_PORT))

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

                        val xPositionM = elapsedTimeS * SIMULATED_SPEED_X_M_S

                        val xyzPoints = buildXyzScan(
                            currentScanAngles,
                            currentScanDistances,
                            xPositionM
                        )

                        pointsReceived += xyzPoints.size / 3

                        val stats = ScanStats(
                            distanceM = xPositionM,
                            elapsedTimeS = elapsedTimeS,
                            scans = scanCount,
                            frames = frameCount,
                            pointsReceived = pointsReceived,
                        )

                        onScanReady(xyzPoints, stats)

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
            // Normal when socket is closed during stop().
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

        val magic = String(magicBytes)
        if (magic != PACKET_MAGIC) return null

        val version = buffer.get().toInt() and 0xFF
        val packetType = buffer.get().toInt() and 0xFF

        if (version != PROTOCOL_VERSION) return null
        if (packetType != PACKET_TYPE_RAW_LIDAR) return null

        buffer.int // sequence
        buffer.long // timestamp ns

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

        if (header != FRAME_HEADER || versionLength != FRAME_VER_LEN) {
            return null
        }

        val startAngleDeg = readUInt16LE(frame, 4) / 100.0f

        val rawPoints = ArrayList<LidarPoint>(POINTS_PER_FRAME)

        var offset = 6

        repeat(POINTS_PER_FRAME) {
            val distanceMm = readUInt16LE(frame, offset)
            val confidence = frame[offset + 2].toInt() and 0xFF
            offset += 3

            val distanceM = distanceMm / 1000.0f

            rawPoints.add(
                LidarPoint(
                    angleDeg = 0.0f,
                    distanceM = distanceM,
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

    private fun buildXyzScan(
        angles: List<Float>,
        distances: List<Float>,
        xPositionM: Float
    ): FloatArray {
        val pointCount = min(angles.size, distances.size)
        val xyz = FloatArray(pointCount * 3)

        var out = 0

        for (i in 0 until pointCount) {
            val angleRad = angles[i] * PI.toFloat() / 180.0f
            val distance = distances[i]

            val x = xPositionM
            val y = distance * cos(angleRad)
            val z = distance * sin(angleRad)

            xyz[out++] = x
            xyz[out++] = y
            xyz[out++] = z
        }

        return xyz
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

        if (buffer.size > 8192) {
            buffer = buffer.copyOfRange(buffer.size - 8192, buffer.size)
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

            if (buffer.size < FRAME_LENGTH) {
                return null
            }

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


class GroundStationView(
    context: android.content.Context
) : android.view.View(context) {

    private val lock = Any()

    private val pointCloud = ArrayList<Float>(MAX_POINTS_IN_MEMORY * 3)

    private var latestStats = ScanStats()

    private val localIpAddress = findLocalIpv4Address() ?: "0.0.0.0"

    private var scalePxPerMeter = 45.0f
    private var panX = 0.0f
    private var panY = 0.0f

    private var lastTouchX = 0.0f
    private var lastTouchY = 0.0f
    private var isPanning = false

    private val scaleDetector = ScaleGestureDetector(
        context,
        object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
            override fun onScale(detector: ScaleGestureDetector): Boolean {
                scalePxPerMeter *= detector.scaleFactor
                scalePxPerMeter = scalePxPerMeter.coerceIn(8.0f, 220.0f)
                invalidate()
                return true
            }
        }
    )

    private val backgroundPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(3, 5, 8)
        style = Paint.Style.FILL
    }

    private val panelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(7, 12, 18)
        style = Paint.Style.FILL
    }

    private val panelStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(23, 35, 52)
        style = Paint.Style.STROKE
        strokeWidth = 2.0f
    }

    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(23, 35, 51)
        strokeWidth = 1.4f
        alpha = 160
    }

    private val gridTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(96, 120, 145)
        textSize = 22.0f
    }

    private val trajectoryPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(255, 32, 53)
        strokeWidth = 5.0f
        style = Paint.Style.STROKE
    }

    private val dronePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(255, 32, 53)
        style = Paint.Style.FILL
    }

    private val titlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 44.0f
        isFakeBoldText = true
    }

    private val subtitlePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(127, 149, 178)
        textSize = 22.0f
    }

    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.rgb(113, 134, 161)
        textSize = 24.0f
    }

    private val valuePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 32.0f
        isFakeBoldText = true
    }

    private val pointPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        strokeWidth = 2.0f
    }

    fun addScan(xyzPoints: FloatArray, stats: ScanStats) {
        synchronized(lock) {
            latestStats = stats

            for (value in xyzPoints) {
                pointCloud.add(value)
            }

            val maxValues = MAX_POINTS_IN_MEMORY * 3

            if (pointCloud.size > maxValues) {
                val removeCount = pointCloud.size - maxValues
                pointCloud.subList(0, removeCount).clear()
            }
        }

        invalidate()
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        scaleDetector.onTouchEvent(event)

        if (event.pointerCount > 1) {
            isPanning = false
            return true
        }

        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                isPanning = true
                lastTouchX = event.x
                lastTouchY = event.y
            }

            MotionEvent.ACTION_MOVE -> {
                if (isPanning) {
                    val dx = event.x - lastTouchX
                    val dy = event.y - lastTouchY

                    panX += dx
                    panY += dy

                    lastTouchX = event.x
                    lastTouchY = event.y

                    invalidate()
                }
            }

            MotionEvent.ACTION_UP,
            MotionEvent.ACTION_CANCEL -> {
                isPanning = false
            }
        }

        return true
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        canvas.drawRect(0.0f, 0.0f, width.toFloat(), height.toFloat(), backgroundPaint)

        val panelWidth = width * 0.24f
        val mapWidth = width - panelWidth

        drawMap(canvas, mapWidth, height.toFloat())
        drawSidePanel(canvas, mapWidth, panelWidth, height.toFloat())
    }

    private fun drawMap(canvas: Canvas, mapWidth: Float, mapHeight: Float) {
        val originX = mapWidth * 0.12f + panX
        val originY = mapHeight * 0.72f + panY

        drawGrid(canvas, originX, originY, mapWidth, mapHeight)
        drawPointCloud(canvas, originX, originY)
        drawTrajectory(canvas, originX, originY)
        drawDrone(canvas, originX, originY)
    }

    private fun drawGrid(canvas: Canvas, originX: Float, originY: Float, mapWidth: Float, mapHeight: Float) {
        val maxDistanceM = max(20.0f, latestStats.distanceM + 10.0f)
        val xTickCount = ceil(maxDistanceM / GRID_SPACING_M).toInt()

        for (i in 0..xTickCount) {
            val xM = i * GRID_SPACING_M
            val p1 = projectPoint(xM, -8.0f, 0.0f, originX, originY)
            val p2 = projectPoint(xM, 8.0f, 0.0f, originX, originY)

            canvas.drawLine(p1.first, p1.second, p2.first, p2.second, gridPaint)

            canvas.drawText(
                "${xM.toInt()} m",
                p1.first,
                p1.second + 32.0f,
                gridTextPaint
            )
        }

        for (yM in -10..10 step 5) {
            val p1 = projectPoint(0.0f, yM.toFloat(), 0.0f, originX, originY)
            val p2 = projectPoint(maxDistanceM, yM.toFloat(), 0.0f, originX, originY)

            canvas.drawLine(p1.first, p1.second, p2.first, p2.second, gridPaint)
        }
    }

    private fun drawPointCloud(canvas: Canvas, originX: Float, originY: Float) {
        val snapshot: List<Float>
        val distanceSnapshot: Float

        synchronized(lock) {
            snapshot = ArrayList(pointCloud)
            distanceSnapshot = latestStats.distanceM
        }

        if (snapshot.size < 3) return

        val totalPoints = snapshot.size / 3
        val step = max(1, totalPoints / MAX_POINTS_DISPLAYED)

        var pointIndex = 0

        while (pointIndex < totalPoints) {
            val base = pointIndex * 3

            val x = snapshot[base]
            val y = snapshot[base + 1]
            val z = snapshot[base + 2]

            val projected = projectPoint(x, y, z, originX, originY)

            val progress = if (distanceSnapshot > 1.0f) {
                (x / distanceSnapshot).coerceIn(0.0f, 1.0f)
            } else {
                0.0f
            }

            pointPaint.color = scanColor(progress)

            canvas.drawCircle(projected.first, projected.second, 2.1f, pointPaint)

            pointIndex += step
        }
    }

    private fun drawTrajectory(canvas: Canvas, originX: Float, originY: Float) {
        val currentX = latestStats.distanceM

        if (currentX <= 0.0f) return

        val start = projectPoint(0.0f, 0.0f, 0.0f, originX, originY)
        val end = projectPoint(currentX, 0.0f, 0.0f, originX, originY)

        canvas.drawLine(start.first, start.second, end.first, end.second, trajectoryPaint)
    }

    private fun drawDrone(canvas: Canvas, originX: Float, originY: Float) {
        val currentX = latestStats.distanceM
        val projected = projectPoint(currentX, 0.0f, 0.0f, originX, originY)

        val glow = RadialGradient(
            projected.first,
            projected.second,
            38.0f,
            Color.argb(190, 255, 32, 53),
            Color.argb(0, 255, 32, 53),
            Shader.TileMode.CLAMP
        )

        val glowPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            shader = glow
            style = Paint.Style.FILL
        }

        canvas.drawCircle(projected.first, projected.second, 38.0f, glowPaint)
        canvas.drawCircle(projected.first, projected.second, 13.0f, dronePaint)
    }

    private fun drawSidePanel(canvas: Canvas, mapWidth: Float, panelWidth: Float, fullHeight: Float) {
        val left = mapWidth + 18.0f
        val top = 18.0f
        val right = mapWidth + panelWidth - 18.0f
        val bottom = fullHeight - 18.0f

        canvas.drawRoundRect(left, top, right, bottom, 26.0f, 26.0f, panelPaint)
        canvas.drawRoundRect(left, top, right, bottom, 26.0f, 26.0f, panelStrokePaint)

        var y = top + 64.0f
        val x = left + 32.0f

        canvas.drawText("COVENANT", x, y, titlePaint)

        y += 36.0f
        canvas.drawText("GROUND STATION", x, y, subtitlePaint)

        y += 72.0f

        drawStat(canvas, x, y, "IP", localIpAddress)
        y += 78.0f

        drawStat(canvas, x, y, "PORT", UDP_LISTEN_PORT.toString())
        y += 78.0f

        drawStat(canvas, x, y, "PROTOCOL", UDP_PROTOCOL_DISPLAY)
        y += 98.0f

        drawStat(canvas, x, y, "DISTANCE", "%.2f m".format(latestStats.distanceM))
        y += 78.0f

        drawStat(canvas, x, y, "TIME", "%.1f s".format(latestStats.elapsedTimeS))
    }

    private fun drawStat(canvas: Canvas, x: Float, y: Float, label: String, value: String) {
        canvas.drawText(label, x, y, labelPaint)
        canvas.drawText(value, x, y + 36.0f, valuePaint)
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
                    y * scalePxPerMeter * 0.42f

        val screenY =
            originY -
                    z * scalePxPerMeter -
                    y * scalePxPerMeter * 0.22f

        return Pair(screenX, screenY)
    }

    private fun scanColor(progress: Float): Int {
        val p = progress.coerceIn(0.0f, 1.0f)

        val r: Int
        val g: Int
        val b: Int

        if (p < 0.5f) {
            val t = p / 0.5f

            r = lerp(114, 157, t)
            g = lerp(247, 255, t)
            b = lerp(255, 210, t)
        } else {
            val t = (p - 0.5f) / 0.5f

            r = lerp(157, 255, t)
            g = lerp(255, 255, t)
            b = lerp(210, 255, t)
        }

        return Color.rgb(r, g, b)
    }

    private fun lerp(a: Int, b: Int, t: Float): Int {
        return (a + (b - a) * t).toInt().coerceIn(0, 255)
    }

    companion object {
        fun findLocalIpv4Address(): String? {
            return try {
                val interfaces = NetworkInterface.getNetworkInterfaces()

                for (networkInterface in interfaces) {
                    val addresses = networkInterface.inetAddresses

                    for (address in addresses) {
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