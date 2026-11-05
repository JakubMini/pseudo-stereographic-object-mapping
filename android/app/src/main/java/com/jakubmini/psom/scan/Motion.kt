package com.jakubmini.psom.scan

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import kotlin.math.sqrt

/**
 * The phone's motion sensors: which way the camera points (gravity-aligned, no compass) for the
 * coverage ring, how fast it turns (fast turns blur), gravity for each photo (the engine levels the
 * scan with it), and raw gyroscope and accelerometer rows for imu.csv.
 */
class Motion(context: Context) : SensorEventListener {
    private val sm = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val gravitySensor = sm.getDefaultSensor(Sensor.TYPE_GRAVITY)
    /** Receives (timestamp, 'g' or 'a', x, y, z) while set. */
    @Volatile var sink: ((Long, Char, Float, Float, Float) -> Unit)? = null
    @Volatile private var rotation: FloatArray? = null
    /** Up, in device axes (x right, y up, z out of the screen), m/s². */
    @Volatile var gravity: FloatArray? = null
        private set
    /** rad/s */
    @Volatile var turnRate = 0.0
        private set

    fun start() {
        sm.unregisterListener(this)  // safe to call again
        for (type in listOf(Sensor.TYPE_GAME_ROTATION_VECTOR, Sensor.TYPE_GRAVITY, Sensor.TYPE_GYROSCOPE, Sensor.TYPE_ACCELEROMETER))
            sm.getDefaultSensor(type)?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
    }

    fun stop() {
        sm.unregisterListener(this)
        sink = null
    }

    /**
     * Direction from the object to the camera (the reverse of where the back camera looks), as
     * (x, up, z) with an arbitrary compass heading. Null before the first reading.
     */
    fun towardCamera(): DoubleArray? {
        val r = rotation ?: return null
        // world-from-device, world z up; the back camera looks along device -z
        return doubleArrayOf(r[2].toDouble(), r[8].toDouble(), r[5].toDouble())
    }

    override fun onSensorChanged(e: SensorEvent) {
        val v = e.values
        when (e.sensor.type) {
            Sensor.TYPE_GAME_ROTATION_VECTOR -> rotation = FloatArray(9).also { SensorManager.getRotationMatrixFromVector(it, v) }
            Sensor.TYPE_GRAVITY -> gravity = floatArrayOf(v[0], v[1], v[2])
            Sensor.TYPE_GYROSCOPE -> {
                turnRate = sqrt((v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).toDouble())
                sink?.invoke(e.timestamp, 'g', v[0], v[1], v[2])
            }
            Sensor.TYPE_ACCELEROMETER -> {
                if (gravitySensor == null) {  // no fused gravity: a slow average of the accelerometer
                    val g = gravity ?: floatArrayOf(v[0], v[1], v[2])
                    gravity = FloatArray(3) { 0.9f * g[it] + 0.1f * v[it] }
                }
                sink?.invoke(e.timestamp, 'a', v[0], v[1], v[2])
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
}
