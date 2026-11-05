package com.jakubmini.psom.scan

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager

/** Gyroscope and accelerometer, recorded for later (the engine relies on ARCore's fused poses today). */
class ImuRecorder(context: Context) : SensorEventListener {
    private val sm = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private var sink: ((Long, Char, Float, Float, Float) -> Unit)? = null

    fun start(sink: (Long, Char, Float, Float, Float) -> Unit) {
        this.sink = sink
        sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
        sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
    }

    fun stop() {
        sm.unregisterListener(this)
        sink = null
    }

    override fun onSensorChanged(e: SensorEvent) {
        val kind = if (e.sensor.type == Sensor.TYPE_GYROSCOPE) 'g' else 'a'
        sink?.invoke(e.timestamp, kind, e.values[0], e.values[1], e.values[2])
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
}
