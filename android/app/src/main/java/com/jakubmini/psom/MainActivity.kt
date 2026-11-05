package com.jakubmini.psom

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import com.google.ar.core.ArCoreApk
import com.jakubmini.psom.scan.ArCapture
import com.jakubmini.psom.ui.App
import com.jakubmini.psom.ui.AppViewModel
import com.jakubmini.psom.ui.Screen
import kotlinx.coroutines.launch

class MainActivity : ComponentActivity() {
    private val vm: AppViewModel by viewModels()
    private lateinit var capture: ArCapture
    private val blocker = mutableStateOf<String?>(null)
    private var installRequested = false

    private val askCamera = registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
        if (!granted) blocker.value = "The camera permission is needed to scan."
        else check()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        capture = ArCapture(this, vm.root)
        // Saved scans stay usable without a camera: the checks run only when the scanner opens.
        lifecycleScope.launch { vm.screen.collect { if (it == Screen.Scan) check() } }
        setContent {
            val b = blocker.value
            if (b != null) MaterialTheme {
                Surface(Modifier.fillMaxSize()) {
                    Column(Modifier.padding(24.dp)) {
                        Text(b)
                        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                            Button(onClick = { blocker.value = null; check() }) { Text("Try again") }
                            OutlinedButton(onClick = { blocker.value = null; vm.openProjects() }) { Text("Back to scans") }
                        }
                    }
                }
            } else App(vm, capture)
        }
    }

    override fun onResume() {
        super.onResume()
        if (vm.screen.value == Screen.Scan) check()
    }

    override fun onDestroy() {
        super.onDestroy()
        if (isFinishing) capture.close()
    }

    /** Camera permission, then ARCore installed and up to date. */
    private fun check() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            askCamera.launch(Manifest.permission.CAMERA)
            return
        }
        try {
            when (ArCoreApk.getInstance().requestInstall(this, !installRequested)) {
                ArCoreApk.InstallStatus.INSTALL_REQUESTED -> installRequested = true
                ArCoreApk.InstallStatus.INSTALLED -> {}
                else -> {}
            }
        } catch (e: Exception) {
            blocker.value = "Scanning needs Google Play Services for AR, which isn't available: " +
                (e.message ?: e.javaClass.simpleName) + ". Saved scans can still be reviewed and exported."
        }
    }
}
