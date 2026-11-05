package com.jakubmini.psom

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.SystemBarStyle
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.statusBarsPadding
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import com.jakubmini.psom.scan.CameraCapture
import com.jakubmini.psom.ui.App
import com.jakubmini.psom.ui.AppViewModel
import com.jakubmini.psom.ui.Palette
import com.jakubmini.psom.ui.PrimaryButton
import com.jakubmini.psom.ui.PsomTheme
import com.jakubmini.psom.ui.QuietButton
import com.jakubmini.psom.ui.Type
import com.jakubmini.psom.ui.Screen
import kotlinx.coroutines.launch

class MainActivity : ComponentActivity() {
    private val vm: AppViewModel by viewModels()
    private lateinit var capture: CameraCapture
    private val blocker = mutableStateOf<String?>(null)

    private val askCamera = registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
        if (!granted) blocker.value = "Scanning needs the camera."
        else check()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge(statusBarStyle = SystemBarStyle.light(android.graphics.Color.TRANSPARENT, android.graphics.Color.TRANSPARENT))
        capture = CameraCapture(this, vm.root)
        // Saved scans stay usable without a camera: the checks run only when the scanner opens.
        lifecycleScope.launch { vm.screen.collect { if (it == Screen.Scan) check() } }
        setContent {
            val b = blocker.value
            if (b != null) PsomTheme {
                Surface(Modifier.fillMaxSize(), color = Palette.Cream) {
                    Column(Modifier.statusBarsPadding().padding(24.dp), verticalArrangement = Arrangement.spacedBy(24.dp)) {
                        Spacer(Modifier.height(48.dp))
                        Text(b, style = Type.Title, color = Palette.Ink)
                        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                            PrimaryButton("Try again") { blocker.value = null; check() }
                            QuietButton("Back to scans") { blocker.value = null; vm.openProjects() }
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

    /** The camera permission. */
    private fun check() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED)
            askCamera.launch(Manifest.permission.CAMERA)
    }
}
