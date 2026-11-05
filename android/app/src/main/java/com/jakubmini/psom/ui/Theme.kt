package com.jakubmini.psom.ui

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.ExperimentalTextApi
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.Font
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontVariation
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp
import com.jakubmini.psom.R

/** The five colours, and what each one is for. */
object Palette {
    val Cream = Color(0xFFF7EEDD)       // ground
    val Ink = Color(0xFF2F4A4D)         // text, primary buttons
    val Ink2 = Color(0xFF5B6E6D)        // secondary text (4.6:1 on cream)
    val Line = Color(0x1F2F4A4D)        // hairlines
    val Wash = Color(0x0F2F4A4D)        // pressed / secondary fills
    val Sea = Color(0xFF86C5B4)         // good: tracking, solid
    val Terracotta = Color(0xFFD97B4A)
    val Mustard = Color(0xFFD6B23E)     // warnings
    val Night = Color(0xFF0E1718)       // the camera's surround
    val Glow = Brush.linearGradient(listOf(Mustard, Terracotta))
}

@OptIn(ExperimentalTextApi::class)
private fun geist(weight: Int) =
    Font(R.font.geist, FontWeight(weight), variationSettings = FontVariation.Settings(FontVariation.weight(weight)))

@OptIn(ExperimentalTextApi::class)
private fun mono(weight: Int) =
    Font(R.font.geist_mono, FontWeight(weight), variationSettings = FontVariation.Settings(FontVariation.weight(weight)))

val Geist = FontFamily(geist(400), geist(600))
val GeistMono = FontFamily(mono(400))

/** Four sizes, two weights. */
object Type {
    val Title = TextStyle(fontFamily = Geist, fontWeight = FontWeight(600), fontSize = 34.sp, lineHeight = 40.sp, letterSpacing = (-0.6).sp)
    val Body = TextStyle(fontFamily = Geist, fontWeight = FontWeight(400), fontSize = 17.sp, lineHeight = 24.sp)
    val BodyStrong = Body.copy(fontWeight = FontWeight(600))
    val Small = TextStyle(fontFamily = Geist, fontWeight = FontWeight(400), fontSize = 13.sp, lineHeight = 18.sp)
    val SmallStrong = Small.copy(fontWeight = FontWeight(600))
    val Number = TextStyle(fontFamily = GeistMono, fontSize = 34.sp, lineHeight = 40.sp, letterSpacing = (-1).sp)
    val SmallMono = TextStyle(fontFamily = GeistMono, fontSize = 13.sp, lineHeight = 18.sp)
}

@Composable
fun PsomTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = lightColorScheme(
            primary = Palette.Ink, onPrimary = Palette.Cream,
            background = Palette.Cream, onBackground = Palette.Ink,
            surface = Palette.Cream, onSurface = Palette.Ink,
            surfaceContainerHigh = Palette.Cream, onSurfaceVariant = Palette.Ink2,
            outline = Palette.Ink2, error = Color(0xFF9A4A00),
        ),
        typography = MaterialTheme.typography.copy(
            bodyLarge = Type.Body, bodyMedium = Type.Body, bodySmall = Type.Small,
            titleLarge = Type.Title, labelLarge = Type.BodyStrong,
        ),
        content = content,
    )
}
